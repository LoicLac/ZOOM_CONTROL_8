/*
  CAPTURE_ANY_NOTIF — diagnostic: log all TX+FLOW notifications after READY,
  count prefix bytes, dump A1 if any. No assumption that init is A1.
*/
#include <Arduino.h>
#include <ArduinoBLE.h>

// ===== Zoom UUIDs =====
static const char* UUID_SERVICE_CUSTOM = "70bcfdf1-3600-185b-d562-b64c851dc87d";
static const char* UUID_TX_NOTIFY      = "69abc935-4eb8-982e-6e55-b812ef754bbf";
static const char* UUID_RX_WWR         = "22eb6fbe-75e4-c334-f1ce-73b59c6847e0";
static const char* UUID_FLOW_CTRL      = "064131ea-592c-3ad9-4f8f-71db1b192828";

// ===== Timing =====
static const uint32_t SCAN_TIMEOUT_MS   = 15000;
static const uint32_t READY_TIMEOUT_MS  = 15000;
static const uint32_t WRITE_GAP_MS      = 30;
// After READY: capture everything for this long
static const uint32_t POST_READY_CAPTURE_MS = 3000;

// ===== Family A frames (same as your working sketch) =====
struct Frame { const uint8_t* p; uint8_t n; };
static const uint8_t F01[]  = {0xB6,0x00};
static const uint8_t F02[]  = {0xBA,0x00};
static const uint8_t F03[]  = {0xD1,0x12,0x32,0x2E,0x33,0x2E,0x31,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
static const uint8_t F04[]  = {0xD2,0x00};
static const uint8_t F05[]  = {0xA7,0x00};
static const uint8_t F06[]  = {0x91,0x00};
static const uint8_t F07[]  = {0x93,0x00};
static const uint8_t F08[]  = {0x95,0x01,0x00};
static const uint8_t F09[]  = {0x95,0x01,0x01};
static const uint8_t F10[]  = {0x97,0x01,0x00};
static const uint8_t F11[]  = {0x97,0x01,0x01};
static const uint8_t F12[]  = {0x99,0x01,0x00};
static const uint8_t F13[]  = {0x99,0x01,0x01};
static const uint8_t F14[]  = {0x9B,0x00};
static const uint8_t F15[]  = {0x9D,0x00};
static const uint8_t F16[]  = {0xC5,0x00};
static const uint8_t F17[]  = {0xCE,0x00};
static const uint8_t F18[]  = {0xC8,0x00};
static const uint8_t F19[]  = {0xAB,0x00};
static const uint8_t F20[]  = {0xAE,0x00};
static const uint8_t F21[]  = {0x8A,0x01,0x01};
static const uint8_t F22[]  = {0x8B,0x01,0x01};
static const uint8_t F23[]  = {0x8C,0x01,0x01};
static const uint8_t F24[]  = {0x8D,0x01,0x01};
static const uint8_t F25[]  = {0x8E,0x01,0x01};
static const uint8_t F26[]  = {0x8F,0x01,0x01};
static const uint8_t F27[]  = {0x90,0x01,0x01};
static const uint8_t F28[]  = {0xE0,0x02,0x00,0x00};
static const uint8_t F29[]  = {0xE0,0x02,0x01,0x00};
static const uint8_t F30[]  = {0xE0,0x02,0x02,0x00};
static const uint8_t F31[]  = {0xE0,0x02,0x41,0x00};
static const uint8_t F32[]  = {0xE0,0x02,0x69,0x00};
static const uint8_t F33[]  = {0xE0,0x02,0x78,0x00};
static const uint8_t F34[]  = {0xE0,0x02,0x14,0x01};
static const uint8_t F35[]  = {0xE8,0x02,0x15,0x01};
static const uint8_t F36[]  = {0xE0,0x02,0x17,0x01};
static const uint8_t F37[]  = {0xE0,0x02,0x18,0x01};
static const uint8_t F38[]  = {0xCA,0x00};

static const Frame FAMILY_A[] = {
  {F01,2},{F02,2},{F03,20},{F04,2},{F05,2},{F06,2},{F07,2},
  {F08,3},{F09,3},{F10,3},{F11,3},{F12,3},{F13,3},
  {F14,2},{F15,2},{F16,2},{F17,2},{F18,2},{F19,2},{F20,2},
  {F21,3},{F22,3},{F23,3},{F24,3},{F25,3},{F26,3},{F27,3},
  {F28,4},{F29,4},{F30,4},{F31,4},{F32,4},{F33,4},{F34,4},
  {F35,4},{F36,4},{F37,4},{F38,2}
};

// ===== BLE handles =====
BLEDevice zoomDev;
BLECharacteristic chTx, chRx, chFlow;

enum State { ST_SCAN=0, ST_CONNECT, ST_DISCOVER, ST_SUBSCRIBE, ST_SEND_HELLO, ST_SEND_FAMILY_A, ST_WAIT_READY, ST_POST_READY, ST_DONE };
State st = ST_SCAN;

uint32_t tEnter = 0;
uint32_t lastWriteMs = 0;

// ===== Stats =====
static uint32_t prefixCount[256] = {0};
static uint32_t totalNotifs = 0;
static uint32_t totalA1 = 0;
static uint32_t total83 = 0;
static uint32_t postReadyStartMs = 0;

// ===== 87/83 capture for static state analysis (order preserved) =====
#define MAX_87   64
#define MAX_83   64
struct F87 { uint8_t b1, b2, b3; };
struct F83 { uint8_t payload[14]; };
static F87 buf87[MAX_87];
static F83 buf83[MAX_83];
static uint16_t n87 = 0, n83 = 0;

static void printHex(const uint8_t* p, int n) {
  for (int i=0;i<n;i++){
    if (p[i] < 16) Serial.print('0');
    Serial.print(p[i], HEX);
    if (i != n-1) Serial.print(' ');
  }
}

static void dumpSummary() {
  Serial.println("\n=== NOTIF SUMMARY (prefix byte 0) ===");
  Serial.print("totalNotifs="); Serial.print(totalNotifs);
  Serial.print(" totalA1="); Serial.print(totalA1);
  Serial.print(" total83="); Serial.println(total83);
  for (int b=0;b<256;b++){
    if (!prefixCount[b]) continue;
    Serial.print("0x");
    if (b<16) Serial.print('0');
    Serial.print(b, HEX);
    Serial.print(" : ");
    Serial.println(prefixCount[b]);
  }
  Serial.println("=== END SUMMARY ===");

  // Parseable 87/83 blocks for analyze_87_83.py (copy from ANALYZE 87 to END ANALYZE 83)
  Serial.println("\n=== ANALYZE 87 ===");
  for (uint16_t i=0;i<n87;i++) {
    Serial.print("87,");
    Serial.print(i);
    Serial.print(",");
    Serial.print(buf87[i].b1, HEX);
    Serial.print(",");
    Serial.print(buf87[i].b2, HEX);
    Serial.print(",");
    Serial.println(buf87[i].b3, HEX);
  }
  Serial.println("=== END ANALYZE 87 ===");
  Serial.println("=== ANALYZE 83 ===");
  for (uint16_t i=0;i<n83;i++) {
    Serial.print("83,");
    Serial.print(i);
    for (int j=0;j<14;j++) {
      Serial.print(",");
      Serial.print(buf83[i].payload[j], HEX);
    }
    Serial.println();
  }
  Serial.println("=== END ANALYZE 83 ===\n");
}

static bool writeBytes(const uint8_t* p, size_t n){
  if (!chRx) return false;
  if (millis() - lastWriteMs < WRITE_GAP_MS) return false;
  bool ok = chRx.writeValue(p, n);
  if (ok) lastWriteMs = millis();
  return ok;
}

static void handleNotif(const char* src, const uint8_t* p, uint8_t n){
  if (!n) return;

  totalNotifs++;
  prefixCount[p[0]]++;
  if (n>=2 && p[0]==0x83 && p[1]==0x0E) total83++;
  if (p[0]==0xA1) totalA1++;

  // Capture 87/83 for static state analysis (POST_READY only, order preserved)
  if (st == ST_POST_READY) {
    if (n >= 6 && p[0]==0x87 && p[1]==0x04 && n87 < MAX_87) {
      buf87[n87].b1 = p[2]; buf87[n87].b2 = p[3]; buf87[n87].b3 = p[4];
      n87++;
    }
    if (n >= 16 && p[0]==0x83 && p[1]==0x0E && n83 < MAX_83) {
      for (int i=0;i<14;i++) buf83[n83].payload[i] = p[2+i];
      n83++;
    }
  }

  // Print every notif during POST_READY window, otherwise only non-83 packets
  bool verbose = (st == ST_POST_READY);
  if (verbose || !(n>=2 && p[0]==0x83 && p[1]==0x0E)) {
    Serial.print(src);
    Serial.print(" n=");
    Serial.print(n);
    Serial.print(" : ");
    printHex(p, n);
    Serial.println();
  }
}

static void enter(State s){
  st = s;
  tEnter = millis();
  if (s == ST_SCAN) {
    BLE.stopScan();
    BLE.scanForUuid(UUID_SERVICE_CUSTOM);
    Serial.println("SCAN");
  }
  if (s == ST_POST_READY) {
    postReadyStartMs = millis();
    Serial.println("POST_READY: capturing all notifications");
  }
  if (s == ST_DONE) {
    Serial.println("DONE");
    dumpSummary();
  }
}

static bool discoverChars(){
  BLEService svc = zoomDev.service(UUID_SERVICE_CUSTOM);
  if (!svc) return false;
  BLECharacteristic tx = svc.characteristic(UUID_TX_NOTIFY);
  BLECharacteristic rx = svc.characteristic(UUID_RX_WWR);
  BLECharacteristic fc = svc.characteristic(UUID_FLOW_CTRL);
  if (!tx || !rx || !fc) return false;
  chTx = tx; chRx = rx; chFlow = fc;
  return true;
}

static bool subscribeAll(){
  if (!chFlow.subscribe()) return false;
  if (!chTx.subscribe()) return false;
  return true;
}

// Same hello as main sketch (ZOOM_CONTROL_V4): D1 12 "2.3.3.1" + padding, 20 bytes
static bool sendHello(){
  const uint8_t hello[] = {0xD1,0x12,0x32,0x2E,0x33,0x2E,0x31,
                           0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
                           0x00,0x00,0x00,0x00};
  return writeBytes(hello, sizeof(hello));
}

static bool sendFamilyA_nonblocking(){
  static bool active=false;
  static size_t idx=0;
  static uint32_t lastMs=0;

  if (!active){ active=true; idx=0; lastMs=0; }

  const size_t cnt = sizeof(FAMILY_A)/sizeof(FAMILY_A[0]);
  if (idx >= cnt){
    active=false;
    return true;
  }

  if (millis() - lastMs < WRITE_GAP_MS) return false;
  if (writeBytes(FAMILY_A[idx].p, FAMILY_A[idx].n)){
    lastMs = millis();
    idx++;
  }
  return false;
}

static bool sawReady83(){
  bool got = false;
  if (chTx && chTx.valueUpdated()){
    uint8_t buf[96];
    int n = chTx.valueLength();
    if (n > (int)sizeof(buf)) n = sizeof(buf);
    chTx.readValue(buf, n);
    handleNotif("TX", buf, (uint8_t)n);
    if (n>=2 && buf[0]==0x83 && buf[1]==0x0E) got = true;
  }
  if (chFlow && chFlow.valueUpdated()){
    uint8_t buf[96];
    int n = chFlow.valueLength();
    if (n > (int)sizeof(buf)) n = sizeof(buf);
    chFlow.readValue(buf, n);
    handleNotif("FLOW", buf, (uint8_t)n);
    if (n>=2 && buf[0]==0x83 && buf[1]==0x0E) got = true;
  }
  return got;
}

static void pollNotifs(){
  if (chTx && chTx.valueUpdated()){
    uint8_t buf[96];
    int n = chTx.valueLength();
    if (n > (int)sizeof(buf)) n = sizeof(buf);
    chTx.readValue(buf, n);
    handleNotif("TX", buf, (uint8_t)n);
  }
  if (chFlow && chFlow.valueUpdated()){
    uint8_t buf[96];
    int n = chFlow.valueLength();
    if (n > (int)sizeof(buf)) n = sizeof(buf);
    chFlow.readValue(buf, n);
    handleNotif("FLOW", buf, (uint8_t)n);
  }
}

void setup(){
  Serial.begin(115200);
  while(!Serial) {}

  Serial.println("BOOT");

  if (!BLE.begin()) while(1);
  enter(ST_SCAN);
}

void loop(){
  BLE.poll();

  if (st == ST_SEND_HELLO || st == ST_SEND_FAMILY_A || st == ST_WAIT_READY || st == ST_POST_READY){
    pollNotifs();
  }

  switch(st){
    case ST_SCAN: {
      BLEDevice dev = BLE.available();
      if (dev){
        zoomDev = dev;
        BLE.stopScan();
        enter(ST_CONNECT);
      } else if (millis() - tEnter > SCAN_TIMEOUT_MS){
        enter(ST_SCAN);
      }
    } break;

    case ST_CONNECT: {
      if (!zoomDev.connect()){
        enter(ST_SCAN);
        break;
      }
      Serial.println("CONNECTED");
      enter(ST_DISCOVER);
    } break;

    case ST_DISCOVER: {
      if (!zoomDev.discoverAttributes()){
        zoomDev.disconnect();
        enter(ST_SCAN);
        break;
      }
      if (!discoverChars()){
        zoomDev.disconnect();
        enter(ST_SCAN);
        break;
      }
      enter(ST_SUBSCRIBE);
    } break;

    case ST_SUBSCRIBE: {
      if (!subscribeAll()){
        zoomDev.disconnect();
        enter(ST_SCAN);
        break;
      }
      enter(ST_SEND_HELLO);
    } break;

    case ST_SEND_HELLO: {
      if (sendHello()){
        enter(ST_SEND_FAMILY_A);
      }
    } break;

    case ST_SEND_FAMILY_A: {
      if (sendFamilyA_nonblocking()){
        enter(ST_WAIT_READY);
      }
    } break;

    case ST_WAIT_READY: {
      if (sawReady83()){
        Serial.println("READY");
        enter(ST_POST_READY);
      } else if (millis() - tEnter > READY_TIMEOUT_MS){
        zoomDev.disconnect();
        enter(ST_SCAN);
      }
    } break;

    case ST_POST_READY: {
      if (millis() - postReadyStartMs >= POST_READY_CAPTURE_MS){
        enter(ST_DONE);
      }
      if (!zoomDev.connected()){
        enter(ST_SCAN);
      }
    } break;

    case ST_DONE:
    default:
      break;
  }
}
