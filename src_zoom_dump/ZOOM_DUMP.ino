/*
  ZOOM_DUMP — Zoom F8n Pro BLE test sketch
  Connect/handshake/READY + capture trames 87 uniquement.
  Capture auto : démarre à la première trame 87 reçue, s'arrête 3 s après la dernière.
  Board: Arduino Nano 33 BLE / BLE Rev2
  Library: ArduinoBLE

  Serial commands:
    - SNAP   (affiche la table 87)
    - CLEAR  (efface les données)
*/

#include <ArduinoBLE.h>

// ===== Zoom UUIDs =====
static const char* UUID_SERVICE_CUSTOM = "70bcfdf1-3600-185b-d562-b64c851dc87d";
static const char* UUID_TX_NOTIFY      = "69abc935-4eb8-982e-6e55-b812ef754bbf";
static const char* UUID_RX_WWR         = "22eb6fbe-75e4-c334-f1ce-73b59c6847e0";
static const char* UUID_FLOW_CTRL      = "064131ea-592c-3ad9-4f8f-71db1b192828";

// ===== Timing =====
static const uint32_t SCAN_TIMEOUT_MS  = 15000;
static const uint32_t READY_TIMEOUT_MS = 15000;
static const uint32_t WRITE_GAP_MS     = 30;

// ===== Family A frames (from your working sketch) =====
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

enum State {
  ST_SCAN=0, ST_CONNECT, ST_DISCOVER, ST_SUBSCRIBE,
  ST_SEND_HELLO, ST_SEND_FAMILY_A, ST_WAIT_READY, ST_READY
};
State st = ST_SCAN;

uint32_t tEnter = 0;
uint32_t lastWriteMs = 0;

// ===== Capture storage =====
// Raw "87" table: keep last value per (b2,b3,b4,b5) signature
struct Raw87 {
  bool used;
  uint8_t b1,b2,b3,b4,b5,b6; // full 6 bytes (b1 should be 0x87)
  uint32_t seenCount;
};
static const uint16_t RAW87_MAX = 256;
Raw87 raw87[RAW87_MAX];

static const uint32_t CAPTURE87_IDLE_MS = 3000;  // fin capture si plus de trame 87 depuis 3 s
static bool capture87Active = false;             // true dès la première 87 reçue
static uint32_t last87Ms = 0;                    // millis() de la dernière trame 87

// ===== Utils =====
static void clearStorage() {
  for (uint16_t i=0;i<RAW87_MAX;i++) raw87[i].used=false;
  capture87Active = false;
}

static void printHex(const uint8_t* p, int n) {
  for (int i=0;i<n;i++){
    if (p[i] < 16) Serial.print('0');
    Serial.print(p[i], HEX);
    if (i != n-1) Serial.print(' ');
  }
}

static void snapPrint() {
  Serial.println("start snapshot");
  for (uint16_t i=0;i<RAW87_MAX;i++){
    if (!raw87[i].used) continue;
    Serial.print(i); Serial.print(" : ");
    uint8_t b[6] = {raw87[i].b1,raw87[i].b2,raw87[i].b3,raw87[i].b4,raw87[i].b5,raw87[i].b6};
    printHex(b,6);
    Serial.print(" ; ");
    Serial.println(raw87[i].seenCount);
  }
  Serial.println("end snapshot");
}

// simplistic hashing on bytes 2..6
static uint16_t hash87(uint8_t b2,uint8_t b3,uint8_t b4,uint8_t b5,uint8_t b6){
  uint16_t h = 0x1234;
  h ^= (uint16_t)b2 * 257u;
  h ^= (uint16_t)b3 * 131u;
  h ^= (uint16_t)b4 *  97u;
  h ^= (uint16_t)b5 *  59u;
  h ^= (uint16_t)b6 *  17u;
  return (uint16_t)(h % RAW87_MAX);
}

static void store87(const uint8_t* p, uint8_t n){
  if (n < 6) return;
  if (!capture87Active) {
    capture87Active = true;
    Serial.println("CAPTURING");
  }
  last87Ms = millis();
  uint16_t idx = hash87(p[1],p[2],p[3],p[4],p[5]);
  for (uint16_t k=0;k<RAW87_MAX;k++){
    uint16_t i = (idx + k) % RAW87_MAX;
    if (!raw87[i].used){
      raw87[i].used = true;
      raw87[i].b1=p[0]; raw87[i].b2=p[1]; raw87[i].b3=p[2];
      raw87[i].b4=p[3]; raw87[i].b5=p[4]; raw87[i].b6=p[5];
      raw87[i].seenCount = 1;
      return;
    }
    if (raw87[i].b2==p[1] && raw87[i].b3==p[2] && raw87[i].b4==p[3] && raw87[i].b5==p[4] && raw87[i].b6==p[5]){
      raw87[i].seenCount++;
      return;
    }
  }
}

// ===== BLE write helper =====
static bool writeBytes(const uint8_t* p, size_t n){
  if (!chRx) return false;
  if (millis() - lastWriteMs < WRITE_GAP_MS) return false;
  bool ok = chRx.writeValue(p, n);
  if (ok) lastWriteMs = millis();
  return ok;
}

// ===== Notification handler =====
static void handleNotif(const uint8_t* p, uint8_t n){
  if (n == 0) return;
  // READY: 0x83 0x0E (unchanged for state machine)
  if (n >= 2 && p[0] == 0x83 && p[1] == 0x0E) return;
  // Store 87 frames only
  if (n >= 6 && p[0] == 0x87) store87(p, n);
}

// ===== State machine =====
static void enter(State s){
  st = s;
  tEnter = millis();
  if (s == ST_SCAN) {
    BLE.stopScan();
    BLE.scanForUuid(UUID_SERVICE_CUSTOM);
    Serial.println("SCAN");
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
  // flow control notify
  if (!chFlow.subscribe()) return false;
  // tx notify
  if (!chTx.subscribe()) return false;
  return true;
}

static bool sendHello(){
  // from your working sequence: "WRITE A OK" twice etc.
  // Keep minimal: write 0x41 0x00 to flow ctrl twice (matches "A OK" pattern in your log).
  const uint8_t A[] = {0x41,0x00};
  return writeBytes(A,2);
}

static bool sendFamilyA_nonblocking(){
  static bool active=false;
  static size_t idx=0;
  static uint32_t lastMs=0;

  if (!active){ active=true; idx=0; lastMs=0; }

  if (idx >= (sizeof(FAMILY_A)/sizeof(FAMILY_A[0]))){
    active=false;
    return true; // finished
  }

  if (millis() - lastMs < WRITE_GAP_MS) return false;
  if (writeBytes(FAMILY_A[idx].p, FAMILY_A[idx].n)){
    lastMs = millis();
    idx++;
  }
  return false;
}

static bool sawReady83(){
  // ArduinoBLE only gives "value updated" API, so we poll characteristics for updates.
  // We treat any notification starting with 0x83 0x0E as READY witness.
  if (chTx && chTx.valueUpdated()){
    uint8_t buf[64];
    int n = chTx.valueLength();
    if (n > (int)sizeof(buf)) n = sizeof(buf);
    chTx.readValue(buf, n);
    handleNotif(buf, (uint8_t)n);
    if (n >= 2 && buf[0]==0x83 && buf[1]==0x0E) return true;
  }
  if (chFlow && chFlow.valueUpdated()){
    uint8_t buf[64];
    int n = chFlow.valueLength();
    if (n > (int)sizeof(buf)) n = sizeof(buf);
    chFlow.readValue(buf, n);
    handleNotif(buf, (uint8_t)n);
    if (n >= 2 && buf[0]==0x83 && buf[1]==0x0E) return true;
  }
  return false;
}

static void pollNotifs(){
  if (chTx && chTx.valueUpdated()){
    uint8_t buf[64];
    int n = chTx.valueLength();
    if (n > (int)sizeof(buf)) n = sizeof(buf);
    chTx.readValue(buf, n);
    handleNotif(buf, (uint8_t)n);
  }
  if (chFlow && chFlow.valueUpdated()){
    uint8_t buf[64];
    int n = chFlow.valueLength();
    if (n > (int)sizeof(buf)) n = sizeof(buf);
    chFlow.readValue(buf, n);
    handleNotif(buf, (uint8_t)n);
  }
}

// ===== Serial parser =====
static bool readLine(String& out){
  while (Serial.available()){
    char c = (char)Serial.read();
    if (c == '\r') continue;
    if (c == '\n'){
      out.trim();
      return out.length() > 0;
    }
    out += c;
    if (out.length() > 120) { out = ""; } // safety
  }
  return false;
}

static void handleCommand(const String& line){
  String s = line; s.trim();
  if (s.length() == 0) return;

  // Tokenize
  String tok[5];
  int nt = 0;
  int i = 0;
  while (i < (int)s.length() && nt < 5){
    while (i < (int)s.length() && s.charAt(i) == ' ') i++;
    if (i >= (int)s.length()) break;
    int j = i;
    while (j < (int)s.length() && s.charAt(j) != ' ') j++;
    tok[nt++] = s.substring(i,j);
    i = j;
  }
  if (nt == 0) return;

  String cmd = tok[0]; cmd.toUpperCase();

  if (cmd == "SNAP"){
    snapPrint();
    return;
  }
  if (cmd == "CLEAR"){
    clearStorage();
    return;
  }
}

// ===== Arduino setup/loop =====
void setup(){
  Serial.begin(115200);
  while(!Serial) { /* wait */ }

  Serial.println("BOOT");

  clearStorage();

  if (!BLE.begin()) while(1);
  BLE.setEventHandler(BLEDiscovered, NULL);
  enter(ST_SCAN);
}

void loop(){
  BLE.poll();

  // Read notifications as soon as subscribed (to catch 87 before/after READY)
  if (st == ST_SEND_HELLO || st == ST_SEND_FAMILY_A || st == ST_WAIT_READY || st == ST_READY){
    pollNotifs();
  }

  // Fin capture 87 : plus de trame depuis 3 s -> SNAP puis stop
  if (capture87Active && (uint32_t)(millis() - last87Ms) >= CAPTURE87_IDLE_MS){
    capture87Active = false;
    snapPrint();
  }

  // Serial
  static String line;
  if (readLine(line)){
    handleCommand(line);
    line = "";
  }

  // State machine
  switch(st){
    case ST_SCAN: {
      BLEDevice dev = BLE.available();
      if (dev){
        if (dev.hasLocalName() && dev.localName().length()){
          // optional filter by service uuid already used by scanForUuid
        }
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
      // send "A" twice like your working log
      static uint8_t step=0;
      if (step == 0){
        if (sendHello()) { step=1; }
      } else if (step == 1){
        if (sendHello()) { step=0; enter(ST_SEND_FAMILY_A); }
      }
    } break;

    case ST_SEND_FAMILY_A: {
      if (sendFamilyA_nonblocking()){
        enter(ST_WAIT_READY);
      }
    } break;

    case ST_WAIT_READY: {
      if (sawReady83()){
        enter(ST_READY);
      } else if (millis() - tEnter > READY_TIMEOUT_MS){
        zoomDev.disconnect();
        enter(ST_SCAN);
      }
    } break;

    case ST_READY: {
      if (!zoomDev.connected()) enter(ST_SCAN);
    } break;

    default: break;
  }
}
