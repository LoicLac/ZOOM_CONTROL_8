/*
  Zoom F8n Pro BLE - Auto connect + handshake + READY + Serial control (fader/trim/pan)
  Board: Arduino Nano 33 BLE / BLE Rev2
  Library: ArduinoBLE

  LED states:
    - SCAN  : slow blink (1.2 s)
    - CONNECT/DISC/SUB/HANDSHAKE : fast blink (120 ms)
    - READY : slow breathing (4 s cycle)
    - FAIL / disconnected : triple-blink burst

  Serial commands (normalized 0..1023):
    - fader  <ch 0..7> <val 0..1023>      -> maps to BLE 0..125
    - trim   <ch 0..7> <val 0..1023>      -> maps to BLE 0..65
    - pan    <ch 0..7> <val 0..1023>      -> maps to BLE 16-bit (piecewise around center)
    - NO SERIAL mode (absolutely no Serial I/O):
        hold MODE button while booting/resetting


    - Read continuously (when READY)
    - A button cycles mode: FADER -> PAN -> TRIM -> ...
    - For current mode, each pot i controls channel i with normalized 0..1023

  Notes:
    - PAN encoding is two separate bytes: A1 04 04 ch <amount> <side>
        side=0: left/center, amount 0 (L100) .. 127 (center)
        side=1: right,       amount 0 (near center) .. 72 (R100)
      Values 128..255 in the amount byte are INVALID (gap confirmed by btsnoop).
*/

#include <ArduinoBLE.h>

// ============================================================================
//  COMPILE-TIME CONFIG — all user-tunable values grouped here
// ============================================================================

// ---- Hardware: Pins --------------------------------------------------------
static const int LED_PIN       = LED_BUILTIN;
static const int POT_PINS[8]   = {A0, A1, A2, A3, A4, A5, A6, A7};
static const int MODE_BTN_PIN  = 2;            // digital input — change to your wiring
static const bool MODE_BTN_PULLUP = true;      // true = INPUT_PULLUP, pressed = LOW

// ---- Hardware: Button ------------------------------------------------------
static const uint32_t BTN_DEBOUNCE_MS = 35;    // debounce window (ms)

// ---- BLE: Timing -----------------------------------------------------------
static const uint32_t SCAN_TIMEOUT_MS   = 15000;  // max scan duration before retry
static const uint32_t READY_TIMEOUT_MS  = 15000;  // max wait for 0x83 ready notification
static const uint32_t WRITE_GAP_MS      = 30;     // min gap between BLE writes (ms)
static const uint32_t RETRY_DELAY_MS    = 1200;   // pause before auto-retry after failure

// ---- BLE: Keepalive --------------------------------------------------------
static const bool     USE_KEEPALIVE        = false;
static const uint32_t KEEPALIVE_PERIOD_MS  = 200;

// ---- BLE: Liveness watchdog ------------------------------------------------
// If no notification received for this long while READY, assume link dead.
static const uint32_t NOTIF_LIVENESS_MS = 5000;

// ---- BLE: UUIDs (Zoom F8n Pro) — do not change unless protocol changes -----
static const char* UUID_SERVICE_CUSTOM = "70bcfdf1-3600-185b-d562-b64c851dc87d";
static const char* UUID_TX_NOTIFY      = "69abc935-4eb8-982e-6e55-b812ef754bbf";
static const char* UUID_RX_WWR         = "22eb6fbe-75e4-c334-f1ce-73b59c6847e0";
static const char* UUID_FLOW_CTRL      = "064131ea-592c-3ad9-4f8f-71db1b192828";

// ---- Pot: Reading & filtering ----------------------------------------------
// Pots are read round-robin (1 ch per tick), then filtered before sending.
//   Pipeline: raw ADC → noise gate → median-of-N → EMA → snap-to-ends → hysteresis
static const uint32_t POTS_SCAN_PERIOD_MS = 1;     // round-robin read interval (ms)
static const uint8_t  POT_MEDIAN_WIN      = 3;     // median filter window size (fixed median-of-3)

// ---- Pot: EMA smoothing ----------------------------------------------------
// Exponential Moving Average:  filtered += (raw - filtered) >> POT_EMA_SHIFT
//   shift 1 → alpha 1/2   (fast, noisy)
//   shift 2 → alpha 1/4
//   shift 3 → alpha 1/8   (default — good balance)
//   shift 4 → alpha 1/16  (smooth, sluggish)
static const uint8_t  POT_EMA_SHIFT       = 3;

// ---- Pot: Noise gate & endpoint snap ---------------------------------------
static const uint16_t POT_NOISE_GATE      = 2;     // ignore raw ADC change smaller than this
static const uint16_t POT_SNAP_MARGIN     = 6;     // snap to 0 or 1023 when within this margin

// ---- Pot: Send throttle ----------------------------------------------------
static const uint32_t POT_SEND_MIN_GAP_MS = 20;    // per-channel min send interval (ms)

// ---- Pot: Hysteresis (on normalized 0..1023, per mode) ---------------------
// Change vs last-sent value must exceed this to trigger a new BLE send.
static const uint16_t POT_HYST_FADER = 4;
static const uint16_t POT_HYST_TRIM  = 4;
static const uint16_t POT_HYST_PAN   = 6;          // slightly higher to avoid center jitter

// ---- PAN encoding ----------------------------------------------------------
// Two-field BLE encoding:  <amount> <side>
//   side=0  amount 0..127  (0 = L100, 127 = center)
//   side=1  amount 0..72   (0 ≈ center, 72 = R100)
static const uint8_t PAN_LEFT_MAX  = 127;          // amount at center  (side 0)
static const uint8_t PAN_RIGHT_MAX = 74;           // amount at R100    (side 1, +2 margin)
static const uint16_t PAN_CENTER_DEAD = 8;          // ±dead zone around norm 512 snaps to center

// ============================================================================

// ---------- Internal state ----------
enum AutoState {
  ST_BOOT=0, ST_SCAN, ST_CONNECT, ST_DISCOVER, ST_SUBSCRIBE, ST_KEEPALIVE_ON,
  ST_SEND_HELLO, ST_SEND_FAMILY_A, ST_WAIT_READY, ST_READY, ST_FAIL
};

AutoState st = ST_BOOT;

BLEDevice zoomDev;
BLEService svc;
BLECharacteristic chTx, chRx, chFlow;

bool hasSvc=false, hasTx=false, hasRx=false, hasFlow=false;
bool readyFlag=false;

uint32_t lastKeepaliveMs=0;
uint32_t tStateEnterMs=0;

uint32_t lastAnyNotifMs=0;
uint32_t last83Ms=0;

String cmdBuf;

// ---------- Runtime options ----------
static bool gSerialEnabled = true; // when false: no Serial.begin, no prints, no reads

// ---------- Mode control ----------
enum CtrlMode : uint8_t { MODE_FADER=0, MODE_PAN=1, MODE_TRIM=2 };
static CtrlMode gMode = MODE_FADER;

// ---------- Family A frames ----------
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

// Forward decl
static bool writeBytes(const uint8_t* p, size_t n);

// ---------- Family A sender (non-blocking) ----------
// Sends 1 frame per tick, spaced by WRITE_GAP_MS, so BLE.poll()/notifs keep running.
static bool     gFamAActive = false;
static size_t   gFamAIdx = 0;
static uint32_t gFamALastWriteMs = 0;

static inline void famAReset() {
  gFamAActive = false;
  gFamAIdx = 0;
  gFamALastWriteMs = 0;
}

static inline void famAStart() {
  gFamAActive = true;
  gFamAIdx = 0;
  gFamALastWriteMs = 0;
}

// Return: -1 fail, 0 in progress, 1 done
static int famATick() {
  if (!gFamAActive) famAStart();
  const size_t cnt = sizeof(FAMILY_A) / sizeof(FAMILY_A[0]);
  if (gFamAIdx >= cnt) {
    gFamAActive = false;
    return 1;
  }

  const uint32_t now = millis();
  if (gFamALastWriteMs && (now - gFamALastWriteMs < WRITE_GAP_MS)) return 0;

  if (!writeBytes(FAMILY_A[gFamAIdx].p, FAMILY_A[gFamAIdx].n)) {
    gFamAActive = false;
    return -1;
  }

  gFamALastWriteMs = now;
  gFamAIdx++;
  return 0;
}

// ---------- Startup-only print (no String, no heap alloc) ----------
static void startupPrint(const char* msg) {
  if (!gSerialEnabled) return;
  Serial.print(millis());
  Serial.print(" | ");
  Serial.println(msg);
}

static void enterState(AutoState ns, const char* why) {
  st = ns;
  tStateEnterMs = millis();
  if (gSerialEnabled) {
    Serial.print(millis());
    Serial.print(" | ST ");
    Serial.print((int)st);
    Serial.print(" ");
    Serial.println(why);
  }
}

static bool stateTimedOut(uint32_t limitMs) {
  return (millis() - tStateEnterMs) > limitMs;
}

static void hardFail(const char* why) {
  enterState(ST_FAIL, why);
}

// ---------- LED patterns ----------
// SCAN       : slow symmetric blink   (1.2 s period)
// Handshake  : fast symmetric blink   (120 ms period)
// READY      : heartbeat pulse        (brief 80 ms flash every 2 s)
// FAIL       : triple-blink burst     (3 short flashes then long pause)

static void ledUpdate() {
  static uint32_t tRef = 0;     // reference timestamp for current pattern
  uint32_t now = millis();

  switch (st) {

    // --- Scanning: slow blink (600 ms ON / 600 ms OFF) ---
    case ST_SCAN: {
      uint32_t elapsed = (now - tRef) % 1200u;
      digitalWrite(LED_PIN, elapsed < 600 ? HIGH : LOW);
      break;
    }

    // --- Handshake: fast blink (60 ms ON / 60 ms OFF) ---
    case ST_CONNECT:
    case ST_DISCOVER:
    case ST_SUBSCRIBE:
    case ST_KEEPALIVE_ON:
    case ST_SEND_HELLO:
    case ST_SEND_FAMILY_A:
    case ST_WAIT_READY: {
      uint32_t elapsed = (now - tRef) % 120u;
      digitalWrite(LED_PIN, elapsed < 60 ? HIGH : LOW);
      break;
    }

    // --- Ready: slow breathing via PWM-like soft ramp (4 s cycle) ---
    case ST_READY: {
      const uint32_t BREATH_MS = 4000;
      uint32_t elapsed = (now - tRef) % BREATH_MS;
      // Triangle wave 0 → 255 → 0 over one cycle
      uint16_t half = (uint16_t)(BREATH_MS / 2);
      uint16_t phase = (elapsed < half)
        ? (uint16_t)((elapsed * 255u) / half)           // ramp up
        : (uint16_t)(((BREATH_MS - elapsed) * 255u) / half); // ramp down
      // Gamma-ish curve: square for perceived linearity
      uint8_t pwm = (uint8_t)((phase * phase) >> 8);
      analogWrite(LED_PIN, pwm);
      break;
    }

    // --- Fail: triple-blink burst (3×80 ms flashes, 120 ms gaps, then 1 s pause) ---
    // Timeline within 1600 ms cycle:
    //   0-80 ON, 80-200 OFF, 200-280 ON, 280-400 OFF, 400-480 ON, 480-1600 OFF
    case ST_FAIL: {
      uint32_t elapsed = (now - tRef) % 1600u;
      bool on = (elapsed <  80) ||
                (elapsed >= 200 && elapsed < 280) ||
                (elapsed >= 400 && elapsed < 480);
      digitalWrite(LED_PIN, on ? HIGH : LOW);
      break;
    }

    default:
      digitalWrite(LED_PIN, LOW);
      break;
  }
}

// ---------- BLE helpers ----------
static void resetVarsButKeepBLE() {
  zoomDev = BLEDevice();
  svc = BLEService();
  chTx = BLECharacteristic();
  chRx = BLECharacteristic();
  chFlow = BLECharacteristic();
  hasSvc=hasTx=hasRx=hasFlow=false;
  readyFlag=false;
  lastKeepaliveMs=0;
  lastAnyNotifMs=last83Ms=0;
  famAReset();
}

static void disconnectIfNeeded() {
  if (zoomDev && zoomDev.connected()) zoomDev.disconnect();
}

static void pollNotifs() {
  if (!zoomDev || !zoomDev.connected()) return;

  if (hasTx && chTx.valueUpdated()) {
    int n = chTx.valueLength();
    uint8_t buf[32];
    if (n > (int)sizeof(buf)) n = sizeof(buf);
    chTx.readValue(buf, n);

    lastAnyNotifMs = millis();
    if (n>=2 && buf[0]==0x83 && buf[1]==0x0E) last83Ms = millis();
  }

  if (hasFlow && chFlow.valueUpdated()) {
    int n = chFlow.valueLength();
    uint8_t buf[8];
    if (n > (int)sizeof(buf)) n = sizeof(buf);
    chFlow.readValue(buf, n);
    lastAnyNotifMs = millis();
  }
}

static bool writeBytes(const uint8_t* p, size_t n) {
  if (!hasRx) return false;
  return chRx.writeValue(p, n);
}

static void keepaliveTick() {
  if (!USE_KEEPALIVE) return;
  if (!zoomDev || !zoomDev.connected()) return;
  if (!hasRx) return;
  uint32_t now = millis();
  if (now - lastKeepaliveMs >= KEEPALIVE_PERIOD_MS) {
    const uint8_t a[] = {0x80,0x01,0x00};
    chRx.writeValue(a, sizeof(a));
    lastKeepaliveMs = now;
  }
}

// ---------- Protocol tx ----------
static bool sendHello() {
  const uint8_t hello[] = {0xD1,0x12,0x32,0x2E,0x33,0x2E,0x31,
                           0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
                           0x00,0x00,0x00,0x00};
  return writeBytes(hello, sizeof(hello));
}

// Raw (device format)
static bool sendFaderRaw(uint8_t ch, uint8_t val) {
  uint8_t p[5] = {0xA1,0x03,0x05,ch,val};
  return writeBytes(p, 5);
}

// From btsnoop: A1 04 03 ch trim 00, trim in [0..65]
static bool sendTrimRaw(uint8_t ch, uint8_t trimVal) {
  uint8_t p[6] = {0xA1,0x04,0x03,ch,trimVal,0x00};
  return writeBytes(p, 6);
}

// From btsnoop: A1 04 04 ch <amount> <side>
//   side=0 amount=0..127 (left/center)   side=1 amount=0..72 (right)
static bool sendPanRaw(uint8_t ch, uint8_t amount, uint8_t side) {
  uint8_t p[6] = {0xA1,0x04,0x04,ch,amount,side};
  return writeBytes(p, 6);
}

// ---------- Normalization helpers (0..1023) ----------
static uint16_t clampU16(int v, int lo, int hi) {
  if (v < lo) return (uint16_t)lo;
  if (v > hi) return (uint16_t)hi;
  return (uint16_t)v;
}

static uint8_t mapNormToFaderRaw(uint16_t norm0_1023) {
  // 0..1023 -> 0..125 with rounding
  uint32_t x = norm0_1023;
  uint32_t y = (x * 125u + 511u) / 1023u;
  if (y > 125u) y = 125u;
  return (uint8_t)y;
}

static uint8_t mapNormToTrimRaw(uint16_t norm0_1023) {
  // 0..1023 -> 0..65 with rounding
  uint32_t x = norm0_1023;
  uint32_t y = (x * 65u + 511u) / 1023u;
  if (y > 65u) y = 65u;
  return (uint8_t)y;
}

// Map normalized 0..1023 to PAN two-field encoding.
//   norm=0    -> side=0 amount=0   (L100, full left)
//   norm=512  -> side=0 amount=127 (center)
//   norm=1023 -> side=1 amount=72  (R100, full right)
// A dead zone of ±PAN_CENTER_DEAD around 512 snaps to exact center.
static void mapNormToPan(uint16_t norm0_1023, uint8_t& amount, uint8_t& side) {
  const uint16_t cLo = 512u - PAN_CENTER_DEAD; // 504
  const uint16_t cHi = 512u + PAN_CENTER_DEAD; // 520

  if (norm0_1023 >= cLo && norm0_1023 <= cHi) {
    // Dead zone: snap to exact center
    side = 0;
    amount = PAN_LEFT_MAX; // 127 = center
  } else if (norm0_1023 < cLo) {
    // Left: norm 0..cLo-1 -> amount 0..126, side=0
    side = 0;
    uint32_t x = norm0_1023;
    uint32_t y = (x * 126u + (cLo / 2)) / cLo;
    if (y > 126u) y = 126u;
    amount = (uint8_t)y;
  } else {
    // Right: norm cHi+1..1023 -> amount 1..PAN_RIGHT_MAX, side=1
    side = 1;
    uint32_t range = 1023u - cHi;  // 503
    uint32_t x = (uint32_t)(norm0_1023 - cHi);  // 1..503
    uint32_t y = (x * (uint32_t)PAN_RIGHT_MAX + (range / 2)) / range;
    if (y > PAN_RIGHT_MAX) y = PAN_RIGHT_MAX;
    if (y == 0) y = 1;
    amount = (uint8_t)y;
  }
}

// Public normalized API used by serial + pot placeholder
static bool sendFaderNorm(uint8_t ch, uint16_t norm0_1023) {
  return sendFaderRaw(ch, mapNormToFaderRaw(norm0_1023));
}

static bool sendTrimNorm(uint8_t ch, uint16_t norm0_1023) {
  return sendTrimRaw(ch, mapNormToTrimRaw(norm0_1023));
}

static bool sendPanNorm(uint8_t ch, uint16_t norm0_1023) {
  uint8_t amount, side;
  mapNormToPan(norm0_1023, amount, side);
  return sendPanRaw(ch, amount, side);
}

// ---------- Serial commands (fader/trim/pan only, no log output) ----------
static void handleSerialLine(String s) {
  if (!gSerialEnabled) return;
  s.trim();
  if (s.length()==0) return;
  if (st != ST_READY) return;

  auto parseChVal = [&](int& ch, int& val)->bool {
    int sp1 = s.indexOf(' ');
    int sp2 = (sp1>0) ? s.indexOf(' ', sp1+1) : -1;
    if (sp1<0 || sp2<0) return false;
    ch = s.substring(sp1+1, sp2).toInt();
    val = s.substring(sp2+1).toInt();
    return true;
  };

  int ch, val;

  if (s.startsWith("fader")) {
    if (!parseChVal(ch, val)) return;
    if (ch<0 || ch>7 || val<0 || val>1023) return;
    sendFaderNorm((uint8_t)ch, (uint16_t)val);
    return;
  }

  if (s.startsWith("trim")) {
    if (!parseChVal(ch, val)) return;
    if (ch<0 || ch>7 || val<0 || val>1023) return;
    sendTrimNorm((uint8_t)ch, (uint16_t)val);
    return;
  }

  if (s.startsWith("pan")) {
    if (!parseChVal(ch, val)) return;
    if (ch<0 || ch>7 || val<0 || val>1023) return;
    sendPanNorm((uint8_t)ch, (uint16_t)val);
    return;
  }
}

// ---------- AUTO state machine ----------
static void autoStep() {
  switch(st) {
    case ST_BOOT:
      enterState(ST_SCAN, "boot");
      break;

    case ST_SCAN: {
      resetVarsButKeepBLE();
      startupPrint("scanning...");
      BLE.scanForUuid(UUID_SERVICE_CUSTOM);
      enterState(ST_CONNECT, "scan started");
      break;
    }

    case ST_CONNECT: {
      BLEDevice dev = BLE.available();
      if (dev) {
        BLE.stopScan();
        zoomDev = dev;
        if (gSerialEnabled) {
          Serial.print(millis());
          Serial.print(" | FOUND ");
          Serial.println(dev.address());
        }
        bool ok = zoomDev.connect();
        if (!ok) { hardFail("connect failed"); return; }
        enterState(ST_DISCOVER, "connected");
        return;
      }
      if (stateTimedOut(SCAN_TIMEOUT_MS)) {
        BLE.stopScan();
        hardFail("scan timeout");
      }
      break;
    }

    case ST_DISCOVER: {
      startupPrint("discovering...");
      bool ok = zoomDev.discoverAttributes();
      if (!ok) { hardFail("discover failed"); return; }

      svc = zoomDev.service(UUID_SERVICE_CUSTOM);
      hasSvc = (bool)svc;
      if (!hasSvc) { hardFail("service not found"); return; }

      chTx   = svc.characteristic(UUID_TX_NOTIFY);
      chRx   = svc.characteristic(UUID_RX_WWR);
      chFlow = svc.characteristic(UUID_FLOW_CTRL);

      hasTx   = (bool)chTx;
      hasRx   = (bool)chRx;
      hasFlow = (bool)chFlow;

      if (!hasTx || !hasRx) { hardFail("missing tx/rx"); return; }

      enterState(ST_SUBSCRIBE, "discovered");
      break;
    }

    case ST_SUBSCRIBE: {
      if (hasTx && chTx.canSubscribe()) {
        if (!chTx.subscribe()) { hardFail("sub TX failed"); return; }
      } else {
        hardFail("TX cannot subscribe");
        return;
      }

      if (hasFlow && chFlow.canSubscribe()) {
        chFlow.subscribe(); // FLOW not mandatory
      }

      enterState(ST_KEEPALIVE_ON, "subscribed");
      break;
    }

    case ST_KEEPALIVE_ON:
      enterState(ST_SEND_HELLO, "keepalive stage");
      break;

    case ST_SEND_HELLO: {
      bool ok = sendHello();
      if (!ok) { hardFail("hello write failed"); return; }
      enterState(ST_SEND_FAMILY_A, "hello sent");
      break;
    }

    case ST_SEND_FAMILY_A: {
      int r = famATick();
      if (r < 0) {
        hardFail("family_a write failed");
        return;
      }
      if (r > 0) {
        enterState(ST_WAIT_READY, "family_a sent");
        return;
      }
      // still in progress
      break;
    }

    case ST_WAIT_READY: {
      if (last83Ms != 0 && (last83Ms >= tStateEnterMs)) {
        readyFlag = true;
        startupPrint("READY");
        enterState(ST_READY, "ready");
        return;
      }
      if (stateTimedOut(READY_TIMEOUT_MS)) {
        hardFail("ready timeout");
        return;
      }
      break;
    }

    case ST_READY:
      // Liveness watchdog: Zoom sends periodic notifications.
      // If none received for NOTIF_LIVENESS_MS, assume link is dead.
      if (lastAnyNotifMs && (millis() - lastAnyNotifMs > NOTIF_LIVENESS_MS)) {
        hardFail("notif timeout");
      }
      break;

    case ST_FAIL:
      if (stateTimedOut(RETRY_DELAY_MS)) {
        disconnectIfNeeded();
        enterState(ST_SCAN, "auto-retry");
      }
      break;
  }

  if (st != ST_BOOT && st != ST_SCAN && st != ST_CONNECT && st != ST_FAIL
      && zoomDev && !zoomDev.connected()) {
    hardFail("link lost");
  }
}

// ---------- Per-bank value memory + pickup (catch) state ----------
// 3 banks: MODE_FADER=0, MODE_PAN=1, MODE_TRIM=2
static uint16_t gBankVal[3][8] = {{0}};  // last-sent value per bank per channel
static bool     gCaught[3][8]  = {{0}};  // true = pot has picked up the stored value

// ---------- Mode button + Pots ----------
static uint32_t gLastBtnChangeMs = 0;
static int gLastBtnStable = MODE_BTN_PULLUP ? HIGH : LOW;
static int gLastBtnRead = MODE_BTN_PULLUP ? HIGH : LOW;

static void modeButtonTick() {
  int raw = digitalRead(MODE_BTN_PIN);
  uint32_t now = millis();

  if (raw != gLastBtnRead) {
    gLastBtnRead = raw;
    gLastBtnChangeMs = now;
  }

  if ((now - gLastBtnChangeMs) >= BTN_DEBOUNCE_MS) {
    if (gLastBtnStable != gLastBtnRead) {
      gLastBtnStable = gLastBtnRead;

      bool pressed = MODE_BTN_PULLUP ? (gLastBtnStable == LOW) : (gLastBtnStable == HIGH);
      if (pressed) {
        // cycle
        if (gMode == MODE_FADER) gMode = MODE_PAN;
        else if (gMode == MODE_PAN) gMode = MODE_TRIM;
        else gMode = MODE_FADER;

        // Reset pickup flags: all pots must catch the stored value before sending
        for (uint8_t i = 0; i < 8; i++) gCaught[gMode][i] = false;
      }
    }
  }
}

// Per-channel pot filter state
static uint16_t gPotRawRing[8][POT_MEDIAN_WIN] = {{0}}; // last N raw reads (median window)
static uint8_t  gPotRawIdx[8] = {0};
static uint16_t gPotLastRaw[8] = {0};                   // previous raw read (for noise gate)
static uint16_t gPotFilt[8] = {0};                      // filtered normalized 0..1023
static bool     gPotInited[8] = {0};

static uint16_t gPotPrevFilt[8] = {0};   // previous filtered value (for crossing detection)

static uint32_t gPotLastSentMs[8] = {0};
static uint32_t gLastPotScanMs = 0;
static uint8_t  gPotScanCh = 0;

static inline uint16_t u16absdiff(uint16_t a, uint16_t b) {
  return (a > b) ? (a - b) : (b - a);
}

static inline uint16_t median3_u16(uint16_t a, uint16_t b, uint16_t c) {
  // Branchy but fast and tiny.
  if (a > b) { uint16_t t=a; a=b; b=t; } // a<=b
  if (b > c) { uint16_t t=b; b=c; c=t; } // b<=c
  if (a > b) { uint16_t t=a; a=b; b=t; } // a<=b
  return b;
}

static inline uint16_t potHysteresis(CtrlMode m, uint16_t norm0_1023) {
  if (m == MODE_FADER) return POT_HYST_FADER;
  if (m == MODE_TRIM)  return POT_HYST_TRIM;
  // PAN: increase hysteresis near center to avoid "tick-tick" around 512
  uint16_t d = u16absdiff(norm0_1023, 512);
  return (d < 16) ? (uint16_t)(POT_HYST_PAN + 4) : POT_HYST_PAN;
}

static void potsTick() {
  if (st != ST_READY) return;

  uint32_t now = millis();
  if (now - gLastPotScanMs < POTS_SCAN_PERIOD_MS) return;
  gLastPotScanMs = now;

  // Round-robin: read 1 channel per tick for smoother loop timing
  uint8_t ch = gPotScanCh;
  gPotScanCh = (uint8_t)((gPotScanCh + 1) & 0x07);

  int a = analogRead(POT_PINS[ch]); // Nano 33 BLE: typically 0..1023
  uint16_t norm = clampU16(a, 0, 1023);

  // Noise gate: reject jitter smaller than POT_NOISE_GATE
  if (gPotInited[ch] && u16absdiff(norm, gPotLastRaw[ch]) < POT_NOISE_GATE) {
    norm = gPotLastRaw[ch];
  }
  gPotLastRaw[ch] = norm;

  // Update median-of-3 ring
  uint8_t idx = gPotRawIdx[ch];
  gPotRawRing[ch][idx] = norm;
  gPotRawIdx[ch] = (uint8_t)((idx + 1) % POT_MEDIAN_WIN);

  uint16_t r0 = gPotRawRing[ch][0];
  uint16_t r1 = gPotRawRing[ch][1];
  uint16_t r2 = gPotRawRing[ch][2];
  uint16_t med = median3_u16(r0, r1, r2);

  // EMA filter
  if (!gPotInited[ch]) {
    gPotInited[ch] = true;
    gPotFilt[ch] = med;
    gPotPrevFilt[ch] = med;
    // Seed all 3 banks with the initial reading
    for (uint8_t m = 0; m < 3; m++) {
      gBankVal[m][ch] = med;
      gCaught[m][ch] = (m == (uint8_t)gMode); // only current mode is caught
    }
    gPotLastSentMs[ch] = now;
    return;
  }

  uint16_t prevF = gPotFilt[ch];
  int16_t err = (int16_t)med - (int16_t)prevF;
  uint16_t f = (uint16_t)((int32_t)prevF + ((int32_t)err >> POT_EMA_SHIFT));

  // Snap to endpoints when near 0 or 1023
  if (f <= POT_SNAP_MARGIN)              f = 0;
  else if (f >= (1023 - POT_SNAP_MARGIN)) f = 1023;

  gPotFilt[ch] = f;

  uint8_t bank = (uint8_t)gMode;

  // --- Pickup / catch gate ---
  if (!gCaught[bank][ch]) {
    uint16_t target = gBankVal[bank][ch];
    uint16_t hyst = potHysteresis(gMode, target);

    // Catch if pot is within hysteresis of target, or has crossed through it
    bool withinHyst = (u16absdiff(f, target) <= hyst);
    bool crossed = ((prevF <= target && f >= target) ||
                    (prevF >= target && f <= target));

    if (withinHyst || crossed) {
      gCaught[bank][ch] = true;
      // Snap to target so first send doesn't jump
      f = target;
      gPotFilt[ch] = f;
    } else {
      gPotPrevFilt[ch] = f;
      return; // not caught yet, skip send
    }
  }

  gPotPrevFilt[ch] = f;

  // Send only on meaningful change vs bank value (hysteresis + min gap)
  if (now - gPotLastSentMs[ch] < POT_SEND_MIN_GAP_MS) return;

  uint16_t hyst = potHysteresis(gMode, f);
  if (u16absdiff(f, gBankVal[bank][ch]) < hyst) return;

  bool ok = false;
  switch (gMode) {
    case MODE_FADER: ok = sendFaderNorm(ch, f); break;
    case MODE_PAN:   ok = sendPanNorm(ch, f);   break;
    case MODE_TRIM:  ok = sendTrimNorm(ch, f);  break;
  }

  if (ok) {
    gBankVal[bank][ch] = f;
    gPotLastSentMs[ch] = now;
  }
}

void setup() {
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  pinMode(MODE_BTN_PIN, MODE_BTN_PULLUP ? INPUT_PULLUP : INPUT);

  // Boot-time "NO SERIAL" latch: hold MODE button during reset/power-up
  delay(5);
  bool bootPressed = MODE_BTN_PULLUP ? (digitalRead(MODE_BTN_PIN) == LOW) : (digitalRead(MODE_BTN_PIN) == HIGH);
  if (bootPressed) {
    gSerialEnabled = false;
  } else {
    gSerialEnabled = true;
  }

  if (gSerialEnabled) {
    Serial.begin(115200);
    uint32_t t0 = millis();
    while (!Serial && (millis() - t0 < 2000)) {}
    Serial.println("\nZoom BLE AUTO handshake");
  }

  if (!BLE.begin()) {
    if (gSerialEnabled) Serial.println("BLE.begin() failed");
    while(1){}
  }

  BLE.setEventHandler(BLEDisconnected, [](BLEDevice) {
    startupPrint("BLE disconnected");
    disconnectIfNeeded();
    enterState(ST_FAIL, "disconnected");
  });

  enterState(ST_BOOT, "setup done");
}

void loop() {
  BLE.poll();
  pollNotifs();
  keepaliveTick();

  autoStep();
  ledUpdate();

  // placeholder I/O layer
  modeButtonTick();
  potsTick();

  // Non-blocking serial read (line-based)
  if (gSerialEnabled) {
    while (Serial.available()) {
      char c = (char)Serial.read();
      if (c=='\n' || c=='\r') {
        if (cmdBuf.length()>0) {
          handleSerialLine(cmdBuf);
          cmdBuf="";
        }
      } else {
        cmdBuf += c;
        if (cmdBuf.length()>220) cmdBuf = "";
      }
    }
  }
}