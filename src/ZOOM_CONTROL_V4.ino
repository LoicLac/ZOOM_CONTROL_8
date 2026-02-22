/*
  Zoom F8n Pro BLE Controller
  Board: Arduino Nano 33 BLE / BLE Rev2
  Libraries: ArduinoBLE, Adafruit AW9523

  Connects to a Zoom F8n Pro via BLE, performs the full handshake,
  then controls 27 parameters with 8 physical potentiometers:
    - 8 input faders  (bank FADER, 0..121 raw)
    - 8 trims         (bank TRIM,  0..65 raw)
    - 8 pans          (bank PAN,   amount+side encoding)
    - 3 output faders (bank HOLD: LR 0..109, MAIN/SUB 0..121)

  At connection, the Zoom sends a state dump (3 A1 notifications:
  sub-cmd 0x00=trim, 0x01=pan, 0x02=faders). These are parsed to
  initialize gBankVal[][] so pots start with the real Zoom state.
  All pots must "catch" (cross through) the target value before sending.

  MODE button:
    - Short press: cycle FADER -> PAN -> TRIM
    - Long press (hold): enter HOLD bank (LR/MAIN/SUB + brightness)
    - Hold during boot: disable serial entirely

  LED (AW9523):
    - Pins 0-7: catch LEDs (brightness proportional to distance from target)
    - Pins 8-10: RGB status (green=FADER, yellow=PAN, red=TRIM, blue=HOLD)
    - Not connected: slow white blink; AW9523 absent: built-in LED fast blink

  Serial (115200, when enabled):
    - fader <ch 0..7> <val 0..1023>
    - trim  <ch 0..7> <val 0..1023>
    - pan   <ch 0..7> <val 0..1023>
*/

#include <ArduinoBLE.h>
#include <Wire.h>
#include <Adafruit_AW9523.h>
#include <NanoBLEFlashPrefs.h>

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

// ---- Pot: EMA smoothing (per type) -----------------------------------------
// Exponential Moving Average:  filtered += (raw - filtered) >> shift
//   shift 0 → alpha 1     (pass-through, no smoothing)
//   shift 1 → alpha 1/2   (fast)
//   shift 2 → alpha 1/4   (responsive)
//   shift 3 → alpha 1/8   (smooth)
//   shift 4 → alpha 1/16  (very smooth, sluggish)
static const uint8_t  POT_EMA_SHIFT_FADER = 3;     // smooth continuous fader feel
static const uint8_t  POT_EMA_SHIFT_PAN   = 2;     // responsive, less lag than fader
static const uint8_t  POT_EMA_SHIFT_TRIM  = 0;     // no smoothing — immediate discrete steps
static const uint8_t  POT_FRAC_BITS       = 8;     // fixed-point fractional bits (eliminates EMA integer stall)

// ---- Pot: Noise gate & endpoint snap ---------------------------------------
static const uint16_t POT_NOISE_GATE      = 2;     // ignore raw ADC change smaller than this
static const uint16_t POT_SNAP_MARGIN     = 6;     // snap to 0 or 1023 when within this margin

// ---- Pot: Send throttle ----------------------------------------------------
static const uint32_t POT_SEND_MIN_GAP_MS = 12;    // per-channel min send interval (ms)
static const uint32_t BLE_GLOBAL_MIN_GAP_MS = 15;  // min gap between ANY two pot BLE writes (global)

// ---- Pot: Hysteresis (on normalized 0..1023, per mode) ---------------------
// Change vs last-sent value must exceed this to trigger a new BLE send.
static const uint16_t POT_HYST_FADER = 4;
static const uint16_t POT_HYST_TRIM  = 4;
static const uint16_t POT_HYST_PAN   = 6;          // slightly higher to avoid center jitter

// ---- PAN encoding ----------------------------------------------------------
static const uint8_t PAN_LEFT_MAX  = 127;          // side=0 amount at center
static const uint8_t PAN_RIGHT_MAX = 74;           // side=1 amount at R100 (+2 margin)
static const uint16_t PAN_CENTER_DEAD = 8;          // ±dead zone around norm 512

// ---- HOLD (4th bank) -------------------------------------------------------
static const uint32_t HOLD_MODE_TIME_MS = 500;
static const uint8_t  HOLDED_BANK       = 3;

// ---- AW9523 / Catch LEDs --------------------------------------------------
static const uint8_t AW9523_ADDR         = 0x58;
static const uint8_t CATCH_LED_DIM_MIN   = 10;   // % of global ceiling when far
static const uint8_t CATCH_LED_DIM_MAX   = 80;   // % of global ceiling when near
static const uint8_t CATCH_LED_BLINK_PCT = 100;  // % of global ceiling during blink
static const uint32_t CATCH_LED_BLINK_HALF_MS = 40; // half-period per blink flash

// ---- VisionLed (common-anode RGB on AW9523 pins 8,9,10) -------------------
static const uint8_t VLED_PIN_R = 8;
static const uint8_t VLED_PIN_G = 9;
static const uint8_t VLED_PIN_B = 10;
static const uint32_t VLED_NOT_CONNECTED_ON_MS  = 50;
static const uint32_t VLED_NOT_CONNECTED_OFF_MS = 750;

// Bank color table (R, G, B) -- indexed by effective bank 0-3
static const uint8_t BANK_COLOR[][3] = {
  {0,   255, 0  },  // 0 FADER = Green
  {255, 255, 0  },  // 1 PAN   = Yellow
  {255, 0,   0  },  // 2 TRIM  = Red
  {0,   0,   255},  // 3 HOLD  = Blue
};

// ============================================================================

// ---------- BLE connection state machine ----------
enum AutoState {
  ST_BOOT=0, ST_SCAN, ST_CONNECT, ST_DISCOVER, ST_SUBSCRIBE,
  ST_SEND_HELLO, ST_SEND_FAMILY_A, ST_WAIT_READY, ST_READY, ST_FAIL
};

AutoState st = ST_BOOT;

BLEDevice zoomDev;
BLEService svc;
BLECharacteristic chTx, chRx, chFlow;

bool hasSvc=false, hasTx=false, hasRx=false, hasFlow=false;
bool readyFlag=false;

// State dump reception flags (set when A1 sub-cmd 0x00/0x01/0x02 parsed)
static bool gGotDumpTrim  = false;
static bool gGotDumpPan   = false;
static bool gGotDumpFader = false;

uint32_t tStateEnterMs=0;
uint32_t lastAnyNotifMs=0;
uint32_t last83Ms=0;

String cmdBuf;

static bool gSerialEnabled = true;

// ---------- Mode / bank control ----------
enum CtrlMode : uint8_t { MODE_FADER=0, MODE_PAN=1, MODE_TRIM=2 };
static CtrlMode gMode = MODE_FADER;

enum HoldState : uint8_t { HOLD_IDLE=0, HOLD_PRESSED, HOLD_ACTIVE };
static HoldState gHoldState = HOLD_IDLE;
static uint32_t  gModePressMs = 0;
static uint16_t  gHoldEntryBank3[8] = {0};

// ---------- AW9523 LED driver ----------
static Adafruit_AW9523 aw9523;
static bool gAw9523Ok = false;
static uint8_t gVisionLedBrightnessPercent = 100;   // HOLD POT 6
uint8_t gCatchLedGlobalBrightnessPercent = 100;      // HOLD POT 7
static uint8_t  gBlinkPhase[8] = {0};
static uint32_t gBlinkStartMs[8] = {0};

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

static bool writeBytes(const uint8_t* p, size_t n);

// ---------- Family A sender (non-blocking, 1 frame per tick) ----------
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

// ---------- Serial helpers ----------
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

// ---------- LED: built-in fallback when AW9523 absent ----------
static void ledUpdate() {
  if (gAw9523Ok) {
    digitalWrite(LED_PIN, LOW);
    return;
  }
  uint32_t elapsed = millis() % 120u;
  digitalWrite(LED_PIN, elapsed < 60 ? HIGH : LOW);
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
  lastAnyNotifMs=last83Ms=0;
  gGotDumpTrim=gGotDumpPan=gGotDumpFader=false;
  famAReset();
}

static void disconnectIfNeeded() {
  if (zoomDev && zoomDev.connected()) zoomDev.disconnect();
}

// ---------- Per-bank value memory (0=FADER, 1=PAN, 2=TRIM, 3=HOLD) ----------
// Initialized from Zoom state dump at connection; pots must catch before sending.
static uint16_t gBankVal[4][8] = {{0}};  // normalized 0..1023 per bank per channel
static bool     gCaught[4][8]  = {{0}};  // true = pot has picked up the stored value

// Notification callback — fires for EVERY notification, no loss from single-buffer
static void onTxNotification(BLEDevice, BLECharacteristic) {
  int n = chTx.valueLength();
  uint8_t buf[32];
  if (n > (int)sizeof(buf)) n = sizeof(buf);
  chTx.readValue(buf, n);

  lastAnyNotifMs = millis();
  if (n >= 2 && buf[0] == 0x83 && buf[1] == 0x0E) last83Ms = millis();

  // Flow control ACK: the Zoom expects 80 01 00 after each notification
  if (hasRx) {
    static const uint8_t ack[] = {0x80, 0x01, 0x00};
    chRx.writeValue(ack, sizeof(ack));
  }

  if (buf[0] == 0xA1 && n >= 17) {
    uint8_t subcmd = buf[2];

    if (subcmd == 0x00 && n >= 19 && !gGotDumpTrim) {
      for (uint8_t i = 0; i < 8; i++)
        gBankVal[MODE_TRIM][i] = mapTrimRawToNorm(buf[3 + i * 2]);
      gGotDumpTrim = true;
    }

    if (subcmd == 0x01 && n >= 19 && !gGotDumpPan) {
      for (uint8_t i = 0; i < 8; i++)
        gBankVal[MODE_PAN][i] = mapPanRawToNorm(buf[3 + i * 2], buf[4 + i * 2]);
      gGotDumpPan = true;
    }

    if (subcmd == 0x02 && !gGotDumpFader) {
      for (uint8_t i = 0; i < 8; i++)
        gBankVal[MODE_FADER][i] = mapFaderRawToNorm(buf[3 + i]);
      gBankVal[HOLDED_BANK][0] = mapOutputFaderRawToNorm(buf[11], 121);
      gBankVal[HOLDED_BANK][1] = mapOutputFaderRawToNorm(buf[13], 121);
      gBankVal[HOLDED_BANK][2] = mapOutputFaderRawToNorm(buf[15], 121);
      gGotDumpFader = true;
    }
  }
}

static void pollNotifs() {
  if (!zoomDev || !zoomDev.connected()) return;

  if (hasFlow && chFlow.valueUpdated()) {
    int n = chFlow.valueLength();
    uint8_t buf[32];
    if (n > (int)sizeof(buf)) n = sizeof(buf);
    chFlow.readValue(buf, n);
    lastAnyNotifMs = millis();
  }
}

static bool writeBytes(const uint8_t* p, size_t n) {
  if (!hasRx) return false;
  return chRx.writeValue(p, n);
}

// ---------- Protocol tx: send parameter values to Zoom ----------
static bool sendHello() {
  const uint8_t hello[] = {0xD1,0x12,0x32,0x2E,0x33,0x2E,0x31,
                           0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
                           0x00,0x00,0x00,0x00};
  return writeBytes(hello, sizeof(hello));
}

static bool sendFaderRaw(uint8_t ch, uint8_t val) {
  uint8_t p[5] = {0xA1,0x03,0x05,ch,val};
  return writeBytes(p, 5);
}

static const uint8_t FADER_OUT_ID_LR   = 0x08;
static const uint8_t FADER_OUT_ID_MAIN = 0x0A;
static const uint8_t FADER_OUT_ID_SUB  = 0x0C;
static const uint8_t FADER_OUT_VAL_MAX = 121;

static bool sendOutputFaderRaw(uint8_t outFaderId, uint8_t val) {
  uint8_t p[5] = {0xA1, 0x03, 0x05, outFaderId, val};
  return writeBytes(p, 5);
}

static bool sendTrimRaw(uint8_t ch, uint8_t trimVal) {
  uint8_t p[6] = {0xA1,0x04,0x03,ch,trimVal,0x00};
  return writeBytes(p, 6);
}

static bool sendPanRaw(uint8_t ch, uint8_t amount, uint8_t side) {
  uint8_t p[6] = {0xA1,0x04,0x04,ch,amount,side};
  return writeBytes(p, 6);
}

// ---------- Norm <-> Raw mapping (0..1023 <-> BLE values) ----------
static uint16_t clampU16(int v, int lo, int hi) {
  if (v < lo) return (uint16_t)lo;
  if (v > hi) return (uint16_t)hi;
  return (uint16_t)v;
}

static uint8_t mapNormToFaderRaw(uint16_t norm0_1023) {
  uint32_t x = norm0_1023;
  uint32_t y = (x * 125u + 511u) / 1023u;
  if (y > 125u) y = 125u;
  return (uint8_t)y;
}

static uint8_t mapNormToOutputFaderRaw(uint16_t norm0_1023) {
  uint32_t x = norm0_1023;
  uint32_t y = (x * (uint32_t)FADER_OUT_VAL_MAX + 511u) / 1023u;
  if (y > (uint32_t)FADER_OUT_VAL_MAX) y = FADER_OUT_VAL_MAX;
  return (uint8_t)y;
}

static uint8_t mapNormToTrimRaw(uint16_t norm0_1023) {
  uint32_t x = norm0_1023;
  uint32_t y = (x * 65u + 511u) / 1023u;
  if (y > 65u) y = 65u;
  return (uint8_t)y;
}

static void mapNormToPan(uint16_t norm0_1023, uint8_t& amount, uint8_t& side) {
  const uint16_t cLo = 512u - PAN_CENTER_DEAD; // 504
  const uint16_t cHi = 512u + PAN_CENTER_DEAD; // 520

  if (norm0_1023 >= cLo && norm0_1023 <= cHi) {
    side = 0;
    amount = PAN_LEFT_MAX;
  } else if (norm0_1023 < cLo) {
    side = 0;
    uint32_t x = norm0_1023;
    uint32_t y = (x * 126u + (cLo / 2)) / cLo;
    if (y > 126u) y = 126u;
    amount = (uint8_t)y;
  } else {
    side = 1;
    uint32_t range = 1023u - cHi;  // 503
    uint32_t x = (uint32_t)(norm0_1023 - cHi);  // 1..503
    uint32_t y = (x * (uint32_t)PAN_RIGHT_MAX + (range / 2)) / range;
    if (y > PAN_RIGHT_MAX) y = PAN_RIGHT_MAX;
    if (y == 0) y = 1;
    amount = (uint8_t)y;
  }
}

// ---------- Inverse mapping (Zoom state dump raw -> norm 0..1023) ----------

static uint16_t mapFaderRawToNorm(uint8_t raw) {
  if (raw >= 121) return 1023;
  return (uint16_t)((uint32_t)raw * 1023u + 60u) / 121u;
}

static uint16_t mapOutputFaderRawToNorm(uint8_t raw, uint8_t maxVal) {
  if (raw >= maxVal) return 1023;
  return (uint16_t)((uint32_t)raw * 1023u + (maxVal / 2)) / (uint32_t)maxVal;
}

static uint16_t mapTrimRawToNorm(uint8_t raw) {
  if (raw >= 65) return 1023;
  return (uint16_t)((uint32_t)raw * 1023u + 32u) / 65u;
}

static uint16_t mapPanRawToNorm(uint8_t amount, uint8_t side) {
  const uint16_t cLo = 512u - PAN_CENTER_DEAD; // 504
  const uint16_t cHi = 512u + PAN_CENTER_DEAD; // 520

  if (side == 0) {
    if (amount >= PAN_LEFT_MAX) return 512; // center
    // amount 0..126 -> norm 0..cLo-1
    return (uint16_t)((uint32_t)amount * cLo + 63u) / 126u;
  }
  // side == 1: amount 0..72 -> norm cHi+1..1023
  uint32_t range = 1023u - cHi; // 503
  uint32_t n = cHi + ((uint32_t)amount * range + 36u) / (uint32_t)PAN_RIGHT_MAX;
  if (n > 1023u) n = 1023u;
  if (n <= cHi) n = cHi + 1;
  return (uint16_t)n;
}

// ---------- Public normalized send API ----------
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

// HOLD bank output faders
static bool sendLrFaderNorm(uint16_t norm0_1023) {
  return sendOutputFaderRaw(FADER_OUT_ID_LR, mapNormToOutputFaderRaw(norm0_1023));
}
static bool sendMainFaderNorm(uint16_t norm0_1023) {
  return sendOutputFaderRaw(FADER_OUT_ID_MAIN, mapNormToOutputFaderRaw(norm0_1023));
}
static bool sendSubFaderNorm(uint16_t norm0_1023) {
  return sendOutputFaderRaw(FADER_OUT_ID_SUB, mapNormToOutputFaderRaw(norm0_1023));
}

// ---------- Serial commands ----------
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
        chTx.setEventHandler(BLEUpdated, onTxNotification);
      } else {
        hardFail("TX cannot subscribe");
        return;
      }

      if (hasFlow && chFlow.canSubscribe()) {
        chFlow.subscribe(); // FLOW not mandatory
      }

      enterState(ST_SEND_HELLO, "subscribed");
      break;
    }

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
        for (uint8_t b = 0; b < 4; b++)
          for (uint8_t c = 0; c < 8; c++)
            gCaught[b][c] = false;

        if (gSerialEnabled && gGotDumpFader && gGotDumpTrim && gGotDumpPan) {
          Serial.print("DUMP F:");
          for (uint8_t i = 0; i < 8; i++) { Serial.print(' '); Serial.print(gBankVal[MODE_FADER][i]); }
          Serial.print(" T:");
          for (uint8_t i = 0; i < 8; i++) { Serial.print(' '); Serial.print(gBankVal[MODE_TRIM][i]); }
          Serial.print(" P:");
          for (uint8_t i = 0; i < 8; i++) { Serial.print(' '); Serial.print(gBankVal[MODE_PAN][i]); }
          Serial.print(" LR:"); Serial.print(gBankVal[HOLDED_BANK][0]);
          Serial.print(" MAIN:"); Serial.print(gBankVal[HOLDED_BANK][1]);
          Serial.print(" SUB:"); Serial.println(gBankVal[HOLDED_BANK][2]);
        } else if (gSerialEnabled) {
          startupPrint("DUMP incomplete");
        }

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

  // While pressed and in PRESSED state, check for hold timeout
  if (gHoldState == HOLD_PRESSED) {
    bool stillPressed = MODE_BTN_PULLUP ? (gLastBtnRead == LOW) : (gLastBtnRead == HIGH);
    if (stillPressed && (now - gModePressMs >= HOLD_MODE_TIME_MS)) {
      gHoldState = HOLD_ACTIVE;
      for (uint8_t i = 0; i < 8; i++) {
        gHoldEntryBank3[i] = gBankVal[HOLDED_BANK][i];
        gBlinkPhase[i] = 0;
      }
      // So LR/MAIN/SUB are only sent when user actually moves fader 0/1/2 while HOLD is active
      gCaught[HOLDED_BANK][0] = false;
      gCaught[HOLDED_BANK][1] = false;
      gCaught[HOLDED_BANK][2] = false;
    }
  }

  if ((now - gLastBtnChangeMs) >= BTN_DEBOUNCE_MS) {
    if (gLastBtnStable != gLastBtnRead) {
      bool pressed = MODE_BTN_PULLUP ? (gLastBtnRead == LOW) : (gLastBtnRead == HIGH);
      gLastBtnStable = gLastBtnRead;

      if (pressed) {
        // Press: record time; do not cycle
        gModePressMs = now;
        gHoldState = HOLD_PRESSED;
      } else {
        // Release
        if (gHoldState == HOLD_ACTIVE) {
          for (uint8_t i = 0; i < 8; i++)
            gBlinkPhase[i] = 0;
          gHoldState = HOLD_IDLE;
        } else if (gHoldState == HOLD_PRESSED) {
          // Short press: cycle FADER -> PAN -> TRIM -> FADER
          if (gMode == MODE_FADER) gMode = MODE_PAN;
          else if (gMode == MODE_PAN) gMode = MODE_TRIM;
          else gMode = MODE_FADER;
          for (uint8_t i = 0; i < 8; i++) {
            gCaught[(uint8_t)gMode][i] = false;
            gBlinkPhase[i] = 0;
          }
          gHoldState = HOLD_IDLE;
        } else {
          gHoldState = HOLD_IDLE;
        }
      }
    }
  }
}

// Brightness persistence (own NVM, separate from HOLDED_BANK; survives power cycle)
#define BRIGHT_PREFS_MAGIC 0xAB
typedef struct { uint8_t magic; uint8_t visionPct; uint8_t catchPct; } BrightPrefs;
static NanoBLEFlashPrefs brightFlash;
static BrightPrefs brightPrefsBuf;

static void brightLoad() {
  int rc = brightFlash.readPrefs(&brightPrefsBuf, sizeof(brightPrefsBuf));
  if (rc == 0 && brightPrefsBuf.magic == BRIGHT_PREFS_MAGIC &&
      brightPrefsBuf.visionPct <= 100 && brightPrefsBuf.catchPct <= 100) {
    gVisionLedBrightnessPercent      = brightPrefsBuf.visionPct;
    gCatchLedGlobalBrightnessPercent = brightPrefsBuf.catchPct;
  }
  // else keep defaults (100)
}

static void brightSaveVision() {
  if (brightFlash.readPrefs(&brightPrefsBuf, sizeof(brightPrefsBuf)) != 0) {
    brightPrefsBuf.magic   = BRIGHT_PREFS_MAGIC;
    brightPrefsBuf.visionPct = 100;
    brightPrefsBuf.catchPct  = 100;
  }
  brightPrefsBuf.visionPct = gVisionLedBrightnessPercent;
  brightFlash.writePrefs(&brightPrefsBuf, sizeof(brightPrefsBuf));
}

static void brightSaveCatch() {
  if (brightFlash.readPrefs(&brightPrefsBuf, sizeof(brightPrefsBuf)) != 0) {
    brightPrefsBuf.magic   = BRIGHT_PREFS_MAGIC;
    brightPrefsBuf.visionPct = 100;
    brightPrefsBuf.catchPct  = 100;
  }
  brightPrefsBuf.catchPct = gCatchLedGlobalBrightnessPercent;
  brightFlash.writePrefs(&brightPrefsBuf, sizeof(brightPrefsBuf));
}

// ---------- Pot filter state ----------
static uint16_t gPotRawRing[8][POT_MEDIAN_WIN] = {{0}};
static uint8_t  gPotRawIdx[8] = {0};
static uint16_t gPotLastRaw[8] = {0};
static int32_t  gPotFiltFP[8] = {0};               // fixed-point EMA accumulator (<<POT_FRAC_BITS)
static uint16_t gPotFilt[8] = {0};                  // integer output (for LED distance display)
static bool     gPotInited[8] = {0};
static uint16_t gPotPrevFilt[8] = {0};

static uint32_t gPotLastSentMs[8] = {0};
static uint32_t gLastPotScanMs = 0;
static uint8_t  gPotScanCh = 0;
static uint32_t gLastGlobalBleWriteMs = 0;

static inline uint16_t u16absdiff(uint16_t a, uint16_t b) {
  return (a > b) ? (a - b) : (b - a);
}

static inline uint16_t median3_u16(uint16_t a, uint16_t b, uint16_t c) {
  if (a > b) { uint16_t t=a; a=b; b=t; }
  if (b > c) { uint16_t t=b; b=c; c=t; }
  if (a > b) { uint16_t t=a; a=b; b=t; }
  return b;
}

static inline uint16_t potHysteresis(CtrlMode m, uint16_t norm0_1023) {
  if (m == MODE_FADER) return POT_HYST_FADER;
  if (m == MODE_TRIM)  return POT_HYST_TRIM;
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

  // Determine effective bank/mode early (needed for per-type EMA shift)
  uint8_t bank = (gHoldState == HOLD_ACTIVE) ? HOLDED_BANK : (uint8_t)gMode;
  CtrlMode effMode = (bank == HOLDED_BANK) ? MODE_FADER : gMode;

  uint8_t emaShift = POT_EMA_SHIFT_FADER;
  if (effMode == MODE_PAN)  emaShift = POT_EMA_SHIFT_PAN;
  if (effMode == MODE_TRIM) emaShift = POT_EMA_SHIFT_TRIM;

  // EMA filter (fixed-point accumulator — no integer stall zone)
  if (!gPotInited[ch]) {
    gPotInited[ch] = true;
    gPotFiltFP[ch] = (int32_t)med << POT_FRAC_BITS;
    gPotFilt[ch] = med;
    gPotPrevFilt[ch] = med;
    if (!(gGotDumpFader && gGotDumpTrim && gGotDumpPan)) {
      gBankVal[MODE_FADER][ch] = 0;
      gBankVal[MODE_TRIM][ch]  = 0;
      gBankVal[MODE_PAN][ch]   = 512;
      if (ch <= 2)
        gBankVal[HOLDED_BANK][ch] = 818;
      for (uint8_t b = 0; b < 4; b++)
        gCaught[b][ch] = false;
    }
    gPotLastSentMs[ch] = now;
    return;
  }

  uint16_t prevF = gPotFilt[ch];
  int32_t med_fp = (int32_t)med << POT_FRAC_BITS;
  int32_t err_fp = med_fp - gPotFiltFP[ch];
  gPotFiltFP[ch] += (err_fp >> emaShift);
  uint16_t f = (uint16_t)(gPotFiltFP[ch] >> POT_FRAC_BITS);

  // Snap to endpoints when near 0 or 1023 (sync fixed-point accumulator)
  if (f <= POT_SNAP_MARGIN) {
    f = 0;
    gPotFiltFP[ch] = 0;
  } else if (f >= (1023 - POT_SNAP_MARGIN)) {
    f = 1023;
    gPotFiltFP[ch] = (int32_t)1023 << POT_FRAC_BITS;
  }

  gPotFilt[ch] = f;

  if (!gCaught[bank][ch]) {
    uint16_t target = gBankVal[bank][ch];
    uint16_t hyst = potHysteresis(effMode, target);

    bool withinHyst = (u16absdiff(f, target) <= hyst);
    bool crossed = ((prevF <= target && f >= target) ||
                    (prevF >= target && f <= target));

    if (withinHyst || crossed) {
      gCaught[bank][ch] = true;
      if (bank != HOLDED_BANK) {
        gBlinkPhase[ch] = 1;
        gBlinkStartMs[ch] = millis();
      }
      f = target;
      gPotFilt[ch] = f;
    } else {
      gPotPrevFilt[ch] = f;
      return; // not caught yet, skip send
    }
  }

  gPotPrevFilt[ch] = f;

  if (now - gPotLastSentMs[ch] < POT_SEND_MIN_GAP_MS) return;
  if (now - gLastGlobalBleWriteMs < BLE_GLOBAL_MIN_GAP_MS) return;

  uint16_t nextVal = f;
  if (nextVal == gBankVal[bank][ch]) return;

  // Raw-change filters: only send when the actual BLE value changes.
  // Fader/output fader: no remap (send function handles norm→raw conversion).
  // Trim/pan: remap to snap gBankVal to raw step boundary (consistent mappings).
  if (bank == HOLDED_BANK) {
    if (ch <= 2) {
      uint8_t newRaw = mapNormToOutputFaderRaw(nextVal);
      uint8_t curRaw = mapNormToOutputFaderRaw(gBankVal[bank][ch]);
      if (newRaw == curRaw) return;
    }
  } else if (effMode == MODE_FADER) {
    uint8_t newRaw = mapNormToFaderRaw(nextVal);
    uint8_t curRaw = mapNormToFaderRaw(gBankVal[bank][ch]);
    if (newRaw == curRaw) return;
  } else if (effMode == MODE_TRIM) {
    uint8_t newRaw = mapNormToTrimRaw(nextVal);
    uint8_t curRaw = mapNormToTrimRaw(gBankVal[bank][ch]);
    if (newRaw == curRaw) return;
    nextVal = mapTrimRawToNorm(newRaw);
  } else if (effMode == MODE_PAN) {
    uint8_t newAmt, newSide;
    mapNormToPan(nextVal, newAmt, newSide);
    uint8_t curAmt, curSide;
    mapNormToPan(gBankVal[bank][ch], curAmt, curSide);
    if (curAmt == newAmt && curSide == newSide) return;
    nextVal = mapPanRawToNorm(newAmt, newSide);
  }
  if (nextVal == gBankVal[bank][ch]) return;

  bool ok = false;
  if (bank == HOLDED_BANK) {
    if (ch == 0) {
      ok = sendLrFaderNorm(nextVal);
    } else if (ch == 1) {
      ok = sendMainFaderNorm(nextVal);
    } else if (ch == 2) {
      ok = sendSubFaderNorm(nextVal);
    } else if (ch <= 5) {
      ok = true;
    } else if (ch == 6) {
      gVisionLedBrightnessPercent = (uint8_t)((nextVal * 100u) / 1023u);
      brightSaveVision();
      ok = true;
    } else {
      gCatchLedGlobalBrightnessPercent = (uint8_t)((nextVal * 100u) / 1023u);
      brightSaveCatch();
      ok = true;
    }
  } else {
    switch (gMode) {
      case MODE_FADER: ok = sendFaderNorm(ch, nextVal); break;
      case MODE_PAN:   ok = sendPanNorm(ch, nextVal);   break;
      case MODE_TRIM:  ok = sendTrimNorm(ch, nextVal);  break;
    }
  }

  if (ok) {
    gBankVal[bank][ch] = nextVal;
    gPotLastSentMs[ch] = now;
    gLastGlobalBleWriteMs = now;
  }
}

// ---------- Catch LEDs (AW9523 pins 0-7) ----------
static void ledCatchUpdate() {
  if (!gAw9523Ok) return;
  uint8_t bank = (gHoldState == HOLD_ACTIVE) ? HOLDED_BANK : (uint8_t)gMode;
  uint32_t now = millis();

  for (uint8_t ch = 0; ch < 8; ch++) {
    if (gBlinkPhase[ch] > 0) {
      uint32_t elapsed = now - gBlinkStartMs[ch];
      uint8_t phase = (uint8_t)(elapsed / CATCH_LED_BLINK_HALF_MS);
      if (phase >= 6) {
        gBlinkPhase[ch] = 0;
        aw9523.analogWrite(ch, 0);
      } else {
        bool on = ((phase & 1) == 0);
        uint8_t val = on
          ? (uint8_t)((uint32_t)CATCH_LED_BLINK_PCT * gCatchLedGlobalBrightnessPercent * 255u / 10000u)
          : 0;
        aw9523.analogWrite(ch, val);
      }
      continue;
    }

    if (gCaught[bank][ch]) {
      aw9523.analogWrite(ch, 0);
      continue;
    }

    uint16_t target = gBankVal[bank][ch];
    uint16_t dist = u16absdiff(gPotFilt[ch], target);
    uint8_t dimPct = CATCH_LED_DIM_MAX
      - (uint8_t)((uint32_t)dist * (CATCH_LED_DIM_MAX - CATCH_LED_DIM_MIN) / 1023u);
    uint8_t val = (uint8_t)((uint32_t)dimPct * gCatchLedGlobalBrightnessPercent * 255u / 10000u);
    aw9523.analogWrite(ch, val);
  }
}

// ---------- VisionLed RGB (AW9523 pins 8,9,10) ----------
static void visionLedUpdate() {
  if (!gAw9523Ok) return;

  if (st == ST_READY) {
    uint8_t bank = (gHoldState == HOLD_ACTIVE) ? HOLDED_BANK : (uint8_t)gMode;
    uint8_t r = (uint8_t)((uint16_t)BANK_COLOR[bank][0] * gVisionLedBrightnessPercent / 100u);
    uint8_t g = (uint8_t)((uint16_t)BANK_COLOR[bank][1] * gVisionLedBrightnessPercent / 100u);
    uint8_t b = (uint8_t)((uint16_t)BANK_COLOR[bank][2] * gVisionLedBrightnessPercent / 100u);
    aw9523.analogWrite(VLED_PIN_R, r);
    aw9523.analogWrite(VLED_PIN_G, g);
    aw9523.analogWrite(VLED_PIN_B, b);
  } else {
    const uint32_t period = VLED_NOT_CONNECTED_ON_MS + VLED_NOT_CONNECTED_OFF_MS;
    uint32_t phase = millis() % period;
    uint8_t val = (phase < VLED_NOT_CONNECTED_ON_MS) ? 255 : 0;
    aw9523.analogWrite(VLED_PIN_R, val);
    aw9523.analogWrite(VLED_PIN_G, val);
    aw9523.analogWrite(VLED_PIN_B, val);
  }
}

void setup() {
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  pinMode(MODE_BTN_PIN, MODE_BTN_PULLUP ? INPUT_PULLUP : INPUT);

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


  // AW9523 init
  Wire1.begin();
  gAw9523Ok = aw9523.begin(AW9523_ADDR, &Wire1);
  if (gAw9523Ok) {
    for (uint8_t i = 0; i <= 10; i++)
      aw9523.pinMode(i, AW9523_LED_MODE);
    for (uint8_t i = 0; i <= 10; i++)
      aw9523.analogWrite(i, 0);
  } else {
    if (gSerialEnabled) startupPrint("AW9523 init FAILED");
  }
  brightLoad();

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
  autoStep();
  ledUpdate();

  modeButtonTick();
  potsTick();

  if (gAw9523Ok) {
    if (st == ST_READY) ledCatchUpdate();
    visionLedUpdate();
  }

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