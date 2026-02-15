# Plan: HOLD and LED MISSION

Single document, two main sections. Implement **HOLD** first (no hardware), then **LED MISSION** (AW9523, 8 catch LEDs). LED MISSION assumes the 4-bank + hold model is in place.

---

# HOLD

## 1. Four banks (not three)

- **Banks 0–2 (normal/idle)**: FADER, PAN, TRIM. Cycled **only** by a **short press** (MODE press + release before HoldModeTime). Same behaviour as today.
- **Bank 3 (HOLDED_BANK)**: Active **only** when in hold mode (MODE long-pressed and held). Same catch/process as the others; only the way we **access** it changes (via hold).

All four banks use the same process: catch value vs stored value, hysteresis, send when caught. Only which bank is “current” and what “send” does per channel differ.

## 2. Compile-time

- **HOLD_MODE_TIME_MS**: Button is considered in hold mode only after held this many ms (e.g. 500). Below that = short press.

## 3. Button behaviour

- **Short press** (press + release **before** HOLD_MODE_TIME_MS):  
  - **Only** action: cycle gMode FADER → PAN → TRIM → FADER.  
  - Clear `gCaught[gMode][i]` for the **new** gMode.  
  - **No** cycle in any other scenario (never cycle when leaving hold).

- **Long press** (held **≥** HOLD_MODE_TIME_MS):  
  - Enter hold: effective current bank = HOLDED_BANK (index 3).  
  - **No pot event**: release → quit hold silently, back to normal (same gMode), no cycle.  
  - **Pot event(s)** in hold: run catch/send for bank 3 per rules below; on release → quit hold, back to normal (same gMode), no cycle. Button still pressed → stay in hold for more events.

## 4. Data and code changes (HOLD)

- **Arrays**: `gBankVal[4][8]`, `gCaught[4][8]` (today’s `[3][8]` → `[4][8]`). Bank 3 = HOLDED_BANK.
- **Effective bank**: `uint8_t bank = (holdState == HOLD_ACTIVE) ? 3 : (uint8_t)gMode;` Use `bank` everywhere in pot/catch logic (potsTick, LED update).
- **modeButtonTick**:  
  - On **press** (debounced down): record press time; start hold timer; do **not** cycle.  
  - On **release**:  
    - If (releaseTime - pressTime) **<** HOLD_MODE_TIME_MS → **cycle** gMode (FADER→PAN→TRIM→FADER), clear gCaught[gMode][i].  
    - If (releaseTime - pressTime) **≥** HOLD_MODE_TIME_MS → quit hold (if we were in hold), **no** cycle; back to normal with same gMode.
- **potsTick**: Use `bank` as above. When `bank == 3`, apply HOLDED_BANK send rules (see below). Hysteresis for bank 3: e.g. use FADER hysteresis for ch 0–2 and 7; ch 3–6 can use same or no send.

## 5. HOLDED_BANK send semantics (channel rules)

- **POT 0–2**: Act as **FADER** (send BLE fader; type of BLE message may change later). Same catch process; on send use current fader BLE encoding.
- **POT 3–6**: **Do nothing** (no BLE, no global brightness). Keep place for future use; **read and store** pot state (gBankVal[3][ch], gCaught[3][ch]) so catch/LED and future features can use it.
- **POT 7**: **Change global brightness only** (no BLE). Update `gCatchLedGlobalBrightnessPercent` from pot value (0–1023 → 0–100); do not send BLE; do not update recorder.

## 6. HOLDED_BANK persistence (non-volatile memory)

- **What is stored**: Values of HOLDED_BANK (`gBankVal[3][0..7]`) are stored in non-volatile memory (e.g. EEPROM or Flash) and **recalled at startup** so they survive power cycles.
- **FADER / PAN / TRIM**: No change; stay as now (no persistence).
- **When to write NVM**: **Only when we quit hold mode**, and **only if** at least one value has changed during that hold session. On transition from hold to normal: compare current `gBankVal[3][ch]` to the values at hold entry (or last-saved); if any channel changed, write the full bank 3 to NVM; if no change, do not write.
- **Startup**: Read HOLDED_BANK from NVM into `gBankVal[3][0..7]`; set `gCaught[3][i]` as needed (e.g. all false so pots must catch again when next entering hold, or leave as stored if you persist catch state too; recommend “all false” on load so behaviour is predictable).

---

# LED MISSION

## 1. Goal

- 8 LEDs (one per channel) driven by an **AW9523** over I2C (Wire1).  
- **Catch feedback**: brightness = distance to catch value (near = brighter, far = dimmer); when value is caught → 3 ultra-fast blinks at max then OFF.  
- **Global brightness**: Ceiling for all LEDs; set by POT 7 when in HOLDED_BANK (0–100%). Dim/blink levels are a % of that ceiling.  
- **Implement after** HOLD is done (4 banks, hold mode, POT 7 = brightness in bank 3).

## 2. I2C and hardware

- **Bus**: Use **Wire1** (Nano 33 BLE: D20 SDA, D21 SCL) so pots stay on A0–A7.  
- **AW9523**: VCC 3.3 V, GND, SDA → D20, SCL → D21. Default address (e.g. 0x58).  
- **8 LEDs**: Current-sink wiring (anode to 3.3 V, cathode to AW9523 pin). One LED per channel 0–7. No series resistor in LED mode (AW9523 constant current). Set GCR for max current per pin (e.g. ~11 mA from your tests).  
- **Pin mapping**: Channel 0 → AW9523 pin 0, … channel 7 → AW9523 pin 7.

## 3. Compile-time config (LED)

- **HOLD_MODE_TIME_MS**: Already in HOLD section; reuse for button.  
- **CATCH_LED_GLOBAL_BRIGHTNESS_PERCENT**: Default at boot (e.g. 80) if no NVM; runtime value from POT 7 in HOLDED_BANK.  
- **CATCH_LED_DIM_MIN_PERCENT**, **CATCH_LED_DIM_MAX_PERCENT**, **CATCH_LED_BLINK_MAX_PERCENT**: e.g. 10, 80, 100 (percent of global brightness).  
- **CATCH_LED_CURVE_LINEAR** / **CATCH_LED_CURVE_SQUARED**: Distance-to-brightness curve.  
- **CATCH_LED_BLINK_HALF_MS**: e.g. 40 ms per half-blink; 3 blinks = 6 half-phases then OFF.

## 4. Software (LED)

- **Library**: Adafruit AW9523; `Wire1.begin()` then `aw9523.begin(&Wire1)`. Configure 8 pins in LED mode; set GCR.  
- **Effective bank**: Use same `bank` as HOLD (hold ? 3 : gMode).  
- **ledCatchUpdate()** (when st == ST_READY):  
  - For each channel ch: if in blink sequence, advance phase and set 0 or blink level; else if gCaught[bank][ch] (and not blinking) → 0; else distance-based dim (map distance to dim_min%–dim_max% of global, apply curve, write 0–255 to AW9523).  
  - Global brightness ceiling = `gCatchLedGlobalBrightnessPercent` (set by POT 7 in HOLDED_BANK).  
- **Catch transition**: When gCaught[bank][ch] goes true in potsTick(), start 3-blink for that channel.  
- **On bank change** (short-press cycle): Reset per-channel blink state so no orphaned blink.  
- **Integer math**: Scale brightness in 32-bit and divide once (e.g. global_pct * level_pct * 255 / 10000) to avoid truncation.  
- **AW9523 missing**: If `aw9523.begin(&Wire1)` fails, set a flag and skip ledCatchUpdate() so the rest of the firmware runs.

## 5. Hardware summary (LED MISSION)

| Connection   | From          | To              |
|-------------|---------------|-----------------|
| VCC         | 3.3 V         | AW9523 VCC      |
| GND         | GND           | AW9523 GND      |
| SDA         | Nano D20      | AW9523 SDA      |
| SCL         | Nano D21      | AW9523 SCL      |
| LEDs 0–7    | 3.3 V (anode) | Resistor optional; cathode to AW9523 pin 0–7 |

---

# Order of implementation

1. **HOLD**: 4 banks, HoldModeTime, short press = cycle only, long press = bank 3, HOLDED_BANK send rules (0–2 fader, 3–6 no BLE, 7 brightness), persistence on quit hold if changed.  
2. **LED MISSION**: Wire1, AW9523, 8 LEDs, ledCatchUpdate(), distance dim, 3-blink, global brightness from POT 7 in hold.
