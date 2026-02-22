# Plan: LED VISION V2

Single document for all LED behaviour: 8 catch LEDs (one per channel) and VisionLed (F10 RGB status). Both driven by one AW9523 on I2C. Replaces/consolidates the “LED MISSION” section of `PLAN_HOLD_AND_LED_MISSION.md` and adds VisionLed.

---

# Chapter 1 – 8 catch LEDs

## Purpose

- One LED per channel (0–7).
- **Feedback:** Brightness = distance to catch value (dim when far, brighter when near).
- **On catch:** 3 fast blinks at max then OFF.
- Global brightness ceiling set by POT 7 in HOLDED_BANK (persisted separately; see below).

## Hardware / connections

- **AW9523** on **Wire1** (Nano 33 BLE: D20 SDA, D21 SCL). VCC 3.3 V, GND. I2C address **0x58** (default).
- **Catch LEDs:** Channel ch → AW9523 pin ch (0–7). Anode to 3.3 V, cathode to AW9523 pin (current-sink). No series resistor in LED mode (AW9523 constant current). Set GCR for max current per pin (e.g. ~11 mA).

## Code (to accomplish goal)

- **Init order:** AW9523 is initialised **first**, before BLE and the rest. `Wire1.begin()`; `aw9523.begin(&Wire1)` (address 0x58); configure pins 0–7 in LED mode; set GCR. If I2C / AW9523 init fails: set flag, **blink built-in LED fast** to indicate I2C failure, **keep going** (do not stop firmware); skip `ledCatchUpdate()` and `visionLedUpdate()`.
- Compile-time: `CATCH_LED_*` (dim min/max %, blink half ms, curve). Brightness default = 100% (see persistence).
- `ledCatchUpdate()` when `st == ST_READY`: effective bank = hold ? 3 : gMode; per ch: if in 3-blink sequence advance phase, else if caught and not blinking → 0, else distance-based dim (0–255 to AW9523). Global ceiling = `gCatchLedGlobalBrightnessPercent`.
- On catch in `potsTick()`: start 3-blink for that channel. On bank change (short-press): reset blink state.
- If AW9523 init fails: flag and skip `ledCatchUpdate()` (and `visionLedUpdate()`).

---

# Chapter 2 – VisionLed

## Purpose

- One **F10 RGB** common-anode LED (4-wire, 10 mm) as main status indicator, replacing built-in LED behaviour.
- **Only two messages:**
  1. **Connected** — when READY: solid bank color (Green / Yellow / Red / Blue by mode).
  2. **Not connected** — when connection is not established or has been lost: white, **20 ms on, 750 ms off**, repeat.
- Message colour (Not connected) is **white at max intensity** (not dimmed). Built-in LED is off when VisionLed is in use; **fast blink** only when AW9523/I2C init fails.

## Hardware / connections

- Same AW9523, same I2C bus (address 0x58). VisionLed R, G, B → AW9523 pins **8, 9, 10** (R=8, G=9, B=10). Common anode to 3.3 V (or VCC); R,G,B cathodes to AW9523 pins. No series resistor if within AW9523 per-pin current (constant-current LED mode).
- Part: F10 RGB 4-wire, 10 mm, common anode.

## Bank color RGB table (full saturation, 0–255)

| Bank | Mode  | Color  | R   | G   | B   |
|------|-------|--------|-----|-----|-----|
| 0    | FADER | Green  | 0   | 255 | 0   |
| 1    | PAN   | Yellow | 255 | 255 | 0   |
| 2    | TRIM  | Red    | 255 | 0   | 0   |
| 3    | HOLD  | Blue   | 0   | 0   | 255 |

When READY, effective bank = hold ? 3 : gMode; apply the row above to VisionLed R,G,B, then scale by `gVisionLedBrightnessPercent`. White messages are not scaled.

## Code (to accomplish goal)

- In same init: configure AW9523 pins 8, 9, 10 as LED mode (with pins 0–7).
- **Brightness (separate from bank):** POT 6 → `gVisionLedBrightnessPercent` (0–100%), POT 7 → `gCatchLedGlobalBrightnessPercent` (0–100%). **Persistence:** These two values have **their own NVM storage** (not part of HOLDED_BANK). When updated (user moves POT 6 or 7 in HOLD), **store immediately** to NVM. **Recall at boot**; **default = 100%** (max brightness). White messages on VisionLed are not dimmed (always max).
- `visionLedUpdate()` called every loop:
  - **Connected (st == ST_READY):** solid bank color from RGB table, scaled by `gVisionLedBrightnessPercent`.
  - **Not connected (any other state):** white at max (R=G=B=255). Pattern: **20 ms on, 750 ms off**, repeat.
- When AW9523 OK: set LED_BUILTIN always LOW. When AW9523 init failed: fast blink built-in LED to indicate I2C fail; keep running.
- **Lost-connection consistency:** Disconnect and notification liveness timeout → ST_FAIL; same “Not connected” pattern (20 ms on, 750 ms off).

---

# VisionLed messages (summary)

| Message         | When                      | VisionLed behaviour                                                                 |
|-----------------|----------------------------|--------------------------------------------------------------------------------------|
| **Connected**   | st == ST_READY             | Solid bank color (Green / Yellow / Red / Blue), dimmed by POT 6 (VisionLed brightness). |
| **Not connected** | Not READY (never or lost) | White, max intensity (R=G=B=255). **20 ms on, 750 ms off**, repeat.                  |

Lost connection (disconnect event or notification liveness timeout) must transition to ST_FAIL so the same “Not connected” pattern is shown.

---

# Implementation notes

- **Init order:** AW9523 first (Wire1, then aw9523 at 0x58, configure pins 0–10 in LED mode). If init fails: set `aw9523Ok = false`, fast-blink LED_BUILTIN in loop, skip `ledCatchUpdate()` and `visionLedUpdate()`; do **not** halt. Then BLE and rest of setup.
- **Single AW9523 (0x58):** Pins 0–7 = catch LEDs; pins 8–10 = VisionLed R,G,B. One `ledCatchUpdate()` for 0–7; one `visionLedUpdate()` for 8–10.
- **Brightness persistence (separate from HOLDED_BANK):** `gVisionLedBrightnessPercent` and `gCatchLedGlobalBrightnessPercent` have **their own NVM storage**. When user moves POT 6 or 7 in HOLD, update the variable and **write to NVM immediately**. At boot, **load both** from NVM; **default = 100%** if no valid stored value.
- **Order in loop:** When AW9523 OK: LED_BUILTIN LOW; `ledCatchUpdate()` (when READY); `visionLedUpdate()` (always). When AW9523 fail: fast blink LED_BUILTIN only.

---

# HOLDED_BANK pot assignment (LED brightness)

- **POT 6:** VisionLed brightness — updates `gVisionLedBrightnessPercent` (0–100%). Used only for **Connected** (solid bank color). **Store to NVM immediately** when value changes.
- **POT 7:** Catch LEDs global brightness — updates `gCatchLedGlobalBrightnessPercent` (0–100%). **Store to NVM immediately** when value changes.
- Both recalled at boot from their own persistence; default = 100%.
