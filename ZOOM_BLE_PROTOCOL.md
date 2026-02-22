# Zoom F8n Pro — BLE Protocol Overview

Full documentation of the BLE communication protocol between the Zoom F8n Pro audio recorder and an external controller (Arduino Nano 33 BLE).

---

## 1. BLE Service & Characteristics

| Role | UUID | Usage |
|------|------|-------|
| Service | `70bcfdf1-3600-185b-d562-b64c851dc87d` | Custom Zoom service |
| TX (notify) | `69abc935-4eb8-982e-6e55-b812ef754bbf` | Zoom → Controller (notifications) |
| RX (write) | `22eb6fbe-75e4-c334-f1ce-73b59c6847e0` | Controller → Zoom (write without response) |
| Flow ctrl | `064131ea-592c-3ad9-4f8f-71db1b192828` | Secondary notification channel |

---

## 2. Connection Sequence

```
SCAN for UUID_SERVICE_CUSTOM
  ↓
CONNECT → discoverAttributes()
  ↓
SUBSCRIBE to TX (notify) + register callback
SUBSCRIBE to FLOW (notify, optional)
  ↓
SEND hello: D1 12 32 2E 33 2E 31 00...
  ↓
SEND Family A (38 frames, 30ms spacing)
  ↓  ← Zoom sends state dump + 83 0E ready
READY
```

### Family A Frames

38 initialization frames sent sequentially on the RX characteristic.
Key frames: `B6 00`, `BA 00`, `D1...` (version), `D2 00`, `A7 00`, `91 00`, `93 00`, `8A-90 01 01`, `E0 02 xx xx`, `CA 00`.

The `A7 00` frame triggers the Zoom to begin sending the state dump.

---

## 3. Flow Control — ACK `80 01 00`

After every notification received from the Zoom, the controller must send:

```
80 01 00
```

This ACK is required for the Zoom to continue sending. Without it, the Zoom stops after the first notification of a burst. This is implemented as an automatic response in the `onTxNotification()` callback.

---

## 4. State Dump (Zoom → Controller at Connection)

After `A7 00` is sent, the Zoom responds with a burst of notifications containing its full internal state. The 27 controllable parameters are encoded in **3 specific A1 notifications**:

### Sub-cmd 0x00 — Trim (19 bytes)

```
A1 11 00 <tr0> 00 <tr1> 00 <tr2> 00 <tr3> 00 <tr4> 00 <tr5> 00 <tr6> 00 <tr7> 00
```

Trim values at odd byte positions (3, 5, 7, 9, 11, 13, 15, 17). Range 0..65.

### Sub-cmd 0x01 — Pan (19 bytes)

```
A1 11 01 <amt0> <side0> <amt1> <side1> ... <amt7> <side7>
```

Pan as (amount, side) pairs. side=0: left/center (amt 0=L100, 127=center). side=1: right (amt 0..72=R100).

### Sub-cmd 0x02 — Faders (17 bytes)

```
A1 0F 02 <f0> <f1> <f2> <f3> <f4> <f5> <f6> <f7> <LR> 00 <MAIN> 00 <SUB> 00
```

- Input faders ch 0..7: bytes 3-10, range 0..121
- Output LR: byte 11, range 0..109
- Output MAIN: byte 13, range 0..121
- Output SUB: byte 15, range 0..121

### Other notifications in the burst

| First byte | Description |
|------------|-------------|
| `80 01 00` | Keepalive / heartbeat |
| `83 0E ...` | Ready signal (16 bytes, periodic) |
| `84`, `85`, `86` | Status flags |
| `87 04 ...` | Meter / level data |
| `88`, `89` | Device info |
| `A5 11 ...` | Channel names ("1".."8", "L/R", "SUB", "MAIN") |
| `BB 12 ...` | Device name ("ZOOM F8n Pro") |
| `D3 12 ...` | Firmware version |
| `E6 00` | Periodic tick |

---

## 5. Sending Parameters (Controller → Zoom)

### Input Fader (ch 0..7)

```
A1 03 05 <ch> <val>         val: 0..125
```

### Output Fader (LR / MAIN / SUB)

```
A1 03 05 <ID> <val>         val: 0..121
  ID: 0x08=LR, 0x0A=MAIN, 0x0C=SUB
```

### Trim (ch 0..7)

```
A1 04 03 <ch> <val> 00      val: 0..65
```

### Pan (ch 0..7)

```
A1 04 04 <ch> <amount> <side>
  side=0: amount 0..127 (L100 to center)
  side=1: amount 0..72  (center to R100)
```

---

## 6. Ready Detection

The Zoom sends `83 0E 00 00 00 00 00 00 00 00 00 00 00 00 00 00` periodically once ready. The first `83 0E` after Family A completes signals that the controller can begin sending parameters.

---

## 7. Liveness / Disconnection

The Zoom sends periodic notifications (0x80, 0x83, 0xE6, 0x87). If no notification is received for 5 seconds (`NOTIF_LIVENESS_MS`), the controller assumes the link is dead and retries.

The `BLEDisconnected` event handler provides immediate detection for clean disconnections.

---

## 8. Implementation Notes

- **Callback vs polling**: ArduinoBLE's `valueUpdated()` only stores the last notification per characteristic. Using `setEventHandler(BLEUpdated, callback)` on the TX characteristic ensures every notification is processed, which is critical during the state dump burst.

- **Inverse mapping**: The state dump values use different ranges than the send values for input faders (121 received vs 125 sent). The `mapFaderRawToNorm()` / `mapTrimRawToNorm()` / `mapPanRawToNorm()` functions convert raw BLE values to the normalized 0..1023 internal representation.

- **Pot catch**: After receiving the state dump, all potentiometers are marked as "not caught" (`gCaught = false`). A pot must physically cross through the stored Zoom value before it is allowed to send, preventing jumps.

- **Fallback**: If the state dump is incomplete (dump fail), fallback values are: faders=0, trims=0, pans=512 (center), output faders=818 (~80% of 1023).

---

Source: reverse engineering from Apple PacketLogger btsnoop captures (iPad ↔ Zoom F8n Pro), validated against live Arduino serial logs.
Analysis scripts: `analyze_receive.py`, `analyze_a1_decode.py`, `analyze_trigger.py`.
