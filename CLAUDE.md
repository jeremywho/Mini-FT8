# CLAUDE.md — Mini-FT8 × (tr)uSDX  (`trusdx-rx` branch)

Project-specific guide for working in this repo. Read **`TRUSDX_RX_STATUS.md`** first for
current status and open problems — this file is the durable how-to (build / flash / test /
hardware gotchas).

---

## What this project is

Make **Mini-FT8** (M5 Cardputer ADV, ESP32-S3) **receive and decode FT8 from a (tr)uSDX**
QRP radio over a **single USB-C cable**, using PE1NNZ "CAT streaming" — CAT commands and
8-bit RX audio multiplexed on one USB-CDC serial link @ 115200 baud. **Decode (RX) first**,
transmit later.

This branch (`trusdx-rx`) = the fork's `design/trusdx-backend` (TX synth + newer base)
**+ merged `upstream/trusdx` RX**. Fork `origin` = github.com/jeremywho/Mini-FT8,
`upstream` = wcheng95/Mini-FT8.

---

## Build (ESP-IDF v5.4, installed at `C:\esp\esp-idf`)

```powershell
. C:\esp\esp-idf\export.ps1
idf.py -DENABLE_BLE=ON build
```

- **Always pass `-DENABLE_BLE=ON`.** BLE-off is a different RAM layout; a `POST_BUILD`
  DRAM budget check (`tools/check_dram_budget.py`, ~50 KB threshold) fails the build if
  static DIRAM falls too low. (BLE-off was used only for the abandoned WiFi-debug build.)
- If the build dir gets confused (after toggling `-D` flags, or a killed/half build):
  `idf.py fullclean` first, then build. A clean build is ~5–10 min; incremental is ~1–2 min.
- **Never run two `idf.py build` in the same directory at once** — ninja collides and both
  fail (and a failed `fullclean` build leaves no app binary).

---

## Flash — IMPORTANT, non-obvious on the ESP32-S3

Mini-FT8 is **USB-host firmware**: it does **not** expose a USB CDC serial port, so esptool
cannot auto-reset it. Use **download mode**:

1. Hold **BOOT/G0**, tap **Reset**, keep holding ~2 s, release → a COM port (COM8,
   `VID_303A`) appears and **stays**.
2. `idf.py -DENABLE_BLE=ON -p COM8 flash`
3. The post-flash auto-reset does **not** exit download mode on the S3 → press **plain
   Reset (no BOOT)** to boot the new firmware.

Port auto-detect (PowerShell):
```powershell
Get-PnpDevice -Class Ports -PresentOnly |
  Where-Object { $_.InstanceId -match 'VID_303A' -and $_.FriendlyName -match '\(COM\d+\)' }
```
(The truSDX is a different device: CH340, `VID_1A86&PID_7523`, on its own COM port when
plugged into the PC directly.)

---

## Testing the truSDX on the PC (ground-truth reference)

The PC pipeline is the **known-good reference** — use it to confirm the rig itself streams
and decodes, independent of the Cardputer.

```powershell
C:\Users\jerem\trusdx-pc-test\Decode-Trusdx.ps1                  # 20m (14.074 MHz)
C:\Users\jerem\trusdx-pc-test\Decode-Trusdx.ps1 -FreqHz 7074000  # 40m
```

What it does (this is the template the Cardputer connect must match):
- Opens COM10 @115200 with **`DtrEnable=$false`** (DTR is wired to the ATmega RESET — leaving
  it asserted reboots the rig).
- Waits ~4 s for the rig to boot, sends `;` (resync), `ID;`, `MD2;` (USB mode), `FA<freq>;`,
  then **`UA1;`** to start RX audio streaming.
- Captures 15 s of raw 8-bit audio (~7820 B/s), resamples 7820→12000 Hz (`raw_to_wav.py`),
  decodes with **Ft8DotNet** (`C:\Data\Repos\ft8-may-2026`, .NET 10).

This sequence streams reliably and has decoded real off-air FT8.

---

## Testing on the Cardputer

1. Flash (above) and **Reset** to boot.
2. **Power-cycle the truSDX** for a clean start; connect USB-C Cardputer ↔ truSDX (Cardputer
   on battery — it hosts the rig like it does the QMX).
3. STATUS screen (`S`) → press **`2`** to connect/sync.
4. **Watch the truSDX's own OLED**: it must stay normal. Vertical-line garbage = its firmware
   crashed → power-cycle it (see `UA2` gotcha below).
5. On the Cardputer STATUS screen, diagnostic readouts (added for bring-up):
   - **Band line** → `truSDX 7825->6000` = the resampler in→out rates (expected healthy values).
   - **Counter line** → `RX r<raw> a<audio> o<out>`, updated ~1/s. Toggle `S` off/on to
     force-refresh (the redraw is laggy). **Healthy: `raw`≈7800/s, `out`≈6000/s.**
6. The main RX/waterfall screen should show a **dark field with discrete vertical traces**
   (not a solid block). Decodes appear at the FT8 boundaries (:00 / :15 / :30 / :45).
7. **Clock:** FT8 needs UTC within ~2 s. Set manually on STATUS: `5` = Date (YYYY-MM-DD),
   `6` = Time (HH:MM:SS); type digits in place, `,`/`/` move, **Enter commits**, backtick
   cancels (release the last digit before Enter). Normal source = GPS on PORTA.

---

## Key files

| File | Role |
|---|---|
| `main/audio_trusdx_serial.cpp` | truSDX RX driver: connect sequence, CAT (write-only), byte→audio parser + resampler (7825→6000), `r/a/o` counters |
| `main/radio_trusdx.cpp` | truSDX CAT/control wrapper |
| `components/ch340_usb_serial/` | CH340 USB-host driver |
| `main/ft8_audio_pipeline.cpp` | shared FT8 monitor/decode pipeline (6000 Hz) |
| `main/main.cpp` | UI, `RadioType::TRUSDX`, STATUS screen (`draw_status_view`), connect (`begin_usb_host_mode`) |
| `main/audio_source.cpp` | source router (`AUDIO_SOURCE_TRUSDX_SERIAL`), debug-line getters |
| `TRUSDX_RX_STATUS.md` | living status + open problems |

---

## Hardware facts / gotchas (hard-won — do not relearn these the hard way)

- **truSDX firmware must be R2.00x beta** (CAT streaming + 115200). Stable 2.00i has **no**
  streaming and runs CAT at 38400.
- truSDX = **CH340**, **micro-USB**, needs a **data** cable (charge-only shows no COM port).
  CAT = Kenwood TS-480 subset.
- **DTR is wired to the ATmega RESET** — opening the port reboots the rig. Keep DTR
  deasserted; wait ~4 s after open before CAT.
- **`UA2;` CRASHES the truSDX R2.00x firmware** (its OLED fills with vertical-line garbage;
  needs a power-cycle). Use **`UA1;`** — raw 8-bit audio, no `;CAT;` framing, the same command
  the PC uses. It's stable.
- The connect must **not** send `RX;` before the `UA` command — `RX;` suppresses the stream.
  Working order: `ID` / `PS` / `FA` / `IF` / `MD2` / `UA1`.
- `send_cat` / `send_cat_locked` are **write-only** (they do not wait for an ACK).
- `UA1` is raw audio → the parser must run in **pure-audio mode** (every byte is a sample;
  a byte equal to 59 / `';'` is audio, not a delimiter). The `UA2` multiplexed `;TOKEN;`
  parser corrupts a `UA1` stream.
- Single USB cable only: truSDX is self-powered; the Cardputer hosts it on battery. **No
  PORTA 5 V** (PORTA 5 V is only for the bus-powered KH1 USB-C audio adapter).
- truSDX OLED is a normal **dual-color** panel (yellow top stripe, blue bottom). Its built-in
  **CW decoder shows garbage** ("E E E", random callsigns) when fed FT8 — ignore that line.

---

## Don't repeat

- **WiFi debug channel** (`main/wifi_debug.*`, gitignored, not in the build): failed twice —
  UDP broadcast didn't cross the PC's Ethernet↔WiFi boundary, and the build crash-looped on
  boot (RAM). For on-device visibility prefer **on-screen counters** (already wired into the
  STATUS screen), not WiFi.
