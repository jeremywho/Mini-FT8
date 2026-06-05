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

## Build — toolchain & how it's deployed

**Yes, this is a standard ESP-IDF project** (CMake + `idf.py`). It is NOT Arduino/PlatformIO.

| Thing | Value |
|---|---|
| Build system | ESP-IDF (`idf.py` → CMake → Ninja). Top-level `CMakeLists.txt` does `project(mini_ft8)` |
| ESP-IDF version | **v5.4.x**, installed at `C:\esp\esp-idf` (the env has also shown 5.5.x; `sdkconfig`/`dependencies.lock` track whichever ran — see "IDF version churn" below) |
| Target chip | **esp32s3** (`CONFIG_IDF_TARGET="esp32s3"`) — the Cardputer ADV's StampS3 |
| Flash size | 8 MB (`CONFIG_ESPTOOLPY_FLASHSIZE="8MB"`) |
| Partition table | `partitions.csv`: nvs(24K)@0x9000, phy(4K)@0xf000, **factory app 0x5F0000@0x10000**, **spiffs 2M@0x600000** |
| Host OS | Windows; build from **PowerShell** |

### Build commands
```powershell
. C:\esp\esp-idf\export.ps1        # puts idf.py + xtensa toolchain on PATH (run once per shell)
idf.py -DENABLE_BLE=ON build       # from the trusdx-rx worktree root
```
`export.ps1` is the IDF environment script — it sets `IDF_PATH` and prepends the
`xtensa-esp32s3-elf` GCC toolchain + the IDF python venv. Without sourcing it, `idf.py`
isn't found. (Top-level CMake pulls IDF via `include($ENV{IDF_PATH}/tools/cmake/project.cmake)`.)

### Build outputs (in `build/`)
- `mini_ft8.bin` — the app image (flashed at 0x10000).
- `MiniFT8_Merged_Auto.bin` — a single merged image (bootloader+parts+app), auto-generated
  by a `POST_BUILD` step in the top-level `CMakeLists.txt` (`esptool merge_bin`) for releases.

### Feature flags (compile-time, default ON; override with `idf.py ... -D<FLAG>=OFF`)
Defined in `main/feature_flags.h` (guarded `#ifndef`, default `1`):
- **`ENABLE_BLE`** (default ON) — NimBLE controller + GATT UI. **Always build `-DENABLE_BLE=ON`**
  for the truSDX/normal firmware. BLE-off is a different RAM layout and exists only for the
  abandoned WiFi-debug experiment. A `POST_BUILD` DRAM budget check
  (`tools/check_dram_budget.py`, ~50 KB threshold) FAILS the build if static DIRAM is too low.
- **`ENABLE_FT4`** (default ON) — FT4 protocol + Mode menu item.
- **`GPS_ON_PORTA`** (default OFF) — see GPS wiring below; default targets the LoRa+GPS Cap (G13/G15).

### Gotchas
- If the build dir gets confused (after toggling `-D` flags, or a killed/half build):
  `idf.py fullclean` first, then build. Clean build ~5–10 min; incremental ~1–2 min.
- **Never run two `idf.py build` in the same directory at once** — Ninja collides and both
  fail (a failed `fullclean` build can leave no app binary; just rebuild).
- **IDF version churn:** building under IDF 5.5.x then 5.4.x (or vice-versa) rewrites
  `sdkconfig` + `dependencies.lock` (~500 lines of `CONFIG_SOC_*` diff). That churn is a
  byproduct of the IDF version, not real changes — revert those two files
  (`git checkout -- sdkconfig dependencies.lock`) before committing unless you intend the bump.

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

This sequence streams reliably and has decoded real off-air FT8. **Use it to sanity-check
conditions:** if Ft8DotNet pulls nothing off a captured slot, the Cardputer won't either —
sparse/zero decodes indoors are conditions, not a firmware fault (confirmed 2026-06-04 with
`C:\Users\jerem\trusdx_capture.py` → Ft8DotNet: 0 real decodes off the same antenna).

---

## Testing on the Cardputer

1. Flash (above) and **Reset** to boot.
2. **Power-cycle the truSDX** for a clean start; connect USB-C Cardputer ↔ truSDX (Cardputer
   on battery — it hosts the rig like it does the QMX).
3. STATUS screen (`S`) → press **`2`** to connect/sync.
4. **Watch the truSDX's own OLED**: it must stay normal. Vertical-line garbage = its firmware
   crashed → power-cycle it (see `UA2` gotcha below).
5. The STATUS screen shows normal **Band / Tune** lines. (The bring-up `truSDX 7825->6000` /
   `RX r<raw> a<audio> o<out>` diagnostic overlay was removed 2026-06-04 — there's no on-screen
   byte counter anymore; gauge stream health from the waterfall.)
6. The main RX/waterfall screen should show a **dark field with discrete vertical traces**
   (not a solid block). Decodes appear at the FT8 boundaries (:00 / :15 / :30 / :45) **when
   there's a decodable signal.** Indoors on a dead band you may see ≤1/slot or none — that's
   **conditions, not a fault** (a PC reference decoder pulls nothing off the same antenna; see
   `TRUSDX_RX_STATUS.md` "Update 2026-06-04").
7. **Clock:** FT8 needs UTC within ~2 s. Best source = **GPS** (auto-sets the clock).
   Press **`G`** for the GPS screen (sat count, fix, time, grid). Manual fallback on STATUS:
   `5` = Date (YYYY-MM-DD), `6` = Time (HH:MM:SS); digits in place, `,`/`/` move,
   **Enter commits**, backtick cancels (release the last digit before Enter).

### GPS module wiring (pins differ by module — set in `gps.cpp`)
- **Default build = M5 LoRa+GPS Cap** (Cardputer ADV, ATGM336H GPS): GPS UART on **G13/G15**
  (ESP RX = G15 ← module GPS-TX; ESP TX = G13 → module GPS-RX). These pins are free on the
  ADV (it uses an I2C keypad). Coexists with the truSDX (native USB, GPIO19/20). CONFIRMED
  WORKING: GPS fix → auto UTC → FT8 decodes, no manual clock.
- **`-DGPS_ON_PORTA` build = Grove PortA GPS unit** on G1/G2.
- Baud is auto-probed (9600/115200), so only the pins matter. If a known-good GPS shows
  0 sats, suspect RX/TX swapped → swap `kGpsTxPin`/`kGpsRxPin` in `gps.cpp`.

---

## Key files

| File | Role |
|---|---|
| `main/audio_trusdx_serial.cpp` | truSDX RX driver: connect sequence, CAT (write-only), byte→audio parser + resampler (7825→6000), `r/a/o` counters |
| `main/radio_trusdx.cpp` | truSDX CAT/control wrapper |
| `components/ch340_usb_serial/` | CH340 USB-host driver — wraps the maintained `cdc_acm_host` + `ch34x_vcp_open` C API (since the 2026-06 migration); exposes `droppedRxBytes()` |
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
- The parser runs in **pure-audio mode** (treat every byte as a sample). NOTE: `UA1` audio is
  actually lightly *framed* — `US`<span>`;`, samples = pcm + 128, a `;` (0x3B) escaped to 0x3C —
  but the spans are long, so the pure-audio parser only mis-ingests the one-time `UA1;US` prefix
  as a few noise samples → harmless for FT8. (Corrected understanding: see `TRUSDX_FACTS.md` §1;
  the old "UA1 = raw / UA2 = multiplexed-framed" note was **wrong**.) Don't run the `UA2`
  `;TOKEN;` parser on a `UA1` stream.
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
