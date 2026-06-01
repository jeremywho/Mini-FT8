# truSDX RX Support for Mini-FT8 — Status & Handoff

**Last updated:** 2026-05-31
**Branch:** `trusdx-rx` (worktree: `C:\Data\Repos\.worktrees\Mini-FT8\trusdx-rx`)
**Fork:** `origin` = github.com/jeremywho/Mini-FT8  ·  `upstream` = wcheng95/Mini-FT8

> See **`CLAUDE.md`** for the durable build / flash / test how-to. This file = current status.

---

## Status: RX DECODE WORKING ✅ (2026-05-31)

Mini-FT8 on the M5 Cardputer ADV **decodes FT8 received from the (tr)uSDX over a single
USB-C cable.** First on-air decode achieved 2026-05-31. The full chain works on the
Cardputer: truSDX → USB host → CAT-stream parse → resample 7825→6000 Hz → FT8 monitor →
decode + waterfall.

### The path that got us here
1. **PC validation first** — `Decode-Trusdx.ps1` proved the rig streams (`UA1;`) and decodes
   real FT8. This was the ground-truth reference for everything below.
2. **Merged `upstream/trusdx` RX** into the fork; built on ESP-IDF v5.4; flashed.
3. **`UA2;` crashes the truSDX R2.00x firmware** (its OLED fills with vertical-line garbage,
   needs a power-cycle). Switched the connect to **`UA1;`** — stable, and the same command the
   PC uses.
4. **The connect's `RX;` command suppressed the stream.** Removing it (order is now
   `ID`/`PS`/`FA`/`IF`/`MD2`/`UA1`) made `raw` jump from ~6 bytes to the full **~7800 B/s**.
5. **Pure-audio parser.** `UA1` is raw 8-bit audio with no `;CAT;` framing, so the parser must
   treat every byte as a sample (a byte = 59 / `';'` is audio, not a delimiter). Added
   `s_pure_audio_stream` flag; the old `UA2` multiplexed parser corrupted the stream.
6. **The `out=60` scare was a display red herring** — the counter line was clipped at the
   screen edge; `out` was always ~6000 (= raw × 6000/7825). Resampler was fine all along.
7. **Clock was the last decode-blocker.** FT8 needs UTC seconds within ~2 s. Hand-set the
   clock accurately → decodes appeared.

---

## What works

- **PC pipeline** (`C:\Users\jerem\trusdx-pc-test\Decode-Trusdx.ps1`): captures a 15 s slot via
  `UA1;`, resamples 7820→12000, decodes with **Ft8DotNet** (`C:\Data\Repos\ft8-may-2026`).
  Decoded real 20 m FT8 (`CQ KF8BRC EM98`, `V31ZA W6GOK`).
- **Cardputer**: connects over USB host, CAT works (band change retunes the rig), `UA1` streams
  ~7800 B/s, parser + resampler produce ~6000 samples/s of clean audio, waterfall renders,
  and FT8 **decodes**.
- **GPS auto-time WORKING** (M5 LoRa+GPS Cap, ATGM336H). GPS UART on **G13/G15** (set in
  `gps.cpp`; `-DGPS_ON_PORTA` for a PortA unit). Gets a fix → sets UTC automatically → decodes
  with no manual clock entry. Press `G` for the GPS screen. Coexists with the truSDX.

### On-screen diagnostics (currently in the STATUS screen — temporary)
While streaming, the STATUS screen (`S`) shows live instrumentation in place of the normal
Band/Tune lines:
- **`truSDX 7825->6000`** — resampler in→out rates (`audio_source_get_debug_line1`).
- **`RX r<raw> a<audio> o<out>`** — bytes received / bytes reaching the resampler / samples out,
  per second (`audio_source_get_debug_line2`). Healthy: `r`≈`a`≈7800, `o`≈6000 (the `o` digits
  may be clipped at the right screen edge — it really is ~6000).

---

## Known limitations / next steps

- **Flaky first connect** (open): the first `S`→`2` after a truSDX power-cycle sometimes comes
  up with `raw` stuck near 0 (~6 B/s); reconnecting once gets the full ~7800 B/s. Pre-existing,
  not caused by the recent fixes. Likely needs a settle delay or auto-retry in the connect.
- **Decode list rebuilds every 15 s slot** (`main.cpp`: `s_dec_count = 0` → fill →
  `ui_set_rx_list_static`), so a decode shows for one slot then clears if the next slot decodes
  nothing. With GPS time + a decent antenna this is far less noticeable. Optional polish: make
  the list *accumulate* across slots (WSJT-X style).
- **Diagnostic display still active (intentional, gated):** while the truSDX streams, the STATUS
  Band/Tune lines show the rate + `RX r/a/o` counter. Gated on `TRUSDX && streaming`, so QMX/KH1
  are unaffected. Kept as a stream-health indicator while the flaky-connect issue is open;
  restore Band/Tune later if you want a clean production UI.
- **TX is not done** — this branch focused on RX. TX synth exists (`ft8_tx_synth`,
  `trusdx_serial_begin_ft8_tx`) but is unverified on-air.

---

## Code changes on this branch (vs merge base)

| File | Change |
|---|---|
| `main/audio_trusdx_serial.cpp` | connect sends `UA1;` (not `UA2;`); removed `RX;` from connect; pure-audio parse via `TrusdxParser.pure_audio` (was a cross-core volatile global); counter line `r/a/o` |
| `main/radio_trusdx.cpp` | `sync_frequency_mode` skips `RX;` while streaming (RX; suppresses the UA1 stream) |
| `main/gps.cpp` | GPS UART on **G13/G15** for the M5 LoRa+GPS Cap (default); `-DGPS_ON_PORTA` for a Grove PortA unit |
| `main/main.cpp` | STATUS diagnostic gated on `TRUSDX && streaming`; STATUS-exit re-sync only when the band changed (clock edits no longer poke the rig) |
| `.gitignore` | ignore `main/wifi_debug.*` (abandoned, kept on disk) and `build_*.log` |

Commits: `65cb66a` (working RX decode), `186a019` (GPS time + review fixes + stop mid-stream re-tune).

---

## Hardware facts (full list in CLAUDE.md)

- truSDX firmware **R2.00x beta** (streaming + 115200); stable 2.00i has no streaming (CAT 38400).
- truSDX = CH340, micro-USB, **data** cable required; DTR wired to MCU RESET.
- **`UA2;` crashes the rig → use `UA1;`.** Connect must not send `RX;` before `UA1;`.
- Single USB cable; truSDX self-powered; Cardputer hosts on battery (no PORTA 5 V).
- truSDX built-in CW decoder shows garbage when fed FT8 — ignore its text line.

## Don't repeat
- **WiFi debug channel** (`main/wifi_debug.*`, gitignored, out of build) failed: broadcast
  didn't cross the PC Ethernet↔WiFi boundary, and the build crash-looped on boot (RAM). Use
  **on-screen counters** for visibility instead.

---

## File / branch map

| What | Where |
|---|---|
| Firmware branch / worktree | `trusdx-rx` @ `C:\Data\Repos\.worktrees\Mini-FT8\trusdx-rx` |
| truSDX RX driver | `main/audio_trusdx_serial.{cpp,h}` |
| truSDX CAT/control | `main/radio_trusdx.{cpp,h}` |
| CH340 USB-host driver | `components/ch340_usb_serial/` |
| Shared FT8 pipeline | `main/ft8_audio_pipeline.{cpp,h}` (6000 Hz) |
| UI / STATUS / connect | `main/main.cpp` |
| PC decode pipeline | `C:\Users\jerem\trusdx-pc-test\` |
| C# FT8 decoder | `C:\Data\Repos\ft8-may-2026` (Ft8DotNet, .NET 10) |
| ESP-IDF | `C:\esp\esp-idf` (v5.4) |
