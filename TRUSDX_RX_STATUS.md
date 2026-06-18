# truSDX RX + TX Support for Mini-FT8 — Status & Handoff

**Last updated:** 2026-06-08  (RX decode 2026-05-31; TX fixed + RF-verified 2026-06-07)
**Branch:** `trusdx-rx` (worktree: `C:\Data\Repos\.worktrees\Mini-FT8\trusdx-rx`)
**Fork:** `origin` = github.com/jeremywho/Mini-FT8  ·  `upstream` = wcheng95/Mini-FT8

> See **`CLAUDE.md`** for the durable build / flash / test how-to. This file = current status.

---

## Status: RX DECODE WORKING ✅ (2026-05-31)

Mini-FT8 on the M5 Cardputer ADV **decodes FT8 received from the (tr)uSDX over a single
USB-C cable.** First on-air decode achieved 2026-05-31. The full chain works on the
Cardputer: truSDX → USB host → CAT-stream parse → resample 7825→6000 Hz → FT8 monitor →
decode + waterfall.

---

## Update 2026-06-08 — TX WORKING ✅ (root-caused, fixed, RF-verified)

The truSDX **TX** path now works end to end (the branch is no longer RX-only).

**Root cause** of the long-standing "TX sticks / crashes the rig (OLED vertical lines)": the
firmware sent **`;US` before the TX audio**. `US` is the rig→host marker the rig *emits* to
announce its RX stream (see §1 of `TRUSDX_FACTS.md`) — it is NOT a host→rig command — so the
ATmega parsed an unterminated CAT token and crashed. **Fix** (`audio_trusdx_serial.cpp`
`tx_task`): drop `;US`; key on with `UA1;TX0;` (the proven SQ3SWF sequence); keep `;` escaping
and `;RX;`×3 key-off. Pacing was ruled out (a byte-rate sweep 11000–11542 B/s crashed
identically with `;US`, cleanly without it).

**Verified (2026-06-07):** Stage A — the firmware-exact TX audio of a real FT8 frame decodes
via Ft8DotNet (SNR +17). Stage B — the truSDX, keyed by the fixed sequence into a dummy load,
transmits a frame an RTL-SDR (direct-sampling 20 m) decodes off-air as `CQ K1ABC EL09`
(SNR −3.2); DT −0.09 confirms the rig clocks TX audio at exactly 11520 Hz. PC tools: encoder
`C:\Users\jerem\ft8enc` (text → FT8 tones), proofs `trusdx_tx_proof.py` (A) +
`trusdx_rf_live_proof.py` (B).

**Still untested:** a real two-way **QSO on a real antenna** (testing used a dummy load).
Mini-FT8 is a full auto-sequencing FT8 transceiver, so the next step is an on-air contact
(real 20 m antenna + open band). One cosmetic rig-firmware quirk remains — the TX wattmeter
lingers on the truSDX OLED after a CAT-controlled TX; see **`TRUSDX_DISPLAY_ISSUE.md`**
(not host-fixable).

---

## Update 2026-06-04 — driver migration · UI cleanup · decode-rate mystery SOLVED

**Sparse decodes = RF CONDITIONS, not a firmware bug — PROVEN.** The "≤1 decode per 15 s slot"
was chased to the ground: UTC-aligned slots captured off the truSDX on the PC
(`C:\Users\jerem\trusdx_capture.py`) and decoded with the gold-standard **Ft8DotNet** reference
(`C:\Data\Repos\ft8-may-2026`) got **0 real decodes** too (2 garbage CRC false-positives),
seeing the *same* weak Costas candidates (top sync score ~11) the Cardputer sees. A WSJT-X-parity
decoder pulling nothing off the same antenna ⇒ no decodable signal in the air (indoors / dead
band). The Cardputer decoder is **fine** — it performs comparably to the reference. Ruled out
(none changed the count): capture-window clipping, sample rate (measured 7814 ≈ assumed 7825),
per-block AGC, waterfall scaling, GPS/timing. For decodes you need: real antenna + open band +
correct frequency (14.074 etc.).

**USB-serial driver MIGRATED to the maintained Espressif stack** (`cdc_acm_host` +
`ch34x_vcp_open`; see `TRUSDX_FACTS.md` §2–3). The hand-rolled `ch340_usb_serial` USB-host state
machine was the root of the open / reconnect / stream-death failures. The rewrite backs the
*same* `Ch340UsbSerial` interface, so `audio_trusdx_serial.cpp` (CAT sequence, `UA1;` retry,
resample → decode) is unchanged; RX now arrives via a data callback into an SPSC ring.

**STATUS-screen diagnostics REMOVED** — Band/Tune lines are back to normal (the bring-up
`truSDX 7825->6000` / `RX r/a/o` overlay is gone). **S→3** now retunes the truSDX immediately via
`sync_radio_to_current_band()` (end-TX + ready guard); **S→2** is a dead-pipe-aware re-sync — a
2nd press on a *flowing* stream re-pushes the VFO instead of killing the connection, while a dead
`UA1` pipe still falls through to the recovering teardown. Non-standard per-block **AGC dropped**.

**Robustness issues found, DEFERRED** (NOT decode-count related): the ~1.66 s synchronous decode
blocks the audio task → the 8 KB CH340 RX ring overflows (~13 KB/slot dropped — a harmless
dead-zone); free internal heap ~8 KB (NimBLE + USB-host + 80 KB static waterfall; the StampS3 has
no PSRAM). A `droppedRxBytes()` counter was added. Fix later = decouple decode from capture + claw
back DRAM; memory-risky on ~8 KB free, so deferred per Codex review.

**Commits (atop `a856669`):** `e250f06` migration · `da36a3d` S-menu fixes · `44d7600` AGC removal.
Builds clean (`-Werror` + DRAM-budget gate), flashed, on-device smoke test passed (clean S-menu,
S→3 retunes).

---

## Original bring-up record (2026-05-31)

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

### On-screen diagnostics (REMOVED 2026-06-04 — see "Update 2026-06-04" above)
The bring-up overlay that replaced the STATUS Band/Tune lines while streaming
(`truSDX 7825->6000` resampler rates + `RX r<raw> a<audio> o<out>` counter, from
`audio_source_get_debug_line1/2`) has been removed — STATUS shows normal Band/Tune again. The
`audio_source_get_debug_line1/2` + `droppedRxBytes()` getters still exist (uncalled by the UI)
for ad-hoc stream-health checks.

---

## Known limitations / next steps

- **Flaky first connect** (open): the first `S`→`2` after a truSDX power-cycle sometimes comes
  up with `raw` stuck near 0 (~6 B/s); reconnecting once gets the full ~7800 B/s. Pre-existing,
  not caused by the recent fixes. Likely needs a settle delay or auto-retry in the connect.
- **Decode list rebuilds every 15 s slot** (`main.cpp`: `s_dec_count = 0` → fill →
  `ui_set_rx_list_static`), so a decode shows for one slot then clears if the next slot decodes
  nothing. With GPS time + a decent antenna this is far less noticeable. Optional polish: make
  the list *accumulate* across slots (WSJT-X style).
- **Decode rate is conditions-limited, not a bug** (see Update 2026-06-04): a WSJT-X-parity PC
  decoder pulls 0 real decodes off the same indoor antenna. Needs a real antenna feed + open band.
- **Robustness (deferred):** the synchronous decode blocks the audio task ~1.66 s/slot → the 8 KB
  RX ring overflows (~13 KB/slot dropped, harmless dead-zone); free heap ~8 KB. Decouple decode
  from capture + claw back DRAM later (memory-risky on ~8 KB free now).
- **TX WORKING** — root-caused, fixed, and RF-verified 2026-06-07 (the `;US` marker bug; see the TX update at the top). TX synth (`ft8_tx_synth`,
  `trusdx_serial_begin_ft8_tx`) drives it. Only a real two-way QSO on a real antenna is still untested.

---

## Code changes on this branch (vs merge base)

| File | Change |
|---|---|
| `main/audio_trusdx_serial.cpp` | connect sends `UA1;` (not `UA2;`); removed `RX;` from connect; pure-audio parse via `TrusdxParser.pure_audio` (was a cross-core volatile global); counter line `r/a/o` |
| `main/radio_trusdx.cpp` | `sync_frequency_mode` skips `RX;` while streaming (RX; suppresses the UA1 stream) |
| `main/gps.cpp` | GPS UART on **G13/G15** for the M5 LoRa+GPS Cap (default); `-DGPS_ON_PORTA` for a Grove PortA unit |
| `main/main.cpp` | S→3 retunes the truSDX immediately (`sync_radio_to_current_band`); S→2 dead-pipe-aware re-sync; STATUS bring-up diagnostics **removed** (Band/Tune restored); STATUS-exit re-sync only when the band changed |
| `components/ch340_usb_serial/` | **rewritten** to wrap the maintained `cdc_acm_host` + `ch34x_vcp_open` C API (was a hand-rolled USB-host state machine); `+ droppedRxBytes()` counter |
| `main/ft8_audio_pipeline.cpp` | removed non-standard per-block AGC (it distorted the monitor's overlapping FFT frames) |
| `main/audio_source.{cpp,h}`, `main/audio_trusdx_serial.{cpp,h}` | `droppedRxBytes()` plumbing for the RX ring |
| `.gitignore` | ignore `main/wifi_debug.*` (abandoned, kept on disk) and `build_*.log` |

Commits: `65cb66a` (working RX decode), `186a019` (GPS time + review fixes + stop mid-stream re-tune), `e250f06` (cdc_acm/ch34x migration), `da36a3d` (S-menu fixes + diagnostics removed), `44d7600` (AGC removal).

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
