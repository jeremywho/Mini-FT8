# truSDX / Mini-FT8 — Reliability + Polish Work Plan

**Created:** 2026-06-09
**Branch:** `trusdx-rx` (worktree `C:\Data\Repos\.worktrees\Mini-FT8\trusdx-rx`), reset clean off `main` @`3d2a83e`.
**Context:** TX is fixed (`;US` crash) + RF-verified and merged to fork `main`. This plan
addresses the remaining reliability bugs, robustness, and polish surfaced 2026-06-08.
Language/platform: C++ on ESP-IDF v5.4 (FreeRTOS + LVGL 8.3), ESP32-S3.

Do NOT commit `sdkconfig` / `dependencies.lock` drift (local IDF 5.4.4 vs committed 5.5.1) —
add specific files when committing, not `-A`.

## Items

### A — Housekeeping
- [ ] **A1.** Drop the WIP diagnostics/IDF-drift commit; start clean off `main`.
- [ ] **A2.** Refresh stale `TRUSDX_CONNECT_ISSUE.md` (reconnect likely fixed by the cdc_acm/ch34x migration; pending soak).

### B — Reliability fixes (bench-testable now: truSDX + GPS connected)
- [ ] **B1. GPS-lock stream drop.** `gps_runtime_tick` (`main.cpp:2682`) does SPIFFS writes
  (`save_station_data`, `log_gps_grid_line`) on first GPS lock, **unguarded vs `g_streaming`**
  → the flash write stalls the USB-host task → the truSDX RX stream dies → "connect to truSDX".
  **Fix:** defer the persists (flag + flush when `!g_streaming`, e.g. between slots), keep the
  fast `settimeofday`/RTC sync inline. Repro before/after on the bench. [small/surgical]
- [ ] **B2. Flaky first connect** (~6 B/s until one reconnect after a truSDX power-cycle).
  Connect path in `audio_trusdx_serial.cpp` — tune the existing UA1/enum retry: longer settle
  + auto-retry on first connect. [small–medium]

### C — Verification
- [ ] **C1. Reconnect soak.** Repeated `S→2` teardown + reconnect, sustained-stream soak; then
  close or reopen `TRUSDX_CONNECT_ISSUE.md` based on results. [test-only]
- [ ] **C2. Real on-air QSO test.** Operational — needs a real 20 m antenna + open band. Use the
  Cardputer auto-seq flow (answer a CQ). [test-only, user-driven]

### D — Architectural / robustness
- [ ] **D1. Decouple decode from audio capture + reclaim DRAM.** The ~1.66 s synchronous FT8
  decode blocks the audio task → 8 KB CH340 RX ring overflows (~13 KB/slot dropped); free heap
  ~8 KB, no PSRAM. Move decode off the USB-host/audio timeline (separate task/priority or
  chunked) + trim the 80 KB static waterfall / NimBLE. **The deep fix for the same stall class
  as B1.** [large/risky — design first]

### E — Polish
- [ ] **E1. Decode list accumulates across slots** (WSJT-X style) instead of rebuilding each
  slot (`main.cpp`: stop resetting `s_dec_count` each slot; accumulate + age out). [small–medium]

### Not our code
- TX wattmeter display quirk — rig firmware (see `TRUSDX_DISPLAY_ISSUE.md`). Options: report to
  DL2MAN / try a newer truSDX beta firmware. Not host-fixable.

## Sequence
A → B1 → B2 → C1 → C2 → D1 → E1.
(B1 = best value/effort. B1 and D1 are the same root class — B1 is the targeted patch, D1 the
deep fix.)

## Progress log
- 2026-06-09: plan created; A1 + A2 done (branch reset clean off main @3d2a83e; connect-issue marked likely-resolved).
- 2026-06-09: **B1 implemented** — `gps_runtime_tick` defers `save_station_data()`/`log_gps_grid_line()` while `audio_source_is_streaming()`, flushes when idle; fast RTC/time sync stays inline. Builds clean. **Pending bench verify** (boot connected to truSDX, confirm GPS lock no longer drops the stream).
