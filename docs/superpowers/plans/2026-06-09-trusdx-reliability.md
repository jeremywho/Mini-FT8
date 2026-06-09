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
  as B1.** [large/risky — design first] → **DESIGNED; deferred to a focused bench session. See "D1 — design assessment" below.**

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
- 2026-06-09: **B2 implemented** — `TRUSDX_UA1_RETRIES` 3→6 (longer cold-boot dead-pipe self-heal: ~10 s post-open window). Builds clean. **Pending bench verify** (power-cycle truSDX, first `S→2` should reach full ~7800 B/s without a manual reconnect).
- 2026-06-09: **E1 implemented** — RX list persists the last decodes for `DEC_KEEP_EMPTY_SLOTS` (2) empty slots before clearing (`publish_rx_list_persist`) instead of blanking on the next empty slot; `s_dec` stays current-slot so auto-seq/beacon are unaffected. Builds clean.
- 2026-06-09: **D1 investigated + designed; deferred to bench (NOT implemented).** Traced the full live path and the memory model (code-grounded). Conclusion: every real fix needs DRAM the StampS3 does not have (~8 KB free heap, no PSRAM). A blind refactor would risk regressing the verified B1/B2/E1, and the benefit is unmeasurable on a dead band. Documented the design + the first experiment to run on the bench. See "D1 — design assessment" below.

## D1 — design assessment (deferred to a focused bench session)

### The live decode path (verified in code, not comments)
1. `trusdx_serial_start` creates **one** task: `stream_task` ("trusdx_rx", core 1, prio 4, 8 KB stack) — `audio_trusdx_serial.cpp:750`.
2. `stream_task` calls `ft8_audio_pipeline_run(&cfg)` with `cfg.read = trusdx_read_ft8_samples` — `audio_trusdx_serial.cpp:541,547`.
3. `ft8_audio_pipeline_run` loops: `cfg->read(...)` to pull audio, `monitor_process` per 160 ms block, and at the slot boundary calls `decode_monitor_results(&mon, ...)` **inline** — `ft8_audio_pipeline.cpp:196`.
4. `trusdx_read_ft8_samples` → `transport_read_one` → `s_ch340.poll()` (moves USB-host bytes into the CH340 RX ring) **+** `s_ch340.readBytes()` (drains the ring) — `audio_trusdx_serial.cpp:304,314`. **This is the only thing that drains the ring.**

So a **single task** both drains the CH340 ring and runs the ~1.66 s decode. During the decode, `poll()` never runs → the ring backs up → `droppedRxBytes()` climbs (~13 KB/slot). That is the entire bug: the decode and the ring-drain are on the same timeline.

### Why it can't be fixed without more RAM (the wall)
The waterfall the decoder reads is a **static BSS singleton**, not a heap buffer:
`WF_STATIC_SIZE = 93*2*1*433 = 80,538 bytes`, `static WF_ELEM_T waterfall_static_buf[...]` — `components/ft8_lib/common/monitor.c:61-62`. `monitor_init` hands every `monitor_t` this **same** buffer (`monitor.c:126,143`). Plus static FFT arena 12 KB (`monitor.c:100`) and static window/last_frame (~960 floats each). Free heap at decode entry is logged on-device by `DECODE_HEAP ENTER` (`main.cpp:3581`); prior runs show ~8 KB free, no PSRAM.

The three textbook fixes each need DRAM that isn't there:
- **Separate decode task** (keep draining while decoding): the decoder reads the singleton waterfall; the next slot's `monitor_process` would corrupt the in-flight decode. A clean hand-off needs a **second** 80.5 KB waterfall — impossible (static singleton + no heap headroom).
- **Bigger RX ring** to bridge the 1.66 s gap (~13–19 KB of raw 8-bit @ 11520 Hz): needs ~16–24 KB ring vs ~8 KB free. No DRAM.
- **Chunked/yielding decode**: requires surgery into `ftx_decode` to yield mid-search, and even then, polling-without-a-buffer still drops unless one of the above frees RAM. High risk, deep change.

### Viable paths (all have real trade-offs — none is free or zero-risk)
- **(a) `time_osr` 2→1 — best value, try first.** Halves the static waterfall (~80.5 KB → ~40 KB) **and** roughly halves decode time (fewer time sub-blocks searched → shorter stall → fewer dropped bytes). `g_time_osr` is already a runtime variable. **Cost:** lower decode sensitivity (time-oversampling helps weak/timing-offset signals). Must be A/B'd on a live band, not committed blind.
- **(b) Conditionally `nimble` deinit while truSDX is connected** — frees tens of KB to fund (a)'s ring or a second small buffer. **Cost:** BLE dump/remote unavailable during truSDX sessions; deinit/reinit lifecycle is crash-prone. Feature change + risk.
- **(c) PSRAM hardware** (module with 2–8 MB PSRAM) — makes double-buffering trivial. **Cost:** hardware change.
- **(d) Chunked decode** — see above; deepest/riskiest.

### Recommendation
Run **(a)** on the bench as a measured experiment: flip `time_osr` 2→1, watch `trusdx_serial_dropped_rx_bytes()` and decode count before/after on a real signal. Keep it only if decodes hold up. This attacks both memory and decode-time with an existing knob and zero structural risk. Everything else waits on either a hardware decision (c) or a feature trade (b). **Not implemented now** because the gain is unverifiable without a live band, and a structural refactor here would jeopardize the verified B1/B2/E1 fixes that are about to flash.
