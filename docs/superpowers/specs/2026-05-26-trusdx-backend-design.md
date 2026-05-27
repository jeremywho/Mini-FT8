# trusdx Backend for Mini-FT8 — Design

**Status:** Approved design, ready for implementation planning
**Date:** 2026-05-26
**Branch:** `design/trusdx-backend` (worktree under `C:\Data\Repos\.worktrees\Mini-FT8\trusdx-backend`)
**Upstream:** `wcheng95/Mini-FT8` @ commit `204f43c`
**Target hardware:** M5Stack Cardputer ADV (ESP32-S3) + (tr)uSDX with firmware ≥ 2.00t

## Summary

Add the (tr)uSDX as a third radio backend in Mini-FT8 alongside QMX and KH1. The cardputer connects to the trusdx over a single USB-C cable using PE1NNZ's "CAT streaming" protocol — CAT commands and audio bytes share one USB-CDC serial link at 115200 baud. The cardputer synthesizes FT8 audio on-device (a new capability for this project) and streams it to the trusdx during TX; the trusdx streams 8-bit RX audio back during RX.

## Goals

- Full FT8 QSO capability (RX + TX) between Cardputer ADV and (tr)uSDX
- Single USB-C cable; no analog audio path
- User-selectable from the existing radio menu (N → 3)
- Numeric correctness of FT8 audio synthesis verified before any hardware exists

## Non-goals

- Analog (3.5mm) audio path. Single-cable USB CDC only.
- Pre-PE1NNZ trusdx firmware. Requires ≥ 2.00t for `UA1;` CAT-streaming support.
- TX-only or RX-only sub-modes. Full QSO is the only supported configuration.
- Power delivery from cardputer to trusdx. Documented as a hardware concern; users supply via powered hub or external battery.
- Multi-band auto-switching beyond what existing band selection already provides.

## Background

### Mini-FT8 today

Mini-FT8 is a Cardputer-based FT8 station. The codebase has a clean radio-control abstraction:

```c
typedef struct {
    const char* name;
    bool (*ready)(void);
    esp_err_t (*on_audio_start)(void);
    esp_err_t (*sync_frequency_mode)(int freq_hz);
    esp_err_t (*begin_tx)(int freq_hz, int tx_base_hz);
    esp_err_t (*set_tone_hz)(float tone_hz);
    esp_err_t (*end_tx)(void);
    esp_err_t (*set_tune)(bool enable, int freq_hz, int tone_hz);
    esp_err_t (*set_time)(int hour, int minute, int second);
} radio_control_ops_t;
```

Two backends exist: `radio_control_qmx.cpp` (USB CDC CAT + USB UAC audio) and `radio_control_kh1_cat.cpp` (UART CAT + UAC adapter or direct mic). Both backends share a common property: **the radio generates the FT8 tones in hardware**, driven by `set_tone_hz` CAT commands (`TA` on QMX, `FO` on KH1). The cardputer never synthesizes audio.

### trusdx is fundamentally different

The (tr)uSDX exposes:
- USB-CDC virtual serial @ 115200 8N1, Kenwood TS-480 CAT subset
- 3.5mm audio jacks (separate mic-in and headphone-out)
- **No** USB Audio Class device
- **No** CAT command to generate FT8 tones in the radio

For digital modes, the trusdx relies on PE1NNZ's "CAT streaming" extension: the host streams 8-bit PCM audio over the same USB-CDC link, interleaved with CAT commands. The trusdx firmware modulates incoming audio bytes as SSB.

### Reference implementation

`olgierd/trusdx-audio` (Python) is the canonical reference for the CAT-streaming protocol on PC. Key parameters extracted:

- Serial: 115200 baud, 8N1
- TX audio: **11525 Hz, 8-bit unsigned PCM, mono**
- RX audio: **7820 Hz, 8-bit signed PCM, mono**
- Enter streaming: send `UA1;`
- Start TX: send `UA1;TX0;` then raw audio bytes
- Stop TX: send `;RX;` (×3 for reliability — leading `;` closes any in-flight audio bytes)
- PTT: VOX-driven (non-silence audio → TX; `;RX;` → RX)
- Asymmetric sample rates, asymmetric signedness

The protocol has a known weakness: ASCII `;` (0x3B) is the CAT terminator, and a TX audio sample of exactly 0x3B will be interpreted as the start of a CAT command. The reference implementation accepts this corruption risk; we will mitigate via byte-stuffing.

## Architecture

Three new modules + one ops-table signature change.

### New modules

| Module | Responsibility |
|---|---|
| `main/ft8_tx_synth.{h,cpp}` | Numeric: FT8 tone array → 11525 Hz u8 PCM, byte-stuffed |
| `main/radio_control_trusdx.cpp` | Implements `radio_control_ops_t` for trusdx CAT |
| `main/stream_trusdx_cdc.{h,cpp}` | PE1NNZ streaming protocol; multiplexes CAT + audio bytes |

### Ops-table change

`begin_tx` gains two params so trusdx can stream a pre-rendered audio sequence:

```c
esp_err_t (*begin_tx)(int freq_hz, int tx_base_hz,
                      const uint8_t* tones, int n_tones);
```

QMX and KH1 ignore the new params (`(void)tones; (void)n_tones;`). trusdx hands `tones` to `ft8_tx_synth_stream_init` and runs an audio task. `set_tone_hz` becomes a no-op for trusdx (the synth knows the full schedule); kept in the ops table so the main TX loop stays uniform.

### New top-level state

```c
enum class RadioType { QMX, KH1_USBC, KH1_MIC, TRUSDX };
enum radio_control_backend_t { ..., RADIO_CONTROL_TRUSDX };
enum audio_source_backend_t  { ..., AUDIO_SOURCE_TRUSDX_CDC };
```

`RadioType::TRUSDX` binds to `(AUDIO_SOURCE_TRUSDX_CDC, RADIO_CONTROL_TRUSDX)` in `main.cpp::radio_binding`.

### File inventory

**New files**
```
main/ft8_tx_synth.h
main/ft8_tx_synth.cpp
main/radio_control_trusdx.cpp
main/stream_trusdx_cdc.h
main/stream_trusdx_cdc.cpp
tools/ft8_synth_reference.py
host_mock/test_ft8_synth.cpp
```

**Touched (small edits)**
```
main/radio_control.h            — add RADIO_CONTROL_TRUSDX, dispatch
main/radio_control.cpp          — switch case for TRUSDX
main/radio_control_backend.h    — begin_tx signature + radio_control_trusdx_get_ops
main/radio_control_qmx.cpp      — adopt new begin_tx signature (params unused)
main/radio_control_kh1_cat.cpp  — adopt new begin_tx signature (params unused)
main/audio_source.h             — add AUDIO_SOURCE_TRUSDX_CDC
main/audio_source.cpp           — dispatch to stream_trusdx_cdc
main/stream_uac.h               — split cat_cdc_* into shared header
main/stream_uac.cpp             — refactor: extract cat_cdc layer
main/resample.h                 — add resample_7820_to_6k declaration
main/resample.cpp               — add resample_7820_to_6k impl
main/main.cpp                   — RadioType::TRUSDX, menu N→3, radio_binding,
                                  parse_radio_type, radio_type_to_string,
                                  sync_radio_to_current_band paths
main/CMakeLists.txt             — add new sources
README.md                       — ## trusdx Connections section
host_mock/Makefile              — host_synth_test target (Phase 1 only)
```

## Phased delivery

### Phase 1 — FT8 audio synthesis + golden-file test (hardware-free)

**Deliverable:** `ft8_tx_synth` module that takes a 79-tone FT8 array and produces byte-exact-correct 11525 Hz u8 PCM audio, verified against a Python reference implementation.

**Done when:** `make host_synth_test && ./host_synth_test` passes for all three canned fixtures (CQ, signal report, 73) with zero byte differences against `tools/ft8_synth_reference.py` output, with `apply_byte_stuffing = false`. Stuffing path additionally checked by `grep -c '\x3B'` returning 0 on the stuffed output.

> **Status: PHASE 1 COMPLETE.** Tagged `phase-1-done` at commit `f9599f9`. All 9 tests pass (3 golden, 3 stuffing, 3 stream-equivalence) byte-exact against Python reference (numpy float32 + platform-libm sinf). Implementation: `main/ft8_tx_synth.{h,cpp}`. Reference: `tools/ft8_synth_reference.py`. Tests: `host_mock/test_ft8_synth.cpp`.

**Public interface**

```c
// One-shot: render full FT8 message into caller-provided buffer
#define FT8_TX_SYNTH_SAMPLE_RATE 11525
#define FT8_TX_SYNTH_SYMBOLS     79
#define FT8_TX_SYNTH_SPS         1844    // round(11525 * 0.160)
#define FT8_TX_SYNTH_SAMPLES     (FT8_TX_SYNTH_SPS * FT8_TX_SYNTH_SYMBOLS)

// tones[i] ∈ {0..7} (FT8 8-FSK symbol indices, output of ft8_encode())
void ft8_tx_synth_render(
    const uint8_t tones[FT8_TX_SYNTH_SYMBOLS],
    float base_hz,                  // typically 1500 Hz
    uint8_t* out_pcm_u8,            // FT8_TX_SYNTH_SAMPLES bytes
    bool apply_byte_stuffing        // skip 0x3B values
);

// Streaming variant for memory-constrained paths
typedef struct ft8_tx_synth_stream_s ft8_tx_synth_stream_t;
void ft8_tx_synth_stream_init(ft8_tx_synth_stream_t* s,
                              const uint8_t tones[79], float base_hz,
                              bool stuff);
int  ft8_tx_synth_stream_pull(ft8_tx_synth_stream_t* s,
                              uint8_t* out, int max_bytes);
bool ft8_tx_synth_stream_done(const ft8_tx_synth_stream_t* s);
```

**Synthesis details**

- 11525 Hz, 8-bit unsigned PCM, midpoint = 128 (silence)
- Continuous-phase FSK; tone frequency `f = base_hz + tone_idx * 6.25`
- Phase carried across symbol boundaries to avoid spectral splatter
- Gaussian-shaped tone transitions over ~3 samples (standard for FT8 TX)
- Volume ~70% of full scale (peak 0xE0, trough 0x20). Headroom from both 0x00/0xFF edges.
- Sample math: 11525 × 0.160 = 1844.0 exact; no fractional-sample drift

**Byte-stuffing for the 0x3B issue**

`apply_byte_stuffing = true` (default): output values of 0x3B are substituted to 0x3C. ~0.4% of samples affected statistically; inaudible distortion. Required for safe streaming over PE1NNZ.

`apply_byte_stuffing = false`: raw output, used by golden-file test for byte-exact diffing against Python reference.

**Golden-file test**

- `tools/ft8_synth_reference.py` generates reference output for known tone arrays
- `host_mock/test_ft8_synth.cpp` compiles synth standalone (no FreeRTOS, no ESP) and compares output byte-for-byte
- Three canned test cases: CQ, signal report, 73. Tone arrays committed as fixtures (output of `ft8_encode`) so the test doesn't link the encoder.
- New host_mock Makefile target: `host_synth_test`. Builds standalone, runtime ~10ms.

**Out of scope for Phase 1**

No USB CDC, no protocol streaming, no radio backend integration, no menu changes. Pure buffer-in / buffer-out.

**Risks**

- Continuous-phase math is easy to get subtly wrong; golden-file test catches this exactly.
- Python reference implementation must itself be correct. Cross-checked against WSJT-X-decoded recordings where possible.

### Phase 2 — radio backend + protocol streamer + hardware bring-up

**Deliverable:** end-to-end trusdx support; user can select TRUSDX in the menu and complete a real QSO on hardware.

**Done when:** All seven bring-up checklist steps below pass on real hardware; at least one full FT8 QSO is logged with the trusdx as the radio.

**Gating:** Phase 2 begins only after Phase 1 is done (golden-file test green and committed). The `ft8_tx_synth` module is a build-time dependency of `stream_trusdx_cdc`.

**Component 1: `main/radio_control_trusdx.cpp` (~120 lines)**

Implements `radio_control_ops_t` using the shared CDC layer (see Component 4):

- `ready()` — CDC link open + at least one valid trusdx response observed (e.g. `ID;` → `ID020;`)
- `on_audio_start()` — send `UA1;` to enter streaming mode (one-time per session)
- `sync_frequency_mode(freq)` — `MD2;FA<11d>;` (TS-480 USB, 11-digit FA)
- `begin_tx(freq, base, tones, 79)` — delegate to `stream_trusdx_cdc_begin_tx(tones, base)`
- `set_tone_hz(_)` — no-op (returns ESP_OK)
- `end_tx()` — delegate to `stream_trusdx_cdc_end_tx()`
- `set_tune(enable, freq, tone)` — render single repeating tone via synth, stream via `stream_trusdx_cdc_begin_tx`; `end_tx` on disable
- `set_time(...)` — `nullptr` (TS-480 has no TM)

**Component 2: `main/stream_trusdx_cdc.{h,cpp}` (~400 lines)**

The protocol layer. Owns the audio task and the CDC byte multiplexer.

```c
bool stream_trusdx_cdc_start(void);
void stream_trusdx_cdc_stop(void);
esp_err_t stream_trusdx_cdc_begin_tx(const uint8_t tones[79], float base_hz);
esp_err_t stream_trusdx_cdc_end_tx(void);
bool stream_trusdx_cdc_get_latest_waterfall_row(uint8_t* out, int len);
const char* stream_trusdx_cdc_get_status_string(void);
```

Internal state machine: `IDLE → RX_STREAMING ⇄ TX_STREAMING`.

- **Two FreeRTOS tasks** (rx_task, tx_task), share CDC handle via mutex
- **RX task**: reads bytes from CDC into 16KB ring buffer; converts int8 → float; resamples 7820 → 6000 Hz; feeds `ft8_audio_pipeline_run`
- **TX task**: pulls bytes from `ft8_tx_synth_stream` in ~500-byte chunks (matches Python reference); writes to CDC; on synth `done`, sends `;RX;` ×3 with 100ms gaps; returns to RX_STREAMING
- **Backpressure**: if TX CDC write blocks > 10ms, log warn and continue (trusdx idle-fills better than aborting mid-message)
- **CDC disconnect mid-TX**: stop audio task; log warn; transition to IDLE; next user action retriggers via existing connect/sync flow

**Component 3: `resample_7820_to_6k`**

New helper in `resample.cpp`. Linear interpolation is fine for 8-bit input quality. Header declaration in `resample.h`.

**Component 4: `cat_cdc_*` refactor**

Today's `stream_uac.cpp` interleaves QMX-specific UAC handling with generic CDC. Phase 2 extracts the shared CDC layer so both QMX and trusdx can call `cat_cdc_send`. Concrete change:

- Move `cat_cdc_ready`, `cat_cdc_send` decls from `stream_uac.h` to a new shared header (or keep in `stream_uac.h` if minimal additional decoupling is needed)
- Ensure the underlying CDC class driver is started once and not tied to UAC enumeration
- Both backends reference the same CDC endpoint

**Component 5: Menu + plumbing**

- `RadioType::TRUSDX` added to enum, `parse_radio_type`, `radio_type_to_string`
- `radio_binding` entry: `{AUDIO_SOURCE_TRUSDX_CDC, RADIO_CONTROL_TRUSDX}`
- Menu N→3 display: `QMX / KH1-USBC / KH1-MIC / TRUSDX`
- `sync_radio_to_current_band` path: same shape as QMX (CDC-based), KH1-style explicit-connect not needed

**Component 6: Instrumentation (test substrate)**

Every state transition and CAT send gets `ESP_LOGI` with a stable prefix. Example:

```
RADIO_TRUSDX: sync MD2 FA=14074000
TRUSDX_CDC: state RX_STREAMING -> TX_STREAMING (tones[0..4]=0,3,1,4,7)
TRUSDX_CDC: sent UA1;TX0; (8 bytes)
TRUSDX_CDC: pump 500 audio bytes (synth 14580/145676)
TRUSDX_CDC: cdc write backpressure 12ms
TRUSDX_CDC: synth done, sending ;RX; x3
TRUSDX_CDC: state TX_STREAMING -> RX_STREAMING
```

Goal: a real-hardware misbehavior is diagnosable from logs alone.

**First-power-on checklist (hardware bring-up)**

Each step independently verifiable; failure at step N localizes the bug.

1. CDC enumerates; verify VID/PID, baud, line state
2. CAT `ID;` returns `ID020;` (trusdx responds at all)
3. `MD2;FA14074000;` — trusdx display shows the new frequency
4. `UA1;` accepted (document the response pattern observed)
5. RX-only decode on a band with known activity
6. Tune-mode single continuous tone — trusdx keys, RF present on dummy load
7. Full FT8 TX of a canned CQ; confirm via nearby SDR or another rig

## Cross-cutting concerns

### Error handling

- **CAT send failures**: log + return `ESP_FAIL` from ops; existing autoseq "CAT not ready" path handles cleanup
- **CDC disconnect mid-TX**: detect, stop audio task, log warn, transition IDLE
- **Audio task underrun**: log warn, pad with silence (0x80); trusdx interprets as VOX silence
- **Synth vs streamer rate mismatch**: synth is the master clock; streamer pulls on demand; cannot drift

### Menu / UI

- Only change: N→3 (radio selector) gains TRUSDX option. One enum value, one parser branch, one display string.
- No new menu pages. trusdx-specific tuning (baud override, stuffing on/off) hardcoded for now.

### Documentation

`README.md` gains a `## trusdx Connections` section matching the QMX/KH1 style:

```text
┌──────────────────┐                 ┌─────────────────────────────┐
│ trusdx           │                 │ Cardputer ADV               │
│                  │                 │ USB-C                       │
│ USB-C ───────────┼─────────────────┤ USB-C (host mode)           │
│                  │                 │                             │
└──────────────────┘                 └─────────────────────────────┘
```

Plus notes on:
- trusdx firmware version requirement (≥ 2.00t with PE1NNZ CAT streaming)
- Power: cardputer cannot supply trusdx TX current; use powered hub or separate power
- Mode selection: N→3 → TRUSDX

## Risks and open questions

| Risk | Mitigation |
|---|---|
| 0x3B byte collision in TX audio | Synth byte-stuffing on by default; verifies on hardware |
| ESP32-S3 USB host CDC bidirectional concurrency quirks | Spike during Phase 2; fall back to half-duplex if needed |
| Power: cardputer host mode cannot supply trusdx TX current | Document in README; user supplies hub/external |
| 8-bit signed RX dynamic range (~48 dB) vs QMX's 24-bit UAC | Accepted limitation; "good enough" target not "competitive" |
| trusdx firmware response timing assumptions (CAT vs audio bytes) | Verified at first power-on; instrumentation reveals reality |
| Python reference correctness | Cross-checked against WSJT-X-decoded recordings |

## Out of scope (deferred or rejected)

- Analog (3.5mm) audio path as a fallback
- fake_trusdx host mock + JSON test scenarios (cut during design — testing mental model of trusdx, not real trusdx)
- Configurable baud rate menu (115200 hardcoded; 38400 mode of trusdx not supported)
- Power-management features on the trusdx side
- Upstream PR (decision deferred; design supports either fork or upstream)

## References

- Mini-FT8 upstream: https://github.com/wcheng95/Mini-FT8
- (tr)uSDX manual: https://dl2man.de/4-trusdx-manual/
- (tr)uSDX details (CAT, audio bridge): https://dl2man.de/5-trusdx-details/
- olgierd/trusdx-audio (Python reference): https://github.com/olgierd/trusdx-audio
- ft8_lib: https://github.com/kgoba/ft8_lib
