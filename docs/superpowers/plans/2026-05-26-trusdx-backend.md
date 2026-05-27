# trusdx Backend for Mini-FT8 — Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add full FT8 QSO support (RX + TX) for the (tr)uSDX QRP transceiver to Mini-FT8, using PE1NNZ's CAT-streaming protocol over a single USB-C cable.

**Architecture:** Two phases. Phase 1 builds a hardware-free FT8 audio synthesis module verified against a Python reference (golden-file test). Phase 2 adds the trusdx radio backend, USB-CDC protocol streamer, menu wiring, and hardware bring-up. Phase 2 is gated on Phase 1 being green.

**Tech Stack:** C++ (ESP-IDF, FreeRTOS, ESP32-S3); C for `ft8_lib`; Python 3 for the synth reference; g++ for `host_mock` host-side tests.

**Spec:** [`docs/superpowers/specs/2026-05-26-trusdx-backend-design.md`](../specs/2026-05-26-trusdx-backend-design.md)

**Worktree:** `C:\Data\Repos\.worktrees\Mini-FT8\trusdx-backend` on branch `design/trusdx-backend`.

---

## File Structure

**Phase 1 files (new):**
- `main/ft8_tx_synth.h` — public synth interface (one-shot + streaming variants)
- `main/ft8_tx_synth.cpp` — implementation: continuous-phase FSK, byte-stuffing
- `tools/ft8_synth_reference.py` — Python reference for golden-file comparison
- `host_mock/test_ft8_synth_fixtures.h` — canned 79-tone FT8 arrays
- `host_mock/test_ft8_synth.cpp` — golden-file test harness
- `host_mock/Makefile` — add `host_synth_test` target

**Phase 2 files (new):**
- `main/radio_control_trusdx.cpp` — radio_control_ops_t implementation
- `main/stream_trusdx_cdc.h` — PE1NNZ protocol streamer public API
- `main/stream_trusdx_cdc.cpp` — protocol streamer implementation

**Phase 2 files (modified):**
- `main/radio_control_backend.h` — `begin_tx` signature change, new ops getter
- `main/radio_control.h` / `.cpp` — `RADIO_CONTROL_TRUSDX` enum, dispatch
- `main/radio_control_qmx.cpp` — adopt new `begin_tx` signature (ignore new params)
- `main/radio_control_kh1_cat.cpp` — adopt new `begin_tx` signature (ignore new params)
- `main/audio_source.h` / `.cpp` — `AUDIO_SOURCE_TRUSDX_CDC` enum, dispatch
- `main/stream_uac.h` / `.cpp` — split `cat_cdc_*` from QMX-specific code
- `main/resample.h` / `.cpp` — add `resample_7820_to_6k`
- `main/main.cpp` — `RadioType::TRUSDX`, menu N→3, parse/serialize, `radio_binding`
- `main/CMakeLists.txt` — add new sources
- `README.md` — add `## trusdx Connections` section

---

## Repository Notes

- File line endings: project uses `core.autocrlf=true`. After writing any file with the `Write` tool, convert LF→CRLF via PowerShell:
  ```powershell
  $p="<file>"; $c=[IO.File]::ReadAllText($p); [IO.File]::WriteAllText($p, ($c -replace "`r`n","`n" -replace "`n","`r`n"))
  ```
- Working directory for git commands: `C:\Data\Repos\.worktrees\Mini-FT8\trusdx-backend`
- Build system: ESP-IDF `idf.py build` for firmware; `make` in `host_mock/` for host tests
- Existing host_mock pattern: g++ compiles `autoseq.cpp` with `-DHOST_MOCK` to stub FreeRTOS/ESP

---

# Phase 1 — FT8 audio synthesis (hardware-free)

**Phase 1 done when:** `make host_synth_test && ./host_synth_test` passes for all three canned fixtures (CQ, signal report, 73) with zero byte differences against `tools/ft8_synth_reference.py` output. Stuffed-output check shows zero 0x3B bytes.

---

### Task 1: Bootstrap the host-side synth test target

**Files:**
- Modify: `host_mock/Makefile`
- Create: `host_mock/test_ft8_synth.cpp` (empty stub)

- [ ] **Step 1: Create empty test stub**

Create `host_mock/test_ft8_synth.cpp`:
```cpp
#include <cstdio>
int main(int argc, char* argv[]) {
    printf("test_ft8_synth stub OK\n");
    return 0;
}
```

- [ ] **Step 2: Add `host_synth_test` target to Makefile**

Open `host_mock/Makefile` and append after the existing `host_test` rule:
```makefile
# Phase 1 standalone synth test (no FreeRTOS, no autoseq)
SYNTH_TARGET = host_synth_test
SYNTH_SRC = test_ft8_synth.cpp
SYNTH_OBJ = $(SYNTH_SRC:.cpp=.o)

$(SYNTH_TARGET): $(SYNTH_OBJ)
	$(CXX) $(CXXFLAGS) -o $@ $^

synth: $(SYNTH_TARGET)

clean_synth:
	rm -f $(SYNTH_OBJ) $(SYNTH_TARGET)

.PHONY: synth clean_synth
```

- [ ] **Step 3: Build and run the stub**

Run from `host_mock/`:
```bash
make host_synth_test && ./host_synth_test
```
Expected: prints `test_ft8_synth stub OK`, exit code 0.

- [ ] **Step 4: Commit**

```bash
git add host_mock/Makefile host_mock/test_ft8_synth.cpp
git commit -m "build(host_mock): add host_synth_test target stub"
```

---

### Task 2: Python reference implementation (skeleton)

**Files:**
- Create: `tools/ft8_synth_reference.py`

- [ ] **Step 1: Write the Python synth**

Create `tools/ft8_synth_reference.py`:
```python
#!/usr/bin/env python3
"""
FT8 audio synthesis reference implementation.

Generates 11525 Hz, 8-bit unsigned PCM audio from a 79-symbol FT8 tone array.
Used as the golden source for host_mock/test_ft8_synth.cpp.

Usage:
    ft8_synth_reference.py <tones_hex_79_bytes> <base_hz> <out_raw_pcm_path>

Where:
    tones_hex_79_bytes — 158 hex chars (79 bytes), each byte in 0..7
    base_hz            — audio center frequency, e.g. 1500
    out_raw_pcm_path   — raw u8 PCM written here, FT8_TX_SYNTH_SAMPLES bytes
"""
import math
import struct
import sys

SAMPLE_RATE = 11525
SYMBOLS = 79
SPS = 1844              # round(11525 * 0.160)
TOTAL_SAMPLES = SPS * SYMBOLS
TONE_SPACING = 6.25
PEAK_EXCURSION = 89     # ~70% of full 8-bit range from midpoint (128 +/- 89)
MID = 128


def synth_ft8(tones, base_hz):
    """Continuous-phase FSK, hard tone edges (no smoothing)."""
    assert len(tones) == SYMBOLS
    out = bytearray(TOTAL_SAMPLES)
    phase = 0.0
    idx = 0
    for sym in range(SYMBOLS):
        f = base_hz + tones[sym] * TONE_SPACING
        dphi = 2.0 * math.pi * f / SAMPLE_RATE
        for _ in range(SPS):
            s = math.sin(phase)
            b = MID + int(round(s * PEAK_EXCURSION))
            # Defensive clamp (round can hit 218 from 217.5)
            if b < 0: b = 0
            if b > 255: b = 255
            out[idx] = b
            idx += 1
            phase += dphi
            if phase > 2.0 * math.pi:
                phase -= 2.0 * math.pi
    return bytes(out)


def main():
    if len(sys.argv) != 4:
        print(__doc__, file=sys.stderr)
        sys.exit(2)
    tones_hex, base_hz, out_path = sys.argv[1], float(sys.argv[2]), sys.argv[3]
    tones = bytes.fromhex(tones_hex)
    if len(tones) != SYMBOLS:
        sys.exit(f"tones must be {SYMBOLS} bytes, got {len(tones)}")
    if any(t > 7 for t in tones):
        sys.exit("tone values must be in 0..7")
    audio = synth_ft8(tones, base_hz)
    with open(out_path, 'wb') as f:
        f.write(audio)


if __name__ == '__main__':
    main()
```

- [ ] **Step 2: Verify it runs**

Run (from repo root):
```bash
python3 tools/ft8_synth_reference.py 0000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000 1500 /tmp/cq_ref.bin
ls -l /tmp/cq_ref.bin
```
Expected: file size = 145676 bytes (1844 × 79). 158-char tones string = 79 hex bytes of all-zeros.

- [ ] **Step 3: Commit**

```bash
git add tools/ft8_synth_reference.py
git commit -m "tools: FT8 synth Python reference for golden-file tests"
```

---

### Task 3: Canned FT8 tone-array fixtures

**Files:**
- Create: `host_mock/test_ft8_synth_fixtures.h`

We commit canned tone arrays so the test doesn't need to link `ft8_lib`. These are the outputs of `ft8_encode()` for three messages. To regenerate later: see "Regenerating fixtures" comment in the file.

- [ ] **Step 1: Write the fixtures header**

Create `host_mock/test_ft8_synth_fixtures.h`:
```cpp
#pragma once
// Canned outputs of ft8_encode() for three FT8 messages.
// Each array is 79 bytes, values in 0..7 (8-FSK symbol indices).
//
// Regenerating fixtures (when you need new ones):
//   1. Build ft8_lib's gen_ft8 tool (components/ft8_lib/demo)
//   2. ./gen_ft8 "<message>" <wav_path>
//   3. The 79 symbols are printed in the gen_ft8 output; copy them here.
//
// fixture_cq:     "CQ AG6AQ CM87"
// fixture_report: "AG6AQ K1ABC -10"
// fixture_73:     "AG6AQ K1ABC 73"

#include <cstdint>

// Layout per FT8 spec (constants.h): S(7) D(29) S(7) D(29) S(7) = 79 symbols.
// Sync block (S): 3,1,4,0,6,5,2  (Costas pattern)
// Data block (D): 29 placeholder data tones, each in 0..7
//
// PLACEHOLDER DATA — replace with real ft8_encode() output before merging.
// Until then, the golden-file test compares synth(this_fixture) vs
// ft8_synth_reference.py(this_fixture); both read the same tones, so the
// test validates the synthesizer regardless of whether these are real FT8
// messages or arbitrary tone arrays in 0..7.
static const uint8_t fixture_cq[79] = {
    /* sync 1 (7)  */ 3,1,4,0,6,5,2,
    /* data 1 (29) */ 0,1,2,3,4,5,6,7, 7,6,5,4,3,2,1,0, 1,3,5,7,2,4,6,0, 1,2,3,4,5,
    /* sync 2 (7)  */ 3,1,4,0,6,5,2,
    /* data 2 (29) */ 0,7,1,6,2,5,3,4, 4,3,5,2,6,1,7,0, 0,1,2,3,4,5,6,7, 7,6,5,4,3,
    /* sync 3 (7)  */ 3,1,4,0,6,5,2
};
static const uint8_t fixture_report[79] = {
    /* sync 1 (7)  */ 3,1,4,0,6,5,2,
    /* data 1 (29) */ 1,0,3,2,5,4,7,6, 0,7,1,6,2,5,3,4, 7,5,3,1,6,4,2,0, 1,2,3,4,5,
    /* sync 2 (7)  */ 3,1,4,0,6,5,2,
    /* data 2 (29) */ 6,1,7,0,2,3,4,5, 5,4,3,2,1,0,7,6, 7,0,1,2,3,4,5,6, 7,7,7,7,7,
    /* sync 3 (7)  */ 3,1,4,0,6,5,2
};
static const uint8_t fixture_73[79] = {
    /* sync 1 (7)  */ 3,1,4,0,6,5,2,
    /* data 1 (29) */ 4,5,6,7,0,1,2,3, 3,2,1,0,7,6,5,4, 0,2,4,6,1,3,5,7, 0,1,2,3,4,
    /* sync 2 (7)  */ 3,1,4,0,6,5,2,
    /* data 2 (29) */ 2,7,4,1,6,3,0,5, 5,0,3,6,1,4,7,2, 2,3,4,5,0,1,6,7, 0,0,0,1,1,
    /* sync 3 (7)  */ 3,1,4,0,6,5,2
};
```

NOTE: First 7 bytes of each fixture are the FT8 Costas pattern `3,1,4,0,6,5,2`. Real `ft8_encode()` output will share these. The remaining bytes are placeholders that need to be replaced with real `ft8_encode()` output before final merge — but the synth test is valid regardless because synth(fixture) and python(fixture) consume the same array.

- [ ] **Step 2: Compile-check**

Update `host_mock/test_ft8_synth.cpp`:
```cpp
#include <cstdio>
#include "test_ft8_synth_fixtures.h"
int main(int argc, char* argv[]) {
    printf("fixture_cq[0]=%d fixture_73[78]=%d\n",
           (int)fixture_cq[0], (int)fixture_73[78]);
    return 0;
}
```

Run:
```bash
cd host_mock && make host_synth_test && ./host_synth_test
```
Expected: prints `fixture_cq[0]=3 fixture_73[78]=4`, exit 0.

- [ ] **Step 3: Commit**

```bash
git add host_mock/test_ft8_synth_fixtures.h host_mock/test_ft8_synth.cpp
git commit -m "test: add canned FT8 tone-array fixtures for synth golden tests"
```

---

### Task 4: ft8_tx_synth.h skeleton

**Files:**
- Create: `main/ft8_tx_synth.h`

- [ ] **Step 1: Write the public header**

Create `main/ft8_tx_synth.h`:
```c
#pragma once
// FT8 audio synthesis on the cardputer.
//
// Renders a 79-symbol FT8 tone array to 11525 Hz, 8-bit unsigned PCM audio,
// suitable for streaming to the (tr)uSDX over PE1NNZ's CAT-streaming protocol.
//
// Two variants:
//   - One-shot: ft8_tx_synth_render() — full message into caller buffer
//   - Streaming: ft8_tx_synth_stream_*() — pull bytes on demand
//
// Both consume tones[0..78] in 0..7 (FT8 8-FSK indices from ft8_encode()).

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define FT8_TX_SYNTH_SAMPLE_RATE 11525
#define FT8_TX_SYNTH_SYMBOLS     79
#define FT8_TX_SYNTH_SPS         1844  // round(11525 * 0.160) — exact
#define FT8_TX_SYNTH_SAMPLES     (FT8_TX_SYNTH_SPS * FT8_TX_SYNTH_SYMBOLS)

// One-shot render.
//   tones                 — 79 bytes, each in 0..7
//   base_hz               — audio center frequency (typ. 1500)
//   out_pcm_u8            — caller buffer of FT8_TX_SYNTH_SAMPLES bytes
//   apply_byte_stuffing   — if true, samples == 0x3B are substituted to 0x3C
//                           (avoids PE1NNZ CAT-streaming protocol collision)
void ft8_tx_synth_render(const uint8_t* tones, float base_hz,
                         uint8_t* out_pcm_u8, bool apply_byte_stuffing);

// Streaming variant. State machine across multiple ft8_tx_synth_stream_pull
// calls; no internal buffering of the full message. Suitable for feeding
// a CDC streaming task with bounded RAM.
typedef struct {
    const uint8_t* tones;
    float          base_hz;
    bool           stuff;
    int            sym_idx;       // current symbol 0..79 (79 = done)
    int            sample_in_sym; // current sample within symbol 0..SPS
    float          phase;         // continuous-phase FSK phase
} ft8_tx_synth_stream_t;

void ft8_tx_synth_stream_init(ft8_tx_synth_stream_t* s,
                              const uint8_t* tones, float base_hz, bool stuff);

// Pulls up to max_bytes audio samples into out. Returns count written.
// Returns 0 when stream is exhausted (see ft8_tx_synth_stream_done).
int ft8_tx_synth_stream_pull(ft8_tx_synth_stream_t* s,
                             uint8_t* out, int max_bytes);

bool ft8_tx_synth_stream_done(const ft8_tx_synth_stream_t* s);

#ifdef __cplusplus
}
#endif
```

- [ ] **Step 2: Compile-check (from test file)**

Update `host_mock/test_ft8_synth.cpp`:
```cpp
#include <cstdio>
#include "test_ft8_synth_fixtures.h"
#include "ft8_tx_synth.h"
int main(int argc, char* argv[]) {
    printf("samples per message = %d\n", FT8_TX_SYNTH_SAMPLES);
    return 0;
}
```

Update `host_mock/Makefile` SYNTH section to include `../main` in CXXFLAGS:
```makefile
SYNTH_CXXFLAGS = $(CXXFLAGS) -I../main
$(SYNTH_TARGET): $(SYNTH_OBJ)
	$(CXX) $(SYNTH_CXXFLAGS) -o $@ $^

%_synth.o: %.cpp
	$(CXX) $(SYNTH_CXXFLAGS) -c $< -o $@
```

(Adjust as needed so the test pulls in `../main/ft8_tx_synth.h`.)

Run:
```bash
cd host_mock && make clean_synth && make host_synth_test && ./host_synth_test
```
Expected: prints `samples per message = 145676`, exit 0.

- [ ] **Step 3: Commit**

```bash
git add main/ft8_tx_synth.h host_mock/Makefile host_mock/test_ft8_synth.cpp
git commit -m "synth: ft8_tx_synth public interface (no impl yet)"
```

---

### Task 5: First failing golden test (CQ fixture)

**Files:**
- Modify: `host_mock/test_ft8_synth.cpp`

- [ ] **Step 1: Write the failing test**

Update `host_mock/test_ft8_synth.cpp`:
```cpp
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <vector>
#include "test_ft8_synth_fixtures.h"
#include "ft8_tx_synth.h"

static std::vector<uint8_t> run_python_reference(const uint8_t* tones, float base_hz) {
    // Write tones as hex
    char hex[2 * FT8_TX_SYNTH_SYMBOLS + 1] = {0};
    for (int i = 0; i < FT8_TX_SYNTH_SYMBOLS; ++i) {
        snprintf(hex + i * 2, 3, "%02x", tones[i]);
    }
    char cmd[1024];
    snprintf(cmd, sizeof(cmd),
        "python3 ../tools/ft8_synth_reference.py %s %g /tmp/_synth_ref.bin",
        hex, base_hz);
    if (system(cmd) != 0) {
        fprintf(stderr, "python reference failed: %s\n", cmd);
        std::exit(2);
    }
    FILE* f = fopen("/tmp/_synth_ref.bin", "rb");
    if (!f) { perror("ref"); std::exit(2); }
    std::vector<uint8_t> v(FT8_TX_SYNTH_SAMPLES);
    size_t n = fread(v.data(), 1, FT8_TX_SYNTH_SAMPLES, f);
    fclose(f);
    if (n != FT8_TX_SYNTH_SAMPLES) {
        fprintf(stderr, "ref read short: %zu\n", n);
        std::exit(2);
    }
    return v;
}

static int compare_byte_exact(const char* name, const std::vector<uint8_t>& a,
                              const std::vector<uint8_t>& b) {
    if (a.size() != b.size()) {
        printf("[%s] FAIL: size %zu vs %zu\n", name, a.size(), b.size());
        return 1;
    }
    int diffs = 0;
    int first_diff = -1;
    for (size_t i = 0; i < a.size(); ++i) {
        if (a[i] != b[i]) {
            if (first_diff < 0) first_diff = (int)i;
            ++diffs;
        }
    }
    if (diffs == 0) {
        printf("[%s] PASS: %zu bytes match\n", name, a.size());
        return 0;
    }
    printf("[%s] FAIL: %d byte diffs, first at index %d (got 0x%02X, expected 0x%02X)\n",
           name, diffs, first_diff, a[first_diff], b[first_diff]);
    return 1;
}

static int test_fixture(const char* name, const uint8_t* tones) {
    std::vector<uint8_t> got(FT8_TX_SYNTH_SAMPLES);
    ft8_tx_synth_render(tones, 1500.0f, got.data(), false);
    std::vector<uint8_t> ref = run_python_reference(tones, 1500.0f);
    return compare_byte_exact(name, got, ref);
}

int main(int argc, char* argv[]) {
    int fails = 0;
    fails += test_fixture("cq", fixture_cq);
    if (fails == 0) printf("ALL OK\n");
    return fails ? 1 : 0;
}
```

- [ ] **Step 2: Update Makefile to link the synth source**

Edit `host_mock/Makefile` SYNTH section:
```makefile
SYNTH_SRC = test_ft8_synth.cpp ../main/ft8_tx_synth.cpp
SYNTH_OBJ = $(SYNTH_SRC:.cpp=.o)
```

- [ ] **Step 3: Run — expect link failure (no impl yet)**

```bash
cd host_mock && make clean_synth && make host_synth_test
```
Expected: link error — `undefined reference to ft8_tx_synth_render`. This is the failing-test step in TDD: the test cannot even build until we add the implementation.

- [ ] **Step 4: Add empty impl file to make it link (but fail the assertion)**

Create `main/ft8_tx_synth.cpp`:
```cpp
#include "ft8_tx_synth.h"
#include <cstring>

extern "C" void ft8_tx_synth_render(const uint8_t* tones, float base_hz,
                                    uint8_t* out_pcm_u8, bool stuff) {
    (void)tones; (void)base_hz; (void)stuff;
    // Emit silence (midpoint) so the test runs and reports byte diffs.
    std::memset(out_pcm_u8, 128, FT8_TX_SYNTH_SAMPLES);
}

extern "C" void ft8_tx_synth_stream_init(ft8_tx_synth_stream_t* s,
        const uint8_t* tones, float base_hz, bool stuff) {
    s->tones = tones; s->base_hz = base_hz; s->stuff = stuff;
    s->sym_idx = 0; s->sample_in_sym = 0; s->phase = 0.0f;
}

extern "C" int ft8_tx_synth_stream_pull(ft8_tx_synth_stream_t* s,
        uint8_t* out, int max_bytes) {
    (void)s; (void)out; (void)max_bytes;
    return 0;
}

extern "C" bool ft8_tx_synth_stream_done(const ft8_tx_synth_stream_t* s) {
    return s->sym_idx >= FT8_TX_SYNTH_SYMBOLS;
}
```

Run:
```bash
cd host_mock && make clean_synth && make host_synth_test && ./host_synth_test
```
Expected: `[cq] FAIL: N byte diffs, first at index 0 (got 0x80, expected 0x80)` — actually the first non-zero ref sample. The test failing here means the harness works.

- [ ] **Step 5: Commit (failing test + stub)**

```bash
git add main/ft8_tx_synth.cpp host_mock/test_ft8_synth.cpp host_mock/Makefile
git commit -m "test(synth): failing golden test for CQ fixture (impl stubbed)"
```

---

### Task 6: Implement the synthesizer (passes CQ test)

**Files:**
- Modify: `main/ft8_tx_synth.cpp`

- [ ] **Step 1: Implement `ft8_tx_synth_render`**

Replace `main/ft8_tx_synth.cpp`:
```cpp
#include "ft8_tx_synth.h"
#include <cmath>
#include <cstring>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

static const float TONE_SPACING = 6.25f;
static const float PEAK_EXCURSION = 89.0f;  // ~70% of 8-bit range
static const uint8_t MID = 128;

static inline uint8_t sample_to_u8(float s) {
    int v = (int)lrintf(s * PEAK_EXCURSION) + MID;
    if (v < 0) v = 0;
    if (v > 255) v = 255;
    return (uint8_t)v;
}

extern "C" void ft8_tx_synth_render(const uint8_t* tones, float base_hz,
                                    uint8_t* out, bool stuff) {
    float phase = 0.0f;
    int idx = 0;
    const float fs = (float)FT8_TX_SYNTH_SAMPLE_RATE;
    for (int sym = 0; sym < FT8_TX_SYNTH_SYMBOLS; ++sym) {
        float f = base_hz + (float)tones[sym] * TONE_SPACING;
        float dphi = 2.0f * (float)M_PI * f / fs;
        for (int i = 0; i < FT8_TX_SYNTH_SPS; ++i) {
            uint8_t b = sample_to_u8(sinf(phase));
            if (stuff && b == 0x3B) b = 0x3C;
            out[idx++] = b;
            phase += dphi;
            if (phase > 2.0f * (float)M_PI) phase -= 2.0f * (float)M_PI;
        }
    }
}

extern "C" void ft8_tx_synth_stream_init(ft8_tx_synth_stream_t* s,
        const uint8_t* tones, float base_hz, bool stuff) {
    s->tones = tones; s->base_hz = base_hz; s->stuff = stuff;
    s->sym_idx = 0; s->sample_in_sym = 0; s->phase = 0.0f;
}

extern "C" int ft8_tx_synth_stream_pull(ft8_tx_synth_stream_t* s,
        uint8_t* out, int max_bytes) {
    int written = 0;
    const float fs = (float)FT8_TX_SYNTH_SAMPLE_RATE;
    while (written < max_bytes && s->sym_idx < FT8_TX_SYNTH_SYMBOLS) {
        float f = s->base_hz + (float)s->tones[s->sym_idx] * TONE_SPACING;
        float dphi = 2.0f * (float)M_PI * f / fs;
        uint8_t b = sample_to_u8(sinf(s->phase));
        if (s->stuff && b == 0x3B) b = 0x3C;
        out[written++] = b;
        s->phase += dphi;
        if (s->phase > 2.0f * (float)M_PI) s->phase -= 2.0f * (float)M_PI;
        ++s->sample_in_sym;
        if (s->sample_in_sym >= FT8_TX_SYNTH_SPS) {
            s->sample_in_sym = 0;
            ++s->sym_idx;
        }
    }
    return written;
}

extern "C" bool ft8_tx_synth_stream_done(const ft8_tx_synth_stream_t* s) {
    return s->sym_idx >= FT8_TX_SYNTH_SYMBOLS;
}
```

- [ ] **Step 2: Run the test**

```bash
cd host_mock && make clean_synth && make host_synth_test && ./host_synth_test
```
Expected: `[cq] PASS: 145676 bytes match\nALL OK`, exit 0.

- [ ] **Step 3: If the test fails by 1–2 bytes due to float-precision differences**

`lrintf` vs Python's `round()` can disagree on midway cases. Fix by changing `sample_to_u8` to use `int v = (int)lrintf(...)` consistently with Python's banker's rounding, OR by clamping the comparison tolerance to 1 LSB. Prefer the strict equality fix:

```cpp
// Use round-half-away-from-zero to match Python's int(round(...))
static inline uint8_t sample_to_u8(float s) {
    float scaled = s * PEAK_EXCURSION;
    int v = (scaled >= 0.0f) ? (int)(scaled + 0.5f) : (int)(scaled - 0.5f);
    v += MID;
    if (v < 0) v = 0;
    if (v > 255) v = 255;
    return (uint8_t)v;
}
```

Update Python reference to match if needed (Python uses banker's rounding by default with `round()`). Simplest: change Python ref to `int(scaled + 0.5)` for positive / `int(scaled - 0.5)` for negative.

Re-run, verify PASS.

- [ ] **Step 4: Commit**

```bash
git add main/ft8_tx_synth.cpp tools/ft8_synth_reference.py
git commit -m "synth: implement continuous-phase FSK render, CQ fixture passes"
```

---

### Task 7: Add tests for the other two fixtures

**Files:**
- Modify: `host_mock/test_ft8_synth.cpp`

- [ ] **Step 1: Add report and 73 tests**

In `test_ft8_synth.cpp`, expand `main()`:
```cpp
int main(int argc, char* argv[]) {
    int fails = 0;
    fails += test_fixture("cq", fixture_cq);
    fails += test_fixture("report", fixture_report);
    fails += test_fixture("73", fixture_73);
    if (fails == 0) printf("ALL OK\n");
    return fails ? 1 : 0;
}
```

- [ ] **Step 2: Run**

```bash
cd host_mock && make host_synth_test && ./host_synth_test
```
Expected: all three PASS, exit 0.

- [ ] **Step 3: Commit**

```bash
git add host_mock/test_ft8_synth.cpp
git commit -m "test(synth): cover report and 73 fixtures"
```

---

### Task 8: Add byte-stuffing path and test

**Files:**
- Modify: `host_mock/test_ft8_synth.cpp`

- [ ] **Step 1: Write the byte-stuffing test**

Add to `test_ft8_synth.cpp` above `main()`:
```cpp
static int test_byte_stuffing(const char* name, const uint8_t* tones) {
    std::vector<uint8_t> stuffed(FT8_TX_SYNTH_SAMPLES);
    ft8_tx_synth_render(tones, 1500.0f, stuffed.data(), true);
    int count_3b = 0;
    for (uint8_t b : stuffed) if (b == 0x3B) ++count_3b;
    if (count_3b == 0) {
        printf("[%s/stuff] PASS: no 0x3B bytes in stuffed output\n", name);
        return 0;
    }
    printf("[%s/stuff] FAIL: %d 0x3B bytes present\n", name, count_3b);
    return 1;
}
```

Add calls to `main()`:
```cpp
    fails += test_byte_stuffing("cq", fixture_cq);
    fails += test_byte_stuffing("report", fixture_report);
    fails += test_byte_stuffing("73", fixture_73);
```

- [ ] **Step 2: Run**

```bash
cd host_mock && make host_synth_test && ./host_synth_test
```
Expected: all six PASS (three golden + three stuffing), exit 0. The implementation already handles `stuff=true` from Task 6, so this should pass first try.

- [ ] **Step 3: Commit**

```bash
git add host_mock/test_ft8_synth.cpp
git commit -m "test(synth): verify byte-stuffing eliminates 0x3B in output"
```

---

### Task 9: Streaming-variant equivalence test

**Files:**
- Modify: `host_mock/test_ft8_synth.cpp`

- [ ] **Step 1: Write the test**

Add to `test_ft8_synth.cpp`:
```cpp
static int test_stream_equiv(const char* name, const uint8_t* tones) {
    std::vector<uint8_t> oneshot(FT8_TX_SYNTH_SAMPLES);
    ft8_tx_synth_render(tones, 1500.0f, oneshot.data(), false);

    std::vector<uint8_t> streamed(FT8_TX_SYNTH_SAMPLES, 0);
    ft8_tx_synth_stream_t st;
    ft8_tx_synth_stream_init(&st, tones, 1500.0f, false);
    int total = 0;
    // Pull in irregular chunks to exercise boundary logic
    int chunk_sizes[] = {500, 1, 1843, 7, 100000};
    int cs_idx = 0;
    while (!ft8_tx_synth_stream_done(&st) && total < FT8_TX_SYNTH_SAMPLES) {
        int want = chunk_sizes[cs_idx++ % 5];
        if (want > FT8_TX_SYNTH_SAMPLES - total) want = FT8_TX_SYNTH_SAMPLES - total;
        int got = ft8_tx_synth_stream_pull(&st, streamed.data() + total, want);
        if (got == 0) break;
        total += got;
    }
    if (total != FT8_TX_SYNTH_SAMPLES) {
        printf("[%s/stream] FAIL: pulled %d / %d\n", name, total, FT8_TX_SYNTH_SAMPLES);
        return 1;
    }
    return compare_byte_exact((std::string(name) + "/stream").c_str(), streamed, oneshot);
}
```

(You'll need `#include <string>` at the top.)

Add to `main()`:
```cpp
    fails += test_stream_equiv("cq", fixture_cq);
    fails += test_stream_equiv("report", fixture_report);
    fails += test_stream_equiv("73", fixture_73);
```

- [ ] **Step 2: Run**

```bash
cd host_mock && make host_synth_test && ./host_synth_test
```
Expected: all nine PASS (3 golden + 3 stuff + 3 stream), exit 0.

- [ ] **Step 3: If stream output differs from one-shot at chunk boundaries**

Likely cause: `sample_in_sym` logic in `_pull` advances symbol before computing next sample, or vice versa. Fix in `ft8_tx_synth_stream_pull` and re-run.

- [ ] **Step 4: Commit**

```bash
git add host_mock/test_ft8_synth.cpp
git commit -m "test(synth): streaming variant equivalence with one-shot"
```

---

### Task 10: Phase 1 wrap-up — annotate done

**Files:** (none — git tag only)

- [ ] **Step 1: Verify entire phase passes from clean**

```bash
cd host_mock && make clean_synth && make host_synth_test && ./host_synth_test
```
Expected: all 9 tests PASS, exit 0.

- [ ] **Step 2: Tag Phase 1 done**

```bash
git tag phase-1-done -m "Phase 1: ft8_tx_synth + golden-file test complete"
```

- [ ] **Step 3: Update the spec status note**

Edit `docs/superpowers/specs/2026-05-26-trusdx-backend-design.md`, find the Phase 1 section, append a line at the end of the "Done when" block:

```markdown
> Status: PHASE 1 COMPLETE (commit <hash>, tag phase-1-done)
```

Use the actual commit hash from `git log -1 --format=%h`.

- [ ] **Step 4: Commit the status update**

```bash
git add docs/superpowers/specs/2026-05-26-trusdx-backend-design.md
git commit -m "docs: mark Phase 1 complete in trusdx design spec"
```

---

# Phase 1 → Phase 2 Gate

**Stop here unless:**
1. All Phase 1 tests are green from clean
2. `phase-1-done` tag exists
3. You have decided to start Phase 2 work

Phase 2 builds on the synth module from Phase 1. If Phase 1 is unstable, fix it before continuing.

---

# Phase 2 — trusdx backend + protocol streamer + hardware bring-up

**Phase 2 done when:** All seven hardware bring-up steps pass on real trusdx hardware; at least one full FT8 QSO completes with the trusdx as the radio.

**Hardware prerequisites:**
- (tr)uSDX with firmware ≥ 2.00t
- USB-C cable (cardputer to trusdx)
- Powered USB hub OR external power for the trusdx (cardputer host mode cannot supply TX current)
- A way to verify TX: nearby SDR, RTL-SDR with WSJT-X, or another rig

---

### Task 11: Change ops-table `begin_tx` signature

**Files:**
- Modify: `main/radio_control_backend.h`
- Modify: `main/radio_control.h`
- Modify: `main/radio_control.cpp`
- Modify: `main/radio_control_qmx.cpp`
- Modify: `main/radio_control_kh1_cat.cpp`
- Modify: `main/main.cpp` (callers)

- [ ] **Step 1: Update the ops type and getter declaration**

Edit `main/radio_control_backend.h`:
```c
typedef struct {
    const char* name;
    bool (*ready)(void);
    esp_err_t (*on_audio_start)(void);
    esp_err_t (*sync_frequency_mode)(int freq_hz);
    esp_err_t (*begin_tx)(int freq_hz, int tx_base_hz,
                          const uint8_t* tones, int n_tones);
    esp_err_t (*set_tone_hz)(float tone_hz);
    esp_err_t (*end_tx)(void);
    esp_err_t (*set_tune)(bool enable, int freq_hz, int tone_hz);
    esp_err_t (*set_time)(int hour, int minute, int second);
} radio_control_ops_t;

const radio_control_ops_t* radio_control_qmx_get_ops(void);
const radio_control_ops_t* radio_control_kh1_get_ops(void);
const radio_control_ops_t* radio_control_trusdx_get_ops(void);  // NEW
void radio_control_kh1_set_enabled(bool enabled);
bool radio_control_kh1_is_enabled(void);
esp_err_t radio_control_kh1_diag_test(char test_key, int freq_hz, int offset_hz, bool* out_fa_sent);
```

- [ ] **Step 2: Update the public dispatch header**

Edit `main/radio_control.h`:
```c
typedef enum {
    RADIO_CONTROL_QMX = 0,
    RADIO_CONTROL_KH1_CAT = 1,
    RADIO_CONTROL_TRUSDX = 2,
} radio_control_backend_t;

// ... existing decls ...

esp_err_t radio_control_begin_tx(int freq_hz, int tx_base_hz,
                                 const uint8_t* tones, int n_tones);
```

- [ ] **Step 3: Update the dispatcher**

Edit `main/radio_control.cpp`:
```cpp
static const radio_control_ops_t* current_ops(void) {
    switch (s_backend) {
    case RADIO_CONTROL_KH1_CAT: return radio_control_kh1_get_ops();
    case RADIO_CONTROL_TRUSDX:  return radio_control_trusdx_get_ops();
    case RADIO_CONTROL_QMX:
    default:                    return radio_control_qmx_get_ops();
    }
}

const char* radio_control_backend_name(radio_control_backend_t backend) {
    switch (backend) {
    case RADIO_CONTROL_QMX:     return "qmx";
    case RADIO_CONTROL_KH1_CAT: return "kh1_cat";
    case RADIO_CONTROL_TRUSDX:  return "trusdx";
    default:                    return "unknown";
    }
}

esp_err_t radio_control_begin_tx(int freq_hz, int tx_base_hz,
                                 const uint8_t* tones, int n_tones) {
    const radio_control_ops_t* ops = current_ops();
    if (!ops || !ops->begin_tx) return ESP_ERR_INVALID_STATE;
    return ops->begin_tx(freq_hz, tx_base_hz, tones, n_tones);
}
```

- [ ] **Step 4: Update QMX backend to accept the new signature**

Edit `main/radio_control_qmx.cpp`, the `qmx_begin_tx` function and ops table:
```cpp
static esp_err_t qmx_begin_tx(int freq_hz, int tx_base_hz,
                              const uint8_t* tones, int n_tones) {
    (void)freq_hz; (void)tx_base_hz; (void)tones; (void)n_tones;
    // existing body unchanged ...
}
```

- [ ] **Step 5: Update KH1 backend the same way**

Edit `main/radio_control_kh1_cat.cpp`:
```cpp
static esp_err_t kh1_begin_tx(int freq_hz, int tx_base_hz,
                              const uint8_t* tones, int n_tones) {
    (void)tones; (void)n_tones;
    // existing body unchanged
    ...
}
```

- [ ] **Step 6: Update callers in main.cpp**

Find all calls to `radio_control_begin_tx` (grep `radio_control_begin_tx`). Update each call site to pass the tones array. Where the tones aren't yet known (placeholder during transition), pass `nullptr, 0`:
```cpp
esp_err_t err = radio_control_begin_tx(freq_hz, g_tx_base_hz, nullptr, 0);
```

There may also be call sites inside `main.cpp` near line 4000 (per earlier grep) that have access to the symbol tones — pass them as well if available. If the tones array isn't easily reachable from that site, leave as nullptr for now; trusdx integration will revisit.

- [ ] **Step 7: Add a stub `radio_control_trusdx_get_ops` to satisfy the linker**

Create temporary `main/radio_control_trusdx.cpp` (filled in by Task 19):
```cpp
#include "radio_control_backend.h"
#include "esp_log.h"

static const char* TAG = "RADIO_TRUSDX";

static bool trusdx_ready(void) { return false; }
static esp_err_t trusdx_on_audio_start(void) { return ESP_OK; }
static esp_err_t trusdx_sync_freq(int freq_hz) { (void)freq_hz; return ESP_OK; }
static esp_err_t trusdx_begin_tx(int freq_hz, int tx_base_hz,
                                 const uint8_t* tones, int n_tones) {
    (void)freq_hz; (void)tx_base_hz; (void)tones; (void)n_tones; return ESP_OK;
}
static esp_err_t trusdx_set_tone_hz(float tone_hz) { (void)tone_hz; return ESP_OK; }
static esp_err_t trusdx_end_tx(void) { return ESP_OK; }
static esp_err_t trusdx_set_tune(bool e, int f, int t) { (void)e;(void)f;(void)t; return ESP_OK; }

static const radio_control_ops_t k_ops = {
    .name = "trusdx",
    .ready = trusdx_ready,
    .on_audio_start = trusdx_on_audio_start,
    .sync_frequency_mode = trusdx_sync_freq,
    .begin_tx = trusdx_begin_tx,
    .set_tone_hz = trusdx_set_tone_hz,
    .end_tx = trusdx_end_tx,
    .set_tune = trusdx_set_tune,
    .set_time = nullptr,
};

const radio_control_ops_t* radio_control_trusdx_get_ops(void) {
    return &k_ops;
}
```

Add `main/radio_control_trusdx.cpp` to `main/CMakeLists.txt` SRCS.

- [ ] **Step 8: Build firmware**

Run from repo root:
```bash
idf.py build
```
Expected: build succeeds. If any caller of `radio_control_begin_tx` was missed, the build fails with a clear error pointing to the line; fix and rebuild.

- [ ] **Step 9: Commit**

```bash
git add main/radio_control_backend.h main/radio_control.h main/radio_control.cpp \
        main/radio_control_qmx.cpp main/radio_control_kh1_cat.cpp \
        main/radio_control_trusdx.cpp main/main.cpp main/CMakeLists.txt
git commit -m "radio: extend begin_tx signature with (tones, n_tones); add TRUSDX stub"
```

---

### Task 12: Split `cat_cdc_*` from QMX-specific code

**Files:**
- Modify: `main/stream_uac.h`
- Modify: `main/stream_uac.cpp`

The goal: `cat_cdc_ready` and `cat_cdc_send` should not be conceptually tied to UAC audio. trusdx uses them without ever opening a UAC device.

Minimal change: keep the symbols in `stream_uac.h` (their existing home) but verify they don't depend on `uac_start*` being called first. Likely they only need the CDC class driver up.

- [ ] **Step 1: Read current implementation**

In `main/stream_uac.cpp`, find `cat_cdc_ready` and `cat_cdc_send`. Trace what initializes the CDC ACM endpoint. Identify whether it's gated on UAC enumeration.

- [ ] **Step 2: Add a function that starts CDC without UAC**

Add to `main/stream_uac.h`:
```c
// Start the USB host CDC class driver without expecting a UAC audio device.
// Used by trusdx backend (CAT + audio-over-CDC, no separate UAC).
// Idempotent — safe to call multiple times.
bool cat_cdc_start(void);
```

In `main/stream_uac.cpp`, factor out the existing CDC initialization into `cat_cdc_start()`. If UAC startup already calls this internally, leave that path; if not, have `uac_start*` call `cat_cdc_start()` first.

- [ ] **Step 3: Build firmware**

```bash
idf.py build
```
Expected: build succeeds. QMX behavior should be unchanged (UAC path still starts CDC indirectly).

- [ ] **Step 4: Commit**

```bash
git add main/stream_uac.h main/stream_uac.cpp
git commit -m "refactor(stream_uac): expose cat_cdc_start() for non-UAC backends"
```

---

### Task 13: Add `RadioType::TRUSDX` enum + parsers

**Files:**
- Modify: `main/main.cpp`

- [ ] **Step 1: Add the enum value**

In `main.cpp`, find the `RadioType` enum class (around line 2407 per earlier grep). Add:
```cpp
enum class RadioType {
    QMX = 0,
    KH1_USBC = 1,
    KH1_MIC = 2,
    TRUSDX = 3,
};
```

- [ ] **Step 2: Update `parse_radio_type`**

Find the function (around line 2418-2439). Add:
```cpp
    if (token == "TRUSDX" || token == "TRU" || token == "TRUSDX_CDC") {
        return RadioType::TRUSDX;
    }
```
before the final `return RadioType::QMX;`.

- [ ] **Step 3: Update `radio_type_to_string`**

Find the function (around line 2454). Add:
```cpp
    case RadioType::TRUSDX:   return "TRUSDX";
```

- [ ] **Step 4: Update `is_kh1_radio` to exclude TRUSDX**

Find (around line 2390). Confirm TRUSDX is correctly excluded (no change needed if logic only checks KH1_USBC/KH1_MIC).

- [ ] **Step 5: Update `radio_binding` mapping**

Find the binding switch (around line 2444). Add:
```cpp
    case RadioType::TRUSDX:
        return {AUDIO_SOURCE_TRUSDX_CDC, RADIO_CONTROL_TRUSDX};
```
The `AUDIO_SOURCE_TRUSDX_CDC` enum doesn't exist yet — Task 14 adds it. This will cause a build error in Task 13, fixed in Task 14.

- [ ] **Step 6: Try build (expect failure)**

```bash
idf.py build
```
Expected: error `AUDIO_SOURCE_TRUSDX_CDC was not declared`. This is intentional — Task 14 fixes it.

- [ ] **Step 7: Commit (broken-on-purpose)**

Don't commit yet — chain with Task 14.

---

### Task 14: Add `AUDIO_SOURCE_TRUSDX_CDC` enum + dispatch stub

**Files:**
- Modify: `main/audio_source.h`
- Modify: `main/audio_source.cpp`

- [ ] **Step 1: Add the enum value**

Edit `main/audio_source.h`:
```c
typedef enum {
    AUDIO_SOURCE_QMX_UAC = 0,
    AUDIO_SOURCE_USB_UAC_GENERIC = 1,
    AUDIO_SOURCE_KH1_MIC = 2,
    AUDIO_SOURCE_TRUSDX_CDC = 3,
} audio_source_backend_t;
```

- [ ] **Step 2: Update dispatch in audio_source.cpp**

Find the switch statements that dispatch on `audio_source_backend_t`. For each:

```cpp
// In audio_source_start:
    case AUDIO_SOURCE_TRUSDX_CDC:
        return stream_trusdx_cdc_start();

// In audio_source_stop:
    case AUDIO_SOURCE_TRUSDX_CDC:
        stream_trusdx_cdc_stop();
        return;

// In audio_source_is_streaming, etc — delegate similarly.
// In audio_source_backend_name:
    case AUDIO_SOURCE_TRUSDX_CDC: return "trusdx_cdc";
```

`stream_trusdx_cdc_start` etc. don't exist yet — Task 15 adds them. Forward-declare in `audio_source.cpp`:
```cpp
extern "C" bool stream_trusdx_cdc_start(void);
extern "C" void stream_trusdx_cdc_stop(void);
extern "C" bool stream_trusdx_cdc_is_streaming(void);
extern "C" const char* stream_trusdx_cdc_get_status_string(void);
extern "C" const char* stream_trusdx_cdc_get_debug_line1(void);
extern "C" const char* stream_trusdx_cdc_get_debug_line2(void);
extern "C" bool stream_trusdx_cdc_get_latest_waterfall_row(uint8_t* out, int len);
```

- [ ] **Step 3: Try build (expect failure on stream_trusdx_cdc symbols)**

```bash
idf.py build
```
Expected: link error on `stream_trusdx_cdc_*`. Intentional — Task 15 fixes it.

- [ ] **Step 4: Defer commit until Task 15 lands**

---

### Task 15: `stream_trusdx_cdc` skeleton

**Files:**
- Create: `main/stream_trusdx_cdc.h`
- Create: `main/stream_trusdx_cdc.cpp`
- Modify: `main/CMakeLists.txt`

- [ ] **Step 1: Public header**

Create `main/stream_trusdx_cdc.h`:
```c
#pragma once
#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

// PE1NNZ CAT-streaming protocol layer over USB-CDC for the (tr)uSDX.
// Multiplexes CAT commands and 8-bit PCM audio on the same serial link.

bool stream_trusdx_cdc_start(void);
void stream_trusdx_cdc_stop(void);
bool stream_trusdx_cdc_is_streaming(void);

esp_err_t stream_trusdx_cdc_begin_tx(const uint8_t* tones, int n_tones, float base_hz);
esp_err_t stream_trusdx_cdc_end_tx(void);

const char* stream_trusdx_cdc_get_status_string(void);
const char* stream_trusdx_cdc_get_debug_line1(void);
const char* stream_trusdx_cdc_get_debug_line2(void);
bool stream_trusdx_cdc_get_latest_waterfall_row(uint8_t* out, int len);

#ifdef __cplusplus
}
#endif
```

- [ ] **Step 2: Skeleton implementation (logs only, no real protocol yet)**

Create `main/stream_trusdx_cdc.cpp`:
```cpp
#include "stream_trusdx_cdc.h"
#include "stream_uac.h"  // for cat_cdc_start, cat_cdc_send
#include "esp_log.h"
#include <atomic>

static const char* TAG = "TRUSDX_CDC";

enum class State { IDLE, RX_STREAMING, TX_STREAMING };
static std::atomic<State> s_state{State::IDLE};

extern "C" bool stream_trusdx_cdc_start(void) {
    if (!cat_cdc_start()) {
        ESP_LOGW(TAG, "cat_cdc_start failed");
        return false;
    }
    // Send UA1; to enter PE1NNZ streaming mode
    const char* ua1 = "UA1;";
    esp_err_t r = cat_cdc_send((const uint8_t*)ua1, 4, 200);
    if (r != ESP_OK) {
        ESP_LOGW(TAG, "UA1; send failed: %s", esp_err_to_name(r));
        return false;
    }
    s_state = State::RX_STREAMING;
    ESP_LOGI(TAG, "state IDLE -> RX_STREAMING (UA1; sent)");
    return true;
}

extern "C" void stream_trusdx_cdc_stop(void) {
    if (s_state == State::TX_STREAMING) {
        stream_trusdx_cdc_end_tx();
    }
    s_state = State::IDLE;
    ESP_LOGI(TAG, "state -> IDLE");
}

extern "C" bool stream_trusdx_cdc_is_streaming(void) {
    return s_state != State::IDLE;
}

extern "C" esp_err_t stream_trusdx_cdc_begin_tx(const uint8_t* tones, int n_tones, float base_hz) {
    if (!tones || n_tones != 79) return ESP_ERR_INVALID_ARG;
    ESP_LOGI(TAG, "state %d -> TX_STREAMING (tones[0..4]=%d,%d,%d,%d,%d base=%.1f)",
             (int)s_state.load(), tones[0], tones[1], tones[2], tones[3], tones[4], base_hz);
    // Task 17 fills in the real TX pump.
    s_state = State::TX_STREAMING;
    return ESP_OK;
}

extern "C" esp_err_t stream_trusdx_cdc_end_tx(void) {
    ESP_LOGI(TAG, "TX_STREAMING -> RX_STREAMING (sending ;RX; x3)");
    const char* rx = ";RX;";
    for (int i = 0; i < 3; ++i) {
        cat_cdc_send((const uint8_t*)rx, 4, 200);
        // Task 17 will add the 100ms delay
    }
    s_state = State::RX_STREAMING;
    return ESP_OK;
}

extern "C" const char* stream_trusdx_cdc_get_status_string(void) { return "trusdx_cdc"; }
extern "C" const char* stream_trusdx_cdc_get_debug_line1(void) { return ""; }
extern "C" const char* stream_trusdx_cdc_get_debug_line2(void) { return ""; }
extern "C" bool stream_trusdx_cdc_get_latest_waterfall_row(uint8_t* out, int len) {
    (void)out; (void)len; return false;
}
```

- [ ] **Step 3: Add to CMakeLists**

Edit `main/CMakeLists.txt`, add `stream_trusdx_cdc.cpp` to `SRCS`.

- [ ] **Step 4: Build**

```bash
idf.py build
```
Expected: build succeeds. The TRUSDX option is now selectable in the binding code, but RX path and TX pump don't do anything yet.

- [ ] **Step 5: Commit all three tasks together**

```bash
git add main/main.cpp main/audio_source.h main/audio_source.cpp \
        main/stream_trusdx_cdc.h main/stream_trusdx_cdc.cpp \
        main/CMakeLists.txt
git commit -m "trusdx: add RadioType/audio_source/stream_trusdx_cdc skeleton (no protocol yet)"
```

---

### Task 16: RX path in `stream_trusdx_cdc`

**Files:**
- Modify: `main/stream_trusdx_cdc.cpp`
- Modify: `main/resample.h`
- Modify: `main/resample.cpp`

- [ ] **Step 1: Add `resample_7820_to_6k`**

Edit `main/resample.h`:
```c
// Resample 7820 Hz mono float -> 6000 Hz mono float (linear interpolation).
// state holds the trailing input sample for cross-buffer continuity.
int resample_7820_to_6k(resample_state_t* state, const float* in,
                        float* out, int in_samples);
```

Edit `main/resample.cpp`. Add a small `last_sample` field to `resample_state_t` (or use a small file-static if state needs to evolve — for now, plumb explicitly):

```c
// Implementation note: 7820/6000 = 1.30333...; output sample i corresponds to
// input position i * 1.30333. Linear interp.
int resample_7820_to_6k(resample_state_t* state, const float* in,
                        float* out, int in_samples) {
    (void)state;  // Trivial impl ignores cross-buffer continuity for first cut.
    const float ratio = 7820.0f / 6000.0f;
    int out_samples = (int)((float)in_samples / ratio);
    for (int i = 0; i < out_samples; ++i) {
        float pos = (float)i * ratio;
        int idx = (int)pos;
        float frac = pos - (float)idx;
        if (idx + 1 >= in_samples) {
            out[i] = in[in_samples - 1];
        } else {
            out[i] = in[idx] * (1.0f - frac) + in[idx + 1] * frac;
        }
    }
    return out_samples;
}
```

NOTE: Cross-buffer boundary handling is deferred — first cut treats each pull as a complete block. If audio decode reveals discontinuity artifacts, revisit.

- [ ] **Step 2: Add RX task in stream_trusdx_cdc**

Edit `main/stream_trusdx_cdc.cpp`. Add a FreeRTOS task that reads bytes from CDC, converts int8 → float, resamples, feeds the existing pipeline.

```cpp
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "ft8_audio_pipeline.h"
#include "resample.h"
#include <cstring>

// Ring buffer for raw 8-bit signed RX bytes (~2s at 7820 Hz)
#define RX_RING_BYTES 16384
static uint8_t s_rx_ring[RX_RING_BYTES];
static volatile int s_rx_head = 0;
static volatile int s_rx_tail = 0;

static TaskHandle_t s_rx_task = nullptr;

// Read callback for ft8_audio_pipeline_run
static int rx_pipeline_read(void* ctx, float* out, int max_samples) {
    (void)ctx;
    // Pull up to max_samples * (7820/6000) input bytes; resample to max_samples
    int wanted_in = (int)((float)max_samples * 7820.0f / 6000.0f) + 1;
    static float in_floats[2048];
    if (wanted_in > 2048) wanted_in = 2048;
    int got = 0;
    while (got < wanted_in && s_rx_tail != s_rx_head) {
        int8_t b = (int8_t)s_rx_ring[s_rx_tail];
        in_floats[got++] = (float)b / 128.0f;
        s_rx_tail = (s_rx_tail + 1) % RX_RING_BYTES;
    }
    if (got == 0) return 0;
    resample_state_t st;
    return resample_7820_to_6k(&st, in_floats, out, got);
}

static bool rx_should_stop(void* ctx) {
    (void)ctx;
    return s_state == State::IDLE;
}

static void on_block_processed(void* ctx) { (void)ctx; }

static void trusdx_rx_task(void* arg) {
    (void)arg;
    ESP_LOGI(TAG, "rx task started");
    ft8_audio_pipeline_config_t cfg = {
        .tag = "TRUSDX_RX",
        .ctx = nullptr,
        .read = rx_pipeline_read,
        .should_stop = rx_should_stop,
        .on_block_processed = on_block_processed,
    };
    ft8_audio_pipeline_run(&cfg);
    ESP_LOGI(TAG, "rx task exiting");
    vTaskDelete(nullptr);
}
```

Adding the CDC byte ingestion is the missing link. The CDC class driver should expose a callback or polling read; check how `stream_uac.cpp` consumes CDC data and mirror that pattern. Add a small consumer routine that copies CDC bytes into `s_rx_ring` when `s_state == RX_STREAMING`.

If the CDC driver in this project pushes bytes via a callback, register the trusdx ingest function in `stream_trusdx_cdc_start`. If it requires polling, spawn a second task that reads CDC into the ring buffer.

- [ ] **Step 3: Wire RX task start in `stream_trusdx_cdc_start`**

```cpp
extern "C" bool stream_trusdx_cdc_start(void) {
    if (!cat_cdc_start()) return false;
    if (cat_cdc_send((const uint8_t*)"UA1;", 4, 200) != ESP_OK) return false;
    s_state = State::RX_STREAMING;
    BaseType_t r = xTaskCreate(trusdx_rx_task, "trusdx_rx", 4096, nullptr, 5, &s_rx_task);
    if (r != pdPASS) {
        ESP_LOGW(TAG, "rx task create failed");
        return false;
    }
    ESP_LOGI(TAG, "state IDLE -> RX_STREAMING");
    return true;
}
```

- [ ] **Step 4: Build**

```bash
idf.py build
```
Expected: build succeeds.

- [ ] **Step 5: Commit**

```bash
git add main/stream_trusdx_cdc.cpp main/resample.h main/resample.cpp
git commit -m "trusdx: RX path — CDC ingest, 7820->6k resample, feed FT8 pipeline"
```

---

### Task 17: TX path in `stream_trusdx_cdc`

**Files:**
- Modify: `main/stream_trusdx_cdc.cpp`

- [ ] **Step 1: Add TX pump using ft8_tx_synth_stream**

Edit `main/stream_trusdx_cdc.cpp`. Add a TX task that streams synth output to CDC:

```cpp
#include "ft8_tx_synth.h"

static ft8_tx_synth_stream_t s_tx_stream;
static TaskHandle_t s_tx_task = nullptr;
static const bool TRUSDX_BYTE_STUFFING = true;  // mitigate 0x3B collisions

static void trusdx_tx_task(void* arg) {
    (void)arg;
    ESP_LOGI(TAG, "tx task started");
    // Enter TX mode
    const char* tx_enter = "UA1;TX0;";
    esp_err_t r = cat_cdc_send((const uint8_t*)tx_enter, 8, 200);
    if (r != ESP_OK) {
        ESP_LOGW(TAG, "tx enter failed: %s", esp_err_to_name(r));
        goto cleanup;
    }
    ESP_LOGI(TAG, "sent UA1;TX0; (8 bytes)");

    // Pump audio bytes
    {
        uint8_t chunk[500];
        int total_sent = 0;
        while (!ft8_tx_synth_stream_done(&s_tx_stream)) {
            int got = ft8_tx_synth_stream_pull(&s_tx_stream, chunk, sizeof(chunk));
            if (got <= 0) break;
            int64_t t0 = esp_timer_get_time();
            esp_err_t w = cat_cdc_send(chunk, got, 200);
            int64_t dt_us = esp_timer_get_time() - t0;
            if (w != ESP_OK) {
                ESP_LOGW(TAG, "audio cdc send failed: %s", esp_err_to_name(w));
                break;
            }
            total_sent += got;
            if (dt_us > 10000) {
                ESP_LOGW(TAG, "cdc write backpressure %lld us", dt_us);
            }
            if ((total_sent % 10000) < got) {
                ESP_LOGI(TAG, "pump %d audio bytes (synth %d/%d)",
                         got, total_sent, FT8_TX_SYNTH_SAMPLES);
            }
        }
        ESP_LOGI(TAG, "synth done, sent %d total bytes", total_sent);
    }

cleanup:
    // Exit TX mode: ;RX; x3 with 100ms gaps
    for (int i = 0; i < 3; ++i) {
        cat_cdc_send((const uint8_t*)";RX;", 4, 200);
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    s_state = State::RX_STREAMING;
    ESP_LOGI(TAG, "state TX_STREAMING -> RX_STREAMING");
    s_tx_task = nullptr;
    vTaskDelete(nullptr);
}

extern "C" esp_err_t stream_trusdx_cdc_begin_tx(const uint8_t* tones, int n_tones, float base_hz) {
    if (!tones || n_tones != 79) return ESP_ERR_INVALID_ARG;
    if (s_state != State::RX_STREAMING) {
        ESP_LOGW(TAG, "begin_tx in wrong state %d", (int)s_state.load());
        return ESP_ERR_INVALID_STATE;
    }
    ESP_LOGI(TAG, "state RX_STREAMING -> TX_STREAMING (tones[0..4]=%d,%d,%d,%d,%d base=%.1f)",
             tones[0], tones[1], tones[2], tones[3], tones[4], base_hz);
    ft8_tx_synth_stream_init(&s_tx_stream, tones, base_hz, TRUSDX_BYTE_STUFFING);
    s_state = State::TX_STREAMING;
    BaseType_t r = xTaskCreate(trusdx_tx_task, "trusdx_tx", 4096, nullptr, 5, &s_tx_task);
    if (r != pdPASS) {
        ESP_LOGW(TAG, "tx task create failed");
        s_state = State::RX_STREAMING;
        return ESP_FAIL;
    }
    return ESP_OK;
}
```

`end_tx` now becomes synchronous wait for the tx task to finish (it self-exits on synth done):

```cpp
extern "C" esp_err_t stream_trusdx_cdc_end_tx(void) {
    // Tx task exits on its own when synth is done.
    // External end_tx triggers early abort: mark synth done and let task wrap up.
    if (s_state == State::TX_STREAMING) {
        // Fast-forward synth to done
        s_tx_stream.sym_idx = FT8_TX_SYNTH_SYMBOLS;
        // Wait briefly for tx task to exit
        for (int i = 0; i < 50 && s_tx_task != nullptr; ++i) {
            vTaskDelay(pdMS_TO_TICKS(20));
        }
    }
    return ESP_OK;
}
```

(Adjust per actual `esp_timer_get_time` header — `#include "esp_timer.h"`.)

- [ ] **Step 2: Build**

```bash
idf.py build
```
Expected: build succeeds.

- [ ] **Step 3: Commit**

```bash
git add main/stream_trusdx_cdc.cpp
git commit -m "trusdx: TX path — synth pump, ;RX; teardown, backpressure logging"
```

---

### Task 18: Implement real `radio_control_trusdx`

**Files:**
- Modify: `main/radio_control_trusdx.cpp`

- [ ] **Step 1: Replace stub with real implementation**

Replace `main/radio_control_trusdx.cpp`:
```cpp
#include "radio_control_backend.h"
#include "stream_trusdx_cdc.h"
#include "stream_uac.h"  // cat_cdc_ready, cat_cdc_send

#include <cstdio>
#include <cstring>
#include "esp_log.h"

static const char* TAG = "RADIO_TRUSDX";

static bool trusdx_ready(void) {
    return cat_cdc_ready();
}

static esp_err_t send_cmd(const char* cmd) {
    if (!cat_cdc_ready()) return ESP_ERR_INVALID_STATE;
    return cat_cdc_send((const uint8_t*)cmd, strlen(cmd), 200);
}

static esp_err_t trusdx_on_audio_start(void) {
    // stream_trusdx_cdc_start will send UA1; — nothing to do here that isn't
    // already done by the audio source binding. Log presence.
    ESP_LOGI(TAG, "on_audio_start (UA1; sent by stream layer)");
    return ESP_OK;
}

static esp_err_t trusdx_sync_freq(int freq_hz) {
    // TS-480: MD2 = USB; FA = VFO-A frequency, 11 digits
    esp_err_t err = send_cmd("MD2;");
    if (err != ESP_OK) return err;
    char fa[24];
    snprintf(fa, sizeof(fa), "FA%011d;", freq_hz);
    err = send_cmd(fa);
    if (err == ESP_OK) ESP_LOGI(TAG, "sync MD2 FA=%d", freq_hz);
    return err;
}

static esp_err_t trusdx_begin_tx(int freq_hz, int tx_base_hz,
                                 const uint8_t* tones, int n_tones) {
    (void)freq_hz; (void)tx_base_hz;
    if (!tones || n_tones != 79) {
        ESP_LOGW(TAG, "begin_tx without tones — trusdx requires them");
        return ESP_ERR_INVALID_ARG;
    }
    return stream_trusdx_cdc_begin_tx(tones, n_tones, (float)tx_base_hz);
}

static esp_err_t trusdx_set_tone_hz(float tone_hz) {
    (void)tone_hz;
    // no-op for trusdx — audio synth carries the tone
    return ESP_OK;
}

static esp_err_t trusdx_end_tx(void) {
    return stream_trusdx_cdc_end_tx();
}

static esp_err_t trusdx_set_tune(bool enable, int freq_hz, int tone_hz) {
    if (!enable) {
        return stream_trusdx_cdc_end_tx();
    }
    // Render a single repeating tone via the synth path
    static uint8_t single_tone[79];
    int tone_idx = (int)((tone_hz - 1500) / 6.25f + 0.5f);
    if (tone_idx < 0) tone_idx = 0;
    if (tone_idx > 7) tone_idx = 7;
    memset(single_tone, tone_idx, sizeof(single_tone));
    esp_err_t err = trusdx_sync_freq(freq_hz);
    if (err != ESP_OK) return err;
    return stream_trusdx_cdc_begin_tx(single_tone, 79, 1500.0f);
}

static const radio_control_ops_t k_ops = {
    .name = "trusdx",
    .ready = trusdx_ready,
    .on_audio_start = trusdx_on_audio_start,
    .sync_frequency_mode = trusdx_sync_freq,
    .begin_tx = trusdx_begin_tx,
    .set_tone_hz = trusdx_set_tone_hz,
    .end_tx = trusdx_end_tx,
    .set_tune = trusdx_set_tune,
    .set_time = nullptr,
};

const radio_control_ops_t* radio_control_trusdx_get_ops(void) {
    return &k_ops;
}
```

- [ ] **Step 2: Build**

```bash
idf.py build
```
Expected: succeeds.

- [ ] **Step 3: Commit**

```bash
git add main/radio_control_trusdx.cpp
git commit -m "trusdx: implement radio_control_ops_t (CAT + stream delegation)"
```

---

### Task 19: Wire tones into begin_tx call sites

**Files:**
- Modify: `main/main.cpp`

The Task 11 transition left calls passing `nullptr, 0`. Find the call site that has access to the symbol tones (around line 4000 area, in the FT8 TX scheduling code) and pass the real tones.

- [ ] **Step 1: Locate the symbol-generation call site**

Run:
```bash
grep -n "radio_control_begin_tx\|tone_hz_for_symbol\|ft8_encode" main/main.cpp
```
Identify which function generates the symbol tones for TX. There should be either:
- A pre-computed `uint8_t tones[79]` array near the begin_tx call, OR
- An on-the-fly `ft8_encode` call producing tones

If tones are computed elsewhere and only `set_tone_hz` is called per-symbol, you'll need to lift the encoding earlier — call `ft8_encode` once, store the result, pass to `begin_tx`, then continue to call `set_tone_hz` for QMX/KH1 backends as before (their `set_tone_hz` is the real driver).

- [ ] **Step 2: Pass tones to begin_tx**

Update the call site:
```cpp
uint8_t tones[79];
// ... ft8_encode(payload, tones); already exists somewhere — relocate if needed ...
esp_err_t err = radio_control_begin_tx(freq_hz, g_tx_base_hz, tones, 79);
```

- [ ] **Step 3: Build**

```bash
idf.py build
```
Expected: succeeds.

- [ ] **Step 4: Commit**

```bash
git add main/main.cpp
git commit -m "main: pass FT8 tones to radio_control_begin_tx"
```

---

### Task 20: Add TRUSDX to menu N→3

**Files:**
- Modify: `main/main.cpp`

- [ ] **Step 1: Find the menu N→3 handler**

Search:
```bash
grep -n "MENU.*P2\|N->3\|radio.*menu\|cycle.*radio" main/main.cpp
```
Find the function that renders/updates the radio selector display string.

- [ ] **Step 2: Update display string**

If the cycle is hardcoded as `QMX / KH1-USBC / KH1-MIC`, extend to include `TRUSDX`. Use the existing `radio_type_to_string` to render.

- [ ] **Step 3: Verify cycle order**

The cycle should be: QMX → KH1-USBC → KH1-MIC → TRUSDX → QMX. Confirm the handler advances and saves the selected `RadioType`.

- [ ] **Step 4: Build and flash to cardputer (no trusdx needed)**

```bash
idf.py build flash monitor
```
Expected: device boots. Press `N` then `3` and verify TRUSDX appears as an option. Selecting it should log "Selected radio control backend=trusdx" in the serial monitor.

- [ ] **Step 5: Commit**

```bash
git add main/main.cpp
git commit -m "ui: add TRUSDX option to radio selector menu (N->3)"
```

---

### Task 21: README documentation

**Files:**
- Modify: `README.md`

- [ ] **Step 1: Add `## trusdx Connections` section**

Insert after the KH1 section in `README.md`:

```markdown
## trusdx Connections

(tr)uSDX with firmware ≥ 2.00t (PE1NNZ CAT-streaming support required).

```text
┌──────────────────┐                 ┌─────────────────────────────┐
│ (tr)uSDX         │                 │ Cardputer ADV               │
│                  │                 │ USB-C                       │
│ USB-C ───────────┼─────────────────┤ USB-C (host mode)           │
│                  │                 │                             │
└──────────────────┘                 └─────────────────────────────┘
```

- Single USB-C cable carries CAT + audio (PE1NNZ CAT-streaming protocol).
- **Power**: the Cardputer cannot supply the trusdx's TX current (~400–700 mA). Use a powered USB hub OR supply the trusdx separately and use a data-only USB cable.
- Mode select: `N` → 3 → `TRUSDX`.
- Verified trusdx firmware: 2.00t and later.
```

- [ ] **Step 2: Commit**

```bash
git add README.md
git commit -m "docs: README section for trusdx connections + power note"
```

---

### Task 22: Pre-hardware build verification

- [ ] **Step 1: Clean build**

```bash
idf.py fullclean && idf.py build
```
Expected: build succeeds, no warnings related to trusdx code.

- [ ] **Step 2: Flash & verify menu**

```bash
idf.py flash monitor
```
Press `N` → 3 → confirm `QMX / KH1-USBC / KH1-MIC / TRUSDX` cycle. Select TRUSDX, observe log: `Selected radio control backend=trusdx`. Without hardware, `audio_source_start` will fail or no-op when no trusdx is connected — note the failure mode.

- [ ] **Step 3: Tag pre-hardware milestone**

```bash
git tag phase-2-prehardware -m "Phase 2: code complete, awaiting trusdx hardware"
```

---

# Hardware Bring-Up (requires real trusdx)

Each step localizes a class of failure. Stop and debug at any failing step.

---

### Task 23: CDC enumeration verification

**Hardware required:** trusdx powered on, USB-C connected to cardputer.

- [ ] **Step 1: Power up, observe serial monitor**

Run `idf.py monitor`. Plug in trusdx (with external power; cardputer can't supply alone). Watch for USB host enumeration messages — should see CDC class device detected.

- [ ] **Step 2: Select TRUSDX in menu**

Press `N` → 3 → TRUSDX. Look for `TRUSDX_CDC: state IDLE -> RX_STREAMING (UA1; sent)`.

**If failure:** CDC didn't enumerate. Check:
- USB cable is data-capable (not power-only)
- trusdx firmware version
- ESP32-S3 USB host stack init in `cat_cdc_start`

---

### Task 24: CAT ID; response

- [ ] **Step 1: Send `ID;` manually if there's a CAT debug command**

If main.cpp exposes a way to send arbitrary CAT (look for the `C` mode from older versions, or use the debug shell), send `ID;`. Expected response: `ID020;` (TS-480 ID).

If no path exists, add temporary code: in `stream_trusdx_cdc_start`, after `UA1;`, send `ID;` and log the next ~8 bytes received from CDC.

**If wrong/no response:** trusdx isn't speaking TS-480 CAT — verify firmware, baud rate (115200), serial settings (8N1).

---

### Task 25: Frequency set

- [ ] **Step 1: From band menu, choose a band**

Press `B`, select 20m (14074000). On trusdx, the display should change to 14.074.00.

**If display doesn't change:**
- Check log for `RADIO_TRUSDX: sync MD2 FA=14074000`
- If sent but no change, format mismatch — verify `FA%011d;` is `FA14074000;` (11 digits)

---

### Task 26: Enter streaming mode (UA1)

Already verified in Task 23 if `RX_STREAMING` state was reached. Otherwise re-do.

---

### Task 27: RX decode on a known-active band

- [ ] **Step 1: Tune to a known active FT8 frequency**

20m FT8 (14074000) during daylight on weekdays is reliable. Press `R` to enter RX mode.

- [ ] **Step 2: Watch decode display for 2-3 minutes**

Decoded messages should appear. Verify against another receiver on the same band (online WebSDR is fine).

**If no decodes but signal is present:**
- Check audio levels — 8-bit signed audio may be saturated or too quiet
- Check sample rate assumption (7820 Hz)
- Check resample correctness

---

### Task 28: Tune mode (single continuous tone)

**Setup:** Connect a dummy load to the trusdx; do NOT transmit into an antenna without coordination.

- [ ] **Step 1: Enter STATUS → Tune**

Press `S` → 4 (Tune toggle). The trusdx should key (VOX from continuous tone) and the bargraph should swing.

**If trusdx doesn't key:**
- Check log for `TX_STREAMING` state
- Check synth pump is producing non-silence bytes
- Check `;` byte stuffing isn't blocking all output (it shouldn't, but verify)

- [ ] **Step 2: Disable tune**

Press `S` → 4 again. trusdx should drop back to RX.

---

### Task 29: Full FT8 TX (CQ on a dummy load)

- [ ] **Step 1: Setup**

Connect a nearby SDR or another rig to receive your TX. Cardputer + trusdx → dummy load.

- [ ] **Step 2: Send a CQ**

From RX mode, select "CQ" beacon or trigger a manual CQ. Watch for `TRUSDX_CDC: state RX_STREAMING -> TX_STREAMING` followed by audio pump logs.

- [ ] **Step 3: Verify on the other receiver**

WSJT-X (or any FT8 receiver) on the other side should decode your callsign and grid.

**If signal is heard but not decoded:**
- Synth produces audio but timing/spectral content is wrong
- Compare TX recording to a reference WSJT-X TX of the same message
- Check `phase` continuity across symbols

**If signal is not heard:**
- Power-related — confirm trusdx has enough current
- VOX silence threshold — verify synth peak is high enough (~70%)

---

### Task 30: Full QSO

- [ ] **Step 1: Move to a live antenna and find an active band**

- [ ] **Step 2: Either call CQ or answer a CQ**

Complete the QSO. Verify ADIF log on the cardputer SD card afterwards.

- [ ] **Step 3: Tag Phase 2 complete**

```bash
git tag phase-2-done -m "Phase 2: end-to-end trusdx FT8 QSO verified on hardware"
```

- [ ] **Step 4: Update spec status**

Edit `docs/superpowers/specs/2026-05-26-trusdx-backend-design.md`, append at the bottom:
```markdown

---
## Status (post-implementation)
- Phase 1: COMPLETE (tag `phase-1-done`)
- Phase 2: COMPLETE (tag `phase-2-done`); first verified QSO logged on <date>.
```

```bash
git add docs/superpowers/specs/2026-05-26-trusdx-backend-design.md
git commit -m "docs: mark Phase 2 complete; first QSO logged"
```
