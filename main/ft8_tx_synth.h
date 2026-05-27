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
    bool           apply_byte_stuffing;
    int            sym_idx;       // current symbol 0..79 (79 = done)
    int            sample_in_sym; // current sample within symbol 0..SPS-1 (reset to 0 when SPS reached)
    float          phase;         // continuous-phase FSK accumulator, radians in [0, 2π)
} ft8_tx_synth_stream_t;

// Initialises stream state. Must be called once before the first pull.
void ft8_tx_synth_stream_init(ft8_tx_synth_stream_t* s,
                              const uint8_t* tones, float base_hz,
                              bool apply_byte_stuffing);

// Pulls up to max_bytes audio samples into out. Returns count written.
// Returns 0 when stream is exhausted (see ft8_tx_synth_stream_done).
int ft8_tx_synth_stream_pull(ft8_tx_synth_stream_t* s,
                             uint8_t* out, int max_bytes);

bool ft8_tx_synth_stream_done(const ft8_tx_synth_stream_t* s);

#ifdef __cplusplus
}
#endif
