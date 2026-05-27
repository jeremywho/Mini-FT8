#include "ft8_tx_synth.h"
#include <cstring>

extern "C" void ft8_tx_synth_render(const uint8_t* tones, float base_hz,
                                    uint8_t* out_pcm_u8, bool apply_byte_stuffing) {
    (void)tones; (void)base_hz; (void)apply_byte_stuffing;
    // Emit silence (midpoint) so the test runs and reports byte diffs.
    std::memset(out_pcm_u8, 128, FT8_TX_SYNTH_SAMPLES);
}

extern "C" void ft8_tx_synth_stream_init(ft8_tx_synth_stream_t* s,
        const uint8_t* tones, float base_hz, bool apply_byte_stuffing) {
    s->tones = tones;
    s->base_hz = base_hz;
    s->apply_byte_stuffing = apply_byte_stuffing;
    s->sym_idx = 0;
    s->sample_in_sym = 0;
    s->phase = 0.0f;
}

extern "C" int ft8_tx_synth_stream_pull(ft8_tx_synth_stream_t* s,
        uint8_t* out, int max_bytes) {
    (void)s; (void)out; (void)max_bytes;
    return 0;  // stub returns nothing — T6 implements
}

extern "C" bool ft8_tx_synth_stream_done(const ft8_tx_synth_stream_t* s) {
    return s->sym_idx >= FT8_TX_SYNTH_SYMBOLS;
}
