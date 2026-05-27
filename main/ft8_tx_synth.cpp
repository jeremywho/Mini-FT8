#include "ft8_tx_synth.h"
#include <cmath>
#include <cstring>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

static const float TONE_SPACING = 6.25f;
static const float PEAK_EXCURSION = 89.0f;  // ~70% of 8-bit range
static const uint8_t MID = 128;

// Use round-half-away-from-zero to match Python's int(scaled + 0.5)
// convention. lrintf would use banker's rounding which diverges from Python's
// round() on .5 boundaries.
static inline uint8_t sample_to_u8(float s) {
    float scaled = s * PEAK_EXCURSION;
    int v = (scaled >= 0.0f) ? (int)(scaled + 0.5f) : (int)(scaled - 0.5f);
    v += MID;
    if (v < 0) v = 0;
    if (v > 255) v = 255;
    return (uint8_t)v;
}

extern "C" void ft8_tx_synth_render(const uint8_t* tones, float base_hz,
                                    uint8_t* out, bool apply_byte_stuffing) {
    float phase = 0.0f;
    int idx = 0;
    const float fs = (float)FT8_TX_SYNTH_SAMPLE_RATE;
    for (int sym = 0; sym < FT8_TX_SYNTH_SYMBOLS; ++sym) {
        float f = base_hz + (float)tones[sym] * TONE_SPACING;
        float dphi = 2.0f * (float)M_PI * f / fs;
        for (int i = 0; i < FT8_TX_SYNTH_SPS; ++i) {
            uint8_t b = sample_to_u8(sinf(phase));
            if (apply_byte_stuffing && b == 0x3B) b = 0x3C;
            out[idx++] = b;
            phase += dphi;
            if (phase > 2.0f * (float)M_PI) phase -= 2.0f * (float)M_PI;
        }
    }
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
    int written = 0;
    const float fs = (float)FT8_TX_SYNTH_SAMPLE_RATE;
    while (written < max_bytes && s->sym_idx < FT8_TX_SYNTH_SYMBOLS) {
        float f = s->base_hz + (float)s->tones[s->sym_idx] * TONE_SPACING;
        float dphi = 2.0f * (float)M_PI * f / fs;
        uint8_t b = sample_to_u8(sinf(s->phase));
        if (s->apply_byte_stuffing && b == 0x3B) b = 0x3C;
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
