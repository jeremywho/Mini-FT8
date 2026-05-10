#include "synth.h"
#include <math.h>
#include <string.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

void synth_fsk(const uint8_t* tones, int nn,
               float symbol_period_s, float base_hz, float spacing_hz,
               int sample_rate, float amplitude,
               float* out_samples, int* out_len) {
    int samples_per_symbol = (int)roundf(symbol_period_s * sample_rate);
    double phase = 0.0;
    int idx = 0;

    for (int i = 0; i < nn; i++) {
        // Current tone frequency
        double freq = base_hz + spacing_hz * tones[i];
        double phase_inc = 2.0 * M_PI * freq / sample_rate;

        // Generate samples for this symbol
        for (int s = 0; s < samples_per_symbol; s++) {
            out_samples[idx++] = amplitude * sinf((float)phase);
            phase += phase_inc;

            // Keep phase bounded to avoid precision loss
            if (phase > 2.0 * M_PI) {
                phase -= 2.0 * M_PI;
            }
        }
    }

    *out_len = idx;
}
