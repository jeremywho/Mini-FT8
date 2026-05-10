#ifndef _TX_E2E_SYNTH_H_
#define _TX_E2E_SYNTH_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/// Synthesize phase-continuous FSK waveform from tone sequence
///
/// @param tones       Array of tone indices (0..7 for FT8, 0..3 for FT4)
/// @param nn          Number of tones
/// @param symbol_period_s  Duration of each symbol in seconds
/// @param base_hz     Base frequency in Hertz (e.g., 1500.0)
/// @param spacing_hz  Tone spacing in Hertz (6.25 for FT8, 20.8333 for FT4)
/// @param sample_rate Sample rate in Hertz (e.g., 12000)
/// @param amplitude   Peak amplitude (e.g., 0.5)
/// @param out_samples Output PCM buffer (pre-allocated)
/// @param out_len     [OUT] Number of samples written
void synth_fsk(const uint8_t* tones, int nn,
               float symbol_period_s, float base_hz, float spacing_hz,
               int sample_rate, float amplitude,
               float* out_samples, int* out_len);

#ifdef __cplusplus
}
#endif

#endif // _TX_E2E_SYNTH_H_
