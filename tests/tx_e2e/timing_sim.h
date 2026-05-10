// timing_sim.h
// ──────────────────────────────────────────────────────────────────────────────
// Shared deterministic simulation utilities used by L3 and L4 TX timing tests.
//
// These functions model the firmware's tx_tick() scheduling in two
// architectures:
//
//  COUPLED   — tx_tick() called from the main UI loop.
//              Effective poll period = vTaskDelay_ms + ui_work_ms.
//              ui_work_ms represents time spent doing display updates, etc.
//
//  ISOLATED  — tx_tick() called from a hardware timer (e.g. esp_timer).
//              Poll period = timer_ms exactly; ui_work_ms is irrelevant.
//
// Both models are single-threaded and deterministic so tests produce
// identical results on every run without actual FreeRTOS or hardware.
// ──────────────────────────────────────────────────────────────────────────────

#pragma once

#include <vector>
#include <cmath>
#include <cassert>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// ── Core poll simulation ──────────────────────────────────────────────────────

// Simulate the firmware's absolute-time tx_tick():
//   next_tone_time = slot_start + tone_idx * symbol_period_ms
//
// Called every poll_ms.  Returns the actual wall-clock fire time (ms from
// slot start) at which each of the nn tones is sent.
//
// This is the same for both COUPLED and ISOLATED: in COUPLED the caller
// passes (vTaskDelay_ms + ui_work_ms) as poll_ms; in ISOLATED the caller
// passes timer_ms regardless of ui_work_ms.
inline std::vector<int> simulate_poll(int poll_ms, int nn, int symbol_period_ms)
{
    assert(poll_ms > 0);
    std::vector<int> fire_ms(nn, 0);
    int now_ms = 0;
    for (int tone_idx = 0; tone_idx < nn; tone_idx++) {
        int target_ms = tone_idx * symbol_period_ms;
        while (now_ms < target_ms)
            now_ms += poll_ms;
        fire_ms[tone_idx] = now_ms;
    }
    return fire_ms;
}

// ── Timing-jitter statistics ──────────────────────────────────────────────────

struct JitterStats {
    int max_late_ms;      // worst single-tone lateness vs ideal
    int min_sym_dur_ms;   // shortest inter-tone interval
    int max_sym_dur_ms;   // longest  inter-tone interval
    int target_sym_ms;
};

inline JitterStats compute_jitter(const std::vector<int>& fire_ms,
                                  int nn, int symbol_period_ms)
{
    JitterStats s{};
    s.target_sym_ms = symbol_period_ms;
    s.min_sym_dur_ms = symbol_period_ms;
    s.max_sym_dur_ms = symbol_period_ms;
    for (int i = 0; i < nn; i++) {
        int late = fire_ms[i] - i * symbol_period_ms;
        if (late > s.max_late_ms) s.max_late_ms = late;
        if (i + 1 < nn) {
            int dur = fire_ms[i + 1] - fire_ms[i];
            if (dur < s.min_sym_dur_ms) s.min_sym_dur_ms = dur;
            if (dur > s.max_sym_dur_ms) s.max_sym_dur_ms = dur;
        }
    }
    return s;
}

// ── Phase-continuous FSK synthesis from fire times ───────────────────────────

// Synthesize a phase-continuous FSK signal using actual (potentially jittered)
// tone fire times rather than the ideal equal-duration grid.
//
// fire_ms[i]     : wall-clock time (ms from slot start) when tone i was sent
// last_sym_ms    : duration to use for the last symbol (nominal period)
// Returns float PCM samples with silence padding prepended and appended.
inline std::vector<float> fire_times_to_pcm(
        const std::vector<int>& fire_ms,
        const uint8_t* tones, int nn,
        float base_hz, float spacing_hz,
        int sample_rate, float amplitude,
        int last_sym_ms,
        int pad_ms = 500)
{
    // Build per-symbol sample counts from actual fire times
    std::vector<int> samp(nn);
    for (int i = 0; i < nn - 1; i++) {
        int dur_ms = fire_ms[i + 1] - fire_ms[i];
        samp[i] = (int)lrintf((float)dur_ms * sample_rate / 1000.0f);
    }
    samp[nn - 1] = (int)lrintf((float)last_sym_ms * sample_rate / 1000.0f);

    int pad_samp  = (int)(pad_ms * sample_rate / 1000);
    int sig_samp  = 0;
    for (int i = 0; i < nn; i++) sig_samp += samp[i];
    int total     = pad_samp + sig_samp + pad_samp;

    std::vector<float> pcm(total, 0.0f);
    float* sig = pcm.data() + pad_samp;

    double phase = 0.0;
    int idx = 0;
    for (int i = 0; i < nn; i++) {
        double freq      = base_hz + spacing_hz * tones[i];
        double phase_inc = 2.0 * M_PI * freq / sample_rate;
        for (int s = 0; s < samp[i]; s++) {
            sig[idx++] = amplitude * (float)sin(phase);
            phase      += phase_inc;
            if (phase > 2.0 * M_PI) phase -= 2.0 * M_PI;
        }
    }
    return pcm;
}
