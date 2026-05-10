// test_ta_format.cpp
// ──────────────────────────────────────────────────────────────────────────────
// Validates the TA command string formatting logic used by qmx_set_tone_hz()
// and tx_send_ta() in the firmware.
//
// The firmware formats QMX audio-frequency commands as:
//   TA%04d.%02d;   e.g. "TA1500.00;" or "TA1520.83;"
//
// BUG (fixed): Using lrintf() for ta_int rounds to nearest, so for a tone like
// 1520.8333 Hz the integer part rounds UP to 1521, making the fractional part
// negative (-0.1667).  The resulting command "TA1521.-17;" is invalid.
//
// FIX: Use floorf() so ta_int always truncates DOWN → frac ∈ [0, 1) →
// ta_frac is always 0..100.
//
// This test enumerates every tone index for both FT8 (8 levels, 6.25 Hz) and
// FT4 (4 levels, 20.8333 Hz) at several representative base frequencies and
// asserts:
//   1. ta_frac >= 0
//   2. ta_frac <= 100
//   3. Formatted string matches "TA%04d.%02d;" with no negative fields
//   4. The formatted command round-trips to within 0.01 Hz of the input
// ──────────────────────────────────────────────────────────────────────────────

#include <cstdio>
#include <cmath>
#include <cstring>
#include <cstdlib>

// Reimplementation of firmware's qmx_set_tone_hz formatting logic — both the
// old (buggy) and new (fixed) versions — so this test is self-contained.

// Fixed version (uses floorf)
static void format_ta_fixed(float tone_hz, char* out, int out_len) {
    int   ta_int  = (int)floorf(tone_hz);
    float frac    = tone_hz - (float)ta_int;   // always [0, 1)
    int   ta_frac = (int)lrintf(frac * 100.0f); // 0..100
    snprintf(out, out_len, "TA%04d.%02d;", ta_int, ta_frac);
}

// Old (buggy) version (uses lrintf) — kept so we can demonstrate the failure
static void format_ta_buggy(float tone_hz, char* out, int out_len) {
    int   ta_int  = (int)lrintf(tone_hz);
    float frac    = tone_hz - (float)ta_int;
    int   ta_frac = (int)lrintf(frac * 100.0f);
    snprintf(out, out_len, "TA%04d.%02d;", ta_int, ta_frac);
}

struct ToneSet {
    const char* name;
    float       base_hz;
    int         n_tones;
    float       spacing_hz;
};

static const ToneSet SETS[] = {
    { "FT8@1500", 1500.0f,  8, 6.25f      },
    { "FT8@300",   300.0f,  8, 6.25f      },
    { "FT8@2700", 2700.0f,  8, 6.25f      },
    { "FT4@1500", 1500.0f,  4, 20.8333f   },
    { "FT4@300",   300.0f,  4, 20.8333f   },
    { "FT4@2700", 2700.0f,  4, 20.8333f   },
};
static const int N_SETS = (int)(sizeof(SETS) / sizeof(SETS[0]));

int main()
{
    printf("TA Command Format Test\n");
    printf("======================================\n\n");

    int passed = 0, failed = 0;

    // ── Part 1: Demonstrate the bug (informational, not a failure) ────────
    printf("Part 1: buggy lrintf() formatting (informational — shows fixed cases)\n");
    int buggy_count = 0;
    for (int s = 0; s < N_SETS; s++) {
        const ToneSet& ts = SETS[s];
        for (int t = 0; t < ts.n_tones; t++) {
            float hz = ts.base_hz + t * ts.spacing_hz;
            char buggy[32];
            format_ta_buggy(hz, buggy, sizeof(buggy));
            // Check if the command contains a negative number
            bool has_negative = (strstr(buggy, ".-") != nullptr || strstr(buggy, " -") != nullptr);
            if (has_negative) {
                printf("  BUG: %s tone %d @ %.4f Hz → \"%s\"\n",
                       ts.name, t, (double)hz, buggy);
                buggy_count++;
            }
        }
    }
    if (buggy_count == 0) {
        printf("  (no buggy cases found — all tones happen to be exact)\n");
    }
    printf("\n");

    // ── Part 2: Validate fixed formatting ────────────────────────────────
    printf("Part 2: fixed floorf() formatting\n");
    for (int s = 0; s < N_SETS; s++) {
        const ToneSet& ts = SETS[s];
        bool set_ok = true;
        for (int t = 0; t < ts.n_tones; t++) {
            float hz = ts.base_hz + t * ts.spacing_hz;
            char cmd[32];
            format_ta_fixed(hz, cmd, sizeof(cmd));

            // Parse ta_int and ta_frac back out of the formatted string
            int  parsed_int, parsed_frac;
            int n = sscanf(cmd, "TA%d.%d;", &parsed_int, &parsed_frac);

            bool ok = true;

            // Must parse correctly
            if (n != 2) {
                printf("  FAIL %s tone %d @ %.4f Hz: sscanf failed on \"%s\"\n",
                       ts.name, t, (double)hz, cmd);
                ok = false;
            }

            // ta_frac must be non-negative
            if (ok && parsed_frac < 0) {
                printf("  FAIL %s tone %d @ %.4f Hz: negative ta_frac=%d in \"%s\"\n",
                       ts.name, t, (double)hz, parsed_frac, cmd);
                ok = false;
            }

            // ta_frac must be 0..100
            if (ok && parsed_frac > 100) {
                printf("  FAIL %s tone %d @ %.4f Hz: ta_frac=%d > 100 in \"%s\"\n",
                       ts.name, t, (double)hz, parsed_frac, cmd);
                ok = false;
            }

            // Round-trip: reconstructed Hz must be within 0.01 of original
            if (ok) {
                float reconstructed = (float)parsed_int + (float)parsed_frac / 100.0f;
                float diff = fabsf(reconstructed - hz);
                if (diff > 0.01f) {
                    printf("  FAIL %s tone %d @ %.4f Hz: round-trip error %.4f Hz (\"%s\")\n",
                           ts.name, t, (double)hz, (double)diff, cmd);
                    ok = false;
                }
            }

            // No minus sign anywhere in the command
            if (ok && strchr(cmd, '-') != nullptr) {
                printf("  FAIL %s tone %d @ %.4f Hz: minus sign in \"%s\"\n",
                       ts.name, t, (double)hz, cmd);
                ok = false;
            }

            if (!ok) {
                set_ok = false;
                failed++;
            } else {
                passed++;
            }
        }
        printf("  %s: %s\n", ts.name, set_ok ? "all OK" : "FAILED");
    }

    printf("\n======================================\n");
    printf("Results: %d/%d passed\n", passed, passed + failed);
    return (failed > 0) ? 1 : 0;
}
