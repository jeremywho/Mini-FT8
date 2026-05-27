#pragma once
// Canned outputs of ft8_encode() for three FT8 messages.
// Each array is 79 bytes, values in 0..7 (8-FSK symbol indices).
//
// Regenerating fixtures (when you need new ones):
//   1. Clone upstream ft8_lib (https://github.com/kgoba/ft8_lib) — this repo
//      vendors only ft8_lib/ft8/* and ft8_lib/common/*, not the demo/ tree
//      where gen_ft8.c lives.
//   2. Temporarily add a printf loop right after the ft8_encode() call in
//      ft8_lib/demo/gen_ft8.c:
//          for (int i = 0; i < FT8_NN; ++i) printf("%d,", tones[i]);
//          printf("\n");
//      Then build (per ft8_lib's build instructions) and run:
//          ./gen_ft8 "<message>" /tmp/out.wav
//   3. Copy the printed 79 comma-separated tone values into the fixture array.
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
