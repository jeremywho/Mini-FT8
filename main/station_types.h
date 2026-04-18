#pragma once

// ============================================================================
// station_types.h
//
// Enums and plain types describing Mini-FT8 station state. Shared between
// main.cpp (owner of the runtime globals) and core_api.cpp (exposing the
// functional-core API). Kept separate so core_api.cpp can access the
// underlying globals directly without main.cpp having to expose its
// internal headers.
// ============================================================================

#include <string>

// Beacon (automatic CQ) mode.
enum class BeaconMode { OFF = 0, EVEN, ODD };

// CQ prefix variants.
enum class CqType { CQ, CQSOTA, CQPOTA, CQQRP, CQFD, CQFREETEXT };

// How the TX audio offset is chosen for new QSOs.
enum class OffsetSrc { RANDOM, CURSOR, RX };

// Supported radios.
enum class RadioType { NONE, TRUSDX, QMX, KH1 };

// One entry in the band list (name + frequency in kHz).
struct BandItem {
    const char* name;
    int freq;  // kHz
};
