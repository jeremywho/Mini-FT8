#!/usr/bin/env python3
"""
FT8 audio synthesis reference implementation.

Generates 11525 Hz, 8-bit unsigned PCM audio from a 79-symbol FT8 tone array.
Used as the golden source for host_mock/test_ft8_synth.cpp.

Usage:
    ft8_synth_reference.py <tones_hex_79_bytes> <base_hz> <out_raw_pcm_path>

Where:
    tones_hex_79_bytes -- 158 hex chars (79 bytes), each byte in 0..7
    base_hz            -- audio center frequency, e.g. 1500
    out_raw_pcm_path   -- raw u8 PCM written here, FT8_TX_SYNTH_SAMPLES bytes
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
