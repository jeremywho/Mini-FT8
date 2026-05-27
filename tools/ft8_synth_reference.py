#!/usr/bin/env python3
"""
FT8 audio synthesis reference implementation.

Generates 11525 Hz, 8-bit unsigned PCM audio from a 79-symbol FT8 tone array.
Used as the golden source for host_mock/test_ft8_synth.cpp.

Requires: numpy (for float32 phase accumulation matching C's float behavior).

Usage:
    ft8_synth_reference.py <tones_hex_79_bytes> <base_hz> <out_raw_pcm_path>

Where:
    tones_hex_79_bytes -- 158 hex chars (79 bytes), each byte in 0..7
    base_hz            -- audio center frequency, e.g. 1500
    out_raw_pcm_path   -- raw u8 PCM written here, FT8_TX_SYNTH_SAMPLES bytes
"""
import ctypes
import platform
import sys

import numpy as np

SAMPLE_RATE = 11525
SYMBOLS = 79
SPS = 1844              # round(11525 * 0.160)
TOTAL_SAMPLES = SPS * SYMBOLS
TONE_SPACING = 6.25
PEAK_EXCURSION = 89     # ~70% of full 8-bit range from midpoint (128 +/- 89)
MID = 128

# Call the C runtime's sinf directly because numpy.sin(float32) is not
# bit-identical to MSVC/glibc sinf, and byte-exact parity with the C
# synthesiser requires matching the actual libm the C compiler linked
# against. NB: parity may still break across compiler+platform combinations
# (Windows MSVC sinf != Linux glibc sinf); Windows is the dev platform.
_LIBM_NAME = {
    'Windows': 'msvcrt',
    'Linux':   'libm.so.6',
    'Darwin':  'libm.dylib',
}.get(platform.system(), 'libm.so.6')
_libm = ctypes.CDLL(_LIBM_NAME)
_libm.sinf.restype = ctypes.c_float
_libm.sinf.argtypes = [ctypes.c_float]


def _sinf(x):
    """Single-precision sine exactly matching C sinf()."""
    return np.float32(_libm.sinf(ctypes.c_float(float(x))))


def synth_ft8(tones, base_hz):
    """Continuous-phase FSK, hard tone edges (no smoothing).

    All arithmetic uses numpy.float32 to match the C implementation which
    uses float / sinf throughout.  Phase accumulation with numpy.float32
    stays bit-exact with C because numpy operations stay in float32 without
    promoting to float64.

    Round-half-away-from-zero matches C's (int)(scaled + 0.5f) convention.
    """
    assert len(tones) == SYMBOLS
    out = bytearray(TOTAL_SAMPLES)
    phase = np.float32(0.0)
    idx = 0
    fs = np.float32(SAMPLE_RATE)
    two_pi = np.float32(2.0 * np.pi)
    half = np.float32(0.5)
    peak = np.float32(PEAK_EXCURSION)
    for sym in range(SYMBOLS):
        f = np.float32(base_hz) + np.float32(tones[sym]) * np.float32(TONE_SPACING)
        dphi = np.float32(2.0) * np.float32(np.pi) * f / fs
        for _ in range(SPS):
            s = _sinf(phase)
            scaled = s * peak
            # Round-half-away-from-zero (matches C sample_to_u8)
            if scaled >= np.float32(0.0):
                v = int(float(scaled + half))
            else:
                v = int(float(scaled - half))
            b = MID + v
            if b < 0:
                b = 0
            if b > 255:
                b = 255
            out[idx] = b
            idx += 1
            phase += dphi
            if phase > two_pi:
                phase -= two_pi
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
