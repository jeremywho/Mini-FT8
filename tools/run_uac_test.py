#!/usr/bin/env python3
"""Build, flash, and capture UART log for the UAC OUT test bench.

Usage:
    python3 tools/run_uac_test.py [capture_seconds]

Default capture window is 12 seconds. Output is the captured log to
stdout, with a summary footer showing whether the verifier reported
any sample errors.

Ports (hard-coded for the bench rig):
    flash : /dev/cu.usbmodem5A7A0113341  (USB JTAG, ESP32-S3 PROG)
    uart  : /dev/cu.usbserial-A50285BI   (separate USB-UART, 115200)
"""

import os
import re
import subprocess
import sys
import time

import serial

PROG_PORT = "/dev/cu.usbmodem5A7A0113341"
UART_PORT = "/dev/cu.usbserial-A50285BI"
UART_BAUD = 115200
DEFAULT_CAPTURE_S = 12

ROOT = os.path.expanduser("~/Mini-FT8")


def run(cmd, **kw):
    """Run a shell command, stream stdout/stderr live."""
    print(f"$ {cmd}", flush=True)
    return subprocess.run(cmd, shell=True, cwd=ROOT, **kw).returncode


def open_uart():
    """Open the UART port at UART_BAUD via pyserial."""
    return serial.Serial(UART_PORT, UART_BAUD, timeout=0.1)


def capture(ser, seconds, keys_after_s=None, keys=None):
    """Capture UART for `seconds`. If `keys` is provided, after
    `keys_after_s` has elapsed write each character to the serial port
    with a short delay between them so the firmware menu has time to
    process each press."""
    deadline = time.time() + seconds
    keys_deadline = (time.time() + keys_after_s) if keys_after_s else None
    keys_sent = False
    buf = bytearray()
    while time.time() < deadline:
        chunk = ser.read(4096)
        if chunk:
            buf.extend(chunk)
            sys.stdout.write(chunk.decode("utf-8", errors="replace"))
            sys.stdout.flush()
        if keys and not keys_sent and keys_deadline and time.time() >= keys_deadline:
            print(f"\n[harness] sending keys: {keys!r}\n", flush=True)
            for ch in keys:
                ser.write(ch.encode("ascii"))
                ser.flush()
                time.sleep(0.4)  # let firmware process each press
            keys_sent = True
    return buf.decode("utf-8", errors="replace")


def summarize(log: str):
    print("\n" + "=" * 60)
    print("SUMMARY")
    print("=" * 60)
    patterns = {
        "Speaker captured":     r"Speaker captured: addr=(\d+) iface=(\d+)",
        "Mic captured":         r"USB dev attached:.*PID:0x([0-9a-fA-F]+)",
        "TX test started":      r"uac_tx_test: streaming",
        "TX test stopped":      r"uac_tx_test: stopped, packets=(\d+) errors=(\d+)",
        "Verify checked":       r"VERIFY:.*checked=(\d+).*errors=(\d+)",
        "Stream format":        r"Selected stream format: (\d+)Hz, (\d+)-bit, (\d+)ch",
        "EP Alloc errs":        r"EP Alloc error",
    }
    for label, pat in patterns.items():
        matches = re.findall(pat, log)
        if matches:
            print(f"  {label:18}: {matches[-1] if not isinstance(matches[-1], tuple) else matches[-1]}  (count={len(matches)})")
        else:
            print(f"  {label:18}: NOT SEEN")


def main():
    capture_s = int(sys.argv[1]) if len(sys.argv) > 1 else DEFAULT_CAPTURE_S

    # Source IDF + build/flash
    idf_export = "source ~/esp/esp-idf/export.sh > /dev/null 2>&1"
    rc = run(f"{idf_export} && idf.py build")
    if rc != 0:
        print("Build failed", file=sys.stderr)
        sys.exit(rc)

    # Open UART BEFORE flash so we catch the boot.
    print(f"Opening UART {UART_PORT} @ {UART_BAUD}")
    ser = open_uart()

    rc = run(f"{idf_export} && idf.py -p {PROG_PORT} flash")
    if rc != 0:
        print("Flash failed", file=sys.stderr)
        ser.close()
        sys.exit(rc)

    print(f"\n--- capturing UART for {capture_s}s ---\n")
    # After 8s (UAC enum + QMX sync done), send 'S 1 S' to enable
    # beacon EVEN. This lets the harness drive an actual TX cycle
    # without manual interaction.
    log = capture(ser, capture_s, keys_after_s=8.0, keys="S1S")
    ser.close()

    summarize(log)


if __name__ == "__main__":
    main()
