# truSDX Display Issue — wattmeter/callsign line not restored after CAT TX

**Status:** OPEN (cosmetic). Root cause identified; **NOT fixable from the Cardputer** —
needs a truSDX firmware change.
**Last updated:** 2026-06-07
**Scope:** truSDX driven over CAT streaming (the TX path). Display-only — RX and TX
function are correct. Found while validating the TX `;US`-crash fix (see git log / TRUSDX_FACTS.md).

---

## Symptom

After Mini-FT8 keys the truSDX over CAT streaming (`UA1;TX0;` -> 8-bit audio -> `RX;`)
and returns to RX:

- The rig **is** back in RX — the S-meter is live, RX audio streaming resumes, it is not
  transmitting.
- BUT the bottom line that normally shows the operator **callsign** keeps showing the
  **TX wattmeter** readout from the last transmission; it never repaints to the callsign.
- Pressing & releasing the **physical PTT button** immediately restores the callsign line.

Cosmetic only. The radio is functionally correct; just stale on-screen text.

## Root cause (from the uSDX/QCX-SSB source, `usdx.ino`)

- The RX-side display restore is **`show_banner()`** (redraws the top/callsign line). It is
  called only from inside **`switch_rxtx(0)`** (gated `#ifdef SWR_METER` + runtime
  `swrmeter > 0`, which is on by default), plus menu-exit and boot.
- The physical PTT button reaches that path on release. The truSDX R2.00x **CAT-streaming-TX
  exit (`TX0;`...`RX;`) returns to RX electrically but does not reach the bottom-line repaint.**
- Frequency/mode CAT commands only set `change=true` -> `display_vfo()` repaints the freq/mode
  line, NOT the callsign/banner line. (That is why `FA`/`MD`/`UA0` do nothing here.)

## What was tested on the real rig — all FAILED to restore the line

| Attempt | Result |
|---|---|
| `MD2;` (re-set mode) | no (no-op: not a change, no repaint) |
| `UA0;` (stop streaming) | no (does not call `switch_rxtx(0)`) |
| `FA` real freq change (14.074 -> 14.075 -> back) | no |
| `MD` real toggle (LSB -> USB) | no |
| `UA0;RX;` (stop stream, then RX repaint) | no |
| `UK20;/UK00;` (emulate the PTT over CAT) | **unavailable** — `UD;` -> `?;` proves `CAT_EXT` (`UK`/`UD`) is not compiled |
| **RTS hardware-PTT pulse** (RTS HIGH keys CW/PTT) | RTS **does** key the rig (RX drops to ~4 B/s) but its release is handled like CAT-PTT — no repaint |

CAT is definitely processed after key-off (repeated `TX0;`/`RX;` cycles work). The **only**
thing that restores the line is the physical PTT button.

## Why there is no host-side fix

The official truSDX CAT command set (dl2man.de/5-trusdx-details) is the TS-480 subset:
`FA IF ID PS AI MD RX TX TX0 TX1 TX2 AG0 XT1 RT1 RC FL0 RS VX UA0/1/2`.
Among these, only `RX;` reaches `switch_rxtx(0)` -> `show_banner()`, and that does not repaint
the bottom line in the CAT-streaming-TX context. There is **no remote-key/display command**
(`UK`/`UD`, the `CAT_EXT` feature) compiled into this firmware, so the physical-PTT path cannot
be emulated over CAT — and the RTS hardware-PTT line does not run the manual-RX restore either.

## Path forward (rig firmware, not the Cardputer)

1. **Try a newer truSDX beta firmware.** dl2man.de/3b-trusdx-firmware ships beta with a version
   history — this may already be fixed (or fixed in a future build). Flash via AVRDUDESS. NOTE:
   flashing erases calibration; write down + re-enter PA bias (8.2), Ref Freq (8.3), Rshunt
   (8.6), LPF (8.7) first.
2. **Report it to the DL2MAN forum.** Concrete bug report:
   > With CAT streaming, returning to RX via `RX;` (and via the RTS PTT line) does not repaint
   > the callsign/banner line the way the physical PTT release does — the wattmeter readout from
   > the last TX lingers on screen. Please run the RX display restore (`show_banner()`) on the
   > CAT/streaming RX exit.

## References / repro tooling (on the PC)

- Firmware source pulled for analysis: `C:\Users\jerem\usdx.ino` (threeme3/usdx, the
  uSDX/QCX-SSB base): `switch_rxtx()`, `show_banner()`, `Command_RX()`, and the `CAT_EXT`
  `UK`/`UD` docs/handlers.
- Display test harnesses (drive the truSDX on COM14 from the PC, watch the OLED):
  `C:\Users\jerem\trusdx_unkey_display.py` (one CAT candidate per run),
  `trusdx_rts_clean.py` / `trusdx_rts_ptt.py` (RTS hardware-PTT), and
  `trusdx_display_probe.py` (the `UD;` read attempt that returned `?;`).
- Investigated 2026-06-07 using the firmware source + a Codex second opinion + the official
  DL2MAN docs (7 on-rig tests).
