# truSDX + ESP-IDF USB-host — verified facts

Ground-truth reference for this project, established 2026-06 by: on-device byte capture
(`rawrx` boot-trace), a deep multi-source web-research pass (adversarially verified), a
Codex code review, and the local ESP-IDF source/docs. Supersedes several earlier
assumptions that turned out wrong (flagged below). Cite this instead of re-guessing.

## 1. truSDX CAT audio-streaming protocol  (HIGH confidence)
Sources: [dl2man.de/5-trusdx-details](https://dl2man.de/5-trusdx-details/),
[dl2man.de/4-trusdx-manual](https://dl2man.de/4-trusdx-manual/),
[threeme3/usdx `usdx.ino`](https://github.com/threeme3/usdx/blob/master/usdx.ino),
[olgierd/trusdx-audio](https://github.com/olgierd/trusdx-audio).

- **Link:** 115200 8N1, no flow control (firmware 2.00t+ requires 115200, not 38400).
  Audio-over-CAT needs firmware 2.00u+. CAT = Kenwood TS-480 subset.
- **`UA0;` / `UA1;` / `UA2;` = streaming OFF / ON+speaker-ON / ON+speaker-OFF.** All three
  share *identical* framing; UA1 vs UA2 differ only in the local speaker.
  ⚠️ The old project note "UA1 = raw 8-bit / UA2 = multiplexed-framed" was **WRONG**.
- **RX audio IS framed** (not a raw pipe): the rig sends `US` (`0x55 0x53`) then a span of
  audio samples, terminated by `;`. Samples are **unsigned 8-bit = pcm + 128**; a sample
  equal to `0x3B` (`;`) is **escaped by incrementing to `0x3C`** so `;` never appears mid-audio.
- **Measured on the wire** (our `rawrx` capture): `55 41 31 3b 55 53 <audio…>` = `UA1;` echo,
  then `US`, then a **long continuous audio span (40+ bytes, no `;`)**. Because spans are long,
  the legacy `pure_audio` "every byte is a sample" parser only injects the one-time 6-byte
  `UA1;US` prefix as noise → **a proper `US…;` de-framer is a ~no-op for FT8 decode.**
- **Mid-stream CAT is handled by the firmware:** on each received `;` it auto-pauses the
  stream, runs the CAT command, then resumes a fresh `US` span if the durable streaming flag
  (set by `UA1`) is still set. **You do NOT need to send `RX;` to pause for a query.** `RX;`
  is only a TX→RX switch. (Refutes the old "set CAT before UA1 / pause to do CAT" belief.)
- **Dead pipe** (`UA1;` accepted, `ID;` still replies, but no audio): there is **no documented
  mode/RX/booted/order gate** — emission is gated only by the live streaming flag. Most likely
  cause is `UA1;` arriving before the ATmega finished its post-DTR-reset boot. **Correct
  recovery: re-assert `UA1;`** (it unconditionally re-arms the flag). The "wait a fixed 3 s
  after open" claim is **not** a documented requirement (refuted), though DTR-on-open does
  reset the ATmega328 on this rig (confirmed hardware behavior).

## 2. ESP-IDF v5.4 USB Host (ESP32-S3)  (HIGH confidence)
Sources: local ESP-IDF docs/source; esp-idf issues
[#14319](https://github.com/espressif/esp-idf/issues/14319),
[#12412](https://github.com/espressif/esp-idf/issues/12412),
[#14244](https://github.com/espressif/esp-idf/issues/14244).

- **Documented client teardown order = `usb_host_endpoint_halt` + `usb_host_endpoint_flush`
  → `usb_host_interface_release` → `usb_host_device_close`.** Skipping halt/flush makes
  `interface_release` return `ESP_ERR_INVALID_STATE`, leaving the interface "claimed" so the
  next `interface_claim` also fails `INVALID_STATE`. (This was the custom driver's reconnect wedge.)
- **Adopt-by-address is supported:** check `usb_host_device_addr_list_fill` first, only wait for
  `NEW_DEV` if not already present.
- **Attached-at-boot enumeration is achievable in software** with the DEFAULT (root-port-powered-
  at-install) config — no physical replug edge is fundamentally required (#14319). BUT it is
  **timing-fragile for slow-starting / multi-interface devices** (#12412, #14244), which may need
  a replug; there is no documented software fix. `root_port_unpowered` is the opt-in path, **not**
  the reliable one (it was this project's failed "attempt #5").
- **`known=0` (device not enumerated) vs the dead pipe are SEPARATE layers:** the CH340 is a
  USB-VBUS-powered bridge that enumerates independently of the ATmega; a `known=0` failure is
  host/USB timing, not the rig's boot state.

## 3. Decision — use the MAINTAINED driver
There is an official, maintained Espressif driver for our exact chip:
`espressif/usb_host_ch34x_vcp` + `espressif/usb_host_cdc_acm`. A standalone spike
(`main/vcp_spike.cpp` + a `-DVCP_SPIKE=ON` build switch, both since removed) proved, on hardware,
that it:
- **opens reliably** (0 open failures over many cycles),
- **sustains** the `UA1;` audio stream at the full ~7,900 B/s for 30 s, repeatedly (the custom
  driver's "comes up briefly then stops" did NOT occur), and
- **reconnects in place** (close + reopen) cleanly — the exact path that wedged the custom driver.

Residual intermittent issues (occasional dead pipe / disconnect) are rig/protocol-side and
recover on reconnect. **Conclusion: the hand-rolled `ch340_usb_serial` USB-host state machine was
the root of the open/reconnect/stream-death failures; migrate to the maintained driver.**

`components/ch340_usb_serial/ch340_usb_serial.cpp` was rewritten to back the *same*
`Ch340UsbSerial` interface with the maintained `cdc_acm_host` + `ch34x_vcp_open` **C API** (no
app-wide C++ exceptions), so `audio_trusdx_serial.cpp` (CAT sequence, `UA1;` retry, resample →
FFT → decode → waterfall) is unchanged. DTR is held LOW (no rig reset). RX is delivered by a
data callback into an SPSC ring that `readBytes()` drains.

> Status (2026-06-04): migration **committed** (`e250f06`) and stripped of the spike scaffolding;
> builds clean, flashed, on-device smoke test passed (clean S-menu, S→3 retunes). Still pending a
> long sustained-streaming + reconnect soak on the real rig (the truSDX was on the PC for the
> decode-conditions validation below).

## 4. Decode rate is conditions-limited, not a firmware bug  (HIGH confidence, 2026-06-04)
The Cardputer's "≤1 FT8 decode per 15 s slot" indoors is **RF conditions**, proven against a
gold-standard reference:
- Captured UTC-aligned slots off the truSDX on the PC (`C:\Users\jerem\trusdx_capture.py`:
  pyserial → de-frame `US…;` → resample 7825→12000 → WAV) and decoded with **Ft8DotNet**
  (`C:\Data\Repos\ft8-may-2026`, WSJT-X parity) → **0 real decodes** (2 garbage CRC
  false-positives), the *same* weak Costas candidates (top sync score ~11) the Cardputer's
  monitor sees.
- A WSJT-X-parity decoder pulling nothing off the same antenna ⇒ no decodable signal in the air
  (indoor / dead band). The Cardputer decoder performs comparably to the reference — it is fine.
- Ruled out as causes (none changed the count): capture-window clipping (79→92 blocks made it
  *worse* — slot skips), sample rate (measured 7814 ≈ assumed 7825), per-block AGC (removed, no
  change), waterfall scaling (p95 ~130, clip 0), GPS/timing.
- ⚠️ The earlier worry "the firmware only decodes 1/cycle" was a **misdiagnosis** — it's the band.
  For decodes: real antenna + open band + correct frequency (14.074 etc.).

**Separate, real robustness issues** (found here, deferred — NOT decode-count related): the
~1.66 s synchronous decode blocks the audio task → the 8 KB CH340 RX ring overflows (~13 KB/slot
dropped, a harmless dead-zone — `droppedRxBytes()` counts it); free internal heap ~8 KB (NimBLE +
USB-host + 80 KB static waterfall; the StampS3 = ESP32-S3FN8, **no PSRAM**). Fix later = decouple
decode from audio capture + claw back DRAM.
