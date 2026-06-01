# truSDX Connect Issue — device attached at boot won't enumerate

**Status:** OPEN — root cause understood (a `NEW_DEV` enumeration race); the most principled
fix (root-port power sequencing) was tried on hardware and **failed**; one untried idea remains
(`usb_host_device_addr_list_fill()`); a 100%-reliable workaround exists (hot-plug, case 3).
**Last updated:** 2026-06-01
**Scope:** truSDX RX over USB host on the M5 Cardputer ADV (`trusdx-rx` branch). Affects
the USB-host connect path only — decode, GPS, resampling all work once connected.

---

## TL;DR

If the **truSDX is already plugged in when the Cardputer boots**, the connect fails and
`S → 2` does nothing. The cause is an **ESP-IDF USB-host enumeration race**: the driver only
opens a device in response to a `USB_HOST_CLIENT_EVENT_NEW_DEV` event, and that event is
**unreliably delivered (often lost)** for a device already attached when the host installs —
the daemon can enumerate and fire `NEW_DEV` before our client is registered to receive it.
Nothing in the code path opens a present-but-never-announced device.

**Reliable workaround:** boot the Cardputer **with the cable unplugged**, let it come up,
**then plug in the truSDX**, then `S → 2`. The physical plug-in is a clean connect transition
that reliably fires the event the whole open path depends on.

---

## Symptoms (observed on hardware)

| # | Sequence | Result |
|---|----------|--------|
| 1 | Radio on → connect cable → **then** boot Cardputer | `S` shows "Connect to truSDX"; pressing `2` does nothing, repeatedly |
| 2 | Cable attached at boot (variants of #1) | Same — `S → 2` is a no-op |
| 3 | Boot Cardputer **unplugged** → **then** plug in cable → `S → 2` | Radio clicks, "Sync to truSDX", waterfall + decodes ✅ |

Diagnostic readout in the dead state (from the on-screen `st bi tot` / `r a o` HUD):
- A failed boot-with-cable connect that *did* limp through CAT once showed `st5 bi0 tot459`
  then **frozen** (459 = the CAT handshake reply bytes; nothing after), `r0 a0 o0`.
- The fully-dead case shows "Connect to truSDX" (never even reached streaming state).

The deciding variable is **USB enumeration order**: the truSDX must appear *after* the
Cardputer's USB host is running.

---

## Root cause (traced in code)

Files: `components/ch340_usb_serial/ch340_usb_serial.cpp`

1. The driver learns about a device **only** through the client event callback:
   ```cpp
   // clientEventCb()
   case USB_HOST_CLIENT_EVENT_NEW_DEV:
       self->dev_addr_ = event_msg->new_dev.address;
       self->device_connected_ = true;     // <-- the ONLY place this is set true
       ...
   ```
   `device_connected_` is set true in exactly one place: the `NEW_DEV` handler.

2. The device is opened **only** when that flag is set, on the next `poll()`:
   ```cpp
   // poll()
   if (device_connected_ && dev_hdl_ == nullptr && !device_gone_pending_) {
       openAndProbeDevice();            // -> usb_host_device_open() -> claim -> init -> Ready
   }
   ```

3. `trusdx_serial_start()` → `wait_until_ready()` spins until `state_ == Ready`. Ready is
   only reached via step 2, which only runs via step 1.

**The gap:** the whole open path hinges on receiving `NEW_DEV`. If that event does not reach
our client, `device_connected_` stays false → `openAndProbeDevice()` never runs →
`usb_host_device_open()` is never called → `Ready` is never reached → connect times out. And
there is **no code path that opens a device by address without first having received the
event** (this is the gap a future `usb_host_device_addr_list_fill()` fix would close).

Case 3 works because plugging in *after* the host is up is a clean connect transition → the
event fires → the chain completes.

### Refined root cause: it's a RACE, not a hard wall (corrected 2026-06-01)
The original framing here was "`NEW_DEV` never fires for a device present at boot." That is
**too absolute, and our own diagnostics disprove it**: a boot-with-cable run was captured at
`st5` (Ready) with `tot459`. Reaching `st5` is *impossible* without `NEW_DEV` having fired
(it's the only thing that sets `device_connected_`). So `NEW_DEV` **does** fire at boot — just
**not reliably**. The real mechanism is a timing race in `begin()`:

`usb_host_install()` powers the root port → creates the daemon task (which drives enumeration)
→ **then** registers the client. The daemon is running before the client exists. With a device
already on the powered port, the daemon can enumerate it and fire `NEW_DEV` **before the client
is registered**, and ESP-IDF does not replay `NEW_DEV` to a late-registering client → the event
is lost. Sometimes the timing lands such that the client is registered first and it works; mostly
it doesn't. This race is what attempt #5 (root-port power sequencing) tried to eliminate.

**Also note two DISTINCT bugs were originally conflated here:**
- **Bug A (this doc's headline):** "Connect to truSDX", device never opens — the `NEW_DEV`
  race above.
- **Bug B (separate, streaming):** device *does* open (`st5`) but then `tot` freezes (e.g.
  `st5 bi0 tot459`) — the CH340 init's DTR assertion (`modem dtr/rts`, ch340_usb_serial.cpp:39)
  resets the truSDX ATmega and the `UA1;` streaming command is lost during the reboot. This is
  a connect-sequence/timing fix in `audio_trusdx_serial.cpp`, unrelated to enumeration. The 4 s
  settle (attempt #1) targets Bug B and helps the warm case.

This matches ESP-IDF's documented behavior: the USB Host lifecycle assumes devices connect
**after** the host library is installed; pre-connected-at-boot is not robustly covered.

---

## What we tried (and why each failed)

All attempts below are **uncommitted experiments** on top of commit `cb68a85`. None fixed the
boot-with-cable case.

1. **4 s rig-reboot settle + `;` resync + ID retries** (`audio_trusdx_serial.cpp`).
   Rationale: the CH340 init asserts DTR, which is wired to the truSDX ATmega RESET, so
   connecting reboots the rig; the PC capture script waits 4 s for exactly this.
   Result: helps the *warm* "first connect after a power-cycle is flaky" case, but does
   **nothing** for boot-with-cable — the problem there is upstream of CAT (the device never
   opens at all), so a settle delay on a device that never enumerated changes nothing.

2. **Force stop+start on `S → 2`** (`main.cpp`).
   Rationale: a stale `s_started` / `s_streaming` flag made `begin_usb_host_mode()` no-op.
   Result: clears the stale guard, but the subsequent re-`begin()` still can't enumerate an
   already-attached device (no `NEW_DEV`), so `S → 2` still can't recover boot-with-cable.
   Side effect: the teardown+retry made `S → 2` **block the UI for ~15 s** (5 s stop-wait +
   4 × 2.5 s enum attempts + 4 s settle), which looks like "flashes and does nothing".

3. **Enumeration retry: uninstall + reinstall the host up to 4×** (`audio_trusdx_serial.cpp`).
   Rationale: ESP-IDF issues (#12412, #17918) say the documented remedy for enumeration
   trouble is app-level retry / reinstall.
   Result: did **not** work. Reinstalling the host within the same boot does not re-scan and
   re-announce a device that is sitting there already attached — still no `NEW_DEV`.

4. **(Researched, not tried) ESP-IDF USB-host timing Kconfig knobs**
   (`CONFIG_USB_HOST_DEBOUNCE_DELAY_MS`, `_RESET_HOLD_MS`, `_RESET_RECOVERY_MS`,
   `_SET_ADDR_RECOVERY_MS`). Issue #12412's reporter confirmed these do **not** fix the
   device-present-at-boot case. Skipped as a known dead end.

5. **Root-port power sequencing — register client FIRST, then power the port** (the most
   principled attempt; `components/ch340_usb_serial/ch340_usb_serial.cpp`).

   **What we thought would work:** This was framed as a *race*, not a hard wall (see the
   "Refined root cause" note below). In `begin()` the original order is:
   `usb_host_install()` (root port **powered** by default) → create daemon task → register
   client. The daemon drives enumeration. So with a device already on a powered port, the
   daemon can enumerate it and fire `NEW_DEV` **before our client is registered** → the event
   is delivered to zero clients and lost (ESP-IDF does not replay `NEW_DEV` to a
   late-registering client). The proposed fix removes the timing nondeterminism *by
   construction*: install with the root port **unpowered**, register the client, **then**
   power the port on, so no connection can be detected until a client is already listening.

   **API basis (verified in this repo's IDF 5.4 headers, `components/usb/include/usb/usb_host.h`):**
   - `usb_host_config_t::root_port_unpowered` (usb_host.h:108) — "the USB Host Library will
     not power on the root port on installation."
   - `usb_host_lib_set_root_port_power(bool)` (usb_host.h:244) — header note: "users must call
     this function to power ON the root port before any device connections can occur."

   **What we did:** set `host_config.root_port_unpowered = true` at install, and added
   `usb_host_lib_set_root_port_power(true)` immediately after `usb_host_client_register()`
   succeeds. Built and flashed.

   **Result: DID NOT WORK.** Boot-with-cable still does not auto-connect, and `S → 2` still
   does not recover it. Confirmed on hardware 2026-06-01.

   **Why it most likely failed (best explanation):** `set_root_port_power(true)` toggles the
   hub driver's **logical** port power. On a real external USB hub, powering a port triggers
   connection detection even for an already-attached device. But the ESP32-S3 **internal
   root-port PHY** appears to detect connections on an **electrical edge** (a D+ transition —
   what physically plugging in produces), not on logical power state. A device that is already
   electrically present at boot (M5's Cap VBUS is always-on) never produces that edge, so
   toggling logical power detects nothing → no enumeration → no `NEW_DEV`. This is exactly the
   "works regardless of M5's always-on VBUS — sound but unverified on this board" caveat that
   was flagged before the attempt; the hardware did not cooperate. Note this does **not**
   regress the working hot-plug path: the port ends up powered, so plugging in afterward still
   produces a real edge → `NEW_DEV` → opens.

---

## On-screen diagnostics (useful, keep)

While streaming, the STATUS screen (`S`) shows (gated to truSDX + streaming):
- **`st# bi# tot#`** — CH340 driver state (`5`=Ready), last bulk-IN status (`0`=OK),
  cumulative USB bytes received. `tot` frozen = stream dead; `tot` climbing = healthy.
- **`r# a# o#`** — stream task: raw bytes / bytes-to-resampler / output samples per second.

These are how we localized the bug (`st5 bi0 tot459`-frozen proved CAT-worked-but-no-audio,
and "Connect to truSDX" proved the device never opened at all).

---

## Candidate real fixes

1. **Poll for already-present devices via `usb_host_device_addr_list_fill()` — THE REMAINING
   UNTRIED IDEA, and the most promising.** Confirmed present in this repo's IDF 5.4 headers
   (`components/usb/include/usb/usb_host.h:381`). This is **event-independent** — it does not
   rely on `NEW_DEV` at all, which is exactly why it could beat both the race (#5) and the
   logical-power dead end. Approach: in `poll()`, when `!device_connected_ && dev_hdl_ ==
   nullptr`, call `usb_host_device_addr_list_fill()`; if it returns ≥1 address, adopt
   `list[0]` as `dev_addr_`, set `device_connected_ = true`, and let the existing
   `openAndProbeDevice()` path run. Instead of *waiting* for the device to announce itself, we
   actively *scan* for the device that is already there.
   **Risk / chicken-and-egg:** if the device is absent from the stack's list precisely
   *because* it was never enumerated (no edge → never added), the scan returns empty too. So
   this might also fail. It is the best remaining shot but is **not** a sure thing, and it is
   still a blind iteration (see meta-blocker below).

2. **Accept the hot-plug workflow (case 3)** as the supported procedure and document it. The
   device is 100% reliable when booted unplugged then connected. This is the current
   recommendation.

### Tried and ruled out
- **Root-port power sequencing (attempt #5 above)** — logical port power doesn't create the
  electrical edge the S3 root-port PHY needs. Failed on hardware.
- **Host uninstall/reinstall retry (attempt #3)** — reinstalling doesn't re-scan an
  already-attached device.
- **Timing Kconfig knobs (attempt #4)** — confirmed not to fix present-at-boot (issue #12412).

### Meta-blocker: debugging this is BLIND
This firmware is USB-host, so the ESP32-S3's native USB is **not** a serial console — the
`CH340_LOG(...)` lines (which would show whether `NEW_DEV` fired, whether
`set_root_port_power` returned OK, where enumeration stalls) **cannot be read**. The on-screen
`st/bi/tot` HUD only updates *after* a connect attempt completes; it can't show the enumeration
sequence itself. So every hypothesis costs a full build + flash + the download-mode dance +
a hardware boot test, with no visibility into the actual failure point. This — more than any
single bug — is why Bug A has resisted a fix. A real next step would be to get log visibility
first (e.g. route `CH340_LOG` to the on-screen debug list, or temporarily build a
USB-CDC-console variant) before throwing more blind iterations at it.

---

## Recommended startup procedure (until a real fix lands)

1. Power on the Cardputer **with the truSDX cable unplugged**.
2. Let it fully boot (splash → RX screen).
3. Plug the truSDX USB-C cable in.
4. `S → 2` to connect. Status should go "Sync to truSDX" and the waterfall should appear.

If a connect ever comes up dead, unplug the cable and repeat from step 3 (re-plugging is what
generates the enumeration event).

---

## Build / flash (for whoever picks this up)

Standard **ESP-IDF** project (`idf.py`, target **esp32s3**), IDF **v5.4.x** at `C:\esp\esp-idf`.
Full details in `CLAUDE.md`. Quick version (PowerShell, from this worktree):
```powershell
. C:\esp\esp-idf\export.ps1
idf.py -DENABLE_BLE=ON build
# download mode: hold BOOT/G0, tap Reset (~2s), release -> COM8 appears
idf.py -DENABLE_BLE=ON -p COM8 flash
# then tap plain Reset (no BOOT) to boot
```
The on-screen `st/bi/tot` + `r/a/o` diagnostics referenced above are compiled in by the
current (uncommitted) experiments; they live in `audio_trusdx_serial.cpp` / `main.cpp`.

## Key files

| File | Relevance |
|------|-----------|
| `components/ch340_usb_serial/ch340_usb_serial.cpp` | USB host driver; `clientEventCb` (NEW_DEV), `poll()` (open trigger), `openAndProbeDevice()` |
| `main/audio_trusdx_serial.cpp` | `trusdx_serial_start()` connect sequence, `wait_until_ready()` |
| `main/main.cpp` | `begin_usb_host_mode()` (boot auto-connect + `S → 2` handler) |

## References
- ESP-IDF issue #12412 — "Root port reset failed on slow starting USB device" (timing knobs don't fix present-at-boot)
- ESP-IDF issue #17918 — ESP32-S3 USB host enumeration timing (GET_FULL_CONFIG_DESC delay)
- ESP-IDF USB Host docs — lifecycle assumes devices connect *after* host install; pre-connected case not covered
