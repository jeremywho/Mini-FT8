# truSDX Connect Issue — device attached at boot won't enumerate

> **UPDATE 2026-06-09 — LIKELY RESOLVED.** The root cause below (the hand-rolled USB-host
> driver failing to open/reconnect on a reused host) was eliminated by migrating to the
> maintained Espressif `cdc_acm_host` + `ch34x_vcp` driver (commit `e250f06`, 2026-06-04),
> which the spike proved reconnects in place cleanly. Pending a sustained reconnect **soak on
> the real rig** (reliability plan item C1) to close it definitively. Original investigation
> kept below for history.

**Status (historical, 2026-06-01):** OPEN — but the boot-trace (2026-06-01) **changed the diagnosis**. The real problem
is **RECONNECT**, not boot-with-cable: the *first* connect of a boot works (even cable-attached);
every subsequent connect (after a teardown via `S → 2`) fails. We have peeled back three layers
with on-device logging — host-reuse fixed, address-adopt fixed — and are now stuck at the device
**open/probe failing on a reused host** (driver lands in Error `st=6`). Reliable workaround:
power-cycle the Cardputer (not just the radio) to get a clean first connect.
**Last updated:** 2026-06-01 (boot-trace findings added)
**Scope:** truSDX RX over USB host on the M5 Cardputer ADV (`trusdx-rx` branch). Affects
the USB-host connect path only — decode, GPS, resampling all work once connected.

---

## TL;DR (updated after boot-trace)

The original theory below ("device attached at boot won't enumerate") was **only partly right**
and is **superseded** by the boot-trace evidence. What the on-device log actually shows:

- **The first connect of a boot SUCCEEDS** — including cable-attached-at-boot. `a1 READY
  vid=1a86 pid=7523`, audio flows. So boot-with-cable is NOT the core failure.
- **Every RECONNECT fails.** Once anything tears the connection down (`S → 2` while connected
  calls `audio_source_stop()` → `s_ch340.end()`), bringing it back up fails — and historically
  only a Cardputer power-cycle recovered. This is a **USB teardown/re-init** bug, not enumeration.
- We fixed two layers of it with evidence (host reuse, address adopt). The remaining failure is
  the device **open/probe** on a reused host. See "Boot-trace findings" below for the full chain.

**Reliable workaround:** power-cycle the **Cardputer** to force a clean first connect (the
first connect always works). Avoid `S → 2` reconnects until the open-on-reuse layer is fixed.

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

## Boot-trace findings (2026-06-01) — the real diagnosis

To stop debugging blind, we added an on-device boot/connect trace (`main/boot_trace.{h,cpp}`):
a bounded ring buffer written to `/storage/bootlog.txt`, every line prefixed `BTLOG|`. It is
pulled back **without an SD card or serial console** by reading the SPIFFS partition over the
download-mode USB connection:
```
esptool --chip esp32s3 -p COM8 -b 921600 read_flash 0x600000 0x200000 spiffs_dump.bin
strings -n 8 spiffs_dump.bin | grep "BTLOG|"
```
The trace logs, per connect attempt: `begin=` result, driver state (`st`, 5=Ready/6=Error),
`known=` (count from `usb_host_device_addr_list_fill()`, i.e. devices the stack sees regardless
of events), `newdev=` (did NEW_DEV fire / `device_connected_`), CAT result, and
`tot=`/`flowing=` (bytes received / audio actually flowing).

### What the trace proved
1. **First connect of a boot works, even cable-attached:**
   `a1 begin=ESP_OK st=1` → `a1 READY vid=1a86 pid=7523` → `connected OK tot=1807 (flowing=1)`.
   This **disproved the original "boot-with-cable never enumerates" headline.** Boot-with-cable
   is fine; the failures are all on reconnect.
2. **Reconnect originally wedged hard:** every reconnect attempt showed
   `a1 begin=ESP_ERR_INVALID_STATE st=6`. Root cause: `end()`'s `usb_host_uninstall()` was
   failing (it stops the daemon before device-free/uninstall finishes, so uninstall has
   "unfinished actions") yet the code cleared its installed-flag anyway → next
   `usb_host_install()` returned `INVALID_STATE` → Error. **FIXED** (see attempts #6/#7).
3. **`flowing=0` on some "successful" connects:** e.g. `connected OK tot=64 (flowing=0)` — the
   connect succeeds and CAT replies (`ID020;`), but `UA1;` doesn't start audio and only ~64
   bytes arrive. This is **Bug B** (DTR-reset eats `UA1;`), distinct from the reconnect bug.

### Layer-by-layer progress on the reconnect bug (each fix verified by the next trace)
| Fix | Trace before | Trace after | Verdict |
|-----|--------------|-------------|---------|
| **#6 `end()` pumps events + only clears flag on real uninstall success** | `begin=INVALID_STATE st=6` | still `INVALID_STATE` | Necessary but insufficient — flag lived in `impl_`, which `Ch340UsbSerial::end()` **deletes** (`delete impl_`), so it didn't survive the stop/start |
| **#7 move install-state to a process-scope static `g_idf_host_installed`** (survives `delete impl_`); `begin()` reuses an already-installed host | `begin=INVALID_STATE st=6` | `begin=ESP_OK`, `known=1` | **FIXED the wedge.** Reconnect's `begin` now succeeds and the stack reports the device present (`known=1`) — but `newdev=0`, so the device was never opened → TIMEOUT |
| **#8 adopt-from-address-list** in `poll()`: if no NEW_DEV but `addr_list_fill` returns a device, adopt `addrs[0]` and run `openAndProbeDevice()` | `known=1 newdev=0` (never opened) | `newdev=1` then `st=6` (Error) | **Adopt fires** (sets device_connected_ → newdev=1) but the **device open/probe FAILS on the reused host** → driver goes to Error. This is the current wall. |

### Current wall (where it's stuck now)
Reconnect trace, latest build:
```
79792  connect start
79799  a1 begin=ESP_OK st=1                 ← host reuse OK (#7)
80301  a1 after500: st=1 known=1 newdev=0   ← stack sees device, not yet adopted
82810  a1 TIMEOUT st=6 known=1 newdev=1      ← adopt fired (newdev=1), but open/probe failed -> st=6 Error
```
So the chain is now: **reuse host ✓ → see device ✓ → adopt address ✓ → `usb_host_device_open()`
(or the descriptor read / interface claim that follows) FAILS on a device that was previously
opened on this host and not cleanly released.** The likely cause is that the prior connection's
device handle / interface claim was not fully torn down (the same incomplete-teardown family as
#6), so re-opening the same address errors. Next probe: log the exact failing call inside
`openAndProbeDevice()` (open vs get_descriptor vs claimInterface) and whether
`usb_host_device_free_all()` actually freed the handle on the prior `end()`.

### Honest assessment
Each layer we fix exposes the next, all in the **teardown/re-init** family — the ESP-IDF USB
host does not cleanly support tear-down-and-reconnect-in-place within one boot for this driver.
The first connect per boot is reliable. A pragmatic option is to **stop trying to reconnect
in-place** and instead, on `S → 2`, do the one thing that always works: trigger a full restart
of the connect from a clean host (or document "power-cycle the Cardputer to reconnect"). The
"correct" fix is to make `end()` fully release the device handle + interface so a re-open
succeeds — which needs the per-call logging described above to pinpoint.

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

   **POSTSCRIPT (boot-trace):** the boot-trace later showed the first connect works fine anyway,
   so root-port sequencing was solving a problem that mostly wasn't there. The real bug is
   reconnect (#6–#8). Left in the build; harmless.

6. **`end()`: pump events to completion + only clear installed-flag on real uninstall success**
   (`ch340_usb_serial.cpp`). The teardown stopped the USB daemon task, then called
   `usb_host_device_free_all()` / `usb_host_uninstall()` — but those finish asynchronously and
   need `usb_host_lib_handle_events()` pumped (which only the daemon did). So uninstall failed
   with "unfinished actions," yet the code cleared its installed-flag anyway → next
   `usb_host_install()` → `INVALID_STATE` → wedge. Fix: pump events until `ALL_FREE` ourselves
   after stopping the daemon, and only clear the flag if uninstall returned `ESP_OK`.
   **Result: insufficient alone** — the flag lived in the per-`Impl` object, and
   `Ch340UsbSerial::end()` does `delete impl_`, so the flag didn't survive to the next
   `begin()`. Trace still showed `begin=INVALID_STATE st=6`.

7. **Move install-state to a process-scope static `g_idf_host_installed`** + `begin()` reuses an
   already-installed host (`ch340_usb_serial.cpp`). Because `delete impl_` destroys any member
   flag, the "is the IDF host installed" truth must live at process scope to survive a
   stop/start. `begin()` now skips `usb_host_install()` when `g_idf_host_installed` is true and
   reuses the host (resetting `device_connected_`/`dev_addr_`).
   **Result: FIXED THE WEDGE.** Trace flipped from `begin=INVALID_STATE st=6` to `begin=ESP_OK`
   **and `known=1`** (stack now reports the device present on the reused host). But `newdev=0`
   → device never opened → TIMEOUT. Exposed the next layer.

8. **Adopt-from-address-list in `poll()`** (`ch340_usb_serial.cpp`). Since a reused host doesn't
   fire a fresh `NEW_DEV` but `usb_host_device_addr_list_fill()` reports the device (`known=1`),
   adopt `addrs[0]` as `dev_addr_`, set `device_connected_`, and let the existing
   `openAndProbeDevice()` run — no event needed.
   **Result: adopt fires, but open/probe FAILS.** Trace now shows `newdev=1` (adopt set the
   flag) then `st=6` (Error) — `usb_host_device_open()` or the descriptor/claim that follows
   fails on a device previously opened on this host and not cleanly released. **This is the
   current wall** (see "Boot-trace findings → Current wall" above).

While streaming, the STATUS screen (`S`) shows (gated to truSDX + streaming):
- **`st# bi# tot#`** — CH340 driver state (`5`=Ready), last bulk-IN status (`0`=OK),
  cumulative USB bytes received. `tot` frozen = stream dead; `tot` climbing = healthy.
- **`r# a# o#`** — stream task: raw bytes / bytes-to-resampler / output samples per second.

These are how we localized the bug (`st5 bi0 tot459`-frozen proved CAT-worked-but-no-audio,
and "Connect to truSDX" proved the device never opened at all).

---

## Candidate real fixes (updated after boot-trace)

The bug is now isolated to **device open/probe failing on a reused host** (attempt #8 wall).
Both candidates below target that, with the next concrete diagnostic step first.

1. **NEXT PROBE — log exactly which call fails in `openAndProbeDevice()`.** We know adopt fires
   and then the driver hits Error (`st=6`). Add boot-trace lines inside `openAndProbeDevice()`
   for: `usb_host_device_open()` result, `usb_host_get_device_descriptor()` result, and
   `claimInterface()` result. Also log whether the prior `end()`'s `usb_host_device_free_all()`
   actually freed the handle (vs. still-referenced). This pinpoints whether the re-open fails
   because (a) the address is stale, (b) the device is still open/claimed from the prior
   connection, or (c) a descriptor/claim step errors. One flash, then pull the trace.

2. **Fully release the device on `end()` so re-open succeeds.** Likely fix once the probe above
   identifies the failing call: ensure `closeDevice(true)` + `usb_host_device_free_all()` truly
   release the handle/interface before `end()` returns, OR if the address is stale after a
   partial free, force a real re-enumeration. The recurring theme across #6/#7/#8 is
   **incomplete teardown** — the device handle/interface from the first connection isn't fully
   released, so anything that tries to re-open the same address fails.

3. **Pragmatic fallback — don't reconnect in-place.** The first connect per boot is reliable.
   Make `S → 2` (or a dead-stream detection) do a **full controlled restart** of the connect
   path from a guaranteed-clean state, or simply document "power-cycle the Cardputer to
   reconnect." Lower-effort, ships today, sidesteps the ESP-IDF teardown limitation.

### Tried and ruled out
- **Address-adopt alone (attempt #8)** — adopt works but open/probe fails on the reused host
  (the new wall, not a dead end — it advanced us one layer).
- **Root-port power sequencing (attempt #5)** — logical port power doesn't create the
  electrical edge the S3 root-port PHY needs; and the boot-trace showed first-connect works
  anyway, so it was solving a non-problem. Failed on hardware.
- **Host uninstall/reinstall retry (attempt #3)** — reinstalling doesn't re-scan an
  already-attached device.

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
