# Autoseq Inactive Queue: Preserving QSO Metadata Across Retry Exhaustion

## Problem Statement

The autoseq engine stores all per-QSO metadata (dxcall, dxgrid, snr_tx, snr_rx,
logged, is_fd) in queue entries (`QsoContext`). When a context exhausts its retries,
it transitions to IDLE and is popped from the queue, destroying all metadata.

If DX is more patient than us and continues retrying, their message creates a
"reincarnated" context with default values (snr_tx=-99, logged=false). This
reincarnated context can complete the QSO and log an ADIF entry with wrong metadata.

### The Concrete Bug: REPORT + TX3 Reincarnation

```
T=0:00   We send TX2 (rst_sent=-12), retry 1
T=0:30   DX sends TX3 (R-08) — we don't decode (QRM)
T=1:00   We send TX2, retry 2
T=1:30   DX sends TX3 — we don't decode
  ...
T=2:30   Retries exhausted → IDLE → pop. snr_tx=-12 is gone forever.
T=3:00   DX sends TX3 — propagation improves, we decode!
         → on_decode: no matching ctx → new ctx (snr_tx=-99)
         → REPORT+TX3 → ROGERS → log_qso_if_needed
         → ADIF entry has rst_sent=-99 ← WRONG, no prior correct entry exists
```

### Root Cause

Our retry window (~2.5 min) can be shorter than DX's retry window (WSJT-X: up to
7+ min with 15 retries). After our context is popped, DX is still actively working
the QSO. The queue's lifetime is bounded by OUR retry limit, but the QSO's real
lifetime is bounded by DX's.

### Safety Invariant Analysis

Every pop from the queue falls into one of these cases:

| State at exhaustion | Logged? | DX received our TX | DX did NOT receive our TX |
|---|---|---|---|
| CALLING (CQ) | No | DX sends TX1 → normal new QSO | Nothing happens |
| REPLYING (TX1) | No | DX sends TX2 → new ctx, fresh exchange, consistent | DX resends CQ → not addressed to us |
| REPORT (TX2) | No | DX sends TX3 → new ctx, **snr_tx=-99 (BUG)** | DX resends TX2/TX1 → new ctx, snr_tx from msg.snr, OK |
| ROGER_REPORT (TX3) | No | DX sends TX4 → signoff guard filters it | DX resends TX2 → new ctx, fresh exchange, OK |
| ROGERS (TX4) | **Yes** | DX sends TX4/TX5 → signoff guard filters it | DX resends TX3 → **duplicate ADIF, snr_tx=-99** (original correct entry exists) |
| SIGNOFF (TX5) | **Yes** | QSO done | DX sends TX4/TX5 → signoff guard filters it |

The **REPORT + DX received** row is the critical gap: no prior correct ADIF entry
exists, and the reincarnated context has lost snr_tx.

## Solution: Active + Inactive Queue

Instead of popping contexts when retries exhaust, move them to an **inactive zone**
at the back of the queue. The single `s_queue[]` array serves two logical partitions:

```
s_queue[0 .. s_active_count-1]   → Active zone: has retries, gets scheduled by tick()
s_queue[s_active_count .. s_queue_size-1] → Inactive zone: retries exhausted, metadata preserved
```

### Key Design Rules

1. **Retry exhaustion → inactive, not IDLE.** When tick() exhausts retries, the
   context is marked `inactive = true` and moved to the back via sort. It is NOT
   popped. All metadata (snr_tx, snr_rx, dxgrid, logged, is_fd) is preserved.

2. **on_decode finds inactive contexts.** The existing dxcall-matching loop in
   on_decode() already scans the full queue. When DX retries, the dormant context
   is found, generate_response() advances its state, `inactive` is cleared, and
   it moves back to the active zone with fresh retries.

3. **Eviction only when safe.** A context can be evicted (popped) only if:
   - `logged == true` — ADIF has the correct entry, safe to discard
   - No exchange occurred — bare CQ with dxcall="CQ", nothing to preserve

4. **Reject unknown mid-QSO messages.** If DX sends a mid-QSO message (TX2, TX3)
   but has no context in either active or inactive zone, reject it. Only TX1 (grid,
   i.e. a new QSO start) from an unknown DX creates a new context. This prevents
   reincarnation with lost metadata entirely.

5. **Time-based expiry.** Inactive contexts expire after a configurable window.
   Expired inactive contexts are evicted regardless of logged state — if we haven't
   heard from DX in that long, the QSO is truly dead.

### Queue Sizing

Queue size: **120 entries.** This accommodates a busy 1-hour activation:
- Active zone: ~9 concurrent QSOs (same as before)
- Inactive zone: up to ~111 dormant contexts preserving metadata

The QsoContext struct is small (~80 bytes with std::string SSO), so 120 entries
is ~10 KB — negligible on ESP32-S3 (512 KB SRAM).

### Sort Order

```
Active entries (sorted by state priority, same as before):
  IDLE > SIGNOFF > ROGERS > ROGER_REPORT > REPORT > REPLYING > CALLING

Inactive entries (sorted to the back):
  All inactive entries sort BELOW all active entries.
  Among inactive entries: order doesn't matter (they don't TX).
```

### Modified tick() Behavior

```
Before:  retry exhausted → state = IDLE → pop_front()
After:   retry exhausted → inactive = true → sort to back (metadata preserved)
```

tick() only processes the front entry. If the front entry is inactive (shouldn't
happen after sort, but as a guard), skip it.

### Modified on_decode() Behavior

```
Before:  no matching ctx + not signoff → append_ctx + generate_response(override=true)
After:   no matching ctx + not signoff → check if mid-QSO (TX2/TX3):
           if TX2/TX3: reject (DX not in our queue = we have no record of this QSO)
           if TX1: allow (new QSO start, fresh context is correct)
```

### Reincarnation Eliminated

The reincarnation problem disappears because:
- Dormant contexts stay in the queue with all metadata
- on_decode finds them by dxcall, reactivates them
- Mid-QSO messages from unknown DX are rejected (no memory-less reincarnation)
- The only new contexts created are for TX1 (fresh QSO starts) where default
  metadata is correct
