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
   - State is CALLING or REPLYING — no reports exchanged yet. CALLING is a bare
     CQ. REPLYING means we only sent TX1 (grid); if DX retries with TX2, a fresh
     context gets correct snr_tx from msg.snr, so nothing valuable is lost.
   - Contexts in REPORT or beyond (sent TX2+) must be preserved — they carry the
     snr_tx we actually transmitted, which would be lost on eviction.

4. **Reject only R+report (TX3) from unknown DX.** If DX sends TX3 (R+report)
   and has no context in either active or inactive zone, reject it — TX3 skips
   the snr_tx latch, producing wrong ADIF metadata. All other message types are
   allowed for new context creation:
   - TX1 (grid): new QSO start, correct metadata
   - TX2 (report): DX skipped TX1 (WSJT-X "skip TX1" feature), snr_tx set from
     msg.snr, snr_rx from report text — correct metadata
   - FD exchange: equivalent to TX2 in Field Day mode — correct metadata
   - TX4/TX5 (RR73/73): filtered separately by signoff guard

5. **Time-based expiry.** Inactive contexts expire after a configurable window.
   Expired inactive contexts are evicted regardless of logged state — if we haven't
   heard from DX in that long, the QSO is truly dead.

### Queue Layout: Dynamic Boundary

The active and inactive zones share a single array and grow from opposite ends.
No fixed partition — both zones expand until their pointers meet.

```
s_queue[]:
 index 0                                               index AUTOSEQ_MAX_QUEUE-1
 ┌──────────────────────┬────────────┬──────────────────────┐
 │   Active zone        │  (free)    │   Inactive zone      │
 │   grows →            │            │            ← grows   │
 └──────────────────────┴────────────┴──────────────────────┘
       s_active_count ──┘            └── s_inactive_start
```

- `s_active_count`: number of active entries at the front (grows right)
- `s_inactive_start`: index of first inactive entry at the back (grows left)
- Free space: `s_inactive_start - s_active_count`
- When both pointers meet (free space = 0), queue is full:
  evict the oldest inactive entry (rightmost) to make room.

Queue size: **120 entries.** This accommodates a busy 1-hour activation.
QsoContext is ~80 bytes with std::string SSO, so 120 entries ≈ 10 KB —
negligible on ESP32-S3 (512 KB SRAM).

### Sort Order

Active entries are sorted by state priority (same as before):
```
  IDLE > SIGNOFF > ROGERS > ROGER_REPORT > REPORT > REPLYING > CALLING
```

Inactive entries are stored at the back in insertion order (oldest at the
rightmost end). When eviction is needed, the oldest (rightmost) entry is
removed first.

### Modified tick() Behavior

```
Before:  retry exhausted → state = IDLE → pop_front()
After:   retry exhausted + state > REPLYING → move to inactive zone at back
         retry exhausted + state <= REPLYING → evict (no reports exchanged)
```

tick() only processes the front of the active zone.

### Modified on_decode() Behavior

```
Before:  no matching ctx + not signoff → append_ctx + generate_response(override=true)
After:   no matching ctx → check message type:
           TX4/TX5 (RR73/73): reject (signoff guard, as before)
           TX3 (R+report): reject (snr_tx would be -99, reincarnation bug)
           TX1/TX2/FD exchange: allow (fresh context gets correct metadata)
```

### Reincarnation Eliminated

The reincarnation problem disappears because:
- Dormant contexts stay in the queue with all metadata
- on_decode finds them by dxcall, reactivates them
- Mid-QSO messages from unknown DX are rejected (no memory-less reincarnation)
- The only new contexts created are for TX1 (fresh QSO starts) where default
  metadata is correct

## Invariant: Active entries always have a valid next_tx

The autoseq engine maintains this invariant for every active queue entry:

> **If `ctx` is in the active zone and `state != IDLE`, then `next_tx != TX_UNDEF`.**

This invariant is what lets `autoseq_fetch_pending_tx()` safely return the TX
at `s_queue[0]` without checking for degenerate states. Violating it causes a
hard deadlock: `fetch_pending_tx` returns false, no TX fires, `tick()` never
runs to advance the state, and the dead entry blocks every subsequent TX —
including beacon CQs sorted behind it.

### How the invariant was maintained (pre-inactive-queue)

1. `set_state()` is the only legitimate way to change `state`, and it always
   writes `next_tx` at the same time.
2. The override path in `generate_response()` sets `next_tx = TX_UNDEF` as a
   "clean slate" but is always immediately followed by a state machine
   transition that restores a valid `next_tx`.
3. `autoseq_tick()` refreshes `next_tx` idempotently based on `state` on every
   call (within retry limit).
4. For an existing active ctx passed to `generate_response(override=false)`,
   the invariant holds by induction: if the state machine transitions, the new
   `set_state` maintains it; if it falls to `default`, `next_tx` was already
   valid from the prior tick or transition.

### How the inactive queue almost broke it

`move_to_inactive()` clobbers `next_tx = TX_UNDEF` before parking an entry.
When DX retries and `on_decode()` reactivates the dormant context, the
reactivated entry lands in the active zone with `next_tx == TX_UNDEF`.
`generate_response(override=false)` then runs the state machine; if the
(state, rcvd) combination falls to `default: return false`, `next_tx` is
*never* restored. The active zone now contains an entry violating the
invariant, and the queue deadlocks.

### Fix: make `generate_response` locally total

Every state handler in `generate_response()` must explicitly handle every
possible `rcvd` type — there is no implicit `default` that silently leaves
`next_tx` stale. For `(state, rcvd)` combinations that don't advance the QSO,
the handler refreshes `next_tx` to the state's canonical value and returns
false. The canonical mapping:

| State | Canonical next_tx | Meaning |
|---|---|---|
| CALLING | TX6 | Send CQ |
| REPLYING | TX1 | Send our grid |
| REPORT | TX2 | Send our report |
| ROGER_REPORT | TX3 | Send R+report |
| ROGERS | TX4 | Send RR73 |
| SIGNOFF | TX5 | Send 73 |

The semantics of every "no-op" case is identical: **"DX sent something that
doesn't advance our state — they probably didn't decode our last message.
Keep sending what matches our current state."** Encoding this in the
switch/case (rather than a centralized fallback) makes every state handler
self-contained and locally reasonable — when you read the REPORT case, you
see exactly what happens for every possible input, including the ones that
don't advance the QSO.

The invariant is restored even across the inactive queue boundary: if
`reactivate()` brings back an entry with stale `next_tx`, the very next call
to `generate_response()` repairs it unconditionally.
