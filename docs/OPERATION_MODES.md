# Mini-FT8 Operation Menu (V1.5)

## Quick Mode Map

| Key | Mode | Purpose |
|---|---|---|
| `R` | RX | View decoded messages and tap one to start/continue QSO flow. |
| `T` | TX Queue | View and manage autoseq TX queue. |
| `B` | BAND | Edit per-band frequencies. |
| `S` | STATUS | Beacon, connect/sync, band step, tune, date/time. |
| `M` | MENU P1 | Core station/operator settings. |
| `N` | MENU P2 | Radio/input/comment settings. |
| `O` | MENU P3 | Logging/active bands/RTC/copy-to-SD/retry settings. |
| `Q` | QSO | Browse QSO/log files and view entries. |
| `F` | Fetch | Browse files and fetch/dump selected file to BLE client. |
| `D` | Delete Files | Browse and delete files in SPIFFS. |
| `C` | Connect | USB serial command mode. |

## Global Keys and Navigation

- `R/T/B/S/Q/F/D/C`: toggle target mode; pressing same mode key again returns to `RX`.
- `M/N/O`: jump to MENU pages 1/2/3; pressing the same page key again on that page returns to `RX`.
- `` ` `` (ESC/backtick): global TX cancel in `RX`, `TX`, and `STATUS` when not editing.
- `▲` `▼` page up/down (RX/TX/BAND/MENU/QSO/Fetch/Delete).
- `◀` `▶` left/right (QSO-SNR/Status-Date-Time/MENU-Fixed-LongEdit)
- `1..6` always target the currently visible row/slot for the active mode.
-  BLE Terminal
   - `u` `v` page up/down
   - `z` `x` left/right (QSO-SNR)

## BLE Screen Layout

1. Text waterfall frame:
   ```text
   =============================  (29 `=` chars)
   [                           ]  (27 bins inside brackets)
   -----------------------------  (29 `-` chars)
   ```
4. Line 7 meta/edit line:
   - normal: `[MODE uv]` (`u`/`v` show page-up/page-down availability; `-` means not available)
   - text edit mode: `[Edit <item>]` (edit on BLE terminal, enter to send, no escape)

Notes:
- Waterfall bins use `space`, `.`, `:`, `|` to indicate signal strength for about 100Hz
- counter: `|(slot boundary)`, `4`, `8`, `12`, `: (even)`, `. (odd)`, `o (Tx indicator)`
- decoded message count: `[D:n]`

## Per-Mode Controls
 - `` ` `` (ESC/backtick) applies wherever is needed.
 - Long Edit (for FreeText, Comment and IgnoreList): Enter save, Backspace delete, `` ` `` Cancel, `◀` `▶` Move cursor, Hold a key to repeat
 - Edit (in place): Enter save, Backspace delete, `` ` `` Cancel
   
| Mode | Item | Notes |
|---|---|---|
| `R` (RX) | `1..6` | Select decoded line to reply. CQ messages are sorted from strong to weak |
|  | `▲` `▼` | Page up/down if 1 or 6 turns cyan |
| `T` (TX Queue) | `1` | Rotate queue to next same-parity entry. |
|  | `2..6` | Drop queue item on current page. |
|  | `` ` `` (ESC) | Cancel a TX immediately. |
| `S` (STATUS) | `1` | Cycle Beacon mode; applies after idle delay or on exit. |
|  | `2` | Connect/sync action now; audio start + CAT sync path. |
|  | `3` | Step to next active band; applies after idle delay or on exit. |
|  | `4` | Tune toggle. |
|  | `5` | Edit Date. (in place) `G` indicates Date/Time is synced to GPS|
|  | `6` | Edit Time. (in place) |
| `M` (MENU P1) | `1` | CQ Type cycle. For CQ FD, enter operating class and ARRL/RAC section in FreeText, e.g. `1B SCV` |
|  | `2` | Send FreeText once. |
|  | `3` | Edit FreeText. (Long Edit) |
|  | `4` | Edit Call. (in place) |
|  | `5` | Edit Grid. (in place) 4-char for FT8, 6/8-char for comment only. If GPS is available, grid from GPS will be used but not saved |
|  | `6` | Enter Sleep. Battery Info |
| `N` (MENU P2) | `1` | Offset source cycle. |
|  | `2` | Edit fixed cursor offset. (in place) Direct Enter or use `▲` `▼` `◀` `▶` |
|  | `3` | Radio select (`QMX` / `KH1`). |
|  | `4` | Edit ignore list (Long Edit). |
|  | `5` | Edit comment. (Long Edit) Support Macro `/Radio`, `/Grid` Expansion |
|  | `6` | BLE on/off. Device name is `Mini-FT8-<callsign>` |
| `O` (MENU P3) | `1` | RxTx log on/off. |
|  | `2` | SkipTX1 on/off. |
|  | `3` | Edit active bands. (in place) |
|  | `4` | Edit RTC compensation. (in place) |
|  | `5` | Copy files to SD. Feedback with `Copied OK` or `Missed [n]` |
|  | `6` | Edit max retry. (in place) |
| `Q` (QSO) | `1..6` | Open selected ADIF file. |
|  | `◀` `▶` |  switch columns (Default/SNR). |
| `F` (Fetch) | `1..6` | Select/Send file over BLE. |
|  | | On device: BLE send only occurs when BLE is enabled and connected; otherwise selection is ignored. |
| `D` (Delete Files) | `1..6` | Delete selected file IMMEDIATELY without prompt |
| `C` (Connect) |  | USB serial command mode for host commands. Only available before connected to a radio. Type `help`on PC to get host side commands |
| `B` (BAND) | `1..6` | Choose a band slot to edit. |
