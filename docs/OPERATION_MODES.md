# Mini-FT8 Operation Menu (V2.0)

## Quick Mode Map

| Key | Mode | Purpose |
|---|---|---|
| `R` | RX | View decoded messages and tap one to start QSO. |
| `T` | TX Queue | View and manage TX queue. |
| `S` | STATUS | Beacon, connect/sync, band step, tune, date/time. |
| `M` | MENU P1 | Core station/operator settings. |
| `N` | MENU P2 | Radio/input/comment settings. |
| `O` | MENU P3 | Logging/active bands/RTC/copy-to-SD/retry settings. |
| `Q` | QSO | Browse QSO/log files and view entries. |
| `F` | Fetch | Browse files and fetch/dump selected file to BLE client. |
| `D` | Delete Files | Browse and delete files in SPIFFS. |
| `B` | BAND | Edit per-band frequencies. |
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
   =============================  (29 `=` Frame Boundary)
   |                           |  (27 bins inside bars)
   ---+----+----+----+----+----+  (29 `-` with 500Hz tick mark)
   ```
4. Line 7 meta/edit line:
   - normal: `[MODE uv]` (`u`/`v` show page-up/page-down availability; `-` means not available)
   - text edit mode: `[Edit <item>]` (edit on BLE terminal first, enter to send, no escape)

Notes:
- Waterfall bins use `space`, `.`, `:`, `!` to indicate signal strength for about 100Hz
- counter: `|(slot boundary)`, `4`, `8`, `12`, `: (even)`, `. (odd)`, `o (Tx indicator)`
- decoded message count: `[D:n]`

## Per-Mode Controls
 - `` ` `` (ESC/backtick) applies wherever is needed.
 - Tex Edit: Backspace delete,  `` ` `` Cancel, Enter save
   
| Mode | Item | Notes |
|---|---|---|
| `R` (RX) | `1..6` | Select decoded line to reply. CQ messages are sorted from strong to weak. If the click happens within 4s, Tx starts immediately |
|  | `▲` `▼` | Page up/down is appliable if 1 or 6 turns cyan |
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
|  | `3` | Edit FreeText. (Long Edit) Multi-purpose for sotamat, park/summit reference, ARRL field day, CQ modifier (CQ EU, CQ ASIA, etc.) |
|  | `4` | Edit Call. (in place) |
|  | `5` | Edit Grid. (in place) 4/6/8-char grid. If GPS is available, grid from GPS will be shown/used but not saved |
|  | `6` | Enter Sleep. Battery Info |
| `N` (MENU P2) | `1` | Offset source Random/Rx/Fixed. Random values are within 500-2500Hz|
|  | `2` | Edit fixed cursor offset. (in place) Direct Enter or use `▲` `▼` `◀` `▶` |
|  | `3` | Radio select (`QMX` / `KH1`). |
|  | `4` | Edit ignore list (Long Edit). Prefixes separated by space, maximum 64-characters|
|  | `5` | Edit comment. (Long Edit) For ADIF logging, Supports Macro `/Radio`, `/Grid` Expansion |
|  | `6` | BLE on/off. Device name is `Mini-FT8-<callsign>` |
| `O` (MENU P3) | `1` | RxTx log on/off. Note: RxTxLog has been renamed to RT\[YYMMDD\].txt |
|  | `2` | SkipTX1 on/off. Skip `dxcall mycall mygrid` and reply with SNR report |
|  | `3` | Edit active bands. (Long Edit) For Status Key 3|
|  | `4` | Edit RTC compensation. (in place) |
|  | `5` | Copy files to SD. Feedback with `Copied OK` or `Missed [n]` |
|  | `6` | Edit max retry. (in place) any natural number or 0|
| `Q` (QSO) | `1..6` | Open selected ADIF file. |
|  | `◀` `▶` |  switch columns (choose Default view or SNR view). |
| `F` (Fetch) | `1..6` | Select/Send file over BLE. |
| `D` (Delete Files) | `1..6` | Delete selected file IMMEDIATELY without prompt |
| `B` (BAND) | `1..6` | Choose a band slot to edit. |
| `C` (Connect) |  | USB serial command mode for host commands. Only available before connected to a radio. Type `help`on PC to get host side commands |

## Download Logs
 - Use SD
   - Insert a FAT/FAT32 formatted SD card
   - Menu P3 (O), press 5 (Copy files to SD) — all files will be copied to the SD card (if "Missed", reboot will like fix it)
 - Use BLE
   - Use BLE Terminal, send F and choose the file
   - Use "Send Log file" on the BLE Terminal to save/email
 - Use pc_terminal.py
   - On M5 Carputer, click C to enter communication 
   - On PC: python .\pc_terminal.py COM11 for multiple commands 
   - On PC: python .\pc_terminal.py COM11 read 20260113.adi for single command 

## GPS Connections
Both 9600 and 115200 GPS will work
```text
┌──────────────────┐                 ┌─────────────────────────────┐
│ GPS              │                 │ Cardputer ADV               │
│                  │                 │ PORTA                       │
│ GND ─────────────┼─────────────────┤ GND                         │
│ VDD ─────────────┼─────────────────┤ 5V                          │
│ RX  ─────────────┼<──(Not Used)────┤ TX (G2)                     │
│ TX  ─────────────┼────────────────>┤ RX (G1)                     │
└──────────────────┘                 │                             │
                                     │ SW: 5VOUT (Left)            │
                                     └─────────────────────────────┘
```
## KH1 Connections

 - TX Only (sotamat)
```text
┌──────────────────┐                 ┌────────────────────────────┐
│ KH1 RS232        │                 │ Cardputer ADV              │
│                  │                 │ PORTA                      │
│ GND ─────────────┼─────────────────┤ GND                        │
│                  │                 │ 5V (NC)                    │
│ Tip(Rx) ─────────┼<────────────────┤ TX (G2)                    │
│ Ring(TX) ────────┼───(Not Used)───>┤ RX (G1)                    │
└──────────────────┘                 │                            │
                                     │ SW: NA                     │
                                     └────────────────────────────┘
```
- TX + RX (FT8 QSO)
  - Use a USB-C to audio/mic adapter for RX (Amazon search: B0FWC9ZFC4, other calbes may work too, but this is the one I tested)
  - Add 5V to PORTA, otherwise USB-C OTG port has no power
```text
┌──────────────────┐
│ Power Cable      │
│ GND ─────────────┼─────────┐
│ 5V  ─────────────┼─────┐   │
└──────────────────┘     |   |
┌──────────────────┐     |   |       ┌────────────────────────────┐
│ KH1 RS232        │     |   |       │ Cardputer ADV              │
│                  │     |   |       │ PORTA                      │
│ GND ─────────────┼─────)───┴───────┤ GND                        │
│                  │     └───────────┤ 5V                         │
│ Tip(Rx) ─────────┼<────────────────┤ TX (G2)                    │
│ Ring(TX) ────────┼── (Not Used)───>┤ RX (G1)                    │
└──────────────────┘                 │                            │
                                     │ SW: 5VIN (Right)           │
                                     └────────────────────────────┘
```

