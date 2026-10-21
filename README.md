# RC Car Control App - Phase 1

Manual Bluetooth serial control for Arduino RC car with HC-05.

## Install

```
pip install pyserial
```

## Run

```
python car_control_app.py
```

## Setup

1. Pair HC-05 in Windows Bluetooth settings so it shows as COM port.
2. Open app, pick COM port from dropdown (or type e.g. COM5), click Connect.
3. If sketch uses different baud, edit BAUD_RATE at top of car_control_app.py.

## Controls

- Hold F/B/L/R buttons (or keys 8/2/4/6) to move, release sends stop 5.
- S button (or 5 key) sends stop.
- Mode button sends A (auto) / M (manual). Any drive press returns to manual.
- Raw box sends custom commands. Check add newline if sketch uses readStringUntil.

## Protocol

F->8, B->2, R->4, L->6, S->5, AUTO->A, MANUAL->M

## No COM ports?

Pair HC-05 first. Check Device Manager Ports (COM & LPT).
