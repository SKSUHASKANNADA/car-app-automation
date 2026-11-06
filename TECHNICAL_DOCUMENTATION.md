# RC Car Automation — Technical Documentation

**Project Version:** 2.0  
**Last Updated:** 2026-10-30  
**Platform:** Windows 10/11 + Arduino Uno/Nano  

---

## Table of Contents

1. [Executive Summary](#1-executive-summary)
2. [System Architecture & Interconnects](#2-system-architecture--interconnects)
3. [Hardware Platform](#3-hardware-platform)
4. [Wiring Diagram](#4-wiring-diagram)
5. [Arduino Firmware](#5-arduino-firmware)
6. [Python Application Stack](#6-python-application-stack)
7. [Operational Modes — Detailed](#7-operational-modes--detailed)
8. [Calibration System](#8-calibration-system)
9. [A* Pathfinding](#9-a-pathfinding)
10. [Bluetooth Communication Protocol](#10-bluetooth-communication-protocol)
11. [Ultrasonic Safety System (4-Layer Stack)](#11-ultrasonic-safety-system-4-layer-stack)
12. [Classroom Maze Navigation State Machine](#12-classroom-maze-navigation-state-machine)
13. [Vision Route Planner](#13-vision-route-planner)
14. [File & Directory Structure](#14-file--directory-structure)
15. [Dependencies & Installation](#15-dependencies--installation)
16. [Known Constraints & Design Decisions](#16-known-constraints--design-decisions)
17. [Calibration Values Reference](#17-calibration-values-reference)

---

## 1. Executive Summary

This project implements a fully autonomous RC car capable of operating in **four distinct modes**:

| Mode | Trigger | Description |
|------|---------|-------------|
| **Manual Drive** | `car_control_app.py` | Keyboard-controlled driving with WASD/arrow keys; records drive profile for calibration |
| **Autonomous Avoid** | `A` mode in firmware | Onboard Arduino A* avoidance with servo scan; car navigates obstacles independently |
| **Classroom Maze** | `classroom_maze_app.py` | PC-side A* path planning on a user-drawn grid map; heading-aware discrete move/turn commands sent via Bluetooth |
| **Vision Route** | `vision_route_app.py` (camera tab) | Overhead camera + ArUco markers; real-time car and goal detection, 8-directional A*, continuous drive loop |

**Architecture overview:** A Python desktop application (`car_control_app.py`) serves as the central hub. It manages a persistent Bluetooth serial connection via `SerialManager`, dispatches incoming telemetry to the active child module, and routes keyboard/GUI commands to the Arduino. All intelligence for Classroom Maze and Vision Route modes runs on the PC; the Arduino acts as a **motor controller + ultrasonic safety guard**.

---

## 2. System Architecture & Interconnects

```
┌────────────────────────────────────────────────────────────────┐
│                    Windows PC (Python 3.x)                     │
│                                                                │
│  car_control_app.py  ←──────────────────────────────────────┐ │
│       │                                                      │ │
│       ├── SerialManager (thread)                             │ │
│       │       │  RFCOMM Bluetooth (9600 baud)                │ │
│       │       ▼                                              │ │
│       │   HC-05 Module ──► Arduino Uno/Nano                  │ │
│       │                        │                             │ │
│       │                        ├── Motor Board (L298N/TB6612)│ │
│       │                        ├── HC-SR04 Ultrasonic        │ │
│       │                        └── SG90 Servo (scan)         │ │
│       │                                                      │ │
│       ├── classroom_maze_app.py  (Classroom Maze tab)        │ │
│       ├── vision_route_app.py   (Vision Route tab)           │ │
│       └── calibration.json      (speed/turn constants)       │ │
└────────────────────────────────────────────────────────────────┘
```

### Thread Model

| Thread | Name | Purpose |
|--------|------|---------|
| Main | Tkinter event loop | GUI rendering, user input |
| Serial RX | `SerialManager._reader_thread` | Reads bytes from BT, queues lines |
| Navigation | `_nav_thread` in classroom_maze_app | Executes A* action sequence |
| Vision | Camera capture loop | Frame grab + ArUco detection + drive |

---

## 3. Hardware Platform

### RC Car Base

| Parameter | Value |
|-----------|-------|
| Car length | ~18 cm |
| Car width | ~22 cm |
| Drive type | Differential (2 driven wheels) |
| Turning type | Single-motor pivot (one wheel drives, other stationary) |
| Motor rated RPM | 200 RPM |
| Measured forward speed | 30.03 cm/s |
| 90° turn time | 960 ms (single-motor pivot) |

### Microcontroller

| Parameter | Value |
|-----------|-------|
| MCU | Arduino Uno / Nano (ATmega328P) |
| Clock | 16 MHz |
| Flash | 32 KB |
| SRAM | 2 KB |
| Operating voltage | 5 V |
| Firmware file | `car_final.ino` / `car_bridge_route.ino` |

### HC-05 Bluetooth Module

| Parameter | Value |
|-----------|-------|
| Protocol | SPP (RFCOMM) |
| Baud rate | 9600 |
| Role | Slave |
| Pairing PIN | 1234 (default) |
| Range | ~10 m line-of-sight |

### HC-SR04 Ultrasonic Sensor

| Parameter | Value |
|-----------|-------|
| Range | 2 cm – 400 cm |
| Beam angle | ~15° |
| Trigger pulse | 10 µs |
| pulseIn timeout | 22,000 µs (~3.8 m max) |
| Obstacle threshold | 36 cm (`OBSTACLE_CM`) |
| Confirmation filter | 2-stage, 15 ms apart |

### SG90 Servo (Scan)

| Parameter | Value |
|-----------|-------|
| Range | 0°–180° |
| Control | PWM on D9 |
| Use | Autonomous Avoid mode — sweeps left/center/right |
| Sweep angles | 30° (right), 90° (center), 150° (left) |

### Motor Board

| Parameter | Value |
|-----------|-------|
| Type | L298N or TB6612FNG |
| Control | Single PWM/direction line from D3 |
| Motor commands | Single-char ASCII (`'8'`, `'2'`, `'4'`, `'6'`, `'5'`) |

---

## 4. Wiring Diagram

```
Arduino Pin   →  Component / Net
─────────────────────────────────────────────────────────────
D10 (TX)      →  HC-05 RX  (SoftwareSerial TX to BT)
D11 (RX)      →  HC-05 TX  (SoftwareSerial RX from BT)
                  Note: D11 input uses 5V→3.3V voltage divider
                  (1kΩ + 2kΩ) to protect HC-05
D3            →  Motor board signal input (PWM-capable)
D8            →  HC-SR04 TRIG
D7            →  HC-SR04 ECHO
D9            →  SG90 Servo signal wire
5V            →  HC-05 VCC, HC-SR04 VCC
GND           →  HC-05 GND, HC-SR04 GND, Motor board GND, Servo GND
Vin / RAW     →  Motor board 12 V supply (separate battery pack)
```

> **Voltage divider for HC-05 RX line:**  
> Arduino D10 → 1kΩ → junction → HC-05 RX  
> junction → 2kΩ → GND  
> This steps 5 V signal down to ~3.33 V, within HC-05 3.3 V logic spec.

---

## 5. Arduino Firmware

### File Locations

| File | Purpose |
|------|---------|
| `C:\Users\sumuk\Downloads\car_final\car_final.ino` | Active sketch flashed to the physical car |
| `e:\car app automation\car_bridge_route.ino` | Workspace copy (identical to car_final.ino) |
| `e:\car app automation\car_bridge.ino` | Alternate build — also supports Route mode |

### Compile-time Constants

| Constant | Value | Description |
|----------|-------|-------------|
| `OBSTACLE_CM` | 36 | Distance (cm) below which obstacle is confirmed |
| `BT_BAUD` | 9600 | Bluetooth baud rate |
| `MOTOR_PIN` | 3 | Motor board signal pin |
| `TRIG_PIN` | 8 | Ultrasonic trigger |
| `ECHO_PIN` | 7 | Ultrasonic echo |
| `SERVO_PIN` | 9 | Servo PWM |

### Firmware State Machine

```
Power-on / Reset
      │
      ▼
   [IDLE] ←──────────────────────────────────────────────────────┐
      │  Receives mode char via BT                                │
      ├──── 'M' ──► [MANUAL MODE]   — forward all BT chars        │
      │                 to motor; ultrasonic passive              │
      │                                                           │
      ├──── 'A' ──► [AUTO AVOID]    — servo sweep loop;           │
      │                 measureCm per direction; steer away       │
      │                                                           │
      ├──── 'R' ──► [ROUTE MODE]    — forward BT motor commands   │
      │                 to motor; streams cm= telemetry every     │
      │                 180 ms; sends BLOCKED:<cm> on obstacle     │
      │                                                           │
      └──── 'S' ──► [STOP / IDLE]   — motorSend('5'); return      │
                                       to IDLE ──────────────────┘
```

### Mode Command Table

| Char | Mode Activated | Description |
|------|---------------|-------------|
| `M` | Manual | Raw passthrough; Python sends motor chars directly |
| `A` | Autonomous Avoid | Firmware-side servo scan + obstacle avoidance |
| `R` | Route | Motor command passthrough + ultrasonic guard |
| `S` | Stop/Idle | Sends stop, returns to idle |

### Motor Command Table

| Char | Action | Physical Effect |
|------|--------|----------------|
| `8` | Forward | Both motors drive forward |
| `2` | Backward | Both motors drive backward |
| `4` | Right pivot | One motor forward, other off |
| `6` | Left pivot | One motor forward, other off |
| `5` | Stop | All motors off |

> **CMD_MAP note:** In Python `classroom_maze_app.py`, `"R"` maps to `"6"` and `"L"` maps to `"4"`. This asymmetry is intentional — it compensates for the motor board wiring polarity on this specific car chassis.

### Key Firmware Functions

#### `measureCm()`
```
1. Send 10 µs pulse on TRIG_PIN
2. pulseIn(ECHO_PIN, HIGH, 22000UL)  ← 22 ms timeout ≈ 3.8 m
3. Return duration / 58 (cm)
4. Returns 0 if timeout (wire fault / out of range)
```

#### `readScan()` — 2-Stage Confirmation Filter
```
Stage 1: d1 = measureCm()
         if d1 > OBSTACLE_CM → return 999  (clear, no obstacle)
         delay(15)
Stage 2: d2 = measureCm()
         if d2 > OBSTACLE_CM → return 999  (transient noise, ignore)
         return min(d1, d2)               (confirmed obstacle)
```
This eliminates single-sample false positives from electrical noise or reflections.

#### `avoidObstacle()` (Auto Avoid mode)
```
1. Stop motors
2. Sweep servo: right (30°), center (90°), left (150°)
3. Measure distances at each angle
4. Choose direction with maximum clearance
5. Execute turn + forward
```

#### Route Mode Loop
```
while mode == 'R':
    if bt.available():
        char c = bt.read()
        if c in {'8','2','4','6','5'}: motorSend(c)
        if c == 'S': break

    d = readScan()
    if d <= OBSTACLE_CM:
        motorSend('2')   // active reverse
        delay(70)
        motorSend('5')   // stop
        bt.println("BLOCKED:" + d)
    else:
        bt.println("cm=" + d)   // telemetry every 180 ms

    delay(180)
```

---

## 6. Python Application Stack

### Entry Point: `car_control_app.py`

- Builds the main Tkinter window with a `ttk.Notebook` (tabs)
- Instantiates `SerialManager` for persistent BT connection
- Hosts child modules as tab frames:
  - `ClassroomMazeApp` (classroom_maze_app.py)
  - Vision Route tab (camera + ArUco)
- Routes incoming RX lines to the active child module's `handle_rx_message()`

### `SerialManager`

| Feature | Detail |
|---------|--------|
| Port auto-detection | Scans all COM ports; connects to first responding with `ping` → `pong` |
| Read thread | Daemon thread; reads lines; puts to `rx_queue` |
| Write method | `send(char_or_string)` — thread-safe |
| Reconnect | Auto-retry on disconnect |

### Child Modules

| Module | Class | Tab Label |
|--------|-------|-----------|
| `classroom_maze_app.py` | `ClassroomMazeApp` | Classroom Maze |
| `vision_route_app.py` | `VisionRouteApp` | Vision Route |

---

## 7. Operational Modes — Detailed

### Mode 1: Manual Drive

- **Trigger:** User clicks "Manual" button; Python sends `'M'` to Arduino
- **Control:** WASD or arrow keys → Python sends `'8'`/`'2'`/`'4'`/`'6'`/`'5'`
- **Safety:** No ultrasonic guard active in firmware (passthrough)
- **Profiler:** Records keypress durations; used to compute `speed_cm_s` for calibration

### Mode 2: Autonomous Avoid

- **Trigger:** User clicks "Auto" button; Python sends `'A'` to Arduino
- **Control:** Fully onboard — no PC commands during operation
- **Ultrasonic:** Servo sweeps 3 angles; firmware chooses best direction
- **No Python interaction** during avoidance loop

### Mode 3: Classroom Maze Navigation

- **Trigger:** User draws grid, sets car/goal, clicks "Start Navigation"
- **PC sends:** `'R'` mode, then discrete move/turn commands timed by `calibration.json`
- **Arduino:** Passes motor chars to motor board; streams `cm=` + `BLOCKED:` back
- **Details:** See Section 12

### Mode 4: Vision Route Planner

- **Trigger:** Camera tab; user selects source; clicks "Start"
- **PC:** Captures frames, detects ArUco markers, runs A*, sends drive commands
- **Arduino:** `'R'` mode (motor passthrough + ultrasonic guard)
- **Details:** See Section 13

---

## 8. Calibration System

### `calibration.json`

```json
{
  "speed_cm_s": 30.03,
  "turn_90_ms": 960
}
```

| Key | Value | Description |
|-----|-------|-------------|
| `speed_cm_s` | 30.03 | Measured forward speed of the car (cm/s) |
| `turn_90_ms` | 960 | Time (ms) for a 90° single-motor pivot turn |

### How It Was Measured

| Test | Method | Result |
|------|--------|--------|
| Speed | Car driven 100 cm; timed at 3.33 s | 100/3.33 = 30.03 cm/s |
| Speed (verify) | 200 cm; timed at 6.66 s | 200/6.66 = 30.03 cm/s |
| Turn | Car placed on grid; 90° turn timed | 960 ms |

### How Calibration Is Used in Code

In `classroom_maze_app.py`:

```python
# Distance → time conversion
move_time_ms = (cell_size_cm / speed_cm_s) * 1000

# Turn timing (direct lookup)
turn_time_ms = turn_90_ms           # for 90° turns
turn_time_ms = turn_90_ms * 2       # for 180° U-turns (two sequential 90° turns)
```

The default grid cell size is **25 cm**, giving:
- `move_time_ms = (25 / 30.03) × 1000 ≈ 832 ms` per cell forward move

---

## 9. A* Pathfinding

### Classroom Maze Mode — Cardinal-Only A*

- **Grid:** User-drawn cells on Tkinter canvas; each cell = 25 cm
- **Neighbors:** 4-directional only (N, S, E, W — no diagonals)
- **Heuristic:** Manhattan distance
- **Obstacle cells:** Blocked; not traversable
- **Output:** List of `(row, col)` waypoints from car to goal

#### Why No Diagonals?

The physical car executes only 90° pivot turns. Diagonal movement would require 45° turns which the single-motor pivot cannot produce consistently. Removing diagonals ensures every planned path is physically executable.

### Heading-Aware Action Generation (`_generate_path_actions()`)

The path (list of grid cells) is converted to a sequence of actions accounting for the car's current heading:

```
DIR_TO_HDG = {
    (−1,  0): 0,    # North (up on screen)
    ( 0, +1): 90,   # East  (right)
    (+1,  0): 180,  # South (down)
    ( 0, −1): 270,  # West  (left)
}
```

For each step in the path:
1. Compute required heading from `DIR_TO_HDG`
2. Compute `delta = (required − current) % 360`
3. Map delta to turns:

| Delta | Action |
|-------|--------|
| 0° | `MOVE` (straight ahead) |
| 90° | `TURN_RIGHT` then `MOVE` |
| 270° | `TURN_LEFT` then `MOVE` |
| 180° | `TURN_RIGHT` + `TURN_RIGHT` then `MOVE` (U-turn) |

### Vision Route Mode — 8-Directional A*

- **Neighbors:** 8-directional (includes diagonals — allowed because vision mode uses continuous drive, not discrete turns)
- **Obstacle inflation:** Minkowski sum with car footprint (cells within car half-width are also blocked)
- **Heuristic:** Euclidean distance

---

## 10. Bluetooth Communication Protocol

### Python → Arduino Commands

| Command | Meaning |
|---------|---------|
| `M` | Enter Manual mode |
| `A` | Enter Autonomous Avoid mode |
| `R` | Enter Route mode |
| `S` | Stop / return to idle |
| `8` | Motor: Forward |
| `2` | Motor: Backward |
| `4` | Motor: Right pivot |
| `6` | Motor: Left pivot |
| `5` | Motor: Stop |

### Arduino → Python Telemetry

| Message Format | When Sent | Python Handler |
|---------------|-----------|---------------|
| `cm=<distance>` | Every 180 ms during Route mode | HUD distance display; backup obstacle guard |
| `BLOCKED:<distance>` | On confirmed obstacle in Route mode | `_handle_dynamic_obstacle()` |
| `pong` | Response to `ping` during auto-detect | `SerialManager` connection confirm |

### RX Routing Logic

```
SerialManager._reader_thread
    → puts line in rx_queue
    
car_control_app.py polling loop (every 50 ms)
    → gets line from rx_queue
    → if active tab == Classroom Maze:
          classroom_maze_app.handle_rx_message(line)
      elif active tab == Vision Route:
          vision_route_app.handle_rx_message(line)
      else:
          manual mode HUD update
```

---

## 11. Ultrasonic Safety System (4-Layer Stack)

The system uses multiple redundant layers to prevent collisions:

### Layer 1 — 2-Stage Confirmation Filter (Arduino firmware)

- `readScan()` takes two measurements 15 ms apart
- Both must read ≤ `OBSTACLE_CM` to trigger
- Single-sample noise is rejected
- Eliminates false stops from electrical interference or transient reflections

### Layer 2 — Active Reverse-Brake Pulse (Arduino firmware)

When obstacle confirmed:
```
motorSend('2')   // full reverse
delay(70)        // 70 ms reverse pulse (~2 cm rollback)
motorSend('5')   // stop
```
This counteracts motor coasting (12–18 cm overrun on CMD_STOP alone).

### Layer 3 — Python BLOCKED Handler

On receiving `BLOCKED:<cm>`:
- `_navigation_loop` sets `obstacle_detected_event`
- Navigation thread immediately pauses
- `_handle_dynamic_obstacle()` marks obstacle cells on map
- A* replanned from current position
- Retry counter incremented (max 12 retries)

### Layer 4 — Python cm= Backup Guard

On receiving `cm=<d>` where `0 < d ≤ 32`:
- Python also halts navigation (secondary trigger)
- Guards against `BLOCKED:` message being lost or delayed
- Threshold is 32 cm (tighter than firmware 36 cm to avoid double-firing)

### Layer 5 — Wire Fault Watchdog

`measureCm()` returns 0 on pulseIn timeout (no echo received):
- Timeout = 22,000 µs (beyond 3.8 m range)
- Return value 0 is treated as "sensor fault / out of range"
- Does **not** trigger obstacle stop (avoids false halt on open space)
- Python HUD shows `---` for cm= 0 values

---

## 12. Classroom Maze Navigation State Machine

### Grid Map

| Parameter | Value |
|-----------|-------|
| Cell size | 25 cm × 25 cm |
| Default grid | 10 × 10 cells |
| Cell states | Empty / Obstacle / Car / Goal |
| Map persistence | JSON file (`maze_map.json`) |
| Map format | `{"grid": [[0,0,...],[1,0,...],...], "car": [r,c], "goal": [r,c]}` |

### Car Orientation Selector

Toolbar Row 3 contains a "Car Faces" radio group:

| Button | Label | Heading (°) |
|--------|-------|-------------|
| ⬆ | N | 0 |
| ➡ | E | 90 |
| ⬇ | S | 180 |
| ⬅ | W | 270 |

Selecting an orientation immediately previews the heading arrow on the car marker before navigation starts. The selected heading is stored in `_heading_var` and used as the initial `car_heading` in `_navigation_loop`.

### `_navigation_loop()` Flowchart

```
Start → Send 'R' mode → 200 ms delay
      ↓
Compute A* path (car pos → goal pos)
      ↓
path empty? → YES → "Cannot Reach Goal ⚠️" → END
      ↓ NO
Generate action sequence (heading-aware)
      ↓
For each action:
  ├─ MOVE  → send '8', sleep move_time_ms, send '5', sleep 120 ms
  ├─ TURN_RIGHT → _execute_discrete_turn(RIGHT)
  │              car_heading = (car_heading + 90) % 360
  └─ TURN_LEFT  → _execute_discrete_turn(LEFT)
                  car_heading = (car_heading − 90) % 360
      ↓
obstacle_detected_event set?
  YES → _handle_dynamic_obstacle()
      → retry_count += 1
      → retry_count > MAX_RETRIES? → "Cannot Reach Goal ⚠️" → END
      → replan A* from current pos
  NO  → continue
      ↓
All actions done → check position:
  abs(car_row − goal_row) ≤ 1
  AND abs(car_col − goal_col) ≤ 1 ?
  YES → "Goal Reached! 🎉"
  NO  → "Cannot Reach Goal ⚠️"
      ↓
Send 'S' → END
```

### `_execute_discrete_turn()` Timing

```
send '5'       (pre-stop)         → 180 ms
send turn char ('4' or '6')       → turn_90_ms (960 ms)
send '5'       (post-settle)      → 380 ms
```
Total per 90° turn: ~1520 ms.

### `_handle_dynamic_obstacle()` — Dual Marking Strategy

**Primary (heading-based geometry):** Mark cells directly in front of car based on `car_heading`:
```
North (0°):   mark (car_row−1, car_col), (car_row−2, car_col)
East  (90°):  mark (car_row, car_col+1), (car_row, car_col+2)
South (180°): mark (car_row+1, car_col), (car_row+2, car_col)
West  (270°): mark (car_row, car_col−1), (car_row, car_col−2)
```

**Secondary (path-based):** Scan remaining path for the first cell ahead of current position and mark it as obstacle.

Both strategies run; cells outside grid bounds are silently skipped.

---

## 13. Vision Route Planner

### Camera Source Types

| Source | Description |
|--------|-------------|
| Webcam (index) | Local USB or built-in camera (`cv2.VideoCapture(0)`) |
| IP Camera (URL) | HTTP MJPEG stream (e.g., phone camera app) |

### ArUco Marker Configuration

| Parameter | Value |
|-----------|-------|
| Dictionary | `DICT_4X4_50` (OpenCV ArUco) |
| Car marker ID | 10 |
| Goal marker ID | 20 |
| Marker size | ~8–10 cm physical print size recommended |
| Detection | Per-frame; `cv2.aruco.detectMarkers()` |

### Occupancy Map

- Resolution: configurable (default 10×10 mapped to camera frame)
- Obstacle cells: manually drawn on map OR detected from camera frame edges
- Inflation: Minkowski sum with half-car-width radius

### 5 Hz Drive Loop

Each cycle (200 ms):
1. Grab frame from camera
2. Detect all ArUco markers
3. Extract car pose (ID 10 centroid + orientation from corner order)
4. Extract goal pose (ID 20 centroid)
5. Map pixel coords → grid coords
6. Run 8-directional A* from car → goal
7. Compute next waypoint direction
8. Send appropriate motor command (`'8'`/`'4'`/`'6'`)
9. Update canvas overlay (path, car arrow, goal marker)

---

## 14. File & Directory Structure

```
e:\car app automation\
│
├── car_control_app.py          # Main application entry point
├── classroom_maze_app.py       # Classroom Maze tab (A* grid navigation)
├── vision_route_app.py         # Vision Route tab (camera + ArUco)
├── calibration.json            # Speed and turn calibration constants
│
├── car_bridge_route.ino        # Arduino firmware (workspace copy, = car_final.ino)
├── car_bridge.ino              # Alternate Arduino firmware build
│
├── maze_map.json               # Saved grid map (auto-generated on save)
├── TECHNICAL_DOCUMENTATION.md # This document
│
└── [logs/screenshots]          # Any debug output or captured frames

C:\Users\sumuk\Downloads\car_final\
└── car_final.ino               # Active Arduino sketch flashed to physical car
```

---

## 15. Dependencies & Installation

### Python Dependencies (`requirements.txt`)

```
pyserial>=3.5
opencv-python>=4.5
numpy>=1.21
Pillow>=9.0
tkinter          # built-in with Python 3.x on Windows
```

Install with:
```
pip install pyserial opencv-python numpy Pillow
```

### Arduino Upload Steps

1. Open `C:\Users\sumuk\Downloads\car_final\car_final.ino` in Arduino IDE
2. Select Board: `Arduino Uno` (or `Arduino Nano` with correct bootloader)
3. Select Port: the COM port for the USB connection
4. Click **Upload**
5. Open Serial Monitor at 9600 baud to verify startup message

> **Library note:** If `Multiple libraries found for Servo.h` warning appears, this is harmless. The IDE uses the first found. Verify it uses `C:\Users\sumuk\OneDrive\Documents\Arduino\libraries\Servo`.

### Windows Bluetooth Pairing Steps

1. Turn on the HC-05 module (power on the car)
2. Open Windows Settings → Bluetooth → Add Device
3. Pair with `HC-05` (or `hc-05`); PIN = `1234`
4. After pairing, open Device Manager → Ports (COM & LPT)
5. Note the COM port assigned to HC-05 (outgoing port)
6. The Python app auto-detects the correct COM port on startup

---

## 16. Known Constraints & Design Decisions

| Constraint | Reason / Design Decision |
|-----------|--------------------------|
| 9600 baud Bluetooth | HC-05 default; sufficient for single-char commands and telemetry lines. Higher baud rates risk data loss on SoftwareSerial |
| Timer1 conflict | `SoftwareSerial` uses Timer1 on ATmega328P. If `Servo` is also active (Auto Avoid mode), Timer1 is shared — servo jitter possible during BT receive. Accepted trade-off; modes don't overlap |
| SoftwareSerial dual-use | SoftwareSerial on D10/D11 handles both BT RX and TX. Only one SoftwareSerial can listen at a time; `bt.listen()` keeps it active. Motor serial is TX-only, so no conflict |
| No diagonal A* (Classroom) | Single-motor pivot cannot produce 45° turns. Cardinal-only A* ensures all planned paths are physically executable |
| Single-motor pivot turns | Car chassis has two driven wheels. Turning by driving one wheel while the other is stationary gives a pivot turn. Turn radius ≈ half the car width (~11 cm) |
| Tkinter threading | Tkinter is not thread-safe. All GUI updates from navigation/serial threads use `widget.after(0, ...)` to schedule on the main thread |
| RFCOMM latency | BT RFCOMM adds 20–80 ms latency per command. Navigation timing accounts for this with settle delays (380 ms post-turn) |
| cm= threshold = 32 cm | Python backup safety uses 32 cm (tighter than firmware's 36 cm `OBSTACLE_CM`). The 4 cm gap prevents both layers from triggering simultaneously on the same reading |
| Phone / gyro removed | Phone-based gyroscope integration was removed entirely. Car heading is now computed purely from command history (dead reckoning) |
| MAX_RETRIES = 12 | Navigation retries up to 12 times on obstacle. If path still not found after 12 replans, "Cannot Reach Goal" is shown |

---

## 17. Calibration Values Reference

### Speed & Turn Summary

| Parameter | Value | Unit |
|-----------|-------|------|
| Forward speed | 30.03 | cm/s |
| 90° turn time | 960 | ms |
| 180° turn time | 1920 | ms (two 90° turns) |
| Active brake pulse | 70 | ms |
| Pre-turn stop pause | 180 | ms |
| Post-turn settle | 380 | ms |
| Route telemetry interval | 180 | ms |
| pulseIn timeout | 22,000 | µs |
| Confirmation filter delay | 15 | ms |
| Python backup safety threshold | 32 | cm |
| Firmware obstacle threshold | 36 | cm (`OBSTACLE_CM`) |

### Distance → Time Table

| Distance | Time at 30.03 cm/s |
|----------|-------------------|
| 10 cm | 333 ms |
| 15 cm | 499 ms |
| 20 cm | 666 ms |
| 25 cm (1 cell) | 832 ms |
| 50 cm (2 cells) | 1664 ms |
| 75 cm (3 cells) | 2497 ms |
| 100 cm | 3330 ms |

### Turn Reference Table

| Turn Type | Command Sequence | Total Time |
|-----------|-----------------|-----------|
| 90° Right | Stop(180ms) → `'4'`(960ms) → Stop(380ms) | ~1520 ms |
| 90° Left | Stop(180ms) → `'6'`(960ms) → Stop(380ms) | ~1520 ms |
| 180° U-Turn | 90° Right + 90° Right | ~3040 ms |

---

*End of Technical Documentation*  
*Generated for RC Car Automation Project v2.0*
