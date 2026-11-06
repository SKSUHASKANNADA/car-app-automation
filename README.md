# Smart Autonomous RC Vehicle Navigation & Multi-Mode Guidance Platform

A complete multi-modal autonomous robotic vehicle navigation platform built with Python, OpenCV, Arduino, Bluetooth RFCOMM, and mobile sensor fusion.

Operates across **4 navigation modalities**:
1. **Manual Teleoperation Mode:** Low-latency keyboard/D-pad driving with instant release-to-stop.
2. **Auto Freeplay Mode:** Embedded Arduino autonomous obstacle roaming with ultrasonic range detection.
3. **Goal Maze Mode (Vision Autopilot):** Overhead IP webcam / USB camera closed-loop guidance using OpenCV ArUco markers (`ID 10` for Car, `ID 20` for Goal), dynamic obstacle inflation, and $A^*$ path planning.
4. **Classroom Maze Mode (No-Camera Mode):** 2D arena grid builder with photo imports, saved JSON maps, Realme 8 smartphone orientation (Gyro/Compass) HTTP bridge over Wi-Fi, continuous straight-line driving, and ultrasonic loose-wire watchdog protection.

---

## 🚀 Quick Start

### 1. Installation
Ensure Python 3.10+ (64-bit) is installed, then install the required dependencies:
```powershell
pip install -r requirements.txt
```

### 2. Microcontroller Setup
1. Open [`car_bridge_route.ino`](file:///e:/car%20app%20automation/car_bridge_route.ino) in the Arduino IDE.
2. Upload the sketch to your Arduino Uno / Nano.
3. Wire the hardware components according to the pin table below.

### 3. Launching the App
Run the launcher script or Python command directly:
```powershell
.\run_app.bat
# or
python car_control_app.py
```

---

## 🔌 Hardware Wiring & Pinout

| Module / Part | Pin | Arduino Pin | Description |
| :--- | :--- | :--- | :--- |
| **HC-05 Bluetooth** | `TXD` | `D10` | SoftwareSerial RX (receives commands from laptop) |
| **HC-05 Bluetooth** | `RXD` | `D11` | 3.3V via voltage divider ($1\text{k}\Omega / 2\text{k}\Omega$) |
| **HC-SR04 Ultrasonic** | `Trig` | `D8` | 10 $\mu\text{s}$ pulse trigger output |
| **HC-SR04 Ultrasonic** | `Echo` | `D7` | Return echo pulse receiver |
| **Micro Servo (Optional)** | `Signal`| `D9` | Center-locked at 90° (detached during route to free Timer1) |
| **Motor Driver Board** | `Data` | `D3` | Digital drive commands from Arduino to motor bridge |
| **Power** | `5V` / `GND` | `5V` / `GND` | Regulated logic supply |

---

## 🕹️ Driving & Controls

### Keyboard & D-Pad Shortcuts
- **Forward (`8` / `KP_8` / `▲`):** Move forward (Hold to move, release to stop)
- **Backward (`2` / `KP_2` / `▼`):** Move backward
- **Left (`4` / `KP_4` / `◄`):** Pivot turn left
- **Right (`6` / `KP_6` / `►`):** Pivot turn right
- **Stop (`5` / `KP_5` / `■`):** Instant emergency stop
- **Mode Toggle:** Press `A` to enter Auto Freeplay, or any drive key to return to Manual (`M`)

---

## 🧭 Navigation Modes

### 1. Goal Maze Mode (Overhead Camera Autopilot)
- Click **🎯 OPEN GOAL MAZE** in the main control dashboard.
- Enter your overhead phone camera IP stream (e.g. `http://192.168.137.21:8080/video`) or webcam index (`0`).
- Place printable markers from the `markers/` folder:
  - **Marker 10:** Affixed to car roof pointing forward.
  - **Marker 20:** Affixed to floor at the target destination.
- Click **Plan A* Route** $\rightarrow$ Click **▶ START DRIVE**.

### 2. Classroom Maze Mode (No External Camera Needed)
- Click **🏫 CLASSROOM MAZE** in the main control dashboard.
- Upload photos of your classroom or select a saved map (e.g. `My_Real_Room.json` in `classroom_maps/`).
- Connect your phone (Realme 8 / Android / iOS) to your laptop Wi-Fi Hotspot.
- Open the phone's browser and go to `http://<laptop-ip>:8080`.
- Mount phone on the car chassis to provide real-time Compass/Gyro heading.
- Click **🚀 START AUTOPILOT**. The car navigates smoothly along the $A^*$ path with active ultrasonic collision avoidance.

---

## 🛡️ Built-In Safety Features
- **Ultrasonic Loose-Wire Watchdog:** If sensor jumper wires become loose or disconnected (stuck at 0 cm or 999 cm / 400+ cm for 6+ consecutive cycles), the car automatically hard-stops and alerts the user on screen.
- **Dynamic Re-Routing:** If an unmapped obstacle blocks the vehicle's forward path closer than 35 cm, the Arduino immediately halts and requests a real-time detour from the $A^*$ planner.
- **Timer1 Interrupt Protection:** Servos are dynamically detached after centering so Arduino `pulseIn()` timing is never corrupted.

---

## 📁 Repository Structure

- [`car_control_app.py`](file:///e:/car%20app%20automation/car_control_app.py) - Main control dashboard & Bluetooth connection manager
- [`classroom_maze_app.py`](file:///e:/car%20app%20automation/classroom_maze_app.py) - 2D Classroom grid builder, phone gyro HTTP server & path runner
- [`route_planner_app.py`](file:///e:/car%20app%20automation/route_planner_app.py) - Overhead ArUco vision engine & closed-loop visual steering
- [`car_bridge_route.ino`](file:///e:/car%20app%20automation/car_bridge_route.ino) - Arduino firmware with ultrasonic collision guard & watchdog
- [`classroom_maps/`](file:///e:/car%20app%20automation/classroom_maps/) - Saved classroom and indoor room layout JSON files
- [`markers/`](file:///e:/car%20app%20automation/markers/) - High-contrast printable ArUco markers (`marker_10.png`, `marker_20.png`)
- [`TECHNICAL_DOCUMENTATION.md`](file:///e:/car%20app%20automation/TECHNICAL_DOCUMENTATION.md) - Deep-dive technical specifications and architectural documentation
