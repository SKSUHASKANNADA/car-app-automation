"""classroom_maze_app.py - Classroom Goal Maze without Phone / Camera.
Includes:
- Multi-photo classroom viewer + 2D interactive grid map builder (with save/load/delete)
- A* pathfinder with real-time waypoint rendering
- Cardinal Direction Indicators: North (N), South (S), East (E), West (W) & Compass Rose badge
- Calibration integration (speed cm/s & turn_90_ms) from manual Bluetooth driving
- Ultrasonic Loose-Connection Watchdog (alerts if sensor hangs at 0 or disconnects)
- Closed-loop Autopilot runner (steers car via Arduino Bluetooth link with ultrasonic obstacle stop)
"""
import heapq
import json
import math
import os
import threading
import time
import tkinter as tk
from datetime import datetime
from tkinter import filedialog, messagebox, ttk

try:
    from PIL import Image, ImageTk
except ImportError:
    Image = None
    ImageTk = None

try:
    import cv2
    import numpy as np
except ImportError:
    cv2 = None
    np = None

# Folder where saved classroom maps are stored
MAPS_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "classroom_maps")
os.makedirs(MAPS_DIR, exist_ok=True)

CALIBRATION_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)), "calibration.json")

DEFAULT_ROOM_WIDTH_M = 8.0
DEFAULT_ROOM_HEIGHT_M = 6.0
DEFAULT_CELL_SIZE_CM = 25.0

# Commands (Motor board mapping: 8=Fwd, 2=Back, 4=Right, 6=Left, 5=Stop)
CMD_FWD = "8"
CMD_BACK = "2"
CMD_RIGHT = "4"
CMD_LEFT = "6"
CMD_STOP = "5"
MODE_ROUTE = "R"


def load_calibration():
    """Loads speed and turn parameters from calibration.json."""
    calib = {"speed_cm_s": 68.0, "turn_90_ms": 220}
    if os.path.exists(CALIBRATION_FILE):
        try:
            with open(CALIBRATION_FILE, "r", encoding="utf-8") as f:
                data = json.load(f)
                calib["speed_cm_s"] = float(data.get("speed_cm_s", 68.0))
                calib["turn_90_ms"] = int(data.get("turn_90_ms", 220))
        except Exception:
            pass
    return calib


class ClassroomMazeApp:
    def __init__(self, root, shared_link=None):
        self.root = root
        self.shared_link = shared_link
        self.root.title("Classroom Goal Maze (Autonomous Map Navigation)")
        self.root.geometry("1220x790")
        self.root.minsize(1020, 680)

        # Calibration state
        self.calib = load_calibration()

        # Map state
        self.room_w_m = DEFAULT_ROOM_WIDTH_M
        self.room_h_m = DEFAULT_ROOM_HEIGHT_M
        self.cell_cm = DEFAULT_CELL_SIZE_CM
        self.cols = int(round((self.room_w_m * 100.0) / self.cell_cm))
        self.rows = int(round((self.room_h_m * 100.0) / self.cell_cm))

        # 0 = free, 1 = obstacle
        self.grid = [[0 for _ in range(self.cols)] for _ in range(self.rows)]
        self.start_pos = (1, 1)                      # (row, col)
        self.goal_pos = (self.rows - 2, self.cols - 2)
        self.path = []

        # Current virtual car pose in grid
        self.car_pos = list(self.start_pos)
        self.car_heading = 0.0

        # Multi-photo management
        self.photos = []
        self.current_photo_idx = 0
        self.photo_tk_cache = None

        # Editor tool
        self.current_tool = tk.StringVar(value="brush_obs")

        # Autopilot state
        self.is_navigating = False
        self.nav_thread = None
        self._stop_nav = threading.Event()
        self.obstacle_detected_event = threading.Event()

        # Ultrasonic loose connection watchdog tracking
        self.last_ultrasonic_cm = 999
        self.last_ultrasonic_time = time.time()
        self.stuck_zero_count = 0
        self.stuck_timeout_count = 0
        self.ultrasonic_alert_active = False

        self._build_ui()
        self._refresh_saved_maps_list()

        # Find and load My_Real_Room if it exists, otherwise select first
        items = self.maps_listbox.get(0, tk.END)
        idx = 0
        if "My_Real_Room" in items:
            idx = items.index("My_Real_Room")
        if self.maps_listbox.size() > 0:
            self.maps_listbox.selection_set(idx)
            self._load_selected_map()
        else:
            self._run_pathfinding()
            self._redraw_grid()

    # ================= UI BUILDER =================
    def _build_ui(self):
        top_bar = tk.Frame(self.root, bg="#1e293b", height=48)
        top_bar.pack(side=tk.TOP, fill=tk.X)

        title_lbl = tk.Label(
            top_bar,
            text="🧭 CLASSROOM GOAL MAZE (NO PHONE / 100% AUTONOMOUS)",
            font=("Segoe UI", 11, "bold"),
            fg="#f8fafc",
            bg="#1e293b"
        )
        title_lbl.pack(side=tk.LEFT, padx=12, pady=6)

        # Ultrasonic sensor pill
        self.sonic_pill = tk.Label(
            top_bar,
            text="📡 Ultrasonic: OK",
            font=("Segoe UI", 9, "bold"),
            fg="#10b981",
            bg="#1e293b"
        )
        self.sonic_pill.pack(side=tk.LEFT, padx=10, pady=6)

        # Live Telemetry: Distance to Goal & Estimated Speed
        self.telemetry_pill = tk.Label(
            top_bar,
            text=f"🎯 Goal Dist: -- m | Calibrated Speed: {self.calib['speed_cm_s']:.1f} cm/s",
            font=("Segoe UI", 9, "bold"),
            fg="#38bdf8",
            bg="#1e293b"
        )
        self.telemetry_pill.pack(side=tk.LEFT, padx=12, pady=6)

        self.status_lbl = tk.Label(
            top_bar,
            text="Ready. Draw obstacles or run Autopilot.",
            font=("Segoe UI", 9),
            fg="#94a3b8",
            bg="#1e293b"
        )
        self.status_lbl.pack(side=tk.RIGHT, padx=12, pady=6)

        main_pane = tk.PanedWindow(self.root, orient=tk.HORIZONTAL, sashrelief=tk.RAISED, sashwidth=4)
        main_pane.pack(fill=tk.BOTH, expand=True)

        # 1. Left Panel (Photos Reference + Saved Maps Manager)
        left_frame = tk.Frame(main_pane, width=320, bg="#f8fafc")
        main_pane.add(left_frame, minsize=290)

        self._build_photo_panel(left_frame)
        self._build_saved_maps_panel(left_frame)

        # 2. Center/Right Panel (Canvas & Tool Palette)
        center_frame = tk.Frame(main_pane, bg="#ffffff")
        main_pane.add(center_frame, minsize=620)

        self._build_toolbar(center_frame)
        self._build_canvas(center_frame)

    def _build_photo_panel(self, parent):
        box = tk.LabelFrame(parent, text=" Classroom Photos Reference ", font=("Segoe UI", 9, "bold"), padx=6, pady=6)
        box.pack(side=tk.TOP, fill=tk.BOTH, expand=True, padx=8, pady=(8, 4))

        btn_row = tk.Frame(box)
        btn_row.pack(fill=tk.X, pady=(0, 4))

        up_btn = tk.Button(
            btn_row,
            text="+ Upload Photos",
            bg="#2563eb",
            fg="white",
            font=("Segoe UI", 9, "bold"),
            command=self._upload_photos
        )
        up_btn.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=2)

        clr_btn = tk.Button(
            btn_row,
            text="Clear",
            font=("Segoe UI", 9),
            command=self._clear_photos
        )
        clr_btn.pack(side=tk.RIGHT, padx=2)

        # Photo Preview Canvas
        self.photo_canvas = tk.Canvas(box, bg="#e2e8f0", height=180, highlightthickness=1)
        self.photo_canvas.pack(fill=tk.BOTH, expand=True, pady=4)

        # 1-Click AI / Vision Auto-Grid Generator from photo
        auto_grid_btn = tk.Button(
            box,
            text="⚡ Auto-Detect Obstacles From Photo",
            bg="#059669",
            fg="white",
            font=("Segoe UI", 9, "bold"),
            command=self._auto_detect_grid_from_photo
        )
        auto_grid_btn.pack(fill=tk.X, pady=(2, 4))

        nav_row = tk.Frame(box)
        nav_row.pack(fill=tk.X)

        self.prev_photo_btn = tk.Button(nav_row, text="< Prev", command=self._prev_photo, state=tk.DISABLED)
        self.prev_photo_btn.pack(side=tk.LEFT, padx=2)

        self.photo_counter_lbl = tk.Label(nav_row, text="No photos added", font=("Segoe UI", 8))
        self.photo_counter_lbl.pack(side=tk.LEFT, expand=True)

        self.next_photo_btn = tk.Button(nav_row, text="Next >", command=self._next_photo, state=tk.DISABLED)
        self.next_photo_btn.pack(side=tk.RIGHT, padx=2)

    def _build_saved_maps_panel(self, parent):
        box = tk.LabelFrame(parent, text=" Saved Maps (Manage & Delete) ", font=("Segoe UI", 9, "bold"), padx=6, pady=6)
        box.pack(side=tk.BOTTOM, fill=tk.X, padx=8, pady=(4, 8))

        self.maps_listbox = tk.Listbox(box, height=5, font=("Segoe UI", 9), selectmode=tk.SINGLE)
        self.maps_listbox.pack(fill=tk.X, pady=(0, 4))

        btn_grid = tk.Frame(box)
        btn_grid.pack(fill=tk.X)

        save_btn = tk.Button(btn_grid, text="Save As...", bg="#16a34a", fg="white", font=("Segoe UI", 8, "bold"), command=self._save_map_dialog)
        save_btn.grid(row=0, column=0, sticky="ew", padx=2, pady=2)

        load_btn = tk.Button(btn_grid, text="Load Map", bg="#334155", fg="white", font=("Segoe UI", 8), command=self._load_selected_map)
        load_btn.grid(row=0, column=1, sticky="ew", padx=2, pady=2)

        del_btn = tk.Button(btn_grid, text="Delete Map", bg="#dc2626", fg="white", font=("Segoe UI", 8, "bold"), command=self._delete_selected_map)
        del_btn.grid(row=0, column=2, sticky="ew", padx=2, pady=2)

        btn_grid.columnconfigure(0, weight=1)
        btn_grid.columnconfigure(1, weight=1)
        btn_grid.columnconfigure(2, weight=1)

    def _build_toolbar(self, parent):
        # 2-Row clean header so all controls fit without overflowing
        toolbar_container = tk.Frame(parent, bg="#f1f5f9", padx=8, pady=4)
        toolbar_container.pack(side=tk.TOP, fill=tk.X)

        # ROW 1: Room Dimensions + Tools
        row1 = tk.Frame(toolbar_container, bg="#f1f5f9")
        row1.pack(fill=tk.X, pady=(2, 4))

        tk.Label(row1, text="Room:", font=("Segoe UI", 9, "bold"), bg="#f1f5f9").pack(side=tk.LEFT, padx=(2, 4))
        self.dim_lbl = tk.Label(row1, text=f"{self.room_w_m:.1f}m x {self.room_h_m:.1f}m ({self.cols}x{self.rows})", font=("Segoe UI", 9), bg="#f1f5f9")
        self.dim_lbl.pack(side=tk.LEFT, padx=(0, 6))

        resize_btn = tk.Button(row1, text="Change Size", font=("Segoe UI", 8), command=self._open_resize_dialog)
        resize_btn.pack(side=tk.LEFT, padx=(0, 14))

        tk.Label(row1, text="Tool:", font=("Segoe UI", 9, "bold"), bg="#f1f5f9").pack(side=tk.LEFT, padx=(0, 4))
        tools = [
            ("Obstacle", "brush_obs", "#ef4444"),
            ("Clear", "brush_free", "#0284c7"),
            ("Start (🟢)", "set_start", "#10b981"),
            ("Goal (🔴)", "set_goal", "#f97316"),
        ]
        for text, val, col in tools:
            rb = tk.Radiobutton(
                row1,
                text=text,
                variable=self.current_tool,
                value=val,
                indicatoron=0,
                padx=6,
                pady=1,
                font=("Segoe UI", 8, "bold"),
                selectcolor=col,
                bg="#ffffff"
            )
            rb.pack(side=tk.LEFT, padx=2)

        clear_all_btn = tk.Button(row1, text="Reset Grid", font=("Segoe UI", 8), command=self._reset_grid)
        clear_all_btn.pack(side=tk.RIGHT, padx=2)

        # ROW 2: Calibration Settings + Autopilot Launch
        row2 = tk.Frame(toolbar_container, bg="#f1f5f9")
        row2.pack(fill=tk.X, pady=(2, 2))

        tk.Label(row2, text="Calibration:", font=("Segoe UI", 9, "bold"), bg="#f1f5f9").pack(side=tk.LEFT, padx=(2, 4))
        self.calib_info_lbl = tk.Label(
            row2,
            text=f"Speed: {self.calib['speed_cm_s']:.1f} cm/s | 90° Turn: {self.calib['turn_90_ms']} ms",
            font=("Segoe UI", 8, "bold"),
            fg="#2563eb",
            bg="#f1f5f9"
        )
        self.calib_info_lbl.pack(side=tk.LEFT, padx=4)

        reload_calib_btn = tk.Button(row2, text="🔄 Reload Calibration", font=("Segoe UI", 8), command=self._reload_calibration)
        reload_calib_btn.pack(side=tk.LEFT, padx=6)

        # ROW 3: Car Starting Orientation selector + Autopilot
        row3 = tk.Frame(toolbar_container, bg="#f1f5f9")
        row3.pack(fill=tk.X, pady=(2, 4))

        tk.Label(row3, text="Car Faces:", font=("Segoe UI", 9, "bold"), bg="#f1f5f9").pack(side=tk.LEFT, padx=(2, 4))
        self._heading_var = tk.IntVar(value=0)  # 0=N, 90=E, 180=S, 270=W
        heading_opts = [("⬆ North", 0), ("➡ East", 90), ("⬇ South", 180), ("⬅ West", 270)]
        self._heading_btns = {}
        for label, deg in heading_opts:
            btn = tk.Radiobutton(
                row3, text=label, variable=self._heading_var, value=deg,
                indicatoron=0, padx=6, pady=1,
                font=("Segoe UI", 8, "bold"),
                selectcolor="#7c3aed", fg="white" if deg == 0 else "black",
                bg="#e2e8f0", command=self._on_heading_changed
            )
            btn.pack(side=tk.LEFT, padx=2)
            self._heading_btns[deg] = btn
        self._heading_lbl = tk.Label(row3, text="(set which way car physically faces at START)", font=("Segoe UI", 7), fg="#64748b", bg="#f1f5f9")
        self._heading_lbl.pack(side=tk.LEFT, padx=6)

        # Autopilot Start/Stop button
        self.nav_btn = tk.Button(
            row3,
            text="🚀 START AUTOPILOT",
            bg="#7c3aed",
            fg="white",
            font=("Segoe UI", 9, "bold"),
            padx=14,
            pady=1,
            command=self._toggle_autopilot
        )
        self.nav_btn.pack(side=tk.RIGHT, padx=2)

    def _reload_calibration(self):
        self.calib = load_calibration()
        self.calib_info_lbl.config(text=f"Speed: {self.calib['speed_cm_s']:.1f} cm/s | 90° Turn: {self.calib['turn_90_ms']} ms")
        self.telemetry_pill.config(text=f"🎯 Goal Dist: -- m | Calibrated Speed: {self.calib['speed_cm_s']:.1f} cm/s")
        messagebox.showinfo("Calibration Loaded", f"Loaded settings:\nSpeed: {self.calib['speed_cm_s']:.1f} cm/s\nTurn 90°: {self.calib['turn_90_ms']} ms")

    def _on_heading_changed(self):
        """Called when the user clicks a different Car Faces direction button."""
        deg = self._heading_var.get()
        names = {0: "North ⬆ (top of map)", 90: "East ➡ (right side)", 180: "South ⬇ (bottom)", 270: "West ⬅ (left side)"}
        self._heading_lbl.config(text=f"Car will start facing {names.get(deg, deg)}°")
        # Preview the heading arrow on the map immediately
        self.car_heading = float(deg)
        self._redraw_grid()

    def _build_canvas(self, parent):
        wrap = tk.Frame(parent, bg="#0f172a")
        wrap.pack(fill=tk.BOTH, expand=True, padx=6, pady=6)

        self.canvas = tk.Canvas(wrap, bg="#ffffff", highlightthickness=0)
        self.canvas.pack(fill=tk.BOTH, expand=True)

        self.canvas.bind("<Configure>", lambda e: self._redraw_grid())
        self.canvas.bind("<Button-1>", self._on_canvas_click)
        self.canvas.bind("<B1-Motion>", self._on_canvas_drag)

    # ================= ULTRASONIC SENSOR WATCHDOG =================
    def handle_rx_message(self, line):
        """Processes incoming Bluetooth serial lines from Arduino."""
        line = line.strip()
        if "FAULT:ULTRASONIC_LOOSE" in line:
            self.sonic_pill.config(text="⚠️ Ultrasonic: WIRING LOOSE!", fg="#dc2626")
            if self.is_navigating:
                self._stop_navigation()
        elif line.startswith("BLOCKED:"):
            try:
                cm = int(line.split(":")[1])
                self.handle_ultrasonic_reading(cm)
                self.obstacle_detected_event.set()
                self._handle_dynamic_obstacle(cm)
            except Exception:
                pass
        elif line.startswith("cm="):
            try:
                val = line.split("=")[1].split()[0]
                cm = int(val)
                self.handle_ultrasonic_reading(cm)
                # Fail-safe backup: if car is navigating and confirmed sensor reading drops <= 32cm, emergency brake!
                if self.is_navigating and 0 < cm <= 32:
                    print(f"[Autopilot Ultrasonic Guard] cm={cm} <= 32cm! Emergency brake & re-route.")
                    self.obstacle_detected_event.set()
                    self._handle_dynamic_obstacle(cm)
            except Exception:
                pass

    def handle_ultrasonic_reading(self, cm):
        """Called whenever a distance reading or BLOCKED event arrives."""
        now = time.time()
        self.last_ultrasonic_cm = cm
        self.last_ultrasonic_time = now

        # Check for stuck 0
        if cm == 0:
            self.stuck_zero_count += 1
        else:
            self.stuck_zero_count = 0

        if self.stuck_zero_count >= 8 and not self.ultrasonic_alert_active:
            self.ultrasonic_alert_active = True
            self.sonic_pill.config(text="⚠️ Ultrasonic: WIRING LOOSE (STUCK)!", fg="#dc2626")
            if self.is_navigating:
                self._stop_navigation()
            messagebox.showwarning(
                "Ultrasonic Wire Loose Alert",
                "⚠️ Check Ultrasonic Connectivity!\n\n"
                f"The sensor has been reading {cm} cm continuously (stuck at 0).\n"
                "Please check the Trig, Echo, VCC, and GND jumper wires on the car."
            )
        elif self.stuck_zero_count == 0:
            self.ultrasonic_alert_active = False
            self.sonic_pill.config(text=f"📡 Ultrasonic: {cm} cm (OK)", fg="#10b981")

    # ================= AUTOPILOT NAVIGATION =================
    def _toggle_autopilot(self):
        if self.is_navigating:
            self._stop_navigation()
        else:
            self._start_navigation()

    def _start_navigation(self):
        if not (self.shared_link and self.shared_link.connected):
            messagebox.showerror("Not Connected", "Please connect to the Car via Bluetooth first in the main window.")
            return

        if not self.path:
            messagebox.showwarning("No Path", "No valid path to Goal! Please clear obstacles blocking the way.")
            return

        # Refresh calibration before starting
        self.calib = load_calibration()

        # Reset car heading from the UI selector (user picks which way car physically faces)
        self.car_heading = float(self._heading_var.get())
        self.car_pos = list(self.start_pos)

        self.is_navigating = True
        self._stop_nav.clear()
        self.obstacle_detected_event.clear()
        self.nav_btn.config(text="⏹ STOP AUTOPILOT", bg="#dc2626")
        self.status_lbl.config(text="Autopilot navigating along classroom map...")

        # Switch Arduino to ROUTE mode
        self.shared_link.send(MODE_ROUTE)
        time.sleep(0.20)  # Let Arduino acknowledge ROUTE mode

        self.nav_thread = threading.Thread(target=self._navigation_loop, daemon=True)
        self.nav_thread.start()

    def _stop_navigation(self):
        self.is_navigating = False
        self._stop_nav.set()
        if self.shared_link and self.shared_link.connected:
            self.shared_link.send(CMD_STOP)
        self.nav_btn.config(text="🚀 START AUTOPILOT", bg="#7c3aed")
        self.status_lbl.config(text="Autopilot stopped.")

    def _navigation_loop(self):
        """Drives through A* path using calibrated distance & turn timings.
        Supports dynamic re-routing when an obstacle is detected mid-path.
        """
        MAX_RETRIES = 12
        retries = 0

        while not self._stop_nav.is_set() and retries < MAX_RETRIES:
            # Always re-fetch path (may have been updated by _handle_dynamic_obstacle)
            actions = self._generate_path_actions()
            if not actions:
                # Path is empty — either already at goal or no route
                break
            print(f"[Autopilot] Attempt {retries + 1} — Actions: {actions}")

            obstacle_hit = False
            for act in actions:
                if self._stop_nav.is_set():
                    return  # User pressed STOP — exit immediately

                act_type = act[0]

                if act_type == "FORWARD":
                    ok = self._drive_continuous_segment(
                        num_cells=act[1], dir_r=act[2], dir_c=act[3]
                    )
                    if not ok:
                        # Obstacle detected mid-segment; _handle_dynamic_obstacle has
                        # already updated the grid and recomputed A*.  Wait for the
                        # map to settle then restart the outer loop with the new path.
                        print("[Autopilot] Obstacle hit — waiting for re-route...")
                        time.sleep(0.55)
                        obstacle_hit = True
                        retries += 1
                        break

                elif act_type == "TURN_RIGHT":
                    self._execute_discrete_turn(CMD_RIGHT)
                    self.car_heading = (self.car_heading + 90) % 360
                    self.root.after(0, self._redraw_grid)

                elif act_type == "TURN_LEFT":
                    self._execute_discrete_turn(CMD_LEFT)
                    self.car_heading = (self.car_heading - 90) % 360
                    self.root.after(0, self._redraw_grid)

            if not obstacle_hit:
                # All actions completed without hitting an obstacle — goal reached!
                break

        # Navigation ended — show result if user didn't press STOP
        if not self._stop_nav.is_set():
            # Check if car actually arrived at the Goal location
            cr, cc = self.car_pos
            gr, gc = self.goal_pos
            # Goal is reached if car is within 1 cell of goal
            reached_goal = (abs(cr - gr) <= 1 and abs(cc - gc) <= 1)

            if reached_goal:
                self.root.after(0, lambda: messagebox.showinfo("Goal Reached! 🏁", "Car has reached the Goal!"))
            else:
                self.status_lbl.config(text="⚠️ Cannot reach Goal! All routes blocked by obstacles.", fg="#ef4444")
                self.root.after(0, lambda: messagebox.showwarning("Cannot Reach Goal ⚠️", "Cannot reach the Goal!\nAll routes are blocked by obstacles."))
            self.root.after(0, self._stop_navigation)

    def _generate_path_actions(self):
        """Compresses A* waypoints into heading-aware drive commands.

        Uses the car's actual current heading (self.car_heading) to compute
        correct LEFT / RIGHT / U-TURN actions at each direction change.
        This fixes orientation when the car starts facing South, East, or West,
        and correctly handles 180-degree U-turns in the path.

        Direction convention (grid coords):
          North = heading   0 deg  -> grid dir (-1,  0)  row decreases
          East  = heading  90 deg  -> grid dir ( 0, +1)  col increases
          South = heading 180 deg  -> grid dir (+1,  0)  row increases
          West  = heading 270 deg  -> grid dir ( 0, -1)  col decreases
        """
        if len(self.path) < 2:
            return []

        # Map from grid-step to compass heading in degrees
        DIR_TO_HDG = {
            (-1,  0):   0,   # North
            ( 0,  1):  90,   # East
            ( 1,  0): 180,   # South
            ( 0, -1): 270,   # West
        }

        def turns_needed(from_hdg, to_hdg):
            """Return list of ('TURN_RIGHT',90) / ('TURN_LEFT',90) actions to
            rotate from from_hdg to to_hdg using the fewest 90-degree steps."""
            diff = int((to_hdg - from_hdg) % 360)
            if diff == 0:
                return []
            if diff == 90:
                return [("TURN_RIGHT", 90)]
            if diff == 270:
                return [("TURN_LEFT", 90)]
            # 180 degree U-turn — two right turns (shorter total time)
            return [("TURN_RIGHT", 90), ("TURN_RIGHT", 90)]

        actions = []
        cur_hdg = self.car_heading  # start from actual physical heading

        i = 0
        while i < len(self.path) - 1:
            # Determine direction of next segment
            r0, c0 = self.path[i]
            r1, c1 = self.path[i + 1]
            raw_dr = r1 - r0
            raw_dc = c1 - c0

            # Normalise to cardinal unit step (handles A* diagonals gracefully)
            if raw_dr != 0 and raw_dc != 0:
                # Diagonal step: skip and let A* handle it as two separate moves
                i += 1
                continue

            seg_dir = (max(-1, min(1, raw_dr)), max(-1, min(1, raw_dc)))
            seg_hdg = DIR_TO_HDG.get(seg_dir)
            if seg_hdg is None:
                i += 1
                continue

            # Insert turning actions if the car is not already facing this direction
            for turn_act in turns_needed(cur_hdg, seg_hdg):
                actions.append(turn_act)
            cur_hdg = seg_hdg

            # Count consecutive cells in this same direction
            count = 0
            while i < len(self.path) - 1:
                ra, ca = self.path[i]
                rb, cb = self.path[i + 1]
                step = (max(-1, min(1, rb - ra)), max(-1, min(1, cb - ca)))
                if step != seg_dir:
                    break
                count += 1
                i += 1

            if count > 0:
                actions.append(("FORWARD", count, seg_dir[0], seg_dir[1]))

        return actions

    def _execute_discrete_turn(self, cmd):
        """Executes a calibrated 90-degree pivot turn for the 200 RPM single-motor pivot.

        Pre-stop ensures the car is fully stationary before turning (avoids 'sliding'
        into the turn at speed).  Post-stop settle lets the car stabilise before the
        next forward move, preventing heading drift.
        """
        turn_ms = self.calib.get("turn_90_ms", 960)
        # 1) Hard-stop first so car is stationary (brake + BT latency buffer)
        self.shared_link.send(CMD_STOP)
        time.sleep(0.18)
        # 2) Pivot for calibrated duration
        self.shared_link.send(cmd)
        time.sleep(turn_ms / 1000.0)
        # 3) Stop and let the car fully settle before next action
        self.shared_link.send(CMD_STOP)
        time.sleep(0.38)

    def _handle_dynamic_obstacle(self, cm):
        """Called immediately when Arduino ultrasonic detects an unmapped obstacle.

        Marks cells directly ahead of the car (using current car heading) as obstacles,
        plus upcoming path waypoints, then re-runs A* from the current car position.
        """
        print(f"[Dynamic Obstacle] Detected at {cm} cm! Triggering Re-route...")
        self.status_lbl.config(text=f"⚠️ Obstacle at {cm}cm! Re-planning path...", fg="#ef4444")

        cr, cc = self.car_pos
        marked = 0

        # Method 1: Heading-based front cells (obstacle is directly ahead in driving direction)
        HDG_TO_DIR = {
            0: (-1, 0),    # North -> row decreases
            90: (0, 1),    # East  -> col increases
            180: (1, 0),   # South -> row increases
            270: (0, -1),  # West  -> col decreases
        }
        dr, dc = HDG_TO_DIR.get(int(self.car_heading) % 360, (0, 0))
        for step in (1, 2):
            fr, fc = cr + dr * step, cc + dc * step
            if 0 <= fr < self.rows and 0 <= fc < self.cols:
                if (fr, fc) != self.goal_pos and (fr, fc) != (cr, cc):
                    self.grid[fr][fc] = 1
                    marked += 1

        # Method 2: Also mark the next 1-2 upcoming path waypoints ahead
        for i, p in enumerate(self.path):
            if p == (cr, cc):
                for j in range(i + 1, min(i + 3, len(self.path))):
                    fr, fc = self.path[j]
                    if (fr, fc) != self.goal_pos and (fr, fc) != (cr, cc):
                        self.grid[fr][fc] = 1
                        marked += 1
                break

        print(f"[Dynamic Obstacle] Marked {marked} cell(s) as obstacle ahead of {(cr, cc)} (heading {self.car_heading}°).")

        # Recompute A* path from current car position to the goal
        self.start_pos = (cr, cc)
        self._run_pathfinding()
        self.root.after(0, self._redraw_grid)

    def _drive_continuous_segment(self, num_cells, dir_r=0, dir_c=0):
        """Drives forward smoothly across the full straight segment using calibrated speed."""
        self.obstacle_detected_event.clear()
        self.shared_link.send(CMD_FWD)

        speed_cm_s = max(10.0, self.calib.get("speed_cm_s", 68.0))
        dist_cm = num_cells * self.cell_cm
        total_duration = dist_cm / speed_cm_s

        start_time = time.time()
        end_time = start_time + total_duration
        initial_pos = list(self.car_pos)

        while time.time() < end_time and not self._stop_nav.is_set():
            if self.obstacle_detected_event.is_set():
                # Ultrasonic sensor saw unexpected obstacle! Hard brake
                self.shared_link.send(CMD_STOP)
                self.telemetry_pill.config(text=f"🎯 Goal Dist: -- m | Speed: 0 cm/s")
                return False

            elapsed = time.time() - start_time
            progress_fraction = min(1.0, elapsed / max(0.001, total_duration))
            current_r = initial_pos[0] + (dir_r * num_cells * progress_fraction)
            current_c = initial_pos[1] + (dir_c * num_cells * progress_fraction)
            self.car_pos = [int(round(current_r)), int(round(current_c))]

            # Compute real Euclidean distance to Goal in meters
            gr, gc = self.goal_pos
            dist_cells = math.hypot(gr - current_r, gc - current_c)
            dist_meters = (dist_cells * self.cell_cm) / 100.0

            # Live HUD update
            self.telemetry_pill.config(
                text=f"🎯 Goal Dist: {dist_meters:.1f} m | Speed: {speed_cm_s:.1f} cm/s"
            )
            self._redraw_grid()
            time.sleep(0.04)

        self.shared_link.send(CMD_STOP)
        gr, gc = self.goal_pos
        dist_cells = math.hypot(gr - self.car_pos[0], gc - self.car_pos[1])
        dist_meters = (dist_cells * self.cell_cm) / 100.0
        self.telemetry_pill.config(text=f"🎯 Goal Dist: {dist_meters:.1f} m | Speed: 0 cm/s")
        time.sleep(0.12)
        return True

    def _auto_detect_grid_from_photo(self):
        """Analyzes the current photo and automatically populates the 2D obstacle grid."""
        if not self.photos:
            messagebox.showinfo("No Photo", "Please upload at least one room photo first using '+ Upload Photos'.")
            return

        if cv2 is None or np is None:
            messagebox.showerror("Missing OpenCV", "OpenCV / numpy is required for auto-detection.")
            return

        photo_path = self.photos[self.current_photo_idx]
        img = cv2.imread(photo_path)
        if img is None:
            messagebox.showerror("Error", "Could not read image file.")
            return

        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        h, w, _ = img.shape
        floor_sample = hsv[int(h * 0.6):int(h * 0.9), int(w * 0.3):int(w * 0.7)]
        mean_h = np.median(floor_sample[:, :, 0])
        mean_s = np.median(floor_sample[:, :, 1])
        mean_v = np.median(floor_sample[:, :, 2])

        lower_floor = np.array([max(0, mean_h - 25), max(30, mean_s - 60), max(30, mean_v - 70)])
        upper_floor = np.array([min(180, mean_h + 25), min(255, mean_s + 60), min(255, mean_v + 70)])

        floor_mask = cv2.inRange(hsv, lower_floor, upper_floor)
        obstacle_mask = cv2.bitwise_not(floor_mask)

        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (15, 15))
        obstacle_mask = cv2.morphologyEx(obstacle_mask, cv2.MORPH_CLOSE, kernel)
        obstacle_mask = cv2.morphologyEx(obstacle_mask, cv2.MORPH_OPEN, kernel)

        grid_thumb = cv2.resize(obstacle_mask, (self.cols, self.rows), interpolation=cv2.INTER_AREA)

        for r in range(self.rows):
            for c in range(self.cols):
                if (r, c) == self.start_pos or (r, c) == self.goal_pos:
                    self.grid[r][c] = 0
                else:
                    self.grid[r][c] = 1 if grid_thumb[r, c] > 100 else 0

        self._run_pathfinding()
        self._redraw_grid()
        self.status_lbl.config(
            text="⚡ Auto-detected obstacles from photo! (Use Brush/Clear to touch up if needed)",
            fg="#10b981"
        )
        messagebox.showinfo(
            "Auto-Grid Generated",
            "Auto-generated obstacles from your photo!\n\n"
            "You can use the 'Obstacle' or 'Clear' tools to fine-tune any cells before starting Autopilot."
        )

    # ================= PHOTO VIEWER =================
    def _upload_photos(self):
        files = filedialog.askopenfilenames(
            title="Select Classroom Photos",
            filetypes=[("Image Files", "*.jpg *.jpeg *.png *.bmp *.webp"), ("All Files", "*.*")]
        )
        if not files:
            return
        self.photos.extend(files)
        self.current_photo_idx = len(self.photos) - 1
        self._update_photo_view()
        self.status_lbl.config(text=f"Loaded {len(self.photos)} photo(s).")

    def _clear_photos(self):
        self.photos.clear()
        self.current_photo_idx = 0
        self._update_photo_view()

    def _prev_photo(self):
        if self.current_photo_idx > 0:
            self.current_photo_idx -= 1
            self._update_photo_view()

    def _next_photo(self):
        if self.current_photo_idx < len(self.photos) - 1:
            self.current_photo_idx += 1
            self._update_photo_view()

    def _update_photo_view(self):
        self.photo_canvas.delete("all")
        if not self.photos or Image is None:
            self.photo_counter_lbl.config(text="No photos added")
            self.prev_photo_btn.config(state=tk.DISABLED)
            self.next_photo_btn.config(state=tk.DISABLED)
            self.photo_canvas.create_text(
                self.photo_canvas.winfo_width() // 2 or 120,
                self.photo_canvas.winfo_height() // 2 or 90,
                text="Upload photos of\nclassroom desks/layout\nto reference here",
                fill="#64748b",
                justify=tk.CENTER,
                font=("Segoe UI", 9)
            )
            return

        idx = self.current_photo_idx
        path = self.photos[idx]
        self.photo_counter_lbl.config(text=f"Photo {idx + 1} of {len(self.photos)}\n{os.path.basename(path)}")
        self.prev_photo_btn.config(state=tk.NORMAL if idx > 0 else tk.DISABLED)
        self.next_photo_btn.config(state=tk.NORMAL if idx < len(self.photos) - 1 else tk.DISABLED)

        try:
            pil_img = Image.open(path)
            cw = max(100, self.photo_canvas.winfo_width())
            ch = max(100, self.photo_canvas.winfo_height())
            pil_img.thumbnail((cw, ch), Image.Resampling.LANCZOS)
            self.photo_tk_cache = ImageTk.PhotoImage(pil_img)
            self.photo_canvas.create_image(cw // 2, ch // 2, image=self.photo_tk_cache)
        except Exception as e:
            self.photo_canvas.create_text(100, 50, text=f"Error loading image:\n{e}", fill="red")

    # ================= SAVED MAPS MANAGER =================
    def _refresh_saved_maps_list(self):
        self.maps_listbox.delete(0, tk.END)
        if not os.path.exists(MAPS_DIR):
            return
        files = [f for f in os.listdir(MAPS_DIR) if f.endswith(".json")]
        for f in sorted(files):
            self.maps_listbox.insert(tk.END, f[:-5])

    def _save_map_dialog(self):
        name = simple_input_dialog(self.root, "Save Map", "Enter a name for this classroom map:")
        if not name or not name.strip():
            return
        name = "".join(c for c in name.strip() if c.isalnum() or c in ("_", "-", " "))
        filepath = os.path.join(MAPS_DIR, f"{name}.json")

        data = {
            "room_width_m": self.room_w_m,
            "room_height_m": self.room_h_m,
            "cell_cm": self.cell_cm,
            "cols": self.cols,
            "rows": self.rows,
            "start": list(self.start_pos),
            "goal": list(self.goal_pos),
            "grid": self.grid,
            "photos": [os.path.abspath(p) for p in self.photos if os.path.exists(p)]
        }

        try:
            with open(filepath, "w", encoding="utf-8") as f:
                json.dump(data, f, indent=2)
            self._refresh_saved_maps_list()
            self.status_lbl.config(text=f"Saved map: '{name}'")
            messagebox.showinfo("Map Saved", f"Successfully saved classroom map '{name}'!")
        except Exception as e:
            messagebox.showerror("Save Error", f"Failed to save map: {e}")

    def _load_selected_map(self):
        sel = self.maps_listbox.curselection()
        if not sel:
            messagebox.showwarning("No Selection", "Please select a map from the list to load.")
            return
        map_name = self.maps_listbox.get(sel[0])
        filepath = os.path.join(MAPS_DIR, f"{map_name}.json")
        if not os.path.exists(filepath):
            messagebox.showerror("Error", f"File '{filepath}' not found.")
            return

        try:
            with open(filepath, "r", encoding="utf-8") as f:
                data = json.load(f)

            self.room_w_m = float(data.get("room_width_m", DEFAULT_ROOM_WIDTH_M))
            self.room_h_m = float(data.get("room_height_m", DEFAULT_ROOM_HEIGHT_M))
            self.cell_cm = float(data.get("cell_cm", DEFAULT_CELL_SIZE_CM))
            self.cols = int(data.get("cols", len(data["grid"][0])))
            self.rows = int(data.get("rows", len(data["grid"])))
            self.grid = data["grid"]
            self.start_pos = tuple(data.get("start", (1, 1)))
            self.goal_pos = tuple(data.get("goal", (self.rows - 2, self.cols - 2)))
            self.car_pos = list(self.start_pos)

            loaded_photos = [p for p in data.get("photos", []) if os.path.exists(p)]
            if loaded_photos:
                self.photos = loaded_photos
                self.current_photo_idx = 0
                self._update_photo_view()

            self.dim_lbl.config(text=f"{self.room_w_m:.1f}m x {self.room_h_m:.1f}m ({self.cols}x{self.rows})")
            self._run_pathfinding()
            self._redraw_grid()
            self.status_lbl.config(text=f"Loaded map: '{map_name}'")
        except Exception as e:
            messagebox.showerror("Load Error", f"Failed to load map: {e}")

    def _delete_selected_map(self):
        sel = self.maps_listbox.curselection()
        if not sel:
            messagebox.showwarning("No Selection", "Please select a map to delete.")
            return
        map_name = self.maps_listbox.get(sel[0])
        filepath = os.path.join(MAPS_DIR, f"{map_name}.json")

        confirm = messagebox.askyesno(
            "Confirm Delete",
            f"Are you sure you want to permanently delete map '{map_name}'?"
        )
        if not confirm:
            return

        try:
            if os.path.exists(filepath):
                os.remove(filepath)
            self._refresh_saved_maps_list()
            self.status_lbl.config(text=f"Deleted map '{map_name}'")
        except Exception as e:
            messagebox.showerror("Delete Error", f"Failed to delete map: {e}")

    # ================= GRID INTERACTION & RESIZE =================
    def _open_resize_dialog(self):
        dlg = tk.Toplevel(self.root)
        dlg.title("Configure Classroom Size")
        dlg.geometry("320x220")
        dlg.resizable(False, False)
        dlg.grab_set()

        tk.Label(dlg, text="Classroom Dimensions & Resolution", font=("Segoe UI", 9, "bold")).pack(pady=8)
        f = tk.Frame(dlg)
        f.pack(pady=4)

        tk.Label(f, text="Width (meters):").grid(row=0, column=0, sticky="e", padx=4, pady=3)
        w_ent = tk.Entry(f, width=10)
        w_ent.insert(0, str(self.room_w_m))
        w_ent.grid(row=0, column=1, padx=4, pady=3)

        tk.Label(f, text="Length (meters):").grid(row=1, column=0, sticky="e", padx=4, pady=3)
        h_ent = tk.Entry(f, width=10)
        h_ent.insert(0, str(self.room_h_m))
        h_ent.grid(row=1, column=1, padx=4, pady=3)

        tk.Label(f, text="Cell Size (cm):").grid(row=2, column=0, sticky="e", padx=4, pady=3)
        c_ent = tk.Entry(f, width=10)
        c_ent.insert(0, str(self.cell_cm))
        c_ent.grid(row=2, column=1, padx=4, pady=3)

        def _apply():
            try:
                nw = float(w_ent.get())
                nh = float(h_ent.get())
                nc = float(c_ent.get())
                if nw <= 1.0 or nh <= 1.0 or nc < 10.0:
                    messagebox.showerror("Invalid Input", "Classroom must be >1m, cell >=10cm.")
                    return
                self.room_w_m = nw
                self.room_h_m = nh
                self.cell_cm = nc
                self.cols = int(round((self.room_w_m * 100.0) / self.cell_cm))
                self.rows = int(round((self.room_h_m * 100.0) / self.cell_cm))
                self.grid = [[0 for _ in range(self.cols)] for _ in range(self.rows)]
                self.start_pos = (1, 1)
                self.goal_pos = (max(1, self.rows - 2), max(1, self.cols - 2))
                self.car_pos = list(self.start_pos)
                self.dim_lbl.config(text=f"{self.room_w_m:.1f}m x {self.room_h_m:.1f}m ({self.cols}x{self.rows})")
                self._run_pathfinding()
                self._redraw_grid()
                dlg.destroy()
            except ValueError:
                messagebox.showerror("Error", "Invalid numeric values entered.")

        tk.Button(dlg, text="Apply & Reset Grid", bg="#2563eb", fg="white", font=("Segoe UI", 9, "bold"), command=_apply).pack(pady=12)

    def _reset_grid(self):
        if messagebox.askyesno("Reset Grid", "Clear all obstacles on the grid?"):
            self.grid = [[0 for _ in range(self.cols)] for _ in range(self.rows)]
            self._run_pathfinding()
            self._redraw_grid()

    def _cell_from_event(self, event):
        cw = self.canvas.winfo_width()
        ch = self.canvas.winfo_height()
        if cw <= 0 or ch <= 0:
            return None
        cell_w = cw / self.cols
        cell_h = ch / self.rows
        c = int(event.x // cell_w)
        r = int(event.y // cell_h)
        if 0 <= r < self.rows and 0 <= c < self.cols:
            return (r, c)
        return None

    def _on_canvas_click(self, event):
        cell = self._cell_from_event(event)
        if not cell:
            return
        r, c = cell
        tool = self.current_tool.get()

        if tool == "set_start":
            self.start_pos = (r, c)
            self.car_pos = [r, c]
            self.grid[r][c] = 0
        elif tool == "set_goal":
            self.goal_pos = (r, c)
            self.grid[r][c] = 0
        elif tool == "brush_obs":
            self.grid[r][c] = 1
        elif tool == "brush_free":
            self.grid[r][c] = 0

        self._run_pathfinding()
        self._redraw_grid()

    def _on_canvas_drag(self, event):
        cell = self._cell_from_event(event)
        if not cell:
            return
        r, c = cell
        tool = self.current_tool.get()
        if tool == "brush_obs":
            if (r, c) != self.start_pos and (r, c) != self.goal_pos:
                self.grid[r][c] = 1
                self._run_pathfinding()
                self._redraw_grid()
        elif tool == "brush_free":
            self.grid[r][c] = 0
            self._run_pathfinding()
            self._redraw_grid()

    # ================= A* PATHFINDING =================
    def _run_pathfinding(self):
        start = self.start_pos
        goal = self.goal_pos

        if self.grid[start[0]][start[1]] == 1 or self.grid[goal[0]][goal[1]] == 1:
            self.path = []
            self.status_lbl.config(text="Warning: Start or Goal is inside an obstacle!")
            return

        def heuristic(a, b):
            return math.hypot(a[0] - b[0], a[1] - b[1])

        open_set = []
        heapq.heappush(open_set, (0, start))
        came_from = {}
        g_score = {start: 0}
        f_score = {start: heuristic(start, goal)}

        neighbors = [
            (-1, 0, 1.0), (1, 0, 1.0), (0, -1, 1.0), (0, 1, 1.0),
            # Diagonal moves removed: the car can only drive forward/pivot turn.
            # Allowing diagonals produces path steps the motor has no command for.
        ]

        found = False
        while open_set:
            _, current = heapq.heappop(open_set)

            if current == goal:
                found = True
                break

            for dr, dc, cost in neighbors:
                nr, nc = current[0] + dr, current[1] + dc
                if 0 <= nr < self.rows and 0 <= nc < self.cols:
                    if self.grid[nr][nc] == 1:
                        continue
                    # No diagonal check needed (diagonal moves removed)

                    tentative_g = g_score[current] + cost
                    if (nr, nc) not in g_score or tentative_g < g_score[(nr, nc)]:
                        came_from[(nr, nc)] = current
                        g_score[(nr, nc)] = tentative_g
                        f_score[(nr, nc)] = tentative_g + heuristic((nr, nc), goal)
                        heapq.heappush(open_set, (f_score[(nr, nc)], (nr, nc)))

        if found:
            curr = goal
            path = []
            while curr in came_from:
                path.append(curr)
                curr = came_from[curr]
            path.append(start)
            path.reverse()
            self.path = path
            path_len_m = (len(path) * self.cell_cm) / 100.0
            self.status_lbl.config(text=f"A* Path: {len(path)} waypoints (~{path_len_m:.1f} m)")
        else:
            self.path = []
            self.status_lbl.config(text="No path to Goal! Obstacles block every way.")

    # ================= RENDER GRID, COMPASS & PATH =================
    def _redraw_grid(self):
        self.canvas.delete("all")
        cw = self.canvas.winfo_width()
        ch = self.canvas.winfo_height()
        if cw <= 10 or ch <= 10:
            return

        cell_w = cw / self.cols
        cell_h = ch / self.rows

        # 1. Obstacles
        for r in range(self.rows):
            for c in range(self.cols):
                if self.grid[r][c] == 1:
                    x0, y0 = c * cell_w, r * cell_h
                    x1, y1 = x0 + cell_w, y0 + cell_h
                    self.canvas.create_rectangle(x0, y0, x1, y1, fill="#1e293b", outline="#334155")

        # 2. Grid lines
        for c in range(self.cols + 1):
            x = c * cell_w
            self.canvas.create_line(x, 0, x, ch, fill="#f1f5f9", width=1)
        for r in range(self.rows + 1):
            y = r * cell_h
            self.canvas.create_line(0, y, cw, y, fill="#f1f5f9", width=1)

        # 3. Path
        if len(self.path) > 1:
            pts = []
            for r, c in self.path:
                pts.append(c * cell_w + cell_w / 2)
                pts.append(r * cell_h + cell_h / 2)
            self.canvas.create_line(*pts, fill="#eab308", width=4, joinstyle=tk.ROUND, smooth=True)

        # 4. Start & Goal
        sr, sc = self.start_pos
        sx, sy = sc * cell_w, sr * cell_h
        self.canvas.create_rectangle(sx + 2, sy + 2, sx + cell_w - 2, sy + cell_h - 2, fill="#10b981", outline="#059669", width=2)
        self.canvas.create_text(sx + cell_w / 2, sy + cell_h / 2, text="START", font=("Segoe UI", 8, "bold"), fill="white")

        gr, gc = self.goal_pos
        gx, gy = gc * cell_w, gr * cell_h
        self.canvas.create_rectangle(gx + 2, gy + 2, gx + cell_w - 2, gy + cell_h - 2, fill="#ef4444", outline="#dc2626", width=2)
        self.canvas.create_text(gx + cell_w / 2, gy + cell_h / 2, text="GOAL", font=("Segoe UI", 8, "bold"), fill="white")

        # 5. Live Car Marker with heading arrow
        cr, cc = self.car_pos
        cx, cy = cc * cell_w + cell_w / 2, cr * cell_h + cell_h / 2
        radius = min(cell_w, cell_h) * 0.45
        self.canvas.create_oval(
            cx - radius, cy - radius, cx + radius, cy + radius,
            fill="#38bdf8", outline="#0284c7", width=2
        )
        # Heading arrow: 0°=North=up, 90°=East=right, 180°=South=down, 270°=West=left
        # Convert heading to canvas angle (canvas Y increases downward)
        rad = math.radians(self.car_heading - 90)  # rotate so 0° points up
        ax = cx + radius * math.cos(rad)
        ay = cy + radius * math.sin(rad)
        self.canvas.create_line(
            cx, cy, ax, ay,
            fill="#dc2626", width=3, arrow=tk.LAST
        )
        self.canvas.create_text(
            cx, cy - radius - 9,
            text=f"{int(self.car_heading)}°",
            font=("Segoe UI", 7, "bold"),
            fill="#dc2626"
        )

        # 6. CARDINAL DIRECTIONS: NORTH, SOUTH, EAST, WEST
        # Top banner: NORTH
        self.canvas.create_text(cw / 2, 14, text="▲ NORTH (Front of Classroom)", font=("Segoe UI", 10, "bold"), fill="#2563eb")
        # Bottom banner: SOUTH
        self.canvas.create_text(cw / 2, ch - 14, text="▼ SOUTH (Back of Classroom)", font=("Segoe UI", 10, "bold"), fill="#64748b")
        # Left banner: WEST
        self.canvas.create_text(24, ch / 2, text="◄ WEST", font=("Segoe UI", 10, "bold"), fill="#64748b")
        # Right banner: EAST
        self.canvas.create_text(cw - 24, ch / 2, text="EAST ►", font=("Segoe UI", 10, "bold"), fill="#64748b")

        # 7. Compass Rose Badge in top-right corner
        rose_cx, rose_cy = cw - 50, 48
        r_radius = 24
        self.canvas.create_oval(rose_cx - r_radius, rose_cy - r_radius, rose_cx + r_radius, rose_cy + r_radius, fill="#0f172a", outline="#38bdf8", width=2)
        # North arrow (red)
        self.canvas.create_line(rose_cx, rose_cy, rose_cx, rose_cy - 18, fill="#ef4444", width=3, arrow=tk.LAST)
        self.canvas.create_text(rose_cx, rose_cy - 10, text="N", fill="#ef4444", font=("Segoe UI", 8, "bold"))
        # South line
        self.canvas.create_line(rose_cx, rose_cy, rose_cx, rose_cy + 16, fill="#94a3b8", width=1.5)
        self.canvas.create_text(rose_cx, rose_cy + 10, text="S", fill="#94a3b8", font=("Segoe UI", 7, "bold"))
        # East / West line
        self.canvas.create_line(rose_cx - 16, rose_cy, rose_cx + 16, rose_cy, fill="#94a3b8", width=1.5)
        self.canvas.create_text(rose_cx + 11, rose_cy, text="E", fill="#94a3b8", font=("Segoe UI", 7, "bold"))
        self.canvas.create_text(rose_cx - 11, rose_cy, text="W", fill="#94a3b8", font=("Segoe UI", 7, "bold"))


def simple_input_dialog(parent, title, prompt):
    dlg = tk.Toplevel(parent)
    dlg.title(title)
    dlg.geometry("300x130")
    dlg.resizable(False, False)
    dlg.grab_set()

    tk.Label(dlg, text=prompt, font=("Segoe UI", 9)).pack(pady=(12, 6))
    entry = tk.Entry(dlg, width=28, font=("Segoe UI", 10))
    entry.pack(pady=4)
    entry.focus_set()

    res = [None]

    def _ok():
        res[0] = entry.get()
        dlg.destroy()

    def _cancel():
        dlg.destroy()

    btn_row = tk.Frame(dlg)
    btn_row.pack(pady=8)
    tk.Button(btn_row, text="OK", width=8, bg="#2563eb", fg="white", command=_ok).pack(side=tk.LEFT, padx=4)
    tk.Button(btn_row, text="Cancel", width=8, command=_cancel).pack(side=tk.LEFT, padx=4)

    entry.bind("<Return>", lambda e: _ok())
    entry.bind("<Escape>", lambda e: _cancel())
    parent.wait_window(dlg)
    return res[0]
