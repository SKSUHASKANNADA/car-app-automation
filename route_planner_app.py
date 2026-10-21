"""route_planner_app.py - overhead-camera A* route planner for RC car.
Supports 2-marker mode:
  Marker ID 10: Car position & heading (top edge faces front)
  Marker ID 20: Goal position
Obstacle detection: Auto-threshold from overhead picture/stream + manual brush paint/erase.
Can run standalone OR be launched as Goal Maze Mode from car_control_app.py sharing the serial link.
"""
import math
import os
import queue
import threading
import time
import tkinter as tk
from datetime import datetime
from tkinter import filedialog, ttk

import numpy as np

try:
    import cv2
except ImportError:
    cv2 = None
try:
    from PIL import Image, ImageTk
except ImportError:
    Image = None
    ImageTk = None

from car_control_app import (BAUD_RATE, SerialManager, list_ports, probe_port_once,
                             HC05_HWID_HINT)

# ============================== CONFIG ==============================
# Car dimensions: Length 18 cm, Width 22 cm, Height 12 cm.
# Half diagonal = sqrt((18/2)^2 + (22/2)^2) = sqrt(81 + 121) = ~14.2 cm
DEFAULT_CAR_RADIUS_CM = 14.0
DEFAULT_MARGIN_CM = 4.0

ARENA_W_CM = 200.0      # virtual arena width (cm)
ARENA_H_CM = 150.0      # virtual arena height (cm)
CELL_CM = 4.0           # occupancy grid cell size in cm

WAYPOINT_TOL_CM = 10.0
GOAL_TOL_CM = 12.0
TURN_THRESHOLD_DEG = 22.0

TURN_PULSE_MS = 200
FORWARD_PULSE_MS = 320
SETTLE_MS = 300
LOOP_HZ = 5

POSE_MAX_AGE_S = 0.5    # pose older than this = stale -> stop
MARKER_LOST_S = 0.8     # car marker unseen this long -> stop
NO_PROGRESS_S = 60.0

CMD_FWD, CMD_RIGHT, CMD_LEFT, CMD_STOP = "8", "4", "6", "5"
MODE_MANUAL, MODE_AUTO, MODE_ROUTE = "M", "A", "R"

TURN_SIGN = 1           # flip to -1 if car turns opposite direction

ARUCO_DICT_ID = cv2.aruco.DICT_4X4_50 if cv2 is not None else None
CAR_ID = 10
GOAL_ID = 20

VIEW_W, VIEW_H = 640, 480
RECT_W, RECT_H = VIEW_W, VIEW_H
GW = int(math.ceil(ARENA_W_CM / CELL_CM))
GH = int(math.ceil(ARENA_H_CM / CELL_CM))


def wrap180(a):
    while a > 180:
        a -= 360
    while a <= -180:
        a += 360
    return a


def px_to_cm(px, py):
    return (px / RECT_W) * ARENA_W_CM, (py / RECT_H) * ARENA_H_CM


def cm_to_px(x_cm, y_cm):
    return (x_cm / ARENA_W_CM) * RECT_W, (y_cm / ARENA_H_CM) * RECT_H


def cm_to_grid(x_cm, y_cm):
    gx = int(x_cm / CELL_CM)
    gy = int(y_cm / CELL_CM)
    gx = max(0, min(GW - 1, gx))
    gy = max(0, min(GH - 1, gy))
    return gx, gy


def grid_to_cm(gx, gy):
    return (gx + 0.5) * CELL_CM, (gy + 0.5) * CELL_CM


# ---------- ArUco Detection ----------
_ARUCO_CACHE = {}


def get_aruco():
    if cv2 is None:
        raise RuntimeError("OpenCV not installed. pip install opencv-contrib-python")
    if "ok" in _ARUCO_CACHE:
        return _ARUCO_CACHE["ok"]
    ar = cv2.aruco
    dictionary = ar.getPredefinedDictionary(ARUCO_DICT_ID)
    params = None
    detector = None
    try:
        if hasattr(ar, "DetectorParameters"):
            params = ar.DetectorParameters()
        elif hasattr(ar, "DetectorParameters_create"):
            params = ar.DetectorParameters_create()
    except Exception:
        params = None
    try:
        if hasattr(ar, "ArucoDetector") and params is not None:
            detector = ar.ArucoDetector(dictionary, params)
    except Exception:
        detector = None
    _ARUCO_CACHE["ok"] = (dictionary, params, detector)
    return dictionary, params, detector


def detect_markers_both(bgr):
    dictionary, params, detector = get_aruco()
    gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
    try:
        if detector is not None:
            corners, ids, _ = detector.detectMarkers(gray)
        else:
            if params is not None:
                corners, ids, _ = cv2.aruco.detectMarkers(gray, dictionary, parameters=params)
            else:
                corners, ids, _ = cv2.aruco.detectMarkers(gray, dictionary)
    except Exception:
        return {}
    out = {}
    if ids is None or len(ids) == 0:
        return out
    for c, i in zip(corners, ids.flatten().tolist()):
        out[int(i)] = np.asarray(c).reshape(4, 2).astype(np.float32)
    return out


def pose_from_marker(corners):
    cx = float(corners[:, 0].mean())
    cy = float(corners[:, 1].mean())
    tx = float((corners[0, 0] + corners[1, 0]) * 0.5)
    ty = float((corners[0, 1] + corners[1, 1]) * 0.5)
    dx, dy = tx - cx, ty - cy
    heading = math.degrees(math.atan2(dy, dx))
    return cx, cy, wrap180(heading)


# ---------- Camera Thread ----------
class CameraThread:
    def __init__(self):
        self.cap = None
        self._lock = threading.Lock()
        self._frame = None
        self._ts = 0.0
        self.running = False
        self._thread = None
        self.source_str = ""

    def start(self, source_str):
        self.stop()
        self.source_str = source_str
        s = source_str.strip()
        if s.isdigit():
            src = int(s)
        else:
            # IP Webcam stream normalization:
            # 1. OpenCV struggles with self-signed HTTPS on IP Webcam -> use http://
            if s.startswith("https://"):
                s = "http://" + s[len("https://"):]
            elif not s.startswith("http://") and not s.startswith("rtsp://") and not os.path.exists(s):
                s = "http://" + s
            # 2. Append /video endpoint if given raw root IP/port (e.g. http://192.168.137.21:8080 -> /video)
            if (s.startswith("http://") or s.startswith("https://")) and not s.endswith("/video") and not s.endswith(".mjpg"):
                if s.endswith("/"):
                    s = s + "video"
                else:
                    s = s + "/video"
            src = s

        cap = cv2.VideoCapture(src)
        try:
            cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        except Exception:
            pass
        if not cap.isOpened():
            raise OSError("Cannot open camera source: %s (resolved to %s)" % (source_str, src))
        self.cap = cap
        self.running = True
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()

    def set_static_frame(self, frame):
        self.stop()
        with self._lock:
            self._frame = frame
            self._ts = time.time()

    def _loop(self):
        while self.running and self.cap is not None:
            try:
                ok, frame = self.cap.read()
            except Exception:
                ok, frame = False, None
            if ok and frame is not None:
                with self._lock:
                    self._frame = frame
                    self._ts = time.time()
            else:
                time.sleep(0.02)

    def get(self):
        with self._lock:
            f = None if self._frame is None else self._frame.copy()
            return f, self._ts

    def stop(self):
        self.running = False
        if self._thread is not None:
            self._thread.join(timeout=0.5)
            self._thread = None
        if self.cap is not None:
            try:
                self.cap.release()
            except Exception:
                pass
            self.cap = None


# ---------- Occupancy Map & Auto-Obstacle Extractor ----------
class OccMap:
    def __init__(self):
        self.grid = np.zeros((GH, GW), dtype=np.uint8)
        self.car_radius_cm = DEFAULT_CAR_RADIUS_CM
        self.margin_cm = DEFAULT_MARGIN_CM

    def clear(self):
        self.grid[:] = 0

    def paint_cm(self, x_cm, y_cm, erase=False, radius_cm=8.0):
        gx, gy = cm_to_grid(x_cm, y_cm)
        r = max(1, int(round(radius_cm / CELL_CM)))
        for dy in range(-r, r + 1):
            for dx in range(-r, r + 1):
                if dx * dx + dy * dy > r * r:
                    continue
                xx, yy = gx + dx, gy + dy
                if 0 <= xx < GW and 0 <= yy < GH:
                    self.grid[yy, xx] = 0 if erase else 1

    def auto_extract_obstacles(self, bgr_img, threshold_val=100, invert=False):
        if bgr_img is None:
            return
        gray = cv2.cvtColor(bgr_img, cv2.COLOR_BGR2GRAY)
        blur = cv2.GaussianBlur(gray, (5, 5), 0)
        if invert:
            _, binary = cv2.threshold(blur, threshold_val, 255, cv2.THRESH_BINARY)
        else:
            _, binary = cv2.threshold(blur, threshold_val, 255, cv2.THRESH_BINARY_INV)

        small = cv2.resize(binary, (GW, GH), interpolation=cv2.INTER_AREA)
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (2, 2))
        opened = cv2.morphologyEx((small > 100).astype(np.uint8), cv2.MORPH_OPEN, kernel)
        self.grid = opened.astype(np.uint8)

    def load_png(self, path):
        img = cv2.imread(path, cv2.IMREAD_GRAYSCALE)
        if img is None:
            raise OSError("Cannot read map image: %s" % path)
        small = cv2.resize(img, (GW, GH), interpolation=cv2.INTER_NEAREST)
        self.grid = (small < 128).astype(np.uint8)

    def save_png(self, path):
        img = np.where(self.grid > 0, 0, 255).astype(np.uint8)
        big = cv2.resize(img, (RECT_W, RECT_H), interpolation=cv2.INTER_NEAREST)
        cv2.imwrite(path, big)

    def inflated(self):
        total_r = self.car_radius_cm + self.margin_cm
        r_cells = int(math.ceil(total_r / CELL_CM))
        if r_cells <= 0:
            return self.grid.copy()
        k = 2 * r_cells + 1
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (k, k))
        return cv2.dilate(self.grid, kernel).astype(np.uint8)


# ---------- A* Algorithm ----------
def astar(infl, start, goal):
    if infl[goal[1], goal[0]] or infl[start[1], start[0]]:
        return None
    import heapq
    SQ2 = math.sqrt(2.0)

    def h(a, b):
        dx, dy = abs(a[0] - b[0]), abs(a[1] - b[1])
        return max(dx, dy) + (SQ2 - 1) * min(dx, dy)

    openh = [(h(start, goal), 0.0, start)]
    came = {start: None}
    gs = {start: 0.0}
    closed = set()
    nbrs = [(1, 0, 1.0), (-1, 0, 1.0), (0, 1, 1.0), (0, -1, 1.0),
            (1, 1, SQ2), (1, -1, SQ2), (-1, 1, SQ2), (-1, -1, SQ2)]
    while openh:
        _, g, cur = heapq.heappop(openh)
        if cur in closed:
            continue
        if cur == goal:
            path = []
            while cur is not None:
                path.append(cur)
                cur = came[cur]
            path.reverse()
            return path
        closed.add(cur)
        for dx, dy, c in nbrs:
            nx, ny = cur[0] + dx, cur[1] + dy
            if not (0 <= nx < GW and 0 <= ny < GH):
                continue
            if infl[ny, nx]:
                continue
            if dx != 0 and dy != 0:
                if infl[cur[1], nx] or infl[ny, cur[0]]:
                    continue
            ng = g + c
            key = (nx, ny)
            if key in gs and gs[key] <= ng:
                continue
            gs[key] = ng
            came[key] = cur
            heapq.heappush(openh, (ng + h(key, goal), ng, key))
    return None


def los_free(infl, a, b):
    x0, y0, x1, y1 = a[0], a[1], b[0], b[1]
    dx, dy = abs(x1 - x0), abs(y1 - y0)
    sx, sy = (1 if x1 > x0 else -1), (1 if y1 > y0 else -1)
    x, y = x0, y0
    if dx >= dy:
        err = dx / 2.0
        while x != x1:
            if infl[y, x]:
                return False
            err -= dy
            if err < 0:
                y += sy
                err += dx
            x += sx
    else:
        err = dy / 2.0
        while y != y1:
            if infl[y, x]:
                return False
            err -= dx
            if err < 0:
                x += sx
                err += dy
            y += sy
    return not bool(infl[y1, x1])


def simplify_path(cells, infl):
    if cells is None or len(cells) <= 2:
        return cells
    out = [cells[0]]
    i = 0
    while i < len(cells) - 1:
        j = len(cells) - 1
        while j > i + 1 and not los_free(infl, cells[i], cells[j]):
            j -= 1
        out.append(cells[j])
        i = j
    return out


# ============================== ROUTE PLANNER APP ==============================
class RoutePlannerApp:
    def __init__(self, root, shared_link=None):
        self.root = root
        self.root.title("Goal Maze & Route Planner (2 Markers: Car 10 & Goal 20)")
        self.events = queue.Queue()
        self.is_embedded = (shared_link is not None)
        self.link = shared_link
        if self.link is None:
            try:
                from car_control_app import SerialManager as _SM
                self.link = _SM(self.events)
            except Exception:
                self.link = None

        self.cam = CameraThread()
        self.omap = OccMap()
        self._vlock = threading.Lock()
        self.pose = None
        self.pose_ts = 0.0
        self.last_seen_car_ts = 0.0
        self.goal_detected_cm = None
        self.last_seen_goal_ts = 0.0
        self.last_frame = None

        self._rlock = threading.Lock()
        self.goal_cm = None
        self.path_cm = []
        self.path_cells = None
        self.infl_cache = None

        self.driving = False
        self._drive_thread = None
        self._cancel = threading.Event()
        self._paused_blocked = False
        self._start_time = 0.0
        self._last_progress = 0.0
        self._last_progress_pos = None

        self.root.configure(bg="#12161f")
        self._build_ui()
        if not self.is_embedded:
            self._refresh_ports()
        else:
            self._set_ser(self.link.connected if self.link else False)

        self.log("Ready! Tripod overhead camera mode enabled.")
        self.log("Marker IDs: Car = 10, Goal = 20. Car size: 18x22cm (radius ~14cm).")
        self.root.protocol("WM_DELETE_WINDOW", self._on_close)
        self.root.bind_all("<space>", lambda e: self.stop_route())

        self.root.after(80, self._video_tick)
        self.root.after(50, self._poll_events)

    def log(self, msg):
        stamp = datetime.now().strftime("%H:%M:%S.%f")[:-3]
        try:
            self.log_txt.configure(state="normal")
            self.log_txt.insert("end", "%s  %s\n" % (stamp, msg))
            self.log_txt.see("end")
            self.log_txt.configure(state="disabled")
        except Exception:
            pass

    def _build_ui(self):
        # Top banner
        banner = tk.Frame(self.root, bg="#ffffff", bd=1, relief="solid")
        banner.pack(fill="x", pady=(0, 10))
        banner_inner = tk.Frame(banner, bg="#ffffff")
        banner_inner.pack(fill="x", padx=16, pady=10)

        tk.Label(banner_inner, text="🎯 OVERHEAD A* AUTOPILOT & MAZE NAVIGATOR", font=("Segoe UI", 12, "bold"),
                 fg="#0f172a", bg="#ffffff").pack(side="left")
        tk.Label(banner_inner, text="Real-Time 2-Marker Vision Guidance (Car 10 • Goal 20)",
                 font=("Segoe UI", 9), fg="#64748b", bg="#ffffff").pack(side="left", padx=12)

        top = tk.Frame(self.root, bg="#f8fafc")
        top.pack(fill="both", expand=True, padx=14, pady=4)

        left = tk.Frame(top, bg="#f8fafc")
        left.grid(row=0, column=0, sticky="nsew", padx=(0, 14))

        # Canvas with clean card container
        canvas_card = tk.Frame(left, bg="#ffffff", bd=1, relief="solid")
        canvas_card.grid(row=0, column=0, padx=0, pady=0)

        self.canvas = tk.Canvas(canvas_card, width=RECT_W, height=RECT_H, bg="#0f172a",
                                highlightthickness=0)
        self.canvas.pack(padx=2, pady=2)
        self._photo = None
        self.canvas_img = self.canvas.create_image(0, 0, anchor="nw")
        self.canvas.bind("<Button-1>", self._on_canvas_down)
        self.canvas.bind("<B1-Motion>", self._on_canvas_drag)
        self.canvas.bind("<Button-3>", self._on_canvas_erase)
        self.canvas.bind("<B3-Motion>", self._on_canvas_erase)

        pose_box = tk.Frame(left, bg="#ffffff", bd=1, relief="solid")
        pose_box.grid(row=1, column=0, sticky="ew", padx=0, pady=8)
        self.pose_lbl = tk.Label(pose_box, text="Car [10]: searching... | Goal [20]: searching...",
                                 font=("Segoe UI", 9, "bold"), fg="#2563eb", bg="#ffffff",
                                 padx=10, pady=8)
        self.pose_lbl.pack(anchor="w")

        right = tk.Frame(top, bg="#f8fafc")
        right.grid(row=0, column=1, sticky="ns")

        # Section 1: Camera
        cf = tk.LabelFrame(right, text=" 1. Camera / Map Picture ", font=("Segoe UI", 9, "bold"),
                           fg="#0f172a", bg="#ffffff", bd=1, relief="solid")
        cf.pack(fill="x", pady=(0, 6))
        cf_inner = tk.Frame(cf, bg="#ffffff")
        cf_inner.pack(fill="x", padx=10, pady=8)

        tk.Label(cf_inner, text="Source (URL / index / image):", font=("Segoe UI", 8),
                 fg="#64748b", bg="#ffffff").pack(anchor="w")
        self.cam_var = tk.StringVar(value="0")
        cam_ent = tk.Entry(cf_inner, textvariable=self.cam_var, width=32,
                           bg="#f8fafc", fg="#0f172a", font=("Consolas", 9), bd=1, relief="solid")
        cam_ent.pack(fill="x", pady=4)

        crow = tk.Frame(cf_inner, bg="#ffffff")
        crow.pack(fill="x", pady=2)
        tk.Button(crow, text="Connect Cam", font=("Segoe UI", 8, "bold"),
                  bg="#2563eb", fg="#ffffff", activebackground="#1d4ed8", bd=0, padx=8, pady=3,
                  cursor="hand2", command=self._cam_connect).pack(side="left")
        tk.Button(crow, text="Upload Pic", font=("Segoe UI", 8),
                  bg="#f1f5f9", fg="#334155", activebackground="#e2e8f0", bd=1, relief="solid", padx=8, pady=2,
                  cursor="hand2", command=self._upload_picture).pack(side="left", padx=4)
        tk.Button(crow, text="Stop Cam", font=("Segoe UI", 8),
                  bg="#f1f5f9", fg="#334155", activebackground="#e2e8f0", bd=1, relief="solid", padx=8, pady=2,
                  cursor="hand2", command=self._cam_stop).pack(side="left")

        self.cam_lbl = tk.Label(cf_inner, text="status: camera idle", font=("Segoe UI", 8),
                                fg="#64748b", bg="#ffffff")
        self.cam_lbl.pack(anchor="w", pady=(4, 0))

        # Section 2: Obstacles & Maze Detection
        of = tk.LabelFrame(right, text=" 2. Obstacles & Maze Grid ", font=("Segoe UI", 9, "bold"),
                           fg="#0f172a", bg="#ffffff", bd=1, relief="solid")
        of.pack(fill="x", pady=(0, 6))
        of_inner = tk.Frame(of, bg="#ffffff")
        of_inner.pack(fill="x", padx=10, pady=8)

        tk.Label(of_inner, text="Dark Wall Detection Threshold:", font=("Segoe UI", 8),
                 fg="#64748b", bg="#ffffff").pack(anchor="w")
        self.thresh_var = tk.IntVar(value=90)
        t_slider = ttk.Scale(of_inner, from_=10, to=240, variable=self.thresh_var, orient="horizontal")
        t_slider.pack(fill="x", pady=2)

        orow = tk.Frame(of_inner, bg="#ffffff")
        orow.pack(fill="x", pady=2)
        tk.Button(orow, text="Auto Detect", font=("Segoe UI", 8, "bold"),
                  bg="#f1f5f9", fg="#0f172a", activebackground="#e2e8f0", bd=1, relief="solid", padx=10, pady=2,
                  cursor="hand2", command=self._auto_detect_walls).pack(side="left")
        tk.Button(orow, text="Clear Grid", font=("Segoe UI", 8),
                  bg="#f1f5f9", fg="#64748b", activebackground="#e2e8f0", bd=1, relief="solid", padx=8, pady=2,
                  cursor="hand2", command=self._map_clear).pack(side="left", padx=4)

        paint_row = tk.Frame(of_inner, bg="#ffffff")
        paint_row.pack(fill="x", pady=4)
        self.paint_var = tk.StringVar(value="paint")
        tk.Radiobutton(paint_row, text="Brush Wall (L-click)", variable=self.paint_var,
                       value="paint", bg="#ffffff", activebackground="#ffffff").pack(side="left")
        tk.Radiobutton(paint_row, text="Manual Goal", variable=self.paint_var,
                       value="goal", bg="#ffffff", activebackground="#ffffff").pack(side="left", padx=6)

        c_row = tk.Frame(of_inner, bg="#ffffff")
        c_row.pack(fill="x", pady=2)
        tk.Label(c_row, text="Safety Margin (cm):", font=("Segoe UI", 8),
                 fg="#64748b", bg="#ffffff").pack(side="left")
        self.margin_var = tk.IntVar(value=int(DEFAULT_MARGIN_CM))
        ttk.Spinbox(c_row, from_=1, to=20, width=4, textvariable=self.margin_var,
                    command=self._update_margin).pack(side="left", padx=6)

        # Section 3: Route & Autonomous Drive
        rf = tk.LabelFrame(right, text=" 3. Autonomous A* Route ", font=("Segoe UI", 9, "bold"),
                           fg="#0f172a", bg="#ffffff", bd=1, relief="solid")
        rf.pack(fill="x", pady=(0, 6))
        rf_inner = tk.Frame(rf, bg="#ffffff")
        rf_inner.pack(fill="x", padx=10, pady=8)

        rrow1 = tk.Frame(rf_inner, bg="#ffffff")
        rrow1.pack(fill="x", pady=2)
        tk.Button(rrow1, text="Plan A* Route", font=("Segoe UI", 9, "bold"),
                  bg="#f1f5f9", fg="#2563eb", activebackground="#e2e8f0", bd=1, relief="solid",
                  padx=10, pady=4, cursor="hand2", command=self.plan_route).pack(side="left")

        self.start_btn = tk.Button(rrow1, text="▶ START DRIVE", bg="#16a34a", fg="#ffffff",
                                   activebackground="#15803d", font=("Segoe UI", 9, "bold"),
                                   bd=0, relief="flat", padx=12, pady=4, cursor="hand2",
                                   command=self.start_route)
        self.start_btn.pack(side="left", padx=6)

        tk.Button(rrow1, text="■ STOP", bg="#ef4444", fg="#ffffff",
                  activebackground="#dc2626", font=("Segoe UI", 9, "bold"),
                  bd=0, relief="flat", padx=10, pady=4, cursor="hand2",
                  command=self.stop_route).pack(side="left")

        rrow2 = tk.Frame(rf_inner, bg="#ffffff")
        rrow2.pack(fill="x", pady=4)
        self.dry_var = tk.BooleanVar(value=False)
        tk.Checkbutton(rrow2, text="Dry Run (Simulate on Screen)", variable=self.dry_var,
                       bg="#ffffff", activebackground="#ffffff").pack(side="left")
        tk.Button(rrow2, text="Test Turn", font=("Segoe UI", 8),
                  bg="#f1f5f9", fg="#334155", activebackground="#e2e8f0", bd=1, relief="solid",
                  padx=8, pady=1, cursor="hand2", command=self.test_turn).pack(side="left", padx=8)

        if not self.is_embedded:
            bf = tk.LabelFrame(right, text=" Bluetooth Connection ", font=("Segoe UI", 9, "bold"),
                               fg="#0f172a", bg="#ffffff", bd=1, relief="solid")
            bf.pack(fill="x", pady=(0, 6))
            bf_inner = tk.Frame(bf, bg="#ffffff")
            bf_inner.pack(fill="x", padx=10, pady=8)

            tk.Label(bf_inner, text="COM port:", font=("Segoe UI", 8), fg="#64748b", bg="#ffffff").pack(anchor="w")
            self.port_var = tk.StringVar()
            self.port_box = ttk.Combobox(bf_inner, textvariable=self.port_var, width=24)
            self.port_box.pack(fill="x", pady=2)

            brow = tk.Frame(bf_inner, bg="#ffffff")
            brow.pack(fill="x", pady=2)
            tk.Button(brow, text="Refresh", font=("Segoe UI", 8), bg="#f1f5f9", fg="#334155",
                      bd=1, relief="solid", padx=6, command=self._refresh_ports).pack(side="left")
            self.conn_btn = tk.Button(brow, text="Connect", font=("Segoe UI", 8, "bold"),
                                      bg="#2563eb", fg="#ffffff", bd=0, padx=10, command=self._toggle_serial)
            self.conn_btn.pack(side="left", padx=4)
            self.ser_lbl = tk.Label(bf_inner, text="serial: disconnected", font=("Segoe UI", 8),
                                    fg="#64748b", bg="#ffffff")
            self.ser_lbl.pack(anchor="w", pady=(2, 0))
        else:
            self.ser_lbl = tk.Label(rf_inner, text="HC-05: Using shared main link",
                                    font=("Segoe UI", 8, "bold"), fg="#16a34a", bg="#ffffff")
            self.ser_lbl.pack(anchor="w", pady=(4, 0))

        # Status Log card
        lf = tk.LabelFrame(right, text=" System Activity Log ", font=("Segoe UI", 9, "bold"),
                           fg="#0f172a", bg="#ffffff", bd=1, relief="solid")
        lf.pack(fill="both", expand=True)
        self.log_txt = tk.Text(lf, width=42, height=9, state="disabled", wrap="word",
                               bg="#f8fafc", fg="#1e293b", bd=0, padx=6, pady=4,
                               font=("Consolas", 8))
        self.log_txt.pack(padx=6, pady=6, fill="both", expand=True)

    def _update_margin(self):
        try:
            self.omap.margin_cm = float(self.margin_var.get())
        except Exception:
            pass

    def _set_ser(self, ok):
        if hasattr(self, "ser_lbl"):
            self.ser_lbl.configure(text="serial: %s" % ("connected" if ok else "disconnected"))
        if hasattr(self, "conn_btn"):
            self.conn_btn.configure(text="Disconnect" if ok else "Connect")

    def _refresh_ports(self):
        if list_ports is None:
            return
        try:
            ports = [p.device for p in list_ports.comports()]
        except Exception:
            ports = []
        self.port_box["values"] = ports
        if ports and not self.port_var.get():
            self.port_var.set(ports[0])

    def _toggle_serial(self):
        if self.link is None:
            self.log("pyserial missing.")
            return
        if self.link.connected:
            self.link.disconnect()
            self._set_ser(False)
            self.log("Disconnected")
            return
        port = (self.port_var.get() or "").split(" ")[0].strip()
        if not port:
            self.log("Pick a COM port first.")
            return
        try:
            self.link.connect(port, BAUD_RATE)
            self._set_ser(True)
            self.log("Connected %s @%d" % (port, BAUD_RATE))
        except Exception as exc:
            self.log("Connect failed: %s" % exc)
            self._set_ser(False)

    def _cam_connect(self):
        if cv2 is None:
            self.log("OpenCV missing. pip install opencv-contrib-python")
            return
        src = self.cam_var.get().strip()
        self.cam_lbl.configure(text="status: connecting...")
        self.log("Opening camera source: %s" % src)

        def _op():
            try:
                self.cam.start(src)
            except Exception as exc:
                self.root.after(0, lambda: (
                    self.cam_lbl.configure(text="status: failed"),
                    self.log("Camera failed: %s" % exc)))
                return
            self.root.after(0, lambda: (
                self.cam_lbl.configure(text="status: live feed"),
                self.log("Camera opened: %s" % src)))
        threading.Thread(target=_op, daemon=True).start()

    def _upload_picture(self):
        path = filedialog.askopenfilename(
            filetypes=[("Image files", "*.jpg;*.jpeg;*.png;*.bmp"), ("All files", "*.*")])
        if not path:
            return
        img = cv2.imread(path)
        if img is None:
            self.log("Failed to load picture: %s" % path)
            return
        resized = cv2.resize(img, (RECT_W, RECT_H))
        self.cam.set_static_frame(resized)
        self.cam_lbl.configure(text="status: picture loaded")
        self.log("Uploaded map picture: %s" % os.path.basename(path))
        self._auto_detect_walls()

    def _cam_stop(self):
        self.cam.stop()
        self.cam_lbl.configure(text="status: camera idle")

    def _auto_detect_walls(self):
        with self._vlock:
            frame = None if self.last_frame is None else self.last_frame.copy()
        if frame is None:
            self.log("Auto-detect: No image/camera frame available.")
            return
        th = self.thresh_var.get()
        self.omap.auto_extract_obstacles(frame, threshold_val=th)
        self.log("Obstacles extracted using threshold %d." % th)

    def _canvas_cm(self, event):
        px = max(0, min(RECT_W - 1, event.x))
        py = max(0, min(RECT_H - 1, event.y))
        return px_to_cm(px, py)

    def _on_canvas_down(self, event):
        x_cm, y_cm = self._canvas_cm(event)
        if self.paint_var.get() == "goal":
            with self._rlock:
                self.goal_cm = (x_cm, y_cm)
            self.log("Manual Goal set: (%.0f, %.0f) cm" % (x_cm, y_cm))
        else:
            self.omap.paint_cm(x_cm, y_cm, erase=False)

    def _on_canvas_drag(self, event):
        if self.paint_var.get() == "goal":
            return
        x_cm, y_cm = self._canvas_cm(event)
        self.omap.paint_cm(x_cm, y_cm, erase=False)

    def _on_canvas_erase(self, event):
        x_cm, y_cm = self._canvas_cm(event)
        self.omap.paint_cm(x_cm, y_cm, erase=True)

    def _map_clear(self):
        self.omap.clear()
        with self._rlock:
            self.path_cm = []
            self.path_cells = None
        self.log("Cleared obstacle map & paths.")

    def _video_tick(self):
        frame, ts = self.cam.get()
        if frame is None:
            disp = np.zeros((RECT_H, RECT_W, 3), dtype=np.uint8)
            if cv2 is not None:
                cv2.putText(disp, "No Camera / Image - Connect Cam or Upload Pic",
                            (30, RECT_H // 2), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (200, 200, 200), 2)
            self._show(disp)
            self.root.after(100, self._video_tick)
            return

        rect = cv2.resize(frame, (RECT_W, RECT_H))
        with self._vlock:
            self.last_frame = rect.copy()

        try:
            rdet = detect_markers_both(rect)
        except Exception:
            rdet = {}

        car_txt = "Car [10]: NOT SEEN"
        goal_txt = "Goal [20]: NOT SEEN"

        if CAR_ID in rdet:
            cx, cy, hd = pose_from_marker(rdet[CAR_ID])
            x_cm, y_cm = px_to_cm(cx, cy)
            now = time.time()
            with self._vlock:
                self.pose = (x_cm, y_cm, hd)
                self.pose_ts = now
                self.last_seen_car_ts = now
            car_txt = "Car: (%.0f, %.0f)cm  θ:%.0f deg" % (x_cm, y_cm, hd)

        if GOAL_ID in rdet:
            gcx = float(rdet[GOAL_ID][:, 0].mean())
            gcy = float(rdet[GOAL_ID][:, 1].mean())
            gx_cm, gy_cm = px_to_cm(gcx, gcy)
            with self._rlock:
                self.goal_cm = (gx_cm, gy_cm)
            goal_txt = "Goal: (%.0f, %.0f)cm" % (gx_cm, gy_cm)
        else:
            with self._rlock:
                if self.goal_cm is not None:
                    goal_txt = "Goal (manual): (%.0f, %.0f)cm" % (self.goal_cm[0], self.goal_cm[1])

        self.pose_lbl.configure(text="%s  |  %s" % (car_txt, goal_txt))

        overlay = self._render_overlay(rect, rdet)
        self._show(overlay)

        self.root.after(60, self._video_tick)

    def _render_overlay(self, rect, rdet):
        vis = rect.copy()
        infl = self.omap.inflated()
        with self._rlock:
            path = list(self.path_cm)
            goal = self.goal_cm

        red = np.zeros_like(vis)
        mask = cv2.resize(infl, (RECT_W, RECT_H), interpolation=cv2.INTER_NEAREST) > 0
        red[mask] = (0, 0, 220)
        vis = cv2.addWeighted(vis, 1.0, red, 0.35, 0)

        for mid, c in rdet.items():
            pts = c.astype(int)
            col = (0, 255, 0) if mid == GOAL_ID else ((0, 0, 255) if mid == CAR_ID else (255, 255, 0))
            cv2.polylines(vis, [pts], True, col, 2)
            cx, cy = int(c[:, 0].mean()), int(c[:, 1].mean())
            label = "CAR" if mid == CAR_ID else ("GOAL" if mid == GOAL_ID else str(mid))
            cv2.putText(vis, label, (cx + 8, cy - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.7, col, 2)

        if goal is not None:
            gpx, gpy = cm_to_px(goal[0], goal[1])
            cv2.circle(vis, (int(gpx), int(gpy)), 10, (0, 255, 0), -1)
            cv2.circle(vis, (int(gpx), int(gpy)), 14, (255, 255, 255), 2)

        with self._vlock:
            pose = self.pose
        if pose is not None:
            cpx, cpy = cm_to_px(pose[0], pose[1])
            rad = math.radians(pose[2])
            hx = int(cpx + 30 * math.cos(rad))
            hy = int(cpy + 30 * math.sin(rad))
            cv2.arrowedLine(vis, (int(cpx), int(cpy)), (hx, hy), (0, 255, 255), 3, tipLength=0.3)

        if len(path) >= 2:
            pts = [cm_to_px(x, y) for x, y in path]
            for i in range(len(pts) - 1):
                p1 = (int(pts[i][0]), int(pts[i][1]))
                p2 = (int(pts[i + 1][0]), int(pts[i + 1][1]))
                cv2.line(vis, p1, p2, (255, 255, 0), 3)
                cv2.circle(vis, p2, 4, (0, 255, 255), -1)

        return vis

    def _show(self, bgr):
        if Image is None or ImageTk is None:
            return
        rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
        im = Image.fromarray(rgb)
        self._photo = ImageTk.PhotoImage(image=im)
        self.canvas.itemconfig(self.canvas_img, image=self._photo)

    def _poll_events(self):
        try:
            while True:
                kind, payload = self.events.get_nowait()
                if kind == "rx":
                    self.log("RX: %s" % payload)
                elif kind == "error":
                    self.log("Error: %s" % payload)
        except queue.Empty:
            pass
        self.root.after(50, self._poll_events)

    def plan_route(self):
        with self._vlock:
            pose = self.pose
        with self._rlock:
            goal = self.goal_cm

        if pose is None:
            self.log("Plan: Car marker 10 not visible. Place car marker in view.")
            return
        if goal is None:
            self.log("Plan: No goal. Show Goal marker 20 or set Goal mode and click.")
            return

        infl = self.omap.inflated()
        s = cm_to_grid(pose[0], pose[1])
        g = cm_to_grid(goal[0], goal[1])

        if infl[g[1], g[0]]:
            self.log("Plan: Goal is inside an obstacle zone. Adjust margin or move goal.")
            return
        if infl[s[1], s[0]]:
            self.log("Plan: Car start position is marked as an obstacle. Adjust margin.")
            return

        cells = astar(infl, s, g)
        if not cells:
            self.log("Plan: A* found no path (route completely blocked).")
            return

        simp = simplify_path(cells, infl)
        wp = [grid_to_cm(gx, gy) for gx, gy in simp]
        wp[-1] = goal

        with self._rlock:
            self.path_cells = simp
            self.path_cm = wp
            self.infl_cache = infl

        total_dist = sum(math.hypot(wp[i + 1][0] - wp[i][0], wp[i + 1][1] - wp[i][1])
                         for i in range(len(wp) - 1)) if len(wp) > 1 else 0
        self.log("Plan OK: %d waypoints, distance ~%.0f cm." % (len(wp), total_dist))

    def _is_dry(self):
        try:
            return bool(self.dry_var.get())
        except Exception:
            return False

    def _tx(self, text, note=""):
        if self._is_dry():
            self.log("DRY TX: %s%s" % (text, note))
            return True
        if self.link is None or not self.link.connected:
            self.log("TX FAILED: Bluetooth not connected%s" % note)
            return False
        try:
            self.link.send(text)
            self.log("TX: %s%s" % (text, note))
            return True
        except Exception as exc:
            self.log("TX failed: %s" % exc)
            return False

    def start_route(self):
        if self.driving:
            self.log("Already driving.")
            return
        with self._rlock:
            wp = list(self.path_cm)
            goal = self.goal_cm

        if not wp or goal is None:
            self.log("Press Plan A* Route first.")
            return
        with self._vlock:
            pose = self.pose
        if pose is None:
            self.log("Start: Car marker not detected.")
            return

        dry = self._is_dry()
        if not dry and (self.link is None or not self.link.connected):
            self.log("Start: Bluetooth HC-05 not connected (or enable Dry Run).")
            return

        self._cancel.clear()
        self.driving = True
        self._start_time = time.time()
        self._last_progress = time.time()
        self._last_progress_pos = pose[:2]
        self._drive_thread = threading.Thread(target=self._drive_loop, daemon=True)
        self._drive_thread.start()
        self.log("Autonomous navigation started (%s)." % ("SIMULATION" if dry else "LIVE"))

    def stop_route(self):
        self._cancel.set()
        self.driving = False
        if not self._is_dry() and self.link is not None and self.link.connected:
            for _ in range(3):
                try:
                    self.link.send(CMD_STOP)
                    time.sleep(0.02)
                except Exception:
                    break
        self.log("STOPPED route navigation.")

    def test_turn(self):
        self.log("Testing turn pulse right (%s)..." % CMD_RIGHT)
        if not self._is_dry() and self.link and self.link.connected:
            self._tx(CMD_RIGHT, " (test right)")
            time.sleep(TURN_PULSE_MS / 1000.0)
            self._tx(CMD_STOP, " (stop)")
        else:
            self.log("DRY turn test.")

    def _drive_loop(self):
        period = 1.0 / LOOP_HZ
        wi = 0
        with self._rlock:
            wps = list(self.path_cm)

        while not self._cancel.is_set():
            t0 = time.time()
            with self._vlock:
                pose = self.pose
                pts = self.pose_ts
                lost_for = time.time() - self.last_seen_car_ts

            now = time.time()
            if pose is None or (now - pts) > POSE_MAX_AGE_S or lost_for > MARKER_LOST_S:
                self._tx(CMD_STOP, " (marker lost)")
                time.sleep(0.2)
                continue

            if wi >= len(wps):
                wi = len(wps) - 1

            while wi < len(wps):
                dist = math.hypot(wps[wi][0] - pose[0], wps[wi][1] - pose[1])
                tol = GOAL_TOL_CM if wi == len(wps) - 1 else WAYPOINT_TOL_CM
                if dist < tol:
                    wi += 1
                else:
                    break

            if wi >= len(wps):
                self._tx(CMD_STOP, " (reached goal!)")
                self.log("GOAL REACHED! Successfully navigated maze.")
                break

            tx, ty = wps[wi]
            want = math.degrees(math.atan2(ty - pose[1], tx - pose[0]))
            err = wrap180(want - pose[2])
            dist = math.hypot(tx - pose[0], ty - pose[1])

            if abs(err) > TURN_THRESHOLD_DEG:
                turn = CMD_RIGHT if (err > 0) == (TURN_SIGN > 0) else CMD_LEFT
                self._tx(turn, " (turn)")
                time.sleep(TURN_PULSE_MS / 1000.0)
                self._tx(CMD_STOP, " (settle)")
                time.sleep(SETTLE_MS / 1000.0)
            else:
                self._tx(CMD_FWD, " (fwd)")
                time.sleep(FORWARD_PULSE_MS / 1000.0)
                self._tx(CMD_STOP, " (settle)")
                time.sleep(SETTLE_MS / 1000.0)

            dt = time.time() - t0
            if dt < period:
                time.sleep(period - dt)

        self.driving = False

    def _on_close(self):
        try:
            self._cancel.set()
            self.driving = False
            self.cam.stop()
            if not self.is_embedded and self.link is not None and self.link.connected:
                self.link.disconnect()
        except Exception:
            pass
        self.root.destroy()


def main():
    try:
        root = tk.Tk()
        RoutePlannerApp(root)
        root.mainloop()
    except Exception as exc:
        print("Error: %s" % exc)


if __name__ == "__main__":
    main()
