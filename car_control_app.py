"""
RC Car Control App - Phase 1 (manual Bluetooth serial control)
Requirements: Python 3 + pyserial -> pip install pyserial
Run: python car_control_app.py
HC-05 must be paired in Windows so it shows up as COM port.
Labels F/B/L/R/S are for reading; bytes sent are digits in CMD_MAP.
"""
import json
import os
import queue
import threading
import time
import tkinter as tk
import traceback
from datetime import datetime
from tkinter import messagebox, ttk
try:
    import serial
    from serial.tools import list_ports
    _IMPORT_ERROR = None
except ImportError as _e:
    serial = None
    list_ports = None
    _IMPORT_ERROR = repr(_e)
BAUD_RATE = 9600  # CHANGE THIS if Arduino sketch uses different rate
CMD_MAP = {"F": "8", "B": "2", "R": "6", "L": "4", "S": "5"}
STOP_CMD = CMD_MAP["S"]
MODE_AUTO = "A"
MODE_MANUAL = "M"
KEY_TO_DIGIT = {
    "8": "8", "2": "2", "4": "4", "6": "6", "5": "5",
    "KP_8": "8", "KP_2": "2", "KP_4": "4", "KP_6": "6", "KP_5": "5",
    "KP_Up": "8", "KP_Down": "2", "KP_Left": "4",
    "KP_Right": "6", "KP_Begin": "5",
}
DIGIT_TO_LABEL = {v: k for k, v in CMD_MAP.items()}
LOG_MAX_LINES = 500
POLL_MS = 50
PROBE_PING = STOP_CMD  # safe ping: stop; car stays still even if it hears us
# Prefer ports whose HWID contains our HC-05's MAC tail (prevents picking
# a generic placeholder port like COM5/COM6 whose HWID ends 000000000000).
HC05_HWID_HINT = "FCA89A0004EA"
CALIBRATION_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)), "calibration.json")


def probe_port_once(port, baud):
    """Try opening port and sending a safe STOP ping, then close immediately.

    Returns (openable, replied, sample). Never raises, never blocks on read:
    many HC-05 links accept the open but never answer, and read() on a
    silent BT virtual COM can hang - so we deliberately do NOT wait for a
    reply here. Reply confirmation happens live in the log after Connect
    (any RX line proves the Arduino is talking back).
    """
    if serial is None:
        return False, False, ""
    ser = None
    try:
        ser = serial.Serial(port, baud, timeout=0.2, write_timeout=1)
        try:
            ser.write(PROBE_PING.encode("ascii"))
            ser.flush()
        except Exception:
            pass
        return True, False, ""
    except Exception:
        return False, False, ""
    finally:
        if ser is not None:
            try:
                ser.close()
            except Exception:
                pass


class SerialManager:
    def __init__(self, events):
        self.events = events
        self.ser = None
        self._stop = threading.Event()
        self._thread = None
        self._lock = threading.Lock()
    @property
    def connected(self):
        return self.ser is not None and self.ser.is_open
    def connect(self, port, baud):
        self.disconnect()
        # On Windows Bluetooth RFCOMM, setting write_timeout to None or a larger float
        # prevents spurious timeouts. write_timeout=2 on a jittery BT link causes immediate crashes.
        self.ser = serial.Serial(port, baud, timeout=0.1, write_timeout=3.0)
        # Flush any garbage in buffers from previous sessions
        try:
            self.ser.reset_input_buffer()
            self.ser.reset_output_buffer()
        except Exception:
            pass
        self._stop.clear()
        self._thread = threading.Thread(target=self._reader, args=(self.ser,), daemon=True)
        self._thread.start()

    def disconnect(self):
        self._stop.set()
        ser, self.ser = self.ser, None
        if ser is not None:
            try:
                ser.close()
            except Exception:
                pass
        if self._thread is not None and self._thread is not threading.current_thread():
            self._thread.join(timeout=0.5)
        self._thread = None

    def send(self, text):
        with self._lock:
            if not self.connected:
                raise OSError("Not connected")
            data = text.encode("ascii", errors="replace")
            # Try sending with 1 immediate retry before raising
            try:
                self.ser.write(data)
                self.ser.flush()
            except Exception:
                time.sleep(0.05)
                self.ser.write(data)
                self.ser.flush()

    def _reader(self, ser):
        while not self._stop.is_set():
            try:
                if ser.in_waiting:
                    line = ser.readline()
                    if line:
                        msg = line.decode(errors="replace").strip()
                        if msg:
                            self.events.put(("rx", msg))
                else:
                    time.sleep(0.02)
            except Exception as exc:
                if not self._stop.is_set():
                    try:
                        self.events.put(("error", "Connection lost: %s" % exc))
                    except Exception:
                        pass
                break
class CarApp:
    def __init__(self, root):
        self.root = root
        root.title("RC Car Autopilot & Control Dashboard")
        root.resizable(False, False)

        # Clean modern light palette (Apple / modern dashboard aesthetic)
        self.bg_main = "#f8fafc"        # Ultra-clean light slate
        self.card_bg = "#ffffff"        # Pure white card
        self.card_border = "#e2e8f0"    # Subtle hairline border
        self.text_main = "#0f172a"      # Deep slate black
        self.text_muted = "#64748b"     # Clean slate grey
        self.primary_blue = "#2563eb"   # Vibrant modern blue
        self.primary_hover = "#1d4ed8"
        self.success_green = "#16a34a"  # Crisp emerald
        self.danger_red = "#ef4444"     # Clean crimson

        root.configure(bg=self.bg_main)

        # Configure modern TTK styles
        self.style = ttk.Style()
        try:
            self.style.theme_use("clam")
        except Exception:
            pass

        self.style.configure(".", background=self.bg_main, foreground=self.text_main, font=("Segoe UI", 9))
        self.style.configure("TCombobox", fieldbackground="#ffffff", background="#f1f5f9",
                             foreground="#0f172a", darkcolor="#cbd5e1", lightcolor="#e2e8f0",
                             bordercolor="#cbd5e1", arrowcolor="#334155")
        self.style.map("TCombobox", fieldbackground=[("readonly", "#ffffff")],
                       selectbackground=[("readonly", "#2563eb")],
                       selectforeground=[("readonly", "#ffffff")])

        self.events = queue.Queue()
        self.link = SerialManager(self.events) if serial else None
        self.autonomous = False
        self.held_keys = set()
        self.route_window = None
        self.route_app = None
        self.classroom_window = None
        self.classroom_app = None

        # Calibration & Manual Drive Profiler State
        self.drive_start_time = None
        self.last_drive_elapsed = 0.0
        self.calib_speed = 68.0
        self.calib_turn_ms = 220
        self._load_calibration()

        self._build_header()
        self._build_connection_bar()
        self._build_dpad()
        self._build_bottom()

        root.bind_all("<KeyPress>", self._on_key_press)
        root.bind_all("<KeyRelease>", self._on_key_release)
        root.protocol("WM_DELETE_WINDOW", self._on_close)
        self._refresh_ports()
        self._set_status("Disconnected", "#94a3b8")
        if serial is None:
            self._set_status("Missing Serial", "#ef4444")
            import sys as _sys
            self._log("pyserial import failed: %s | python=%s" % (_IMPORT_ERROR, _sys.executable), "err")
        self.root.after(POLL_MS, self._poll)

    def _build_header(self):
        head = tk.Frame(self.root, bg="#ffffff", bd=1, relief="solid", highlightthickness=0)
        head.grid(row=0, column=0, sticky="ew", padx=0, pady=(0, 10))

        # Add a subtle bottom border effect
        inner = tk.Frame(head, bg="#ffffff")
        inner.pack(fill="x", padx=18, pady=10)

        left = tk.Frame(inner, bg="#ffffff")
        left.pack(side="left")

        tk.Label(left, text="🚗 SMART RC AUTONOMOUS CAR", font=("Segoe UI", 13, "bold"),
                 fg="#0f172a", bg="#ffffff").pack(anchor="w")
        tk.Label(left, text="Bluetooth HC-05 Link  •  Live Overhead Closed-Loop Autopilot",
                 font=("Segoe UI", 8), fg="#64748b", bg="#ffffff").pack(anchor="w")

        self.status_pill = tk.Label(inner, text="● Disconnected", font=("Segoe UI", 9, "bold"),
                                    fg="#ffffff", bg="#94a3b8", padx=12, pady=5, relief="flat")
        self.status_pill.pack(side="right")

    def _build_connection_bar(self):
        card = tk.Frame(self.root, bg=self.card_bg, bd=1, relief="solid", highlightthickness=0)
        card.grid(row=1, column=0, sticky="ew", padx=14, pady=5)

        inner = tk.Frame(card, bg=self.card_bg)
        inner.pack(fill="x", padx=14, pady=10)

        tk.Label(inner, text="Bluetooth Port:", font=("Segoe UI", 9, "bold"),
                 fg=self.text_main, bg=self.card_bg).pack(side="left", padx=(0, 6))

        self.port_var = tk.StringVar()
        self.port_box = ttk.Combobox(inner, textvariable=self.port_var, width=30, state="readonly")
        self.port_box.pack(side="left", padx=4)

        refresh_btn = tk.Button(inner, text="🔄 Refresh", font=("Segoe UI", 9),
                                bg="#f1f5f9", fg="#334155", activebackground="#e2e8f0",
                                bd=1, relief="solid", padx=10, pady=3, cursor="hand2", command=self._refresh_ports)
        refresh_btn.pack(side="left", padx=4)

        self.connect_btn = tk.Button(inner, text="Connect", font=("Segoe UI", 9, "bold"),
                                     bg=self.primary_blue, fg="#ffffff", activebackground=self.primary_hover,
                                     bd=0, relief="flat", padx=16, pady=4, cursor="hand2", command=self._toggle_connection)
        self.connect_btn.pack(side="left", padx=4)

        self.find_btn = tk.Button(inner, text="🔍 Auto-Find HC-05", font=("Segoe UI", 9),
                                  bg="#f8fafc", fg=self.primary_blue, activebackground="#eff6ff",
                                  bd=1, relief="solid", padx=10, pady=3, cursor="hand2", command=self._auto_probe)
        self.find_btn.pack(side="right", padx=2)

    def _build_dpad(self):
        card = tk.Frame(self.root, bg=self.card_bg, bd=1, relief="solid", highlightthickness=0)
        card.grid(row=2, column=0, padx=14, pady=5, sticky="ew")

        # Header for section
        title_box = tk.Frame(card, bg=self.card_bg)
        title_box.pack(fill="x", padx=14, pady=(10, 4))
        tk.Label(title_box, text="DRIVE CONTROLS", font=("Segoe UI", 9, "bold"),
                 fg=self.text_muted, bg=self.card_bg).pack(anchor="w")

        grid_frame = tk.Frame(card, bg=self.card_bg)
        grid_frame.pack(pady=(4, 12))

        def make_btn(label, symbol, row, col, is_stop=False):
            if is_stop:
                bg_col = "#ef4444"
                fg_col = "#ffffff"
                hover_col = "#dc2626"
            else:
                bg_col = "#f1f5f9"
                fg_col = "#0f172a"
                hover_col = "#e2e8f0"

            btn = tk.Button(grid_frame, text="%s\n%s" % (symbol, label), width=7, height=2,
                            font=("Segoe UI", 11, "bold"),
                            bg=bg_col, fg=fg_col, activebackground=hover_col,
                            bd=1, relief="solid" if not is_stop else "flat", cursor="hand2")
            btn.grid(row=row, column=col, padx=5, pady=5)
            if is_stop:
                btn.configure(command=lambda: self._send_drive(CMD_MAP["S"]))
            else:
                digit = CMD_MAP[label]
                btn.bind("<ButtonPress-1>", lambda e, d=digit: self._send_drive(d))
                btn.bind("<ButtonRelease-1>", lambda e: self._send_drive(STOP_CMD, quiet_mode=True))
            return btn

        make_btn("F", "▲", 0, 1)
        make_btn("L", "◄", 1, 0)
        make_btn("S", "■", 1, 1, is_stop=True)
        make_btn("R", "►", 1, 2)
        make_btn("B", "▼", 2, 1)

        modes_frame = tk.Frame(card, bg=self.card_bg)
        modes_frame.pack(fill="x", padx=14, pady=(2, 14))

        self.auto_btn = tk.Button(modes_frame, text="⚡ MODE: AUTO FREEPLAY (Ultrasonic Sensor Roam)",
                                  font=("Segoe UI", 10, "bold"),
                                  bg="#10b981", fg="#ffffff", activebackground="#059669",
                                  bd=0, relief="flat", height=2, cursor="hand2",
                                  command=self._toggle_mode)
        self.auto_btn.pack(fill="x", pady=4)

        self.maze_btn = tk.Button(modes_frame, text="🎯 OPEN GOAL MAZE (A* Camera Autopilot)",
                                  font=("Segoe UI", 10, "bold"),
                                  bg="#2563eb", fg="#ffffff", activebackground="#1d4ed8",
                                  bd=0, relief="flat", height=2, cursor="hand2",
                                  command=self._open_goal_maze)
        self.maze_btn.pack(fill="x", pady=4)

        self.classroom_btn = tk.Button(modes_frame, text="🏫 CLASSROOM MAZE (Map Upload / No Cam Mode)",
                                       font=("Segoe UI", 10, "bold"),
                                       bg="#7c3aed", fg="#ffffff", activebackground="#6d28d9",
                                       bd=0, relief="flat", height=2, cursor="hand2",
                                       command=self._open_classroom_maze)
        self.classroom_btn.pack(fill="x", pady=4)

        # ---------------- Manual Drive Profiler & Calibrator ----------------
        calib_box = tk.LabelFrame(card, text=" 📐 Manual Drive Calibrator (200 RPM Profiler) ",
                                  font=("Segoe UI", 9, "bold"), fg=self.text_main, bg=self.card_bg, padx=10, pady=8)
        calib_box.pack(fill="x", padx=14, pady=(4, 12))

        # Row 1: Last Drive Time & Measured Distance Input
        r1 = tk.Frame(calib_box, bg=self.card_bg)
        r1.pack(fill="x", pady=2)

        self.last_drive_lbl = tk.Label(r1, text="Last Forward Drive: 0.00 s", font=("Segoe UI", 9, "bold"),
                                       fg=self.primary_blue, bg=self.card_bg)
        self.last_drive_lbl.pack(side="left")

        tk.Label(r1, text="Measured Distance (cm):", font=("Segoe UI", 9), fg=self.text_main, bg=self.card_bg).pack(side="left", padx=(14, 4))
        self.measured_dist_var = tk.StringVar(value="100.0")
        dist_ent = tk.Entry(r1, textvariable=self.measured_dist_var, width=8, font=("Segoe UI", 9), justify="center")
        dist_ent.pack(side="left", padx=2)

        calc_btn = tk.Button(r1, text="Calculate Speed", font=("Segoe UI", 8, "bold"),
                             bg="#2563eb", fg="#ffffff", activebackground="#1d4ed8",
                             cursor="hand2", command=self._calculate_speed)
        calc_btn.pack(side="left", padx=6)

        # Row 2: Turn Calibrator & Save
        r2 = tk.Frame(calib_box, bg=self.card_bg)
        r2.pack(fill="x", pady=(6, 2))

        tk.Label(r2, text="90° Turn Time (ms):", font=("Segoe UI", 9), fg=self.text_main, bg=self.card_bg).pack(side="left")
        self.turn_ms_var = tk.StringVar(value=str(self.calib_turn_ms))
        turn_ent = tk.Entry(r2, textvariable=self.turn_ms_var, width=6, font=("Segoe UI", 9), justify="center")
        turn_ent.pack(side="left", padx=4)

        test_turn_btn = tk.Button(r2, text="Test 90° Turn", font=("Segoe UI", 8),
                                  bg="#f1f5f9", fg="#334155", activebackground="#e2e8f0",
                                  cursor="hand2", command=self._test_90_turn)
        test_turn_btn.pack(side="left", padx=4)

        self.calib_summary_lbl = tk.Label(r2, text=f"Speed: {self.calib_speed:.1f} cm/s | Turn: {self.calib_turn_ms} ms",
                                          font=("Segoe UI", 8, "bold"), fg="#16a34a", bg=self.card_bg)
        self.calib_summary_lbl.pack(side="left", padx=10)

        save_calib_btn = tk.Button(r2, text="💾 Save Calibration", font=("Segoe UI", 8, "bold"),
                                   bg="#16a34a", fg="#ffffff", activebackground="#15803d",
                                   cursor="hand2", command=self._save_calibration)
        save_calib_btn.pack(side="right")

    def _build_bottom(self):
        card = tk.Frame(self.root, bg=self.card_bg, bd=1, relief="solid", highlightthickness=0)
        card.grid(row=3, column=0, sticky="ew", padx=14, pady=(5, 12))

        top_bar = tk.Frame(card, bg=self.card_bg)
        top_bar.pack(fill="x", padx=14, pady=(10, 4))

        tk.Label(top_bar, text="LOG CONSOLE", font=("Segoe UI", 9, "bold"),
                 fg=self.text_muted, bg=self.card_bg).pack(side="left")

        self.last_cmd_var = tk.StringVar(value="Last Command: None")
        tk.Label(top_bar, textvariable=self.last_cmd_var, font=("Segoe UI", 9, "bold"),
                 fg=self.primary_blue, bg=self.card_bg).pack(side="right")

        log_frame = tk.Frame(card, bg="#f8fafc", bd=1, relief="solid")
        log_frame.pack(fill="both", padx=14, pady=4)

        self.log = tk.Text(log_frame, height=8, width=74, state="disabled", wrap="none",
                           bg="#f8fafc", fg="#1e293b", insertbackground="#2563eb",
                           font=("Consolas", 9), bd=0, padx=8, pady=6)
        sb = ttk.Scrollbar(log_frame, command=self.log.yview)
        self.log.configure(yscrollcommand=sb.set)
        self.log.pack(side="left", fill="both")
        sb.pack(side="right", fill="y")

        self.log.tag_configure("tx", foreground="#2563eb")
        self.log.tag_configure("rx", foreground="#16a34a")
        self.log.tag_configure("err", foreground="#dc2626")

        raw_bar = tk.Frame(card, bg=self.card_bg)
        raw_bar.pack(fill="x", padx=14, pady=(8, 12))

        tk.Label(raw_bar, text="Manual Byte:", font=("Segoe UI", 9),
                 fg=self.text_muted, bg=self.card_bg).pack(side="left")
        self.raw_var = tk.StringVar()
        entry = tk.Entry(raw_bar, textvariable=self.raw_var, width=16,
                         bg="#ffffff", fg="#0f172a", insertbackground="#2563eb",
                         bd=1, relief="solid", font=("Consolas", 10))
        entry.pack(side="left", padx=6)
        entry.bind("<Return>", lambda e: self._send_raw())

        self.newline_var = tk.BooleanVar(value=False)
        cb = tk.Checkbutton(raw_bar, text="+ \\n", variable=self.newline_var,
                            bg=self.card_bg, fg=self.text_muted, selectcolor="#ffffff",
                            activebackground=self.card_bg, activeforeground=self.text_main)
        cb.pack(side="left", padx=4)

        send_btn = tk.Button(raw_bar, text="Send Byte", font=("Segoe UI", 8, "bold"),
                             bg="#f1f5f9", fg="#334155", activebackground="#e2e8f0",
                             bd=1, relief="solid", padx=12, pady=2, cursor="hand2", command=self._send_raw)
        send_btn.pack(side="left", padx=4)
    def _refresh_ports(self):
        if list_ports is None:
            return
        infos = list(list_ports.comports())
        ports = ["%s - %s" % (p.device, p.description) for p in infos]
        self.port_box["values"] = ports
        if ports and not self.port_var.get():
            hinted = [x for x in infos
                      if HC05_HWID_HINT and HC05_HWID_HINT in (x.hwid or "")]
            if hinted:
                self.port_var.set("%s - %s" % (hinted[0].device, hinted[0].description))
            else:
                self.port_var.set(ports[0])
    def _toggle_connection(self):
        if serial is None:
            self._log("pyserial not installed. Run: pip install pyserial", "err")
            return
        if self.link.connected:
            self._disconnect()
            return
        port = self.port_var.get().split(" - ")[0].strip()
        if not port:
            self._log("Pick or type a COM port first (e.g. COM5).", "err")
            return
        try:
            self.link.connect(port, BAUD_RATE)
        except Exception as exc:
            self._set_status("Error", "red")
            self._log("Could not open %s: %s" % (port, exc), "err")
            return
        self.connect_btn.configure(text="Disconnect", bg="#dc2626", activebackground="#b91c1c")
        self._set_status("Connected", "#16a34a")
        self._log("Connected to %s at %d baud" % (port, BAUD_RATE), "rx")
    def _auto_probe(self):
        if serial is None or list_ports is None:
            self._log("pyserial not installed. Run: pip install pyserial", "err")
            return
        if self.link and self.link.connected:
            self._log("Already connected - disconnect first to probe.", "err")
            return
        ports = [p.device for p in list_ports.comports()]
        if not ports:
            self._log("No COM ports found. Pair HC-05 first.", "err")
            return
        hwids = {}
        try:
            for p in list_ports.comports():
                hwids[p.device] = p.hwid or ""
        except Exception:
            pass
        self.find_btn.configure(state="disabled", text="Probing...")
        self._log("Probing %d port(s) at %d baud (ping=%s)..." % (len(ports), BAUD_RATE, PROBE_PING))
        t = threading.Thread(target=self._probe_worker, args=(ports, hwids), daemon=True)
        t.start()
    def _probe_worker(self, ports, hwids):
        results = []
        for port in ports:
            openable, replied, sample = probe_port_once(port, BAUD_RATE)
            results.append((port, openable, replied, sample, hwids.get(port, "")))
        self.events.put(("probe_done", results))
    def _probe_finished(self, results):
        self.find_btn.configure(state="normal", text="Find HC-05")
        with_reply = [r for r in results if r[1] and r[2]]
        open_only = [r for r in results if r[1] and not r[2]]
        dead = [r for r in results if not r[1]]
        for port, openable, replied, sample, hwid in results:
            tag = " <- matches HC-05" if (HC05_HWID_HINT and HC05_HWID_HINT in (hwid or "")) else ""
            if replied:
                self._log("  %s: OPEN + REPLY [%s]%s <- likely HC-05/Arduino" % (port, sample, tag), "rx")
            elif openable:
                self._log("  %s: open OK, silent (no reply - still a candidate)%s" % (port, tag))
            else:
                self._log("  %s: cannot open (in use / wrong port)" % port, "err")
        hinted_open = [r for r in open_only
                       if HC05_HWID_HINT and HC05_HWID_HINT in (r[4] or "")]
        pick = None
        if with_reply:
            pick = with_reply[0][0]
        elif hinted_open:
            pick = hinted_open[0][0]
        elif open_only:
            pick = open_only[0][0]
        if pick:
            for val in self.port_box["values"]:
                if val.startswith(pick + " ") or val == pick:
                    self.port_var.set(val)
                    break
            else:
                self.port_var.set(pick)
            if with_reply:
                self._log("Auto-selected %s (it replied - strongest match)." % pick, "rx")
            else:
                self._log("Auto-selected %s (opens OK but silent - press Connect to test)." % pick)
        else:
            self._set_status("Error", "red")
            self._log("No usable port. Dead: %s. Is HC-05 powered + paired + not held by phone?" % (
                ", ".join(r[0] for r in dead) or "none"), "err")
    def _disconnect(self, error=None):
        if self.link:
            self.link.disconnect()
        self.held_keys.clear()
        self.connect_btn.configure(text="Connect", bg="#2563eb", activebackground="#1d4ed8")
        if error:
            self._set_status("Error", "#dc2626")
            self._log(error, "err")
        else:
            self._set_status("Disconnected", "#64748b")
            self._log("Disconnected")
        self._set_mode(False, send=False)
    def _set_status(self, text, color):
        dot = "●"
        self.status_pill.configure(text=f"{dot} {text}", bg=color, fg="#ffffff")
    def _transmit(self, text, note=""):
        if not (self.link and self.link.connected):
            self._log("Not connected - command not sent", "err")
            return False
        try:
            self.link.send(text)
        except Exception as exc:
            self._disconnect(error="Write failed: %s" % exc)
            return False
        shown = repr(text) if (not text.isprintable() or text != text.strip()) else text
        self._log("TX: %s%s" % (shown, note), "tx")
        self.last_cmd_var.set("Last command: %s%s" % (shown, note))
        return True
    def _send_drive(self, digit, quiet_mode=False):
        if self.autonomous and not quiet_mode:
            self._set_mode(False, send=True)

        # Track drive duration for calibration when moving forward (digit '8')
        now = time.time()
        if digit == CMD_MAP["F"]:
            self.drive_start_time = now
        elif digit == STOP_CMD:
            if self.drive_start_time is not None:
                self.last_drive_elapsed = max(0.01, now - self.drive_start_time)
                self.drive_start_time = None
                if hasattr(self, "last_drive_lbl"):
                    self.last_drive_lbl.config(text=f"Last Forward Drive: {self.last_drive_elapsed:.2f} s")
        else:
            self.drive_start_time = None

        label = DIGIT_TO_LABEL.get(digit, "?")
        self._transmit(digit, " (%s)" % label)

    def _load_calibration(self):
        if os.path.exists(CALIBRATION_FILE):
            try:
                with open(CALIBRATION_FILE, "r", encoding="utf-8") as f:
                    data = json.load(f)
                    self.calib_speed = float(data.get("speed_cm_s", 68.0))
                    self.calib_turn_ms = int(data.get("turn_90_ms", 220))
            except Exception:
                pass

    def _save_calibration(self):
        try:
            turn_ms = int(self.turn_ms_var.get())
        except ValueError:
            turn_ms = self.calib_turn_ms

        data = {
            "speed_cm_s": round(self.calib_speed, 2),
            "turn_90_ms": turn_ms
        }
        try:
            with open(CALIBRATION_FILE, "w", encoding="utf-8") as f:
                json.dump(data, f, indent=2)
            self.calib_turn_ms = turn_ms
            self.calib_summary_lbl.config(text=f"Speed: {self.calib_speed:.1f} cm/s | Turn: {self.calib_turn_ms} ms")
            self._log(f"Saved calibration: Speed={self.calib_speed:.1f} cm/s, Turn 90°={self.calib_turn_ms} ms", "rx")
            messagebox.showinfo("Calibration Saved", f"Successfully saved calibration to calibration.json!\n\nSpeed: {self.calib_speed:.1f} cm/s\nTurn 90°: {self.calib_turn_ms} ms")
        except Exception as e:
            self._log(f"Failed to save calibration: {e}", "err")
            messagebox.showerror("Error", f"Failed to save calibration: {e}")

    def _calculate_speed(self):
        if self.last_drive_elapsed <= 0.05:
            messagebox.showwarning("No Drive Recorded", "Please press and hold '▲ (F)' to drive the car forward for a measured run first!")
            return
        try:
            dist = float(self.measured_dist_var.get())
            if dist <= 0:
                raise ValueError()
        except ValueError:
            messagebox.showerror("Invalid Input", "Please enter a valid measured distance in cm (e.g. 100).")
            return

        self.calib_speed = dist / self.last_drive_elapsed
        self.calib_summary_lbl.config(text=f"Speed: {self.calib_speed:.1f} cm/s | Turn: {self.calib_turn_ms} ms")
        self._log(f"Calculated Speed: {dist:.1f} cm / {self.last_drive_elapsed:.2f} s = {self.calib_speed:.1f} cm/s", "rx")
        self._save_calibration()

    def _test_90_turn(self):
        if not (self.link and self.link.connected):
            messagebox.showerror("Not Connected", "Please connect to the Car via Bluetooth first.")
            return
        try:
            ms = int(self.turn_ms_var.get())
        except ValueError:
            ms = 220

        def _turn_thread():
            self._log(f"Testing 90° Right Turn pulse ({ms} ms)...")
            self.link.send(CMD_MAP["R"])
            time.sleep(ms / 1000.0)
            self.link.send(STOP_CMD)
            self._log("Turn complete. Check if the car turned exactly 90 degrees.")

        threading.Thread(target=_turn_thread, daemon=True).start()
    def _send_raw(self):
        text = self.raw_var.get()
        if not text:
            return
        if self.newline_var.get():
            text += "\n"
        if self._transmit(text):
            self.raw_var.set("")
    def _toggle_mode(self):
        self._set_mode(not self.autonomous, send=True)
    def _set_mode(self, auto, send):
        if send and not self._transmit(MODE_AUTO if auto else MODE_MANUAL, " (mode)"):
            return
        if not auto and send:
            # Always follow MANUAL with a STOP command so the car stops right away
            self._transmit(STOP_CMD, " (S)")
        self.autonomous = auto
        if auto:
            self.auto_btn.configure(text="AUTO: ON (click to stop)", bg="#c0392b")
        else:
            self.auto_btn.configure(text="MODE: AUTO (FREEPLAY)", bg="#1e8449")

    def _open_goal_maze(self):
        if self.route_window is not None and tk.Toplevel.winfo_exists(self.route_window):
            self.route_window.lift()
            self.route_window.focus_force()
            return

        try:
            from route_planner_app import RoutePlannerApp
        except ImportError as e:
            self._log("Cannot load Goal Maze window: %s" % e, "err")
            return

        top = tk.Toplevel(self.root)
        self.route_window = top
        self._log("Opened Goal Maze Mode window (connected to Bluetooth).")
        self.route_app = RoutePlannerApp(top, shared_link=self.link)

        def _on_maze_close():
            try:
                self.route_app._on_close()
            except Exception:
                pass
            self.route_window = None
            self.route_app = None

        top.protocol("WM_DELETE_WINDOW", _on_maze_close)

    def _open_classroom_maze(self):
        if self.classroom_window is not None and tk.Toplevel.winfo_exists(self.classroom_window):
            self.classroom_window.lift()
            self.classroom_window.focus_force()
            return

        try:
            from classroom_maze_app import ClassroomMazeApp
        except ImportError as e:
            self._log("Cannot load Classroom Maze window: %s" % e, "err")
            return

        top = tk.Toplevel(self.root)
        self.classroom_window = top
        self._log("Opened Classroom Maze Window.")
        self.classroom_app = ClassroomMazeApp(top, shared_link=self.link)

        def _on_cls_close():
            self.classroom_window = None
            self.classroom_app = None

        top.protocol("WM_DELETE_WINDOW", _on_cls_close)
    def _typing_in_entry(self):
        w = self.root.focus_get()
        return isinstance(w, (tk.Entry, ttk.Entry, ttk.Combobox))
    def _on_key_press(self, event):
        digit = KEY_TO_DIGIT.get(event.keysym)
        if digit is None or self._typing_in_entry():
            return
        if digit in self.held_keys:
            return
        self.held_keys.add(digit)
        self._send_drive(digit)
    def _on_key_release(self, event):
        digit = KEY_TO_DIGIT.get(event.keysym)
        if digit is None or digit not in self.held_keys:
            return
        self.held_keys.discard(digit)
        self._send_drive(STOP_CMD, quiet_mode=True)
    def _log(self, msg, tag=""):
        stamp = datetime.now().strftime("%H:%M:%S.%f")[:-3]
        self.log.configure(state="normal")
        self.log.insert("end", "%s  %s\n" % (stamp, msg), tag)
        lines = int(self.log.index("end-1c").split(".")[0])
        if lines > LOG_MAX_LINES:
            self.log.delete("1.0", "%d.0" % (lines - LOG_MAX_LINES))
        self.log.see("end")
        self.log.configure(state="disabled")
    def _poll(self):
        try:
            while True:
                kind, payload = self.events.get_nowait()
                if kind == "rx":
                    self._log("RX: %s" % payload, "rx")
                    if self.classroom_app is not None:
                        try:
                            self.classroom_app.handle_rx_message(payload)
                        except Exception:
                            pass
                    if self.route_app is not None:
                        try:
                            self.route_app.handle_rx_message(payload)
                        except Exception:
                            pass
                    if "FAULT:ULTRASONIC_LOOSE" in payload:
                        # Sensor wire loose alert for all non-manual modes (Auto roam, Maze, etc.)
                        self._log("⚠️ ALERT: Check ultrasonic sensor connectivity! Wire loose (stuck at 0 or timeout)!", "err")
                        if self.autonomous:
                            self._set_mode(False)  # stop auto roam to prevent crash
                        messagebox.showwarning(
                            "Ultrasonic Sensor Alert",
                            "WARNING: Check Ultrasonic Sensor Connectivity!\n\n"
                            "The sensor reading is stuck at 0 cm or 400+ cm.\n"
                            "Please check the Trig, Echo, VCC, and GND jumper wires on the car."
                        )
                elif kind == "probe_done":
                    self._probe_finished(payload)
                elif kind == "error":
                    self._disconnect(error=payload)
        except queue.Empty:
            pass
        self.root.after(POLL_MS, self._poll)
    def _on_close(self):
        try:
            if self.link and self.link.connected:
                self.link.send(STOP_CMD)
        except Exception:
            pass
        if self.link:
            self.link.disconnect()
        self.root.destroy()
def main():
    try:
        root = tk.Tk()
        CarApp(root)
        root.mainloop()
    except KeyboardInterrupt:
        pass
    except Exception:
        try:
            with open("app_error.log", "a") as f:
                f.write(traceback.format_exc() + "\n")
        except Exception:
            pass
        raise
if __name__ == "__main__":
    main()


