"""cam_selftest.py - silent headless check: open phone stream, detect markers, time it.
Run: python cam_selftest.py [url]
"""
import sys
import time

import cv2

URL = sys.argv[1] if len(sys.argv) > 1 else "http://192.168.137.252:8080/video"

sys.path.insert(0, __import__("os").path.dirname(__import__("os").path.abspath(__file__)))
from route_planner_app import detect_markers_both

print("opening", URL, flush=True)
cap = cv2.VideoCapture(URL)
try:
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
except Exception:
    pass
t0 = time.time()
ok = cap.isOpened()
print("opened:", ok, "after %.1fs" % (time.time() - t0), flush=True)
if not ok:
    sys.exit(1)
for i in range(5):
    t0 = time.time()
    ok, frame = cap.read()
    tr = time.time() - t0
    if not ok or frame is None:
        print("frame %d: READ FAIL" % i, flush=True)
        continue
    h, w = frame.shape[:2]
    small = frame
    if max(h, w) > 800:
        sc = 800.0 / max(h, w)
        small = cv2.resize(frame, (int(w * sc), int(h * sc)))
    t1 = time.time()
    det = detect_markers_both(small)
    td = (time.time() - t1) * 1000
    print("frame %d: %dx%d read=%.0fms detect=%.0fms ids=%s" % (
        i, w, h, tr * 1000, td, sorted(det.keys())), flush=True)
cap.release()
print("SELFTEST_DONE", flush=True)
