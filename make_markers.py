"""make_markers.py - printable ArUco markers for route planner.
Saves IDs 0,1,2,3 (arena corners) and 10 (car) as PNGs with white quiet-zone border.

Supports both new (generateImageMarker) and old (drawMarker) OpenCV APIs.
Run:  python make_markers.py
Output: markers/marker_0.png ... marker_10.png

Print: matt (non-glossy) paper, flat. Corners ~8-10 cm, car ~6-8 cm.
In Word / printer dialog set exact size and disable fit-to-page.
Border in file is >=1 cm when printed at those sizes (we add 25% white margin).
"""
import os
import cv2
import numpy as np

MARKER_IDS = [0, 1, 2, 3, 10, 20]
CAR_ID = 10
GOAL_ID = 20
MARKER_PX = 800          # marker itself (without border)
BORDER_PX = 200          # white quiet zone (~25% -> >1cm at 8cm print)
OUT_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "markers")


def get_dict():
    d = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    return d


def make_marker(dictionary, mid, size_px):
    img = np.zeros((size_px, size_px), dtype=np.uint8)
    if hasattr(cv2.aruco, "generateImageMarker"):
        # new API: generateImageMarker(dict, id, sidePixels [, img [, borderBits]])
        try:
            img = cv2.aruco.generateImageMarker(dictionary, mid, size_px)
        except TypeError:
            cv2.aruco.generateImageMarker(dictionary, mid, size_px, img)
    elif hasattr(cv2.aruco, "drawMarker"):
        # old API
        try:
            img = cv2.aruco.drawMarker(dictionary, mid, size_px)
        except TypeError:
            cv2.aruco.drawMarker(dictionary, mid, size_px, img, 1)
    else:
        raise RuntimeError("No ArUco marker generator found in this OpenCV build")
    return img


def main():
    os.makedirs(OUT_DIR, exist_ok=True)
    dictionary = get_dict()
    for mid in MARKER_IDS:
        m = make_marker(dictionary, mid, MARKER_PX)
        # add white border (quiet zone)
        full = cv2.copyMakeBorder(m, BORDER_PX, BORDER_PX, BORDER_PX, BORDER_PX,
                                  cv2.BORDER_CONSTANT, value=255)
        path = os.path.join(OUT_DIR, "marker_%d.png" % mid)
        cv2.imwrite(path, full)
        print("saved %s  (marker %dpx + border %dpx each side)" % (path, MARKER_PX, BORDER_PX))
    print("\nPrint flat on matt paper, no gloss/lamination.")
    print("Corner IDs 0=TL 1=TR 2=BR 3=BL as seen by camera. Car ID=10, top edge = front.")


if __name__ == "__main__":
    main()
