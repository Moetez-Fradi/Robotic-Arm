"""The pick mat: an A4 sheet printed at 100 % scale and taped under the pick location.

It does three jobs:
  * four ArUco markers (4x4 dictionary, IDs 0-3) give the camera pose and the image-to-arm mapping;
  * its neutral grey surface is the white-balance reference (a grey card that is always in view);
  * the printed cross marks where the object is placed.

Geometry (all in mm). The sheet lies flat on the table, landscape, centred on the pick spot.
Mat image coordinates: column c to the right, row r downwards, origin at the top-left corner.
The image's top edge is the side far from the arm base (+x); its left edge is the +y side, so seen
from above with +x pointing up the image, the arm frame is right-handed.
    x_arm = pick_x + MAT_H / 2 - r        y_arm = pick_y + MAT_W / 2 - c
"""
from __future__ import annotations

import cv2
import numpy as np

MAT_W, MAT_H = 297.0, 210.0       # A4 landscape: width across the arm (y), height along it (x)
MARKER = 40.0                     # marker side
MARGIN = 12.0                     # sheet edge to marker edge
GREY = 190                        # printed grey level (0-255), a neutral reference
DICT = cv2.aruco.DICT_4X4_50
# top-left corner of each marker in mat image coordinates (c, r), IDs 0..3 clockwise from top-left
MARKER_TL = {0: (MARGIN, MARGIN), 1: (MAT_W - MARGIN - MARKER, MARGIN),
             2: (MAT_W - MARGIN - MARKER, MAT_H - MARGIN - MARKER), 3: (MARGIN, MAT_H - MARGIN - MARKER)}


def marker_corners_mat(mid: int) -> np.ndarray:
    """4x2 corners (c, r) of marker `mid` in mat mm, in ArUco order: TL, TR, BR, BL."""
    c, r = MARKER_TL[mid]
    return np.array([[c, r], [c + MARKER, r], [c + MARKER, r + MARKER], [c, r + MARKER]], dtype=np.float64)


def mat_to_arm(cr, pick_xy) -> np.ndarray:
    """Mat (c, r) mm -> arm (x, y) mm."""
    cr = np.atleast_2d(np.asarray(cr, dtype=np.float64))
    return np.column_stack([pick_xy[0] + MAT_H / 2 - cr[:, 1], pick_xy[1] + MAT_W / 2 - cr[:, 0]])


def arm_to_mat(xy, pick_xy) -> np.ndarray:
    xy = np.atleast_2d(np.asarray(xy, dtype=np.float64))
    return np.column_stack([pick_xy[1] + MAT_W / 2 - xy[:, 1], pick_xy[0] + MAT_H / 2 - xy[:, 0]])


def render(px_per_mm: float = 300 / 25.4) -> np.ndarray:
    """The mat as a grey-scale image (default 300 dpi, for printing)."""
    w, h = int(round(MAT_W * px_per_mm)), int(round(MAT_H * px_per_mm))
    img = np.full((h, w), GREY, np.uint8)
    d = cv2.aruco.getPredefinedDictionary(DICT)
    for mid, (c, r) in MARKER_TL.items():
        side = int(round(MARKER * px_per_mm))
        # a white quiet zone around each marker makes detection reliable on grey paper
        q = int(round(4 * px_per_mm))
        x0, y0 = int(round(c * px_per_mm)), int(round(r * px_per_mm))
        img[y0 - q:y0 + side + q, x0 - q:x0 + side + q] = 255
        img[y0:y0 + side, x0:x0 + side] = cv2.aruco.generateImageMarker(d, mid, side)
    cx, cy = w // 2, h // 2
    s = lambda v: int(round(v * px_per_mm))
    cv2.circle(img, (cx, cy), s(25), 90, max(1, s(0.6)), cv2.LINE_AA)
    cv2.line(img, (cx - s(8), cy), (cx + s(8), cy), 90, max(1, s(0.6)), cv2.LINE_AA)
    cv2.line(img, (cx, cy - s(8)), (cx, cy + s(8)), 90, max(1, s(0.6)), cv2.LINE_AA)
    cv2.putText(img, "PICK MAT  -  print at 100 %  -  arm base this side", (s(70), h - s(4)), cv2.FONT_HERSHEY_SIMPLEX,
                0.09 * px_per_mm, 110, max(1, s(0.3)), cv2.LINE_AA)
    return img


if __name__ == "__main__":
    from pathlib import Path
    out = Path(__file__).resolve().parent / "assets"
    out.mkdir(exist_ok=True)
    img = render()
    cv2.imwrite(str(out / "pick_mat_A4_300dpi.png"), img)
    print("wrote", out / "pick_mat_A4_300dpi.png", img.shape)
