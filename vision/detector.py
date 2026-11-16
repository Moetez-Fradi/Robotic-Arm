"""Colour detection and object localisation for the pick location.

One call, `analyse(frame_bgr, calib)`, does the whole job on a camera frame:

  1. find the four ArUco markers of the pick mat               -> mat found or not
  2. image -> mat-plane homography and camera pose (solvePnP)    -> image-to-arm calibration
  3. warp the pick area to a top-down view, 2 px per mm          -> fixed geometry, any camera angle
  4. white balance on the grey mat (a grey card always in view)  -> colour independent of the lighting
  5. segment saturated pixels near the pick cross               -> object present or not
  6. hue of the object vs calibrated colour centroids            -> red / blue / unknown
  7. blob centroid -> ray from the camera -> object centre        -> object position in arm mm

Only OpenCV and NumPy. Frames are BGR as delivered by cv2.VideoCapture.
"""
from __future__ import annotations

import json
import time
from dataclasses import asdict, dataclass, field
from pathlib import Path

import cv2
import numpy as np

from . import mat

PX_PER_MM = 2.0          # resolution of the rectified top-down view
ROI_RADIUS = 60.0        # mm around the pick cross searched for the object
CALIB_PATH = Path(__file__).resolve().parent / "calib.json"


@dataclass
class Calib:
    """Everything the detector needs; saved as vision/calib.json."""
    K: list                      # 3x3 camera matrix (pixels)
    dist: list                   # distortion coefficients
    pick_xy: list                # pick cross position in arm mm
    colours: dict = field(default_factory=dict)   # name -> {"hue": deg, "sat": 0-1}
    hue_tol: float = 28.0        # max hue distance (deg) to accept a colour
    min_sat: float = 0.28        # normalised saturation that counts as "coloured"
    min_area_mm2: float = 250.0  # smallest blob accepted as an object
    object_height_mm: float = 30.0
    centroid_height_frac: float = 0.5   # height of the visible blob centre, as a fraction of the object height

    @classmethod
    def load(cls, path=CALIB_PATH) -> "Calib":
        return cls(**json.loads(Path(path).read_text(encoding="utf-8")))

    def save(self, path=CALIB_PATH) -> None:
        Path(path).write_text(json.dumps(asdict(self), indent=2), encoding="utf-8")


@dataclass
class Result:
    mat_found: bool
    present: bool = False
    colour: str = "none"           # "red", "blue", ..., "unknown" or "none"
    confidence: float = 0.0
    x_mm: float = float("nan")     # object centre in the arm frame
    y_mm: float = float("nan")
    z_mm: float = float("nan")      # object centre height above the table (from the known table plane and cube size)
    hue: float = float("nan")
    sat: float = float("nan")
    area_mm2: float = 0.0
    reproj_px: float = float("nan")   # marker reprojection error: calibration quality
    ms: float = 0.0


def intrinsics_from_fov(width: int, height: int, fovy_deg: float) -> np.ndarray:
    """Pinhole camera matrix for square pixels, from the vertical field of view."""
    f = (height / 2) / np.tan(np.radians(fovy_deg) / 2)
    return np.array([[f, 0, width / 2], [0, f, height / 2], [0, 0, 1]], dtype=np.float64)


_DETECTOR = cv2.aruco.ArucoDetector(cv2.aruco.getPredefinedDictionary(mat.DICT), cv2.aruco.DetectorParameters())


def find_markers(gray):
    """{id: 4x2 image corners} for the mat markers that are visible."""
    corners, ids, _ = _DETECTOR.detectMarkers(gray)
    found = {}
    if ids is not None:
        for c, i in zip(corners, ids.ravel()):
            if int(i) in mat.MARKER_TL:
                found[int(i)] = c.reshape(4, 2).astype(np.float64)
    return found


def mat_pose(found, calib: Calib):
    """Homography image->mat (mm) and camera centre in arm mm, from >= 2 markers (8 points)."""
    img_pts = np.vstack([found[i] for i in sorted(found)])
    mat_pts = np.vstack([mat.marker_corners_mat(i) for i in sorted(found)])
    H, _ = cv2.findHomography(img_pts, mat_pts)
    arm_xy = mat.mat_to_arm(mat_pts, calib.pick_xy)
    obj = np.column_stack([arm_xy, np.zeros(len(arm_xy))])
    K, dist = np.array(calib.K), np.array(calib.dist, dtype=np.float64)
    ok, rvec, tvec = cv2.solvePnP(obj, img_pts, K, dist, flags=cv2.SOLVEPNP_IPPE)
    if not ok:
        return H, None, float("nan")
    R, _ = cv2.Rodrigues(rvec)
    centre = (-R.T @ tvec).ravel()
    proj, _ = cv2.projectPoints(obj, rvec, tvec, K, dist)
    reproj = float(np.sqrt(np.mean(np.sum((proj.reshape(-1, 2) - img_pts) ** 2, axis=1))))
    return H, centre, reproj


def rectify(frame, H):
    """Top-down view of the whole mat, PX_PER_MM pixels per mm."""
    S = np.diag([PX_PER_MM, PX_PER_MM, 1.0])
    size = (int(mat.MAT_W * PX_PER_MM), int(mat.MAT_H * PX_PER_MM))
    return cv2.warpPerspective(frame, S @ H, size, flags=cv2.INTER_LINEAR)


def mat_area_mask(shape):
    """Pixels of the rectified view that show bare grey mat (no markers, no printed cross)."""
    h, w = shape[:2]
    m = np.full((h, w), 255, np.uint8)
    q = 6 * PX_PER_MM
    for mid in mat.MARKER_TL:
        c, r = mat.MARKER_TL[mid]
        cv2.rectangle(m, (int(c * PX_PER_MM - q), int(r * PX_PER_MM - q)),
                      (int((c + mat.MARKER) * PX_PER_MM + q), int((r + mat.MARKER) * PX_PER_MM + q)), 0, -1)
    cv2.rectangle(m, (0, h - int(8 * PX_PER_MM)), (w, h), 0, -1)        # printed text strip
    return m


def white_balance(top, obj_mask=None):
    """Divide each channel by the median of the bare grey mat, so the mat comes out neutral grey."""
    ref_mask = mat_area_mask(top.shape)
    if obj_mask is not None:
        ref_mask[cv2.dilate(obj_mask, np.ones((21, 21), np.uint8)) > 0] = 0
    px = top[ref_mask > 0].reshape(-1, 3).astype(np.float64)
    bright = px.sum(axis=1)
    if len(px):
        px = px[bright >= np.percentile(bright, 20)]     # skip shadows and printed lines (>= keeps a uniform mat)
    ref = np.median(px, axis=0) if len(px) else np.full(3, mat.GREY * 0.85)   # no mat visible: leave the colours as they are
    gain = (mat.GREY * 0.85) / np.maximum(ref, 1.0)
    return np.clip(top.astype(np.float64) * gain, 0, 255).astype(np.uint8), ref


def _hsv(img_bgr):
    hsv = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2HSV_FULL).astype(np.float64)
    return hsv[..., 0] * 360 / 256, hsv[..., 1] / 255, hsv[..., 2] / 255


def hue_dist(a, b):
    d = abs(a - b) % 360
    return min(d, 360 - d)


def analyse(frame, calib: Calib, wb: bool = True, debug: dict | None = None) -> Result:
    t0 = time.perf_counter()
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    found = find_markers(gray)
    if len(found) < 2:
        return Result(False, ms=(time.perf_counter() - t0) * 1e3)
    H, centre, reproj = mat_pose(found, calib)
    top = rectify(frame, H)
    if wb:
        top, ref = white_balance(top)
    hue, sat, val = _hsv(top)

    # region of interest: a disc around the printed cross
    h, w = top.shape[:2]
    roi = np.zeros((h, w), np.uint8)
    cv2.circle(roi, (w // 2, h // 2), int(ROI_RADIUS * PX_PER_MM), 255, -1)
    mask = ((sat > calib.min_sat) & (val > 0.12) & (roi > 0)).astype(np.uint8) * 255
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((7, 7), np.uint8))
    n, lab, stats, cents = cv2.connectedComponentsWithStats(mask)
    res = Result(True, reproj_px=reproj)
    if n > 1:
        k = 1 + int(np.argmax(stats[1:, cv2.CC_STAT_AREA]))
        area = stats[k, cv2.CC_STAT_AREA] / PX_PER_MM ** 2
        res.area_mm2 = float(area)
        if area >= calib.min_area_mm2:
            blob = lab == k
            if wb:      # redo the white balance without the object in the reference area
                top, ref = white_balance(rectify(frame, H), blob.astype(np.uint8) * 255)
                hue, sat, val = _hsv(top)
            s = sat[blob]
            ang = np.radians(hue[blob])
            mh = float(np.degrees(np.arctan2(np.sum(s * np.sin(ang)), np.sum(s * np.cos(ang)))) % 360)
            res.present, res.hue, res.sat = True, mh, float(np.median(s))
            best, d_best, d_second = "unknown", 999.0, 999.0
            for name, c in calib.colours.items():
                d = hue_dist(mh, c["hue"])
                if d < d_best:
                    best, d_best, d_second = name, d, d_best
                elif d < d_second:
                    d_second = d
            if d_best <= calib.hue_tol:
                res.colour = best
                res.confidence = float(np.clip(1 - d_best / calib.hue_tol, 0, 1))
            else:
                res.colour = "unknown"
            # position: blob centroid on the mat plane -> ray from the camera -> object centre height
            cx_mm, cy_mm = cents[k][0] / PX_PER_MM, cents[k][1] / PX_PER_MM
            p0 = np.array([*mat.mat_to_arm([cx_mm, cy_mm], calib.pick_xy)[0], 0.0])
            if centre is not None and centre[2] > 1:
                z = calib.object_height_mm * calib.centroid_height_frac
                p = centre + (p0 - centre) * (centre[2] - z) / centre[2]
            else:
                p = p0
            res.x_mm, res.y_mm = float(p[0]), float(p[1])
            res.z_mm = calib.object_height_mm / 2
    res.ms = (time.perf_counter() - t0) * 1e3
    if debug is not None:
        debug.update({"top": top, "mask": mask, "markers": found, "H": H})
    return res


def draw_overlay(frame, res: Result, debug: dict) -> np.ndarray:
    """Camera frame with the detected markers, the mat outline and the decision."""
    out = frame.copy()
    for mid, c in debug.get("markers", {}).items():
        cv2.polylines(out, [c.astype(np.int32)], True, (0, 255, 0), 2, cv2.LINE_AA)
        cv2.putText(out, str(mid), tuple(c[0].astype(int)), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
    if "H" in debug:
        Hi = np.linalg.inv(debug["H"])
        corners = np.array([[0, 0], [mat.MAT_W, 0], [mat.MAT_W, mat.MAT_H], [0, mat.MAT_H]], np.float64)
        pts = cv2.perspectiveTransform(corners.reshape(-1, 1, 2), Hi).reshape(-1, 2)
        cv2.polylines(out, [pts.astype(np.int32)], True, (255, 200, 0), 2, cv2.LINE_AA)
        ring = np.array([[mat.MAT_W / 2 + ROI_RADIUS * np.cos(a), mat.MAT_H / 2 + ROI_RADIUS * np.sin(a)]
                         for a in np.linspace(0, 2 * np.pi, 48)])
        rp = cv2.perspectiveTransform(ring.reshape(-1, 1, 2), Hi).reshape(-1, 2)
        cv2.polylines(out, [rp.astype(np.int32)], True, (200, 200, 200), 1, cv2.LINE_AA)
    col = {"red": (60, 60, 255), "blue": (255, 120, 40)}.get(res.colour, (0, 220, 255))
    label = (f"{res.colour.upper()}  {res.confidence:.0%}  x={res.x_mm:.0f} y={res.y_mm:.0f} mm" if res.present
             else ("no object" if res.mat_found else "MAT NOT FOUND"))
    cv2.rectangle(out, (10, 10), (14 + 15 * len(label), 52), (20, 20, 20), -1)
    cv2.putText(out, label, (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.8, col, 2, cv2.LINE_AA)
    return out


def calibrate_colours(samples: dict, calib: Calib, wb: bool = True) -> dict:
    """samples: {name: [frames]} of known objects. Returns {name: {"hue", "sat", "n"}} and stores it in calib."""
    out = {}
    for name, frames in samples.items():
        hs, ss = [], []
        tmp = Calib(**{**asdict(calib), "colours": {}})
        for f in frames:
            r = analyse(f, tmp, wb)
            if r.present:
                hs.append(r.hue)
                ss.append(r.sat)
        if not hs:
            raise ValueError(f"no object found in the calibration frames for '{name}'")
        a = np.radians(hs)
        out[name] = {"hue": float(np.degrees(np.arctan2(np.sin(a).mean(), np.cos(a).mean())) % 360),
                     "sat": float(np.median(ss)), "n": len(hs)}
    calib.colours = out
    return out
