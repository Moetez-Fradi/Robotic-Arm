"""Calibrate the real webcam. Run once at the workshop, in the final lighting.

  uv run python -m vision.calibrate intrinsics --camera 0
      Show a printed chessboard (9 x 6 inner corners, 25 mm squares) in ~15 poses; SPACE grabs, ENTER solves.
      Gives the camera matrix and lens distortion.
  uv run python -m vision.calibrate colours --camera 0 --names red blue
      Put each object on the cross when asked; SPACE grabs a frame (take 6 per colour, moving it slightly).
      Gives the hue centroid of each colour.
  uv run python -m vision.calibrate check --camera 0
      Reports the marker reprojection error and the measured position of an object on the cross.

Results go to vision/calib.json, which vision.live and the sequencer read.
The pick spot (the printed cross) is at x=280, y=0 mm in the arm frame unless --pick X Y is given; set it to the real
position once the cell is built. (In the CV-robotic-arm repository the default comes from its design values.)
"""
from __future__ import annotations

import argparse
import sys

import cv2
import numpy as np

from .detector import CALIB_PATH, Calib, analyse, calibrate_colours, draw_overlay, intrinsics_from_fov


def open_camera(index: int, width=1280, height=720):
    cap = cv2.VideoCapture(index, cv2.CAP_DSHOW if sys.platform == "win32" else cv2.CAP_ANY)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
    cap.set(cv2.CAP_PROP_AUTO_WB, 0)          # the grey mat does the white balance; a drifting camera WB only hurts
    if not cap.isOpened():
        raise SystemExit(f"camera {index} not found")
    return cap


def default_calib(width, height, pick) -> Calib:
    if CALIB_PATH.exists():
        c = Calib.load()
        if pick:
            c.pick_xy = list(pick)
        return c
    if pick is None:
        try:                              # inside the CV-robotic-arm repository: use its design values
            from arm import params as P
            pick = [P.PICK_XY[0] * 1e3, P.PICK_XY[1] * 1e3]
        except ImportError:               # standalone copy of the vision package
            pick = [280.0, 0.0]
    # until the intrinsics are calibrated: a typical webcam, 45 deg vertical field of view
    return Calib(K=intrinsics_from_fov(width, height, 45).tolist(), dist=[0, 0, 0, 0, 0], pick_xy=list(pick))


def grab_loop(cap, prompt, on_frame=None):
    """Show the camera; SPACE returns the frame, ENTER returns None, ESC quits."""
    while True:
        ok, frame = cap.read()
        if not ok:
            continue
        view = on_frame(frame) if on_frame else frame.copy()
        cv2.putText(view, prompt, (10, view.shape[0] - 15), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
        cv2.imshow("calibrate", view)
        k = cv2.waitKey(1) & 0xFF
        if k == 32:
            return frame
        if k == 13:
            return None
        if k == 27:
            raise SystemExit("cancelled")


def intrinsics(cap, calib: Calib, cols=9, rows=6, square=25.0):
    objp = np.zeros((cols * rows, 3), np.float32)
    objp[:, :2] = np.mgrid[0:cols, 0:rows].T.reshape(-1, 2) * square
    obj_pts, img_pts, size = [], [], None
    while True:
        def show(f):
            g = cv2.cvtColor(f, cv2.COLOR_BGR2GRAY)
            ok, c = cv2.findChessboardCorners(g, (cols, rows), cv2.CALIB_CB_FAST_CHECK)
            v = f.copy()
            if ok:
                cv2.drawChessboardCorners(v, (cols, rows), c, ok)
            return v
        f = grab_loop(cap, f"{len(obj_pts)} views. SPACE grab, ENTER solve", show)
        if f is None:
            break
        g = cv2.cvtColor(f, cv2.COLOR_BGR2GRAY)
        ok, c = cv2.findChessboardCorners(g, (cols, rows))
        if ok:
            c = cv2.cornerSubPix(g, c, (11, 11), (-1, -1), (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 1e-3))
            obj_pts.append(objp)
            img_pts.append(c)
            size = g.shape[::-1]
    if len(obj_pts) < 8:
        raise SystemExit("need at least 8 chessboard views")
    rms, K, dist, _, _ = cv2.calibrateCamera(obj_pts, img_pts, size, None, None)
    calib.K, calib.dist = K.tolist(), dist.ravel().tolist()
    print(f"intrinsics: RMS reprojection {rms:.2f} px over {len(obj_pts)} views")


def colours(cap, calib: Calib, names, per_colour=6):
    samples = {}
    for name in names:
        samples[name] = []
        while len(samples[name]) < per_colour:
            f = grab_loop(cap, f"{name}: {len(samples[name])}/{per_colour} - object on the cross, SPACE")
            if f is not None:
                samples[name].append(f)
    out = calibrate_colours(samples, calib)
    for n, c in out.items():
        print(f"{n}: hue {c['hue']:.1f} deg, saturation {c['sat']:.2f} ({c['n']} frames)")
    hues = [c["hue"] for c in out.values()]
    for i in range(len(hues)):
        for j in range(i + 1, len(hues)):
            d = abs(hues[i] - hues[j]) % 360
            if min(d, 360 - d) < 2 * calib.hue_tol:
                print(f"WARNING: {names[i]} and {names[j]} are only {min(d, 360 - d):.0f} deg apart; lower hue_tol in calib.json")


def check(cap, calib: Calib):
    while True:
        dbg = {}
        def show(f):
            r = analyse(f, calib, debug=dbg)
            v = draw_overlay(f, r, dbg)
            cv2.putText(v, f"reprojection {r.reproj_px:.2f} px, {r.ms:.0f} ms", (10, 80), cv2.FONT_HERSHEY_SIMPLEX,
                        0.6, (255, 255, 255), 2)
            return v
        if grab_loop(cap, "ESC to quit", show) is None:
            return


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("step", choices=["intrinsics", "colours", "check"])
    ap.add_argument("--camera", type=int, default=0)
    ap.add_argument("--names", nargs="+", default=["red", "blue"])
    ap.add_argument("--pick", type=float, nargs=2, metavar=("X", "Y"), help="pick cross in arm mm")
    a = ap.parse_args()
    cap = open_camera(a.camera)
    ok, f = cap.read()
    calib = default_calib(f.shape[1], f.shape[0], a.pick)
    try:
        {"intrinsics": lambda: intrinsics(cap, calib), "colours": lambda: colours(cap, calib, a.names),
         "check": lambda: check(cap, calib)}[a.step]()
    finally:
        cap.release()
        cv2.destroyAllWindows()
    if a.step != "check":
        calib.save()
        print("saved", CALIB_PATH)


if __name__ == "__main__":
    main()
