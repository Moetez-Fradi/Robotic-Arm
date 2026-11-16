"""Live colour detection: the module the task sequencer uses, and a viewer for testing.

  uv run python -m vision.live --camera 0            viewer with the decision drawn on the image
  uv run python -m vision.live --camera 0 --json     one JSON line per decision on stdout, no window
  uv run python -m vision.live --image frame.png     analyse a saved frame

In code:
    cam = Vision(camera=0)
    r = cam.read_stable()       # -> Result(present, colour, confidence, x_mm, y_mm, ...)
    if r.present and r.colour in ("red", "blue"): ...
"""
from __future__ import annotations

import argparse
import json
import sys
from collections import Counter
from dataclasses import asdict

import cv2

from .calibrate import default_calib, open_camera
from .detector import CALIB_PATH, Calib, Result, analyse, draw_overlay


class Vision:
    def __init__(self, camera: int = 0, calib: Calib | None = None):
        self.cap = open_camera(camera)
        ok, f = self.cap.read()
        if not ok:
            raise RuntimeError("camera gives no frames")
        self.calib = calib or default_calib(f.shape[1], f.shape[0], None)
        if not self.calib.colours:
            raise RuntimeError(f"no colours calibrated: run `python -m vision.calibrate colours` ({CALIB_PATH})")

    def read(self) -> Result:
        ok, f = self.cap.read()
        return analyse(f, self.calib) if ok else Result(False)

    def read_stable(self, n: int = 5, agree: int = 4) -> Result:
        """n frames; return the majority decision only if at least `agree` frames agree, else 'unknown'."""
        rs = [self.read() for _ in range(n)]
        votes = Counter((r.colour if r.present else "none") for r in rs)
        label, count = votes.most_common(1)[0]
        best = next(r for r in rs if (r.colour if r.present else "none") == label)
        if count < agree:
            best.colour, best.confidence = "unknown", 0.0
        return best

    def close(self):
        self.cap.release()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--camera", type=int, default=0)
    ap.add_argument("--image")
    ap.add_argument("--json", action="store_true")
    a = ap.parse_args()
    if a.image:
        f = cv2.imread(a.image)
        calib = default_calib(f.shape[1], f.shape[0], None)
        dbg = {}
        r = analyse(f, calib, debug=dbg)
        print(json.dumps(asdict(r)))
        cv2.imshow("vision", draw_overlay(f, r, dbg))
        cv2.waitKey(0)
        return
    v = Vision(a.camera)
    try:
        while True:
            if a.json:
                print(json.dumps(asdict(v.read_stable())), flush=True)
                continue
            ok, f = v.cap.read()
            if not ok:
                continue
            dbg = {}
            r = analyse(f, v.calib, debug=dbg)
            cv2.imshow("vision (ESC quits)", draw_overlay(f, r, dbg))
            if cv2.waitKey(1) & 0xFF == 27:
                break
    except KeyboardInterrupt:
        pass
    finally:
        v.close()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    sys.exit(main())
