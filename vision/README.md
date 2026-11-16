# Vision: colour and position of the cube

A USB webcam looks at the pick spot. This package answers one question before each grasp: **is there a cube, which colour
is it, and where is it in the arm's frame?** Plain Python with OpenCV and NumPy. No ROS and no trained model.

| Priority | Output |
|---|---|
| Required | Colour of the cube: `red`, `blue`, `unknown`, or `none` |
| Planned | `x`, `y` of the cube in the arm's frame (mm) |
| Extra | `z` of the cube centre (mm). The cube rests on the table, so it is half the cube height (15 mm for a 30 mm cube) |

## How it works

One webcam on its own stand (not on the arm), about 300 mm above the table and 220 mm behind the pick spot, looking down at it.
A printed A4 **pick mat** is taped under the cube. It has four ArUco markers, a plain grey surface and a cross for the pick spot.
For every frame:

1. find the four markers (gives the camera pose and a top-down view of the mat);
2. white balance: scale the colour channels so the grey mat looks neutral grey (removes the colour and brightness of the light);
3. find the coloured blob around the cross (present if larger than 250 mm²);
4. compute its hue and compare it with the stored red and blue; accept within 28°, otherwise `unknown`;
5. turn the blob position into `x, y` in the arm's frame, correcting for the camera seeing the sides of the cube.

`Vision.read_stable()` repeats this on 5 frames and keeps the answer only if 4 agree.

## Setup

```bash
pip install -r vision/requirements.txt     # opencv-python, numpy
```

Run everything from the repository root (so `python -m vision...` finds the package).

| Step | Command |
|---|---|
| Print the mat (A4, 100 % scale) | `python -m vision.mat`, then print `vision/assets/pick_mat_A4_300dpi.png` |
| Calibrate the lens (printed chessboard, 9x6 inner corners, 25 mm squares) | `python -m vision.calibrate intrinsics --camera 0` |
| Calibrate the colours (cube on the cross, 6 frames each) | `python -m vision.calibrate colours --camera 0 --names red blue` |
| Check marker error and position | `python -m vision.calibrate check --camera 0` |
| Live viewer, or one JSON line per decision | `python -m vision.live --camera 0` (add `--json`) |

Use `--pick X Y` if the pick cross is not at x=280, y=0 mm. The result is saved in `vision/calib.json`.
`vision/calib_sim.json` is an example made from simulated images.

## Use from the motion code

```python
from vision.live import Vision

cam = Vision(camera=0)
r = cam.read_stable()
# r.present      bool
# r.colour       "red" | "blue" | "unknown" | "none"
# r.confidence   0..1
# r.x_mm, r.y_mm, r.z_mm   cube centre in the arm's frame, millimetres
if r.present and r.colour in ("red", "blue"):
    ...   # pick at (r.x_mm, r.y_mm, r.z_mm), then place in the matching bin
```

**Arm frame.** Origin on the base axis, `x` towards the pick spot, `y` to the left of `x` seen from above, `z` up.
The printed cross defines the pick spot (`pick_xy` in the calibration). If the motion code uses another origin or axis
direction, convert `r.x_mm, r.y_mm` there.

## Results so far

Tested on 600 simulated camera frames (noise, JPEG compression, light from 2700 K to 8000 K and 0.35 to 1.5 times the normal
brightness): 100 % correct, 0 of 477 red or blue cubes sent to the wrong bin, green and yellow cubes refused, empty mat
reported as empty, median position error 1.6 mm. Without the white balance only 81.7 % are correct.
**This is simulation only: the real webcam, lens and cube colours are not tested yet.**

Details: [`docs/E6_computer_vision.md`](docs/E6_computer_vision.md). The pages written for the design book (section 6) are in
[`docs/book_section6/`](docs/book_section6/) (`Section6_ComputerVision.pdf`, LaTeX source and figures).

The simulation that produced these frames (MuJoCo model of the cell, `experiments/e6_vision.py`) is in
https://github.com/Moetez-Fradi/CV-robotic-arm.
