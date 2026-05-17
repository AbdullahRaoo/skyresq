#!/usr/bin/env python3
"""
Z-1 Mini camera intrinsics calibration.

Closes geo-audit blockers #1 (real calibration) and #3 (resolution
consistency) in one artifact: it produces an intrinsics YAML whose
fx/fy/cx/cy are expressed at the SAME resolution geo_localiser maps
detections into, so no hidden scale mismatch remains.

Why this matters
----------------
person_detector emits a normalised (-1..1) pixel error. geo_localiser
reconstructs pixels as `u = cx + ex*(image_width/2)` in an
`image_width x image_height` frame, then rays through (fx,fy,cx,cy).
For the ray to be geometrically correct, (fx,fy,cx,cy) MUST be the
intrinsics at that exact `image_width x image_height`. This tool
calibrates at the capture resolution and rescales the intrinsics to a
chosen target frame (default 320x240, matching the geo node default),
writing both so the deployment is internally consistent.

Usage
-----
  # Live grab from the gimbal RTSP stream (SPACE=capture, q=finish):
  python3 calibrate_camera.py --source rtsp://<pi>:8554/cam \
      --cols 9 --rows 6 --square-mm 25

  # Or from a folder of already-captured checkerboard images:
  python3 calibrate_camera.py --source ./calib_imgs \
      --cols 9 --rows 6 --square-mm 25

  # Target the frame geo_localiser uses (must match the unit's
  # image_width/image_height — default 320x240):
  python3 calibrate_camera.py ... --target-w 320 --target-h 240

`--cols`/`--rows` are INNER corners (a 10x7-square board = 9x6 inner).

Output: camera_intrinsics_z1mini.yaml (next to the config dir by
default) in the exact shape geo_localiser._load_intrinsics expects.

After running
-------------
1. Set the systemd unit so the running node actually loads it AND uses
   the same target frame (the launch file does NOT govern the service):

   ops/systemd/skyresq-geo-localiser.service ExecStart →
     ... ros2 run drone_vision geo_localiser --ros-args \
       -p intrinsics_file:=<abs path to this yaml> \
       -p image_width:=<target-w> -p image_height:=<target-h>

2. daemon-reload + restart skyresq-geo-localiser; the startup banner
   must show this yaml and fx≈ the value printed below.
"""
import argparse
import glob
import os
import sys

import numpy as np

try:
    import cv2
except ImportError:
    sys.exit("OpenCV (cv2) is required: pip install opencv-python")


def collect_images(source):
    """Return a list of BGR frames from a folder, or interactively grab
    them from a video/RTSP source (SPACE=keep, q=done)."""
    if os.path.isdir(source):
        paths = sorted(
            p for ext in ("jpg", "jpeg", "png", "bmp")
            for p in glob.glob(os.path.join(source, f"*.{ext}"))
        )
        if not paths:
            sys.exit(f"No images found in {source}")
        frames = [cv2.imread(p) for p in paths]
        print(f"[calib] loaded {len(frames)} images from {source}")
        return [f for f in frames if f is not None]

    cap = cv2.VideoCapture(source)
    if not cap.isOpened():
        sys.exit(f"Cannot open source: {source}")
    print("[calib] live capture — SPACE to keep a frame, q to finish. "
          "Aim for 15–25 views of the board at varied angles/distances.")
    frames = []
    while True:
        ok, frame = cap.read()
        if not ok:
            print("[calib] stream read failed")
            break
        view = frame.copy()
        cv2.putText(view, f"kept: {len(frames)}  (SPACE=keep, q=done)",
                    (12, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
        cv2.imshow("calib", view)
        k = cv2.waitKey(1) & 0xFF
        if k == ord(" "):
            frames.append(frame.copy())
            print(f"[calib] kept frame {len(frames)}")
        elif k == ord("q"):
            break
    cap.release()
    cv2.destroyAllWindows()
    return frames


def calibrate(frames, cols, rows, square_m):
    objp = np.zeros((rows * cols, 3), np.float32)
    objp[:, :2] = np.mgrid[0:cols, 0:rows].T.reshape(-1, 2)
    objp *= square_m

    objpoints, imgpoints = [], []
    cap_w = cap_h = None
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 1e-3)
    found = 0
    for f in frames:
        gray = cv2.cvtColor(f, cv2.COLOR_BGR2GRAY)
        cap_h, cap_w = gray.shape[:2]
        ok, corners = cv2.findChessboardCorners(
            gray, (cols, rows),
            cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE)
        if not ok:
            continue
        corners = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
        objpoints.append(objp)
        imgpoints.append(corners)
        found += 1
    if found < 8:
        sys.exit(f"Only {found} usable views — need ≥8 (ideally 15+). "
                 "Recapture with the whole board visible, varied angles.")
    print(f"[calib] {found} usable views @ {cap_w}x{cap_h}")
    rms, K, dist, _, _ = cv2.calibrateCamera(
        objpoints, imgpoints, (cap_w, cap_h), None, None)
    return rms, K, dist, cap_w, cap_h


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--source", required=True,
                    help="RTSP/video URL, or a folder of board images")
    ap.add_argument("--cols", type=int, required=True,
                    help="inner corners across (squares-1)")
    ap.add_argument("--rows", type=int, required=True,
                    help="inner corners down (squares-1)")
    ap.add_argument("--square-mm", type=float, required=True,
                    help="checkerboard square size in millimetres")
    ap.add_argument("--target-w", type=int, default=320,
                    help="resolution geo_localiser maps into (image_width)")
    ap.add_argument("--target-h", type=int, default=240,
                    help="resolution geo_localiser maps into (image_height)")
    ap.add_argument("--out", default=os.path.join(
        os.path.dirname(__file__), "..", "config",
        "camera_intrinsics_z1mini.yaml"))
    args = ap.parse_args()

    frames = collect_images(args.source)
    rms, K, dist, cw, ch = calibrate(
        frames, args.cols, args.rows, args.square_mm / 1000.0)

    # Rescale intrinsics from capture (cw x ch) to the geo frame.
    sx = args.target_w / float(cw)
    sy = args.target_h / float(ch)
    fx = float(K[0, 0] * sx)
    fy = float(K[1, 1] * sy)
    cx = float(K[0, 2] * sx)
    cy = float(K[1, 2] * sy)

    out = os.path.abspath(args.out)
    with open(out, "w") as fh:
        fh.write(
            "# Z-1 Mini intrinsics — checkerboard calibrated.\n"
            f"# RMS reprojection error: {rms:.3f} px (aim < 0.5).\n"
            f"# Calibrated at {cw}x{ch}, rescaled to geo frame "
            f"{args.target_w}x{args.target_h}.\n"
            "# geo_localiser image_width/image_height MUST equal the\n"
            "# width/height below, set via the systemd unit ExecStart.\n"
            "camera:\n"
            f"  width:  {args.target_w}\n"
            f"  height: {args.target_h}\n"
            f"  fx: {fx:.4f}\n"
            f"  fy: {fy:.4f}\n"
            f"  cx: {cx:.4f}\n"
            f"  cy: {cy:.4f}\n"
            "  distortion_model: \"plumb_bob\"\n"
            f"  D: [{', '.join(f'{v:.6f}' for v in dist.flatten()[:5])}]\n"
        )

    print(f"\n[calib] RMS reprojection error = {rms:.3f} px "
          f"({'GOOD' if rms < 0.5 else 'recapture — too high' if rms > 1.0 else 'ok'})")
    print(f"[calib] geo-frame intrinsics ({args.target_w}x{args.target_h}): "
          f"fx={fx:.1f} fy={fy:.1f} cx={cx:.1f} cy={cy:.1f}")
    print(f"[calib] wrote {out}")
    print("[calib] NEXT: point skyresq-geo-localiser.service ExecStart at "
          f"this file with -p image_width:={args.target_w} "
          f"-p image_height:={args.target_h}, then restart the service.")


if __name__ == "__main__":
    main()
