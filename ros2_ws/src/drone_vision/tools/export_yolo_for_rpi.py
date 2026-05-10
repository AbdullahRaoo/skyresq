#!/usr/bin/env python3
"""
Export YOLO26-Nano to Pi-friendly formats.

Run on the dev PC (not the Pi). Produces two artifacts under ~/Drone:
  1. yolo26n.onnx         — INT8-quantised, dynamic-shape ONNX (small + fast)
  2. yolo26n_ncnn_model/  — NCNN .param + .bin pair (best ARM-NEON throughput)

Usage:
    cd ~/Drone
    python3 ros2_ws/src/drone_vision/tools/export_yolo_for_rpi.py \\
        --imgsz 320 --calibrate

Then on the Pi, point the detector at one of these:
    ros2 run drone_vision person_detector \\
        --ros-args -p backend:=ncnn -p model_path:=/home/pi/yolo26n_ncnn_model \\
                   -p imgsz:=320 -p target_inference_hz:=8.0

Notes
-----
- Ultralytics export pulls in torch + onnx + onnxruntime, which are already
  in the dev-PC venv. We do NOT install these on the Pi.
- INT8 calibration uses a small set of representative frames. Use
  --calibrate to capture them from the running SITL camera bridge first;
  alternatively pass --calibration-dir to use existing JPEGs.
"""
import argparse
import os
import shutil
import subprocess
import sys
from pathlib import Path

DEFAULT_PT_MODEL = Path.home() / 'Drone' / 'yolo26n.pt'
DEFAULT_OUT_ROOT = Path.home() / 'Drone'


def export_onnx(pt_path: Path, imgsz: int, int8: bool, dynamic: bool,
                calibration_dir: Path | None) -> Path:
    """Export PyTorch model to ONNX. Returns the .onnx path."""
    from ultralytics import YOLO  # heavy import — gated to dev PC
    model = YOLO(str(pt_path), task='detect')
    print(f"[export] ONNX export imgsz={imgsz} int8={int8} dynamic={dynamic}")
    kwargs = dict(format='onnx', imgsz=imgsz, half=False, simplify=True)
    if dynamic:
        kwargs['dynamic'] = True
    if int8:
        kwargs['int8'] = True
        if calibration_dir is not None:
            kwargs['data'] = str(calibration_dir)
    onnx_path = model.export(**kwargs)
    print(f"[export] wrote {onnx_path}")
    return Path(onnx_path)


def export_ncnn(pt_path: Path, imgsz: int) -> Path:
    """Export to NCNN model directory. Returns the directory."""
    from ultralytics import YOLO
    model = YOLO(str(pt_path), task='detect')
    print(f"[export] NCNN export imgsz={imgsz}")
    out = model.export(format='ncnn', imgsz=imgsz, half=False)
    out_path = Path(out)
    print(f"[export] wrote {out_path}")
    return out_path


def capture_calibration_frames(out_dir: Path, n: int):
    """Save ~n JPEGs from /drone/camera_raw using ros2 topic echo. Requires
    the SITL pipeline to be running. Skipped if --no-calibrate."""
    out_dir.mkdir(parents=True, exist_ok=True)
    print(f"[calib] capturing {n} frames from /drone/camera_raw → {out_dir}")
    print("[calib] If this hangs, SITL isn't running. Ctrl-C to skip.")
    # Minimal capture — just save sequential PNGs by piping through cv_bridge.
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import Image
    from cv_bridge import CvBridge
    import cv2

    class _Capture(Node):
        def __init__(self):
            super().__init__('calib_capture')
            self.bridge = CvBridge()
            self.count = 0
            self.create_subscription(Image, '/drone/camera_raw',
                                     self._on_frame, 10)

        def _on_frame(self, msg):
            if self.count >= n:
                return
            try:
                bgr = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
                p = out_dir / f"frame_{self.count:04d}.jpg"
                cv2.imwrite(str(p), bgr)
                self.count += 1
            except Exception as e:
                print(f"capture error: {e}")

    rclpy.init()
    node = _Capture()
    try:
        while rclpy.ok() and node.count < n:
            rclpy.spin_once(node, timeout_sec=0.5)
    finally:
        node.destroy_node()
        rclpy.shutdown()
    print(f"[calib] captured {node.count} frames")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--pt', type=Path, default=DEFAULT_PT_MODEL,
                        help='Source PyTorch model (.pt)')
    parser.add_argument('--out', type=Path, default=DEFAULT_OUT_ROOT,
                        help='Where to drop the exported artifacts')
    parser.add_argument('--imgsz', type=int, default=320)
    parser.add_argument('--no-onnx', action='store_true')
    parser.add_argument('--no-ncnn', action='store_true')
    parser.add_argument('--no-int8', action='store_true',
                        help='Skip INT8 quantisation (FP32 ONNX, larger but no calib needed)')
    parser.add_argument('--no-dynamic', action='store_true',
                        help='Bake imgsz into the ONNX shape (default: dynamic)')
    parser.add_argument('--calibrate', action='store_true',
                        help='Capture ~200 frames from /drone/camera_raw before INT8')
    parser.add_argument('--calibration-dir', type=Path,
                        help='Directory of pre-captured calibration JPEGs')
    parser.add_argument('--n-calib', type=int, default=200)
    args = parser.parse_args()

    if not args.pt.exists():
        print(f"missing {args.pt}; cannot export")
        sys.exit(1)

    args.out.mkdir(parents=True, exist_ok=True)

    calib_dir = args.calibration_dir
    if args.calibrate and calib_dir is None:
        calib_dir = args.out / 'calib_frames'
        capture_calibration_frames(calib_dir, args.n_calib)

    if not args.no_onnx:
        onnx_out = export_onnx(args.pt, args.imgsz,
                               int8=not args.no_int8,
                               dynamic=not args.no_dynamic,
                               calibration_dir=calib_dir)
        # Move beside the source pt
        target = args.out / args.pt.with_suffix('.onnx').name
        if onnx_out != target:
            shutil.move(str(onnx_out), str(target))
            print(f"[export] moved → {target}")

    if not args.no_ncnn:
        ncnn_out = export_ncnn(args.pt, args.imgsz)
        target = args.out / (args.pt.stem + '_ncnn_model')
        if ncnn_out != target:
            if target.exists():
                shutil.rmtree(target)
            shutil.move(str(ncnn_out), str(target))
            print(f"[export] moved → {target}")

    print("[export] done")


if __name__ == '__main__':
    main()
