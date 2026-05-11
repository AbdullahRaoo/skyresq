#!/usr/bin/env python3
"""
Person detector — runs YOLO26-Nano against the (gimbaled) camera stream.

Subscribes to /drone/camera_raw_stabilised (or /drone/camera_raw on hardware
without a gimbal), publishes:
    /detections           vision_msgs/Detection2DArray
    /target_position      geometry_msgs/PointStamped (-1..1 normalised, .z=conf)
    /camera/image_debug   sensor_msgs/Image (annotated frames)

Backends (selected via 'backend' parameter):
  - 'ultralytics'  full Ultralytics framework. Dev PC default. Heavy but easy.
  - 'onnxruntime'  ONNX Runtime CPU EP. Lighter, no torch. Good Pi fallback.
  - 'ncnn'         NCNN (ARM-optimised). Best Pi performance. Requires
                   pre-exported model directory (see tools/export_yolo_ncnn.py).

Adaptive frame-skip: measures rolling inference latency and dynamically
adjusts process_every_n to maintain a target effective rate. Helps the Pi
keep up under load without manual tuning.
"""
import os
import time
from collections import deque

# Set BEFORE importing ultralytics to prevent network hangs on managed Python.
os.environ.setdefault('YOLO_AUTOINSTALL', 'false')
os.environ.setdefault('YOLO_VERBOSE', 'false')

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from sensor_msgs.msg import Image
from geometry_msgs.msg import PointStamped
from vision_msgs.msg import (
    Detection2DArray, Detection2D, ObjectHypothesisWithPose, BoundingBox2D,
)
from cv_bridge import CvBridge

import cv2
import numpy as np


# ──────────────────────────────────────────────────────────────────────
# Backend abstraction. Each backend implements `infer(bgr_image)` returning
# a list of (xyxy_tuple, conf_float, class_id_int) for the requested class.
# ──────────────────────────────────────────────────────────────────────


class _UltralyticsBackend:
    """Pulls in torch + the full Ultralytics framework. Dev-PC friendly."""

    name = 'ultralytics'

    def __init__(self, model_path, imgsz, target_class, conf_thresh, logger):
        from ultralytics import YOLO  # heavy import — gated to this backend
        self.model = YOLO(model_path, task='detect')
        self.imgsz = imgsz
        self.target_class = target_class
        self.conf_thresh = conf_thresh
        logger.info(f"[backend=ultralytics] loaded {model_path} (imgsz={imgsz})")

    def infer(self, bgr):
        results = self.model(
            bgr,
            classes=[self.target_class],
            conf=self.conf_thresh,
            verbose=False,
            imgsz=self.imgsz,
        )
        out = []
        for box in results[0].boxes:
            xyxy = box.xyxy[0].cpu().numpy()
            out.append((tuple(xyxy.tolist()),
                        float(box.conf[0].cpu().numpy()),
                        int(box.cls[0].cpu().numpy()),
                        results[0].names[int(box.cls[0])]))
        return out


class _OnnxRuntimeBackend:
    """Direct ONNX Runtime — no torch dependency. Good Pi fallback."""

    name = 'onnxruntime'

    def __init__(self, model_path, imgsz, target_class, conf_thresh, logger):
        try:
            import onnxruntime as ort
        except ImportError as e:
            raise RuntimeError(
                "backend='onnxruntime' requires the onnxruntime package. "
                "On the Pi: `pip install onnxruntime`") from e

        # Cap thread count so we leave a core for ROS executors.
        opts = ort.SessionOptions()
        opts.intra_op_num_threads = 3
        opts.inter_op_num_threads = 1
        self.session = ort.InferenceSession(
            model_path, sess_options=opts, providers=['CPUExecutionProvider'])
        self.input_name = self.session.get_inputs()[0].name
        self.imgsz = imgsz
        self.target_class = target_class
        self.conf_thresh = conf_thresh
        # COCO class names — used for the detection message.
        self.class_names = _COCO_CLASSES
        logger.info(
            f"[backend=onnxruntime] loaded {model_path} (imgsz={imgsz}, "
            f"intra_op_threads=3)")

    def infer(self, bgr):
        # Letterbox-resize to imgsz × imgsz, normalise, convert to NCHW float32.
        img, ratio, (dw, dh) = _letterbox(bgr, self.imgsz)
        x = img[:, :, ::-1].transpose(2, 0, 1)              # BGR→RGB, HWC→CHW
        x = np.ascontiguousarray(x, dtype=np.float32) / 255.0
        x = x[None]                                          # add batch dim
        outputs = self.session.run(None, {self.input_name: x})
        # YOLO ONNX output is (1, 84, N) for COCO80 or (1, 5, N) for single-class
        # Each column = [cx, cy, w, h, *class_scores]
        preds = outputs[0]
        if preds.ndim == 3 and preds.shape[1] < preds.shape[2]:
            preds = preds[0].T              # (N, 84)
        else:
            preds = preds[0]                # already (N, 84) or (N, 5)
        return _decode_yolo(preds, ratio, (dw, dh),
                            self.conf_thresh, self.target_class,
                            self.class_names)


class _NcnnBackend:
    """NCNN (ARM-optimised) backend. Loads a pre-exported model directory.

    Use tools/export_yolo_ncnn.py on the dev PC to produce the NCNN files,
    then point `model_path` at the directory containing yolo26n.param + .bin.
    """

    name = 'ncnn'

    def __init__(self, model_path, imgsz, target_class, conf_thresh, logger):
        try:
            import ncnn  # type: ignore
        except ImportError as e:
            raise RuntimeError(
                "backend='ncnn' requires the ncnn Python binding. "
                "Install via `pip install ncnn` (Pi: needs cmake + ninja first).") from e
        # Resolve the .param/.bin pair inside the model directory.
        if os.path.isdir(model_path):
            params = [f for f in os.listdir(model_path) if f.endswith('.param')]
            if not params:
                raise RuntimeError(f"No .param file in {model_path}")
            param_path = os.path.join(model_path, params[0])
            bin_path = param_path.replace('.param', '.bin')
        else:
            param_path = model_path
            bin_path = model_path.replace('.param', '.bin')
        if not os.path.exists(bin_path):
            raise RuntimeError(f"Missing NCNN bin file: {bin_path}")

        self.net = ncnn.Net()
        self.net.opt.use_vulkan_compute = False             # CPU-only on Pi
        # Pin to 2 threads on the Pi 4. Standalone benchmark shows 3 threads
        # is the local optimum (9.4 Hz vs 8.3 Hz), BUT in the full pipeline
        # there are 5 other Python processes (mavlink_bridge, rtsp_camera,
        # visual_servo, geo_localiser, gcs_link) all contending for the
        # 4 cores. Leaving 2 cores for those processes drops in-pipeline
        # inference from ~1500 ms to ~400 ms — a net 4x win for the system
        # even though single-frame inference is slightly slower in isolation.
        self.net.opt.num_threads = 2
        self.net.load_param(param_path)
        self.net.load_model(bin_path)
        self.imgsz = imgsz
        self.target_class = target_class
        self.conf_thresh = conf_thresh
        self.class_names = _COCO_CLASSES
        # Names of the input/output tensors are model-dependent; ultralytics
        # default exports use 'in0' / 'out0'. Override here if your export
        # uses different names.
        self.input_name = 'in0'
        self.output_name = 'out0'
        logger.info(
            f"[backend=ncnn] loaded {param_path} (imgsz={imgsz}, "
            "CPU-only, target ARM-NEON)")

    def infer(self, bgr):
        import ncnn  # type: ignore
        img, ratio, (dw, dh) = _letterbox(bgr, self.imgsz)
        # NCNN takes uint8 BGR with normalisation applied internally
        mat_in = ncnn.Mat.from_pixels(
            img, ncnn.Mat.PixelType.PIXEL_BGR, self.imgsz, self.imgsz)
        mat_in.substract_mean_normalize([0.0, 0.0, 0.0],
                                        [1.0 / 255.0, 1.0 / 255.0, 1.0 / 255.0])
        ex = self.net.create_extractor()
        ex.input(self.input_name, mat_in)
        _, mat_out = ex.extract(self.output_name)
        preds = np.array(mat_out)
        if preds.ndim == 3 and preds.shape[1] < preds.shape[2]:
            preds = preds[0].T
        elif preds.ndim == 2 and preds.shape[0] < preds.shape[1]:
            preds = preds.T
        return _decode_yolo(preds, ratio, (dw, dh),
                            self.conf_thresh, self.target_class,
                            self.class_names)


# ──────────────────────────────────────────────────────────────────────
# Helpers shared by ONNX and NCNN backends
# ──────────────────────────────────────────────────────────────────────


_COCO_CLASSES = [
    'person', 'bicycle', 'car', 'motorcycle', 'airplane', 'bus', 'train',
    'truck', 'boat', 'traffic light', 'fire hydrant', 'stop sign',
    'parking meter', 'bench', 'bird', 'cat', 'dog', 'horse', 'sheep', 'cow',
    'elephant', 'bear', 'zebra', 'giraffe', 'backpack', 'umbrella',
    'handbag', 'tie', 'suitcase', 'frisbee', 'skis', 'snowboard',
    'sports ball', 'kite', 'baseball bat', 'baseball glove', 'skateboard',
    'surfboard', 'tennis racket', 'bottle', 'wine glass', 'cup', 'fork',
    'knife', 'spoon', 'bowl', 'banana', 'apple', 'sandwich', 'orange',
    'broccoli', 'carrot', 'hot dog', 'pizza', 'donut', 'cake', 'chair',
    'couch', 'potted plant', 'bed', 'dining table', 'toilet', 'tv',
    'laptop', 'mouse', 'remote', 'keyboard', 'cell phone', 'microwave',
    'oven', 'toaster', 'sink', 'refrigerator', 'book', 'clock', 'vase',
    'scissors', 'teddy bear', 'hair drier', 'toothbrush',
]


def _letterbox(bgr, target):
    """Resize and pad to (target, target) preserving aspect ratio."""
    h0, w0 = bgr.shape[:2]
    r = min(target / h0, target / w0)
    new_w, new_h = int(round(w0 * r)), int(round(h0 * r))
    resized = cv2.resize(bgr, (new_w, new_h), interpolation=cv2.INTER_LINEAR)
    dw = (target - new_w) // 2
    dh = (target - new_h) // 2
    out = np.full((target, target, 3), 114, dtype=resized.dtype)
    out[dh:dh + new_h, dw:dw + new_w] = resized
    return out, r, (dw, dh)


def _decode_yolo(preds, ratio, padding, conf_thresh, target_class, class_names):
    """Convert raw YOLO output array to a list of (xyxy, conf, class, name)."""
    if preds.size == 0:
        return []
    # preds shape (N, 4 + num_classes) — first 4 are xywh in letterbox space.
    boxes = preds[:, :4]
    if preds.shape[1] == 5:
        scores = preds[:, 4]
        classes = np.zeros_like(scores, dtype=np.int32)
    else:
        cls_scores = preds[:, 4:]
        classes = np.argmax(cls_scores, axis=1)
        scores = cls_scores[np.arange(cls_scores.shape[0]), classes]
    # Filter by class + confidence
    mask = (classes == target_class) & (scores >= conf_thresh)
    boxes, scores, classes = boxes[mask], scores[mask], classes[mask]
    if boxes.shape[0] == 0:
        return []
    # xywh → xyxy in letterbox pixels
    cx, cy, w, h = boxes[:, 0], boxes[:, 1], boxes[:, 2], boxes[:, 3]
    x1 = (cx - w / 2 - padding[0]) / ratio
    y1 = (cy - h / 2 - padding[1]) / ratio
    x2 = (cx + w / 2 - padding[0]) / ratio
    y2 = (cy + h / 2 - padding[1]) / ratio
    out = []
    for i in range(boxes.shape[0]):
        out.append(((float(x1[i]), float(y1[i]), float(x2[i]), float(y2[i])),
                    float(scores[i]), int(classes[i]),
                    class_names[int(classes[i])]
                    if int(classes[i]) < len(class_names) else 'unknown'))
    return out


def _build_backend(name, model_path, imgsz, target_class, conf_thresh, logger):
    """Factory: try the requested backend, fall back to onnxruntime, then ultralytics."""
    chain = []
    if name == 'ncnn':
        chain = [_NcnnBackend, _OnnxRuntimeBackend, _UltralyticsBackend]
    elif name == 'onnxruntime':
        chain = [_OnnxRuntimeBackend, _UltralyticsBackend]
    else:
        chain = [_UltralyticsBackend]
    last_err = None
    for cls in chain:
        try:
            return cls(model_path, imgsz, target_class, conf_thresh, logger)
        except Exception as e:
            logger.warn(f"backend {cls.name} failed: {e}; trying next")
            last_err = e
    raise RuntimeError(f"All backends failed; last error: {last_err}")


# ──────────────────────────────────────────────────────────────────────
# ROS node
# ──────────────────────────────────────────────────────────────────────


class PersonDetector(Node):
    def __init__(self):
        super().__init__('person_detector')

        # === Parameters ============================================
        self.declare_parameter('model_path', '')
        self.declare_parameter('confidence_threshold', 0.45)
        self.declare_parameter('image_topic', '/drone/camera_raw')
        self.declare_parameter('target_class', 0)
        self.declare_parameter('process_every_n', 3)
        self.declare_parameter('imgsz', 640)
        self.declare_parameter('backend', 'ultralytics')
        # Adaptive frame-skip: target an effective inference rate. Set to 0
        # to disable (keep process_every_n fixed).
        self.declare_parameter('target_inference_hz', 0.0)
        self.declare_parameter('frame_skip_min', 2)
        self.declare_parameter('frame_skip_max', 6)
        # /camera/image_debug republishes the full-res frame with bboxes drawn
        # over it. Off by default — costs ~30 ms/frame on a Pi 4 at 1920x1080
        # AND another image-msg serialise + publish. The GCS uses /detections
        # (bboxes only) for its overlay, not this topic.
        self.declare_parameter('publish_debug', False)
        # Log a per-stage timing line every N seconds (0 = disabled)
        self.declare_parameter('timing_log_period_s', 0.0)

        model_path = self.get_parameter('model_path').value
        self.conf_thresh = self.get_parameter('confidence_threshold').value
        image_topic = self.get_parameter('image_topic').value
        self.target_class = self.get_parameter('target_class').value
        self.process_every_n = int(self.get_parameter('process_every_n').value)
        self.imgsz = int(self.get_parameter('imgsz').value)
        backend_name = self.get_parameter('backend').value
        self.target_hz = float(self.get_parameter('target_inference_hz').value)
        self.skip_min = int(self.get_parameter('frame_skip_min').value)
        self.skip_max = int(self.get_parameter('frame_skip_max').value)
        self._publish_debug = bool(self.get_parameter('publish_debug').value)
        self._timing_period = float(self.get_parameter('timing_log_period_s').value)
        self._last_timing_log = 0.0

        # === Resolve model path ====================================
        model_path = self._resolve_model_path(model_path, backend_name)
        self.get_logger().info(f"Loading model: {model_path} (backend={backend_name})")

        self.backend = _build_backend(
            backend_name, model_path, self.imgsz,
            self.target_class, self.conf_thresh, self.get_logger())

        self.bridge = CvBridge()
        self.frame_count = 0
        self.latency_window = deque(maxlen=20)   # rolling inference latency (s)

        camera_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.create_subscription(Image, image_topic, self._on_frame, camera_qos)
        self.det_pub = self.create_publisher(Detection2DArray, '/detections', 10)
        self.target_pub = self.create_publisher(PointStamped, '/target_position', 10)
        self.debug_pub = self.create_publisher(Image, '/camera/image_debug', 5)

        self.get_logger().info(
            f"Person Detector Started ({self.backend.name} | conf>{self.conf_thresh} | "
            f"process_every={self.process_every_n} | topic={image_topic})")

    # ── Helpers ────────────────────────────────────────────────────────

    def _resolve_model_path(self, model_path, backend_name):
        if model_path:
            return model_path
        project_root = os.path.expanduser('~/Drone')
        # Backend-specific defaults
        if backend_name == 'ncnn':
            ncnn_dir = os.path.join(project_root, 'yolo26n_ncnn_model')
            if os.path.isdir(ncnn_dir):
                return ncnn_dir
        # Common case: ONNX preferred, .pt fallback
        for candidate in ('yolo26n.onnx', 'yolo26n.pt'):
            p = os.path.join(project_root, candidate)
            if os.path.exists(p):
                return p
        return 'yolo26n.pt'  # let ultralytics auto-download as last resort

    def _maybe_adapt_frame_skip(self):
        """Adaptive frame-skip: keep effective inference rate near target_hz."""
        if self.target_hz <= 0 or len(self.latency_window) < 5:
            return
        avg_latency = sum(self.latency_window) / len(self.latency_window)
        if avg_latency <= 0:
            return
        # Effective rate = source_fps / process_every_n. We don't know
        # source_fps here, so target the period directly: pick n so that
        # n * avg_latency >= 1/target_hz (i.e. inference can keep up).
        period = 1.0 / self.target_hz
        new_n = max(self.skip_min, min(self.skip_max, int(period / avg_latency)))
        if new_n != self.process_every_n:
            self.get_logger().info(
                f"adaptive frame-skip: latency={avg_latency*1000:.0f}ms → "
                f"process_every_n {self.process_every_n} → {new_n}")
            self.process_every_n = new_n

    # ── Main callback ──────────────────────────────────────────────────

    def _on_frame(self, msg):
        self.frame_count += 1
        if self.frame_count % self.process_every_n != 0:
            return

        t_cb_start = time.perf_counter()

        try:
            bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().warn(f"cv_bridge convert failed: {e}", throttle_duration_sec=5.0)
            return
        t_decode = time.perf_counter()

        try:
            detections = self.backend.infer(bgr)
        except Exception as e:
            self.get_logger().error(f"Inference error: {e}", throttle_duration_sec=5.0)
            return
        t_infer = time.perf_counter()

        latency = t_infer - t_decode
        self.latency_window.append(latency)
        self._maybe_adapt_frame_skip()

        # Build /detections — small, always published
        det_msg = Detection2DArray()
        det_msg.header = msg.header
        best_conf = 0.0
        best_cx = 0.0
        best_cy = 0.0
        img_h, img_w = bgr.shape[:2]
        for (x1, y1, x2, y2), conf, cls, name in detections:
            w = x2 - x1
            h = y2 - y1
            cx = x1 + w / 2.0
            cy = y1 + h / 2.0

            d = Detection2D()
            d.header = msg.header
            d.bbox = BoundingBox2D()
            d.bbox.center.position.x = float(cx)
            d.bbox.center.position.y = float(cy)
            d.bbox.size_x = float(w)
            d.bbox.size_y = float(h)
            hyp = ObjectHypothesisWithPose()
            hyp.hypothesis.class_id = name
            hyp.hypothesis.score = conf
            d.results.append(hyp)
            det_msg.detections.append(d)

            if conf > best_conf:
                best_conf = conf
                best_cx = cx
                best_cy = cy

        self.det_pub.publish(det_msg)

        if best_conf > 0:
            tp = PointStamped()
            tp.header = msg.header
            tp.point.x = (best_cx - img_w / 2.0) / (img_w / 2.0)
            tp.point.y = (best_cy - img_h / 2.0) / (img_h / 2.0)
            tp.point.z = float(best_conf)
            self.target_pub.publish(tp)

        t_pubs = time.perf_counter()

        # Debug-image branch is the expensive one — full-res draw + encode +
        # publish. Keep it off in production; on dev box you'd enable it via
        # the publish_debug parameter to see what the detector sees.
        if self._publish_debug:
            for (x1, y1, x2, y2), conf, _cls, name in detections:
                cv2.rectangle(bgr, (int(x1), int(y1)),
                              (int(x2), int(y2)), (0, 255, 0), 2)
                cv2.putText(bgr, f"{name} {conf:.2f}", (int(x1), int(y1) - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
            cv2.putText(
                bgr,
                f"{self.backend.name} | {len(detections)} | "
                f"{latency*1000:.0f}ms | every={self.process_every_n}",
                (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 200, 255), 2,
            )
            self.debug_pub.publish(self.bridge.cv2_to_imgmsg(bgr, encoding='bgr8'))
        t_debug = time.perf_counter()

        if len(detections) > 0:
            self.get_logger().info(
                f"Detected {len(detections)} person(s) | best conf: {best_conf:.2f} | "
                f"pos: ({best_cx:.0f}, {best_cy:.0f}) | inf {latency*1000:.0f}ms",
                throttle_duration_sec=1.0)

        if self._timing_period > 0:
            now = time.monotonic()
            if now - self._last_timing_log >= self._timing_period:
                self._last_timing_log = now
                self.get_logger().info(
                    f"timing | decode={int(1000*(t_decode-t_cb_start))}ms "
                    f"infer={int(1000*(t_infer-t_decode))}ms "
                    f"pubs={int(1000*(t_pubs-t_infer))}ms "
                    f"debug={int(1000*(t_debug-t_pubs))}ms "
                    f"total={int(1000*(t_debug-t_cb_start))}ms"
                )


def main(args=None):
    rclpy.init(args=args)
    node = PersonDetector()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
