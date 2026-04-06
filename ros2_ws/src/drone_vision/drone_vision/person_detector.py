#!/usr/bin/env python3
"""
Enterprise-grade person detection node using YOLO26-Nano (ONNX).
Optimized for edge deployment (Raspberry Pi) with minimal memory footprint.

Subscribes to: /drone/camera_raw (sensor_msgs/Image)
Publishes to:  /detections (vision_msgs/Detection2DArray)
               /target_position (geometry_msgs/PointStamped) — center of best detection
               /camera/image_debug (sensor_msgs/Image) — annotated frame
"""
import os

# ── Prevent Ultralytics from auto-updating or pip-installing anything ──
# This MUST be set before importing ultralytics to avoid PEP-668 conflicts
# and network hangs in headless / managed Python environments.
os.environ.setdefault('YOLO_AUTOINSTALL', 'false')
os.environ.setdefault('YOLO_VERBOSE', 'false')

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image
from geometry_msgs.msg import PointStamped
from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose, BoundingBox2D
from cv_bridge import CvBridge
import cv2
import numpy as np
from ultralytics import YOLO


class PersonDetector(Node):
    def __init__(self):
        super().__init__('person_detector')

        # === Parameters (tunable at launch) ===
        self.declare_parameter('model_path', '')  # Auto-resolve if empty
        self.declare_parameter('confidence_threshold', 0.45)
        self.declare_parameter('image_topic', '/drone/camera_raw')
        self.declare_parameter('target_class', 0)  # COCO class 0 = person
        self.declare_parameter('process_every_n', 2)  # Skip frames for performance

        model_path = self.get_parameter('model_path').value
        self.conf_thresh = self.get_parameter('confidence_threshold').value
        image_topic = self.get_parameter('image_topic').value
        self.target_class = self.get_parameter('target_class').value
        self.process_every_n = self.get_parameter('process_every_n').value

        # === Resolve Model Path ===
        if not model_path:
            # Look for ONNX first, then PT fallback
            project_root = os.path.expanduser('~/Drone')
            onnx_path = os.path.join(project_root, 'yolo26n.onnx')
            pt_path = os.path.join(project_root, 'yolo26n.pt')
            if os.path.exists(onnx_path):
                model_path = onnx_path
            elif os.path.exists(pt_path):
                model_path = pt_path
            else:
                model_path = 'yolo26n.pt'  # Auto-download

        self.get_logger().info(f"Loading model: {model_path}")
        # Explicitly set task='detect' to suppress the
        # "Unable to automatically guess model task" warning for ONNX files.
        self.model = YOLO(model_path, task='detect')
        self.bridge = CvBridge()
        self.frame_count = 0

        # === QoS: Best-effort for camera streams (drop frames, don't queue) ===
        camera_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # === Subscribers ===
        self.create_subscription(Image, image_topic, self.image_callback, camera_qos)

        # === Publishers ===
        self.detection_pub = self.create_publisher(Detection2DArray, '/detections', 10)
        self.target_pub = self.create_publisher(PointStamped, '/target_position', 10)
        self.debug_pub = self.create_publisher(Image, '/camera/image_debug', 5)

        self.get_logger().info(
            f"Person Detector Started (YOLO26 | conf>{self.conf_thresh} | "
            f"process_every={self.process_every_n} | topic={image_topic})")

    def image_callback(self, msg):
        """Process incoming camera frames with frame-skipping for performance."""
        self.frame_count += 1
        if self.frame_count % self.process_every_n != 0:
            return  # Skip this frame

        try:
            # Convert ROS Image to OpenCV (BGR)
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            # Run YOLO26 inference (person class only)
            results = self.model(
                cv_image,
                classes=[self.target_class],
                conf=self.conf_thresh,
                verbose=False,
                imgsz=640
            )

            result = results[0]

            # Build Detection2DArray message
            detections_msg = Detection2DArray()
            detections_msg.header = msg.header

            best_conf = 0.0
            best_cx, best_cy = 0.0, 0.0
            img_h, img_w = cv_image.shape[:2]

            for box in result.boxes:
                x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                conf = float(box.conf[0].cpu().numpy())
                cls = int(box.cls[0].cpu().numpy())

                w = x2 - x1
                h = y2 - y1
                cx = x1 + (w / 2)
                cy = y1 + (h / 2)

                # Build detection message
                detection = Detection2D()
                detection.header = msg.header
                detection.bbox = BoundingBox2D()
                detection.bbox.center.position.x = float(cx)
                detection.bbox.center.position.y = float(cy)
                detection.bbox.size_x = float(w)
                detection.bbox.size_y = float(h)

                hyp = ObjectHypothesisWithPose()
                hyp.hypothesis.class_id = self.model.names[cls]
                hyp.hypothesis.score = conf
                detection.results.append(hyp)
                detections_msg.detections.append(detection)

                # Track best (highest confidence) detection
                if conf > best_conf:
                    best_conf = conf
                    best_cx = cx
                    best_cy = cy

                # Draw on debug image
                cv2.rectangle(cv_image, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)
                label = f"person {conf:.2f}"
                cv2.putText(cv_image, label, (int(x1), int(y1) - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

            # Publish detections
            self.detection_pub.publish(detections_msg)

            # Publish best target position (normalized -1 to 1 from image center)
            if best_conf > 0:
                target_msg = PointStamped()
                target_msg.header = msg.header
                # Normalize: center of image = (0,0), edges = (-1, 1)
                target_msg.point.x = (best_cx - img_w / 2) / (img_w / 2)
                target_msg.point.y = (best_cy - img_h / 2) / (img_h / 2)
                target_msg.point.z = float(best_conf)  # Use Z for confidence
                self.target_pub.publish(target_msg)

            # Publish debug image
            num_det = len(detections_msg.detections)
            cv2.putText(cv_image, f"YOLO26 | {num_det} person(s)", (10, 25),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 200, 255), 2)
            self.debug_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, encoding='bgr8'))

            if num_det > 0:
                self.get_logger().info(
                    f"Detected {num_det} person(s) | best conf: {best_conf:.2f} | "
                    f"pos: ({best_cx:.0f}, {best_cy:.0f})",
                    throttle_duration_sec=1.0)

        except Exception as e:
            self.get_logger().error(f"Detection error: {e}", throttle_duration_sec=5.0)


def main(args=None):
    rclpy.init(args=args)
    node = PersonDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
