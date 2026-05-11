#!/usr/bin/env python3
"""
RTSP Camera Node.

Pulls H.264 video from the XF Z-1 Mini gimbal RTSP endpoint (or any network
camera) and republishes frames as sensor_msgs/Image on /drone/camera_raw.

A compressed JPEG mirror is published on /drone/camera_raw/compressed for
low-bandwidth GCS debug viewing.

Parameters
----------
rtsp_url            str    rtsp://192.168.144.108/stream1
image_topic         str    /drone/camera_raw
publish_compressed  bool   True   also publish /<image_topic>/compressed
compressed_every_n  int    3      send every Nth frame to compressed topic
jpeg_quality        int    75     compressed JPEG quality (0..100)
pipeline            str    ""     optional GStreamer pipeline; overrides rtsp_url
reconnect_s         float  2.0    delay between reconnect attempts on failure

GStreamer hardware-decode pipeline (recommended on Pi 4)
--------------------------------------------------------
  ros2 run drone_vision rtsp_camera --ros-args -p pipeline:='rtspsrc location=rtsp://192.168.144.108/stream1 latency=200 ! rtph264depay ! h264parse ! v4l2h264dec ! videoconvert ! video/x-raw,format=BGR ! appsink'
"""

import threading
import time

import cv2
import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import CompressedImage, Image


class RtspCameraNode(Node):

    def __init__(self):
        super().__init__("rtsp_camera")

        self.declare_parameter("rtsp_url",           "rtsp://192.168.144.108/stream1")
        self.declare_parameter("image_topic",        "/drone/camera_raw")
        self.declare_parameter("publish_compressed", True)
        self.declare_parameter("compressed_every_n", 3)
        self.declare_parameter("jpeg_quality",       75)
        self.declare_parameter("pipeline",           "")
        self.declare_parameter("reconnect_s",        2.0)

        self._url        = self.get_parameter("rtsp_url").value
        topic            = self.get_parameter("image_topic").value
        pub_comp         = bool(self.get_parameter("publish_compressed").value)
        self._comp_n     = max(1, int(self.get_parameter("compressed_every_n").value))
        self._jpeg_q     = int(self.get_parameter("jpeg_quality").value)
        self._pipeline   = self.get_parameter("pipeline").value
        self._reconn_s   = float(self.get_parameter("reconnect_s").value)

        self._bridge    = CvBridge()
        self._pub_image = self.create_publisher(Image, topic, 10)
        self._pub_comp  = (
            self.create_publisher(CompressedImage, f"{topic}/compressed", 10)
            if pub_comp else None
        )

        self._frames_published = 0
        self._stop   = threading.Event()
        self._thread = threading.Thread(target=self._capture_loop, daemon=True)
        self._thread.start()

        src = self._pipeline if self._pipeline else self._url
        self.get_logger().info(f"RTSP camera starting — {src}")

    def destroy_node(self):
        self._stop.set()
        super().destroy_node()

    # ── Capture loop ──────────────────────────────────────────────────

    def _open_capture(self):
        if self._pipeline:
            return cv2.VideoCapture(self._pipeline, cv2.CAP_GSTREAMER)
        return cv2.VideoCapture(self._url, cv2.CAP_FFMPEG)

    def _capture_loop(self):
        cap = None
        frame_idx = 0

        while not self._stop.is_set():
            if cap is None or not cap.isOpened():
                cap = self._open_capture()
                if not cap.isOpened():
                    self.get_logger().warning(
                        f"Stream unavailable — retrying in {self._reconn_s:.1f}s"
                    )
                    time.sleep(self._reconn_s)
                    continue
                self.get_logger().info("Stream connected")

            ok, frame = cap.read()
            if not ok or frame is None:
                self.get_logger().warning("Read failed — reconnecting")
                cap.release()
                cap = None
                continue

            stamp = self.get_clock().now().to_msg()

            try:
                img = self._bridge.cv2_to_imgmsg(frame, encoding="bgr8")
                img.header.stamp    = stamp
                img.header.frame_id = "camera"
                self._pub_image.publish(img)
            except Exception as exc:
                self.get_logger().debug(f"Image publish error: {exc}")

            if self._pub_comp is not None and frame_idx % self._comp_n == 0:
                try:
                    ok2, jpeg = cv2.imencode(
                        ".jpg", frame,
                        [int(cv2.IMWRITE_JPEG_QUALITY), self._jpeg_q],
                    )
                    if ok2:
                        cm = CompressedImage()
                        cm.header.stamp    = stamp
                        cm.header.frame_id = "camera"
                        cm.format = "jpeg"
                        cm.data   = jpeg.tobytes()
                        self._pub_comp.publish(cm)
                except Exception as exc:
                    self.get_logger().debug(f"JPEG encode error: {exc}")

            frame_idx += 1
            self._frames_published += 1
            if self._frames_published % 100 == 0:
                self.get_logger().info(
                    f"Published {self._frames_published} frames | "
                    f"{frame.shape[1]}x{frame.shape[0]}"
                )

        if cap is not None:
            cap.release()


def main(args=None):
    rclpy.init(args=args)
    node = RtspCameraNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
