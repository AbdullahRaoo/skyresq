#!/usr/bin/env python3
"""
cv_viewer.py – Lightweight dual-window camera viewer using OpenCV.
No Qt dependency at all. Works inside or outside the venv.

Usage:
  python3 cv_viewer.py [--raw-only | --debug-only]

Opens:
  Window 1: /drone/camera_raw   (raw Gazebo feed)
  Window 2: /camera/image_debug (YOLO annotated feed)

Press  q  or  ESC  in either window to quit.
"""
import argparse
import sys

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2


class CvViewer(Node):
    def __init__(self, show_raw: bool, show_debug: bool):
        super().__init__('cv_viewer')
        self.bridge = CvBridge()

        cam_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        if show_raw:
            self.create_subscription(
                Image, '/drone/camera_raw',
                self._raw_cb, cam_qos)
            self.get_logger().info('Subscribing to /drone/camera_raw')

        if show_debug:
            self.create_subscription(
                Image, '/camera/image_debug',
                self._debug_cb, cam_qos)
            self.get_logger().info('Subscribing to /camera/image_debug')

        # Poll OpenCV windows at ~30 Hz
        self.create_timer(1.0 / 30.0, self._poll_keys)

    def _raw_cb(self, msg: Image):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            cv2.imshow('Raw Camera (/drone/camera_raw)', frame)
        except Exception as e:
            self.get_logger().error(f'raw frame error: {e}', throttle_duration_sec=5.0)

    def _debug_cb(self, msg: Image):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            cv2.imshow('YOLO Annotated (/camera/image_debug)', frame)
        except Exception as e:
            self.get_logger().error(f'debug frame error: {e}', throttle_duration_sec=5.0)

    def _poll_keys(self):
        key = cv2.waitKey(1) & 0xFF
        if key in (ord('q'), 27):  # q or ESC
            self.get_logger().info('Quit requested — shutting down.')
            cv2.destroyAllWindows()
            rclpy.shutdown()


def main():
    parser = argparse.ArgumentParser(description='OpenCV camera feed viewer')
    group = parser.add_mutually_exclusive_group()
    group.add_argument('--raw-only',   action='store_true', help='Show only raw feed')
    group.add_argument('--debug-only', action='store_true', help='Show only YOLO annotated feed')
    args = parser.parse_args()

    show_raw   = not args.debug_only
    show_debug = not args.raw_only

    rclpy.init()
    node = CvViewer(show_raw, show_debug)
    try:
        rclpy.spin(node)
    except ExternalShutdownException:
        pass
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
