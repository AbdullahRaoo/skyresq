#!/usr/bin/env python3
"""
SITL gimbal stub.

Models a 3-axis gimbal that we cannot physically place in the Gazebo SDF
(would require a model rebuild). Instead we crop a sub-window of the
existing wide source camera (x500_mono_cam, 640x480, ~100 deg HFOV) and
publish it as the "stabilised" feed. The crop center is offset from the
source image center in proportion to the virtual gimbal's body-frame yaw
and pitch.

  ROI center px  =  source center px  +  gimbal_angle * pixels_per_degree

Subscribes:
  /drone/camera_raw                     wide source frame
  /gimbal/cmd/look_at_pixel             pixel-error command from visual_servo
  /gimbal/cmd/set_attitude              direct angle setpoint (deg)
  /gimbal/cmd/recenter                  Trigger (stow to neutral)

Publishes:
  /gimbal/state                         current Vector3(roll,pitch,yaw) deg
  /gimbal/health                        diagnostic_msgs/DiagnosticStatus
  /drone/camera_raw_stabilised          cropped ROI (sensor_msgs/Image)
"""
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from sensor_msgs.msg import Image
from geometry_msgs.msg import PointStamped, Vector3Stamped
from diagnostic_msgs.msg import DiagnosticStatus, KeyValue
from std_srvs.srv import Trigger

from cv_bridge import CvBridge


class GimbalSim(Node):
    HEALTH_OK = DiagnosticStatus.OK
    HEALTH_WARN = DiagnosticStatus.WARN
    HEALTH_ERR = DiagnosticStatus.ERROR

    def __init__(self):
        super().__init__('gimbal_sim')

        # Source-camera geometry (matches x500_mono_cam in SITL).
        self.declare_parameter('source_topic', '/drone/camera_raw')
        self.declare_parameter('stabilised_topic', '/drone/camera_raw_stabilised')
        self.declare_parameter('source_hfov_deg', 100.0)
        self.declare_parameter('crop_width', 320)
        self.declare_parameter('crop_height', 240)

        # Virtual-gimbal limits and gains.
        self.declare_parameter('max_rate_dps', 60.0)
        self.declare_parameter('pitch_min_deg', -90.0)
        self.declare_parameter('pitch_max_deg', 30.0)
        self.declare_parameter('yaw_min_deg', -180.0)
        self.declare_parameter('yaw_max_deg', 180.0)
        self.declare_parameter('servo_kp_yaw', 30.0)     # deg/s per unit normalised pixel error
        self.declare_parameter('servo_kp_pitch', 30.0)
        self.declare_parameter('servo_deadband', 0.03)

        self.declare_parameter('integration_hz', 50.0)
        self.declare_parameter('state_publish_hz', 20.0)
        self.declare_parameter('health_publish_hz', 1.0)

        self.source_topic = self.get_parameter('source_topic').value
        self.stab_topic   = self.get_parameter('stabilised_topic').value
        self.hfov_deg     = float(self.get_parameter('source_hfov_deg').value)
        self.crop_w       = int(self.get_parameter('crop_width').value)
        self.crop_h       = int(self.get_parameter('crop_height').value)

        self.max_rate     = float(self.get_parameter('max_rate_dps').value)
        self.pitch_min    = float(self.get_parameter('pitch_min_deg').value)
        self.pitch_max    = float(self.get_parameter('pitch_max_deg').value)
        self.yaw_min      = float(self.get_parameter('yaw_min_deg').value)
        self.yaw_max      = float(self.get_parameter('yaw_max_deg').value)
        self.kp_yaw       = float(self.get_parameter('servo_kp_yaw').value)
        self.kp_pitch     = float(self.get_parameter('servo_kp_pitch').value)
        self.deadband     = float(self.get_parameter('servo_deadband').value)
        self.integ_hz     = float(self.get_parameter('integration_hz').value)
        self.state_hz     = float(self.get_parameter('state_publish_hz').value)
        self.health_hz    = float(self.get_parameter('health_publish_hz').value)

        # Initial gimbal attitude — body-frame degrees. We start the sim
        # gimbal looking somewhat downward so that during SEARCH the
        # camera actually sees the ground in front of the drone.
        # Real hardware can override via the set_attitude topic.
        self.declare_parameter('init_pitch_deg', -30.0)
        self.declare_parameter('init_yaw_deg',     0.0)
        self.yaw_deg   = float(self.get_parameter('init_yaw_deg').value)
        self.pitch_deg = float(self.get_parameter('init_pitch_deg').value)
        self.cmd_yaw_rate = 0.0
        self.cmd_pitch_rate = 0.0
        self.last_pixel_err = (0.0, 0.0)
        # Watchdog: if no pixel command arrives within this window, zero the
        # rates so the gimbal HOLDS its current angle instead of drifting to
        # its mechanical limits.
        self.declare_parameter('cmd_watchdog_secs', 0.25)
        self.cmd_watchdog_ns = int(float(self.get_parameter('cmd_watchdog_secs').value) * 1e9)
        self.last_pixel_cmd_ns = 0
        self.saturated = False
        self.health_level = self.HEALTH_OK
        self.health_msg = "ok"

        self.bridge = CvBridge()

        camera_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # Subscriptions
        self.create_subscription(Image, self.source_topic, self._on_source_frame, camera_qos)
        self.create_subscription(PointStamped, '/gimbal/cmd/look_at_pixel', self._on_look_at_pixel, 10)
        self.create_subscription(Vector3Stamped, '/gimbal/cmd/set_attitude', self._on_set_attitude, 10)
        self.create_service(Trigger, '/gimbal/cmd/recenter', self._on_recenter)

        # Publications
        self.state_pub = self.create_publisher(Vector3Stamped, '/gimbal/state', 10)
        self.health_pub = self.create_publisher(DiagnosticStatus, '/gimbal/health', 1)
        self.stab_pub = self.create_publisher(Image, self.stab_topic, camera_qos)

        # Timers
        self.create_timer(1.0 / self.integ_hz, self._integrate)
        self.create_timer(1.0 / self.state_hz, self._publish_state)
        self.create_timer(1.0 / self.health_hz, self._publish_health)

        self.get_logger().info(
            f"GimbalSim up | source={self.source_topic} | crop={self.crop_w}x{self.crop_h} | "
            f"hfov={self.hfov_deg:.0f}deg | max_rate={self.max_rate:.0f}deg/s")

    # ── Command callbacks ──────────────────────────────────────────────

    def _on_look_at_pixel(self, msg: PointStamped):
        ex, ey = float(msg.point.x), float(msg.point.y)
        if abs(ex) < self.deadband:
            ex = 0.0
        if abs(ey) < self.deadband:
            ey = 0.0
        self.last_pixel_err = (ex, ey)
        self.last_pixel_cmd_ns = self.get_clock().now().nanoseconds
        # Pixel error → angular rate. Positive ex = target right of centre → yaw right (positive yaw).
        # Positive ey = target below centre → pitch down (negative pitch).
        self.cmd_yaw_rate   = _clamp(self.kp_yaw   * ex,  -self.max_rate, self.max_rate)
        self.cmd_pitch_rate = _clamp(self.kp_pitch * (-ey), -self.max_rate, self.max_rate)

    def _on_set_attitude(self, msg: Vector3Stamped):
        # Direct setpoint (degrees). Rate-limit by chunking; for the sim, snap to bounds.
        self.yaw_deg   = _clamp(msg.vector.z, self.yaw_min,   self.yaw_max)
        self.pitch_deg = _clamp(msg.vector.y, self.pitch_min, self.pitch_max)
        self.cmd_yaw_rate = 0.0
        self.cmd_pitch_rate = 0.0

    def _on_recenter(self, request, response):
        self.yaw_deg = 0.0
        self.pitch_deg = 0.0
        self.cmd_yaw_rate = 0.0
        self.cmd_pitch_rate = 0.0
        response.success = True
        response.message = "recentred"
        return response

    # ── Periodic loops ─────────────────────────────────────────────────

    def _integrate(self):
        dt = 1.0 / self.integ_hz
        # Watchdog: drop stale rate commands so the gimbal HOLDS instead of drifting to its limits.
        age = self.get_clock().now().nanoseconds - self.last_pixel_cmd_ns
        if self.last_pixel_cmd_ns == 0 or age > self.cmd_watchdog_ns:
            self.cmd_yaw_rate = 0.0
            self.cmd_pitch_rate = 0.0
        self.yaw_deg   = _clamp(self.yaw_deg   + self.cmd_yaw_rate   * dt, self.yaw_min,   self.yaw_max)
        self.pitch_deg = _clamp(self.pitch_deg + self.cmd_pitch_rate * dt, self.pitch_min, self.pitch_max)

    def _publish_state(self):
        msg = Vector3Stamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.vector.x = 0.0                  # roll — sim assumes perfect roll stab
        msg.vector.y = self.pitch_deg
        msg.vector.z = self.yaw_deg
        self.state_pub.publish(msg)

    def _publish_health(self):
        status = DiagnosticStatus()
        status.name = 'gimbal_sim'
        status.hardware_id = 'sim'
        status.level = self.health_level
        status.message = self.health_msg
        status.values = [
            KeyValue(key='yaw_deg',   value=f"{self.yaw_deg:.1f}"),
            KeyValue(key='pitch_deg', value=f"{self.pitch_deg:.1f}"),
            KeyValue(key='saturated', value=str(self.saturated)),
        ]
        self.health_pub.publish(status)

    # ── Source frame → cropped stabilised frame ────────────────────────

    def _on_source_frame(self, msg: Image):
        try:
            cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().warn(f"cv_bridge convert failed: {e}", throttle_duration_sec=5.0)
            return

        h, w = cv_img.shape[:2]
        # Pixels per degree assuming square pixels; we use the horizontal ratio
        # for both axes — close enough for the sim crop.
        px_per_deg = w / max(self.hfov_deg, 1e-3)

        # ROI center in source pixels. Note: pitch positive = nose up,
        # so a positive pitch shifts the ROI UP (smaller y).
        cx = int(round(w * 0.5 + self.yaw_deg   * px_per_deg))
        cy = int(round(h * 0.5 - self.pitch_deg * px_per_deg))

        x0 = cx - self.crop_w // 2
        y0 = cy - self.crop_h // 2
        x1 = x0 + self.crop_w
        y1 = y0 + self.crop_h

        # Clamp to source bounds and detect saturation
        sat = (x0 < 0) or (y0 < 0) or (x1 > w) or (y1 > h)
        x0c, y0c = max(0, x0), max(0, y0)
        x1c, y1c = min(w, x1), min(h, y1)

        if x1c <= x0c or y1c <= y0c:
            # Fully off-frame — keep last good frame, mark error
            self.saturated = True
            self.health_level = self.HEALTH_ERR
            self.health_msg = "gimbal beyond source FOV"
            return

        roi = cv_img[y0c:y1c, x0c:x1c]
        # If clamped, pad to crop size with black so downstream gets a fixed-size frame.
        if roi.shape[0] != self.crop_h or roi.shape[1] != self.crop_w:
            padded = np.zeros((self.crop_h, self.crop_w, 3), dtype=roi.dtype)
            ox = max(0, -x0)
            oy = max(0, -y0)
            padded[oy:oy + roi.shape[0], ox:ox + roi.shape[1]] = roi
            roi = padded

        self.saturated = sat
        if sat:
            self.health_level = self.HEALTH_WARN
            self.health_msg = "gimbal partially beyond source FOV"
        else:
            self.health_level = self.HEALTH_OK
            self.health_msg = "ok"

        out = self.bridge.cv2_to_imgmsg(roi, encoding='bgr8')
        out.header = msg.header
        self.stab_pub.publish(out)


def _clamp(x, lo, hi):
    return max(lo, min(hi, x))


def main(args=None):
    rclpy.init(args=args)
    node = GimbalSim()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
