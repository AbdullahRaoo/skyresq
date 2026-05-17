#!/usr/bin/env python3
"""
Geo-localiser.

Fuses the latest pixel detection with gimbal angles and drone pose to
produce a world-frame target estimate (NED + lat/lon).

Inputs (subscribed)
-------------------
/target_position                  PointStamped — normalised pixel error (-1..1)
                                  from person_detector. .z = confidence.
/gimbal/state                     Vector3Stamped — body-frame gimbal RPY (deg).
/fmu/out/vehicle_local_position   VehicleLocalPosition — drone NED position.
/fmu/out/vehicle_attitude         VehicleAttitude — drone quaternion → yaw.
/fmu/out/home_position            HomePosition — geodetic origin for flat-earth
                                  conversion. Optional; if absent we publish
                                  NED only (lat/lon left at zero).

Outputs (published)
-------------------
/target/world                     drone_msgs/TargetWorld at the detector rate
                                  (~detector Hz). Source = SOURCE_VISION.

Algorithm
---------
1. Map normalised pixel error → pixel coordinates in the cropped/stabilised
   view using camera intrinsics (config/camera_intrinsics_*.yaml).
2. Build the unit ray from the camera through that pixel, transform through
   gimbal → body → NED.
3. Intersect with the ground plane at z = ground_z (flat-earth assumption).
4. Reject if the ray points up/sideways or AGL is below safety threshold.
5. Convert ground hit point to lat/lon via flat-earth around home.
"""
import math
import time
from pathlib import Path

import yaml
import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy,
)

from geometry_msgs.msg import PointStamped, Vector3Stamped, PointStamped as _PtS
from sensor_msgs.msg import NavSatFix

from drone_msgs.msg import TargetWorld

from drone_vision.geo.frames import (
    pixel_to_world_ray,
    ground_intersect,
    ned_to_geo,
)


class GeoLocaliser(Node):
    def __init__(self):
        super().__init__('geo_localiser')

        # ── Parameters ─────────────────────────────────────────────────
        self.declare_parameter('intrinsics_file',
                               str(Path.home() / 'Drone/ros2_ws/src/drone_vision/'
                                   'config/camera_intrinsics_sim.yaml'))
        # Sim crop dimensions (these are what the detector actually sees,
        # not the wide-source size). Hardware override in the YAML config.
        self.declare_parameter('image_width',  320)
        self.declare_parameter('image_height', 240)
        self.declare_parameter('ground_z',       0.0)   # NED z of ground plane
        self.declare_parameter('min_below_horizon_deg', 5.0)
        self.declare_parameter('min_agl_m',      1.0)
        self.declare_parameter('confidence_min', 0.40)
        self.declare_parameter('publish_topic', '/target/world')

        intrinsics_path = self.get_parameter('intrinsics_file').value
        self.image_width  = int(self.get_parameter('image_width').value)
        self.image_height = int(self.get_parameter('image_height').value)
        self.ground_z     = float(self.get_parameter('ground_z').value)
        self.min_below_horizon_rad = math.radians(
            float(self.get_parameter('min_below_horizon_deg').value))
        self.min_agl_m    = float(self.get_parameter('min_agl_m').value)
        self.conf_min     = float(self.get_parameter('confidence_min').value)
        publish_topic     = self.get_parameter('publish_topic').value

        self._load_intrinsics(intrinsics_path)

        # ── Latest sensor state ────────────────────────────────────────
        self.gimbal_yaw_deg   = 0.0
        self.gimbal_pitch_deg = 0.0
        self.gimbal_age       = float('inf')
        self.drone_yaw_rad    = 0.0
        self.drone_roll_rad   = 0.0
        self.drone_pitch_rad  = 0.0
        self.drone_yaw_age    = float('inf')
        self.drone_pos_ned    = None   # (x, y, z) or None
        self.drone_pos_age    = float('inf')
        self.home_lat_deg     = None
        self.home_lon_deg     = None
        self.home_alt_m       = 0.0

        # ── Subscriptions ──────────────────────────────────────────────
        # /vehicle/* topics are published by mavlink_bridge — geo_localiser
        # is FC-agnostic.
        # /vehicle/home uses TRANSIENT_LOCAL so we pick up the latched
        # value even if we started after mavlink_bridge sent it.
        home_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.create_subscription(PointStamped, '/target_position',
                                 self._on_target, 10)
        self.create_subscription(Vector3Stamped, '/gimbal/state',
                                 self._on_gimbal, 10)
        self.create_subscription(PointStamped,
                                 '/vehicle/pose_ned',
                                 self._on_local_pos, 10)
        self.create_subscription(Vector3Stamped,
                                 '/vehicle/attitude',
                                 self._on_attitude, 10)
        self.create_subscription(NavSatFix,
                                 '/vehicle/home',
                                 self._on_home, home_qos)

        # ── Publisher ──────────────────────────────────────────────────
        self.world_pub = self.create_publisher(TargetWorld, publish_topic, 10)

        self._last_publish_t = 0.0
        self.get_logger().info(
            f"GeoLocaliser up | intrinsics={intrinsics_path} | "
            f"image={self.image_width}x{self.image_height} | "
            f"fx={self.fx:.1f} fy={self.fy:.1f}")

    # ── Init helpers ───────────────────────────────────────────────────

    def _load_intrinsics(self, path: str):
        """Load fx/fy/cx/cy from a calibration YAML or compute from HFOV."""
        with open(path, 'r') as f:
            cfg = yaml.safe_load(f) or {}
        cam = cfg.get('camera', cfg)
        # Allow either explicit fx/fy or hfov_rad.
        if 'fx' in cam and 'fy' in cam:
            self.fx = float(cam['fx'])
            self.fy = float(cam['fy'])
        else:
            hfov_rad = float(cam.get('hfov_rad', 1.74))
            src_w    = float(cam.get('width',  self.image_width))
            self.fx  = (src_w / 2.0) / math.tan(hfov_rad / 2.0)
            self.fy  = self.fx   # square pixels
        # Principal point — image centre by default.
        self.cx = float(cam.get('cx', self.image_width  / 2.0))
        self.cy = float(cam.get('cy', self.image_height / 2.0))

    # ── Sensor callbacks ───────────────────────────────────────────────

    def _on_gimbal(self, msg: Vector3Stamped):
        self.gimbal_pitch_deg = msg.vector.y
        self.gimbal_yaw_deg   = msg.vector.z
        self.gimbal_age       = time.monotonic()

    def _on_local_pos(self, msg: PointStamped):
        # x=north, y=east, z=NED z (negative above ground)
        self.drone_pos_ned = (msg.point.x, msg.point.y, msg.point.z)
        self.drone_pos_age = time.monotonic()

    def _on_attitude(self, msg: Vector3Stamped):
        # mavlink_bridge publishes ATTITUDE as x=roll, y=pitch, z=yaw (rad).
        self.drone_roll_rad  = msg.vector.x
        self.drone_pitch_rad = msg.vector.y
        self.drone_yaw_rad   = msg.vector.z
        self.drone_yaw_age = time.monotonic()

    def _on_home(self, msg: NavSatFix):
        self.home_lat_deg = msg.latitude
        self.home_lon_deg = msg.longitude
        self.home_alt_m   = msg.altitude

    # ── Main: detection callback drives publication ────────────────────

    def _on_target(self, msg: PointStamped):
        conf = float(msg.point.z)
        if conf < self.conf_min:
            return
        if self.drone_pos_ned is None:
            return
        # Stale state guard — anything older than 1 s, skip.
        now = time.monotonic()
        if (now - self.drone_pos_age > 1.0
                or now - self.drone_yaw_age > 1.0
                or now - self.gimbal_age   > 1.0):
            return

        ex = float(msg.point.x)   # normalised pixel error (-1..1)
        ey = float(msg.point.y)
        # Convert to pixel coordinates in the camera image.
        u = self.cx + ex * (self.image_width  / 2.0)
        v = self.cy + ey * (self.image_height / 2.0)

        ray = pixel_to_world_ray(
            u, v, self.fx, self.fy, self.cx, self.cy,
            self.gimbal_yaw_deg, self.gimbal_pitch_deg,
            self.drone_yaw_rad,
            self.drone_roll_rad, self.drone_pitch_rad,
        )
        # Reject rays that don't head down enough — geometry blows up near
        # horizon and a 1° error becomes huge ground error.
        if ray[2] < math.sin(self.min_below_horizon_rad):
            return

        # AGL = -drone_z (NED z negative when above home/ground)
        agl = -self.drone_pos_ned[2]
        if agl < self.min_agl_m:
            return

        hit = ground_intersect(self.drone_pos_ned, ray, ground_z=self.ground_z)
        if hit is None:
            return

        # Build TargetWorld
        out = TargetWorld()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = 'map'
        out.confidence = conf
        out.source = TargetWorld.SOURCE_VISION

        ned_pt = _PtS()
        ned_pt.header = out.header
        ned_pt.point.x = float(hit[0])    # north
        ned_pt.point.y = float(hit[1])    # east
        ned_pt.point.z = float(hit[2])    # down (= ground_z)
        out.position_ned = ned_pt

        geo = NavSatFix()
        geo.header = out.header
        if self.home_lat_deg is not None:
            lat, lon = ned_to_geo(self.home_lat_deg, self.home_lon_deg,
                                  hit[0], hit[1])
            geo.latitude  = lat
            geo.longitude = lon
            geo.altitude  = self.home_alt_m + (- self.ground_z)
            geo.status.status = 0    # STATUS_FIX
        else:
            geo.status.status = -1   # STATUS_NO_FIX
        out.position_geo = geo

        self.world_pub.publish(out)
        if (now - self._last_publish_t) > 1.0:
            self.get_logger().info(
                f"target NED ({hit[0]:.1f}, {hit[1]:.1f}) m | "
                f"AGL {agl:.1f} m | gimbal "
                f"yaw={self.gimbal_yaw_deg:.0f}° pitch={self.gimbal_pitch_deg:.0f}° | "
                f"conf={conf:.2f}")
            self._last_publish_t = now


def main(args=None):
    rclpy.init(args=args)
    node = GeoLocaliser()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
