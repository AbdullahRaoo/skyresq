#!/usr/bin/env python3
"""
Simulated detection injector — SITL only.

ArduPilot SITL has no camera, so the vision half (person_detector →
geo_localiser → /target/world) can't run from pixels. This node stands
in for that half by publishing a *geo-locked survivor* on
`/target/world` exactly as geo_localiser would, so the **real**
sar_orchestrator runs its genuine DETECTION_HOLD → APPROACH → DROP →
DROP_HOLD → RTL logic and the **real** mavlink_bridge flies SITL to the
computed drop point. Nothing in the autonomy/command path is mocked —
only the camera-to-pixel-to-world stage it replaces.

The survivor is a FIXED ground point. When injection starts we latch
the drone's current GPS, offset it by (offset_north_m, offset_east_m),
and republish that same lat/lon at `rate_hz` until stopped. This is
what makes APPROACH actually fly somewhere instead of chasing the drone.

Trigger
-------
- Publish std_msgs/Bool true on `/sim/inject` to start, false to stop
  (deterministic — drive it from the test script after takeoff), OR
- set `auto_start_delay_s` > 0 to begin that many seconds after the
  first valid GPS fix (hands-off demo).

Absolute placement
-------------------
Set `survivor_lat` / `survivor_lon` (both non-zero) to pin the survivor
at an exact coordinate instead of an offset from the drone.
"""
import math
import time

import rclpy
from rclpy.node import Node

from std_msgs.msg import Bool
from sensor_msgs.msg import NavSatFix
from drone_msgs.msg import TargetWorld


EARTH_R_M = 6_378_137.0


def offset_latlon(lat, lon, north_m, east_m):
    """Flat-earth: shift (lat,lon) by north/east metres. Matches
    geo.frames.ned_to_geo so injected coords are consistent with the
    real geo pipeline."""
    dlat = math.degrees(north_m / EARTH_R_M)
    dlon = math.degrees(east_m / (EARTH_R_M * math.cos(math.radians(lat))))
    return lat + dlat, lon + dlon


class SimDetectionInjector(Node):
    def __init__(self):
        super().__init__("sim_detection_injector")

        self.declare_parameter("offset_north_m", 35.0)
        self.declare_parameter("offset_east_m", 0.0)
        self.declare_parameter("survivor_lat", 0.0)
        self.declare_parameter("survivor_lon", 0.0)
        self.declare_parameter("confidence", 0.92)
        self.declare_parameter("rate_hz", 8.0)
        self.declare_parameter("auto_start_delay_s", 0.0)
        self.declare_parameter("publish_topic", "/target/world")

        self._off_n = float(self.get_parameter("offset_north_m").value)
        self._off_e = float(self.get_parameter("offset_east_m").value)
        self._abs_lat = float(self.get_parameter("survivor_lat").value)
        self._abs_lon = float(self.get_parameter("survivor_lon").value)
        self._conf = float(self.get_parameter("confidence").value)
        rate = max(1.0, float(self.get_parameter("rate_hz").value))
        self._auto_delay = float(self.get_parameter("auto_start_delay_s").value)
        topic = self.get_parameter("publish_topic").value

        self._drone_lat = None
        self._drone_lon = None
        self._first_fix_t = None
        self._active = False
        self._surv = None   # latched (lat, lon) once active

        self.create_subscription(NavSatFix, "/vehicle/gps", self._on_gps, 10)
        self.create_subscription(Bool, "/sim/inject", self._on_inject, 10)
        self._pub = self.create_publisher(TargetWorld, topic, 10)
        self.create_timer(1.0 / rate, self._tick)

        self.get_logger().info(
            f"sim_detection_injector up | offset N={self._off_n} E={self._off_e} m "
            f"| conf={self._conf} | rate={rate:.0f}Hz | "
            f"trigger={'auto@%.0fs' % self._auto_delay if self._auto_delay > 0 else '/sim/inject'}"
        )

    def _on_gps(self, msg: NavSatFix):
        if msg.latitude != 0.0 or msg.longitude != 0.0:
            self._drone_lat = msg.latitude
            self._drone_lon = msg.longitude
            if self._first_fix_t is None:
                self._first_fix_t = time.monotonic()

    def _on_inject(self, msg: Bool):
        self._set_active(bool(msg.data), reason="/sim/inject")

    def _set_active(self, on: bool, reason: str):
        if on == self._active:
            return
        if on:
            if self._abs_lat != 0.0 or self._abs_lon != 0.0:
                self._surv = (self._abs_lat, self._abs_lon)
            elif self._drone_lat is not None:
                self._surv = offset_latlon(self._drone_lat, self._drone_lon,
                                           self._off_n, self._off_e)
            else:
                self.get_logger().warn("inject requested but no GPS fix yet")
                return
            self._active = True
            self.get_logger().info(
                f"INJECT ON ({reason}) — survivor=({self._surv[0]:.7f}, "
                f"{self._surv[1]:.7f})")
        else:
            self._active = False
            self.get_logger().info(f"INJECT OFF ({reason})")

    def _tick(self):
        if (not self._active and self._auto_delay > 0.0
                and self._first_fix_t is not None
                and (time.monotonic() - self._first_fix_t) >= self._auto_delay):
            self._set_active(True, reason=f"auto@{self._auto_delay:.0f}s")
        if not self._active or self._surv is None:
            return
        m = TargetWorld()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = "map"
        m.position_ned.header = m.header           # NED unused by orchestrator
        m.position_geo.header = m.header
        m.position_geo.latitude = float(self._surv[0])
        m.position_geo.longitude = float(self._surv[1])
        m.position_geo.altitude = 0.0
        m.position_geo.status.status = 0           # STATUS_FIX
        m.confidence = float(self._conf)
        m.source = TargetWorld.SOURCE_VISION
        self._pub.publish(m)


def main(args=None):
    rclpy.init(args=args)
    node = SimDetectionInjector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
