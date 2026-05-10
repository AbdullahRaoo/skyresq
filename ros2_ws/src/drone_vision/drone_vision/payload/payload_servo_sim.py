#!/usr/bin/env python3
"""
Payload servo — SITL stub.

Same ROS surface as the real (pigpio) implementation: a /payload/drop
service guarded by safety interlocks, and a /payload/state topic.

Interlocks (refuse drop unless ALL satisfied):
  - AGL within [min_drop_alt_m, max_drop_alt_m]
  - vehicle armed in OFFBOARD
  - recent target sighting (within target_freshness_secs)
  - battery remaining above require_battery_above

If interlocks pass, this stub:
  1. Publishes /payload/state = "dropped"
  2. Logs the drop event with the current target estimate
  3. Returns ok=true
"""
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from std_msgs.msg import String
from px4_msgs.msg import VehicleLocalPosition, VehicleStatus, BatteryStatus

from drone_msgs.msg import TargetWorld
from drone_msgs.srv import DropPayload


class PayloadServoSim(Node):
    STATE_ARMED   = 'armed'      # boot state
    STATE_READY   = 'ready'      # interlocks pass, awaiting drop
    STATE_DROPPED = 'dropped'    # drop completed
    STATE_FAULT   = 'fault'      # hardware fault (real path only)

    def __init__(self):
        super().__init__('payload_servo')

        # ── Parameters (mirror real backend) ──────────────────────────
        self.declare_parameter('backend', 'sim')
        self.declare_parameter('min_drop_alt_m', 1.5)
        self.declare_parameter('max_drop_alt_m', 10.0)
        self.declare_parameter('target_freshness_secs', 1.5)
        self.declare_parameter('require_armed_offboard', True)
        self.declare_parameter('require_battery_above', 0.20)
        self.declare_parameter('open_hold_secs', 3.0)
        self.declare_parameter('state_publish_hz', 1.0)

        self.min_alt = float(self.get_parameter('min_drop_alt_m').value)
        self.max_alt = float(self.get_parameter('max_drop_alt_m').value)
        self.target_freshness = float(self.get_parameter('target_freshness_secs').value)
        self.require_armed = bool(self.get_parameter('require_armed_offboard').value)
        self.require_batt = float(self.get_parameter('require_battery_above').value)
        self.open_hold = float(self.get_parameter('open_hold_secs').value)
        state_hz = float(self.get_parameter('state_publish_hz').value)

        # ── Sensor state (latest values) ──────────────────────────────
        self.agl_m = None              # metres above ground (= -pos.z)
        self.armed = False
        self.offboard = False
        self.battery_remaining = 1.0
        self.last_target_world_t = 0.0
        self.last_target_ned = None    # (x, y, z)

        # ── State + publisher ─────────────────────────────────────────
        self.state = self.STATE_ARMED

        latched_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.state_pub = self.create_publisher(String, '/payload/state', latched_qos)

        # PX4 outputs use BEST_EFFORT + TRANSIENT_LOCAL.
        px4_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.create_subscription(VehicleLocalPosition,
                                 '/fmu/out/vehicle_local_position',
                                 self._on_local_pos, px4_qos)
        self.create_subscription(VehicleStatus,
                                 '/fmu/out/vehicle_status',
                                 self._on_status, px4_qos)
        self.create_subscription(BatteryStatus,
                                 '/fmu/out/battery_status',
                                 self._on_battery, px4_qos)
        self.create_subscription(TargetWorld, '/target/world',
                                 self._on_target_world, 10)

        self.create_service(DropPayload, '/payload/drop', self._handle_drop)
        self.create_timer(1.0 / state_hz, self._publish_state)

        self._publish_state()
        self.get_logger().info(
            f"PayloadServoSim up | alt range [{self.min_alt}, {self.max_alt}] m | "
            f"battery_min {self.require_batt:.0%}")

    # ── Sensor callbacks ───────────────────────────────────────────────

    def _on_local_pos(self, msg: VehicleLocalPosition):
        # NED z is negative when above home → AGL = -z
        self.agl_m = -msg.z

    def _on_status(self, msg: VehicleStatus):
        self.armed = (msg.arming_state == VehicleStatus.ARMING_STATE_ARMED)
        self.offboard = (msg.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD)

    def _on_battery(self, msg: BatteryStatus):
        # PX4 BatteryStatus.remaining is 0..1 if available.
        if 0.0 <= msg.remaining <= 1.0:
            self.battery_remaining = float(msg.remaining)

    def _on_target_world(self, msg: TargetWorld):
        self.last_target_world_t = time.monotonic()
        self.last_target_ned = (msg.position_ned.point.x,
                                msg.position_ned.point.y,
                                msg.position_ned.point.z)

    # ── Service handler ────────────────────────────────────────────────

    def _handle_drop(self, request, response):
        ok, reason = self._check_interlocks(request.arm_drop)
        if not ok:
            response.ok = False
            response.reason = reason
            self.get_logger().warn(f"Drop refused: {reason}")
            return response

        # Honour optional pre-drop delay. We acknowledge the request
        # immediately and arm a timer to do the actual drop — this avoids
        # blocking the rclpy executor (which would freeze every other
        # callback while we wait).
        delay = max(0, int(request.delay_ms)) / 1000.0
        if delay > 0:
            self.get_logger().info(f"Dropping in {delay:.2f}s")
            self.create_timer(delay, self._do_drop_once)
        else:
            self._do_drop_once()

        response.ok = True
        response.reason = ""
        return response

    def _do_drop_once(self):
        # One-shot: cancel the timer that called us if any, perform the
        # log + state transition, then keep going.
        if self.state == self.STATE_DROPPED:
            return
        if self.last_target_ned is not None:
            tn = self.last_target_ned
            self.get_logger().info(
                f"PAYLOAD DROPPED at NED ({tn[0]:.1f}, {tn[1]:.1f}) | "
                f"AGL {self.agl_m:.1f} m | battery {self.battery_remaining:.0%}")
        else:
            self.get_logger().info(
                f"PAYLOAD DROPPED | AGL {self.agl_m:.1f} m (no target lock recorded)")
        self._set_state(self.STATE_DROPPED)

    def _check_interlocks(self, arm_drop):
        if not arm_drop:
            return False, "request.arm_drop must be true"
        if self.agl_m is None:
            return False, "no altitude reading yet"
        if self.agl_m < self.min_alt:
            return False, f"AGL {self.agl_m:.1f} m below min_drop_alt {self.min_alt}"
        if self.agl_m > self.max_alt:
            return False, f"AGL {self.agl_m:.1f} m above max_drop_alt {self.max_alt}"
        if self.require_armed and not (self.armed and self.offboard):
            return False, f"vehicle not armed in OFFBOARD (armed={self.armed}, offboard={self.offboard})"
        if self.battery_remaining < self.require_batt:
            return False, f"battery {self.battery_remaining:.0%} below {self.require_batt:.0%}"
        age = time.monotonic() - self.last_target_world_t
        if self.last_target_world_t == 0.0 or age > self.target_freshness:
            return False, f"no fresh /target/world (age={age:.1f}s)"
        if self.state == self.STATE_DROPPED:
            return False, "already dropped this mission"
        return True, ""

    # ── State publication ─────────────────────────────────────────────

    def _set_state(self, state: str):
        self.state = state
        self._publish_state()

    def _publish_state(self):
        msg = String()
        msg.data = self.state
        self.state_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = PayloadServoSim()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
