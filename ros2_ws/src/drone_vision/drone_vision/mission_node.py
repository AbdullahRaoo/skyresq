#!/usr/bin/env python3
"""
Mission Node – Autonomous Search & Rescue
==========================================
Modes of operation (selected via 'mode' parameter):
  * 'square'  – legacy test: arm → takeoff → fly a square → land.
  * 'search'  – arm → takeoff → loiter/search → follow detected target → land
                when target is reached.

The node subscribes to:
  /fmu/out/vehicle_status          – arming / nav-state feedback
  /fmu/out/vehicle_local_position  – current NED position
  /target_position                 – normalised target from person_detector

And publishes:
  /fmu/in/offboard_control_mode
  /fmu/in/trajectory_setpoint
  /fmu/in/vehicle_command
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from px4_msgs.msg import (
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleCommand,
    VehicleStatus,
    VehicleLocalPosition,
)
from geometry_msgs.msg import PointStamped

import math
import time


class MissionNode(Node):
    """Offboard mission controller with static & dynamic flight modes."""

    # ── Finite-state labels ──────────────────────────────────────────────
    STATE_INIT       = 'INIT'        # waiting for PX4 link
    STATE_PRE_ARM    = 'PRE_ARM'     # sending heartbeats before mode switch
    STATE_TAKEOFF    = 'TAKEOFF'     # climbing to cruise altitude
    STATE_SQUARE     = 'SQUARE'      # flying square waypoints
    STATE_SEARCH     = 'SEARCH'      # loitering / scanning for target
    STATE_TRACK      = 'TRACK'       # actively following a detected person
    STATE_DESCEND    = 'DESCEND'     # descending toward confirmed target
    STATE_LAND       = 'LAND'        # PX4 auto-land issued
    STATE_DONE       = 'DONE'        # mission finished

    def __init__(self):
        super().__init__('mission_node')

        # ── Parameters ───────────────────────────────────────────────────
        self.declare_parameter('mode', 'search')              # 'square' | 'search'
        self.declare_parameter('takeoff_height', 5.0)         # metres (positive = up)
        self.declare_parameter('square_size', 10.0)           # metres per side
        self.declare_parameter('waypoint_threshold', 1.5)     # metres to consider WP reached
        self.declare_parameter('tracking_speed', 2.0)         # m/s max horizontal speed toward target
        self.declare_parameter('descend_altitude', 2.0)       # altitude to descend to before landing
        self.declare_parameter('target_lost_timeout', 5.0)    # seconds before reverting to search
        self.declare_parameter('search_radius', 15.0)         # metres – radius of search orbit
        self.declare_parameter('search_speed', 1.5)           # m/s angular orbit speed
        self.declare_parameter('target_confirm_secs', 3.0)    # seconds of continuous detection to confirm

        self.mode               = self.get_parameter('mode').value
        self.takeoff_height_ned = -abs(self.get_parameter('takeoff_height').value)  # NED
        self.square_size        = self.get_parameter('square_size').value
        self.wp_thresh          = self.get_parameter('waypoint_threshold').value
        self.track_speed        = self.get_parameter('tracking_speed').value
        self.descend_alt        = -abs(self.get_parameter('descend_altitude').value)  # NED
        self.target_lost_timeout = self.get_parameter('target_lost_timeout').value
        self.search_radius      = self.get_parameter('search_radius').value
        self.search_speed       = self.get_parameter('search_speed').value
        self.target_confirm_sec = self.get_parameter('target_confirm_secs').value

        # ── QoS ──────────────────────────────────────────────────────────
        px4_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # ── Publishers ───────────────────────────────────────────────────
        self.offboard_mode_pub = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', px4_qos)
        self.setpoint_pub = self.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', px4_qos)
        self.cmd_pub = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', px4_qos)

        # ── Subscribers ──────────────────────────────────────────────────
        self.create_subscription(
            VehicleStatus, '/fmu/out/vehicle_status',
            self._vehicle_status_cb, px4_qos)
        self.create_subscription(
            VehicleLocalPosition, '/fmu/out/vehicle_local_position',
            self._local_pos_cb, px4_qos)
        self.create_subscription(
            PointStamped, '/target_position',
            self._target_cb, 10)

        # ── Internal state ───────────────────────────────────────────────
        self.state            = self.STATE_INIT
        self.px4_connected    = False
        self.armed            = False
        self.offboard_active  = False
        self.pos              = None   # VehicleLocalPosition msg
        self.offboard_counter = 0

        # Target tracking
        self.target_norm_x    = 0.0    # normalised –1..1
        self.target_norm_y    = 0.0
        self.target_conf      = 0.0
        self.target_last_seen = 0.0    # monotonic time
        self.target_first_seen = 0.0   # time of continuous detection start

        # Square-mission waypoints (filled in _build_square_wps)
        self.waypoints          = []
        self.current_wp_idx     = 0

        # Search orbit angle
        self.search_angle = 0.0

        # ── Main 20 Hz control loop ──────────────────────────────────────
        self.timer = self.create_timer(0.05, self._control_loop)
        self.get_logger().info(
            f"Mission Node started  |  mode={self.mode}  |  "
            f"takeoff={abs(self.takeoff_height_ned):.1f} m")

    # ══════════════════════════════════════════════════════════════════════
    #  Callbacks
    # ══════════════════════════════════════════════════════════════════════
    def _vehicle_status_cb(self, msg: VehicleStatus):
        if not self.px4_connected:
            self.get_logger().info("PX4 Connected!")
            self.px4_connected = True
        self.armed = (msg.arming_state == VehicleStatus.ARMING_STATE_ARMED)
        self.offboard_active = (msg.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD)

    def _local_pos_cb(self, msg: VehicleLocalPosition):
        self.pos = msg

    def _target_cb(self, msg: PointStamped):
        """Receive normalised target from person_detector (x,y in –1..1, z=conf)."""
        self.target_norm_x = msg.point.x
        self.target_norm_y = msg.point.y
        self.target_conf   = msg.point.z
        now = time.monotonic()
        # If we haven't seen a target recently, reset "first seen" clock
        if now - self.target_last_seen > 1.0:
            self.target_first_seen = now
        self.target_last_seen = now

    # ══════════════════════════════════════════════════════════════════════
    #  Helpers – command publishing
    # ══════════════════════════════════════════════════════════════════════
    def _publish_heartbeat(self):
        msg = OffboardControlMode()
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_mode_pub.publish(msg)

    def _publish_setpoint(self, x: float, y: float, z: float, yaw: float = 0.0):
        msg = TrajectorySetpoint()
        msg.position = [x, y, z]
        msg.yaw = yaw
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.setpoint_pub.publish(msg)

    def _send_command(self, command, param1=0.0, param2=0.0):
        msg = VehicleCommand()
        msg.param1 = param1
        msg.param2 = param2
        msg.command = command
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.cmd_pub.publish(msg)

    def _arm(self):
        self._send_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)
        self.get_logger().info("ARM command sent")

    def _engage_offboard(self):
        self._send_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)
        self.get_logger().info("OFFBOARD mode command sent")

    def _land(self):
        self._send_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
        self.get_logger().info("LAND command sent")

    # ── geometry ─────────────────────────────────────────────────────────
    def _dist_to(self, x, y, z):
        if self.pos is None:
            return float('inf')
        dx = self.pos.x - x
        dy = self.pos.y - y
        dz = self.pos.z - z
        return math.sqrt(dx*dx + dy*dy + dz*dz)

    def _has_target(self) -> bool:
        """True if a person was detected within the last `target_lost_timeout` seconds."""
        return (time.monotonic() - self.target_last_seen) < self.target_lost_timeout

    def _target_confirmed(self) -> bool:
        """True if we have had continuous detections for `target_confirm_sec`."""
        if not self._has_target():
            return False
        return (time.monotonic() - self.target_first_seen) >= self.target_confirm_sec

    # ══════════════════════════════════════════════════════════════════════
    #  Square-mission logic
    # ══════════════════════════════════════════════════════════════════════
    def _build_square_wps(self):
        h = self.takeoff_height_ned
        s = self.square_size
        self.waypoints = [
            [0.0, 0.0, h],      # takeoff hover
            [s,   0.0, h],      # north
            [s,   s,   h],      # north-east
            [0.0, s,   h],      # east
            [0.0, 0.0, h],      # home
        ]
        self.current_wp_idx = 0

    def _step_square(self):
        """Advance through square waypoints; returns True when done."""
        if self.current_wp_idx >= len(self.waypoints):
            return True
        wp = self.waypoints[self.current_wp_idx]
        self._publish_setpoint(wp[0], wp[1], wp[2])
        if self._dist_to(wp[0], wp[1], wp[2]) < self.wp_thresh:
            self.get_logger().info(
                f"Reached WP {self.current_wp_idx}: "
                f"[{wp[0]:.0f}, {wp[1]:.0f}, {wp[2]:.0f}]")
            self.current_wp_idx += 1
        return False

    # ══════════════════════════════════════════════════════════════════════
    #  Search-mode logic
    # ══════════════════════════════════════════════════════════════════════
    def _step_search(self):
        """Fly a slow orbit to scan the area while no target is detected."""
        dt = 0.05  # 20 Hz timer
        # angular speed (rad/s) = linear_speed / radius
        omega = self.search_speed / self.search_radius
        self.search_angle += omega * dt

        x = self.search_radius * math.cos(self.search_angle)
        y = self.search_radius * math.sin(self.search_angle)
        # yaw toward center of orbit
        yaw = math.atan2(-y, -x)
        self._publish_setpoint(x, y, self.takeoff_height_ned, yaw)

    def _step_track(self):
        """Translate normalised camera error into a position offset toward target."""
        if self.pos is None:
            return

        # Map normalised target (–1..1) to positional nudges.
        # target_norm_x > 0 → target is to the RIGHT  → move East  (NED +Y)
        # target_norm_y > 0 → target is BELOW centre  → move North (NED +X)
        #   (camera Y axis points down, positive target_y = person below horizon
        #    → we move forward to keep them in frame.)
        gain = self.track_speed * 0.05  # metres per tick at max offset
        dx =  self.target_norm_y * gain   # forward  (NED X = North)
        dy =  self.target_norm_x * gain   # right    (NED Y = East)

        goal_x = self.pos.x + dx
        goal_y = self.pos.y + dy
        yaw = math.atan2(dy, dx) if (abs(dx) + abs(dy)) > 0.01 else 0.0
        self._publish_setpoint(goal_x, goal_y, self.takeoff_height_ned, yaw)

    def _step_descend(self):
        """Lower altitude while keeping the target centred."""
        if self.pos is None:
            return
        gain = self.track_speed * 0.05
        dx = self.target_norm_y * gain
        dy = self.target_norm_x * gain
        goal_x = self.pos.x + dx
        goal_y = self.pos.y + dy
        # Descend smoothly
        current_z = self.pos.z
        step_z = 0.02  # metres per tick descend rate
        goal_z = max(self.descend_alt, current_z - step_z)
        self._publish_setpoint(goal_x, goal_y, goal_z)

    # ══════════════════════════════════════════════════════════════════════
    #  Main state machine (20 Hz)
    # ══════════════════════════════════════════════════════════════════════
    def _control_loop(self):
        # ── Always send heartbeat to keep offboard alive ──
        if self.state not in (self.STATE_INIT, self.STATE_DONE):
            self._publish_heartbeat()

        # ── STATE: INIT – wait for PX4 ───────────────────────────────
        if self.state == self.STATE_INIT:
            if self.px4_connected:
                self.state = self.STATE_PRE_ARM
                self.get_logger().info("PX4 link up → PRE_ARM")
            return

        # ── STATE: PRE_ARM – send heartbeats before mode switch ──────
        if self.state == self.STATE_PRE_ARM:
            self._publish_setpoint(0.0, 0.0, self.takeoff_height_ned)
            self.offboard_counter += 1
            if self.offboard_counter < 20:
                return
            if not self.offboard_active:
                self._engage_offboard()
                return
            if not self.armed:
                self._arm()
                return
            self.state = self.STATE_TAKEOFF
            self.get_logger().info("Armed in OFFBOARD → TAKEOFF")
            return

        # ── STATE: TAKEOFF ───────────────────────────────────────────
        if self.state == self.STATE_TAKEOFF:
            self._publish_setpoint(0.0, 0.0, self.takeoff_height_ned)
            if self.pos is not None and self.pos.z < self.takeoff_height_ned + 0.5:
                self.get_logger().info(
                    f"Takeoff complete at {self.pos.z:.1f} m NED  →  "
                    f"{'SQUARE' if self.mode == 'square' else 'SEARCH'}")
                if self.mode == 'square':
                    self._build_square_wps()
                    self.current_wp_idx = 1  # skip hovering WP0
                    self.state = self.STATE_SQUARE
                else:
                    self.state = self.STATE_SEARCH
            return

        # ── STATE: SQUARE (legacy test) ──────────────────────────────
        if self.state == self.STATE_SQUARE:
            done = self._step_square()
            if done:
                self.state = self.STATE_LAND
                self._land()
                self.get_logger().info("Square mission complete → LAND")
            return

        # ── STATE: SEARCH (orbit scan) ───────────────────────────────
        if self.state == self.STATE_SEARCH:
            if self._has_target():
                self.state = self.STATE_TRACK
                self.get_logger().info("Target detected → TRACK")
                return
            self._step_search()
            return

        # ── STATE: TRACK (follow detected person) ────────────────────
        if self.state == self.STATE_TRACK:
            if not self._has_target():
                self.state = self.STATE_SEARCH
                self.get_logger().info("Target lost → SEARCH")
                return
            if self._target_confirmed():
                self.state = self.STATE_DESCEND
                self.get_logger().info("Target confirmed → DESCEND")
                return
            self._step_track()
            return

        # ── STATE: DESCEND (lower to delivery altitude) ──────────────
        if self.state == self.STATE_DESCEND:
            if not self._has_target():
                # Lost during descent – abort and climb back
                self.state = self.STATE_SEARCH
                self.get_logger().warn("Target lost during descent → SEARCH")
                return
            self._step_descend()
            if self.pos is not None and self.pos.z >= self.descend_alt - 0.3:
                self.state = self.STATE_LAND
                self._land()
                self.get_logger().info("Descent complete → LAND")
            return

        # ── STATE: LAND / DONE ───────────────────────────────────────
        if self.state == self.STATE_LAND:
            if not self.armed:
                self.state = self.STATE_DONE
                self.get_logger().info("Disarmed → Mission DONE")
            return

        # DONE – nothing to do
        return


def main(args=None):
    rclpy.init(args=args)
    node = MissionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
