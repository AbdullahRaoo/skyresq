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
    VehicleAttitude,
)
from geometry_msgs.msg import PointStamped, Vector3Stamped

from drone_msgs.msg import TargetWorld
from drone_msgs.srv import DropPayload

import math
import time
import statistics
from collections import deque


class MissionNode(Node):
    """Offboard mission controller with static & dynamic flight modes."""

    # ── Finite-state labels ──────────────────────────────────────────────
    # Survives unchanged from PR-1b: INIT, PRE_ARM, TAKEOFF, SQUARE, SEARCH,
    # LAND, DONE. New in PR-2b: GEO_LOCK, APPROACH, TERMINAL, DELIVER, RTL.
    # Removed in PR-2b: TRACK and DESCEND — folded into APPROACH/TERMINAL.
    STATE_INIT       = 'INIT'        # waiting for PX4 link
    STATE_PRE_ARM    = 'PRE_ARM'     # sending heartbeats before mode switch
    STATE_TAKEOFF    = 'TAKEOFF'     # climbing to cruise altitude
    STATE_SQUARE     = 'SQUARE'      # flying square waypoints (legacy test)
    STATE_SEARCH     = 'SEARCH'      # loitering / scanning for target
    STATE_GEO_LOCK   = 'GEO_LOCK'    # hovering, collecting stable geo-fix
    STATE_APPROACH   = 'APPROACH'    # GPS-coarse cruise to locked target
    STATE_TERMINAL   = 'TERMINAL'    # vision-authoritative final approach + descent
    STATE_DELIVER    = 'DELIVER'     # hover + servo drop
    STATE_RTL        = 'RTL'         # PX4 NAV_RETURN_TO_LAUNCH
    STATE_LAND       = 'LAND'        # PX4 auto-land issued (failsafe)
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
        # ── PR-2b: hybrid GPS+vision flow parameters ────────────────────
        self.declare_parameter('geo_lock_samples', 8)         # /target/world samples to average for fix
        self.declare_parameter('geo_lock_max_stddev_m', 4.0)  # reject fix if NED stddev exceeds this
        self.declare_parameter('geo_lock_timeout_s', 8.0)     # give up on geo-lock after this
        self.declare_parameter('approach_speed', 3.0)         # m/s during APPROACH cruise
        self.declare_parameter('approach_to_terminal_dist_m', 8.0)
        self.declare_parameter('terminal_align_pitch_deg', 75.0)
        self.declare_parameter('terminal_target_lost_s', 3.0)
        self.declare_parameter('deliver_settle_secs', 1.0)
        self.declare_parameter('rtl_disarm_timeout_s', 60.0)

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
        # PR-2b parameters
        self.geo_lock_samples       = int(self.get_parameter('geo_lock_samples').value)
        self.geo_lock_max_stddev_m  = float(self.get_parameter('geo_lock_max_stddev_m').value)
        self.geo_lock_timeout_s     = float(self.get_parameter('geo_lock_timeout_s').value)
        self.approach_speed         = float(self.get_parameter('approach_speed').value)
        self.approach_to_terminal_d = float(self.get_parameter('approach_to_terminal_dist_m').value)
        self.terminal_align_pitch   = float(self.get_parameter('terminal_align_pitch_deg').value)
        self.terminal_target_lost_s = float(self.get_parameter('terminal_target_lost_s').value)
        self.deliver_settle_secs    = float(self.get_parameter('deliver_settle_secs').value)
        self.rtl_disarm_timeout_s   = float(self.get_parameter('rtl_disarm_timeout_s').value)

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
        # PR-1b: gimbal-aware tracking
        self.create_subscription(
            Vector3Stamped, '/gimbal/state',
            self._gimbal_state_cb, 10)
        self.create_subscription(
            VehicleAttitude, '/fmu/out/vehicle_attitude',
            self._vehicle_attitude_cb, px4_qos)
        # PR-2b: hybrid GPS+vision flow
        self.create_subscription(
            TargetWorld, '/target/world',
            self._target_world_cb, 10)
        self.drop_client = self.create_client(DropPayload, '/payload/drop')

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

        # Gimbal & drone-attitude state (PR-1b)
        self.gimbal_pitch_deg = 0.0    # body-frame pitch; -90 = looking straight down
        self.gimbal_yaw_deg   = 0.0    # body-frame yaw; 0 = looking forward
        self.gimbal_last_seen = 0.0
        self.drone_yaw_rad    = 0.0    # NED yaw from VehicleAttitude

        # ── PR-2b: hybrid flow state ─────────────────────────────────────
        self.target_world_samples = deque(maxlen=self.geo_lock_samples)
        self.target_world_last_t  = 0.0
        self.locked_target_ned    = None    # (x, y) once GEO_LOCK passes
        self.geo_lock_started_t   = 0.0
        self.deliver_started_t    = 0.0
        self.drop_in_flight       = False    # async service call pending
        self.drop_result          = None     # last DropPayload.Response or None
        self.rtl_started_t        = 0.0

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

    def _gimbal_state_cb(self, msg: Vector3Stamped):
        """Latest gimbal angles in body frame, degrees. y=pitch, z=yaw."""
        self.gimbal_pitch_deg = msg.vector.y
        self.gimbal_yaw_deg   = msg.vector.z
        self.gimbal_last_seen = time.monotonic()

    def _vehicle_attitude_cb(self, msg: VehicleAttitude):
        """Drone attitude — we only need yaw to convert body-frame offsets to NED."""
        # PX4 publishes [w, x, y, z] in q[]. Extract yaw (rotation around NED z).
        w, x, y, z = msg.q[0], msg.q[1], msg.q[2], msg.q[3]
        self.drone_yaw_rad = math.atan2(2.0 * (w * z + x * y),
                                        1.0 - 2.0 * (y * y + z * z))

    def _target_world_cb(self, msg: TargetWorld):
        """Latest geo-localised target. Stored as a rolling window for variance check."""
        self.target_world_last_t = time.monotonic()
        self.target_world_samples.append(
            (msg.position_ned.point.x, msg.position_ned.point.y))

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

    # ── PR-1b: gimbal-aware tracking ─────────────────────────────────────
    #
    # Geometry: gimbal angles in body frame point at the target.
    #   pitch=0   → looking forward (along drone's nose)
    #   pitch=-90 → looking straight down (drone is over target)
    #   yaw>0     → target to drone's right
    #
    # If the drone is at altitude H above the ground and the gimbal points
    # at body-frame (pitch_g, yaw_g), the target's horizontal offset in body
    # frame is:
    #   dx_body = H * cos(yaw_g) / tan(-pitch_g)
    #   dy_body = H * sin(yaw_g) / tan(-pitch_g)
    # (only valid for pitch_g < 0; we clamp pitch to a safe negative range
    #  because near-horizon angles produce huge or singular projections.)
    # We rotate (dx_body, dy_body) by the drone's NED yaw to get the world
    # offset, then nudge the drone toward it.

    _MIN_DOWN_PITCH_DEG = -10.0   # only project when gimbal looks at least 10° below horizon
    _NEAR_NADIR_DEG     = 75.0    # |pitch| > 75° = drone is essentially over target → ready to descend

    def _gimbal_ground_offset_body(self):
        """Body-frame (forward, right) offset (m) from drone to where the
        gimbal hits the ground. Returns None if the gimbal isn't looking
        sufficiently downward or no position fix is available."""
        if self.pos is None:
            return None
        # Need a downward-pointing gimbal — clamp to a safe maximum so we
        # never blow up near horizon.
        if self.gimbal_pitch_deg > self._MIN_DOWN_PITCH_DEG:
            return None
        altitude_m = -self.pos.z   # NED z is negative when above home
        if altitude_m <= 0.1:
            return None
        pitch_rad = math.radians(self.gimbal_pitch_deg)
        yaw_rad   = math.radians(self.gimbal_yaw_deg)
        # tan(-pitch_rad) > 0 because pitch_rad < 0
        horiz_distance = altitude_m / math.tan(-pitch_rad)
        dx_body = horiz_distance * math.cos(yaw_rad)
        dy_body = horiz_distance * math.sin(yaw_rad)
        return (dx_body, dy_body)

    def _body_to_ned(self, dx_body: float, dy_body: float):
        """Rotate a body-frame (forward, right) offset into NED (north, east)."""
        c, s = math.cos(self.drone_yaw_rad), math.sin(self.drone_yaw_rad)
        dn = c * dx_body - s * dy_body
        de = s * dx_body + c * dy_body
        return dn, de

    def _step_track(self):
        """Move drone toward the gimbal's projected ground point.

        The visual_servo + gimbal_sim already keep the person centred in
        frame; we just have to put the airframe over them. When the gimbal
        is near nadir we know we're approximately over the target.
        """
        if self.pos is None:
            return
        offset = self._gimbal_ground_offset_body()
        if offset is None:
            # Gimbal not pointing useable direction — hold position
            self._publish_setpoint(self.pos.x, self.pos.y, self.takeoff_height_ned)
            return

        dx_body, dy_body = offset
        dn, de = self._body_to_ned(dx_body, dy_body)

        # Cap step size so PX4's position controller can keep up.
        max_step = self.track_speed * 0.05    # 0.1 m at default 2.0 m/s @ 20 Hz
        norm = math.hypot(dn, de)
        if norm > max_step:
            dn = dn * (max_step / norm)
            de = de * (max_step / norm)

        goal_x = self.pos.x + dn
        goal_y = self.pos.y + de
        # Yaw the drone so the gimbal's view comes back toward zero yaw.
        # Adding gimbal_yaw to drone yaw makes the new heading face the target.
        target_yaw = self.drone_yaw_rad + math.radians(self.gimbal_yaw_deg)
        self._publish_setpoint(goal_x, goal_y, self.takeoff_height_ned, target_yaw)

    def _step_descend(self):
        """Descend toward the target with the gimbal still locked on it.

        We still nudge horizontally to keep the drone over the target, but
        the gimbal compensates for pitch errors so the person stays in
        frame all the way down. This is the PR-1b fix for the 'lost during
        descent' loop.
        """
        if self.pos is None:
            return

        # Horizontal correction from gimbal projection (smaller gain than TRACK
        # so descent doesn't oscillate).
        offset = self._gimbal_ground_offset_body()
        if offset is not None:
            dx_body, dy_body = offset
            dn, de = self._body_to_ned(dx_body, dy_body)
            max_step = (self.track_speed * 0.5) * 0.05
            norm = math.hypot(dn, de)
            if norm > max_step:
                dn = dn * (max_step / norm)
                de = de * (max_step / norm)
            goal_x = self.pos.x + dn
            goal_y = self.pos.y + de
        else:
            goal_x, goal_y = self.pos.x, self.pos.y

        # Vertical: NED z increases (less negative) as altitude drops.
        current_z = self.pos.z
        step_z = 0.02
        goal_z = min(self.descend_alt, current_z + step_z)

        target_yaw = self.drone_yaw_rad + math.radians(self.gimbal_yaw_deg)
        self._publish_setpoint(goal_x, goal_y, goal_z, target_yaw)

    # ══════════════════════════════════════════════════════════════════════
    #  PR-2b: hybrid GPS+vision flow steps
    # ══════════════════════════════════════════════════════════════════════
    def _step_geo_lock(self):
        """Hover at takeoff alt, gimbal continues to track (visual_servo runs
        independently). Watch the rolling /target/world buffer for stability.
        Returns one of: 'collecting' | 'locked' | 'fail'."""
        if self.pos is None:
            return 'collecting'
        # Hover in place
        self._publish_setpoint(self.pos.x, self.pos.y, self.takeoff_height_ned,
                               self.drone_yaw_rad)
        # Need a full window of fresh samples
        if len(self.target_world_samples) < self.geo_lock_samples:
            if (time.monotonic() - self.geo_lock_started_t) > self.geo_lock_timeout_s:
                return 'fail'
            return 'collecting'
        # Compute mean + stddev
        xs = [s[0] for s in self.target_world_samples]
        ys = [s[1] for s in self.target_world_samples]
        try:
            sx = statistics.stdev(xs)
            sy = statistics.stdev(ys)
        except statistics.StatisticsError:
            return 'collecting'
        if sx > self.geo_lock_max_stddev_m or sy > self.geo_lock_max_stddev_m:
            # Variance too high — keep collecting until timeout
            if (time.monotonic() - self.geo_lock_started_t) > self.geo_lock_timeout_s:
                return 'fail'
            return 'collecting'
        # Stable fix
        self.locked_target_ned = (statistics.mean(xs), statistics.mean(ys))
        return 'locked'

    def _step_approach(self):
        """Cruise to the locked target NED at search altitude. Returns horizontal
        distance to target (m) so the caller can decide TERMINAL transition."""
        if self.pos is None or self.locked_target_ned is None:
            return float('inf')
        tx, ty = self.locked_target_ned
        dn = tx - self.pos.x
        de = ty - self.pos.y
        # Cap setpoint step so PX4 can keep up
        max_step = self.approach_speed * 0.05    # m per 50 ms tick
        dist = math.hypot(dn, de)
        if dist > max_step:
            dn = dn * (max_step / dist)
            de = de * (max_step / dist)
        goal_x = self.pos.x + dn
        goal_y = self.pos.y + de
        # Yaw to face the target — keeps the gimbal's view aligned with the wide cam
        target_yaw = math.atan2(ty - self.pos.y, tx - self.pos.x)
        self._publish_setpoint(goal_x, goal_y, self.takeoff_height_ned, target_yaw)
        return dist

    def _step_terminal(self):
        """Vision-authoritative final approach + descent.

        Reuses the gimbal-projection logic from PR-1b. Descent rate scales
        with how nadir-aligned the gimbal is — when far from nadir we hold
        altitude and slide horizontally; aligned, we descend cleanly."""
        if self.pos is None:
            return

        offset = self._gimbal_ground_offset_body()
        if offset is not None:
            dx_body, dy_body = offset
            dn, de = self._body_to_ned(dx_body, dy_body)
            max_step = (self.track_speed * 0.5) * 0.05
            norm = math.hypot(dn, de)
            if norm > max_step:
                dn = dn * (max_step / norm)
                de = de * (max_step / norm)
            goal_x = self.pos.x + dn
            goal_y = self.pos.y + de
        else:
            goal_x, goal_y = self.pos.x, self.pos.y

        # Descend smoothly only when reasonably aligned with nadir.
        align_factor = max(0.0, min(1.0,
            (abs(self.gimbal_pitch_deg) - 30.0) / 60.0))   # 30°→0, 90°→1
        step_z = 0.02 * align_factor
        goal_z = min(self.descend_alt, self.pos.z + step_z)

        target_yaw = self.drone_yaw_rad + math.radians(self.gimbal_yaw_deg)
        self._publish_setpoint(goal_x, goal_y, goal_z, target_yaw)

    def _step_deliver(self):
        """Hover at delivery altitude. Once settled, call /payload/drop once."""
        if self.pos is None:
            return None
        # Hold position
        self._publish_setpoint(self.pos.x, self.pos.y, self.descend_alt,
                               self.drone_yaw_rad)

        # Wait for settle, then trigger drop (only once)
        if self.drop_in_flight or self.drop_result is not None:
            return self.drop_result
        if (time.monotonic() - self.deliver_started_t) < self.deliver_settle_secs:
            return None

        # Service must be available
        if not self.drop_client.service_is_ready():
            self.get_logger().warn("Payload service not yet available", throttle_duration_sec=2.0)
            return None

        req = DropPayload.Request()
        req.arm_drop = True
        req.delay_ms = 0
        future = self.drop_client.call_async(req)
        self.drop_in_flight = True
        future.add_done_callback(self._drop_done)
        self.get_logger().info("Calling /payload/drop …")
        return None

    def _drop_done(self, future):
        try:
            self.drop_result = future.result()
            ok = self.drop_result.ok
            self.get_logger().info(
                f"/payload/drop returned ok={ok} reason='{self.drop_result.reason}'")
        except Exception as e:
            self.get_logger().error(f"/payload/drop call failed: {e}")
            # Synthesise a failed response so the state machine can move on
            self.drop_result = DropPayload.Response()
            self.drop_result.ok = False
            self.drop_result.reason = f"service_call_error: {e}"
        finally:
            self.drop_in_flight = False

    def _step_rtl(self):
        """PX4 owns RTL once we issue the command. We just wait for disarm."""
        if not self._rtl_command_sent():
            self._send_command(VehicleCommand.VEHICLE_CMD_NAV_RETURN_TO_LAUNCH)
            self.rtl_started_t = time.monotonic()
            self._rtl_sent = True
            self.get_logger().info("RTL command sent — handing off to PX4")

    def _rtl_command_sent(self):
        return getattr(self, '_rtl_sent', False)

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
        # PR-2b: confirmed detection → GEO_LOCK (not TRACK directly).
        if self.state == self.STATE_SEARCH:
            if self._has_target() and self._target_confirmed():
                self.geo_lock_started_t = time.monotonic()
                self.target_world_samples.clear()
                self.state = self.STATE_GEO_LOCK
                self.get_logger().info("Target confirmed → GEO_LOCK")
                return
            self._step_search()
            return

        # ── STATE: GEO_LOCK (collect stable world-frame fix) ─────────
        if self.state == self.STATE_GEO_LOCK:
            outcome = self._step_geo_lock()
            if outcome == 'locked':
                tx, ty = self.locked_target_ned
                self.state = self.STATE_APPROACH
                self.get_logger().info(
                    f"Geo-lock acquired at NED ({tx:.1f}, {ty:.1f}) → APPROACH")
            elif outcome == 'fail':
                self.state = self.STATE_SEARCH
                self.get_logger().warn(
                    "Geo-lock variance too high or timed out → SEARCH")
            return

        # ── STATE: APPROACH (GPS-coarse cruise) ──────────────────────
        if self.state == self.STATE_APPROACH:
            if not self._has_target() and (time.monotonic() - self.target_world_last_t > 3.0):
                self.target_world_samples.clear()
                self.geo_lock_started_t = time.monotonic()
                self.state = self.STATE_GEO_LOCK
                self.get_logger().warn("Target lost during APPROACH → GEO_LOCK")
                return
            dist = self._step_approach()
            if dist < self.approach_to_terminal_d:
                self.state = self.STATE_TERMINAL
                self.get_logger().info(
                    f"Within {dist:.1f} m of locked target → TERMINAL")
            return

        # ── STATE: TERMINAL (vision-authoritative final approach) ────
        if self.state == self.STATE_TERMINAL:
            # If we've lost vision for too long, climb back to APPROACH.
            if (time.monotonic() - self.target_last_seen) > self.terminal_target_lost_s:
                self.state = self.STATE_APPROACH
                self.get_logger().warn("Vision lost in TERMINAL → APPROACH")
                return
            self._step_terminal()
            aligned = abs(self.gimbal_pitch_deg) > self.terminal_align_pitch
            if (aligned and self.pos is not None
                    and self.pos.z >= self.descend_alt - 0.3):
                self.deliver_started_t = time.monotonic()
                self.state = self.STATE_DELIVER
                self.get_logger().info(
                    f"Aligned ({self.gimbal_pitch_deg:.0f}°) and at delivery alt → DELIVER")
            return

        # ── STATE: DELIVER (hover + drop) ────────────────────────────
        if self.state == self.STATE_DELIVER:
            result = self._step_deliver()
            if result is not None:
                if result.ok:
                    self.state = self.STATE_RTL
                    self.get_logger().info("Drop OK → RTL")
                else:
                    self.state = self.STATE_LAND
                    self._land()
                    self.get_logger().warn(
                        f"Drop FAILED ({result.reason}) → LAND")
            return

        # ── STATE: RTL (hand off to PX4) ─────────────────────────────
        if self.state == self.STATE_RTL:
            self._step_rtl()
            if not self.armed:
                self.state = self.STATE_DONE
                self.get_logger().info("RTL complete → DONE")
            elif (time.monotonic() - self.rtl_started_t) > self.rtl_disarm_timeout_s:
                self.state = self.STATE_LAND
                self._land()
                self.get_logger().warn("RTL timeout → LAND")
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
