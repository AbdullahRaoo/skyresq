#!/usr/bin/env python3
"""
Mission Node - Square Path Flight
Waits for PX4 connection, arms, takes off, flies square, lands.
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleStatus, VehicleLocalPosition

import math

class MissionNode(Node):
    def __init__(self):
        super().__init__('mission_node')

        # QoS profile for PX4 compatibility
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Publishers
        self.offboard_control_mode_pub = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', qos_profile)
        self.trajectory_setpoint_pub = self.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos_profile)
        self.vehicle_command_pub = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', qos_profile)

        # Subscribers
        self.vehicle_status_sub = self.create_subscription(
            VehicleStatus, '/fmu/out/vehicle_status', self.vehicle_status_callback, qos_profile)
        self.vehicle_local_position_sub = self.create_subscription(
            VehicleLocalPosition, '/fmu/out/vehicle_local_position', self.vehicle_local_position_callback, qos_profile)

        # State
        self.vehicle_status = None
        self.vehicle_local_position = None
        self.px4_connected = False
        self.armed = False
        self.offboard_mode = False
        
        # Mission parameters
        self.takeoff_height = -5.0  # NED (negative = up)
        self.square_size = 10.0
        
        # Waypoints (NED: X=North, Y=East, Z=Down)
        self.waypoints = [
            [0.0, 0.0, self.takeoff_height],                    # Takeoff
            [self.square_size, 0.0, self.takeoff_height],       # North
            [self.square_size, self.square_size, self.takeoff_height],  # North-East
            [0.0, self.square_size, self.takeoff_height],       # East
            [0.0, 0.0, self.takeoff_height],                    # Home
        ]
        self.current_waypoint = 0
        self.waypoint_threshold = 1.5  # meters

        # Offboard counter (need to send commands before switching mode)
        self.offboard_counter = 0
        
        # Timer at 20Hz
        self.timer = self.create_timer(0.05, self.timer_callback)
        
        self.get_logger().info("Mission Node Started - Waiting for PX4 connection...")

    def vehicle_status_callback(self, msg):
        """Called when we receive vehicle status from PX4"""
        if not self.px4_connected:
            self.get_logger().info("PX4 Connected!")
            self.px4_connected = True
        self.vehicle_status = msg
        self.armed = (msg.arming_state == VehicleStatus.ARMING_STATE_ARMED)
        self.offboard_mode = (msg.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD)

    def vehicle_local_position_callback(self, msg):
        """Called when we receive local position from PX4"""
        self.vehicle_local_position = msg

    def arm(self):
        """Send arm command"""
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)
        self.get_logger().info("ARM command sent")

    def disarm(self):
        """Send disarm command"""
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=0.0)
        self.get_logger().info("DISARM command sent")

    def engage_offboard_mode(self):
        """Switch to offboard mode"""
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)
        self.get_logger().info("OFFBOARD mode command sent")

    def land(self):
        """Send land command"""
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
        self.get_logger().info("LAND command sent")

    def publish_vehicle_command(self, command, param1=0.0, param2=0.0):
        """Publish a vehicle command to PX4"""
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
        self.vehicle_command_pub.publish(msg)

    def publish_offboard_control_mode(self):
        """Heartbeat to keep offboard mode active"""
        msg = OffboardControlMode()
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_control_mode_pub.publish(msg)

    def publish_trajectory_setpoint(self, x, y, z, yaw=0.0):
        """Send position setpoint"""
        msg = TrajectorySetpoint()
        msg.position = [x, y, z]
        msg.yaw = yaw
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.trajectory_setpoint_pub.publish(msg)

    def distance_to_waypoint(self, waypoint):
        """Calculate distance from current position to waypoint"""
        if self.vehicle_local_position is None:
            return float('inf')
        dx = self.vehicle_local_position.x - waypoint[0]
        dy = self.vehicle_local_position.y - waypoint[1]
        dz = self.vehicle_local_position.z - waypoint[2]
        return math.sqrt(dx*dx + dy*dy + dz*dz)

    def timer_callback(self):
        """Main control loop - runs at 20Hz"""
        
        # === PHASE 0: Wait for PX4 connection ===
        if not self.px4_connected:
            return  # Do nothing until connected
        
        # === Always publish offboard heartbeat ===
        self.publish_offboard_control_mode()
        
        # Get current waypoint
        if self.current_waypoint < len(self.waypoints):
            target = self.waypoints[self.current_waypoint]
            self.publish_trajectory_setpoint(target[0], target[1], target[2])
        
        # === PHASE 1: Pre-arm (send setpoints before switching mode) ===
        if self.offboard_counter < 20:  # ~1 second of setpoints
            self.offboard_counter += 1
            return
        
        # === PHASE 2: Arm and switch to offboard ===
        if not self.offboard_mode:
            self.engage_offboard_mode()
            return
            
        if not self.armed:
            self.arm()
            return
        
        # === PHASE 3: Flying the mission ===
        if self.current_waypoint < len(self.waypoints):
            target = self.waypoints[self.current_waypoint]
            dist = self.distance_to_waypoint(target)
            
            if dist < self.waypoint_threshold:
                self.get_logger().info(f"Reached waypoint {self.current_waypoint}: {target}")
                self.current_waypoint += 1
        else:
            # Mission complete
            self.land()
            self.get_logger().info("Mission Complete - Landing")


def main(args=None):
    rclpy.init(args=args)
    node = MissionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
