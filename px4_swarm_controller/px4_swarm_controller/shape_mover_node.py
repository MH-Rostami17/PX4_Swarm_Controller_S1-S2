#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import yaml
import numpy as np
from px4_msgs.msg import TrajectorySetpoint, OffboardControlMode, VehicleLocalPosition


class ShapeMoverNode(Node):
    def __init__(self):
        super().__init__('shape_mover_node')

        # params
        self.declare_parameter('namespace', '/px4_1')
        self.declare_parameter('input_wp_path', '')
        self.declare_parameter('dx', 0.0)
        self.declare_parameter('dy', 0.0)
        self.declare_parameter('dz', 0.0)
        self.declare_parameter('yaw', 0.0)
        self.declare_parameter('duration', 6.0)

        ns = self.get_parameter('namespace').value
        wp_path = self.get_parameter('input_wp_path').value
        dx = float(self.get_parameter('dx').value)
        dy = float(self.get_parameter('dy').value)
        dz = float(self.get_parameter('dz').value)
        yaw = float(self.get_parameter('yaw').value)
        self.duration = float(self.get_parameter('duration').value)

        # internal
        self.timer_period = 0.1
        self.t = 0.0
        self.current_pos = None
        self.current_yaw = 0.0
        self.movement_complete = False
        self.last_setpoint = None

        # optional: load waypoints only for info (safe if namespace missing)
        self.original_wp = []
        if wp_path:
            try:
                with open(wp_path, 'r') as f:
                    data = yaml.safe_load(f)
                if 'wp' in data and ns in data['wp']:
                    self.original_wp = data['wp'][ns]
                    self.get_logger().info(f"Loaded {len(self.original_wp)} waypoints for {ns} from {wp_path}")
                else:
                    self.get_logger().warn(f"No waypoints for namespace '{ns}' in {wp_path}.")
            except Exception as e:
                self.get_logger().warn(f"Failed to read waypoint file '{wp_path}': {e}")

        # rotation applied to the translation vector (not to the absolute position)
        self.R = np.array([
            [np.cos(yaw), -np.sin(yaw), 0.0],
            [np.sin(yaw),  np.cos(yaw), 0.0],
            [0.0,          0.0,         1.0]
        ])
        # همیشه تقسیم بر ۳
        self.translation = np.array([dx, dy, dz], dtype=float) / 3.0

        # QoS (PX4 uses BEST_EFFORT often)
        qos = QoSProfile(depth=10)
        qos.reliability = ReliabilityPolicy.BEST_EFFORT
        qos.history = HistoryPolicy.KEEP_LAST

        # subscriber to current local position
        self.pose_sub = self.create_subscription(
            VehicleLocalPosition,
            ns + '/fmu/out/vehicle_local_position',
            self.pose_callback,
            qos
        )

        # publishers
        self.offboard_pub = self.create_publisher(OffboardControlMode, ns + '/fmu/in/offboard_control_mode', 10)
        self.setpoint_pub = self.create_publisher(TrajectorySetpoint, ns + '/fmu/in/trajectory_setpoint', 10)

        # timer
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

        self.get_logger().info(f"{ns}: ShapeMoverNode started. translation={self.translation}")

    def pose_callback(self, msg: VehicleLocalPosition):
        try:
            self.current_pos = np.array([msg.x, msg.y, msg.z], dtype=float)
            if hasattr(msg, 'heading'):
                self.current_yaw = float(msg.heading)
        except Exception as e:
            self.get_logger().warn(f"Error parsing VehicleLocalPosition msg: {e}")

    def timer_callback(self):
        if self.current_pos is None:
            return

        if not self.movement_complete:
            self.t += self.timer_period
            alpha = min(self.t / self.duration, 1.0)

            # rotate translation, add to current position
            rotated_translation = self.R @ self.translation
            target_pos = self.current_pos + alpha * rotated_translation


            target_yaw = self.current_yaw  # no yaw change
            self.last_setpoint = self.publish_setpoint(target_pos, target_yaw)

            if alpha >= 1.0:
                self.get_logger().info("✅ Relative translation complete. Holding position.")
                self.movement_complete = True
        else:
            # keep publishing to avoid landing
            if self.last_setpoint is not None:
                self.publish_setpoint(
                    np.array(self.last_setpoint.position),
                    self.last_setpoint.yaw
                )

    def publish_setpoint(self, pos, yaw):
        sp = TrajectorySetpoint()
        sp.position = [float(pos[0]), float(pos[1]), float(pos[2])]
        sp.yaw = float(yaw)
        sp.timestamp = int(self.get_clock().now().nanoseconds // 1000)
        self.setpoint_pub.publish(sp)
        self.publish_offboard_mode()
        return sp

    def publish_offboard_mode(self):
        msg = OffboardControlMode()
        msg.position = True
        msg.timestamp = int(self.get_clock().now().nanoseconds // 1000)
        self.offboard_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ShapeMoverNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
