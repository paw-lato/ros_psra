import math

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node

import csv
from pathlib import Path
from ament_index_python.packages import get_package_share_directory


class HeadingPid(Node):
    def __init__(self):
        super().__init__('heading_pid')

        self.declare_parameter('target_yaw', 0.0)
        self.declare_parameter('speed', 0.3)
        self.declare_parameter('kp', 3.0)
        self.declare_parameter('ki', 0.01)
        self.declare_parameter('kd', 0.1)

        self.waypoints = self.load_waypoints()

        self.waypoint_index = 0
        self.waypoint_tolerance = 1.0

        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.subscription = self.create_subscription(
            Odometry, '/odom', self.on_odom, 10
        )

        self.previous_time = None
        self.previous_error = None
        self.integral = 0.0

    def load_waypoints(self):
        csv_path = (
            Path(get_package_share_directory('roboracer_sim'))
            / 'worlds' / 'linefollower_track' / 'line_gt.csv'
        )
        with csv_path.open(newline='', encoding='utf-8') as file:
            rows = csv.DictReader(file)
            return [(float(row['X']) / 1000.0, float(row['Y']) / 1000.0) for row in rows]
    
    def on_odom(self, msg):
        q = msg.pose.pose.orientation
        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )

        # target = self.get_parameter('target_yaw').value

        position = msg.pose.pose.position
        x, y = position.x, position.y

        target_x, target_y = self.waypoints[self.waypoint_index]
        dx = target_x - x
        dy = target_y - y
        distance = math.hypot(dx, dy)

        if distance < self.waypoint_tolerance:
            self.waypoint_index = (
                self.waypoint_index + 1
            ) % len(self.waypoints)

            # Anti-windup
            self.integral = 0.0
            self.previous_error = None

            target_x, target_y = self.waypoints[self.waypoint_index]
            dx = target_x - x
            dy = target_y - y

        target = math.atan2(dy, dx)

        error = math.atan2(math.sin(target - yaw), math.cos(target - yaw))

        now = self.get_clock().now()
        dt = 0.0 if self.previous_time is None else (
            now - self.previous_time
        ).nanoseconds / 1e9

        if dt > 0.0:
            self.integral = max(-0.2, min(0.2, self.integral + error * dt))

        derivative = 0.0
        if dt > 1e-4 and self.previous_error is not None:
            delta = math.atan2(
                math.sin(error - self.previous_error),
                math.cos(error - self.previous_error),
            )
            derivative = delta / dt

        steering = (
            self.get_parameter('kp').value * error
            + self.get_parameter('ki').value * self.integral
            + self.get_parameter('kd').value * derivative
        )
        steering = max(-0.5, min(0.5, steering)) # clamp steering angle

        command = Twist()
        command.linear.x = self.get_parameter('speed').value
        command.angular.z = steering
        self.publisher.publish(command)

        self.previous_time = now
        self.previous_error = error


def main(args=None):
    rclpy.init(args=args)
    node = HeadingPid()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()