import csv
import math
from pathlib import Path

import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from nav_msgs.msg import Path as PathMsg
from rclpy.node import Node


class HeadingPid(Node):
    def __init__(self):
        super().__init__('heading_pid')

        self.declare_parameter('speed', 0.3)
        self.declare_parameter('kp', 2.0)
        self.declare_parameter('ki', 0.5)
        self.declare_parameter('kd', 0.3)
        self.declare_parameter('waypoint_tolerance', 1.0)

        self.waypoints = self.load_waypoints()
        if not self.waypoints:
            raise RuntimeError('Plik CSV nie zawiera waypointów.')

        self.waypoint_index = 0
        self.integral = 0.0
        self.previous_error = None
        self.previous_time = None

        target_frame = 'odom_gt'

        self.path_msg = PathMsg()
        self.path_msg.header.frame_id = 'odom_gt'
        for x, y in self.waypoints:
            pose = PoseStamped()
            pose.header.frame_id = 'odom_gt'
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.orientation.w = 1.0
            self.path_msg.poses.append(pose)

        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.path_pub = self.create_publisher(PathMsg, '/pid/path', 10)
        self.subscription = self.create_subscription(
            Odometry,
            '/odom_gt',
            self.on_odom,
            10,
        )
        
        def send_path():
            self.path_msg.header.stamp = self.get_clock().now().to_msg()
            self.path_pub.publish(self.path_msg)

        self.create_timer(1.0, send_path)
    
    def load_waypoints(self):
        csv_path = (
            Path(get_package_share_directory('roboracer_sim'))
            / 'worlds'
            / 'linefollower_track'
            / 'line_gt.csv'
        )

        with csv_path.open(newline='', encoding='utf-8-sig') as file:
            rows = csv.DictReader(file)
            # W CSV współrzędne są w mm; dzielimy przez 1000, aby uzyskać m.
            return [
                (float(row['X']) / 1000.0, float(row['Y']) / 1000.0)
                for row in rows
            ]

    @staticmethod
    def wrap_angle(angle):
        return math.atan2(math.sin(angle), math.cos(angle))

    def on_odom(self, msg):
        q = msg.pose.pose.orientation
        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )

        position = msg.pose.pose.position
        x, y = position.x, position.y

        target_x, target_y = self.waypoints[self.waypoint_index]
        distance = math.hypot(target_x - x, target_y - y)

        tolerance = float(
            self.get_parameter('waypoint_tolerance').value
        )

        if distance < tolerance:
            # Po ostatnim punkcie wraca do pierwszego.
            self.waypoint_index = (
                self.waypoint_index + 1
            ) % len(self.waypoints)

            self.integral = 0.0
            self.previous_error = None

            target_x, target_y = self.waypoints[self.waypoint_index]

        target_yaw = math.atan2(target_y - y, target_x - x)
        error = self.wrap_angle(target_yaw - yaw)

        now = self.get_clock().now()
        dt = 0.0 if self.previous_time is None else (
            now - self.previous_time
        ).nanoseconds / 1e9

        derivative = 0.0
        if dt > 1e-4:
            self.integral += error * dt
            self.integral = max(-0.2, min(0.2, self.integral))

            if self.previous_error is not None:
                derivative = self.wrap_angle(
                    error - self.previous_error
                ) / dt

        angular_speed = (
            float(self.get_parameter('kp').value) * error
            + float(self.get_parameter('ki').value) * self.integral
            + float(self.get_parameter('kd').value) * derivative
        )

        # Limit prędkości kątowej w rad/s.
        angular_speed = max(-0.5, min(0.5, angular_speed))

        command = Twist()
        command.linear.x = float(self.get_parameter('speed').value)
        command.angular.z = angular_speed
        self.publisher.publish(command)

        self.previous_time = now
        self.previous_error = error


def main(args=None):
    rclpy.init(args=args)
    node = None

    try:
        node = HeadingPid()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
