#!/usr/bin/env python3
import math

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from nav_msgs.msg import Path as PathMsg
from nav_msgs.srv import GetPlan
from rclpy.node import Node


class Stanley(Node):
    def __init__(self):
        super().__init__('control_stanley')

        self.declare_parameter('path_source', 'gt')  # gt | camera

        self.k = 1.6
        self.ks = 0.15
        self.speed = 0.8
        self.wheelbase = 0.325
        self.steering_limit = 0.42

        self.path_source = str(self.get_parameter('path_source').value)
        self.waypoints = []
        self.closed_path = self.path_source != 'camera'

        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        if self.path_source == 'camera':
            self.create_subscription(
                PathMsg, '/perception/local_path', self.on_vision_path, 10
            )
            self.get_logger().info('Stanley: waypointy z kamery (/perception/local_path)')
        else:
            self.waypoints = list(self.load_waypoints_gt())
            if not self.waypoints:
                raise RuntimeError('Serwis /track/get_path nie zwrócił waypointów.')
            self.get_logger().info('Stanley: waypointy z GT (/track/get_path)')

        self.create_subscription(Odometry, '/odom_gt', self.odom_callback, 10)

    def load_waypoints_gt(self):
        client = self.create_client(GetPlan, '/track/get_path')
        while rclpy.ok() and not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Czekam na serwis /track/get_path...')

        future = client.call_async(GetPlan.Request())
        rclpy.spin_until_future_complete(self, future)
        result = future.result()
        if result is None:
            raise RuntimeError('Nie udało się pobrać ścieżki z /track/get_path')

        return tuple(
            (p.pose.position.x, p.pose.position.y) for p in result.plan.poses
        )

    def on_vision_path(self, msg: PathMsg):
        self.waypoints = [
            (p.pose.position.x, p.pose.position.y) for p in msg.poses
        ]

    def odom_callback(self, msg):
        if len(self.waypoints) < 2:
            self.publisher.publish(Twist())
            return

        if self.path_source == 'camera':
            # Lokalna ścieżka w base_footprint.
            x, y, theta = 0.0, 0.0, 0.0
            v = abs(msg.twist.twist.linear.x)
        else:
            x = msg.pose.pose.position.x
            y = msg.pose.pose.position.y
            q = msg.pose.pose.orientation
            theta = 2.0 * math.atan2(q.z, q.w)
            v = math.hypot(msg.twist.twist.linear.x, msg.twist.twist.linear.y)

        fx = x + self.wheelbase * math.cos(theta)
        fy = y + self.wheelbase * math.sin(theta)

        i = min(
            range(len(self.waypoints)),
            key=lambda j: math.hypot(
                fx - self.waypoints[j][0], fy - self.waypoints[j][1]
            ),
        )
        px, py = self.waypoints[i]
        if self.closed_path:
            nx, ny = self.waypoints[(i + 1) % len(self.waypoints)]
        else:
            nx, ny = self.waypoints[min(i + 1, len(self.waypoints) - 1)]
        theta_p = math.atan2(ny - py, nx - px)

        psi = math.atan2(math.sin(theta_p - theta), math.cos(theta_p - theta))
        e = (fx - px) * math.sin(theta_p) - (fy - py) * math.cos(theta_p)

        delta = psi + math.atan2(self.k * e, self.ks + v)
        delta = max(-self.steering_limit, min(self.steering_limit, delta))

        command = Twist()
        command.linear.x = self.speed
        command.angular.z = self.speed * math.tan(delta) / self.wheelbase
        self.publisher.publish(command)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = Stanley()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.publisher.publish(Twist())
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
