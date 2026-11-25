import math

import rclpy
from geometry_msgs.msg import PoseStamped, Twist, Vector3Stamped
from nav_msgs.msg import Odometry
from nav_msgs.msg import Path as PathMsg
from nav_msgs.srv import GetPlan
from rclpy.node import Node


class HeadingPid(Node):
    def __init__(self):
        super().__init__('heading_pid')

        self.declare_parameter('speed', 0.3)
        self.declare_parameter('kp', 2.0)
        self.declare_parameter('ki', 0.5)
        self.declare_parameter('kd', 0.3)
        self.declare_parameter('waypoint_tolerance', 1.0)
        # True gdy experiment path_source:=camera
        self.declare_parameter('use_camera', False)

        self.use_camera = bool(self.get_parameter('use_camera').value)
        self.waypoint_index = 0
        self.integral = 0.0
        self.previous_error = None
        self.previous_time = None
        self.waypoints = []

        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        if self.use_camera:
            self.get_logger().info('Tryb: percepcja z kamery (/perception/line_error)')
            self.create_subscription(
                Vector3Stamped,
                '/perception/line_error',
                self.on_line_error,
                10,
            )
        else:
            self.get_logger().info('Tryb: waypointy z CSV (/track/get_path + /odom_gt)')
            self.waypoints = list(self.load_waypoints())
            if not self.waypoints:
                raise RuntimeError('Serwis /track/get_path nie zwrócił waypointów.')

            self.path_msg = PathMsg()
            self.path_msg.header.frame_id = 'odom_gt'
            for x, y in self.waypoints:
                pose = PoseStamped()
                pose.header.frame_id = 'odom_gt'
                pose.pose.position.x = x
                pose.pose.position.y = y
                pose.pose.orientation.w = 1.0
                self.path_msg.poses.append(pose)

            self.path_pub = self.create_publisher(PathMsg, '/pid/path', 10)
            self.create_subscription(Odometry, '/odom_gt', self.on_odom, 10)

            def send_path():
                self.path_msg.header.stamp = self.get_clock().now().to_msg()
                self.path_pub.publish(self.path_msg)

            self.create_timer(1.0, send_path)

    def load_waypoints(self):
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
        tolerance = float(self.get_parameter('waypoint_tolerance').value)
        if distance < tolerance:
            self.waypoint_index = (self.waypoint_index + 1) % len(self.waypoints)
            self.integral = 0.0
            self.previous_error = None
            target_x, target_y = self.waypoints[self.waypoint_index]
        target_yaw = math.atan2(target_y - y, target_x - x)
        error = self.wrap_angle(target_yaw - yaw)
        self.compute_pid(error)

    def on_line_error(self, msg: Vector3Stamped):
        # z < 0.5 tylko gdy naprawdę brak ścieżki (recovery ma z=0.5)
        if msg.vector.z < 0.5:
            command = Twist()
            command.linear.x = float(self.get_parameter('speed').value) * 0.5
            if self.previous_error is not None:
                command.angular.z = 0.4 if self.previous_error > 0 else -0.4
            self.publisher.publish(command)
            return
        error = self.wrap_angle(msg.vector.y)
        self.compute_pid(error)

    def compute_pid(self, error):
        now = self.get_clock().now()
        dt = 0.0 if self.previous_time is None else (
            now - self.previous_time
        ).nanoseconds / 1e9
        derivative = 0.0
        if dt > 1e-4:
            self.integral += error * dt
            self.integral = max(-0.2, min(0.2, self.integral))
            if self.previous_error is not None:
                derivative = self.wrap_angle(error - self.previous_error) / dt
        angular_speed = (
            float(self.get_parameter('kp').value) * error
            + float(self.get_parameter('ki').value) * self.integral
            + float(self.get_parameter('kd').value) * derivative
        )
        angular_speed = max(-0.5, min(0.5, angular_speed))

        speed = float(self.get_parameter('speed').value)
        # Ostry zakręt → zwolnij
        turn_factor = max(0.2, math.cos(error) ** 2)
        if self.use_camera:
            speed = min(speed, 0.25) * turn_factor
        else:
            speed = speed * max(0.35, turn_factor)

        command = Twist()
        command.linear.x = speed
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
