
import math

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from geometry_msgs.msg import Point, Twist
from nav_msgs.msg import Odometry
from nav_msgs.srv import GetPlan
from visualization_msgs.msg import Marker


class PurePursuit(Node):
    def __init__(self):
        super().__init__('pure_pursuit')

        # Parameters
        self.declare_parameter('lookahead_min', 1.2)
        self.declare_parameter('lookahead_max', 3.8)
        self.declare_parameter('desired_speed', 1.2)
        self.declare_parameter('max_lateral_accel', 0.4)
        self.declare_parameter('max_angular_speed', 1.2)
        self.declare_parameter('odom_timeout', 0.5)

        self.ld_min = self.get_parameter('lookahead_min').value
        self.ld_max = self.get_parameter('lookahead_max').value
        self.v_desired = self.get_parameter('desired_speed').value
        self.ay_max = self.get_parameter('max_lateral_accel').value
        self.omega_max = self.get_parameter('max_angular_speed').value
        self.odom_timeout = self.get_parameter('odom_timeout').value

        # Robot state
        self.x = None
        self.y = None
        self.yaw = None
        self.v = 0.0
        self.last_odom_time = None

        # Waypoints z track_viz (ten sam tor co w launchu)
        self.waypoints = self.load_waypoints()
        if not self.waypoints:
            raise RuntimeError('Serwis /track/get_path nie zwrócił waypointów.')
        self.waypoint_index = None

        # Publishers
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.target_pub = self.create_publisher(Marker, '/pure_pursuit/lookahead', 10)
        self.line_pub = self.create_publisher(Marker, '/pure_pursuit/target_line', 10)

        # Odometry
        self.create_subscription(Odometry, '/odom_gt', self.odom_callback, 10)

        # Loop control - 10 Hz
        self.create_timer(0.1, self.control_loop)

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
    def get_yaw(q):
        return math.atan2(2.0*(q.w*q.z + q.x*q.y), 1.0 - 2.0*(q.y**2 + q.z**2))

    def odom_callback(self, msg):
        pose = msg.pose.pose

        self.x = pose.position.x
        self.y = pose.position.y
        self.yaw = self.get_yaw(pose.orientation)
        self.v = msg.twist.twist.linear.x

        self.last_odom_time = self.get_clock().now()

    def get_lookahead(self):
        speed_ratio = min(abs(self.v) / self.v_desired, 1.0)
        ld = self.ld_min + (self.ld_max - self.ld_min) * speed_ratio**2

        return max(self.ld_min, min(self.ld_max, ld))

    def nearest_segment(self):
        n = len(self.waypoints)
        best_dist = float('inf')
        best_index = 0
        best_t = 0.0

        if self.waypoint_index is None:
            indices = range(n)
        else:
            indices = [(self.waypoint_index + offset) % n for offset in range(-5, 30)]

        for i in indices:
            ax, ay = self.waypoints[i]
            bx, by = self.waypoints[(i + 1) % n]

            dx = bx - ax
            dy = by - ay
            length_sq = dx * dx + dy * dy

            if length_sq < 1e-12:
                continue

            t = ((self.x - ax) * dx + (self.y - ay) * dy) / length_sq
            t = max(0.0, min(1.0, t))

            px = ax + t * dx
            py = ay + t * dy

            dist = math.hypot(self.x - px, self.y - py)

            if dist < best_dist:
                best_dist = dist
                best_index = i
                best_t = t

        return best_index, best_t

    def segment_intersections(self, p1, p2, ld):
        dx = p2[0] - p1[0]
        dy = p2[1] - p1[1]

        fx = p1[0] - self.x
        fy = p1[1] - self.y

        a = dx*dx + dy*dy

        if a < 1e-12:
            return []

        b = 2.0*(fx*dx + fy*dy)
        c = fx*fx + fy*fy - ld*ld

        delta = b * b - 4.0 * a * c

        if delta < 0.0:
            return []

        root = math.sqrt(delta)

        t1 = (-b - root) / (2.0 * a)
        t2 = (-b + root) / (2.0 * a)

        return sorted(t for t in (t1, t2) if 0.0 <= t <= 1.0)

    def find_target(self, ld):
        n = len(self.waypoints)

        start_index, start_t = self.nearest_segment()
        self.waypoint_index = start_index

        heading_x = math.cos(self.yaw)
        heading_y = math.sin(self.yaw)

        for offset in range(n):
            i = (start_index + offset) % n

            p1 = self.waypoints[i]
            p2 = self.waypoints[(i + 1) % n]

            intersections = self.segment_intersections(p1, p2, ld)

            for t in intersections:
                if offset == 0 and t < start_t:
                    continue

                x = p1[0] + t * (p2[0] - p1[0])
                y = p1[1] + t * (p2[1] - p1[1])

                local_x = (x - self.x) * heading_x + (y - self.y) * heading_y
                if local_x <= 1e-6:
                    continue

                return (x, y)

        return None

    def calculate_control(self, target):
        dx = target[0] - self.x
        dy = target[1] - self.y

        local_y = -math.sin(self.yaw) * dx + math.cos(self.yaw) * dy

        distance_sq = dx * dx + dy * dy

        if distance_sq < 1e-9:
            return 0.0, 0.0

        kappa = 2.0 * local_y / distance_sq

        # Ograniczenie przyspieszenia bocznego
        v_curve = math.sqrt(self.ay_max / max(abs(kappa), 1e-6))

        # Ograniczenie predkosci katowej
        v_omega = self.omega_max / max(abs(kappa), 1e-6)

        # Dodatkowe zwalnianie na zakretach
        curve_slowdown = 1.0 / (1.0 + 3.0 * abs(kappa))
        v_adaptive = self.v_desired * curve_slowdown

        v = min(self.v_desired, v_curve, v_omega, v_adaptive)

        omega = v * kappa

        return v, omega

    def publish_cmd(self, v, omega):
        msg = Twist()
        msg.linear.x = v
        msg.angular.z = omega
        self.cmd_pub.publish(msg)

    def make_marker(self, marker_id, marker_type):
        marker = Marker()
        marker.header.frame_id = 'odom_gt'
        marker.header.stamp = self.get_clock().now().to_msg()

        marker.ns = 'pure_pursuit'
        marker.id = marker_id
        marker.type = marker_type
        marker.action = Marker.ADD

        marker.pose.orientation.w = 1.0
        marker.color.a = 1.0

        return marker

    def publish_debug(self, target):
        marker = self.make_marker(0, Marker.SPHERE)

        marker.pose.position.x = target[0]
        marker.pose.position.y = target[1]
        marker.pose.position.z = 0.15

        marker.scale.x = 0.25
        marker.scale.y = 0.25
        marker.scale.z = 0.25

        marker.color.r = 1.0

        self.target_pub.publish(marker)

        line = self.make_marker(1, Marker.LINE_STRIP)

        line.scale.x = 0.035
        line.color.g = 1.0

        start = Point()
        start.x = self.x
        start.y = self.y
        start.z = 0.1

        end = Point()
        end.x = target[0]
        end.y = target[1]
        end.z = 0.1

        line.points = [start, end]

        self.line_pub.publish(line)

    def control_loop(self):
        if self.last_odom_time is None:
            self.publish_cmd(0.0, 0.0)
            return

        odom_age = (self.get_clock().now() - self.last_odom_time).nanoseconds / 1e9

        if odom_age > self.odom_timeout:
            self.publish_cmd(0.0, 0.0)
            return

        ld = self.get_lookahead()
        target = self.find_target(ld)

        if target is None:
            self.publish_cmd(0.0, 0.0)
            return

        v, omega = self.calculate_control(target)

        self.publish_cmd(v, omega)
        self.publish_debug(target)


def main(args=None):
    rclpy.init(args=args)
    node = None

    try:
        node = PurePursuit()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            if rclpy.ok():
                node.publish_cmd(0.0, 0.0)
            node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
