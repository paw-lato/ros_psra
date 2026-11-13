
import csv
import math
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy

from ament_index_python.packages import get_package_share_directory

from geometry_msgs.msg import Point, PoseStamped, TransformStamped, Twist
from nav_msgs.msg import Odometry, Path as PathMsg
from visualization_msgs.msg import Marker
from std_srvs.srv import SetBool

from tf2_ros import TransformBroadcaster
from message_filters import Subscriber, ApproximateTimeSynchronizer


class PurePursuit(Node):
    def __init__(self):
        super().__init__('pure_pursuit')

        # Parameters
        self.declare_parameter('lookahead_min', 0.8)
        self.declare_parameter('lookahead_max', 2.0)
        self.declare_parameter('lookahead_time', 2.0)
        self.declare_parameter('desired_speed', 0.8)
        self.declare_parameter('max_lateral_accel', 0.4)
        self.declare_parameter('max_angular_speed', 1.0)
        self.declare_parameter('odom_timeout', 0.5)

        self.ld_min = self.get_parameter('lookahead_min').value
        self.ld_max = self.get_parameter('lookahead_max').value
        self.ld_time = self.get_parameter('lookahead_time').value
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
        self.running = False

        # Waypoints
        self.waypoints = self.load_waypoints()
        self.waypoint_index = None

        # Publishers
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.target_pub = self.create_publisher(Marker, '/pure_pursuit/lookahead', 10)
        self.line_pub = self.create_publisher(Marker, '/pure_pursuit/target_line', 10)

        path_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.path_pub = self.create_publisher( PathMsg, '/pure_pursuit/path', path_qos)

        # Odometry
        self.create_subscription(Odometry, '/odom_gt', self.odom_callback, 10)

        # TF: odom_gt -> odom
        self.tf_broadcaster = TransformBroadcaster(self)

        self.odom_tf_sub = Subscriber(self, Odometry, '/odom')
        self.gt_tf_sub = Subscriber(self, Odometry, '/odom_gt')

        self.tf_sync = ApproximateTimeSynchronizer(
            [self.odom_tf_sub, self.gt_tf_sub],
            queue_size=30,
            slop=0.05)
        self.tf_sync.registerCallback(self.publish_corrected_tf)

        # Start/stop service
        self.create_service(SetBool, '/pure_pursuit/enable', self.enable_callback)

        # Publish path
        self.publish_path()

        # Loop control - 10 Hz
        self.create_timer(0.1, self.control_loop)

    def load_waypoints(self):
        csv_path = (
            Path(get_package_share_directory('roboracer_sim'))
            / 'worlds' / 'linefollower_track' / 'line_gt.csv'
        )

        with csv_path.open(newline='', encoding='utf-8') as file:
            rows = csv.DictReader(file)
            return [
                (float(row['X']) / 1000.0,
                 float(row['Y']) / 1000.0)
                for row in rows
            ]

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
        ld = self.ld_time * abs(self.v)
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

        v_curve = math.sqrt(self.ay_max / max(abs(kappa), 1e-6))

        v_omega = self.omega_max / max(abs(kappa), 1e-6)

        v = min(self.v_desired, v_curve, v_omega)
        omega = v * kappa

        return v, omega

    def publish_cmd(self, v, omega):
        msg = Twist()
        msg.linear.x = v
        msg.angular.z = omega
        self.cmd_pub.publish(msg)

    def publish_path(self):
        msg = PathMsg()
        msg.header.frame_id = 'odom_gt'
        msg.header.stamp = self.get_clock().now().to_msg()

        for x, y in self.waypoints:
            pose = PoseStamped()
            pose.header = msg.header
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.orientation.w = 1.0
            msg.poses.append(pose)

        msg.poses.append(msg.poses[0])
        self.path_pub.publish(msg)

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

    def publish_corrected_tf(self, odom, gt):
        if (odom.header.frame_id != 'odom' or gt.header.frame_id != 'odom_gt'
            or odom.child_frame_id != 'base_footprint' or gt.child_frame_id != 'base_footprint'):
            return

        op = odom.pose.pose
        gp = gt.pose.pose

        angle = self.get_yaw(gp.orientation) - self.get_yaw(op.orientation)

        c = math.cos(angle)
        s = math.sin(angle)

        x = gp.position.x - c * op.position.x + s * op.position.y
        y = gp.position.y - s * op.position.x - c * op.position.y

        tf = TransformStamped()
        tf.header.stamp = gt.header.stamp
        tf.header.frame_id = 'odom_gt'
        tf.child_frame_id = 'odom'

        tf.transform.translation.x = x
        tf.transform.translation.y = y
        tf.transform.translation.z = gp.position.z - op.position.z

        tf.transform.rotation.z = math.sin(angle / 2.0)
        tf.transform.rotation.w = math.cos(angle / 2.0)

        self.tf_broadcaster.sendTransform(tf)

    def enable_callback(self, request, response):
        self.running = request.data

        if not self.running:
            self.publish_cmd(0.0, 0.0)

        state = 'START' if self.running else 'STOP'
        self.get_logger().info(f'Pure Pursuit {state}')

        response.success = True
        response.message = state

        return response

    def control_loop(self):
        if not self.running:
            return

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
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.publish_cmd(0.0, 0.0)
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
