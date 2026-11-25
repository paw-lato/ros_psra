#!/usr/bin/env python3
"""Tor dla RViz i regulatorów.

- /track/gt_path: linia GT z line_gt.csv (nav_msgs/Path, ramka odom_gt)
- /track/mesh:    model 3D toru z plików .obj (MarkerArray, ramka odom_gt)
- TF odom_gt -> odom: korekta dryfu odometrii kół, żeby robot i LiDAR
  (w ramce odom) zgadzały się w RViz z torem (w ramce odom_gt)
"""
import csv
import math
import os

import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import Odometry, Path
from nav_msgs.srv import GetPlan
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from tf2_ros import TransformBroadcaster

class TrackViz(Node):
    def __init__(self):
        super().__init__('track_viz')
        track = self.declare_parameter('track', 'linefollower_track_easy').value
        track_dir = os.path.join(
            get_package_share_directory('roboracer_sim'), 'worlds', track
        )

        # Linia GT
        self.path = Path()
        self.path.header.frame_id = 'odom_gt'
        with open(os.path.join(track_dir, 'line_gt.csv'), newline='', encoding='utf-8-sig') as f:
            for row in list(csv.DictReader(f))[:-1]:
                pose = PoseStamped()
                pose.header.frame_id = 'odom_gt'
                pose.pose.position.x = float(row['X']) / 1000.0
                pose.pose.position.y = float(row['Y']) / 1000.0
                pose.pose.orientation.w = 1.0
                self.path.poses.append(pose)

        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.path_pub = self.create_publisher(Path, '/track/gt_path', latched)
        self.create_service(GetPlan, '/track/get_path', self.get_path_callback)

        self.create_timer(0.5, self.publish_track)
        self.publish_track()

        self.wheel_odom = None
        self.tf_broadcaster = TransformBroadcaster(self)
        self.create_subscription(Odometry, '/odom', self.wheel_odom_callback, 10)
        self.create_subscription(Odometry, '/odom_gt', self.gt_odom_callback, 10)

        self.get_logger().info(f'Tor "{track}": {len(self.path.poses)} punktów')

    def publish_track(self):
        self.path_pub.publish(self.path)

    def get_path_callback(self, request, response):
        response.plan = self.path
        return response

    def wheel_odom_callback(self, msg):
        self.wheel_odom = msg

    def gt_odom_callback(self, gt):
        if self.wheel_odom is None:
            return

        g = gt.pose.pose
        o = self.wheel_odom.pose.pose

        yaw_gt = 2.0 * math.atan2(g.orientation.z, g.orientation.w)
        yaw_wheel = 2.0 * math.atan2(o.orientation.z, o.orientation.w)
        angle = yaw_gt - yaw_wheel
        c, s = math.cos(angle), math.sin(angle)

        tf = TransformStamped()
        tf.header.stamp = gt.header.stamp
        tf.header.frame_id = 'odom_gt'
        tf.child_frame_id = 'odom'
        tf.transform.translation.x = g.position.x - (c * o.position.x - s * o.position.y)
        tf.transform.translation.y = g.position.y - (s * o.position.x + c * o.position.y)
        tf.transform.rotation.z = math.sin(angle / 2.0)
        tf.transform.rotation.w = math.cos(angle / 2.0)
        self.tf_broadcaster.sendTransform(tf)


def main(args=None):
    rclpy.init(args=args)
    node = TrackViz()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
