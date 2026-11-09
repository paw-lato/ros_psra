#!/usr/bin/env python3
import csv
import math
import os

import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PointStamped, TransformStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from tf2_ros import TransformBroadcaster
from visualization_msgs.msg import Marker, MarkerArray


class Stanley(Node):
    def __init__(self):
        super().__init__('control_stanley')

        self.k = 1.6
        self.ks = 0.15
        self.speed = 0.8
        self.wheelbase = 0.325
        self.steering_limit = 0.42

        track_dir = os.path.join(
            get_package_share_directory('roboracer_sim'), 'worlds', 'linefollower_track'
        )

        self.waypoints = []
        with open(os.path.join(track_dir, 'line_gt.csv'), newline='') as f:
            for row in csv.DictReader(f):
                self.waypoints.append((float(row['X']) / 1000.0, float(row['Y']) / 1000.0))
        self.waypoints = self.waypoints[:-1]

        self.track_markers = MarkerArray()
        meshes = [
            ('track.obj', (0.25, 0.25, 0.25, 1.0), 0.0),
            ('line.obj', (1.0, 0.9, 0.0, 1.0), 0.01),
            ('barrier_1.obj', (0.8, 0.2, 0.2, 0.6), 0.0),
            ('barrier_2.obj', (0.8, 0.2, 0.2, 0.6), 0.0),
        ]
        for marker_id, (filename, rgba, z) in enumerate(meshes):
            m = Marker()
            m.header.frame_id = 'odom_gt'
            m.ns = 'linefollower_track'
            m.id = marker_id
            m.type = Marker.MESH_RESOURCE
            m.mesh_resource = (
                'package://roboracer_sim/worlds/linefollower_track/meshes/' + filename
            )
            m.pose.position.z = z
            m.pose.orientation.w = 1.0
            m.scale.x = m.scale.y = m.scale.z = 100.0
            m.color.r, m.color.g, m.color.b, m.color.a = rgba
            self.track_markers.markers.append(m)

        self.wheel_odom = None

        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.track_pub = self.create_publisher(MarkerArray, '/stanley/track_mesh', 10)
        self.target_pub = self.create_publisher(PointStamped, '/stanley/target', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.create_subscription(
            Odometry, '/odom', lambda msg: setattr(self, 'wheel_odom', msg), 10
        )
        self.create_subscription(Odometry, '/odom_gt', self.odom_callback, 10)
        self.create_timer(1.0, lambda: self.track_pub.publish(self.track_markers))

    def odom_callback(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        theta = 2.0 * math.atan2(q.z, q.w)
        v = math.hypot(msg.twist.twist.linear.x, msg.twist.twist.linear.y)

        if self.wheel_odom is not None:
            p = self.wheel_odom.pose.pose.position
            qw = self.wheel_odom.pose.pose.orientation
            th = theta - 2.0 * math.atan2(qw.z, qw.w)

            t = TransformStamped()
            t.header.stamp = msg.header.stamp
            t.header.frame_id = 'odom_gt'
            t.child_frame_id = 'odom'
            t.transform.translation.x = x - (math.cos(th) * p.x - math.sin(th) * p.y)
            t.transform.translation.y = y - (math.sin(th) * p.x + math.cos(th) * p.y)
            t.transform.rotation.z = math.sin(th / 2.0)
            t.transform.rotation.w = math.cos(th / 2.0)
            self.tf_broadcaster.sendTransform(t)

        fx = x + self.wheelbase * math.cos(theta)
        fy = y + self.wheelbase * math.sin(theta)

        i = min(
            range(len(self.waypoints)),
            key=lambda j: math.hypot(fx - self.waypoints[j][0], fy - self.waypoints[j][1]),
        )
        px, py = self.waypoints[i]
        nx, ny = self.waypoints[(i + 1) % len(self.waypoints)]
        theta_p = math.atan2(ny - py, nx - px)

        target = PointStamped()
        target.header.stamp = msg.header.stamp
        target.header.frame_id = 'odom_gt'
        target.point.x = px
        target.point.y = py
        self.target_pub.publish(target)

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
    node = Stanley()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.publisher.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
