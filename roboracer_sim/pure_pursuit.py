import math
import csv
from pathlib import Path
from ament_index_python.packages import get_package_share_directory

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped, PoseStamped, Twist
from nav_msgs.msg import Path as PathMsg
from visualization_msgs.msg import Marker
from std_srvs.srv import SetBool
from tf2_ros import TransformBroadcaster


class PurePursuit(Node):
  def __init__(self):
    super().__init__('pure_pursuit')
    self.x = None
    self.y = None
    self.yaw = None

    self.L = 0.325
    self.declare_parameter("Ld", 1.5)
    self.declare_parameter("speed", 0.5)

    self.waypoints, self.waypoints_len = self.load_waypoints()
    self.waypoint_index = 0
    self.t_prev = None
    
    self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
    self.path_pub = self.create_publisher(PathMsg, '/pid/path', 10)
    self.lookahead_pub = self.create_publisher(Marker, '/pure_pursuit/lookahead', 10)

    self.odom_sub = self.create_subscription(Odometry, '/odom_gt', self.odom_callback, 10)
    
    self.loop_timer = self.create_timer(0.1, self.control_loop)
    self.create_timer(1.0, lambda: self.path_pub.publish(self.path_msg))

    self.tf_broadcaster = TransformBroadcaster(self)

    self.path_msg = PathMsg()
    self.path_msg.header.frame_id = 'odom_gt'
    for x, y in self.waypoints:
        pose = PoseStamped()
        pose.header.frame_id = 'odom_gt'
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.orientation.w = 1.0
        self.path_msg.poses.append(pose)

    self.running = False
    self.control_service = self.create_service(SetBool, '/pure_pursuit/enable', self.enable_callback)

  def load_waypoints(self):
    csv_path = (
        Path(get_package_share_directory('roboracer_sim'))
        / 'worlds' / 'linefollower_track' / 'line_gt.csv')
    with csv_path.open(newline='', encoding='utf-8') as file:
        rows = csv.DictReader(file)
        waypoints = [(float(row['X']) / 1000.0, float(row['Y']) / 1000.0) for row in rows]
    return waypoints, len(waypoints)
  
  def odom_callback(self, msg):
    self.x = msg.pose.pose.position.x
    self.y = msg.pose.pose.position.y

    q = msg.pose.pose.orientation
    self.yaw = math.atan2(2.0*(q.w * q.z + q.x * q.y), 1.0 - 2.0*(q.y**2 + q.z**2))

    t = TransformStamped()

    t.header.stamp = msg.header.stamp
    t.header.frame_id = msg.header.frame_id
    t.child_frame_id = msg.child_frame_id

    t.transform.translation.x = msg.pose.pose.position.x
    t.transform.translation.y = msg.pose.pose.position.y
    t.transform.translation.z = msg.pose.pose.position.z

    t.transform.rotation = msg.pose.pose.orientation

    self.tf_broadcaster.sendTransform(t)

  def publish_lookahead(self, point):
    marker = Marker()

    marker.header.frame_id = 'odom_gt'
    marker.header.stamp = self.get_clock().now().to_msg()

    marker.ns = 'lookahead'
    marker.id = 0
    marker.type = Marker.SPHERE
    marker.action = Marker.ADD

    marker.pose.position.x = point[0]
    marker.pose.position.y = point[1]
    marker.pose.position.z = 0.15
    marker.pose.orientation.w = 1.0

    marker.scale.x = 0.25
    marker.scale.y = 0.25
    marker.scale.z = 0.25

    # Kolor czerwony
    marker.color.r = 1.0
    marker.color.g = 0.0
    marker.color.b = 0.0
    marker.color.a = 1.0

    self.lookahead_pub.publish(marker)

  def enable_callback(self, request, response):
    self.running = request.data

    if self.running:
        self.get_logger().info("Pure Pursuit START")
    else:
        self.get_logger().info("Pure Pursuit STOP")
        self.cmd_vel_pub.publish(Twist())

    response.success = True
    response.message = "Started" if self.running else "Stopped"

    return response

  def lookahead_point(self, p1, p2):
    xr = self.x
    yr = self.y
    Ld = self.get_parameter('Ld').value

    dx = p2[0] - p1[0]
    dy = p2[1] - p1[1]

    fx = p1[0] - xr
    fy = p1[1] - yr

    a = dx**2 + dy**2
    b = 2 * (fx * dx + fy * dy)
    c = fx**2 + fy**2 - Ld**2

    if a == 0:
        return None

    delta = b**2 - 4*a*c

    if delta < 0:
        return None

    t1 = (-b - math.sqrt(delta)) / (2*a)
    t2 = (-b + math.sqrt(delta)) / (2*a)

    valid_t = [t for t in (t1, t2) if 0 <= t <= 1]

    if not valid_t:
        return None

    t = max(valid_t)

    x = p1[0] + t*dx
    y = p1[1] + t*dy
    return (x, y), t

  def control_loop(self):
    if not self.running:
        return
    
    if self.x is None or self.y is None or self.yaw is None:
      return 

    point = None
    for i in range(self.waypoints_len):
      index = (self.waypoint_index + i) % self.waypoints_len

      p1 = self.waypoints[index]
      p2 = self.waypoints[(index + 1) % self.waypoints_len]
      result = self.lookahead_point(p1, p2)

      if result is None:
            continue
      candidate, t = result

      if index == self.waypoint_index and self.t_prev is not None:
            if t < self.t_prev - 1e-6:
                continue

      point = candidate
      self.waypoint_index = index
      self.t_prev = t
      break
    
    if point is None:
      self.get_logger().warn(
          f"No Look Ahead Point! "
          f"Robot: ({self.x:.2f}, {self.y:.2f}), "
          f"Index: {self.waypoint_index}")

      self.cmd_vel_pub.publish(Twist())
      return

    self.publish_lookahead(point)

    alpha = math.atan2(point[1] - self.y, point[0] - self.x) - self.yaw
    Ld = self.get_parameter("Ld").value
    v_desired = self.get_parameter("speed").value

    ay_max = 1.0
    omega_max = 1.5

    kappa = 2.0 * math.sin(alpha) / Ld

    v_curve = math.sqrt(ay_max / max(abs(kappa), 1e-6))

    v_omega = omega_max / max(abs(kappa), 1e-6)

    v = min(v_desired, v_curve, v_omega)
    
    omega = v * kappa

    msg = Twist()
    msg.linear.x = v
    msg.angular.z = omega

    self.cmd_vel_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    node = PurePursuit()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cmd_vel_pub.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()