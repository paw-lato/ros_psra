import math

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from rclpy.qos import qos_profile_sensor_data

class PID(Node):
  def __init__(self):
    super().__init__('control_pid')

    self.kp = 0.8
    self.ki = 0.2
    self.kd = 0.1

    self.integrator = 0
    self.last_error = 0
    self.last_time = self.get_clock().now()

    self.speed = 1.8
    self.max_turn = 0.8
    
    self.publisher = self.create_publisher(
      Twist,
      '/cmd_vel',
      10
    )

    self.subscription = self.create_subscription(
        LaserScan,
        '/scan',
        self.lidar_callback,
        qos_profile_sensor_data
    )

    # self.timer = self.create_timer(
    #   0.1,
    #   self.send_command
    # )

  def get_distance(self, msg, angle_deg):
    angle = math.radians(angle_deg)

    index = round((angle - msg.angle_min) / msg.angle_increment)

    if index < 0 or index >= len(msg.ranges):
        return None

    distance = msg.ranges[index]

    if (not math.isfinite(distance) or distance < msg.range_min or distance > msg.range_max):
      return None

    return distance

  def lidar_callback(self, msg):
    left_45 = self.get_distance(msg, 45)
    right_45 = self.get_distance(msg, -45)

    if left_45 is None:
        left_45 = msg.range_max

    if right_45 is None:
        right_45 = msg.range_max

    command = Twist()

    e = left_45 - right_45

    current_time = self.get_clock().now()
    dt = (current_time - self.last_time).nanoseconds / 1e9
    self.last_time = current_time

    self.integrator += e * dt
    self.integrator = max(-2.0, min(2.0, self.integrator))

    turn = self.kp * e + self.ki * self.integrator + self.kd * (e-self.last_error)/dt
    turn = max(-self.max_turn, min(self.max_turn, turn))
    self.last_error = e

    command.linear.x = self.speed
    command.angular.z = turn

    self.publisher.publish(command)


def main(args=None):
  rclpy.init(args=args)

  node = PID()

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
