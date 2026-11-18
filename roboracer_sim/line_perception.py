import math
import cv2
from cv_bridge import CvBridge
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped, Vector3Stamped
from nav_msgs.msg import Path as PathMsg

class LinePerception(Node):

  def __init__(self):

      super().__init__('line_perception')

      self.declare_parameter('threshold', 60)      # próg jasności czarnej linii (0-255)
      self.declare_parameter('num_bands', 6)       # na ile pasów dzielimy dół obrazu
      
      # Parametry geometryczne kamery z roboracer.urdf.xacro
      self.wheelbase = 0.325
      self.x_cam = 0.395       # odległość kamery od tylnej osi [m]
      self.h_cam = 0.120       # wysokość optyczna kamery nad ziemią [m]
      self.hfov = 1.2043       # 69 stopni [rad]

      self.bridge = CvBridge()

      self.error_pub = self.create_publisher(Vector3Stamped, '/perception/line_error', 10)
      self.path_pub = self.create_publisher(PathMsg, '/perception/local_path', 10)
      self.debug_pub = self.create_publisher(Image, '/perception/debug_image', 10)

      self.create_subscription(
            Image,
            '/camera/color/image_raw',
            self.on_image,
            qos_profile_sensor_data,
        )

  def on_image(self, msg: Image):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        h, w, _ = frame.shape
        cx_img = w / 2.0
        cy_img = h / 2.0
        focal = cx_img / math.tan(self.hfov / 2.0)

        thresh_val = int(self.get_parameter('threshold').value)
        num_bands = int(self.get_parameter('num_bands').value)

        # Analizujemy dolną część obrazu (poniżej horyzontu, od dołu do góry)
        v_min = int(h * 0.58)
        v_max = int(h * 0.95)
        band_h = (v_max - v_min) // num_bands

        points_base = []  # punkty (X, Y) w układzie base_footprint (od najbliższego do najdalszego)
        
        # Idziemy od dołu obrazu (najbliżej auta) w górę (dalej przed autem)
        for i in range(num_bands - 1, -1, -1):
            y1 = v_min + i * band_h
            y2 = y1 + band_h
            band = frame[y1:y2, :]

            gray = cv2.cvtColor(band, cv2.COLOR_BGR2GRAY)
            _, mask = cv2.threshold(gray, thresh_val, 255, cv2.THRESH_BINARY_INV)

            m = cv2.moments(mask)
            if m['m00'] > 100:  # wykryto czarną linię w tym pasku
                u = m['m10'] / m['m00']
                v = y1 + (m['m01'] / m['m00'])
                # Rzutowanie piksela (u, v) na płaszczyznę Z=0 w base_footprint

                x_dist = (self.h_cam * focal) / (v - cy_img)
                x_base = self.x_cam + x_dist
                y_base = -((u - cx_img) * x_dist) / focal
                points_base.append((x_base, y_base))

                cv2.circle(frame, (int(u), int(v)), 5, (0, 255, 0), -1)

        # 1. Publikacja lokalnej ścieżki (dla Pure Pursuit i RViz)
        path_msg = PathMsg()
        path_msg.header.stamp = msg.header.stamp
        path_msg.header.frame_id = 'base_footprint'
        for x_b, y_b in points_base:
            pose = PoseStamped()
            pose.header = path_msg.header
            pose.pose.position.x = x_b
            pose.pose.position.y = y_b
            pose.pose.orientation.w = 1.0
            path_msg.poses.append(pose)
        self.path_pub.publish(path_msg)

        # 2. Publikacja uchybów (dla PID i Stanleya)
        err_msg = Vector3Stamped()
        err_msg.header.stamp = msg.header.stamp
        err_msg.header.frame_id = 'base_footprint'

        if len(points_base) >= 2:
            # Najbliższy punkt (X0, Y0) i kolejny (X1, Y1)
            x0, y0 = points_base[0]
            x1, y1 = points_base[min(2, len(points_base) - 1)]

            # Kąt stycznej linii względem osi podłużnej pojazdu (dla Stanleya: psi)
            psi = math.atan2(y1 - y0, x1 - x0)

            # Błąd poprzeczny e względem przedniej osi (x = wheelbase, y = 0)
            # Zgodny ze znakiem w control_stanley.py:
            fx, fy = self.wheelbase, 0.0
            e_cte = (fx - x0) * math.sin(psi) - (fy - y0) * math.cos(psi)

            # Kąt do punktu docelowego w środku pola widzenia (dla HeadingPid)
            xt, yt = points_base[len(points_base) // 2]
            
            heading_err = math.atan2(yt, xt)
            err_msg.vector.x = e_cte        # błąd poprzeczny [m] (Stanley)
            err_msg.vector.y = heading_err  # błąd kątowy do punktu [rad] (PID) / lub psi
            err_msg.vector.z = 1.0          # flaga: 1.0 = linia widoczna

        elif len(points_base) == 1:
            x0, y0 = points_base[0]
            err_msg.vector.x = -y0
            err_msg.vector.y = math.atan2(y0, x0)
            err_msg.vector.z = 1.0
        else:
            err_msg.vector.z = 0.0          # 0.0 = zgubiono linię
            
        self.error_pub.publish(err_msg)
        self.debug_pub.publish(self.bridge.cv2_to_imgmsg(frame, encoding='bgr8'))

def main(args=None):
    rclpy.init(args=args)
    node = LinePerception()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
            
if __name__ == '__main__':
    main()