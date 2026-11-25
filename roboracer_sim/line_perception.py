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

        self.declare_parameter('num_bands', 6)
        # Czerwona linia w YCrCb (OpenCV = YCbCr, kolejność kanałów: Y, Cr, Cb)
        self.declare_parameter('y_min', 20)
        self.declare_parameter('y_max', 255)
        self.declare_parameter('cr_min', 140)  # czerwony → wysoki Cr
        self.declare_parameter('cr_max', 255)
        self.declare_parameter('cb_min', 0)
        self.declare_parameter('cb_max', 140)  # czerwony → niski/średni Cb

        # Parametry geometryczne kamery z URDF
        self.wheelbase = 0.325
        self.x_cam = 0.395
        self.h_cam = 0.120
        self.hfov = 1.2043

        self.bridge = CvBridge()
        # Ostatnia znana strona linii w obrazie: -1 = lewo, +1 = prawo
        self.last_side = 0

        self.error_pub = self.create_publisher(Vector3Stamped, '/perception/line_error', 10)
        self.path_pub = self.create_publisher(PathMsg, '/perception/local_path', 10)
        self.debug_pub = self.create_publisher(Image, '/perception/debug_image', 10)

        self.create_subscription(
            Image,
            '/camera/color/image_raw',
            self.on_image,
            qos_profile_sensor_data,
        )

    def red_mask_ycbcr(self, bgr):
        """Maska czerwonej linii w przestrzeni YCbCr / YCrCb."""
        ycrcb = cv2.cvtColor(bgr, cv2.COLOR_BGR2YCrCb)
        lower = (
            int(self.get_parameter('y_min').value),
            int(self.get_parameter('cr_min').value),
            int(self.get_parameter('cb_min').value),
        )
        upper = (
            int(self.get_parameter('y_max').value),
            int(self.get_parameter('cr_max').value),
            int(self.get_parameter('cb_max').value),
        )
        mask = cv2.inRange(ycrcb, lower, upper)
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        return mask

    def pixel_to_base(self, u, v, cx_img, cy_img, focal):
        denom = v - cy_img
        if abs(denom) < 1e-3:
            denom = 1e-3 if denom >= 0.0 else -1e-3
        x_dist = (self.h_cam * focal) / denom
        x_base = self.x_cam + x_dist
        y_base = -((u - cx_img) * x_dist) / focal
        return x_base, y_base

    def lost_line_corner_points(self, w, h, v_min, v_max, cx_img, cy_img, focal, frame):
        """Zgubiona linia → dolny narożnik po ostatniej stronie (jedź, nie stój)."""
        side = self.last_side if self.last_side != 0 else -1
        margin = max(5.0, 0.04 * w)
        u = margin if side < 0 else (w - 1.0 - margin)
        v_near = float(v_max - 2)
        v_far = float(v_min + (v_max - v_min) * 0.35)
        points = [
            self.pixel_to_base(u, v_near, cx_img, cy_img, focal),
            self.pixel_to_base(u, v_far, cx_img, cy_img, focal),
        ]
        for vv in (v_near, v_far):
            cv2.circle(frame, (int(u), int(vv)), 8, (0, 140, 255), -1)
        cv2.putText(
            frame, 'LOST->corner', (10, h - 12),
            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 140, 255), 1, cv2.LINE_AA,
        )
        return points

    def on_image(self, msg: Image):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        h, w, _ = frame.shape
        cx_img = w / 2.0
        cy_img = h / 2.0
        focal = cx_img / math.tan(self.hfov / 2.0)

        num_bands = int(self.get_parameter('num_bands').value)

        v_min = int(h * 0.58)
        v_max = int(h * 0.95)
        band_h = max(1, (v_max - v_min) // num_bands)

        # Maska czerwieni na całym ROI (YCbCr), potem centroidy w paskach
        roi = frame[v_min:v_max, :]
        red_mask = self.red_mask_ycbcr(roi)

        points_base = []
        last_uv = None

        for i in range(num_bands - 1, -1, -1):
            y1 = i * band_h
            y2 = min(y1 + band_h, red_mask.shape[0])
            band_mask = red_mask[y1:y2, :]

            m = cv2.moments(band_mask)
            if m['m00'] > 100:
                u = m['m10'] / m['m00']
                v = float(v_min + y1) + (m['m01'] / m['m00'])
                x_base, y_base = self.pixel_to_base(u, v, cx_img, cy_img, focal)
                points_base.append((x_base, y_base))
                cv2.circle(frame, (int(u), int(v)), 5, (0, 255, 0), -1)
                last_uv = (int(u), int(v))

        if points_base:
            y0 = points_base[0][1]
            if abs(y0) > 1e-4:
                self.last_side = -1 if y0 > 0.0 else 1

        line_lost = False
        if not points_base:
            line_lost = True
            points_base = self.lost_line_corner_points(
                w, h, v_min, v_max, cx_img, cy_img, focal, frame
            )
        elif len(points_base) == 1:
            x0, y0 = points_base[0]
            points_base.append((x0 + 0.35, y0))

        # Najdalszy punkt (cel) — czerwony na debugu
        if last_uv is not None:
            cv2.circle(frame, last_uv, 9, (0, 0, 255), 2)

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

        err_msg = Vector3Stamped()
        err_msg.header.stamp = msg.header.stamp
        err_msg.header.frame_id = 'base_footprint'

        x0, y0 = points_base[0]
        x1, y1 = points_base[min(2, len(points_base) - 1)]
        psi = math.atan2(y1 - y0, x1 - x0)
        fx, fy = self.wheelbase, 0.0
        e_cte = (fx - x0) * math.sin(psi) - (fy - y0) * math.cos(psi)
        # Cel = najdalszy punkt widzianej linii
        xt, yt = points_base[-1]
        heading_err = math.atan2(yt, xt)
        err_msg.vector.x = e_cte
        err_msg.vector.y = heading_err
        # 1.0 = widoczna, 0.5 = recovery (PID nadal jedzie: z >= 0.5)
        err_msg.vector.z = 0.5 if line_lost else 1.0

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
