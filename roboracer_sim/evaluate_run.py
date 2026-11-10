"""Ewaluacja przejazdu: pozycja z /odom_gt vs linia GT z CSV."""

import csv
import math
from pathlib import Path

import rclpy
from ament_index_python.packages import get_package_share_directory
from nav_msgs.msg import Odometry
from rclpy.node import Node


########### WCZYTYWANIE GT #########
def load_gt_path(csv_path):
    # CSV ma X,Y w milimetrach — zamieniamy na metry.
    points = []
    with open(csv_path, newline='', encoding='utf-8-sig') as file:
        for row in csv.DictReader(file):
            x = float(row['X']) / 1000.0
            y = float(row['Y']) / 1000.0
            points.append((x, y))
    return points


def path_length(points):
    # Suma długości odcinków polyline.
    total = 0.0
    for i in range(len(points) - 1):
        x0, y0 = points[i]
        x1, y1 = points[i + 1]
        total += math.hypot(x1 - x0, y1 - y0)
    return total


########### GEOMETRIA: BŁĄD POPRZECZNY #########
def distance_point_to_segment(px, py, ax, ay, bx, by):
    # Odległość punktu P od odcinka AB oraz parametr t w [0, 1].
    abx = bx - ax
    aby = by - ay
    apx = px - ax
    apy = py - ay
    ab2 = abx * abx + aby * aby

    if ab2 < 1e-12:
        return math.hypot(apx, apy), 0.0

    t = (apx * abx + apy * aby) / ab2
    if t < 0.0:
        t = 0.0
    elif t > 1.0:
        t = 1.0

    cx = ax + t * abx
    cy = ay + t * aby
    return math.hypot(px - cx, py - cy), t


def lateral_error_and_progress(px, py, points, segment_lengths):
    # Najmniejsza odległość do ścieżki + odległość wzdłuż toru do rzutu.
    best_dist = 1e9
    best_s = 0.0
    s_before = 0.0

    for i in range(len(points) - 1):
        ax, ay = points[i]
        bx, by = points[i + 1]
        dist, t = distance_point_to_segment(px, py, ax, ay, bx, by)
        if dist < best_dist:
            best_dist = dist
            best_s = s_before + t * segment_lengths[i]
        s_before += segment_lengths[i]

    return best_dist, best_s


########### METRYKI #########
def rmse(values):
    if not values:
        return 0.0
    acc = 0.0
    for v in values:
        acc += v * v
    return math.sqrt(acc / len(values))


def mean(values):
    if not values:
        return 0.0
    return sum(values) / len(values)


class EvaluateRun(Node):
    def __init__(self):
        super().__init__('evaluate_run')

        ########### PARAMETRY #########
        self.declare_parameter('controller_name', 'unknown')
        self.declare_parameter('odom_topic', '/odom_gt')
        self.declare_parameter('report_period', 10.0)  # sekundy między raportami
        self.declare_parameter('lap_finish_radius', 1.5)  # [m] powrót do startu
        self.declare_parameter('min_progress_ratio', 0.90)  # min. 90% długości toru
        self.declare_parameter('start_move_distance', 1.0)  # [m] od startu = „wystartował”

        self.controller_name = self.get_parameter('controller_name').value
        report_period = float(self.get_parameter('report_period').value)
        self.lap_finish_radius = float(self.get_parameter('lap_finish_radius').value)
        self.min_progress_ratio = float(self.get_parameter('min_progress_ratio').value)
        self.start_move_distance = float(self.get_parameter('start_move_distance').value)

        ########### ŚCIEŻKA GT #########
        csv_path = (
            Path(get_package_share_directory('roboracer_sim'))
            / 'worlds'
            / 'linefollower_track'
            / 'line_gt.csv'
        )
        self.path = load_gt_path(csv_path)
        if len(self.path) < 2:
            raise RuntimeError('Za mało punktów w line_gt.csv')

        # Długości kolejnych odcinków (raz na start).
        self.segment_lengths = []
        for i in range(len(self.path) - 1):
            x0, y0 = self.path[i]
            x1, y1 = self.path[i + 1]
            self.segment_lengths.append(math.hypot(x1 - x0, y1 - y0))

        self.total_path_length = path_length(self.path)
        self.start_x, self.start_y = self.path[0]

        ########### BUFER POMIARÓW #########
        self.lateral_errors = []  # błąd poprzeczny [m]
        self.speeds = []          # prędkość chwilowa [m/s]
        self.distance_traveled = 0.0
        self.prev_x = None
        self.prev_y = None

        self.max_progress_s = 0.0  # najdalej wzdłuż toru
        self.started = False
        self.finished = False

        self.t0 = None  # czas startu pomiaru (po ruszeniu)
        self.sample_count = 0

        ########### ROS: SUBSKRYPCJA I TIMER #########
        odom_topic = self.get_parameter('odom_topic').value
        self.create_subscription(Odometry, odom_topic, self.on_odom, 10)
        self.create_timer(report_period, self.on_report_timer)

        self.get_logger().info(
            f'Ewaluacja start ({self.controller_name}): '
            f'GT={csv_path.name}, długość toru={self.total_path_length:.1f} m, '
            f'temat={odom_topic}, raport co {report_period:.0f} s, max 1 okrążenie'
        )

    ########### CALLBACK ODOMETRII GT #########
    def on_odom(self, msg):
        if self.finished:
            return

        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        # Prędkość z twist (jeśli pusta, licz z pozycji).
        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y
        speed = math.hypot(vx, vy)

        # Dystans przebyty (łamana trajektoria).
        if self.prev_x is not None:
            step = math.hypot(x - self.prev_x, y - self.prev_y)
            self.distance_traveled += step
            # Gdy twist ~0, szacuj prędkość z przesunięcia (opcjonalnie pomijamy).
            if speed < 1e-6 and step > 0.0:
                # Bez dt nie liczymy prędkości z kroku — zostaw 0.
                pass
        self.prev_x = x
        self.prev_y = y

        # Błąd poprzeczny i postęp wzdłuż GT.
        e_lat, s = lateral_error_and_progress(
            x, y, self.path, self.segment_lengths
        )
        if s > self.max_progress_s:
            self.max_progress_s = s

        dist_to_start = math.hypot(x - self.start_x, y - self.start_y)

        # Start pomiaru dopiero po odjechaniu od linii startu.
        if not self.started:
            if dist_to_start >= self.start_move_distance:
                self.started = True
                self.t0 = self.get_clock().now()
                self.get_logger().info('Pomiar rozpoczęty (pojazd odjechał od startu).')
            else:
                return

        self.lateral_errors.append(e_lat)
        self.speeds.append(speed)
        self.sample_count += 1

        # Koniec okrążenia: dużo postępu wzdłuż toru + powrót blisko startu.
        progress_ok = self.max_progress_s >= (
            self.min_progress_ratio * self.total_path_length
        )
        near_start = dist_to_start <= self.lap_finish_radius
        if progress_ok and near_start:
            self.finished = True
            self.get_logger().info('Wykryto koniec okrążenia.')
            self.print_final_report()

    ########### RAPORT CO 10 S #########
    def on_report_timer(self):
        if self.finished:
            return
        if not self.started or not self.lateral_errors:
            self.get_logger().info(
                f'[{self.controller_name}] czekam na ruch / dane z odom_gt...'
            )
            return
        self.print_partial_report()

    def elapsed_seconds(self):
        if self.t0 is None:
            return 0.0
        return (self.get_clock().now() - self.t0).nanoseconds / 1e9

    def print_partial_report(self):
        e = self.lateral_errors
        v = self.speeds
        self.get_logger().info(
            '---------- raport okresowy ----------\n'
            f'  controller     : {self.controller_name}\n'
            f'  czas [s]       : {self.elapsed_seconds():.1f}\n'
            f'  próbki         : {self.sample_count}\n'
            f'  CTE RMSE [m]   : {rmse(e):.4f}\n'
            f'  CTE mean [m]   : {mean(e):.4f}\n'
            f'  CTE max [m]    : {max(e):.4f}\n'
            f'  CTE min [m]    : {min(e):.4f}\n'
            f'  v mean [m/s]   : {mean(v):.3f}\n'
            f'  v max [m/s]    : {max(v):.3f}\n'
            f'  v min [m/s]    : {min(v):.3f}\n'
            f'  dystans [m]    : {self.distance_traveled:.2f}\n'
            f'  postęp toru [m]: {self.max_progress_s:.2f} / {self.total_path_length:.2f}\n'
            f'------------------------------------'
        )

    ########### RAPORT KOŃCOWY (1 OKRĄŻENIE) #########
    def print_final_report(self):
        e = self.lateral_errors
        v = self.speeds
        t = self.elapsed_seconds()
        avg_v = self.distance_traveled / t if t > 1e-6 else 0.0

        self.get_logger().info(
            '========== RAPORT KOŃCOWY (1 okrążenie) ==========\n'
            f'  controller           : {self.controller_name}\n'
            f'  czas okrążenia [s]   : {t:.2f}\n'
            f'  próbki               : {self.sample_count}\n'
            f'  -- błąd poprzeczny (CTE) --\n'
            f'  RMSE [m]             : {rmse(e):.4f}\n'
            f'  mean [m]             : {mean(e):.4f}\n'
            f'  max [m]              : {max(e) if e else 0.0:.4f}\n'
            f'  min [m]              : {min(e) if e else 0.0:.4f}\n'
            f'  -- analiza szybkościowa --\n'
            f'  v mean (z twist)     : {mean(v):.3f} m/s\n'
            f'  v max                : {max(v) if v else 0.0:.3f} m/s\n'
            f'  v min                : {min(v) if v else 0.0:.3f} m/s\n'
            f'  dystans trajektorii  : {self.distance_traveled:.2f} m\n'
            f'  v średnia (s/t)      : {avg_v:.3f} m/s\n'
            f'  długość GT           : {self.total_path_length:.2f} m\n'
            f'  max postęp na GT     : {self.max_progress_s:.2f} m\n'
            f'=================================================='
        )


########### MAIN #########
def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = EvaluateRun()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None and not node.finished and node.lateral_errors:
            node.get_logger().info('Przerwano przed końcem okrążenia — metryki częściowe:')
            node.print_final_report()
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
