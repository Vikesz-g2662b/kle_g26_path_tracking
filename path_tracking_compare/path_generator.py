"""Palya generalas a /path topicra."""

import math

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile


def circle_points(radius, num_points):
    """Kor, origobol indul."""
    points = []
    for i in range(num_points + 1):
        t = 2.0 * math.pi * i / num_points
        x = radius * math.sin(t)
        y = radius - radius * math.cos(t)
        points.append((x, y))
    return points


def figure8_points(size, num_points):
    """Nyolcas."""
    points = []
    for i in range(num_points + 1):
        t = 2.0 * math.pi * i / num_points
        x = size * math.sin(t)
        y = size * math.sin(t) * math.cos(t)
        points.append((x, y))
    return points


# versenypalya kontrollpontjai, size-zal szorozva
TRACK_POINTS = [
    (0.0, 0.0), (0.9, 0.0), (1.5, 0.15), (1.9, 0.55), (2.0, 1.1),
    (1.75, 1.55), (1.25, 1.7), (0.75, 1.55), (0.2, 1.6), (-0.4, 1.85),
    (-1.0, 1.75), (-1.4, 1.3), (-1.45, 0.65), (-0.9, 0.05),
]


def track_points(size, num_points):
    """Zart Catmull-Rom spline a kontrollpontokon at."""
    ctrl = [(size * x, size * y) for x, y in TRACK_POINTS]
    n = len(ctrl)
    steps = max(1, num_points // n)
    points = []
    for i in range(n):
        p0 = ctrl[i - 1]
        p1 = ctrl[i]
        p2 = ctrl[(i + 1) % n]
        p3 = ctrl[(i + 2) % n]
        for k in range(steps):
            t = k / steps
            # Catmull-Rom keplet (wikipedia)
            xy = []
            for j in range(2):
                a = 2.0 * p1[j]
                b = (p2[j] - p0[j]) * t
                c = (2.0 * p0[j] - 5.0 * p1[j] + 4.0 * p2[j] - p3[j]) * t * t
                d = (-p0[j] + 3.0 * p1[j] - 3.0 * p2[j] + p3[j]) * t * t * t
                xy.append(0.5 * (a + b + c + d))
            points.append((xy[0], xy[1]))
    points.append(points[0])
    return points


class PathGenerator(Node):

    def __init__(self):
        super().__init__('path_generator')

        self.declare_parameter('path_type', 'circle')
        self.declare_parameter('size', 8.0)
        self.declare_parameter('num_points', 200)
        self.declare_parameter('frame_id', 'odom')

        path_type = self.get_parameter('path_type').value
        size = self.get_parameter('size').value
        num_points = self.get_parameter('num_points').value
        frame_id = self.get_parameter('frame_id').value

        if path_type == 'circle':
            points = circle_points(size, num_points)
        elif path_type == 'figure8':
            points = figure8_points(size, num_points)
        elif path_type == 'track':
            points = track_points(size, num_points)
        else:
            self.get_logger().error(f'Ismeretlen palya: {path_type}, kor lesz helyette')
            points = circle_points(size, num_points)

        self.path_msg = self.build_path(points, frame_id)

        # hogy kesobb indulo node is megkapja
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.publisher = self.create_publisher(Path, '/path', qos)
        self.timer = self.create_timer(1.0, self.publish_path)

        self.get_logger().info(
            f'Palya: {path_type}, meret: {size} m, pontok: {len(points)}')

    def build_path(self, points, frame_id):
        path = Path()
        path.header.frame_id = frame_id
        for i, (x, y) in enumerate(points):
            pose = PoseStamped()
            pose.header.frame_id = frame_id
            pose.pose.position.x = x
            pose.pose.position.y = y
            # irany a kovetkezo pont fele
            if i < len(points) - 1:
                nx, ny = points[i + 1]
                yaw = math.atan2(ny - y, nx - x)
            else:
                px, py = points[i - 1]
                yaw = math.atan2(y - py, x - px)
            pose.pose.orientation.z = math.sin(yaw / 2.0)
            pose.pose.orientation.w = math.cos(yaw / 2.0)
            path.poses.append(pose)
        return path

    def publish_path(self):
        self.path_msg.header.stamp = self.get_clock().now().to_msg()
        self.publisher.publish(self.path_msg)


def main(args=None):
    rclpy.init(args=args)
    node = PathGenerator()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
