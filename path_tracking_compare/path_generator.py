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
