"""Palyakoveto szabalyozo (Pure Pursuit)."""

import math

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry, Path
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.signals import SignalHandlerOptions


def yaw_from_quat(q):
    """Quaternionbol yaw szog."""
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class Controller(Node):

    def __init__(self):
        super().__init__('controller')

        self.declare_parameter('rate', 20.0)
        self.declare_parameter('speed', 1.0)
        self.declare_parameter('lookahead', 2.0)
        self.declare_parameter('max_omega', 2.0)
        rate = self.get_parameter('rate').value
        self.speed = self.get_parameter('speed').value
        self.lookahead = self.get_parameter('lookahead').value
        self.max_omega = self.get_parameter('max_omega').value

        self.path = []     # (x, y) pontok
        self.pose = None   # (x, y, yaw)
        self.nearest = None  # legkozelebbi palyapont indexe

        # a path_generator transient local-lal kuldi
        path_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(Path, '/path', self.path_callback, path_qos)
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.timer = self.create_timer(1.0 / rate, self.control_loop)
        self.get_logger().info(
            f'Pure Pursuit: v={self.speed} m/s, Ld={self.lookahead} m')

    def path_callback(self, msg):
        if not self.path:
            self.get_logger().info(f'Palya megjott, {len(msg.poses)} pont')
        self.path = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]

    def odom_callback(self, msg):
        p = msg.pose.pose
        self.pose = (p.position.x, p.position.y, yaw_from_quat(p.orientation))

    def find_nearest(self, x, y):
        n = len(self.path)
        if self.nearest is None:
            # elso korben az egesz palyat nezzuk
            candidates = range(n)
        else:
            # utana csak elore egy darabot, hogy a nyolcas
            # keresztezodesenel ne ugorjon at a masik agra
            candidates = [(self.nearest + k) % n for k in range(30)]
        best = min(candidates, key=lambda i: math.dist((x, y), self.path[i]))
        self.nearest = best
        return best

    def find_goal(self, x, y, start):
        n = len(self.path)
        for k in range(n):
            i = (start + k) % n
            if math.dist((x, y), self.path[i]) >= self.lookahead:
                return self.path[i]
        return self.path[start]

    def control_loop(self):
        if not self.path or self.pose is None:
            return
        x, y, yaw = self.pose

        start = self.find_nearest(x, y)
        gx, gy = self.find_goal(x, y, start)

        # celpont a robot koordinata-rendszereben
        dx = gx - x
        dy = gy - y
        x_r = math.cos(yaw) * dx + math.sin(yaw) * dy
        y_r = -math.sin(yaw) * dx + math.cos(yaw) * dy

        dist2 = x_r * x_r + y_r * y_r
        if dist2 < 1e-6:
            return
        curvature = 2.0 * y_r / dist2

        omega = self.speed * curvature
        omega = max(-self.max_omega, min(self.max_omega, omega))

        cmd = Twist()
        cmd.linear.x = self.speed
        cmd.angular.z = omega
        self.cmd_pub.publish(cmd)

    def stop(self):
        self.cmd_pub.publish(Twist())


def main(args=None):
    # sajat Ctrl+C kezeles, hogy leallaskor meg el tudjuk kuldeni a 0 sebesseget
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = Controller()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
