#!/usr/bin/env python3
#ros2 run flowbit_nav initialpose_pub --ros-args -p x:=1.15 -p y:=0.4
import rclpy, time
from rclpy.node import Node
from rclpy.qos   import QoSProfile, HistoryPolicy, ReliabilityPolicy, DurabilityPolicy
from geometry_msgs.msg import PoseWithCovarianceStamped


class InitialPosePublisher(Node):
    def __init__(self):
        super().__init__('initialpose_publisher')

        # ─── Declare CLI parameters ──────────────────────────────────────────
        self.declare_parameter('x', -0.5010875463485718)
        self.declare_parameter('y', -0.7521085739135742)
        self.declare_parameter('z',  0.0)

        # Read them once
        x = self.get_parameter('x').get_parameter_value().double_value
        y = self.get_parameter('y').get_parameter_value().double_value
        z = self.get_parameter('z').get_parameter_value().double_value

        # ─── Set up latched publisher ────────────────────────────────────────
        qos = QoSProfile(history = HistoryPolicy.KEEP_LAST,
                         depth   = 1,
                         reliability = ReliabilityPolicy.RELIABLE,
                         durability  = DurabilityPolicy.TRANSIENT_LOCAL)
        pub = self.create_publisher(PoseWithCovarianceStamped,
                                    '/initialpose', qos)

        # ─── Build message ───────────────────────────────────────────────────
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp    = self.get_clock().now().to_msg()

        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        msg.pose.pose.position.z = z

        # keep same heading as before
        msg.pose.pose.orientation.z = 0.01937931545470152
        msg.pose.pose.orientation.w =  0.999812203432478

        msg.pose.covariance = [
            0.25, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.25, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 1e6, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 1e6, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 1e6, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.06853891909122467
        ]

        # ─── Wait for at least one subscriber (AMCL) ─────────────────────────
        deadline = time.time() + 5.0
        while pub.get_subscription_count() == 0 and time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)

        self.get_logger().info(
            f'Publishing initial pose at ({x:.3f}, {y:.3f}, {z:.3f})')
        pub.publish(msg)

        # leave node alive 0.5 s so DDS flushes the sample
        rclpy.spin_once(self, timeout_sec=1.5)
        rclpy.shutdown()


def main():
    rclpy.init()
    InitialPosePublisher()


if __name__ == '__main__':
    main()
