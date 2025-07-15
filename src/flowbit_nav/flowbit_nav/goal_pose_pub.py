#!/usr/bin/env python3
"""
Minimal Nav2 action client that tells the robot to drive to a pose.
Usage:
  ros2 run flowbit_nav goto --ros-args -p x:=1.0 -p y:=0.5 -p yaw:=1.57
"""
import math, rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from typing import Tuple, Callable


def quat_from_yaw(yaw):
    return (0.0, 0.0, math.sin(yaw/2.0), math.cos(yaw/2.0))


class GoTo(Node):
    def __init__(self):
        super().__init__('go_to')
        self.declare_parameter('x', 0.0)
        self.declare_parameter('y', 0.0)
        self.declare_parameter('z', 0.0)
        self.declare_parameter('yaw', 0.0)

        self._ac = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self._timer = self.create_timer(0.1, self._send_goal)

    ################
    def create_world_to_robot_converter(self,
            world_tracker: Tuple[float, float, float],
            robot_home: Tuple[float, float, float]
        ) -> Callable[[Tuple[float, float, float]], Tuple[float, float, float]]:
            """
            Creates a converter function with fixed world_tracker and robot_home.
            """
            x_wt, y_wt, z_wt = world_tracker
            x_rh, y_rh, z_rh = robot_home

            def convert(world_interest: Tuple[float, float, float]) -> Tuple[float, float, float]:
                x_wi, y_wi, z_wi = world_interest
                dx = x_wi - x_wt
                dy = y_wi - y_wt
                dz = z_wi - z_wt

                x_ri = x_rh - dy         # World X becomes -robot X
                y_ri = y_rh + dx         # World Z becomes robot Y
                z_ri = z_rh + dz         # World Y becomes robot Z

                return (x_ri, y_ri, z_ri)

            return convert
    ###################
    
    
    
    
    # ---------------------------------------------------------
    def _send_goal(self):
        if not self._ac.wait_for_server(timeout_sec=0.1):
            return                                  # keep waiting
        self._timer.cancel()

        x   = self.get_parameter('x').get_parameter_value().double_value
        y   = self.get_parameter('y').get_parameter_value().double_value
        z   = self.get_parameter('z').get_parameter_value().double_value
        yaw = self.get_parameter('yaw').get_parameter_value().double_value

        WORLD_TRACKER = (0.780949, 2.574092, 0.2400917)
        ROBOT_HOME = (-0.661,-0.949,0.0277)
        convert_point = self.create_world_to_robot_converter(WORLD_TRACKER, ROBOT_HOME)
        world_interest = (z,x,y)
        print("After resort: ")
        print(world_interest)
        x, y, z = convert_point(world_interest)

        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp    = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        qx, qy, qz, qw = quat_from_yaw(yaw)
        goal.pose.pose.orientation.x = qx
        goal.pose.pose.orientation.y = qy
        goal.pose.pose.orientation.z = qz
        goal.pose.pose.orientation.w = qw

        self.get_logger().info(f'Sending NavigateToPose {{x:{x}, y:{y}, yaw:{yaw}}}')
        send_future = self._ac.send_goal_async(goal)
        send_future.add_done_callback(self._on_goal_accepted)

    def _on_goal_accepted(self, future):
        # We get the result when the goal is finished
        result = future.result()
        self.get_logger().info(f'Goal result: {result.status}')
        rclpy.shutdown()  # Shutdown the node after the goal is accepted


def main(args=None):
    rclpy.init(args=args)
    node = GoTo()
    rclpy.spin(node)
    node.destroy_node()
    # rclpy.shutdown()


if __name__ == '__main__':
    main()