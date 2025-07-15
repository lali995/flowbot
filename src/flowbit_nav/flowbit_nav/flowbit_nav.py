#!/usr/bin/env python3

# fridge: -4.61959 0.0 -2.1053 #-4.01952 0.0 -1.50533
# water cooler: -3.14262, 0, 1.04056 #-3.1 0 0.54056
# server: -0.716533 0 -1.45803
# home: 2.02664 0.0117438 1.04818 #2.5 0.0117438 1.1187

 
#ros2 run flowbit_nav goal_pose_pub --ros-args -p x:=-4.61959 -p y:=0.0 -p z:=-2.1053 -p yaw:=1.57
"""
flowbit_nav.py  publishes a green sphere at a fixed pose so RViz can show it.
Run:  ros2 run flowbit_nav flowbit_nav -3.14262 0.0 1.04056 --frame map
"""
import argparse
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from visualization_msgs.msg import Marker
from typing import Tuple, Callable

class PointMarker(Node):
    
    def __init__(self, xyz, frame):
        super().__init__("flowbit_nav")
        
    
        self.xyz = xyz
        self.frame = frame
        self.pub = self.create_publisher(Marker, "visualization_marker", 1)
        self.timer = self.create_timer(0.1, self.publish_marker)  # 10 Hz



    def publish_marker(self):
        m = Marker()
        m.header.frame_id = self.frame
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns, m.id, m.type, m.action = "flowbit", 0, Marker.SPHERE, Marker.ADD
        m.pose.position.x, m.pose.position.y, m.pose.position.z = self.xyz
        m.pose.orientation.w = 1.0
        m.scale.x = m.scale.y = m.scale.z = 0.15          # 15 cm sphere
        m.color.r, m.color.g, m.color.b, m.color.a = 0.0, 1.0, 0.2, 0.9
        m.lifetime = Duration(seconds=0).to_msg()          # forever
        self.pub.publish(m)

################
def create_world_to_robot_converter(
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
def rotate_xyz_to_yzx(point):
    """
    Re-order a 3-tuple (x, y, z) to (y, z, x).

    Parameters
    ----------
    point : tuple[float, float, float]
        A tuple or list containing (x, y, z).

    Returns
    -------
    tuple[float, float, float]
        The reordered tuple (y, z, x).
    """
    x, y, z = point
    return (y, z, x)

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("x", type=float); ap.add_argument("y", type=float); ap.add_argument("z", type=float)
    ap.add_argument("--frame", default="map")
    args = ap.parse_args()
    rclpy.init()
    
    x_value = args.x
    y_value = args.y
    z_value = args.z

    world_interest = (z_value,x_value,y_value)

    WORLD_TRACKER = (0.780949, 2.574092, 0.2400917)
    ROBOT_HOME = (-0.661,-0.949,0.0277)
    convert_point = create_world_to_robot_converter(WORLD_TRACKER, ROBOT_HOME)
    
    #from mesh lab  to johnny (y,z,x)
    # -4.61959,0.0355222,-2.1053
    # (-2.1053, -4.61959, 0.0355222)
    # world_interest = (-2.1053, -4.61959, 0.0355222)#(1.93477, -1.31527, 0.145526)
# water cooler: -3.14262, 0, 1.04056

    robot_interest = convert_point(world_interest)
    print("TESTINGGG!")
    node = PointMarker((robot_interest[0], robot_interest[1], robot_interest[2]), args.frame)
    rclpy.spin(node)

if __name__ == "__main__":
    main()



######################################


######################################