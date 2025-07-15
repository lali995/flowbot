import subprocess
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist

class DestinationController(Node):
    def __init__(self):
        super().__init__('destination_controller')


###
# fridge: -4.61959 0.0 -2.1053 #-4.01952 0.0 -1.50533
# water cooler: -3.14262, 0, 1.04056 #-3.1 0 0.54056
# server: -0.716533 0 -1.45803
# home: 2.02664 0.0117438 1.04818 #2.5 0.0117438 1.1187
###


        # Declare parameters for destinations
        self.destinations = {
            "fridge": (-4.01952, 0.0, -1.50533, 1.5),   
            "home": (2.5, 0.0117438, 1.1187, 1.5),      
            "server": (-0.716533, 0.0, -1.45803, 1.5),  
            "water": (-3.1, 0.0, 0.54056, 1.5)
        }

        # Publisher to stop the robot
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Stop the robot before moving to the new destination
        while(1): 
        # Take user input for destination
            self.get_user_input()
            print("Here5")
    def stop_robot(self):
        # Create a zero velocity command to stop the robot
        stop_msg = Twist()
        self.cmd_pub.publish(stop_msg)
        self.get_logger().info("Stopping the robot.")
        print("Here4")
    def get_user_input(self):
        # Ask for the destination input
        destination = input("Enter Destination (fridge, home, tools, server): ").strip().lower()
        
        # Validate input
        if destination in self.destinations:
            self.stop_robot()
            
            x, y, z, yaw = self.destinations[destination]
            self.get_logger().info(f"Moving to {destination}.")
            self.launch_goal_pose_pub(x, y, z, yaw)
        else:
            self.get_logger().warn("Invalid destination. Please choose from fridge, home, tools, or server.")
            self.get_user_input()  # Recurse until valid input

        print("Here2")
    def launch_goal_pose_pub(self, x, y, z, yaw):
        # Construct the command for launching the ROS 2 goal_pose_pub node
        cmd = f"ros2 run flowbit_nav goal_pose_pub --ros-args -p x:={x} -p y:={y} -p z:={z} -p yaw:={yaw}"
        
        # Launch the ROS node with the specified arguments
        subprocess.Popen(cmd, shell=True)
        self.get_logger().info(f"Sent goal to x: {x}, y: {y}, z: {z}, yaw: {yaw}")
        print("Here1")
def main(args=None):
    rclpy.init(args=args)
    print("Here6")
    node = DestinationController()
    print("Here7")
    rclpy.spin(node)
    print("Here8")
    rclpy.shutdown()
    print("Here9")

if __name__ == '__main__':
    main()