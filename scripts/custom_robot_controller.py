#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

class CustomRobotControllerNode(Node):


    def process_LaserScan(self,data):

        twist_msg = Twist()

        #########################################
        #
        # Write your code here
        #
        # The values from the LiDAR are in data.ranges
        # For more information on the data contained in a LaserScan message, consult http://docs.ros.org/en/api/sensor_msgs/html/msg/LaserScan.html
        #
        # The following are used to set the linear and angular velocities of the robot
        # twist_msg.linear.x = 0;
        # twist_msg.angular.z = 0;
        ########################################

        self.twist_pub.publish(twist_msg)
        


    def __init__(self):
        super().__init__("custom_robot_controller")

        self.scan_topic = "/simple_lidar"   # uncomment only one of these lines at a time, depending on which LiDAR you wish to use
        #self.scan_topic = "/static_laser"   # "/static_laser" uses the complete LiDAR scan, "/simple_lidar" uses a simplified version

        self.twist_topic = "/cmd_vel"

        self.create_subscription(
                LaserScan,
                self.scan_topic,
                self.process_LaserScan)

        self.twist_pub = self.create_publisher(Twist, self.twist_topic)




if __name__ == '__main__':

    rclpy.init()
    custom_robot_controller = CustomRobotControllerNode()

    try:        
        rclpy.spin(custom_robot_controller)        

    except KeyboardInterrupt:
        pass

    rclpy.shutdown()