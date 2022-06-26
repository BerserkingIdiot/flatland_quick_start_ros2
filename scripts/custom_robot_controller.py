#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import ReliabilityPolicy, QoSProfile
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
        # twist_msg.linear.x = 0.0
        # twist_msg.angular.z = 0.0
        ########################################
        
        ########################################
        ## Example 1: A robot that moves forward
        #twist_msg.linear.x = 1 # A positive lenear velocity makes the robot move forward
        #twist_msg.angular.z = 0 # Zero angular velocitiy prevents the robot from turning/rotating
        ########################################
        
        ########################################
        ## Example 2: A robot that moves in a circle
        #twist_msg.linear.x = 1 # A positive lenear velocity makes the robot move forward
        #twist_msg.angular.z = -1 # A negative angular velocitiy makes the robot turn clockwise
        ########################################
        
        ########################################
        ## Example 3: A robot that moves in circles, and turns around itself when it finds an obstacle in front of it
        ## Find if the laser in front of the robot is detecting an obstacle too close to them
        #num_lasers = lidar_scan_msg.ranges.size()
        #max_distance = 1.00 # maximum allowed distance for an obstacle in one of three sensor readings before the robot starts spining around itself
        #middle_laser_range = lidar_scan_msg.ranges[num_lasers / 2]
        #if( not (middle_laser_range >= lidar_scan_msg.range_min and middle_laser_range <= lidar_scan_msg.range_max) # if the distance reading is invalid or...
        #        or middle_laser_range < max_distance): # ... the distance is below the maximum allowed distance
        #   # ROTATE
        #    twist_msg.linear.x = 0
        #    twist_msg.angular.z = -2
        #else:
        #   # MOVE FORWARD
        #    twist_msg.linear.x = 2
        #    twist_msg.angular.z = -2
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
                self.process_LaserScan,
                QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))

        self.twist_pub = self.create_publisher(
                                Twist,
                                self.twist_topic,
                                QoSProfile(depth=10))




if __name__ == '__main__':

    rclpy.init()
    custom_robot_controller = CustomRobotControllerNode()

    try:        
        rclpy.spin(custom_robot_controller)        

    except KeyboardInterrupt:
        pass

    rclpy.shutdown()
