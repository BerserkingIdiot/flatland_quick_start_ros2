#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
using std::placeholders::_1;


class CustomRobotController : public rclcpp::Node
{
    public:
        CustomRobotController();

    private:
        void processLaserScan(const sensor_msgs::msg::LaserScan::SharedPtr lidar_scan_msg);
        //rclcpp::Node node;
        std::string scan_topic, twist_topic;
        rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub;
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_pub;

};

CustomRobotController::CustomRobotController():
    //scan_topic("/simple_lidar"), //uncomment only one of these lines at a time, depending on which LiDAR you wish to use
    scan_topic("/static_laser"), //"/static_laser" uses the complete LiDAR scan, "/simple_lidar" uses a simplified version
    twist_topic("/cmd_vel"),
    Node("custom_robot_controller")
{
    //node = rclcpp::Node::make_shared("custom_robot_controller");
    scan_sub = this->create_subscription<sensor_msgs::msg::LaserScan>(scan_topic, 1, std::bind(&CustomRobotController::processLaserScan, this, _1));
    twist_pub = this->create_publisher<geometry_msgs::msg::Twist>(twist_topic, 1);
}

void CustomRobotController::processLaserScan(const sensor_msgs::msg::LaserScan::SharedPtr lidar_scan_msg)
{

    geometry_msgs::msg::Twist twist_msg;

    /**
        Write your code here

        The values from the LiDAR are in lidar_scan_msg.ranges
        For more information on the data contained in a LaserScan message, consult http://docs.ros.org/en/api/sensor_msgs/html/msg/LaserScan.html

        The following are used to set the linear and angular velocities of the robot
        twist_msg.linear.x = 0;
        twist_msg.angular.z = 0;
     */

    twist_pub->publish(twist_msg);

}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  
  //CustomRobotController robot_controller;

  rclcpp::spin(std::make_shared<CustomRobotController>());

  rclcpp::shutdown();
  
  return(0);
}
