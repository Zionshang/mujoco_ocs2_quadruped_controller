#include "rclcpp/rclcpp.hpp"
#include "robot_hardware/RobotHardwareNode.h"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RobotHardwareNode>());
    rclcpp::shutdown();
    
    return 0;
}