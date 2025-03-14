#pragma once

#include "rclcpp/rclcpp.hpp"
#include "robot_hardware/imu/imu_yis320.h"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "robot_hardware/motor_driver/motor_driver.h"

// #include "custom_msgs/msg/actuator_cmds.hpp"

class RobotHardwareNode : public rclcpp::Node
{
public:
    RobotHardwareNode();
    ~RobotHardwareNode();

    // void sensor_pub_callback();
    void imu_pub_callback();
    void joint_pub_callback();
    void joint_sub_callback(const custom_msgs::msg::LowCmd::SharedPtr cmd);

private:
    // 硬件设备
    std::shared_ptr<ImuYIS320> imu_;            // 智能指针，指向IMU传感器设备
    std::shared_ptr<MotorDriver> motor_driver_; // 智能指针，指向电机驱动器设备

    // 发布
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
    rclcpp::Publisher<custom_msgs::msg::LowState>::SharedPtr joint_pub_;

    // 订阅
    rclcpp::Subscription<custom_msgs::msg::LowCmd>::SharedPtr joint_sub_;

    // 定时器
    rclcpp::TimerBase::SharedPtr timer_imu_;
    rclcpp::TimerBase::SharedPtr timer_joint_;
};
