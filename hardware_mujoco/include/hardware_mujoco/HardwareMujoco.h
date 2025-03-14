//
// Created by biao on 24-9-9.
//

#pragma once

#include "hardware_interface/system_interface.hpp"
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include "custom_msgs/msg/actuator_cmds.hpp"
#include "custom_msgs/msg/mujoco_msg.hpp"
#include "custom_msgs/msg/low_cmd.hpp"
#include "custom_msgs/msg/low_state.hpp"
#include <custom_msgs/msg/motor_cmd.hpp>
#include <custom_msgs/msg/user_cmds.hpp>

#include "eigen3/Eigen/Dense"
using namespace Eigen;

class HardwareMujoco final : public hardware_interface::SystemInterface
{
public:
    CallbackReturn on_init(const hardware_interface::HardwareInfo &info) override;

    std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

    std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

    hardware_interface::return_type read(const rclcpp::Time &time, const rclcpp::Duration &period) override;

    hardware_interface::return_type write(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) override;

protected:
    int controller_flag = 0;
    void imu_callback(const sensor_msgs::msg::Imu imu_state);
    void joint_state_callback(const custom_msgs::msg::LowState joint_state);
    void user_cmd_callback(const custom_msgs::msg::UserCmds user_msg);

    // cmd
    std::unordered_map<std::string, double> joint_position_commands_;
    std::unordered_map<std::string, double> joint_velocity_commands_;
    std::unordered_map<std::string, double> joint_effort_commands_;
    std::unordered_map<std::string, double> joint_kp_commands_;
    std::unordered_map<std::string, double> joint_kd_commands_;

    // state
    std::unordered_map<std::string, double> joint_position_states_;
    std::unordered_map<std::string, double> joint_velocity_states_;
    std::unordered_map<std::string, double> joint_effort_states_;
    std::vector<double> imu_states_;
    // std::vector<double> foot_contact_states_;

    /*node*/
    rclcpp::Node::SharedPtr node_;
    /*publisher*/
    rclcpp::Publisher<custom_msgs::msg::LowCmd>::SharedPtr low_cmd_publisher_;

    /*subscriber*/
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscriber_;
    rclcpp::Subscription<custom_msgs::msg::LowState>::SharedPtr joint_state_subscriber_;
    // rclcpp::Subscription<custom_msgs::msg::MujocoMsg>::SharedPtr foot_contact_state_subscriber_;

    rclcpp::Subscription<custom_msgs::msg::UserCmds>::SharedPtr user_cmds_subscriber_;

    // 底层1ms定时器
    rclcpp::TimerBase::SharedPtr timer_;

    // 电机下层指令
    custom_msgs::msg::LowCmd MotorCmd;
};
