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

class HardwareMujoco final : public hardware_interface::SystemInterface
{
public:
    CallbackReturn on_init(const hardware_interface::HardwareInfo &info) override;

    std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

    std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

    hardware_interface::return_type read(const rclcpp::Time &time, const rclcpp::Duration &period) override;

    hardware_interface::return_type write(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) override;

protected:
    void imu_callback(const sensor_msgs::msg::Imu imu_state);
    void joint_state_callback(const sensor_msgs::msg::JointState joint_state);
    // void foot_contact_callback(const custom_msgs::msg::MujocoMsg foot_contact_state);

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
    rclcpp::Publisher<custom_msgs::msg::ActuatorCmds>::SharedPtr actuator_cmd_publisher_;
    /*subscriber*/
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscriber_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_subscriber_;
    // rclcpp::Subscription<custom_msgs::msg::MujocoMsg>::SharedPtr foot_contact_state_subscriber_;
};
