#pragma once

#include "custom_msgs/msg/low_cmd.hpp"
#include "custom_msgs/msg/low_state.hpp"
#include "custom_msgs/msg/motor_cmd.hpp"
#include "custom_msgs/msg/motor_state.hpp"
#include "robot_hardware/motor_driver/interface.h"
#include "robot_hardware/motor_driver/udp.h"
#include "sensor_msgs/msg/joint_state.hpp"
class MotorDriver
{
public:
    MotorDriver();
    ~MotorDriver();
    custom_msgs::msg::LowState joint_pub_callback();
    void joint_sub_callback(const custom_msgs::msg::LowCmd::SharedPtr msg);

private:
    MotorCmd Convert(const custom_msgs::msg::MotorCmd& msg);
    custom_msgs::msg::MotorState Convert(const MotorState& data);

    LowState low_state;
    LowCmd low_cmd;

    void LowPdCtrler();

    // 单模块通信测试
    Interface interface;
};
