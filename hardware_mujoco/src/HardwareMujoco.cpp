//
// Created by biao on 24-9-9.
//

#include "hardware_mujoco/HardwareMujoco.h"
#include <rclcpp/logging.hpp>

#define TOPIC_LOWCMD "rt/lowcmd"
#define TOPIC_LOWSTATE "rt/lowstate"

using hardware_interface::return_type;

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn HardwareMujoco::on_init(
    const hardware_interface::HardwareInfo &info)
{
    if (SystemInterface::on_init(info) != CallbackReturn::SUCCESS)
    {
        return CallbackReturn::ERROR;
    }

    for (size_t i = 0; i < info.joints.size(); i++)
    {
        joint_position_states_[info.joints[i].name] = 0.0;
        joint_velocity_states_[info.joints[i].name] = 0.0;
        joint_effort_states_[info.joints[i].name] = 0.0;
        joint_position_commands_[info.joints[i].name] = 0.0;
        joint_velocity_commands_[info.joints[i].name] = 0.0;
        joint_effort_commands_[info.joints[i].name] = 0.0;
        joint_kp_commands_[info.joints[i].name] = 0.0;
        joint_kd_commands_[info.joints[i].name] = 0.0;
    }
    imu_states_.resize(info.sensors[0].state_interfaces.size(), 0);
    // foot_contact_states_.resize(info.sensors[1].state_interfaces.size(), 0);

    // 初始化硬件命令的默认状态
    MotorCmd.mode = 1;
    MotorCmd.order.motor_enable = 0; // 初始状态下电机不使能
    MotorCmd.order.motor_disenable = 0;
    MotorCmd.order.motor_power_supply = 0; // 初始状态下电源关闭
    MotorCmd.order.motor_charging_electrodes = 0;
    MotorCmd.order.open_light = 0;
    MotorCmd.order.state_light = 0;

    auto qos = rclcpp::QoS(rclcpp::KeepLast(1), rmw_qos_profile_sensor_data);

    node_ = rclcpp::Node::make_shared("ros2_control_mujoco");
    // subscription
    joint_state_subscriber_ = node_->create_subscription<custom_msgs::msg::LowState>(
        "joint_sensor_data", qos, std::bind(&HardwareMujoco::joint_state_callback, this, std::placeholders::_1));
    imu_subscriber_ = node_->create_subscription<sensor_msgs::msg::Imu>(
        "imu_sensor_data", qos, std::bind(&HardwareMujoco::imu_callback, this, std::placeholders::_1));
    // foot_contact_state_subscriber_ = node_->create_subscription<custom_msgs::msg::MujocoMsg>(
    //     "mujoco_msg", rclcpp::SensorDataQoS(), std::bind(&HardwareMujoco::foot_contact_callback, this, std::placeholders::_1));

    // publish
    low_cmd_publisher_ = node_->create_publisher<custom_msgs::msg::LowCmd>("low_cmds", qos);
    user_cmds_subscriber_ = node_->create_subscription<custom_msgs::msg::UserCmds>(
        "user_cmd", qos, std::bind(&HardwareMujoco::user_cmd_callback, this, std::placeholders::_1));
    return SystemInterface::on_init(info);
}

std::vector<hardware_interface::StateInterface> HardwareMujoco::export_state_interfaces()
{
    std::vector<hardware_interface::StateInterface> state_interfaces;
    state_interfaces.reserve(info_.joints.size() * 3 + info_.sensors.size() * 2);

    // joint state
    for (size_t i = 0; i < info_.joints.size(); i++)
    {
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            info_.joints[i].name, "position", &joint_position_states_[info_.joints[i].name]));
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            info_.joints[i].name, "velocity", &joint_velocity_states_[info_.joints[i].name]));
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            info_.joints[i].name, "effort", &joint_effort_states_[info_.joints[i].name]));
    }

    // imu sensor
    for (size_t i = 0; i < info_.sensors[0].state_interfaces.size(); i++)
    {
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            info_.sensors[0].name, info_.sensors[0].state_interfaces[i].name, &imu_states_[i]));
    }

    return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> HardwareMujoco::export_command_interfaces()
{
    std::vector<hardware_interface::CommandInterface> command_interfaces;
    command_interfaces.reserve(info_.joints.size() * 5);

    for (size_t i = 0; i < info_.joints.size(); i++)
    {
        command_interfaces.emplace_back(hardware_interface::CommandInterface(
            info_.joints[i].name, "position", &joint_position_commands_[info_.joints[i].name]));
        command_interfaces.emplace_back(hardware_interface::CommandInterface(
            info_.joints[i].name, "velocity", &joint_velocity_commands_[info_.joints[i].name]));
        command_interfaces.emplace_back(hardware_interface::CommandInterface(
            info_.joints[i].name, "effort", &joint_effort_commands_[info_.joints[i].name]));
        command_interfaces.emplace_back(hardware_interface::CommandInterface(
            info_.joints[i].name, "kp", &joint_kp_commands_[info_.joints[i].name]));
        command_interfaces.emplace_back(hardware_interface::CommandInterface(
            info_.joints[i].name, "kd", &joint_kd_commands_[info_.joints[i].name]));
    }
    return command_interfaces;
}

return_type HardwareMujoco::read(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{

    if (rclcpp::ok())
    {
        rclcpp::spin_some(node_);
    }

    return return_type::OK;
}

return_type HardwareMujoco::write(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
    std::vector<std::string> joint_names = {
        "FL_abd_joint",
        "FL_hip_joint",
        "FL_knee_joint",
        "FR_abd_joint",
        "FR_hip_joint",
        "FR_knee_joint",
        "HL_abd_joint",
        "HL_hip_joint",
        "HL_knee_joint",
        "HR_abd_joint",
        "HR_hip_joint",
        "HR_knee_joint",
    };
     // 使用映射数组来定义电机索引与关节索引的对应关系
    const int motor_indices[] = {0, 1, 2, 4, 5, 6, 8, 9, 10, 12, 13, 14};

    if (controller_flag == 8)
    {
        for (size_t i = 0; i < 12; i++)
        {
            MotorCmd.motor_cmd[motor_indices[i]].position = joint_position_commands_[joint_names[i]];
            MotorCmd.motor_cmd[motor_indices[i]].velocity = joint_velocity_commands_[joint_names[i]];
            MotorCmd.motor_cmd[motor_indices[i]].torque = joint_effort_commands_[joint_names[i]];
            MotorCmd.motor_cmd[motor_indices[i]].kp = joint_kp_commands_[joint_names[i]];
            MotorCmd.motor_cmd[motor_indices[i]].kd = joint_kd_commands_[joint_names[i]];
        }
    }
    else
    {
        for (size_t i = 0; i < 12; i++)
        {
            MotorCmd.motor_cmd[motor_indices[i]].torque = 0;
            MotorCmd.motor_cmd[motor_indices[i]].position = 0;

            MotorCmd.motor_cmd[motor_indices[i]].velocity = 0;
            MotorCmd.motor_cmd[motor_indices[i]].kp = 0;
            MotorCmd.motor_cmd[motor_indices[i]].kd = 0;
        }
    }

    low_cmd_publisher_->publish(MotorCmd);

    // RCLCPP_INFO(node_->get_logger(), "Torque: %.3f", actuator_cmds.torque[0]);
    return return_type::OK;
}

void HardwareMujoco::user_cmd_callback(const custom_msgs::msg::UserCmds user_msg)
{
    custom_msgs::msg::UserCmds UserCmd = user_msg;
    if (UserCmd.motor_controller == 9)
    {
        MotorCmd.order.motor_power_supply = 1;
    }

    if (UserCmd.motor_controller == 5)
    {
        MotorCmd.order.motor_enable = 1;
        controller_flag = 5;
    }

    if (UserCmd.motor_controller == 8)
    {
        MotorCmd.mode = 1;
        controller_flag = 8;
    }

    if (UserCmd.motor_controller == 10)
    {
        controller_flag = 0;
        MotorCmd.mode = 0;
        MotorCmd.order.motor_power_supply = 0;
    }
}

void HardwareMujoco::imu_callback(const sensor_msgs::msg::Imu imu_state)
{
    imu_states_[0] = imu_state.orientation.w;
    imu_states_[1] = imu_state.orientation.x;
    imu_states_[2] = imu_state.orientation.y;
    imu_states_[3] = imu_state.orientation.z;
    imu_states_[4] = imu_state.angular_velocity.x;
    imu_states_[5] = imu_state.angular_velocity.y;
    imu_states_[6] = imu_state.angular_velocity.z;
    imu_states_[7] = imu_state.linear_acceleration.x;
    imu_states_[8] = imu_state.linear_acceleration.y;
    imu_states_[9] = imu_state.linear_acceleration.z;
    // RCLCPP_INFO(node_->get_logger(), "imu_states_[0]: %.5f", imu_states_[0]);
}
void HardwareMujoco::joint_state_callback(const custom_msgs::msg::LowState joint_state)
{
    std::vector<std::string> joint_names = {
        "FL_abd_joint",
        "FL_hip_joint",
        "FL_knee_joint",
        "FR_abd_joint",
        "FR_hip_joint",
        "FR_knee_joint",
        "HL_abd_joint",
        "HL_hip_joint",
        "HL_knee_joint",
        "HR_abd_joint",
        "HR_hip_joint",
        "HR_knee_joint",
    };

    // 使用映射数组来定义电机索引与关节索引的对应关系
    const int motor_indices[] = {0, 1, 2, 4, 5, 6, 8, 9, 10, 12, 13, 14};
    
    // 使用循环来处理所有关节的状态赋值
    for(size_t i = 0; i < 12; ++i) {
        joint_position_states_[joint_names[i]] = joint_state.motor_state[motor_indices[i]].position;
        joint_velocity_states_[joint_names[i]] = joint_state.motor_state[motor_indices[i]].velocity;
        joint_effort_states_[joint_names[i]] = joint_state.motor_state[motor_indices[i]].torque;
    }
    // RCLCPP_INFO(node_->get_logger(), "joint_position_states[0]_: %.5f",  joint_position_states_[joint_names[0]]);
}

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(HardwareMujoco, hardware_interface::SystemInterface)