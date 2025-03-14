#include "robot_hardware/RobotHardwareNode.h"

using namespace std::chrono_literals;

RobotHardwareNode::RobotHardwareNode()
    : Node("robot_hardware_node", rclcpp::NodeOptions().use_intra_process_comms(true)),
      imu_(std::make_shared<ImuYIS320>("/dev/imu_yis130")), // 130
      motor_driver_(std::make_shared<MotorDriver>())

{
    // 创建一个QoS（Quality of Service）对象，用于配置消息的传输质量。
    auto qos = rclcpp::QoS(rclcpp::KeepLast(1), rmw_qos_profile_sensor_data);

    // 订阅控制器下发指令 rev
    joint_sub_ = this->create_subscription<custom_msgs::msg::LowCmd>(
        "low_cmds",
        qos,
        std::bind(&RobotHardwareNode::joint_sub_callback, this, std::placeholders::_1));

    // 发布关节传感器数据 rev
    joint_pub_ = this->create_publisher<custom_msgs::msg::LowState>("joint_sensor_data", qos);

    // imu发布话题 rev
    imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("imu_sensor_data", qos);

    timer_imu_ = this->create_wall_timer(2.5ms, std::bind(&RobotHardwareNode::imu_pub_callback, this));

    timer_joint_ = this->create_wall_timer(1ms, std::bind(&RobotHardwareNode::joint_pub_callback, this));

    RCLCPP_INFO(this->get_logger(), "RobotHardwareNode init");
}

RobotHardwareNode::~RobotHardwareNode()
{
}

void RobotHardwareNode::joint_sub_callback(const custom_msgs::msg::LowCmd::SharedPtr cmd)
{
    motor_driver_->joint_sub_callback(cmd);
    // RCLCPP_INFO(this->get_logger(), "RobotHardwareNode joint_callback");
}

void RobotHardwareNode::joint_pub_callback()
{
    custom_msgs::msg::LowState joint_state_msg;
    // RCLCPP_INFO(this->get_logger(), "joint run");

    joint_state_msg = motor_driver_->joint_pub_callback();

    joint_pub_->publish(joint_state_msg);
}

void RobotHardwareNode::imu_pub_callback()
{
    sensor_msgs::msg::Imu imu_msg;

    // RCLCPP_INFO(this->get_logger(), "imu run");

    // 下面和ros1实现的一致，封装在：imu_ 的 imu_pub_callback
    imu_msg = imu_->imu_pub_callback();

    imu_pub_->publish(imu_msg);

    // RCLCPP_INFO(this->get_logger(),
    //             "RobotHardwareNode run w:%.3f  x:%.3f  y:%.3f  z:%.3f",
    //             imu_msg.orientation.w,
    //             imu_msg.orientation.x,
    //             imu_msg.orientation.y,
    //             imu_msg.orientation.z);
}