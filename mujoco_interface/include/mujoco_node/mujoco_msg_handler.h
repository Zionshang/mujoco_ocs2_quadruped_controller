//
// Created by lbt on 24-12-6.
//
// #include <geometry_msgs/msg/transform_stamped.hpp>
// #include <nav_msgs/msg/odometry.hpp>
//
//
// #include <sensor_msgs/msg/image.hpp>
// #include <sensor_msgs/msg/imu.hpp>
// #include <sensor_msgs/msg/joint_state.hpp>
// #include <tf2/LinearMath/Quaternion.h>
// #include <tf2_ros/transform_broadcaster.h>

#include <rclcpp/rclcpp.hpp>
#include <rmw/types.h>
#include "array_safety.h"
#include "simulate.h"
#include "custom_msgs/msg/sensor_msg.h"
#include "custom_msgs/msg/actuator_cmds.hpp"
#include "custom_msgs/msg/mujoco_msg.hpp"
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <vector>

using namespace rclcpp;

using namespace std::chrono_literals;

namespace Galileo
{
    namespace mj = ::mujoco;
    namespace mju = ::mujoco::sample_util;

    class MujocoMsgHandler : public rclcpp::Node
    {
    public:
        const std::string xml_file_path() { return xml_file_path_; }

        struct ActuatorCmds
        {
            double time = 0.0;
            std::vector<std::string> actuators_name;
            std::vector<float> kp;
            std::vector<float> pos;
            std::vector<float> kd;
            std::vector<float> vel;
            std::vector<float> torque;
        };

        MujocoMsgHandler(mj::Simulate *sim);
        ~MujocoMsgHandler();

    private:
        void publish_mujoco_callback();
        void imu_callback();
        void contact_callback();
        void joint_callback();
        void actuator_cmd_callback(const custom_msgs::msg::ActuatorCmds::SharedPtr msg) const;

        mj::Simulate *sim_;
        std::vector<rclcpp::TimerBase::SharedPtr> timers_;
        rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_publisher_;
        rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_publisher_;
        rclcpp::Publisher<custom_msgs::msg::MujocoMsg>::SharedPtr mujoco_msg_publisher_;
        rclcpp::Subscription<custom_msgs::msg::ActuatorCmds>::SharedPtr actuator_cmd_subscription_;
        std::shared_ptr<rclcpp::ParameterEventHandler> param_subscriber_;
        std::shared_ptr<rclcpp::ParameterCallbackHandle> cb_handle_;
        std::string xml_file_path_;
    };
} // Galileo
