//
// Created by qiayuan on 2021/11/15.
//

#include "ocs2_quadruped_controller/estimator/StateEstimateBase.h"

#include <ocs2_centroidal_model/FactoryFunctions.h>
#include <ocs2_legged_robot/common/Types.h>
#include <ocs2_robotic_tools/common/RotationDerivativesTransforms.h>

#include <memory>
#include "ocs2_quadruped_controller/control/CtrlComponent.h"

namespace ocs2::legged_robot
{

    StateEstimateBase::StateEstimateBase(PinocchioInterface pinocchio_interface, CentroidalModelInfo info,
                                         const PinocchioEndEffectorKinematics &ee_kinematics,
                                         CtrlComponent &ctrl_component,
                                         rclcpp_lifecycle::LifecycleNode::SharedPtr node)
        : ctrl_component_(ctrl_component),
          pinocchio_interface_(std::move(pinocchio_interface)),
          info_(std::move(info)),
          ee_kinematics_(ee_kinematics.clone()),
          rbd_state_(vector_t::Zero(2 * info_.generalizedCoordinatesNum)), node_(std::move(node))
    {
        odom_pub_ = node_->create_publisher<nav_msgs::msg::Odometry>("odom", 10);
        pose_pub_ = node_->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("pose", 10);
    }

    void StateEstimateBase::updateJointStates()
    {
        const size_t size = ctrl_component_.joint_effort_state_interface_.size();
        vector_t joint_pos(size), joint_vel(size);

        for (int i = 0; i < size; i++)
        {
            joint_pos(i) = ctrl_component_.joint_position_state_interface_[i].get().get_value();
            joint_vel(i) = ctrl_component_.joint_velocity_state_interface_[i].get().get_value();
        }

        rbd_state_.segment(6, info_.actuatedDofNum) = joint_pos;
        rbd_state_.segment(6 + info_.generalizedCoordinatesNum, info_.actuatedDofNum) = joint_vel;
    }

    void StateEstimateBase::updateContact(const contact_flag_t& contact_flag)
    {
        // const size_t size = ctrl_component_.foot_force_state_interface_.size();
        // for (int i = 0; i < size; i++)
        // {
        //     contact_flag_[i] = ctrl_component_.foot_force_state_interface_[i].get().get_value() > 0.1;
        // }
        contact_flag_ = contact_flag;
    }

    void StateEstimateBase::updateImu()
    {
        quat_ = {
            ctrl_component_.imu_state_interface_[0].get().get_value(),
            ctrl_component_.imu_state_interface_[1].get().get_value(),
            ctrl_component_.imu_state_interface_[2].get().get_value(),
            ctrl_component_.imu_state_interface_[3].get().get_value()};

        angular_vel_local_ = {
            ctrl_component_.imu_state_interface_[4].get().get_value(),
            ctrl_component_.imu_state_interface_[5].get().get_value(),
            ctrl_component_.imu_state_interface_[6].get().get_value()};

        linear_accel_local_ = {
            ctrl_component_.imu_state_interface_[7].get().get_value(),
            ctrl_component_.imu_state_interface_[8].get().get_value(),
            ctrl_component_.imu_state_interface_[9].get().get_value()};

        // orientationCovariance_ = orientationCovariance;
        // angularVelCovariance_ = angularVelCovariance;
        // linearAccelCovariance_ = linearAccelCovariance;

        const vector3_t zyx = quatToZyx(quat_) - zyx_offset_;
        const vector3_t angularVelGlobal = getGlobalAngularVelocityFromEulerAnglesZyxDerivatives<scalar_t>(
            zyx, getEulerAnglesZyxDerivativesFromLocalAngularVelocity<scalar_t>(quatToZyx(quat_), angular_vel_local_));
        updateAngular(zyx, angularVelGlobal);
    }

    void StateEstimateBase::updateAngular(const vector3_t &zyx, const vector_t &angularVel)
    {
        rbd_state_.segment<3>(0) = zyx;
        rbd_state_.segment<3>(info_.generalizedCoordinatesNum) = angularVel;
    }

    void StateEstimateBase::updateLinear(const vector_t &pos, const vector_t &linearVel)
    {
        rbd_state_.segment<3>(3) = pos;
        rbd_state_.segment<3>(info_.generalizedCoordinatesNum + 3) = linearVel;
    }

    void StateEstimateBase::publishMsgs(const nav_msgs::msg::Odometry &odom) const
    {
        rclcpp::Time time = odom.header.stamp;
        odom_pub_->publish(odom);

        geometry_msgs::msg::PoseWithCovarianceStamped pose;
        pose.header = odom.header;
        pose.pose.pose = odom.pose.pose;
        pose_pub_->publish(pose);
    }
} // namespace legged
