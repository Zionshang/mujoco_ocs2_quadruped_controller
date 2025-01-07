//
// Created by tlab-uav on 24-9-30.
//

#include "ocs2_quadruped_controller/control/TargetManager.h"

#include <ocs2_core/misc/LoadData.h>
#include <ocs2_robotic_tools/common/RotationTransforms.h>

#include "ocs2_quadruped_controller/control/CtrlComponent.h"

namespace ocs2::legged_robot
{
    TargetManager::TargetManager(CtrlComponent &ctrl_component,
                                 const std::shared_ptr<ReferenceManagerInterface> &referenceManagerPtr,
                                 const std::string &task_file,
                                 const std::string &reference_file,
                                 rclcpp_lifecycle::LifecycleNode::SharedPtr node)
        : ctrl_component_(ctrl_component),
          referenceManagerPtr_(referenceManagerPtr),
          node_(std::move(node))
    {
        default_joint_state_ = vector_t::Zero(12);
        loadData::loadCppDataType(reference_file, "comHeight", command_height_);
        loadData::loadEigenMatrix(reference_file, "defaultJointState", default_joint_state_);
        loadData::loadCppDataType(task_file, "mpc.timeHorizon", time_to_target_);
        loadData::loadCppDataType(reference_file, "targetRotationVelocity", target_rotation_velocity_);
        loadData::loadCppDataType(reference_file, "targetDisplacementVelocity", target_displacement_velocity_);
        odom_pub_ = node_->create_publisher<nav_msgs::msg::Odometry>("reference_odom", 10);
        targetPose = vector_t::Zero(6);
    }

    void TargetManager::update(const rclcpp::Time &time, const rclcpp::Duration &period)
    {
        vector_t cmdGoal = vector_t::Zero(6);
        // cmdGoal is expressed in body frame
        cmdGoal[0] = ctrl_component_.control_inputs_.ly * target_displacement_velocity_;
        cmdGoal[1] = -ctrl_component_.control_inputs_.lx * target_displacement_velocity_;
        cmdGoal[2] = ctrl_component_.control_inputs_.ry;
        cmdGoal[3] = -ctrl_component_.control_inputs_.rx * target_rotation_velocity_;

        const vector_t currentPose = ctrl_component_.observation_.state.segment<6>(6);
        const Eigen::Matrix<scalar_t, 3, 1> zyx = currentPose.tail(3);
        // transform cmdGoal frome body frame to world frame
        vector_t cmd_vel_rot = getRotationMatrixFromZyxEulerAngles(zyx) * cmdGoal.head(3);

        // targetPose is expressed in world frame
        double time_step = period.seconds() / time_to_target_;
        targetPose(0) = targetPose(0) + cmd_vel_rot(0) * time_step; // x
        targetPose(1) = targetPose(1) + cmd_vel_rot(1) * time_step; // y
        targetPose(2) = command_height_;                            // z
        targetPose(3) = targetPose(3) + cmdGoal(3) * time_step;     // yaw
        targetPose(4) = 0;                                          // pitch
        targetPose(5) = 0;                                          // roll

        const scalar_t targetReachingTime = ctrl_component_.observation_.time + time_to_target_;
        auto trajectories = targetPoseToTargetTrajectories(targetPose, ctrl_component_.observation_, targetReachingTime);

        // ! the state in stateTrajectory is (vx, vy, vz, wz, wy, wx, x, y, z, yaw, pitch, roll, joint state)
        trajectories.stateTrajectory[0].head(3) = cmd_vel_rot;
        trajectories.stateTrajectory[1].head(3) = cmd_vel_rot;

        auto odom = getOdomMsg(trajectories);
        odom.header.stamp = time;
        odom.header.frame_id = "odom";
        odom.child_frame_id = "base";
        publishMsgs(odom);
        referenceManagerPtr_->setTargetTrajectories(std::move(trajectories));
    }

    TargetTrajectories TargetManager::targetPoseToTargetTrajectories(const vector_t &targetPose,
                                                                     const SystemObservation &observation,
                                                                     const scalar_t &targetReachingTime)
    {
        // desired time trajectory
        const scalar_array_t timeTrajectory{observation.time, targetReachingTime};

        // desired state trajectory
        vector_t currentPose = observation.state.segment<6>(6);
        // TODO: remove the restrictions on z, pitch, row
        currentPose(2) = command_height_;
        currentPose(4) = 0;
        currentPose(5) = 0;
        // TODO: remove the restrictions on zero velocity
        vector_array_t stateTrajectory(2, vector_t::Zero(observation.state.size()));
        stateTrajectory[0] << vector_t::Zero(6), currentPose, default_joint_state_;
        stateTrajectory[1] << vector_t::Zero(6), targetPose, default_joint_state_;

        // desired input trajectory (just right dimensions, they are not used)
        const vector_array_t inputTrajectory(2, vector_t::Zero(observation.input.size()));

        return {timeTrajectory, stateTrajectory, inputTrajectory};
    }

    nav_msgs::msg::Odometry TargetManager::getOdomMsg(const ocs2::TargetTrajectories &trajectories)
    {
        nav_msgs::msg::Odometry odom;

        odom.pose.pose.position.x = trajectories.stateTrajectory[1](6);
        odom.pose.pose.position.y = trajectories.stateTrajectory[1](7);
        odom.pose.pose.position.z = trajectories.stateTrajectory[1](8);

        const Eigen::Matrix<scalar_t, 3, 1> zyx = trajectories.stateTrajectory[1].segment(9, 3);
        Eigen::Quaternion<scalar_t> quat = getQuaternionFromEulerAnglesZyx(zyx);
        odom.pose.pose.orientation.x = quat.x();
        odom.pose.pose.orientation.y = quat.y();
        odom.pose.pose.orientation.z = quat.z();
        odom.pose.pose.orientation.w = quat.w();
        odom.pose.pose.orientation.x = quat.x();

        //  The twist in this message should be specified in the coordinate frame given by the child_frame_id: "base"
        vector_t twist = getRotationMatrixFromZyxEulerAngles(zyx).transpose() * trajectories.stateTrajectory[1].head(6);

        odom.twist.twist.linear.x = twist(0);
        odom.twist.twist.linear.y = twist(1);
        odom.twist.twist.linear.z = twist(2);

        odom.twist.twist.angular.x = twist(3);
        odom.twist.twist.angular.y = twist(4);
        odom.twist.twist.angular.z = twist(5);
        return odom;
    }

    void TargetManager::publishMsgs(const nav_msgs::msg::Odometry &odom) const
    {
        rclcpp::Time time = odom.header.stamp;
        odom_pub_->publish(odom);
    }
}
