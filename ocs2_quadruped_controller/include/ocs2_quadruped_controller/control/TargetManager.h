//
// Created by tlab-uav on 24-9-30.
//

#ifndef TARGETMANAGER_H
#define TARGETMANAGER_H

#include <memory>
#include <ocs2_mpc/SystemObservation.h>
#include <ocs2_oc/synchronized_module/ReferenceManagerInterface.h>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

struct CtrlComponent;

namespace ocs2::legged_robot
{
    class TargetManager
    {
    public:
        TargetManager(CtrlComponent &ctrl_component,
                      const std::shared_ptr<ReferenceManagerInterface> &referenceManagerPtr,
                      const std::string &task_file,
                      const std::string &reference_file,
                      rclcpp_lifecycle::LifecycleNode::SharedPtr node);

        ~TargetManager() = default;

        void update(const rclcpp::Time &time);

    private:
        TargetTrajectories targetPoseToTargetTrajectories(const vector_t &targetPose,
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
        nav_msgs::msg::Odometry getOdomMsg(const ocs2::TargetTrajectories &trajectories);
        void publishMsgs(const nav_msgs::msg::Odometry &odom) const;

        CtrlComponent &ctrl_component_;
        std::shared_ptr<ReferenceManagerInterface> referenceManagerPtr_;

        vector_t default_joint_state_{};
        scalar_t command_height_{};
        scalar_t time_to_target_{};
        scalar_t target_displacement_velocity_;
        scalar_t target_rotation_velocity_;

        rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
        rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
    };
}

#endif // TARGETMANAGER_H
