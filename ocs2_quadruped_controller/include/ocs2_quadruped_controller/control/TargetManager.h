#ifndef TARGETMANAGER_H
#define TARGETMANAGER_H

#include <memory>
#include <ocs2_mpc/SystemObservation.h>
#include <ocs2_oc/synchronized_module/ReferenceManagerInterface.h>
#include <ocs2_legged_robot/common/Types.h>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

#include "ocs2_quadruped_controller/interface/constraint/SwingTrajectoryPlanner.h"
#include <ocs2_quadruped_controller/model/QuadrupedIK.h>
#include <sensor_msgs/msg/joint_state.hpp>

struct CtrlComponent;

namespace ocs2::legged_robot
{
    class TargetManager
    {
    public:
        TargetManager(CtrlComponent &ctrl_component,
                      const std::shared_ptr<ReferenceManagerInterface> &referenceManagerPtr,
                      const std::shared_ptr<SwingTrajectoryPlanner> &swingTrajectoryPlanner,
                      const std::string &task_file,
                      const std::string &reference_file,
                      rclcpp_lifecycle::LifecycleNode::SharedPtr node);

        ~TargetManager() = default;

        void update(const vector3_t &ground_euler_angle, const rclcpp::Time &time, const rclcpp::Duration &period);

    private:
        TargetTrajectories targetPoseToTargetTrajectories(const vector_t &targetPose,
                                                          const vector_t &targetJointState,
                                                          const SystemObservation &observation,
                                                          const scalar_t &targetReachingTime);
        void updateTargetJointPose(scalar_t time,
                                   const vector_t &targetPose,
                                   vector_t &targetJointPose);

        nav_msgs::msg::Odometry getOdomMsg(const ocs2::TargetTrajectories &trajectories);
        void publishMsgs(const nav_msgs::msg::Odometry &odom) const;
        void publishRefJointStateMsg(const vector_t &targetJointState) const;

        CtrlComponent &ctrl_component_;
        QuadrupedIK ik_solver_;
        std::shared_ptr<ReferenceManagerInterface> referenceManagerPtr_;
        std::shared_ptr<SwingTrajectoryPlanner> swingTrajectoryPlannerPtr_;

        vector_t default_joint_state_{};
        vector_t target_joint_state_{};
        scalar_t command_height_{};
        scalar_t time_to_target_{};
        scalar_t target_displacement_velocity_;
        scalar_t target_rotation_velocity_;
        vector_t targetPose; // target [x, y, z, yaw, pitch, roll] expressed in WORLD frame
        double height_ratio; // the ratio of target height to the nominal height

        Matrix34d target_foot_pos_; // relative to body and expressed in body frame
        rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
        rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr reference_joint_states_publisher_;
        rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
    };
}

#endif // TARGETMANAGER_H
