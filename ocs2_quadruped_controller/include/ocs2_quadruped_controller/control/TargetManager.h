#ifndef TARGETMANAGER_H
#define TARGETMANAGER_H

#include <memory>
#include <ocs2_mpc/SystemObservation.h>
#include <ocs2_oc/synchronized_module/ReferenceManagerInterface.h>
#include <ocs2_legged_robot/common/Types.h>
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

        void update(const vector3_t &ground_euler_angle, const rclcpp::Time &time, const rclcpp::Duration &period);

    private:
        TargetTrajectories targetPoseToTargetTrajectories(const vector_t &targetPose,
                                                          const SystemObservation &observation,
                                                          const scalar_t &targetReachingTime);
        nav_msgs::msg::Odometry getOdomMsg(const ocs2::TargetTrajectories &trajectories);
        void publishMsgs(const nav_msgs::msg::Odometry &odom) const;

        CtrlComponent &ctrl_component_;
        std::shared_ptr<ReferenceManagerInterface> referenceManagerPtr_;

        vector_t default_joint_state_{};
        scalar_t command_height_{};
        scalar_t time_to_target_{};
        scalar_t target_displacement_velocity_;
        scalar_t target_rotation_velocity_;
        vector_t targetPose; // target [x, y, z, yaw, pitch, roll] expressed in WORLD frame
        double height_ratio; // the ratio of target height to the nominal height

        rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
        rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
    };
}

#endif // TARGETMANAGER_H
