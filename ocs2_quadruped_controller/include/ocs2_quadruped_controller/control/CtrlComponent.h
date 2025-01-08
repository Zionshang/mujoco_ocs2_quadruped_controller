//
// Created by biao on 24-9-10.
//

#ifndef INTERFACE_H
#define INTERFACE_H

#include <vector>
#include <hardware_interface/loaned_command_interface.hpp>
#include <hardware_interface/loaned_state_interface.hpp>
#include <custom_msgs/msg/user_cmds.hpp>

#include <ocs2_mpc/SystemObservation.h>
#include <ocs2_legged_robot_ros/visualization/LeggedRobotVisualizer.h>

#include "TargetManager.h"
#include "ocs2_quadruped_controller/estimator/StateEstimateBase.h"

struct CtrlComponent
{
    std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface>>
        joint_torque_command_interface_;
    std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface>>
        joint_position_command_interface_;
    std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface>>
        joint_velocity_command_interface_;
    std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface>>
        joint_kp_command_interface_;
    std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface>>
        joint_kd_command_interface_;

    std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>>
        joint_effort_state_interface_;
    std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>>
        joint_position_state_interface_;
    std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>>
        joint_velocity_state_interface_;

    std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>>
        imu_state_interface_;

    // std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>>
    //     foot_force_state_interface_;

    custom_msgs::msg::UserCmds user_cmds_;
    ocs2::SystemObservation observation_;
    int frequency_{};

    std::shared_ptr<ocs2::legged_robot::StateEstimateBase> estimator_;
    std::shared_ptr<ocs2::legged_robot::TargetManager> target_manager_;
    std::shared_ptr<ocs2::legged_robot::LeggedRobotVisualizer> visualizer_;

    CtrlComponent()
    {
        user_cmds_.linear_x_input = 0.0;
        user_cmds_.linear_y_input = 0.0;
        user_cmds_.angular_y_input = 0.0;
        user_cmds_.angular_z_input = 0.0;
        user_cmds_.height_ratio = 0.0;
        user_cmds_.gait_name = "stance";
        user_cmds_.passive_enable = false;
    }

    void clear()
    {
        joint_torque_command_interface_.clear();
        joint_position_command_interface_.clear();
        joint_velocity_command_interface_.clear();
        joint_kd_command_interface_.clear();
        joint_kp_command_interface_.clear();

        joint_effort_state_interface_.clear();
        joint_position_state_interface_.clear();
        joint_velocity_state_interface_.clear();

        imu_state_interface_.clear();
        // foot_force_state_interface_.clear();
    }
};

#endif // INTERFACE_H
