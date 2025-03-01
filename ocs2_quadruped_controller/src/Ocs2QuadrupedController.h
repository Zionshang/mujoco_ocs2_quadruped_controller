//
// Created by tlab-uav on 24-9-24.
//

#ifndef OCS2QUADRUPEDCONTROLLER_H
#define OCS2QUADRUPEDCONTROLLER_H

#include <controller_interface/controller_interface.hpp>
#include <custom_msgs/msg/mujoco_msg.hpp>
#include <ocs2_centroidal_model/CentroidalModelRbdConversions.h>
#include <ocs2_mpc/MPC_MRT_Interface.h>
#include <ocs2_quadruped_controller/estimator/StateEstimateBase.h>
#include <ocs2_quadruped_controller/interface/LeggedInterface.h>
#include <ocs2_quadruped_controller/wbc/WbcBase.h>
#include <ocs2_msgs/msg/mpc_observation.hpp>
#include "SafetyChecker.h"
#include "ocs2_quadruped_controller/control/CtrlComponent.h"
#include "ocs2_quadruped_controller/estimator/TerrainEstimator.h"
#include <sensor_msgs/msg/joint_state.hpp>

namespace ocs2::legged_robot
{
    class Ocs2QuadrupedController final : public controller_interface::ControllerInterface
    {
    public:
        CONTROLLER_INTERFACE_PUBLIC
        Ocs2QuadrupedController() = default;
        ~Ocs2QuadrupedController() override;

        CONTROLLER_INTERFACE_PUBLIC
        controller_interface::InterfaceConfiguration command_interface_configuration() const override;

        CONTROLLER_INTERFACE_PUBLIC
        controller_interface::InterfaceConfiguration state_interface_configuration() const override;

        CONTROLLER_INTERFACE_PUBLIC
        controller_interface::return_type update(
            const rclcpp::Time &time, const rclcpp::Duration &period) override;

        CONTROLLER_INTERFACE_PUBLIC
        controller_interface::CallbackReturn on_init() override;

        CONTROLLER_INTERFACE_PUBLIC
        controller_interface::CallbackReturn on_configure(
            const rclcpp_lifecycle::State &previous_state) override;

        CONTROLLER_INTERFACE_PUBLIC
        controller_interface::CallbackReturn on_activate(
            const rclcpp_lifecycle::State &previous_state) override;

        CONTROLLER_INTERFACE_PUBLIC
        controller_interface::CallbackReturn on_deactivate(
            const rclcpp_lifecycle::State &previous_state) override;

        CONTROLLER_INTERFACE_PUBLIC
        controller_interface::CallbackReturn on_cleanup(
            const rclcpp_lifecycle::State &previous_state) override;

        CONTROLLER_INTERFACE_PUBLIC
        controller_interface::CallbackReturn on_shutdown(
            const rclcpp_lifecycle::State &previous_state) override;

        CONTROLLER_INTERFACE_PUBLIC
        controller_interface::CallbackReturn on_error(
            const rclcpp_lifecycle::State &previous_state) override;

    protected:
        void setupLeggedInterface();

        void setupMpc();

        void setupMrt();

        void setupStateEstimate();

        void updateStateEstimation(const contact_flag_t &contact_flag,
                                   const rclcpp::Time &time,
                                   const rclcpp::Duration &period);
        void publishRefJointStateMsg(const vector_t &pos_des, const vector_t &vel_des, const vector_t &torque);

        CtrlComponent ctrl_comp_;
        std::vector<std::string> joint_names_;
        std::vector<std::string> feet_names_;
        std::vector<std::string> command_interface_types_;
        std::vector<std::string> state_interface_types_;

        std::unordered_map<
            std::string, std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface>> *>
            command_interface_map_ = {
                {"effort", &ctrl_comp_.joint_torque_command_interface_},
                {"position", &ctrl_comp_.joint_position_command_interface_},
                {"velocity", &ctrl_comp_.joint_velocity_command_interface_},
                {"kp", &ctrl_comp_.joint_kp_command_interface_},
                {"kd", &ctrl_comp_.joint_kd_command_interface_}};
        std::unordered_map<
            std::string, std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>> *>
            state_interface_map_ = {
                {"position", &ctrl_comp_.joint_position_state_interface_},
                {"effort", &ctrl_comp_.joint_effort_state_interface_},
                {"velocity", &ctrl_comp_.joint_velocity_state_interface_}};

        // IMU Sensor
        std::string imu_name_;
        std::vector<std::string> imu_interface_types_;
        // // Foot Force Sensor
        // std::string foot_force_name_;
        // std::vector<std::string> foot_force_interface_types_;

        double default_kp_ = 0;
        double default_kd_ = 6;

        rclcpp::Subscription<custom_msgs::msg::UserCmds>::SharedPtr control_input_subscription_;
        rclcpp::Publisher<ocs2_msgs::msg::MpcObservation>::SharedPtr observation_publisher_;
        rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr reference_joint_states_publisher_;

        std::string task_file_;
        std::string urdf_file_;
        std::string reference_file_;
        std::string gait_file_;

        bool verbose_;

        std::shared_ptr<LeggedInterface> legged_interface_;
        std::shared_ptr<PinocchioEndEffectorKinematics> eeKinematicsPtr_;

        // Whole Body Control
        std::shared_ptr<WbcBase> wbc_;
        std::shared_ptr<SafetyChecker> safety_checker_;

        // Nonlinear MPC
        std::shared_ptr<MPC_BASE> mpc_;
        std::shared_ptr<MPC_MRT_Interface> mpc_mrt_interface_;

        std::shared_ptr<CentroidalModelRbdConversions> rbd_conversions_;
        
        // Terrain Estimator
        std::shared_ptr<TerrainEstimator> terrain_estimator_;

    private:
        size_t planned_mode; // The mode that is active at the time the policy is evaluated at.
        vector_t measured_rbd_state_;
        std::thread mpc_thread_;
        std::atomic_bool controller_running_{}, mpc_running_{};
        benchmark::RepeatedTimer mpc_timer_;
        benchmark::RepeatedTimer wbc_timer_;
    };
}

#endif // OCS2QUADRUPEDCONTROLLER_H
