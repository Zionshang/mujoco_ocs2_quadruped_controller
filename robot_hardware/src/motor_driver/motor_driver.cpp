#include "robot_hardware/motor_driver/motor_driver.h"

MotorDriver::MotorDriver()
{
}

MotorDriver::~MotorDriver()
{
}

custom_msgs::msg::LowState MotorDriver::joint_pub_callback()
{
    // custom_msgs::msg::LowCmd msg1;
    // LowCmd cmd;
    // cmd.mode = 1;
    // for (size_t i = 0; i < 12; i++)
    // {
    //     cmd.motor_cmd[i] = Convert(msg1.motor_cmd[i]);
    // }
    // cmd.order.state_light = msg1.order.state_light;
    // cmd.order.open_light = msg1.order.open_light;
    // cmd.order.motor_enable = 0;
    // cmd.order.motor_disenable = 1;
    // cmd.order.motor_power_supply = 1;

    // cmd.order.motor_charging_electrodes = msg1.order.motor_charging_electrodes;

    // interface.SetMotorData(cmd);
    ///////////////////////

    custom_msgs::msg::LowState msg;
    // LowState low_state;
    interface.GetMotorData(low_state);

    msg.robot_id = 0;
    msg.mode = low_state.mode;

    msg.temperature = low_state.state_list.temp; // rev

    msg.foot_force[0] = low_state.feet_force[0];
    msg.foot_force[1] = low_state.feet_force[1];
    msg.foot_force[2] = low_state.feet_force[2];
    msg.foot_force[3] = low_state.feet_force[3];
    msg.robot_error.motor_not_connect[0] = low_state.state_list.motor_connect[0];
    msg.robot_error.motor_not_connect[1] = low_state.state_list.motor_connect[1];
    msg.robot_error.motor_not_connect[2] = low_state.state_list.motor_connect[2];
    msg.robot_error.motor_not_connect[3] = low_state.state_list.motor_connect[3];
    msg.robot_error.motor_not_connect[4] = low_state.state_list.motor_connect[4];
    msg.robot_error.motor_not_connect[5] = low_state.state_list.motor_connect[5];
    msg.robot_error.motor_not_connect[6] = low_state.state_list.motor_connect[6];
    msg.robot_error.motor_not_connect[7] = low_state.state_list.motor_connect[7];
    msg.robot_error.motor_not_connect[8] = low_state.state_list.motor_connect[8];
    msg.robot_error.motor_not_connect[9] = low_state.state_list.motor_connect[9];
    msg.robot_error.motor_not_connect[10] = low_state.state_list.motor_connect[10];
    msg.robot_error.motor_not_connect[11] = low_state.state_list.motor_connect[11];
    msg.robot_error.motor_not_connect[12] = low_state.state_list.motor_connect[12];
    msg.robot_error.motor_not_connect[13] = low_state.state_list.motor_connect[13];
    msg.robot_error.motor_not_connect[14] = low_state.state_list.motor_connect[14];
    msg.robot_error.motor_not_connect[15] = low_state.state_list.motor_connect[15];

    msg.robot_error.motor_id_error = low_state.state_list.id_error;
    msg.robot_error.motor_over_time = low_state.state_list.motor_time_out;

    msg.robot_error.imu_reset = low_state.state_list.imu_reset; // rev

    msg.robot_error.stop_button_on = low_state.state_list.emergency_stop;
    msg.robot_error.open_charge = low_state.state_list.charging_state;
    msg.robot_error.fan_error[0] = low_state.state_list.fan_0_error;
    msg.robot_error.fan_error[1] = low_state.state_list.fan_1_error;
    msg.robot_error.low_power = low_state.state_list.low_power;
    msg.robot_error.power_warning = low_state.state_list.power_warning;
    msg.robot_error.bms_not_connect = low_state.state_list.BMS_overtime;
    msg.robot_error.all_motor_power_on = low_state.state_list.all_motor_poweron;

    for (size_t i = 0; i < 16; i++)
    {
        msg.motor_state[i] = Convert(low_state.motor_state[i]);
    }

    return msg;
}

void MotorDriver::LowPdCtrler()
{
    for (int i = 0; i < 12; ++i)
    {
        // low_cmd.motor_cmd[i].torque =
        //     low_cmd.motor_cmd[i].kp
        //         * (low_cmd.motor_cmd[i].position - low_state.motor_state[i].position)
        //     + low_cmd.motor_cmd[i].kd
        //           * (low_cmd.motor_cmd[i].velocity - low_state.motor_state[i].velocity);

        // low_cmd.motor_cmd[i].kp = 0;
        // low_cmd.motor_cmd[i].kd = 0;
        std::cout << low_cmd.motor_cmd[i].position << "  " << i << "  "
                  << low_state.motor_state[i].position << std::endl;
    }
    std::cout << std::endl;
}
// 订阅得到电机指令就发给电机
void MotorDriver::joint_sub_callback(const custom_msgs::msg::LowCmd::SharedPtr msg)
{
    // LowCmd cmd;

    //////
    low_cmd.mode = msg->mode;
    for (size_t i = 0; i < 16; i++) // rev
    {
        low_cmd.motor_cmd[i] = Convert(msg->motor_cmd[i]);
    }

    low_cmd.order.state_light = msg->order.state_light;

    low_cmd.order.imu_reset = msg->order.imu_reset; // rev

    low_cmd.order.open_light = msg->order.open_light;

    low_cmd.order.motor_enable = msg->order.motor_enable;
    low_cmd.order.motor_disenable = msg->order.motor_disenable;
    low_cmd.order.motor_power_supply = msg->order.motor_power_supply;
    low_cmd.order.motor_charging_electrodes = msg->order.motor_charging_electrodes;

    // LowPdCtrler();  // todo

    // 判断是否使能成功
    if (low_cmd.order.motor_enable == 1)
    {
        int sum = 0;
        for (size_t i = 0; i < 12; i++)
        {
            sum += low_state.motor_state[i].run_state;
        }

        if (sum == 2 * 12)
            low_cmd.order.motor_enable = 0;
    }

    // for (size_t i = 0; i < 12; i++)
    // {
    //     std::cout << cmd.motor_cmd[i].torque << "  ";
    // }
    // std::cout << std::endl;

    interface.SetMotorData(low_cmd);
}

MotorCmd MotorDriver::Convert(const custom_msgs::msg::MotorCmd &msg)
{
    MotorCmd data;
    data.position = msg.position;
    data.velocity = msg.velocity;
    data.torque = msg.torque;
    data.kp = msg.kp;
    data.kd = msg.kd;

    return data;
}

custom_msgs::msg::MotorState MotorDriver::Convert(const MotorState &data)
{
    custom_msgs::msg::MotorState msg;
    msg.position = data.position;
    msg.velocity = data.velocity;
    msg.torque = data.torque;
    msg.temperature = data.temperature;
    msg.state = data.run_state;

    msg.error.under_voltage = data.error.under_voltage;
    msg.error.over_current = data.error.over_current;
    msg.error.over_heat = data.error.over_heat;
    msg.error.magnetic_encoder = data.error.magnetic_encoder;
    msg.error.hall_encoder = data.error.HALL_encoder;
    msg.error.encoder_calibrated = data.error.encoder_calibrated;

    return msg;
}