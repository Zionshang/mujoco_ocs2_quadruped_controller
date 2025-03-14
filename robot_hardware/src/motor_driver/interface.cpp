#include "motor_driver/interface.h"

Interface::Interface()
    : first_calibration_(true)
{
    module_0_ = new Serial("/dev/stm32_0", 0);
    module_1_ = new Serial("/dev/stm32_1", 1);
    // module_0_ = new Udp("192.168.1.100", 8000, 0);
    // module_1_ = new Udp("192.168.2.100", 8000, 1);
    module_2_ = new Serial("/dev/stm32_2", 2);
    module_0_->Start();
    module_1_->Start();
    module_2_->Start();

    pos_abad_left_ = -0.7556;
    pos_hip_left_ = -3.4907;
    pos_knee_left_ = 0.3558;
    pos_abad_right_ = -0.7556;
    pos_hip_right_ = -1.9199;
    pos_knee_right_ = -2.7857;

    // pos_abad_left_  = 0;
    // pos_hip_left_   = 0;
    // pos_knee_left_  = 0;
    // pos_abad_right_ = 0;
    // pos_hip_right_  = 0;
    // pos_knee_right_ = 0;

    // 数据
    module_0_send_buffer_ = new unsigned char[1024];
    module_0_recv_buffer_ = new unsigned char[1024];
    module_1_send_buffer_ = new unsigned char[1024];
    module_1_recv_buffer_ = new unsigned char[1024];
    module_2_send_buffer_ = new unsigned char[1024];
    module_2_recv_buffer_ = new unsigned char[1024];
    output_ = new MotorData[8];

    memset(module_0_send_buffer_, 0, 1024);
    memset(module_0_recv_buffer_, 0, 1024);
    memset(module_1_send_buffer_, 0, 1024);
    memset(module_1_recv_buffer_, 0, 1024);
    memset(module_2_send_buffer_, 0, 1024);
    memset(module_2_recv_buffer_, 0, 1024);

    // 初始化
    InitAllSenders();
}

Interface::~Interface()
{
    module_0_->Stop();
    module_1_->Stop();
    module_2_->Stop();
    delete module_0_;
    delete module_1_;
    delete module_2_;

    delete module_0_send_buffer_;
    delete module_0_recv_buffer_;
    delete module_1_send_buffer_;
    delete module_1_recv_buffer_;
    delete module_2_send_buffer_;
    delete module_2_recv_buffer_;
    delete output_;
}

void Interface::SetMotorData(LowCmd &cmd)
{
    for (size_t i = 0; i < 16; i++)
    {
        cmd.motor_cmd[i].kp = 0;
        cmd.motor_cmd[i].kd = 0;
        cmd.motor_cmd[i].torque = 0;
    }

    // 交换至定义顺序
    std::swap(cmd.motor_cmd[12], cmd.motor_cmd[8]);
    std::swap(cmd.motor_cmd[13], cmd.motor_cmd[9]);
    std::swap(cmd.motor_cmd[14], cmd.motor_cmd[10]);

    std::swap(cmd.motor_cmd[8], cmd.motor_cmd[4]);
    std::swap(cmd.motor_cmd[9], cmd.motor_cmd[5]);
    std::swap(cmd.motor_cmd[10], cmd.motor_cmd[6]);

    // 需要取反的电机索引
    const int reverse_motors[] = {0, 1, 2, 8, 13, 14};

    // 将指定电机的所有状态取反
    for (int idx : reverse_motors)
    {
        cmd.motor_cmd[idx].position = -cmd.motor_cmd[idx].position;
        cmd.motor_cmd[idx].velocity = -cmd.motor_cmd[idx].velocity;
        cmd.motor_cmd[idx].torque = -cmd.motor_cmd[idx].torque;
    }
    // 零位补偿
    SetZeroPosOffset(cmd);

    // 前8个电机
    std::vector<MotorCmd> motor_data_0;
    motor_data_0.resize(8);

    for (size_t i = 0; i < 8; i++)
    {
        motor_data_0[i] = cmd.motor_cmd[i];
    }

    // 后8个电机
    std::vector<MotorCmd> motor_data_1;
    motor_data_1.resize(8);

    for (size_t i = 0; i < 8; i++)
    {
        motor_data_1[i] = cmd.motor_cmd[i + 8];
    }

    // 标零
    if (cmd.mode == 15)
    {
        // 标零指令只下发一次
        if (first_calibration_)
        {
            first_calibration_ = false;
        }
        else
        {
            cmd.mode = 0;
        }
    }

    SetMotorDataAndOrder(motor_data_0, cmd.order, cmd.mode, cmd.type, module_0_, module_0_send_buffer_);
    SetMotorDataAndOrder(motor_data_1, cmd.order, cmd.mode, cmd.type, module_1_, module_1_send_buffer_);
    SetRobotOrder(cmd.order, module_2_, module_2_send_buffer_);
}

void Interface::GetMotorData(LowState &state)
{
    std::vector<MotorState> motor_data_0, motor_data_1;
    motor_data_0.resize(8);
    motor_data_1.resize(8);
    StateList state_0, state_1;
    std::vector<float> feet_force_0, feet_force_1;
    feet_force_0.resize(2);
    feet_force_1.resize(2);
    int mode_0, mode_1;

    GetMotorAndRobotData(motor_data_0, state_0, feet_force_0, mode_0, module_0_, module_0_recv_buffer_);
    GetMotorAndRobotData(motor_data_1, state_1, feet_force_1, mode_1, module_1_, module_1_recv_buffer_);
    GetRobotState(state_0, module_2_, module_2_recv_buffer_);

    // 赋值
    state.mode = mode_0;
    state.state_list = CombineState(state_0, state_1);
    state.feet_force[0] = feet_force_0[0];
    state.feet_force[1] = feet_force_0[1];
    state.feet_force[2] = feet_force_1[0];
    state.feet_force[3] = feet_force_1[1];

    for (size_t i = 0; i < 8; i++)
    {
        state.motor_state[i] = motor_data_0[i];
    }

    for (size_t i = 0; i < 8; i++)
    {
        state.motor_state[i + 8] = motor_data_1[i];
    }

    // 零位补偿
    SetZeroPosOffset(state);

    // 需要取反的电机索引
    const int reverse_motors[] = {0, 1, 2, 8, 13, 14};

    // 将指定电机的所有状态取反
    for (int idx : reverse_motors)
    {
        state.motor_state[idx].position = -state.motor_state[idx].position;
        state.motor_state[idx].velocity = -state.motor_state[idx].velocity;
        state.motor_state[idx].torque = -state.motor_state[idx].torque;
    }

    // 交换至定义顺序
    std::swap(state.motor_state[4], state.motor_state[8]);
    std::swap(state.motor_state[5], state.motor_state[9]);
    std::swap(state.motor_state[6], state.motor_state[10]);

    std::swap(state.motor_state[8], state.motor_state[12]);
    std::swap(state.motor_state[9], state.motor_state[13]);
    std::swap(state.motor_state[10], state.motor_state[14]);
}

StateList Interface::CombineState(StateList &state_0, StateList &state_1)
{
    StateList result = state_0;

    for (size_t i = 0; i < 8; i++)
    {
        result.motor_connect[i + 8] = state_1.motor_connect[i];
    }

    result.id_error = state_0.id_error || state_1.id_error;

    return result;
}

void Interface::SetZeroPosOffset(LowState &state)
{
    state.motor_state[0].position += pos_abad_left_;
    state.motor_state[1].position += pos_hip_left_;
    state.motor_state[2].position += pos_knee_left_;
    state.motor_state[3].position += 0;
    state.motor_state[4].position += pos_abad_right_;
    state.motor_state[5].position += pos_hip_right_;
    state.motor_state[6].position += pos_knee_right_;
    state.motor_state[7].position += 0;
    state.motor_state[8].position += pos_abad_right_;
    state.motor_state[9].position += pos_hip_right_;
    state.motor_state[10].position += pos_knee_right_;
    state.motor_state[11].position += 0;
    state.motor_state[12].position += pos_abad_left_;
    state.motor_state[13].position += pos_hip_left_;
    state.motor_state[14].position += pos_knee_left_;
    state.motor_state[15].position += 0;
}

void Interface::SetZeroPosOffset(LowCmd &cmd)
{
    cmd.motor_cmd[0].position -= pos_abad_left_;
    cmd.motor_cmd[1].position -= pos_hip_left_;
    cmd.motor_cmd[2].position -= pos_knee_left_;
    cmd.motor_cmd[3].position -= 0;
    cmd.motor_cmd[4].position -= pos_abad_right_;
    cmd.motor_cmd[5].position -= pos_hip_right_;
    cmd.motor_cmd[6].position -= pos_knee_right_;
    cmd.motor_cmd[7].position -= 0;
    cmd.motor_cmd[8].position -= pos_abad_right_;
    cmd.motor_cmd[9].position -= pos_hip_right_;
    cmd.motor_cmd[10].position -= pos_knee_right_;
    cmd.motor_cmd[11].position -= 0;
    cmd.motor_cmd[12].position -= pos_abad_left_;
    cmd.motor_cmd[13].position -= pos_hip_left_;
    cmd.motor_cmd[14].position -= pos_knee_left_;
    cmd.motor_cmd[15].position -= 0;
}

unsigned short Interface::ByteToShort(unsigned char c1, unsigned char c2)
{
    unsigned short data = ((c1 & 0xFF) << 8) | c2;
    return data;
}

unsigned int Interface::ByteToUint(unsigned char c1, unsigned char c2, unsigned char c3, unsigned char c4)
{
    unsigned int data = (unsigned int)(((c1 & 0xFF) << 24) |
                                       ((c2 & 0xFF) << 16) |
                                       ((c3 & 0xFF) << 8) |
                                       ((c4 & 0xFF)));

    return data;
}

int Interface::ByteToSint(unsigned char c1, unsigned char c2, unsigned char c3, unsigned char c4)
{
    int data = (int)(((c1 & 0xFF) << 24) |
                     ((c2 & 0xFF) << 16) |
                     ((c3 & 0xFF) << 8) |
                     ((c4 & 0xFF)));

    return data;
}

void Interface::SetMotorDataAndOrder(std::vector<MotorCmd> &motor_data, InstructionList &order,
                                     int mode, int type, TransformInterface *module, unsigned char *buffer)
{
    // 判断数据大小
    if (motor_data.size() != 8)
    {
        std::cerr << "Send to motor data size not equal to 8, now size = " << motor_data.size() << std::endl;
        exit(-1);
    }

    // 分包-1
    buffer[0] = 0xFE;
    buffer[1] = 0xAA;
    buffer[2] = type;
    buffer[3] = mode;

    for (size_t i = 0; i < 6; i++)
    {
        buffer[4 + i * 10] = (FloatToInt(motor_data[i].torque, T_MIN, T_MAX, 16) >> 8) & 0xFF;
        buffer[5 + i * 10] = (FloatToInt(motor_data[i].torque, T_MIN, T_MAX, 16)) & 0xFF;
        buffer[6 + i * 10] = (FloatToInt(motor_data[i].velocity, V_MIN, V_MAX, 16) >> 8) & 0xFF;
        buffer[7 + i * 10] = (FloatToInt(motor_data[i].velocity, V_MIN, V_MAX, 16)) & 0xFF;
        buffer[8 + i * 10] = (FloatToInt(motor_data[i].position, P_MIN, P_MAX, 16) >> 8) & 0xFF;
        buffer[9 + i * 10] = (FloatToInt(motor_data[i].position, P_MIN, P_MAX, 16)) & 0xFF;
        buffer[10 + i * 10] = (FloatToInt(motor_data[i].kp, KP_MIN, KP_MAX, 16) >> 8) & 0xFF;
        buffer[11 + i * 10] = (FloatToInt(motor_data[i].kp, KP_MIN, KP_MAX, 16)) & 0xFF;
        buffer[12 + i * 10] = (FloatToInt(motor_data[i].kd, KD_MIN, KD_MAX, 16) >> 8) & 0xFF;
        buffer[13 + i * 10] = (FloatToInt(motor_data[i].kd, KD_MIN, KD_MAX, 16)) & 0xFF;
    }

    // 分包-2
    buffer[64] = 0xFE;
    buffer[65] = 0xBB;

    for (size_t i = 0; i < 2; i++)
    {
        buffer[66 + i * 10] = (FloatToInt(motor_data[i + 6].torque, T_MIN, T_MAX, 16) >> 8) & 0xFF;
        buffer[67 + i * 10] = (FloatToInt(motor_data[i + 6].torque, T_MIN, T_MAX, 16)) & 0xFF;
        buffer[68 + i * 10] = (FloatToInt(motor_data[i + 6].velocity, V_MIN, V_MAX, 16) >> 8) & 0xFF;
        buffer[69 + i * 10] = (FloatToInt(motor_data[i + 6].velocity, V_MIN, V_MAX, 16)) & 0xFF;
        buffer[70 + i * 10] = (FloatToInt(motor_data[i + 6].position, P_MIN, P_MAX, 16) >> 8) & 0xFF;
        buffer[71 + i * 10] = (FloatToInt(motor_data[i + 6].position, P_MIN, P_MAX, 16)) & 0xFF;
        buffer[72 + i * 10] = (FloatToInt(motor_data[i + 6].kp, KP_MIN, KP_MAX, 16) >> 8) & 0xFF;
        buffer[73 + i * 10] = (FloatToInt(motor_data[i + 6].kp, KP_MIN, KP_MAX, 16)) & 0xFF;
        buffer[74 + i * 10] = (FloatToInt(motor_data[i + 6].kd, KD_MIN, KD_MAX, 16) >> 8) & 0xFF;
        buffer[75 + i * 10] = (FloatToInt(motor_data[i + 6].kd, KD_MIN, KD_MAX, 16)) & 0xFF;
    }

    unsigned short check_sum = CrcVerify(buffer, 86);

    buffer[86] = (check_sum >> 8) & 0xFF;
    buffer[87] = check_sum & 0xFF;

    module->SetData(buffer, CONTROL_BUFFER_SIZE);
}

void Interface::GetMotorAndRobotData(std::vector<MotorState> &motor_data, StateList &state,
                                     std::vector<float> &feet_force, int &mode, TransformInterface *module, unsigned char *buffer)
{
    // 判断数据容器大小
    if (motor_data.size() != 8)
    {
        std::cerr << "recv from motor data size not equal to 8, now size = " << motor_data.size() << std::endl;
        exit(-1);
    }
    if (feet_force.size() != 2)
    {
        std::cerr << "feet force data size not equal to 2, now size = " << feet_force.size() << std::endl;
        exit(-1);
    }

    int recv_length = 0;
    module->GetData(buffer, recv_length);

    // 校验检查
    if (buffer[0] != 0xFE || buffer[1] != 0xEE)
        return;
    unsigned short check_sum = CrcVerify(buffer, 86);
    if (buffer[recv_length - 2] != (check_sum & 0xFF00) >> 8 || buffer[recv_length - 1] != (check_sum & 0x00FF))
        return;

    RobotData robot_data;
    // 机器人状态
    robot_data.mode = buffer[3];
    robot_data.error.data = ByteToShort(buffer[4], buffer[5]);
    // 足端力传感器暂未安装
    robot_data.feet_force[0] = 0.0;
    robot_data.feet_force[1] = 0.0;

    // 电机数据解析
    for (size_t i = 0; i < 8; i++)
    {
        float temp = ByteToShort(buffer[6 + 10 * i], buffer[7 + 10 * i]) / 10.0;
        unsigned char error = ByteToShort(buffer[8 + 10 * i], buffer[9 + 10 * i]);
        float tau = IntToFloat(ByteToShort(buffer[10 + 10 * i], buffer[11 + 10 * i]), T_MIN, T_MAX, 16);
        float velocity = IntToFloat(ByteToShort(buffer[12 + 10 * i], buffer[13 + 10 * i]), V_MIN, V_MAX, 16);
        float angle = IntToFloat(ByteToShort(buffer[14 + 10 * i], buffer[15 + 10 * i]), P_MIN, P_MAX, 16);

        output_[i].torque = tau;
        output_[i].velocity = velocity;
        output_[i].position = angle;
        output_[i].temperature = temp;
        output_[i].error.data = error;
    }

    // 数据转存
    mode = robot_data.mode;
    state.motor_connect[0] = robot_data.error.value.motor_0;
    state.motor_connect[1] = robot_data.error.value.motor_1;
    state.motor_connect[2] = robot_data.error.value.motor_2;
    state.motor_connect[3] = robot_data.error.value.motor_3;
    state.motor_connect[4] = robot_data.error.value.motor_4;
    state.motor_connect[5] = robot_data.error.value.motor_5;
    state.motor_connect[6] = robot_data.error.value.motor_6;
    state.motor_connect[7] = robot_data.error.value.motor_7;
    state.id_error = robot_data.error.value.id_error;
    state.motor_time_out = robot_data.error.value.motor_time_out;

    feet_force[0] = robot_data.feet_force[0];
    feet_force[1] = robot_data.feet_force[1];

    for (size_t i = 0; i < 8; i++)
    {
        motor_data[i].torque = output_[i].torque;
        motor_data[i].velocity = output_[i].velocity;
        motor_data[i].position = output_[i].position;
        motor_data[i].temperature = output_[i].temperature;
        motor_data[i].run_state = output_[i].error.value.mode_state;

        // 故障
        motor_data[i].error.under_voltage = output_[i].error.value.under_voltage;
        motor_data[i].error.over_current = output_[i].error.value.over_current;
        motor_data[i].error.over_heat = output_[i].error.value.over_heat;
        motor_data[i].error.magnetic_encoder = output_[i].error.value.magnetic_encoder;
        motor_data[i].error.HALL_encoder = output_[i].error.value.HALL_encoder;
        motor_data[i].error.encoder_calibrated = output_[i].error.value.encoder_calibrated;
    }
}

void Interface::SetRobotOrder(InstructionList &order, TransformInterface *module, unsigned char *buffer)
{
    // 下发给模块2的数据
    Command cmd;
    cmd.value.spotlight = order.open_light;
    cmd.value.state_light = order.state_light;
    cmd.value.imu_reset = order.imu_reset;
    cmd.value.not_used = 0;
    cmd.value.motor_power_supply = order.motor_power_supply;
    cmd.value.motor_charging_electrodes = order.motor_charging_electrodes;
    buffer[0] = 0xFE;
    buffer[1] = 0xCC;
    buffer[2] = 0;
    buffer[3] = cmd.data;
    buffer[4] = 0;
    buffer[5] = 0;
    buffer[6] = 0;
    buffer[7] = 0;
    buffer[8] = 0;
    buffer[9] = 0;
    buffer[10] = 0;
    buffer[11] = 0;
    buffer[12] = 0;
    buffer[13] = 0;
    unsigned short check_sum = CrcVerify(buffer, 14);
    buffer[14] = (check_sum >> 8) & 0xFF;
    buffer[15] = check_sum & 0xFF;
    // 发送
    module->SetData(buffer, CMD_BUFFER_SIZE);
}

void Interface::GetRobotState(StateList &state, TransformInterface *module, unsigned char *buffer)
{
    int recv_length = 0;
    module->GetData(buffer, recv_length);

    // 帧头校验
    if (buffer[0] != 0xFE || buffer[1] != 0xCD)
        return;

    // 校验码校验
    unsigned short check_sum = CrcVerify(module_2_recv_buffer_, recv_length - 2);
    if (buffer[recv_length - 2] != (check_sum & 0xFF00) >> 8 ||
        buffer[recv_length - 1] != (check_sum & 0x00FF))
        return;

    // 数据解析
    RobotState robot_state;
    robot_state.state.data = ByteToShort(buffer[2], buffer[3]);
    robot_state.temp = ByteToShort(buffer[4], buffer[5]);
    robot_state.bms_state.pack_soc = ByteToShort(buffer[6], buffer[7]);
    robot_state.bms_state.pack_soh = ByteToShort(buffer[8], buffer[9]);
    robot_state.bms_state.pack_vol = ByteToShort(buffer[10], buffer[11]);
    robot_state.bms_state.charge_pack_vol = ByteToShort(buffer[12], buffer[13]);
    robot_state.bms_state.discharge_pack_vol = ByteToShort(buffer[14], buffer[15]);
    robot_state.bms_state.pack_curr = ByteToSint(buffer[16], buffer[17], buffer[18], buffer[19]);
    robot_state.bms_state.charge_curr = ByteToUint(buffer[20], buffer[21], buffer[22], buffer[23]);
    robot_state.bms_state.discharge_curr = ByteToUint(buffer[24], buffer[25], buffer[26], buffer[27]);
    robot_state.bms_state.battery_feedback_state.data = ByteToUint(buffer[28], buffer[29], buffer[30], buffer[31]);
    robot_state.bms_state.battery_protection_state.data = ByteToUint(buffer[32], buffer[33], buffer[34], buffer[35]);

    // 数据转存
    state.imu_reset = robot_state.state.value.imu_reset;
    state.emergency_stop = robot_state.state.value.emergency_stop;
    state.charging_state = robot_state.state.value.charging_state;
    state.fan_0_error = robot_state.state.value.fan_0_error;
    state.fan_1_error = robot_state.state.value.fan_1_error;
    state.low_power = robot_state.state.value.low_power;
    state.power_warning = robot_state.state.value.power_warning;
    state.BMS_overtime = robot_state.state.value.BMS_overtime;
    state.motor_poweron = robot_state.state.value.motor_poweron;
    state.temp = (float)robot_state.temp / 100.0;
    state.battery_soc = (float)robot_state.bms_state.pack_soc / 10.0;
    state.battery_soh = robot_state.bms_state.pack_soh;
    state.battery_vol = (float)robot_state.bms_state.pack_vol / 10.0;
    state.charge_vol = (float)robot_state.bms_state.charge_pack_vol / 10.0;
    state.discharge_vol = (float)robot_state.bms_state.discharge_pack_vol / 10.0;
    state.battery_curr = (float)robot_state.bms_state.pack_curr / 1000.0;
    state.charge_curr = (float)robot_state.bms_state.charge_curr / 1000.0;
    state.discharge_curr = (float)robot_state.bms_state.discharge_curr / 1000.0;
}

void Interface::InitAllSenders()
{
    // 模块0
    module_0_send_buffer_[0] = 0xFE;
    module_0_send_buffer_[1] = 0xAA;
    module_0_send_buffer_[2] = 0;
    module_0_send_buffer_[3] = 0;

    for (size_t i = 0; i < 6; i++)
    {
        module_0_send_buffer_[4 + i * 10] = (FloatToInt(0, T_MIN, T_MAX, 16) >> 8) & 0xFF;
        module_0_send_buffer_[5 + i * 10] = (FloatToInt(0, T_MIN, T_MAX, 16)) & 0xFF;
        module_0_send_buffer_[6 + i * 10] = (FloatToInt(0, V_MIN, V_MAX, 16) >> 8) & 0xFF;
        module_0_send_buffer_[7 + i * 10] = (FloatToInt(0, V_MIN, V_MAX, 16)) & 0xFF;
        module_0_send_buffer_[8 + i * 10] = (FloatToInt(0, P_MIN, P_MAX, 16) >> 8) & 0xFF;
        module_0_send_buffer_[9 + i * 10] = (FloatToInt(0, P_MIN, P_MAX, 16)) & 0xFF;
        module_0_send_buffer_[10 + i * 10] = (FloatToInt(0, KP_MIN, KP_MAX, 16) >> 8) & 0xFF;
        module_0_send_buffer_[11 + i * 10] = (FloatToInt(0, KP_MIN, KP_MAX, 16)) & 0xFF;
        module_0_send_buffer_[12 + i * 10] = (FloatToInt(0, KD_MIN, KD_MAX, 16) >> 8) & 0xFF;
        module_0_send_buffer_[13 + i * 10] = (FloatToInt(0, KD_MIN, KD_MAX, 16)) & 0xFF;
    }

    module_0_send_buffer_[64] = 0xFE;
    module_0_send_buffer_[65] = 0xBB;

    for (size_t i = 0; i < 2; i++)
    {
        module_0_send_buffer_[66 + i * 10] = (FloatToInt(0, T_MIN, T_MAX, 16) >> 8) & 0xFF;
        module_0_send_buffer_[67 + i * 10] = (FloatToInt(0, T_MIN, T_MAX, 16)) & 0xFF;
        module_0_send_buffer_[68 + i * 10] = (FloatToInt(0, V_MIN, V_MAX, 16) >> 8) & 0xFF;
        module_0_send_buffer_[69 + i * 10] = (FloatToInt(0, V_MIN, V_MAX, 16)) & 0xFF;
        module_0_send_buffer_[70 + i * 10] = (FloatToInt(0, P_MIN, P_MAX, 16) >> 8) & 0xFF;
        module_0_send_buffer_[71 + i * 10] = (FloatToInt(0, P_MIN, P_MAX, 16)) & 0xFF;
        module_0_send_buffer_[72 + i * 10] = (FloatToInt(0, KP_MIN, KP_MAX, 16) >> 8) & 0xFF;
        module_0_send_buffer_[73 + i * 10] = (FloatToInt(0, KP_MIN, KP_MAX, 16)) & 0xFF;
        module_0_send_buffer_[74 + i * 10] = (FloatToInt(0, KD_MIN, KD_MAX, 16) >> 8) & 0xFF;
        module_0_send_buffer_[75 + i * 10] = (FloatToInt(0, KD_MIN, KD_MAX, 16)) & 0xFF;
    }

    unsigned short check_sum = CrcVerify(module_0_send_buffer_, 86);

    module_0_send_buffer_[86] = (check_sum >> 8) & 0xFF;
    module_0_send_buffer_[87] = check_sum & 0xFF;

    module_0_->SetData(module_0_send_buffer_, CONTROL_BUFFER_SIZE);

    // 模块1
    memcpy(module_1_send_buffer_, module_0_send_buffer_, CONTROL_BUFFER_SIZE);
    module_1_->SetData(module_1_send_buffer_, CONTROL_BUFFER_SIZE);

    // 模块2
    module_2_send_buffer_[0] = 0xFE;
    module_2_send_buffer_[1] = 0xCC;
    check_sum = CrcVerify(module_2_send_buffer_, 14);
    module_2_send_buffer_[14] = (check_sum >> 8) & 0xFF;
    module_2_send_buffer_[15] = check_sum & 0xFF;

    module_2_->SetData(module_2_send_buffer_, CMD_BUFFER_SIZE);
}