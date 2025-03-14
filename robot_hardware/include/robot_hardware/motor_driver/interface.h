#ifndef _INTERFACE_H_
#define _INTERFACE_H_

#include <unistd.h>
#include <vector>
#include <iostream>
#include <mutex>
#include <cstdlib>
#include <cstring>
#include <string.h>

#include "data_type.h"
#include "tools.h"
#include "motor_driver/transform_interface.h"
#include "motor_serial.h"
#include "motor_driver/udp.h"

class Interface
{
public:
    Interface();
    ~Interface();

    void SetMotorData(LowCmd& cmd);
    void GetMotorData(LowState& state);

private:
    // 零位补偿
    void SetZeroPosOffset(LowState& state);
    void SetZeroPosOffset(LowCmd& cmd);

    StateList CombineState(StateList& state_0, StateList& state_1);
    
    TransformInterface* module_0_;
    TransformInterface* module_1_;
    TransformInterface* module_2_;

    /**
     * @brief 设置机器人指令
     * @param order 机器人的指令
     * @param module 单片机模块
     */
    void SetRobotOrder(InstructionList& order, TransformInterface* module, unsigned char* buffer);

    /**
     * @brief 获取机器人的状态，不含电机
     * @param state 机器人状态
     * @param module 单片机模块
     */
    void GetRobotState(StateList& state, TransformInterface* module, unsigned char* buffer);

    /**
     * @brief 设置电机和指令
     * @param motor_data 6个电机的控制数据
     * @param type 0是足式，1是轮式
     */
    void SetMotorDataAndOrder(std::vector<MotorCmd>& motor_data, InstructionList& order, 
        int mode, int type, TransformInterface* module, unsigned char* buffer);

    /**
     * @brief 获取电机和机器人状态
     * @param motor_data 返回的6个电机的状态数据
     */
    void GetMotorAndRobotData(std::vector<MotorState>& motor_data, StateList& state, 
        std::vector<float>& feet_force, int& mode, TransformInterface* module, unsigned char* buffer);

    /**
     * @brief 字符拼接
     */
    unsigned short ByteToShort(unsigned char c1, unsigned char c2);
    unsigned int ByteToUint(unsigned char c1, unsigned char c2, unsigned char c3, unsigned char c4);
    int ByteToSint(unsigned char c1, unsigned char c2, unsigned char c3, unsigned char c4);

    /**
     * @brief 初始化发送线程的数组
     */
    void InitAllSenders();

    // 补偿参数
    float pos_abad_left_;  // 肩关节
    float pos_hip_left_;   // 髋关节
    float pos_knee_left_;  // 膝关节
    float pos_abad_right_;  // 肩关节
    float pos_hip_right_;   // 髋关节
    float pos_knee_right_;  // 膝关节

    // 标定相关
    bool first_calibration_;

    // 数据相关
    unsigned char* module_0_send_buffer_;
    unsigned char* module_0_recv_buffer_;
    unsigned char* module_1_send_buffer_;
    unsigned char* module_1_recv_buffer_;
    unsigned char* module_2_send_buffer_;
    unsigned char* module_2_recv_buffer_;

    MotorData* output_;
};

#endif // _INTERFACE_H_