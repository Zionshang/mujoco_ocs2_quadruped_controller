#ifndef _MOTOR_SERIAL_H_
#define _MOTOR_SERIAL_H_

#include <stdlib.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/signal.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <termios.h>
#include <errno.h>
#include <limits.h>
#include <string>
#include <cstring>
#include <iostream>
#include <thread>
#include <vector>
#include <mutex>
#include <chrono>

#include <rclcpp/rclcpp.hpp>
#include "data_type.h"
#include "tools.h"
#include "motor_driver/transform_interface.h"


// 波特率
enum class BaudRate
{
    RATE_2400,
    RATE_4800,
    RATE_9600,
    RATE_19200,
    RATE_38400,
    RATE_57600,
    RATE_115200,
    RATE_460800,
    RATE_921600,
    RATE_2000000, // 2M
    RATE_4000000  // 4M
};

// 数据位大小
enum class DataSize
{
    SIZE_8bit,
    SIZE_7bit,
    SIZE_6bit,
    SIZE_5bit
};

// 校验方式
enum class CheckType
{
    NONE, // 无校验
    ODD, // 奇校验
    EVEN // 偶校验
};

// 停止位
enum class StopSize
{
    STOP_1bit,
    STOP_2bit
};

class Serial : public TransformInterface
{
public:
    Serial(std::string path, int serial_id);
    ~Serial();

    void Start() override;
    void Stop() override;
    void SetData(unsigned char* data, int size) override;
    void GetData(unsigned char* data, int& size) override;

private:
    void OpenSerial(std::string dev_path, BaudRate rate, DataSize data_size,
        CheckType check_type, StopSize stop_size);
    /**
     * @brief 主循环
     */
    void Run();

    /**
     * @brief udp收发一次
     */
    void SendRecv();

    void Close();
    int Send(unsigned char* buffer, int length);
    int Receive(unsigned char* buffer, int length);

    // 串口相关
    int serial_descriptor_;
    struct termios serial_option_; // 串口原始配置
    fd_set read_descriptor_;
    struct timeval time_out_;

    // 线程相关
    bool stop_;
    std::thread serial_thread_;
    std::mutex send_mutex_;
    std::mutex recv_mutex_;

    // 数据相关
    unsigned char* serial_send_buffer_;
    unsigned char* serial_recv_buffer_;

    int send_data_size_;
    int receive_data_size_;

    // 串口id
    int serial_id_;
};




#endif // _MOTOR_SERIAL_H_