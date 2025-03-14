#ifndef _IMU_SERIAL_H_
#define _IMU_SERIAL_H_

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

// 数据缓冲区大小
const int BUFFER_LENGTH = 1024;

// 波特率
enum class BaudRate2
{
    RATE_2400,
    RATE_4800,
    RATE_9600,
    RATE_19200,
    RATE_38400,
    RATE_57600,
    RATE_115200,
    RATE_460800,
    RATE_921600
};

// 数据位大小
enum class DataSize2
{
    SIZE_8bit,
    SIZE_7bit,
    SIZE_6bit,
    SIZE_5bit
};

// 校验方式
enum class CheckType2
{
    NONE, // 无校验
    ODD, // 奇校验
    EVEN // 偶校验
};

// 停止位
enum class StopSize2
{
    STOP_1bit,
    STOP_2bit
};

class Serial2
{
public:
    Serial2();
    ~Serial2();

    void OpenSerial2(std::string dev_path, BaudRate2 rate, DataSize2 data_size,
        CheckType2 check_type, StopSize2 stop_size);
    // 关闭通信
    void Close2();
    int Send2(char* buffer, int length);
    int Receive2(char* buffer, int length);

private:
    int serial_descriptor_; // 串口描述符
    struct termios serial_option_; // 串口原始配置

    fd_set read_descriptor_;
    struct timeval time_out_;
};

#endif // _IMU_SERIAL_H_