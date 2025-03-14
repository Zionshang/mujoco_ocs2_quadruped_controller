#ifndef _UDP_H_
#define _UDP_H_

#include <string>
#include <sys/select.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <cstdlib>
#include <cstdio>
#include <cstring>
#include <iostream>
#include <fcntl.h>
#include <thread>
#include <vector>
#include <mutex>
#include <chrono>

#include "motor_driver/transform_interface.h"

class Udp : public TransformInterface
{
public:
    /**
     * @brief udp通信的构造函数
     * @param ip 电机通信服务的ip
     * @param port 电机通信服务的端口
     */
    Udp(std::string ip, int port, int serial_id);
    ~Udp();

    void Start() override;
    void Stop() override;
    void SetData(unsigned char* data, int size) override;
    void GetData(unsigned char* data, int& size) override;

private:
    /**
     * @brief 主循环
     */
    void Run();

    /**
     * @brief udp收发一次
     */
    void SendRecv();

    // 通信相关
    std::string ip_;
    int port_;
    int socket_client_;
    struct sockaddr_in addr_server_;
    fd_set read_descriptor_;
    struct timeval time_out_;

    // 线程相关
    bool stop_;
    std::thread udp_thread_;
    std::mutex send_mutex_;
    std::mutex recv_mutex_;

    // 收发的数据
    unsigned char* udp_send_buffer_;
    unsigned char* udp_recv_buffer_;

    int send_data_size_;
    int receive_data_size_;
    
    // 串口id
    int serial_id_;
};

#endif // _UDP_H_