#include "motor_driver/udp.h"

Udp::Udp(std::string ip, int port, int serial_id)
    : ip_(ip)
    , port_(port)
    , stop_(false)
    , serial_id_(serial_id)
{
    // 创建udp套接字
    socket_client_ = socket(AF_INET, SOCK_DGRAM, 0);

    if (socket_client_ < 0)
    {
        std::cerr << "socket create failed!" << std::endl;
        exit(-1);
    }

    // 设置服务器地址
    int addr_length = sizeof(addr_server_);
    memset(&addr_server_, 0, addr_length);
    addr_server_.sin_family = AF_INET;
    addr_server_.sin_port = htons(port_);
    addr_server_.sin_addr.s_addr = inet_addr(ip_.c_str());

    udp_send_buffer_ = new unsigned char[1024];
    udp_recv_buffer_ = new unsigned char[1024];
    memset(udp_send_buffer_, 0, 1024);
    memset(udp_recv_buffer_, 0, 1024);

    send_data_size_ = 0;
    receive_data_size_ = 0;

    std::cout << "open udp socket. id = " << serial_id_ << std::endl;
}

Udp::~Udp()
{
    delete[] udp_send_buffer_;
    delete[] udp_recv_buffer_;
}

void Udp::Run()
{
    using clock = std::chrono::high_resolution_clock;
    while (true)
    {
        auto start = clock::now();
        // 退出
        if (stop_)
        {
            break;
        }

        SendRecv();

        auto end = clock::now();
        std::chrono::duration<double> time = end - start;
        // if (serial_id_ == 1)
        //     std::cout << "Elapsed time 1: " << time.count() << " s" << std::endl;
    }

    // 关闭通信
    close(socket_client_);
    std::cerr << "Exit the udp thread..." << std::endl;
}

void Udp::Stop()
{
    stop_ = true;
    udp_thread_.join();
}

void Udp::Start()
{
    // 开启线程
    udp_thread_ = std::thread(&Udp::Run, this);
}

void Udp::SendRecv()
{
    // 发送数据
    int send_length;
    {
        std::unique_lock<std::mutex> lock(send_mutex_);
        send_length = sendto(socket_client_, udp_send_buffer_, send_data_size_, 0, 
            (struct sockaddr*)&addr_server_, sizeof(addr_server_));
    }

    if (send_length < 0)
    {
        perror("udp send data error!");
        exit(-1);
    }

    // 判断超时，超时时间为50ms
    FD_ZERO(&read_descriptor_);
    FD_SET(socket_client_, &read_descriptor_);
    time_out_.tv_sec = 0;
    time_out_.tv_usec = 50000;
    int ready = select(socket_client_ + 1, &read_descriptor_, NULL, NULL, &time_out_);
    if (ready == 0)
    {
        std::cerr << "udp socket reading time out!" << std::endl;
        return;
    }

    // 接收数据
    int recv_length;      
    {
        std::unique_lock<std::mutex> lock(recv_mutex_);
        recv_length = recvfrom(socket_client_, udp_recv_buffer_, 1024, 0,
            nullptr, nullptr);
    }

    // 数据可视化
    if (serial_id_ == 1)
    {
        for (size_t i = 0; i < recv_length; i++)
        {
            printf("%2x ", udp_recv_buffer_[i]);
        }
        std::cout << std::endl;
    }
    
    if (recv_length > 0)
    {
        receive_data_size_ = recv_length;
    }
    else if (recv_length == 0)
    {
        // 连接断开
        close(socket_client_);
        FD_CLR(socket_client_, &read_descriptor_);
        std::cerr << "Server socket has been closed!" << std::endl;
        exit(-1);
    }
    else
    {
        // 接收数据失败
        if (errno == EAGAIN || errno == EWOULDBLOCK)
        {
            // 暂无数据可读，可继续读
            std::cerr << "No data to read, read again! erron code = " << errno << std::endl;
            return;
        }

        if (errno == EINTR)
        {
            // 被中断，可继续读
            std::cerr << "Interrupted, read again! erron code = " << errno << std::endl;
            return;
        }

        // 异常退出
        std::cerr << "Receive data error, error code = " << errno << std::endl;
        exit(-1);
    }
}

void Udp::SetData(unsigned char* data, int size)
{
    std::unique_lock<std::mutex> lock(send_mutex_);
    for (size_t i = 0; i < size; i++)
    {
        udp_send_buffer_[i] = data[i];  
    }
    send_data_size_ = size;
}

void Udp::GetData(unsigned char* data, int& size)
{
    std::unique_lock<std::mutex> lock(recv_mutex_);
    for (size_t i = 0; i < receive_data_size_; i++)
    {
        data[i] = udp_recv_buffer_[i];
    }
    size = receive_data_size_;
}