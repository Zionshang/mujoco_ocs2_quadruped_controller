#include "motor_driver/motor_serial.h"

Serial::Serial(std::string path, int serial_id)
    : stop_(false)
    , serial_id_(serial_id)
{
    serial_descriptor_ = -1;
    // 打开串口
    OpenSerial(path, BaudRate::RATE_115200, DataSize::SIZE_8bit, CheckType::NONE, StopSize::STOP_1bit);
    // 数据
    serial_send_buffer_ = new unsigned char[1024];
    serial_recv_buffer_ = new unsigned char[1024];
    memset(serial_send_buffer_, 0, 1024);
    memset(serial_recv_buffer_, 0, 1024);

    send_data_size_ = 0;
    receive_data_size_ = 0;
}

Serial::~Serial()
{
    delete[] serial_send_buffer_;
    delete[] serial_recv_buffer_;
}

void Serial::OpenSerial(std::string dev_path, BaudRate rate, DataSize data_size,
    CheckType check_type, StopSize stop_size)
{
    // 开启端口 O_NOCTTY不作为这个端口的控制终端，O_NDELAY不关心端口另一端是否激活或者停止
    serial_descriptor_ = open(dev_path.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
    if (serial_descriptor_ < 0)
    {
        std::cerr << "Can not open serial port : " << dev_path << std::endl;
        exit(-1);
    }
    std::cout << "open serial port. id = " << serial_id_ <<  std::endl;

    // 设置描述符为默认配置，即阻塞读取
    int file_control_ret = fcntl(serial_descriptor_, F_SETFL, 0);
    if (file_control_ret < 0)
    {
        std::cerr << "motor fd file set to default faile!" << std::endl;
        exit(-1);
    }

    // // 确认设备是否是终端设备(判断是否工作正常，是否被占用)
    // if (isatty(STDIN_FILENO) == 0)
    // {
    //     std::cerr << "standard input is not a terminal device, motor \
    //         device was not work well." << std::endl;
    //     exit(-1);
    // }

    // 配置
    int result = tcgetattr(serial_descriptor_, &serial_option_); // 保存旧的配置
    if (result == -1)
    {
        std::cerr << "can not get standard original serial port settings!" << std::endl;
        exit(-1);
    }
    struct termios option_new;
    memset(&option_new, 0, sizeof(option_new));
    // 默认配置
    option_new.c_cflag |= CLOCAL; // 确保程序在突发的作业控制或挂起时，不会成为端口的占有者
    option_new.c_cflag |= CREAD; // 启动接收 

    // 设置输入的波特率
    switch (rate)
    {
    case BaudRate::RATE_2400:
        cfsetspeed(&option_new, B2400);
        break;
    case BaudRate::RATE_4800:
        cfsetspeed(&option_new, B4800);
        break;
    case BaudRate::RATE_9600:
        cfsetspeed(&option_new, B9600);
        break;
    case BaudRate::RATE_19200:
        cfsetspeed(&option_new, B19200);
        break;
    case BaudRate::RATE_38400:
        cfsetspeed(&option_new, B38400);
        break;
    case BaudRate::RATE_57600:
        cfsetspeed(&option_new, B57600);
        break;
    case BaudRate::RATE_115200:
        cfsetspeed(&option_new, B115200);
        break;
    case BaudRate::RATE_460800:
        cfsetspeed(&option_new, B460800);
        break;
    case BaudRate::RATE_921600:
        cfsetspeed(&option_new, B921600);
        break;
    case BaudRate::RATE_2000000:
        cfsetspeed(&option_new, B2000000);
        break;
    case BaudRate::RATE_4000000:
        cfsetspeed(&option_new, B4000000);
        break;
        
    default:
        break;
    }

    // 设置数据位
    option_new.c_cflag &= (~CSIZE);
    switch (data_size)
    {
    case DataSize::SIZE_5bit:
        option_new.c_cflag |= CS5;
        break;
    case DataSize::SIZE_6bit:
        option_new.c_cflag |= CS6;
        break;
    case DataSize::SIZE_7bit:
        option_new.c_cflag |= CS7;
        break;
    case DataSize::SIZE_8bit:
        option_new.c_cflag |= CS8;
        break;

    default:
        break;
    }

    // 设置校验位
    switch (check_type)
    {
    case CheckType::NONE:
        option_new.c_cflag &= (~PARENB);
        break;
    case CheckType::ODD: // 奇校验
        option_new.c_cflag |= PARENB;
        option_new.c_cflag |= PARODD;
        option_new.c_iflag |= (INPCK | ISTRIP);
        break;
    case CheckType::EVEN: // 偶校验 
        option_new.c_cflag |= PARENB;
        option_new.c_cflag &= (~PARODD);
        option_new.c_iflag |= (INPCK | ISTRIP);
        break;

    default:
        break;
    }

    // 设置停止位
    switch (stop_size)
    {
    case StopSize::STOP_1bit:
        option_new.c_cflag &= (~CSTOPB);
        break;
    case StopSize::STOP_2bit:
        option_new.c_cflag |= CSTOPB;
        break;       
    
    default:
        break;
    }

    // 设置最少字符和等待时间
    option_new.c_cc[VMIN] = 0;
    option_new.c_cc[VTIME] = 0;

    // 输入输出设置
    option_new.c_iflag |= IGNPAR; // 忽略桢错误和奇偶校验错
    option_new.c_oflag &= (~OPOST); // 不对输出数据进行处理
    option_new.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);  /*raw input*/

    // 刷新输入输出队列
    tcflush(serial_descriptor_, TCIOFLUSH);
    // 设置生效
    result = tcsetattr(serial_descriptor_, TCSANOW, &option_new);
    if (result == -1)
    {
        std::cerr << "failed to set serial config!" << std::endl;
        exit(-1);
    }
}

void Serial::Close()
{
    // 恢复串口原始配置
    if (serial_descriptor_ > 0)
    {
        std::cout << "reset motor serial port options..." << std::endl;
        tcsetattr(serial_descriptor_, TCSADRAIN, &serial_option_);
    }
    close(serial_descriptor_);
}

int Serial::Send(unsigned char* buffer, int length)
{
    int result = write(serial_descriptor_, buffer, length);
    return result;
}

int Serial::Receive(unsigned char* buffer, int length)
{
    FD_ZERO(&read_descriptor_);
    FD_SET(serial_descriptor_, &read_descriptor_);
    time_out_.tv_sec = 1;   // 1秒
    time_out_.tv_usec = 0;
    int ready = select(serial_descriptor_ + 1, &read_descriptor_, NULL, NULL, &time_out_);
    int result = 0;

    if (ready > 0){
        if (FD_ISSET(serial_descriptor_, &read_descriptor_))
            result = read(serial_descriptor_, buffer, length);
    }
    else if (ready == 0)
    {
        std::cerr << "motor serial port reading time out, over 1 seconds!" << std::endl; 
    }
    else
    {
        std::cerr << "motor serial port read error!" << std::endl;
    }

    return result;
}

void Serial::Stop()
{
    stop_ = true;
    serial_thread_.join();
}

void Serial::Start()
{
    // 开启线程
    serial_thread_ = std::thread(&Serial::Run, this);
}

void Serial::Run()
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

        SendRecv();  //电机数据帧

        auto end = clock::now();
        std::chrono::duration<double> time = end - start;
        // if (serial_id_ == 0)
        //     std::cout << "Elapsed time: " << time.count() << " s" << std::endl;
    }

    Close();
    std::cerr << "Exit the serial thread..." << std::endl;
}

void Serial::SendRecv()
{
    // 如果发送数据大小是0，则不发送
    if (send_data_size_ == 0)
    {
        usleep(50);
        return;
    }

    // 发送数据
    int send_length;
    {
        std::unique_lock<std::mutex> lock(send_mutex_);
        send_length = Send(serial_send_buffer_, send_data_size_);
    }

    if (send_length < 0)
    {
        perror("serial send data error!");
        exit(-1);
    }

    // 接收数据
    int recv_length;
    {
        // std::unique_lock<std::mutex> lock(recv_mutex_);
        recv_length = Receive(serial_recv_buffer_, 1024);  
    }

    // 数据可视化 输出的 ff ee ... 是这里
    // if (serial_id_ == 0)
    // {
    //     for (size_t i = 0; i < recv_length; i++)
    //     {
    //         printf("%2x ", serial_recv_buffer_[i]);
    //     }
    //     std::cout << std::endl;
    // }
    
    if (recv_length > 0)
    {
        receive_data_size_ = recv_length;
    }
    else if (recv_length == 0)
    {
        // 连接断开
        Close();
        FD_CLR(serial_descriptor_, &read_descriptor_);
        std::cerr << "serial has been closed!" << std::endl;
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

void Serial::SetData(unsigned char* data, int size)
{
    std::unique_lock<std::mutex> lock(send_mutex_);
    for (size_t i = 0; i < size; i++)
    {
        serial_send_buffer_[i] = data[i];  
    }

    // memcpy(serial_send_buffer_, data, size);
    send_data_size_ = size;
}

void Serial::GetData(unsigned char* data, int& size)
{
    // std::unique_lock<std::mutex> lock(recv_mutex_);
    for (size_t i = 0; i < receive_data_size_; i++)
    {
        data[i] = serial_recv_buffer_[i];
    }

    // memcpy(data, serial_recv_buffer_, receive_data_size_);
    size = receive_data_size_;
}