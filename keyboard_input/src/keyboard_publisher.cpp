#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <termios.h>
#include <unistd.h>
#include <iostream>

class KeyboardPublisher : public rclcpp::Node
{
public:
    KeyboardPublisher() : Node("keyboard_publisher")
    {
        // 创建发布器
        publisher_ = this->create_publisher<std_msgs::msg::String>("keyboard_input", 10);

        RCLCPP_INFO(this->get_logger(), "Keyboard publisher node started in 50ms. Press any key to publish...");
    }

    void run(double frequency)
    {
        rclcpp::Rate rate(frequency);

        while (rclcpp::ok())
        {
            char key = getKey(); // 获取按键
            if (key == '\x03')   // 检测 Ctrl+C（ASCII 码为 3），退出程序
            {
                RCLCPP_INFO(this->get_logger(), "Exiting keyboard publisher...");
                break;
            }

            // 创建 ROS 消息
            auto msg = std_msgs::msg::String();
            msg.data = std::string(1, key); // 将按键值转为字符串
            publisher_->publish(msg);       // 发布键值

            RCLCPP_INFO(this->get_logger(), "Published key: %s", msg.data.c_str());
        }
        rate.sleep(); // 控制循环频率
    }

private:
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;

    // 使用 termios 获取按键值
    char getKey()
    {
        struct termios oldt, newt;
        char c;
        tcgetattr(STDIN_FILENO, &oldt); // 获取当前终端设置
        newt = oldt;
        newt.c_lflag &= ~(ICANON | ECHO); // 设置为非缓冲模式，并关闭回显
        tcsetattr(STDIN_FILENO, TCSANOW, &newt);
        c = getchar();                           // 获取按键值
        tcsetattr(STDIN_FILENO, TCSANOW, &oldt); // 恢复原有设置
        return c;
    }
};

int main(int argc, char **argv)
{
    double frequency = 20; // 50ms发布一次（1秒/50ms = 20Hz）
    rclcpp::init(argc, argv);
    auto node = std::make_shared<KeyboardPublisher>();
    node->run(frequency);
    rclcpp::shutdown();
    return 0;
}
