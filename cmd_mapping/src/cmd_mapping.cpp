#include <rclcpp/rclcpp.hpp>
#include <custom_msgs/msg/user_cmds.hpp>
#include <std_msgs/msg/string.hpp>

using namespace std::chrono_literals;

using std::placeholders::_1;

class CmdMapping : public rclcpp::Node
{
public:
    CmdMapping(std::string name) : Node(name)
    {

        keyboard_subscriptor = this->create_subscription<std_msgs::msg::String>(
            "keyboard_input", 10, std::bind(&CmdMapping::cmdMappingCallback, this, _1));
        cmd_publisher = this->create_publisher<custom_msgs::msg::UserCmds>("user_cmd", 10);

        initUserCmd();
        RCLCPP_INFO(this->get_logger(), "Command mapping node started in 50ms.");
    }

private:
    custom_msgs::msg::UserCmds user_cmd_;
    rclcpp::TimerBase::SharedPtr timer_;

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr keyboard_subscriptor;
    rclcpp::Publisher<custom_msgs::msg::UserCmds>::SharedPtr cmd_publisher;

    void cmdMappingCallback(const std_msgs::msg::String keyboard_input)
    {
        switch (keyboard_input.data[0])
        {
        case 'w':
            user_cmd_.linear_x_input += 0.1;
            break;
        case 's':
            user_cmd_.linear_x_input -= 0.1;
            break;
        case 'a':
            user_cmd_.linear_y_input += 0.1;
            break;
        case 'd':
            user_cmd_.linear_y_input -= 0.1;
            break;
        case 'i':
            user_cmd_.angular_y_input += 0.05;
            break;
        case 'k':
            user_cmd_.angular_y_input -= 0.05;
            break;
        case 'j':
            user_cmd_.angular_z_input += 0.05;
            break;
        case 'b':
            user_cmd_.linear_x_input = 0.0;
            user_cmd_.linear_y_input = 0.0;
            user_cmd_.angular_y_input = 0.0;
            user_cmd_.angular_z_input = 0.0;
            break;
        case 'r':
            user_cmd_.height_ratio += 0.2;
            break;
        case 'f':
            user_cmd_.height_ratio -= 0.2;
            break;
        case 'l':
            user_cmd_.angular_z_input += 0.05;
            break;
        case '1':
            user_cmd_.gait_name = "stance";
            break;
        case '2':
            user_cmd_.gait_name = "trot";
            break;
        case '3':
            user_cmd_.gait_name = "standing_trot";
            break;
        case '4':
            user_cmd_.gait_name = "flying_trot";
            break;
        case ' ':
            user_cmd_.passive_enable = true;
            break;
        }

        user_cmd_.height_ratio = std::clamp(user_cmd_.height_ratio, 0.0, 1.0);

        cmd_publisher->publish(user_cmd_);
    }

    void initUserCmd()
    {
        user_cmd_.linear_x_input = 0.0;
        user_cmd_.linear_y_input = 0.0;
        user_cmd_.angular_y_input = 0.0;
        user_cmd_.angular_z_input = 0.0;

        user_cmd_.gait_name = "stance";
        user_cmd_.passive_enable = false;
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CmdMapping>("cmd_mapping");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
