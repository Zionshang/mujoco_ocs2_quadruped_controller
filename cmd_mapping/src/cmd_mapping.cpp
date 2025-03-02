#include <rclcpp/rclcpp.hpp>
#include <custom_msgs/msg/user_cmds.hpp>
#include <std_msgs/msg/string.hpp>
#include <controller_manager_msgs/srv/switch_controller.hpp>

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

        switch_controller_client = this->create_client<controller_manager_msgs::srv::SwitchController>(
            "/controller_manager/switch_controller");

        initUserCmd();
        RCLCPP_INFO(this->get_logger(), "Command mapping node started in 50ms.");
    }

private:
    custom_msgs::msg::UserCmds user_cmd_;
    rclcpp::TimerBase::SharedPtr timer_;

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr keyboard_subscriptor;
    rclcpp::Publisher<custom_msgs::msg::UserCmds>::SharedPtr cmd_publisher;

    rclcpp::Client<controller_manager_msgs::srv::SwitchController>::SharedPtr switch_controller_client;

    void cmdMappingCallback(const std_msgs::msg::String keyboard_input)
    {
        switch (keyboard_input.data[0])
        {
        case 'w':
        case 'W':
            user_cmd_.linear_x_input += 0.1;
            break;
        case 's':
        case 'S':
            user_cmd_.linear_x_input = 0.0;
            user_cmd_.linear_y_input = 0.0;
            user_cmd_.angular_y_input = 0.0;
            user_cmd_.angular_z_input = 0.0;
            break;
        case 'a':
        case 'A':
            user_cmd_.linear_y_input += 0.1;
            break;
        case 'd':
        case 'D':
            user_cmd_.linear_y_input -= 0.1;
            break;
        case 'i':
        case 'I':
            user_cmd_.angular_y_input += 0.05;
            break;
        case 'k':
        case 'K':
            user_cmd_.angular_y_input -= 0.05;
            break;
        case 'j':
        case 'J':
            user_cmd_.angular_z_input += 0.1;
            break;
        case 'l':
        case 'L':
            user_cmd_.angular_z_input -= 0.1;
            break;
        case 'b':
        case 'B':
            user_cmd_.linear_x_input -= 0.1;
            break;
        case 'r':
        case 'R':
            user_cmd_.height_ratio += 0.2;
            break;
        case 'f':
        case 'F':
            user_cmd_.height_ratio -= 0.2;
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
        case '5':
            user_cmd_.gait_name = "standing_pace";
            break;
        case '6':
            user_cmd_.gait_name = "dynamic_walk";
            break;
        case '7':
            user_cmd_.gait_name = "bound";
            break;
        case ' ':
            user_cmd_.passive_enable = true;
            break;
        case '9':
            handleSwitchController();
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

        user_cmd_.height_ratio = 0.2;
        user_cmd_.gait_name = "stance";
        user_cmd_.passive_enable = false;
    }

    void handleSwitchController()
    {
        if (!switch_controller_client->wait_for_service(1s))
        {
            RCLCPP_ERROR(this->get_logger(), "Service not available after waiting");
            return;
        }

        auto request = std::make_shared<controller_manager_msgs::srv::SwitchController::Request>();
        request->activate_controllers.push_back("ocs2_quadruped_controller");

        switch_controller_client->async_send_request(request);
        RCLCPP_INFO(this->get_logger(), "Request to activate the controller: ocs2_quadruped_controller");
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
