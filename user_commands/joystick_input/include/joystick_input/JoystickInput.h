//
// Created by tlab-uav on 24-9-13.
//

#ifndef JOYSTICKINPUT_H
#define JOYSTICKINPUT_H
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <custom_msgs/msg/user_inputs.hpp>


class JoystickInput final : public rclcpp::Node {
public:
    JoystickInput();

    ~JoystickInput() override = default;

private:
    void joy_callback(sensor_msgs::msg::Joy::SharedPtr msg);

    custom_msgs::msg::UserInputs inputs_;
    rclcpp::Publisher<custom_msgs::msg::UserInputs>::SharedPtr publisher_;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr subscription_;
};


#endif //JOYSTICKINPUT_H
