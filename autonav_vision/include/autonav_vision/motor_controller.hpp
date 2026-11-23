#pragma once

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>

#include <string>

namespace autonav_vision
{

class MotorController : public rclcpp::Node
{
public:
    MotorController();
    ~MotorController() override;

private:
    // Serial communication
    void open_serial();
    bool send_serial_command(int angle);

    // subscriber callback
    void motor_command_callback(
        const std_msgs::msg::Float64::SharedPtr msg);

    // Serial settings
    std::string serial_port_;
    int baud_rate_;
    int serial_fd_ = -1;

    // subscriber
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr
        motor_subscription_;
};

}
