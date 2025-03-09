#include "control.h"
#include "my_robot_msgs/msg/my_custom_message.hpp"

void ControlModule::init(rclcpp::Node::SharedPtr node) {
    pub = node->create_publisher<std_msgs::msg::String>("control_commands", 10);
    auto message = my_robot_msgs::msg::MyCustomMessage();
    message.id = 1;
    message.name = "example";
    message.data = {1.0, 2.0, 3.0};
}

void ControlModule::sendCommand(const std::string& command) {
    std_msgs::msg::String msg;
    msg.data = command;
    pub->publish(msg);
}
