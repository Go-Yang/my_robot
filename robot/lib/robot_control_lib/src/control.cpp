#include "robot_control/control.h"

void ControlModule::init(rclcpp::Node::SharedPtr node) {
    pub = node->create_publisher<std_msgs::msg::String>("control_commands", 10);
}

void ControlModule::sendCommand(const std::string& command) {
    std_msgs::msg::String msg;
    msg.data = command;
    pub->publish(msg);
}
