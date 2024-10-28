#include "sensor_interface/sensor.h"

void SensorModule::init(rclcpp::Node::SharedPtr node) {
    sub = node->create_subscription<std_msgs::msg::String>(
        "control_commands", 10, std::bind(&SensorModule::commandCallback, this, std::placeholders::_1));
}

void SensorModule::commandCallback(const std_msgs::msg::String::SharedPtr msg) {
    RCLCPP_INFO(rclcpp::get_logger("SensorModule"), "Received command: '%s'", msg->data.c_str());
}
