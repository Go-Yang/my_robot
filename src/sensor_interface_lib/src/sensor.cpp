#include "sensor.h"
#include "my_robot_msgs/msg/my_custom_message.hpp"


void SensorModule::init(rclcpp::Node::SharedPtr node) {
    this->node_ = node;
    sub = node_->create_subscription<std_msgs::msg::String>(
        "control_commands", 10, std::bind(&SensorModule::commandCallback, this, std::placeholders::_1));
    this->subscription_ = node_->create_subscription<my_robot_msgs::msg::MyCustomMessage>(
      "custom_topic", 10, std::bind(&SensorModule::messageCallback, this, std::placeholders::_1));
}


void SensorModule::commandCallback(const std_msgs::msg::String::SharedPtr msg) {
    RCLCPP_INFO(node_->get_logger(), "Received command: '%s'", msg->data.c_str());
}

void SensorModule::messageCallback(const my_robot_msgs::msg::MyCustomMessage::SharedPtr msg)
{
    // 打印接收到的消息
    RCLCPP_INFO(node_->get_logger(), "Received: id=%d, name=%s", msg->id, msg->name.c_str());
}