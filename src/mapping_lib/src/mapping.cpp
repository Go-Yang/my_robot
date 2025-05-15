#include "mapping.h"
#include <rclcpp/rclcpp.hpp>
#include "my_robot_msgs/msg/my_custom_message.hpp"

void MappingModule::init(rclcpp::Node::SharedPtr node) {
    this->node_ = node;  // 存储 node 指针，供后续使用
    this->publisher_ = node_->create_publisher<my_robot_msgs::msg::MyCustomMessage>("custom_topic", 10);

    this->timer_ = node_->create_wall_timer(
        std::chrono::milliseconds(1000),
        std::bind(&MappingModule::publishMessage, this)
    );
}

void MappingModule::publishMessage() {
    auto message = my_robot_msgs::msg::MyCustomMessage();
    message.id = 42;
    message.name = "Example Mapping Data";

    RCLCPP_INFO(node_->get_logger(), "Publishing: id=%d, name=%s", message.id, message.name.c_str());

    publisher_->publish(message);
}