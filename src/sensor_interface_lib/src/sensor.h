#ifndef SENSOR_INTERFACE_SENSOR_H
#define SENSOR_INTERFACE_SENSOR_H

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include "my_robot_msgs/msg/my_custom_message.hpp"


class SensorModule {
public:
    void init(rclcpp::Node::SharedPtr node);
    
private:
    rclcpp::Node::SharedPtr node_;  // 存储节点指针
    void messageCallback(const my_robot_msgs::msg::MyCustomMessage::SharedPtr msg);
    void commandCallback(const std_msgs::msg::String::SharedPtr msg);
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub;
    rclcpp::Subscription<my_robot_msgs::msg::MyCustomMessage>::SharedPtr subscription_;

};

#endif // SENSOR_INTERFACE_SENSOR_H
