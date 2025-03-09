#ifndef MAPPING_MODULE_H
#define MAPPING_MODULE_H

#include "rclcpp/rclcpp.hpp"
#include "my_robot_msgs/msg/my_custom_message.hpp"

class MappingModule {
public:
    void init(rclcpp::Node::SharedPtr node);
    void publishMessage();

private:
    rclcpp::Node::SharedPtr node_;  // 存储节点指针
    rclcpp::Publisher<my_robot_msgs::msg::MyCustomMessage>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

#endif // MAPPING_MODULE_H
