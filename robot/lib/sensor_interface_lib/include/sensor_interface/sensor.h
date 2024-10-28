#ifndef SENSOR_INTERFACE_SENSOR_H
#define SENSOR_INTERFACE_SENSOR_H

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

class SensorModule {
public:
    void init(rclcpp::Node::SharedPtr node);
    
private:
    void commandCallback(const std_msgs::msg::String::SharedPtr msg);
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub;
};

#endif // SENSOR_INTERFACE_SENSOR_H
