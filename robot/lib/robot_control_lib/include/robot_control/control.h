#ifndef ROBOT_CONTROL_CONTROL_H
#define ROBOT_CONTROL_CONTROL_H

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

class ControlModule {
public:
    void init(rclcpp::Node::SharedPtr node);
    void sendCommand(const std::string& command);
    
private:
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub;
};

#endif // ROBOT_CONTROL_CONTROL_H
