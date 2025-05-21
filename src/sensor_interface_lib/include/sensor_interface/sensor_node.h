// #pragma once
#ifndef SENSOR_NODE_H
#define SENSOR_NODE_H

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

namespace sensor_interface {

class SensorNode {
public:
    explicit SensorNode(const rclcpp::Node::SharedPtr& node);

    // 外部调用初始化
    void init();

private:
    void laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
    void timer_callback();

    rclcpp::Node::SharedPtr node_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace sensor_interface
#endif //SENSOR_NODE_H