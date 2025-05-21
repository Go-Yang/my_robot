#include "sensor_interface/sensor_node.h"

namespace sensor_interface {

SensorNode::SensorNode(const rclcpp::Node::SharedPtr& node)
    : node_(node)
{
    // 构造函数中仅保存 node，不做初始化
}

void SensorNode::init() {
    // 初始化订阅器
    laser_sub_ = node_->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan",
        10,
        std::bind(&SensorNode::laser_callback, this, std::placeholders::_1)
    );

    // // 初始化定时器（1 Hz）
    // timer_ = node_->create_wall_timer(
    //     std::chrono::seconds(1),
    //     std::bind(&SensorNode::timer_callback, this)
    // );

    RCLCPP_INFO(node_->get_logger(), "SensorNode initialized.");
}

void SensorNode::laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    RCLCPP_INFO(node_->get_logger(), "Received LaserScan with %zu ranges", msg->ranges.size());
    // 打印基础信息
    RCLCPP_INFO_THROTTLE(
      node_->get_logger(),
      *node_->get_clock(),
      1000,  // 每1秒打印一次
      "收到扫描数据:\n"
      "角度范围: [%.2f, %.2f] rad\n"
      "距离范围: [%.2f, %.2f] m\n"
      "当前扫描点数: %zu",
      msg->angle_min, msg->angle_max,
      msg->range_min, msg->range_max,
      msg->ranges.size());
}

void SensorNode::timer_callback() {
    RCLCPP_INFO(node_->get_logger(), "Timer callback triggered.");
}

}  // namespace sensor_interface
