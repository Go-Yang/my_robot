#include "mapping/custom_slam_backend.h"

#include <rclcpp/rclcpp.hpp>

namespace mapping {

void CustomSlamBackend::init(
    const rclcpp::Node::SharedPtr& node,
    const std::shared_ptr<sensor_interface::SensorNode>& sensor)
{
    (void)sensor;
    RCLCPP_WARN(node->get_logger(),
                "CustomSlamBackend 尚未实现：请在 src/custom_slam_backend.cpp 中"
                "实现扫描匹配 / 回环 / 位姿图优化，并发布 /map 与 map -> odom");
}

}  // namespace mapping
