#include "mapping/slam_toolbox_backend.h"

#include <functional>
#include <stdexcept>
#include <utility>

namespace mapping {

void SlamToolboxBackend::init(
    const rclcpp::Node::SharedPtr& node,
    const std::shared_ptr<sensor_interface::SensorNode>& sensor)
{
    (void)sensor;  // slam_toolbox 自己订阅 /scan 和 TF
    node_ = node;
    if (!node_) {
        throw std::invalid_argument("SlamToolboxBackend::init 需要有效的 node");
    }

    const std::string map_topic =
        node_->declare_parameter<std::string>("slam_map_topic", "/map");

    map_sub_ = node_->create_subscription<nav_msgs::msg::OccupancyGrid>(
        map_topic, rclcpp::QoS(1).transient_local(),
        std::bind(&SlamToolboxBackend::mapCallback, this, std::placeholders::_1));

    RCLCPP_INFO(node_->get_logger(),
                "SlamToolboxBackend 已启动：等待外部 slam_toolbox 节点发布 %s",
                map_topic.c_str());
}

void SlamToolboxBackend::mapCallback(
    const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(map_mutex_);
    latest_map_ = msg;
    if (!ready_.exchange(true)) {
        RCLCPP_INFO(node_->get_logger(),
                    "收到 slam_toolbox 地图：%u x %u @ %.3f m，frame=%s",
                    msg->info.width, msg->info.height, msg->info.resolution,
                    msg->header.frame_id.c_str());
    }
}

nav_msgs::msg::OccupancyGrid::SharedPtr SlamToolboxBackend::getMap() const {
    std::lock_guard<std::mutex> lock(map_mutex_);
    return latest_map_;
}

std::string SlamToolboxBackend::mapFrame() const {
    std::lock_guard<std::mutex> lock(map_mutex_);
    if (latest_map_ && !latest_map_->header.frame_id.empty()) {
        return latest_map_->header.frame_id;
    }
    return "map";
}

}  // namespace mapping
