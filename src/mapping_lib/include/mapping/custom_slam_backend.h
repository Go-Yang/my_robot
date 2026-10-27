#ifndef MAPPING_CUSTOM_SLAM_BACKEND_H
#define MAPPING_CUSTOM_SLAM_BACKEND_H

#include <memory>
#include <string>

#include <nav_msgs/msg/occupancy_grid.hpp>

#include "mapping/mapping_backend.h"

namespace mapping {

/**
 * 自研 SLAM 后端占位。
 *
 * 后续在这里实现扫描匹配 / 回环 / 位姿图优化，并发布 /map 与 map -> odom。
 * 目前只提供接口骨架，选中时会打印警告且不产生地图。
 */
class CustomSlamBackend : public MappingBackend {
public:
    std::string name() const override { return "custom_slam"; }

    void init(const rclcpp::Node::SharedPtr& node,
              const std::shared_ptr<sensor_interface::SensorNode>& sensor) override;

    nav_msgs::msg::OccupancyGrid::SharedPtr getMap() const override { return nullptr; }
    bool ready() const override { return false; }
    std::string mapFrame() const override { return "map"; }
};

}  // namespace mapping

#endif  // MAPPING_CUSTOM_SLAM_BACKEND_H
