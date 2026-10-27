#ifndef MAPPING_SLAM_TOOLBOX_BACKEND_H
#define MAPPING_SLAM_TOOLBOX_BACKEND_H

#include <atomic>
#include <memory>
#include <mutex>
#include <string>

#include <nav_msgs/msg/occupancy_grid.hpp>
#include <rclcpp/rclcpp.hpp>

#include "mapping/mapping_backend.h"

namespace mapping {

/**
 * slam_toolbox 后端（外部节点）。
 *
 * 实际的 SLAM 由 my_robot_bringup 启动的 slam_toolbox 节点完成，
 * 本类只订阅 /map 并缓存最新地图，让 MappingModule 的接口保持一致。
 */
class SlamToolboxBackend : public MappingBackend {
public:
    std::string name() const override { return "slam_toolbox"; }

    void init(const rclcpp::Node::SharedPtr& node,
              const std::shared_ptr<sensor_interface::SensorNode>& sensor) override;

    nav_msgs::msg::OccupancyGrid::SharedPtr getMap() const override;
    bool ready() const override { return ready_.load(); }
    std::string mapFrame() const override;

private:
    void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);

    rclcpp::Node::SharedPtr node_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;

    mutable std::mutex map_mutex_;
    nav_msgs::msg::OccupancyGrid::SharedPtr latest_map_;
    std::atomic<bool> ready_{false};
};

}  // namespace mapping

#endif  // MAPPING_SLAM_TOOLBOX_BACKEND_H
