#ifndef MAPPING_ODOM_GRID_BACKEND_H
#define MAPPING_ODOM_GRID_BACKEND_H

#include <atomic>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <tf2_ros/static_transform_broadcaster.h>

#include "mapping/mapping_backend.h"

namespace mapping {

/**
 * 自研占据栅格建图（里程计位姿）。
 *
 * 用 /scan + /odom 做 log-odds 逆传感器模型更新，发布 /map。
 * 地图发布在 map 系，同时广播静态恒等 map -> odom
 * （没有定位修正时，map 就等于 odom）。
 */
class OdomGridBackend : public MappingBackend {
public:
    std::string name() const override { return "odom_grid"; }

    void init(const rclcpp::Node::SharedPtr& node,
              const std::shared_ptr<sensor_interface::SensorNode>& sensor) override;

    nav_msgs::msg::OccupancyGrid::SharedPtr getMap() const override;
    bool ready() const override;
    std::string mapFrame() const override { return map_frame_; }

private:
    void updateCallback();
    void publishCallback();

    void integrateScan(const sensor_msgs::msg::LaserScan& scan,
                       const nav_msgs::msg::Odometry& odom);
    void raycast(int x0, int y0, int x1, int y1, bool hit);

    bool worldToGrid(double x, double y, int& col, int& row) const;
    void updateCell(int col, int row, float delta);
    void publishMap();

    rclcpp::Node::SharedPtr node_;
    std::shared_ptr<sensor_interface::SensorNode> sensor_;

    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;
    rclcpp::TimerBase::SharedPtr update_timer_;
    rclcpp::TimerBase::SharedPtr publish_timer_;
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster_;

    // 地图参数
    double resolution_{0.05};   // m / cell
    int width_{400};            // cells
    int height_{400};           // cells
    double origin_x_{-10.0};
    double origin_y_{-10.0};
    std::string map_frame_{"map"};
    std::string odom_frame_{"odom"};

    // 逆传感器模型参数
    double free_log_odds_{-0.45};
    double occupied_log_odds_{0.9};
    double min_log_odds_{-3.0};
    double max_log_odds_{3.5};
    double occupied_threshold_{0.4};

    mutable std::mutex map_mutex_;
    std::vector<float> log_odds_;
    std::vector<int8_t> occupancy_;
    nav_msgs::msg::OccupancyGrid::SharedPtr latest_map_;
    std::atomic<bool> ready_{false};

    int64_t last_scan_stamp_ns_{-1};
    std::atomic<std::size_t> scans_integrated_{0};
    std::size_t publish_count_{0};
};

}  // namespace mapping

#endif  // MAPPING_ODOM_GRID_BACKEND_H
