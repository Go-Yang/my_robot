#ifndef MAPPING_MAPPING_BACKEND_H
#define MAPPING_MAPPING_BACKEND_H

#include <memory>
#include <string>

#include <nav_msgs/msg/occupancy_grid.hpp>
#include <rclcpp/rclcpp.hpp>

#include "sensor_interface/sensor_node.h"

namespace mapping {

/**
 * 建图后端统一接口。
 *
 * 一个后端负责“产生地图”：
 * - 自研后端（如 odom_grid）自己订阅 /scan + /odom 并发布 /map；
 * - 外部 SLAM 后端（如 slam_toolbox）由独立节点发布 /map，这里只负责缓存。
 *
 * 后端还必须保证 map 坐标系可用：
 * - odom_grid 发布静态 map -> odom（恒等变换）；
 * - slam_toolbox 自己发布 map -> odom。
 *
 * 新增后端时实现下面的纯虚函数，并在 MappingModule 的工厂里注册即可。
 */
class MappingBackend {
public:
    virtual ~MappingBackend() = default;

    /// 后端名字，用于日志和参数选择
    virtual std::string name() const = 0;

    /**
     * 初始化后端。
     * @param node   建图节点
     * @param sensor 已 init() 的传感器模块（自研后端用来取 scan/odom）
     */
    virtual void init(const rclcpp::Node::SharedPtr& node,
                      const std::shared_ptr<sensor_interface::SensorNode>& sensor) = 0;

    /// 最新一帧地图；尚未产生地图时返回 nullptr
    virtual nav_msgs::msg::OccupancyGrid::SharedPtr getMap() const = 0;

    /// 是否已经有可用地图
    virtual bool ready() const = 0;

    /// 地图所在坐标系（通常是 map）
    virtual std::string mapFrame() const = 0;
};

}  // namespace mapping

#endif  // MAPPING_MAPPING_BACKEND_H
