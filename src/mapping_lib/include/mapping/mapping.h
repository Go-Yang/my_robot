#ifndef MAPPING_MODULE_H
#define MAPPING_MODULE_H

#include <memory>
#include <string>

#include <nav_msgs/msg/occupancy_grid.hpp>
#include <rclcpp/rclcpp.hpp>

#include "mapping/mapping_backend.h"
#include "sensor_interface/sensor_node.h"

/**
 * 建图模块 facade：根据参数 mapping_backend 选择后端实现。
 *
 * 可用后端：
 *   odom_grid    - 自研占据栅格建图（默认，里程计位姿）
 *   slam_toolbox - 外部 slam_toolbox 节点（my_robot_bringup 负责启动）
 *   custom_slam  - 自研 SLAM 占位，待实现
 */
class MappingModule {
public:
    /**
     * @param node   建图节点
     * @param sensor 已经 init() 过的传感器模块
     */
    void init(const rclcpp::Node::SharedPtr& node,
              const std::shared_ptr<sensor_interface::SensorNode>& sensor);

    nav_msgs::msg::OccupancyGrid::SharedPtr getMap() const;
    bool ready() const;
    std::string mapFrame() const;
    std::string backendName() const;

private:
    std::unique_ptr<mapping::MappingBackend> backend_;
};

#endif  // MAPPING_MODULE_H
