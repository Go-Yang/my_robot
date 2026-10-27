#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "robot_control/control.h"
#include "sensor_interface/sensor_node.h"
#include "mapping/mapping.h"
#include "navigation/navigation.h"

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);

    // 传感器模块：订阅 /scan /odom /imu，缓存最新数据
    auto sensor_node = rclcpp::Node::make_shared("sensor_node");
    auto sensor = std::make_shared<sensor_interface::SensorNode>(sensor_node);
    sensor->init();

    // 建图模块：消费 sensor 缓存的激光与里程计，发布 /map
    auto mapping_node = rclcpp::Node::make_shared("mapping_node");
    MappingModule mapping;
    mapping.init(mapping_node, sensor);

    // 控制模块（需要时取消注释）
    // auto control_node = rclcpp::Node::make_shared("control_node");
    // ControlModule control;
    // control.init(control_node);
    // control.sendCommand("Start");

    // 导航模块：订阅 /map 与 /goal_pose，发布 /plan 和 /cmd_vel
    auto navigation_node = rclcpp::Node::make_shared("navigation_node");
    NavigationModule navigation;
    navigation.init(navigation_node, sensor);

    // 多个节点交给同一个 MultiThreadedExecutor，
    // 比每个节点各开一个线程单独 spin 更安全（回调间有统一的并发管理）
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(sensor_node);
    executor.add_node(mapping_node);
    executor.add_node(navigation_node);

    executor.spin();

    rclcpp::shutdown();
    return 0;
}
