#include <rclcpp/rclcpp.hpp>
#include "robot_control/control.h"
#include "sensor_interface/sensor.h"
#include "mapping/mapping.h"
#include "navigation/navigation.h"
#include <thread>

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);

    // 创建多个节点
    auto control_node = rclcpp::Node::make_shared("control_node");
    ControlModule control;
    control.init(control_node);
    control.sendCommand("Start");

    auto sensor_node = rclcpp::Node::make_shared("sensor_node");
    SensorModule sensor;
    sensor.init(sensor_node);

    auto mapping_node = rclcpp::Node::make_shared("mapping_node");
    MappingModule mapping;
    mapping.init(mapping_node);

    auto navigation_node = rclcpp::Node::make_shared("navigation_node");
    NavigationModule navigation;
    navigation.init(navigation_node);
    navigation.navigateTo("target_location");

    // 使用多线程运行多个节点
    std::thread control_thread([&]() {
        rclcpp::spin(control_node);
    });
    std::thread sensor_thread([&]() {
        rclcpp::spin(sensor_node);
    });
    std::thread mapping_thread([&]() {
        rclcpp::spin(mapping_node);
    });
    std::thread navigation_thread([&]() {
        rclcpp::spin(navigation_node);
    });

    // 等待所有线程完成
    control_thread.join();
    sensor_thread.join();
    mapping_thread.join();
    navigation_thread.join();

    rclcpp::shutdown();
    return 0;
}
