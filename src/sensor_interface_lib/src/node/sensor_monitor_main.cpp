#include <rclcpp/rclcpp.hpp>

#include "sensor_interface/sensor_node.h"

/**
 * 独立的传感器自检节点：
 *
 *   ros2 run sensor_interface_lib sensor_monitor
 *
 * 打印各话题的第一帧数据和接收速率，用于确认仿真 / 桥接是否正常。
 */
int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    auto node = rclcpp::Node::make_shared("sensor_monitor");
    sensor_interface::SensorNode sensor(node);
    sensor.init();

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
