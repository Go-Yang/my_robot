#include <rclcpp/rclcpp.hpp>
#include "robot_control/control.h"
#include "sensor_interface/sensor.h"
#include "mapping/mapping.h"
#include "navigation/navigation.h"

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("my_robot_node");

    ControlModule control;
    control.init(node);
    control.sendCommand("Start");

    SensorModule sensor;
    sensor.init(node);

    MappingModule mapping;
    mapping.init(node);

    NavigationModule navigation;
    navigation.init(node);
    navigation.navigateTo("target_location");

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
