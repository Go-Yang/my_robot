#ifndef MAPPING_MAPPING_H
#define MAPPING_MAPPING_H

#include <rclcpp/rclcpp.hpp>

class MappingModule {
public:
    void init(rclcpp::Node::SharedPtr node);
    void processData();
};

#endif // MAPPING_MAPPING_H
