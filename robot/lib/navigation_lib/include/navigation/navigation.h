#ifndef NAVIGATION_NAVIGATION_H
#define NAVIGATION_NAVIGATION_H

#include <rclcpp/rclcpp.hpp>

class NavigationModule {
public:
    void init(rclcpp::Node::SharedPtr node);
    void navigateTo(const std::string& destination);
};

#endif // NAVIGATION_NAVIGATION_H
