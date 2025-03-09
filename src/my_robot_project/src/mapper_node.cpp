#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

class MapperNode : public rclcpp::Node {
public:
    MapperNode() : Node("mapper_node") {
        subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "my_robot/scan", 10,
            std::bind(&MapperNode::scanCallback, this, std::placeholders::_1));
    }

private:
    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        RCLCPP_INFO(this->get_logger(), "Received laser scan data");
        // 处理激光雷达数据
    }

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MapperNode>());
    rclcpp::shutdown();
    return 0;
}
