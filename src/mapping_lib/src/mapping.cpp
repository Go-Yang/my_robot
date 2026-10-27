#include "mapping/mapping.h"

#include <memory>
#include <stdexcept>
#include <string>

#include "mapping/custom_slam_backend.h"
#include "mapping/odom_grid_backend.h"
#include "mapping/slam_toolbox_backend.h"

void MappingModule::init(
    const rclcpp::Node::SharedPtr& node,
    const std::shared_ptr<sensor_interface::SensorNode>& sensor)
{
    if (!node || !sensor) {
        throw std::invalid_argument("MappingModule::init 需要有效的 node 和 sensor");
    }

    const std::string backend =
        node->declare_parameter<std::string>("mapping_backend", "odom_grid");

    if (backend == "odom_grid") {
        backend_ = std::make_unique<mapping::OdomGridBackend>();
    } else if (backend == "slam_toolbox") {
        backend_ = std::make_unique<mapping::SlamToolboxBackend>();
    } else if (backend == "custom_slam") {
        backend_ = std::make_unique<mapping::CustomSlamBackend>();
    } else {
        throw std::invalid_argument(
            "未知 mapping_backend: '" + backend +
            "'，可用：odom_grid / slam_toolbox / custom_slam");
    }

    backend_->init(node, sensor);

    RCLCPP_INFO(node->get_logger(),
                "MappingModule 使用后端: %s", backend_->name().c_str());
}

nav_msgs::msg::OccupancyGrid::SharedPtr MappingModule::getMap() const {
    return backend_ ? backend_->getMap() : nullptr;
}

bool MappingModule::ready() const {
    return backend_ && backend_->ready();
}

std::string MappingModule::mapFrame() const {
    return backend_ ? backend_->mapFrame() : "map";
}

std::string MappingModule::backendName() const {
    return backend_ ? backend_->name() : "none";
}
