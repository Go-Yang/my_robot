#include "sensor_interface/sensor_node.h"

#include <chrono>
#include <functional>
#include <memory>
#include <utility>

namespace sensor_interface {

SensorNode::SensorNode(const rclcpp::Node::SharedPtr& node)
    : node_(node)
{
}

void SensorNode::init() {
    scan_topic_ = node_->declare_parameter<std::string>("scan_topic", "/scan");
    odom_topic_ = node_->declare_parameter<std::string>("odom_topic", "/odom");
    imu_topic_ = node_->declare_parameter<std::string>("imu_topic", "/imu");

    scan_sub_ = node_->create_subscription<sensor_msgs::msg::LaserScan>(
        scan_topic_, 10,
        std::bind(&SensorNode::laserCallback, this, std::placeholders::_1));

    odom_sub_ = node_->create_subscription<nav_msgs::msg::Odometry>(
        odom_topic_, 10,
        std::bind(&SensorNode::odomCallback, this, std::placeholders::_1));

    imu_sub_ = node_->create_subscription<sensor_msgs::msg::Imu>(
        imu_topic_, 10,
        std::bind(&SensorNode::imuCallback, this, std::placeholders::_1));

    stats_timer_ = node_->create_wall_timer(
        std::chrono::duration<double>(kStatsPeriodSec),
        std::bind(&SensorNode::reportStatistics, this));

    RCLCPP_INFO(node_->get_logger(),
                "SensorNode 已启动：scan=%s odom=%s imu=%s",
                scan_topic_.c_str(), odom_topic_.c_str(), imu_topic_.c_str());
}

void SensorNode::laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    if (scan_count_.fetch_add(1, std::memory_order_relaxed) == 0) {
        RCLCPP_INFO(node_->get_logger(),
                    "收到第一帧 LaserScan：%zu 点，角度 [%.2f, %.2f] rad，"
                    "量程 [%.2f, %.2f] m，frame_id=%s",
                    msg->ranges.size(), msg->angle_min, msg->angle_max,
                    msg->range_min, msg->range_max, msg->header.frame_id.c_str());
    }

    std::lock_guard<std::mutex> lock(data_mutex_);
    latest_scan_ = std::move(msg);
}

void SensorNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    if (odom_count_.fetch_add(1, std::memory_order_relaxed) == 0) {
        const auto& position = msg->pose.pose.position;
        RCLCPP_INFO(node_->get_logger(),
                    "收到第一帧 Odometry：position=(%.2f, %.2f)，frame_id=%s，child_frame_id=%s",
                    position.x, position.y,
                    msg->header.frame_id.c_str(), msg->child_frame_id.c_str());
    }

    std::lock_guard<std::mutex> lock(data_mutex_);
    latest_odom_ = std::move(msg);
}

void SensorNode::imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg) {
    if (imu_count_.fetch_add(1, std::memory_order_relaxed) == 0) {
        RCLCPP_INFO(node_->get_logger(),
                    "收到第一帧 IMU：frame_id=%s", msg->header.frame_id.c_str());
    }

    std::lock_guard<std::mutex> lock(data_mutex_);
    latest_imu_ = std::move(msg);
}

bool SensorNode::hasScan() const {
    std::lock_guard<std::mutex> lock(data_mutex_);
    return latest_scan_ != nullptr;
}

bool SensorNode::hasOdom() const {
    std::lock_guard<std::mutex> lock(data_mutex_);
    return latest_odom_ != nullptr;
}

bool SensorNode::hasImu() const {
    std::lock_guard<std::mutex> lock(data_mutex_);
    return latest_imu_ != nullptr;
}

sensor_msgs::msg::LaserScan::SharedPtr SensorNode::getLatestScan() const {
    std::lock_guard<std::mutex> lock(data_mutex_);
    return latest_scan_;
}

nav_msgs::msg::Odometry::SharedPtr SensorNode::getLatestOdom() const {
    std::lock_guard<std::mutex> lock(data_mutex_);
    return latest_odom_;
}

sensor_msgs::msg::Imu::SharedPtr SensorNode::getLatestImu() const {
    std::lock_guard<std::mutex> lock(data_mutex_);
    return latest_imu_;
}

void SensorNode::reportStatistics() {
    const std::size_t scan_total = scan_count_.load(std::memory_order_relaxed);
    const std::size_t odom_total = odom_count_.load(std::memory_order_relaxed);
    const std::size_t imu_total = imu_count_.load(std::memory_order_relaxed);

    if (scan_total + odom_total + imu_total == 0) {
        RCLCPP_WARN(node_->get_logger(),
                    "尚未收到任何传感器数据：请确认仿真已启动、ros_gz_bridge 正常运行");
    } else {
        const double scan_rate =
            static_cast<double>(scan_total - last_scan_count_) / kStatsPeriodSec;
        const double odom_rate =
            static_cast<double>(odom_total - last_odom_count_) / kStatsPeriodSec;
        const double imu_rate =
            static_cast<double>(imu_total - last_imu_count_) / kStatsPeriodSec;

        RCLCPP_INFO(node_->get_logger(),
                    "传感器速率：scan %.1f Hz（累计 %zu），odom %.1f Hz（累计 %zu），"
                    "imu %.1f Hz（累计 %zu）",
                    scan_rate, scan_total, odom_rate, odom_total,
                    imu_rate, imu_total);
    }

    last_scan_count_ = scan_total;
    last_odom_count_ = odom_total;
    last_imu_count_ = imu_total;
}

}  // namespace sensor_interface
