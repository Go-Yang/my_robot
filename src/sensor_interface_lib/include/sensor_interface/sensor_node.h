#ifndef SENSOR_INTERFACE_SENSOR_NODE_H
#define SENSOR_INTERFACE_SENSOR_NODE_H

#include <atomic>
#include <cstddef>
#include <mutex>
#include <string>

#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

namespace sensor_interface {

/**
 * 传感器接收模块。
 *
 * 订阅激光雷达 / 里程计 / IMU，缓存最新一帧数据并统计接收速率。
 * 建图、导航等模块通过 getLatestXxx() 取用数据，不需要自己再订阅。
 */
class SensorNode {
public:
    explicit SensorNode(const rclcpp::Node::SharedPtr& node);

    /// 创建订阅与统计定时器。需在 executor 开始 spin 前调用一次。
    void init();

    bool hasScan() const;
    bool hasOdom() const;
    bool hasImu() const;

    /// 返回最新一帧（未收到时返回 nullptr）。返回的是共享指针，可安全跨回调使用。
    sensor_msgs::msg::LaserScan::SharedPtr getLatestScan() const;
    nav_msgs::msg::Odometry::SharedPtr getLatestOdom() const;
    sensor_msgs::msg::Imu::SharedPtr getLatestImu() const;

private:
    void laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg);
    void reportStatistics();

    static constexpr double kStatsPeriodSec = 5.0;

    rclcpp::Node::SharedPtr node_;

    std::string scan_topic_;
    std::string odom_topic_;
    std::string imu_topic_;

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::TimerBase::SharedPtr stats_timer_;

    mutable std::mutex data_mutex_;
    sensor_msgs::msg::LaserScan::SharedPtr latest_scan_;
    nav_msgs::msg::Odometry::SharedPtr latest_odom_;
    sensor_msgs::msg::Imu::SharedPtr latest_imu_;

    std::atomic<std::size_t> scan_count_{0};
    std::atomic<std::size_t> odom_count_{0};
    std::atomic<std::size_t> imu_count_{0};

    std::size_t last_scan_count_{0};
    std::size_t last_odom_count_{0};
    std::size_t last_imu_count_{0};
};

}  // namespace sensor_interface

#endif  // SENSOR_INTERFACE_SENSOR_NODE_H
