#include "mapping/odom_grid_backend.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdlib>
#include <functional>
#include <stdexcept>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>

namespace mapping {

namespace {
constexpr int8_t kUnknown = -1;
constexpr int8_t kFree = 0;
constexpr int8_t kOccupied = 100;
}  // namespace

void OdomGridBackend::init(
    const rclcpp::Node::SharedPtr& node,
    const std::shared_ptr<sensor_interface::SensorNode>& sensor)
{
    node_ = node;
    sensor_ = sensor;
    if (!node_ || !sensor_) {
        throw std::invalid_argument("OdomGridBackend::init 需要有效的 node 和 sensor");
    }

    resolution_ = node_->declare_parameter<double>("resolution", resolution_);
    width_ = node_->declare_parameter<int>("width", width_);
    height_ = node_->declare_parameter<int>("height", height_);
    origin_x_ = node_->declare_parameter<double>("origin_x", origin_x_);
    origin_y_ = node_->declare_parameter<double>("origin_y", origin_y_);
    map_frame_ = node_->declare_parameter<std::string>("map_frame", map_frame_);
    odom_frame_ = node_->declare_parameter<std::string>("odom_frame", odom_frame_);
    const std::string map_topic =
        node_->declare_parameter<std::string>("map_topic", "/map");

    free_log_odds_ = node_->declare_parameter<double>("free_log_odds", free_log_odds_);
    occupied_log_odds_ =
        node_->declare_parameter<double>("occupied_log_odds", occupied_log_odds_);
    min_log_odds_ = node_->declare_parameter<double>("min_log_odds", min_log_odds_);
    max_log_odds_ = node_->declare_parameter<double>("max_log_odds", max_log_odds_);
    occupied_threshold_ =
        node_->declare_parameter<double>("occupied_threshold", occupied_threshold_);

    const double update_rate = node_->declare_parameter<double>("update_rate", 20.0);
    const double publish_rate = node_->declare_parameter<double>("publish_rate", 2.0);

    if (resolution_ <= 0.0 || width_ <= 0 || height_ <= 0 ||
        update_rate <= 0.0 || publish_rate <= 0.0)
    {
        throw std::invalid_argument("建图参数非法：resolution/width/height/rate 必须为正");
    }
    if (min_log_odds_ >= max_log_odds_) {
        throw std::invalid_argument("建图参数非法：min_log_odds 必须小于 max_log_odds");
    }

    log_odds_.assign(static_cast<std::size_t>(width_) * height_, 0.0f);
    occupancy_.assign(static_cast<std::size_t>(width_) * height_, kUnknown);

    map_pub_ = node_->create_publisher<nav_msgs::msg::OccupancyGrid>(
        map_topic, rclcpp::QoS(1).transient_local());

    // 纯里程计建图没有定位修正，map 与 odom 等价，发布静态恒等变换，
    // 这样上层（导航/探索/RViz）可以统一在 map 系下工作。
    if (map_frame_ != odom_frame_) {
        static_tf_broadcaster_ =
            std::make_shared<tf2_ros::StaticTransformBroadcaster>(node_);
        geometry_msgs::msg::TransformStamped tf;
        tf.header.stamp = node_->now();
        tf.header.frame_id = map_frame_;
        tf.child_frame_id = odom_frame_;
        tf.transform.rotation.w = 1.0;
        static_tf_broadcaster_->sendTransform(tf);
        RCLCPP_INFO(node_->get_logger(),
                    "OdomGridBackend：发布静态 %s -> %s（恒等变换）",
                    map_frame_.c_str(), odom_frame_.c_str());
    }

    update_timer_ = node_->create_wall_timer(
        std::chrono::duration<double>(1.0 / update_rate),
        std::bind(&OdomGridBackend::updateCallback, this));
    publish_timer_ = node_->create_wall_timer(
        std::chrono::duration<double>(1.0 / publish_rate),
        std::bind(&OdomGridBackend::publishCallback, this));

    RCLCPP_INFO(node_->get_logger(),
                "OdomGridBackend 已启动：地图 %d x %d 格 @ %.3f m，范围 [%.1f, %.1f] x [%.1f, %.1f] m，"
                "发布到 %s（frame=%s）",
                width_, height_, resolution_,
                origin_x_, origin_x_ + width_ * resolution_,
                origin_y_, origin_y_ + height_ * resolution_,
                map_topic.c_str(), map_frame_.c_str());

    publishMap();
}

nav_msgs::msg::OccupancyGrid::SharedPtr OdomGridBackend::getMap() const {
    std::lock_guard<std::mutex> lock(map_mutex_);
    return latest_map_;
}

bool OdomGridBackend::ready() const {
    return ready_.load();
}

void OdomGridBackend::updateCallback() {
    auto scan = sensor_->getLatestScan();
    if (!scan) {
        return;
    }

    const int64_t stamp_ns = rclcpp::Time(scan->header.stamp).nanoseconds();
    if (stamp_ns == last_scan_stamp_ns_) {
        return;  // 这一帧已经处理过
    }

    auto odom = sensor_->getLatestOdom();
    if (!odom) {
        return;
    }

    integrateScan(*scan, *odom);
    last_scan_stamp_ns_ = stamp_ns;
    ++scans_integrated_;
    ready_.store(true);

    if (scans_integrated_.load() == 1) {
        const auto& p = odom->pose.pose.position;
        RCLCPP_INFO(node_->get_logger(),
                    "第一帧激光已写入地图：pose=(%.2f, %.2f)，%zu 条射线",
                    p.x, p.y, scan->ranges.size());
    }
}

void OdomGridBackend::integrateScan(
    const sensor_msgs::msg::LaserScan& scan,
    const nav_msgs::msg::Odometry& odom)
{
    const auto& position = odom.pose.pose.position;
    const auto& orientation = odom.pose.pose.orientation;

    // 四元数转偏航角（2D 建图只关心 yaw）
    const double yaw = std::atan2(
        2.0 * (orientation.w * orientation.z + orientation.x * orientation.y),
        1.0 - 2.0 * (orientation.y * orientation.y + orientation.z * orientation.z));

    int robot_col = 0;
    int robot_row = 0;
    if (!worldToGrid(position.x, position.y, robot_col, robot_row)) {
        return;  // 机器人已经跑出地图范围
    }

    std::lock_guard<std::mutex> lock(map_mutex_);

    for (std::size_t i = 0; i < scan.ranges.size(); ++i) {
        const double range = scan.ranges[i];
        if (std::isnan(range) || range < scan.range_min) {
            continue;
        }

        bool hit = false;
        double cast_range = 0.0;
        if (std::isinf(range) || range > scan.range_max) {
            // 没有有效回波：仍按最大量程标记射线经过的空闲区域
            cast_range = scan.range_max;
        } else {
            hit = true;
            cast_range = range;
        }

        const double angle =
            scan.angle_min + static_cast<double>(i) * scan.angle_increment;

        // 激光扇面方向与底盘一致（TurtleBot3 激光安装偏航为 0）
        const double end_x = position.x + cast_range * std::cos(yaw + angle);
        const double end_y = position.y + cast_range * std::sin(yaw + angle);

        int end_col = 0;
        int end_row = 0;
        // 端点可能在地图外，保留换算结果，由 raycast 逐格裁剪
        worldToGrid(end_x, end_y, end_col, end_row);

        raycast(robot_col, robot_row, end_col, end_row, hit);
    }
}

void OdomGridBackend::raycast(int x0, int y0, int x1, int y1, bool hit) {
    // Bresenham 直线：沿途格子标记为空闲，命中端点标记为占据
    const int dx = std::abs(x1 - x0);
    const int dy = -std::abs(y1 - y0);
    const int sx = (x0 < x1) ? 1 : -1;
    const int sy = (y0 < y1) ? 1 : -1;
    int err = dx + dy;

    while (true) {
        const bool is_endpoint = (x0 == x1 && y0 == y1);
        if (is_endpoint) {
            updateCell(x0, y0, static_cast<float>(
                hit ? occupied_log_odds_ : free_log_odds_));
            break;
        }

        updateCell(x0, y0, static_cast<float>(free_log_odds_));

        const int e2 = 2 * err;
        if (e2 >= dy) {
            err += dy;
            x0 += sx;
        }
        if (e2 <= dx) {
            err += dx;
            y0 += sy;
        }
    }
}

bool OdomGridBackend::worldToGrid(double x, double y, int& col, int& row) const {
    col = static_cast<int>(std::floor((x - origin_x_) / resolution_));
    row = static_cast<int>(std::floor((y - origin_y_) / resolution_));
    return col >= 0 && col < width_ && row >= 0 && row < height_;
}

void OdomGridBackend::updateCell(int col, int row, float delta) {
    if (col < 0 || col >= width_ || row < 0 || row >= height_) {
        return;
    }

    const std::size_t index = static_cast<std::size_t>(row) * width_ + col;
    const float value = std::clamp(
        log_odds_[index] + delta,
        static_cast<float>(min_log_odds_),
        static_cast<float>(max_log_odds_));
    log_odds_[index] = value;

    if (value > occupied_threshold_) {
        occupancy_[index] = kOccupied;
    } else if (value < -occupied_threshold_) {
        occupancy_[index] = kFree;
    } else {
        occupancy_[index] = kUnknown;
    }
}

void OdomGridBackend::publishCallback() {
    publishMap();

    // 每 10 次发布（默认 5 秒）打印一次地图统计
    if (publish_count_ % 10 != 0) {
        return;
    }

    std::size_t free_cells = 0;
    std::size_t occupied_cells = 0;
    {
        std::lock_guard<std::mutex> lock(map_mutex_);
        for (const int8_t value : occupancy_) {
            if (value == kFree) {
                ++free_cells;
            } else if (value == kOccupied) {
                ++occupied_cells;
            }
        }
    }

    RCLCPP_INFO(node_->get_logger(),
                "地图状态：已处理 %zu 帧，空闲 %zu 格，占据 %zu 格",
                scans_integrated_.load(), free_cells, occupied_cells);
}

void OdomGridBackend::publishMap() {
    auto msg = std::make_shared<nav_msgs::msg::OccupancyGrid>();
    {
        std::lock_guard<std::mutex> lock(map_mutex_);
        msg->header.stamp = node_->now();
        msg->header.frame_id = map_frame_;
        msg->info.map_load_time = msg->header.stamp;
        msg->info.resolution = static_cast<float>(resolution_);
        msg->info.width = static_cast<uint32_t>(width_);
        msg->info.height = static_cast<uint32_t>(height_);
        msg->info.origin.position.x = origin_x_;
        msg->info.origin.position.y = origin_y_;
        msg->info.origin.position.z = 0.0;
        msg->info.origin.orientation.w = 1.0;
        msg->data = occupancy_;
        latest_map_ = msg;
    }

    map_pub_->publish(*msg);
    ++publish_count_;
}

}  // namespace mapping
