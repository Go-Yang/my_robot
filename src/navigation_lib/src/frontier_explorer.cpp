/**
 * 基于前沿（frontier）的自动探索节点。
 *
 *   /map ──> 找“空闲格旁边有未知格”的边界 ──> 按簇聚类 ──> 最近的簇
 *                                                       └─> /goal_pose
 *
 * 由 NavigationModule 负责规划与行驶；本节点只负责选目标、判断到达/超时。
 * 到达一个边界后稍等地图更新，再选下一个；没有剩余边界即认为探索完成。
 */
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <limits>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2/exceptions.h>
#include <tf2/time.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

class FrontierExplorer : public rclcpp::Node {
public:
    FrontierExplorer() : Node("frontier_explorer") {
        map_topic_ = declare_parameter<std::string>("map_topic", "/map");
        goal_topic_ = declare_parameter<std::string>("goal_topic", "/goal_pose");
        odom_topic_ = declare_parameter<std::string>("odom_topic", "/odom");
        // 墙壁边缘的栅格化缝隙会产生 3~7 格的小“伪边界”，过滤掉
        min_frontier_size_ = declare_parameter<int>("min_frontier_size", 10);
        goal_reached_distance_ =
            declare_parameter<double>("goal_reached_distance", 0.30);
        goal_timeout_ = declare_parameter<double>("goal_timeout", 180.0);
        progress_timeout_ = declare_parameter<double>("progress_timeout", 8.0);
        cooldown_ = declare_parameter<double>("cooldown", 0.5);
        blacklist_radius_ = declare_parameter<double>("blacklist_radius", 0.75);
        // 目标要离墙足够远，但 1 m 宽的走廊/门洞里 0.45 m 会没有可行格，
        // 取 0.35 m（机器人半径约 0.1 m + 安全余量）。
        goal_clearance_ = declare_parameter<double>("goal_clearance", 0.35);
        // 建图模块启动时会先发一张全未知地图；已知格太少时不能判定“探索完成”
        min_known_cells_ = declare_parameter<int>("min_known_cells", 500);
        min_target_distance_ =
            declare_parameter<double>("min_target_distance", 2.5);
        heading_weight_ = declare_parameter<double>("heading_weight", 0.6);
        const double period = declare_parameter<double>("explore_period", 1.0);

        if (period <= 0.0 || goal_timeout_ <= 0.0) {
            throw std::invalid_argument("探索参数非法：period / goal_timeout 必须为正");
        }

        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
            map_topic_, rclcpp::QoS(1).transient_local(),
            std::bind(&FrontierExplorer::mapCallback, this, std::placeholders::_1));
        odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
            odom_topic_, 10,
            std::bind(&FrontierExplorer::odomCallback, this, std::placeholders::_1));

        goal_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(goal_topic_, 10);
        marker_pub_ =
            create_publisher<visualization_msgs::msg::MarkerArray>("/frontiers", 10);

        timer_ = create_wall_timer(
            std::chrono::duration<double>(period),
            std::bind(&FrontierExplorer::tick, this));

        goal_sent_time_ = this->now();
        cooldown_until_ = this->now();
        last_progress_time_ = this->now();

        RCLCPP_INFO(get_logger(),
                    "FrontierExplorer 已启动：map=%s goal=%s，最小边界簇 %d 格，"
                    "到达阈值 %.2f m，单目标超时 %.1f s",
                    map_topic_.c_str(), goal_topic_.c_str(),
                    min_frontier_size_, goal_reached_distance_, goal_timeout_);
    }

private:
    struct Cluster {
        std::vector<int> cells;
        double nearest_x{0.0};
        double nearest_y{0.0};
        double nearest_dist{std::numeric_limits<double>::max()};
        double fallback_x{0.0};
        double fallback_y{0.0};
        double fallback_dist{std::numeric_limits<double>::max()};
        double centroid_x{0.0};
        double centroid_y{0.0};
    };

    void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
        map_ = *msg;
        have_map_ = true;
        if (!map_logged_) {
            map_logged_ = true;
            RCLCPP_INFO(get_logger(), "收到地图：%u x %u @ %.3f m，frame=%s",
                        map_.info.width, map_.info.height, map_.info.resolution,
                        map_.header.frame_id.c_str());
        }
    }

    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        const auto& p = msg->pose.pose.position;
        const auto& q = msg->pose.pose.orientation;
        odom_x_ = p.x;
        odom_y_ = p.y;
        odom_yaw_ = std::atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z));
        if (!msg->header.frame_id.empty()) {
            odom_frame_ = msg->header.frame_id;
        }
        have_odom_ = true;
    }

    void tick() {
        if (finished_ || !have_map_ || !have_odom_) {
            return;
        }

        double map_x = 0.0;
        double map_y = 0.0;
        double map_yaw = 0.0;
        if (!getRobotPose(map_x, map_y, map_yaw)) {
            return;  // 等待 map -> odom 变换
        }
        robot_x_ = map_x;
        robot_y_ = map_y;
        robot_yaw_ = map_yaw;

        const rclcpp::Time now = this->now();

        if (moving_) {
            const double dist = std::hypot(goal_x_ - robot_x_, goal_y_ - robot_y_);
            if (dist <= goal_reached_distance_) {
                RCLCPP_INFO(get_logger(),
                            "到达探索目标 (%.2f, %.2f)，加入已访问列表，等待地图更新",
                            goal_x_, goal_y_);
                // 已到达的边界不再重复选择，避免在墙边小缺口之间来回摆动
                blacklist_.emplace_back(goal_x_, goal_y_);
                moving_ = false;
                cooldown_until_ = now + rclcpp::Duration::from_seconds(cooldown_);
            } else {
                if (dist < last_distance_ - 0.15) {
                    last_distance_ = dist;
                    last_progress_time_ = now;
                }

                // 只要还在靠近目标就一直走；真正卡住（无进展）才放弃
                const bool stalled =
                    (now - last_progress_time_).seconds() >= progress_timeout_;
                const bool hard_timeout =
                    (now - goal_sent_time_).seconds() >= goal_timeout_;

                if (stalled || hard_timeout) {
                    RCLCPP_WARN(get_logger(),
                                "前往探索目标 (%.2f, %.2f) %s，加入黑名单",
                                goal_x_, goal_y_,
                                stalled ? "长时间没有进展" : "超过总时限");
                    blacklist_.emplace_back(goal_x_, goal_y_);
                    moving_ = false;
                    cooldown_until_ = now + rclcpp::Duration::from_seconds(0.5);
                } else {
                    return;  // 还在前往目标
                }
            }
        }

        if (now < cooldown_until_) {
            return;
        }

        auto clusters = findClusters();

        std::vector<Cluster> candidates;
        for (auto& cluster : clusters) {
            if (!isBlacklisted(cluster.nearest_x, cluster.nearest_y)) {
                candidates.push_back(std::move(cluster));
            }
        }

        if (candidates.empty()) {
            if (!map_has_enough_data()) {
                if (!waiting_logged_) {
                    waiting_logged_ = true;
                    RCLCPP_INFO(get_logger(), "地图尚未建立，等待激光数据...");
                }
                return;
            }
            finished_ = true;
            publishMarkers(candidates, -1);
            RCLCPP_INFO(get_logger(),
                        "探索完成：没有剩余可到达的未知边界（黑名单 %zu 个）",
                        blacklist_.size());
            return;
        }

        // 选择“距离 + 朝向偏差”最小的边界：优先继续朝当前方向探索，
        // 减少每次到点后 180 度掉头，整体建图更快。
        const Cluster* best = nullptr;
        double best_score = std::numeric_limits<double>::max();
        for (const auto& cluster : candidates) {
            const double heading = std::atan2(
                cluster.nearest_y - robot_y_, cluster.nearest_x - robot_x_);
            const double diff = std::abs(std::atan2(
                std::sin(heading - robot_yaw_), std::cos(heading - robot_yaw_)));
            const double score = cluster.nearest_dist + heading_weight_ * diff;
            if (score < best_score) {
                best_score = score;
                best = &cluster;
            }
        }

        goal_x_ = best->nearest_x;
        goal_y_ = best->nearest_y;
        goal_sent_time_ = now;
        last_progress_time_ = now;
        last_distance_ = best->nearest_dist;
        moving_ = true;

        publishGoal(*best);
        publishMarkers(candidates, static_cast<int>(best - candidates.data()));

        RCLCPP_INFO(get_logger(),
                    "选择探索目标 (%.2f, %.2f)，距机器人 %.2f m，"
                    "边界簇 %zu 格，候选 %zu 个",
                    goal_x_, goal_y_, best->nearest_dist,
                    best->cells.size(), candidates.size());
    }

    std::vector<Cluster> findClusters() const {
        std::vector<Cluster> clusters;
        if (!have_map_) {
            return clusters;
        }

        const int width = static_cast<int>(map_.info.width);
        const int height = static_cast<int>(map_.info.height);
        if (width <= 0 || height <= 0) {
            return clusters;
        }

        const std::size_t total = static_cast<std::size_t>(width) * height;
        const auto& data = map_.data;

        // 1. 标出前沿格：本身空闲，且 8 邻域里至少一个未知格
        std::vector<uint8_t> frontier(total, 0);
        static constexpr int kDx[8] = {1, -1, 0, 0, 1, 1, -1, -1};
        static constexpr int kDy[8] = {0, 0, 1, -1, 1, -1, 1, -1};

        for (int row = 0; row < height; ++row) {
            for (int col = 0; col < width; ++col) {
                const std::size_t index = static_cast<std::size_t>(row) * width + col;
                if (data[index] != 0) {
                    continue;
                }
                for (int k = 0; k < 8; ++k) {
                    const int nc = col + kDx[k];
                    const int nr = row + kDy[k];
                    // slam_toolbox 的地图会裁剪到已知区域，栅格外就是未知，
                    // 因此越界邻居也要算作前沿，否则永远看不到地图边缘。
                    if (nc < 0 || nc >= width || nr < 0 || nr >= height) {
                        frontier[index] = 1;
                        break;
                    }
                    if (data[static_cast<std::size_t>(nr) * width + nc] == -1) {
                        frontier[index] = 1;
                        break;
                    }
                }
            }
        }

        // 2. 8 邻接聚类
        std::vector<uint8_t> visited(total, 0);
        for (int row = 0; row < height; ++row) {
            for (int col = 0; col < width; ++col) {
                const std::size_t start = static_cast<std::size_t>(row) * width + col;
                if (!frontier[start] || visited[start]) {
                    continue;
                }

                Cluster cluster;
                std::vector<int> stack;
                stack.push_back(static_cast<int>(start));
                visited[start] = 1;

                double sum_x = 0.0;
                double sum_y = 0.0;

                while (!stack.empty()) {
                    const int index = stack.back();
                    stack.pop_back();
                    cluster.cells.push_back(index);

                    double wx = 0.0;
                    double wy = 0.0;
                    cellToWorld(index, wx, wy);
                    sum_x += wx;
                    sum_y += wy;

                    const int c = index % width;
                    const int r = index / width;

                    const double dist = std::hypot(wx - robot_x_, wy - robot_y_);
                    if (hasClearance(c, r) && !isBlacklisted(wx, wy)) {
                        // 优先选离机器人至少 min_target_distance 的边界格，
                        // 避免总被脚边由栅格缝隙产生的“伪边界”吸引。
                        if (dist >= min_target_distance_) {
                            if (dist < cluster.nearest_dist) {
                                cluster.nearest_dist = dist;
                                cluster.nearest_x = wx;
                                cluster.nearest_y = wy;
                            }
                        } else if (dist < cluster.fallback_dist) {
                            cluster.fallback_dist = dist;
                            cluster.fallback_x = wx;
                            cluster.fallback_y = wy;
                        }
                    }

                    for (int k = 0; k < 8; ++k) {
                        const int nc = c + kDx[k];
                        const int nr = r + kDy[k];
                        if (nc < 0 || nc >= width || nr < 0 || nr >= height) {
                            continue;
                        }
                        const int neighbor = nr * width + nc;
                        if (frontier[neighbor] && !visited[neighbor]) {
                            visited[neighbor] = 1;
                            stack.push_back(neighbor);
                        }
                    }
                }

                if (cluster.nearest_dist == std::numeric_limits<double>::max() &&
                    cluster.fallback_dist < std::numeric_limits<double>::max())
                {
                    cluster.nearest_dist = cluster.fallback_dist;
                    cluster.nearest_x = cluster.fallback_x;
                    cluster.nearest_y = cluster.fallback_y;
                }
                const bool has_clear_cell =
                    cluster.nearest_dist < std::numeric_limits<double>::max();
                if (static_cast<int>(cluster.cells.size()) >= min_frontier_size_ &&
                    has_clear_cell)
                {
                    const double count = static_cast<double>(cluster.cells.size());
                    cluster.centroid_x = sum_x / count;
                    cluster.centroid_y = sum_y / count;
                    clusters.push_back(std::move(cluster));
                }
            }
        }

        return clusters;
    }

    bool getRobotPose(double& x, double& y, double& yaw) {
        if (!have_odom_ || !have_map_) {
            return false;
        }

        const std::string map_frame =
            map_.header.frame_id.empty() ? "map" : map_.header.frame_id;
        if (map_frame == odom_frame_ || !tf_buffer_) {
            x = odom_x_;
            y = odom_y_;
            yaw = odom_yaw_;
            return true;
        }

        try {
            const auto tf = tf_buffer_->lookupTransform(
                map_frame, odom_frame_, tf2::TimePointZero);
            const auto& r = tf.transform.rotation;
            const double tf_yaw = std::atan2(
                2.0 * (r.w * r.z + r.x * r.y),
                1.0 - 2.0 * (r.y * r.y + r.z * r.z));

            x = tf.transform.translation.x + std::cos(tf_yaw) * odom_x_ -
                std::sin(tf_yaw) * odom_y_;
            y = tf.transform.translation.y + std::sin(tf_yaw) * odom_x_ +
                std::cos(tf_yaw) * odom_y_;
            yaw = std::atan2(std::sin(tf_yaw + odom_yaw_),
                             std::cos(tf_yaw + odom_yaw_));
            return true;
        } catch (const tf2::TransformException& ex) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                                 "等待 %s -> %s 变换失败：%s",
                                 map_frame.c_str(), odom_frame_.c_str(), ex.what());
            return false;
        }
    }

    bool map_has_enough_data() const {
        if (!have_map_) {
            return false;
        }
        std::size_t known = 0;
        for (const int8_t value : map_.data) {
            if (value != -1) {
                ++known;
            }
            if (known >= static_cast<std::size_t>(min_known_cells_)) {
                return true;
            }
        }
        return false;
    }

    bool hasClearance(int col, int row) const {
        if (!have_map_ || goal_clearance_ <= 0.0) {
            return true;
        }
        const int width = static_cast<int>(map_.info.width);
        const int height = static_cast<int>(map_.info.height);
        const int radius = static_cast<int>(
            std::ceil(goal_clearance_ / map_.info.resolution));
        const int radius_sq = radius * radius;

        for (int dr = -radius; dr <= radius; ++dr) {
            const int nr = row + dr;
            if (nr < 0 || nr >= height) {
                continue;
            }
            for (int dc = -radius; dc <= radius; ++dc) {
                if (dc * dc + dr * dr > radius_sq) {
                    continue;
                }
                const int nc = col + dc;
                if (nc < 0 || nc >= width) {
                    continue;
                }
                if (map_.data[static_cast<std::size_t>(nr) * width + nc] >= 50) {
                    return false;
                }
            }
        }
        return true;
    }

    void cellToWorld(int index, double& x, double& y) const {
        const int width = static_cast<int>(map_.info.width);
        const int col = index % width;
        const int row = index / width;
        x = map_.info.origin.position.x + (col + 0.5) * map_.info.resolution;
        y = map_.info.origin.position.y + (row + 0.5) * map_.info.resolution;
    }

    bool isBlacklisted(double x, double y) const {
        for (const auto& point : blacklist_) {
            if (std::hypot(x - point.first, y - point.second) < blacklist_radius_) {
                return true;
            }
        }
        return false;
    }

    void publishGoal(const Cluster& cluster) {
        geometry_msgs::msg::PoseStamped msg;
        msg.header.stamp = this->now();
        msg.header.frame_id =
            map_.header.frame_id.empty() ? "odom" : map_.header.frame_id;
        msg.pose.position.x = cluster.nearest_x;
        msg.pose.position.y = cluster.nearest_y;
        msg.pose.position.z = 0.0;
        msg.pose.orientation.w = 1.0;
        goal_pub_->publish(msg);
    }

    void publishMarkers(const std::vector<Cluster>& clusters, int chosen) {
        if (!have_map_) {
            return;
        }

        visualization_msgs::msg::MarkerArray array;
        const std::string frame =
            map_.header.frame_id.empty() ? "odom" : map_.header.frame_id;

        visualization_msgs::msg::Marker points;
        points.header.stamp = this->now();
        points.header.frame_id = frame;
        points.ns = "frontiers";
        points.id = 0;
        points.type = visualization_msgs::msg::Marker::POINTS;
        points.action = visualization_msgs::msg::Marker::ADD;
        points.pose.orientation.w = 1.0;
        points.scale.x = 0.06;
        points.scale.y = 0.06;
        points.color.r = 0.1f;
        points.color.g = 1.0f;
        points.color.b = 0.2f;
        points.color.a = 1.0f;
        for (const auto& cluster : clusters) {
            geometry_msgs::msg::Point point;
            point.x = cluster.nearest_x;
            point.y = cluster.nearest_y;
            point.z = 0.05;
            points.points.push_back(point);
        }
        array.markers.push_back(points);

        visualization_msgs::msg::Marker goal;
        goal.header.stamp = points.header.stamp;
        goal.header.frame_id = frame;
        goal.ns = "frontier_goal";
        goal.id = 1;
        goal.type = visualization_msgs::msg::Marker::SPHERE;
        if (chosen >= 0 && chosen < static_cast<int>(clusters.size())) {
            goal.action = visualization_msgs::msg::Marker::ADD;
            goal.pose.position.x = clusters[chosen].nearest_x;
            goal.pose.position.y = clusters[chosen].nearest_y;
            goal.pose.position.z = 0.15;
            goal.pose.orientation.w = 1.0;
            goal.scale.x = 0.25;
            goal.scale.y = 0.25;
            goal.scale.z = 0.25;
            goal.color.r = 1.0f;
            goal.color.g = 0.2f;
            goal.color.b = 0.2f;
            goal.color.a = 0.9f;
        } else {
            goal.action = visualization_msgs::msg::Marker::DELETE;
        }
        array.markers.push_back(goal);

        marker_pub_->publish(array);
    }

    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    std::string map_topic_;
    std::string goal_topic_;
    std::string odom_topic_;
    int min_frontier_size_{3};
    double goal_reached_distance_{0.30};
    double goal_timeout_{180.0};
    double progress_timeout_{8.0};
    double cooldown_{0.5};
    double blacklist_radius_{0.75};
    double goal_clearance_{0.35};
    int min_known_cells_{500};
    double min_target_distance_{2.5};
    double heading_weight_{0.6};

    nav_msgs::msg::OccupancyGrid map_;
    bool have_map_{false};
    bool map_logged_{false};
    bool waiting_logged_{false};
    bool have_odom_{false};
    double odom_x_{0.0};
    double odom_y_{0.0};
    double odom_yaw_{0.0};
    std::string odom_frame_{"odom"};
    double robot_x_{0.0};  // map 系
    double robot_y_{0.0};
    double robot_yaw_{0.0};

    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    bool moving_{false};
    bool finished_{false};
    double goal_x_{0.0};
    double goal_y_{0.0};
    double last_distance_{std::numeric_limits<double>::max()};
    rclcpp::Time goal_sent_time_;
    rclcpp::Time cooldown_until_;
    rclcpp::Time last_progress_time_;
    std::vector<std::pair<double, double>> blacklist_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FrontierExplorer>());
    rclcpp::shutdown();
    return 0;
}
