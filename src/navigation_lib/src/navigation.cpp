#include "navigation/navigation.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdlib>
#include <functional>
#include <limits>
#include <queue>
#include <stdexcept>
#include <vector>

#include <tf2/exceptions.h>
#include <tf2/time.h>

namespace {

double normalizeAngle(double angle) {
    return std::atan2(std::sin(angle), std::cos(angle));
}

}  // namespace

void NavigationModule::init(
    const rclcpp::Node::SharedPtr& node,
    const std::shared_ptr<sensor_interface::SensorNode>& sensor)
{
    node_ = node;
    sensor_ = sensor;
    if (!node_ || !sensor_) {
        throw std::invalid_argument("NavigationModule::init 需要有效的 node 和 sensor");
    }

    robot_radius_ = node_->declare_parameter<double>("robot_radius", robot_radius_);
    start_clear_radius_ =
        node_->declare_parameter<double>("start_clear_radius", start_clear_radius_);
    safety_distance_ =
        node_->declare_parameter<double>("safety_distance", safety_distance_);
    unknown_penalty_ =
        node_->declare_parameter<double>("unknown_penalty", unknown_penalty_);
    goal_tolerance_ = node_->declare_parameter<double>("goal_tolerance", goal_tolerance_);
    max_linear_speed_ =
        node_->declare_parameter<double>("max_linear_speed", max_linear_speed_);
    max_angular_speed_ =
        node_->declare_parameter<double>("max_angular_speed", max_angular_speed_);
    lookahead_distance_ =
        node_->declare_parameter<double>("lookahead_distance", lookahead_distance_);
    control_rate_ = node_->declare_parameter<double>("control_rate", control_rate_);
    replan_period_ = node_->declare_parameter<double>("replan_period", replan_period_);
    goal_search_radius_ =
        node_->declare_parameter<double>("goal_search_radius", goal_search_radius_);
    map_topic_ = node_->declare_parameter<std::string>("nav_map_topic", map_topic_);
    goal_topic_ = node_->declare_parameter<std::string>("goal_topic", goal_topic_);
    cmd_vel_topic_ = node_->declare_parameter<std::string>("cmd_vel_topic", cmd_vel_topic_);
    plan_topic_ = node_->declare_parameter<std::string>("plan_topic", plan_topic_);

    if (control_rate_ <= 0.0 || replan_period_ <= 0.0 || goal_tolerance_ <= 0.0) {
        throw std::invalid_argument("导航参数非法：rate / replan_period / goal_tolerance 必须为正");
    }

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    map_sub_ = node_->create_subscription<nav_msgs::msg::OccupancyGrid>(
        map_topic_, rclcpp::QoS(1).transient_local(),
        std::bind(&NavigationModule::mapCallback, this, std::placeholders::_1));

    goal_sub_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
        goal_topic_, 10,
        std::bind(&NavigationModule::goalCallback, this, std::placeholders::_1));

    path_pub_ = node_->create_publisher<nav_msgs::msg::Path>(plan_topic_, 10);
    cmd_pub_ =
        node_->create_publisher<geometry_msgs::msg::TwistStamped>(cmd_vel_topic_, 10);

    control_timer_ = node_->create_wall_timer(
        std::chrono::duration<double>(1.0 / control_rate_),
        std::bind(&NavigationModule::controlCallback, this));

    last_plan_time_ = node_->now();
    next_plan_attempt_ = last_plan_time_;

    RCLCPP_INFO(node_->get_logger(),
                "NavigationModule 已启动：goal=%s，plan=%s，cmd_vel=%s，"
                "膨胀半径 %.2f m，最大速度 %.2f m/s，前视 %.2f m",
                goal_topic_.c_str(), plan_topic_.c_str(), cmd_vel_topic_.c_str(),
                robot_radius_, max_linear_speed_, lookahead_distance_);
}

void NavigationModule::mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
    map_ = *msg;

    const int width = static_cast<int>(map_.info.width);
    const int height = static_cast<int>(map_.info.height);
    const double resolution = map_.info.resolution;

    blocked_.assign(static_cast<std::size_t>(width) * height, 0);
    const int inflation_cells =
        static_cast<int>(std::ceil(robot_radius_ / resolution));
    const int radius_sq = inflation_cells * inflation_cells;

    // 只遍历占据格，在其周围画膨胀圆，比全图卷积快得多
    for (int row = 0; row < height; ++row) {
        for (int col = 0; col < width; ++col) {
            const std::size_t index = static_cast<std::size_t>(row) * width + col;
            if (map_.data[index] < 50) {
                continue;
            }
            for (int dy = -inflation_cells; dy <= inflation_cells; ++dy) {
                const int nr = row + dy;
                if (nr < 0 || nr >= height) {
                    continue;
                }
                for (int dx = -inflation_cells; dx <= inflation_cells; ++dx) {
                    if (dx * dx + dy * dy > radius_sq) {
                        continue;
                    }
                    const int nc = col + dx;
                    if (nc < 0 || nc >= width) {
                        continue;
                    }
                    blocked_[static_cast<std::size_t>(nr) * width + nc] = 1;
                }
            }
        }
    }

    map_ready_ = true;
}

void NavigationModule::goalCallback(
    const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
    if (!map_ready_) {
        // 先把目标存下来，等 /map 到达后由控制定时器继续规划
        RCLCPP_INFO(node_->get_logger(),
                    "收到目标 (%.2f, %.2f)，等待地图后开始规划",
                    msg->pose.position.x, msg->pose.position.y);
    } else if (!msg->header.frame_id.empty() &&
               !map_.header.frame_id.empty() &&
               msg->header.frame_id != map_.header.frame_id)
    {
        RCLCPP_WARN(node_->get_logger(),
                    "目标坐标系 %s 与地图坐标系 %s 不一致，目标坐标将按地图坐标系处理",
                    msg->header.frame_id.c_str(), map_.header.frame_id.c_str());
    }

    goal_ = *msg;
    have_goal_ = true;
    path_.clear();
    path_index_ = 0;
    next_plan_attempt_ = node_->now();

    RCLCPP_INFO(node_->get_logger(), "收到目标：(%.2f, %.2f)",
                goal_.pose.position.x, goal_.pose.position.y);
}

void NavigationModule::controlCallback() {
    if (!have_goal_) {
        return;
    }

    double x = 0.0;
    double y = 0.0;
    double yaw = 0.0;
    if (!getRobotPose(x, y, yaw)) {
        return;
    }

    const double goal_dist = std::hypot(
        goal_.pose.position.x - x, goal_.pose.position.y - y);

    if (goal_dist <= goal_tolerance_) {
        RCLCPP_INFO(node_->get_logger(), "到达目标：(%.2f, %.2f)", x, y);
        have_goal_ = false;
        path_.clear();
        publishStop();
        return;
    }

    const rclcpp::Time now = node_->now();
    const bool need_plan = path_.empty() ||
        (now - last_plan_time_).seconds() >= replan_period_;

    if (need_plan && now >= next_plan_attempt_) {
        if (!planPath(goal_)) {
            next_plan_attempt_ = now + rclcpp::Duration::from_seconds(1.0);
            publishStop();
            return;
        }
    }

    if (path_.empty()) {
        publishStop();
        return;
    }

    // 跳过已经走过的路径点
    while (path_index_ + 1 < path_.size()) {
        const auto& p = path_[path_index_].pose.position;
        if (std::hypot(p.x - x, p.y - y) >= 0.10) {
            break;
        }
        ++path_index_;
    }

    // 选取前视点：从当前路径点开始，第一个距离超过 lookahead 的点
    std::size_t target_index = path_.size() - 1;
    for (std::size_t i = path_index_; i < path_.size(); ++i) {
        const auto& p = path_[i].pose.position;
        if (std::hypot(p.x - x, p.y - y) >= lookahead_distance_) {
            target_index = i;
            break;
        }
    }

    const auto& target = path_[target_index].pose.position;
    const double heading_error = normalizeAngle(
        std::atan2(target.y - y, target.x - x) - yaw);

    double linear = 0.0;
    if (std::abs(heading_error) < 0.6) {
        linear = max_linear_speed_ * std::cos(heading_error);
        // 接近目标时减速
        const double slow_radius = 0.6;
        if (goal_dist < slow_radius) {
            linear *= std::max(0.25, goal_dist / slow_radius);
        }
    }

    double angular = std::clamp(
        1.5 * heading_error, -max_angular_speed_, max_angular_speed_);

    // 激光前向安全保护：贴近障碍时只允许原地转向，避免顶着墙走。
    // 顶着墙会轮子打滑，让仿真里程计快速漂移，进而毁掉地图。
    auto scan = sensor_->getLatestScan();
    if (scan && !scan->ranges.empty()) {
        constexpr double kFrontHalfAngle = 0.40;  // rad
        double front_min = std::numeric_limits<double>::infinity();
        double left_min = std::numeric_limits<double>::infinity();
        double right_min = std::numeric_limits<double>::infinity();

        for (std::size_t i = 0; i < scan->ranges.size(); ++i) {
            const double range = scan->ranges[i];
            if (!std::isfinite(range) || range < scan->range_min) {
                continue;
            }
            const double raw_angle =
                scan->angle_min + static_cast<double>(i) * scan->angle_increment;
            const double angle = normalizeAngle(raw_angle);

            if (std::abs(angle) <= kFrontHalfAngle) {
                front_min = std::min(front_min, range);
            }
            if (angle > 0.2 && angle < 1.2) {
                left_min = std::min(left_min, range);
            }
            if (angle < -0.2 && angle > -1.2) {
                right_min = std::min(right_min, range);
            }
        }

        if (front_min < safety_distance_) {
            linear = 0.0;
            angular = (left_min >= right_min ? 1.0 : -1.0) * max_angular_speed_;
            RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
                                 "前方 %.2f m 有障碍，暂停前进并转向", front_min);
        }
    }

    publishCommand(linear, angular);
}

bool NavigationModule::getRobotPose(double& x, double& y, double& yaw) const {
    auto odom = sensor_->getLatestOdom();
    if (!odom) {
        return false;
    }

    const auto& p = odom->pose.pose.position;
    const auto& q = odom->pose.pose.orientation;
    const double odom_yaw = std::atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z));

    const std::string odom_frame =
        odom->header.frame_id.empty() ? "odom" : odom->header.frame_id;
    const std::string map_frame =
        map_.header.frame_id.empty() ? "map" : map_.header.frame_id;

    // odom_grid 后端发布 map->odom 的静态恒等变换；slam_toolbox 发布动态变换。
    // 两种情况下坐标都能通过 TF 统一到 map 系；同系时直接使用 odom。
    if (!map_ready_ || map_frame == odom_frame || !tf_buffer_) {
        x = p.x;
        y = p.y;
        yaw = odom_yaw;
        return true;
    }

    try {
        const auto tf = tf_buffer_->lookupTransform(
            map_frame, odom_frame, tf2::TimePointZero);
        const auto& r = tf.transform.rotation;
        const double tf_yaw = std::atan2(
            2.0 * (r.w * r.z + r.x * r.y),
            1.0 - 2.0 * (r.y * r.y + r.z * r.z));

        x = tf.transform.translation.x + std::cos(tf_yaw) * p.x -
            std::sin(tf_yaw) * p.y;
        y = tf.transform.translation.y + std::sin(tf_yaw) * p.x +
            std::cos(tf_yaw) * p.y;
        yaw = normalizeAngle(tf_yaw + odom_yaw);
        return true;
    } catch (const tf2::TransformException& ex) {
        RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
                             "等待 %s -> %s 变换失败：%s",
                             map_frame.c_str(), odom_frame.c_str(), ex.what());
        return false;
    }
}

bool NavigationModule::planPath(const geometry_msgs::msg::PoseStamped& goal) {
    if (!map_ready_) {
        return false;
    }

    double robot_x = 0.0;
    double robot_y = 0.0;
    double robot_yaw = 0.0;
    if (!getRobotPose(robot_x, robot_y, robot_yaw)) {
        return false;
    }
    (void)robot_yaw;

    int start_col = 0;
    int start_row = 0;
    if (!worldToCell(robot_x, robot_y, start_col, start_row)) {
        RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000,
                             "机器人当前位置 (%.2f, %.2f) 在地图外",
                             robot_x, robot_y);
        return false;
    }

    // 记录规划起点，isTraversable() 会放行起点附近的一段膨胀区
    plan_robot_col_ = start_col;
    plan_robot_row_ = start_row;
    start_clear_cells_ =
        static_cast<int>(std::ceil(start_clear_radius_ / map_.info.resolution));

    int goal_col = 0;
    int goal_row = 0;
    if (!worldToCell(goal.pose.position.x, goal.pose.position.y, goal_col, goal_row)) {
        RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000,
                             "目标 (%.2f, %.2f) 在地图外",
                             goal.pose.position.x, goal.pose.position.y);
        return false;
    }

    // 目标落在障碍 / 未知 / 膨胀区时，吸附到最近的可通行格
    if (!isTraversable(goal_col, goal_row)) {
        int adjusted_col = 0;
        int adjusted_row = 0;
        if (!findNearestTraversable(goal_col, goal_row, adjusted_col, adjusted_row)) {
            RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000,
                                 "目标 (%.2f, %.2f) 附近 %.1f m 内没有可通行区域",
                                 goal.pose.position.x, goal.pose.position.y,
                                 goal_search_radius_);
            return false;
        }
        goal_col = adjusted_col;
        goal_row = adjusted_row;

        double adjusted_x = 0.0;
        double adjusted_y = 0.0;
        cellCenter(cellIndex(goal_col, goal_row), adjusted_x, adjusted_y);
        goal_.pose.position.x = adjusted_x;
        goal_.pose.position.y = adjusted_y;
    }

    const int start_index = cellIndex(start_col, start_row);
    const int goal_index = cellIndex(goal_col, goal_row);

    std::vector<int> cells = searchAStar(start_index, goal_index);
    if (cells.empty()) {
        RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000,
                             "A* 未找到从 (%d,%d) 到 (%d,%d) 的路径",
                             start_col, start_row, goal_col, goal_row);
        return false;
    }

    // 视线简化：能直线到达的中间点全部跳过
    std::vector<int> simplified;
    simplified.push_back(cells.front());
    std::size_t i = 0;
    while (i + 1 < cells.size()) {
        std::size_t j = cells.size() - 1;
        for (; j > i + 1; --j) {
            if (lineOfSight(cells[i], cells[j])) {
                break;
            }
        }
        simplified.push_back(cells[j]);
        i = j;
    }

    const std::string frame = map_.header.frame_id.empty() ? "odom" : map_.header.frame_id;
    const rclcpp::Time stamp = node_->now();

    path_.clear();
    path_.reserve(simplified.size());
    for (std::size_t k = 0; k < simplified.size(); ++k) {
        double cell_x = 0.0;
        double cell_y = 0.0;
        cellCenter(simplified[k], cell_x, cell_y);

        geometry_msgs::msg::PoseStamped pose;
        pose.header.stamp = stamp;
        pose.header.frame_id = frame;
        pose.pose.position.x = cell_x;
        pose.pose.position.y = cell_y;
        pose.pose.position.z = 0.0;

        double heading = 0.0;
        if (k + 1 < simplified.size()) {
            double next_x = 0.0;
            double next_y = 0.0;
            cellCenter(simplified[k + 1], next_x, next_y);
            heading = std::atan2(next_y - cell_y, next_x - cell_x);
        } else if (k > 0) {
            heading = std::atan2(
                cell_y - path_.back().pose.position.y,
                cell_x - path_.back().pose.position.x);
        }
        pose.pose.orientation.z = std::sin(heading / 2.0);
        pose.pose.orientation.w = std::cos(heading / 2.0);

        path_.push_back(pose);
    }

    path_index_ = 0;
    last_plan_time_ = stamp;
    next_plan_attempt_ = stamp;
    publishPath();

    RCLCPP_INFO(node_->get_logger(),
                "规划成功：栅格路径 %zu 格 -> 简化后 %zu 个路径点",
                cells.size(), path_.size());
    return true;
}

std::vector<int> NavigationModule::searchAStar(int start_index, int goal_index) const {
    const int width = static_cast<int>(map_.info.width);
    const int height = static_cast<int>(map_.info.height);
    const std::size_t total = static_cast<std::size_t>(width) * height;
    if (start_index < 0 || goal_index < 0 ||
        static_cast<std::size_t>(start_index) >= total ||
        static_cast<std::size_t>(goal_index) >= total)
    {
        return {};
    }
    if (start_index == goal_index) {
        return {start_index};
    }

    const int goal_col = goal_index % width;
    const int goal_row = goal_index / width;

    const float inf = std::numeric_limits<float>::infinity();
    std::vector<float> g_score(total, inf);
    std::vector<int> parent(total, -1);
    std::vector<uint8_t> closed(total, 0);

    auto heuristic = [goal_col, goal_row, width](int index) {
        const double dx = static_cast<double>(index % width - goal_col);
        const double dy = static_cast<double>(index / width - goal_row);
        return static_cast<float>(std::hypot(dx, dy));
    };

    using QueueItem = std::pair<float, int>;  // (f_score, index)
    std::priority_queue<QueueItem, std::vector<QueueItem>, std::greater<QueueItem>> open;

    g_score[start_index] = 0.0f;
    open.push({heuristic(start_index), start_index});

    static constexpr int kDx[8] = {1, -1, 0, 0, 1, 1, -1, -1};
    static constexpr int kDy[8] = {0, 0, 1, -1, 1, -1, 1, -1};

    while (!open.empty()) {
        const auto [f_score, current] = open.top();
        open.pop();

        if (closed[current]) {
            continue;
        }
        closed[current] = 1;

        if (current == goal_index) {
            break;
        }

        const int col = current % width;
        const int row = current / width;

        for (int k = 0; k < 8; ++k) {
            const int next_col = col + kDx[k];
            const int next_row = row + kDy[k];
            if (!isTraversable(next_col, next_row)) {
                continue;
            }
            // 禁止斜穿墙角
            if (kDx[k] != 0 && kDy[k] != 0 &&
                (!isTraversable(col + kDx[k], row) || !isTraversable(col, row + kDy[k])))
            {
                continue;
            }

            const int next_index = cellIndex(next_col, next_row);
            float step = (kDx[k] != 0 && kDy[k] != 0)
                ? 1.41421356f
                : 1.0f;
            if (map_.data[next_index] == -1) {
                step *= static_cast<float>(1.0 + unknown_penalty_);
            }
            const float tentative = g_score[current] + step;
            if (tentative < g_score[next_index]) {
                g_score[next_index] = tentative;
                parent[next_index] = current;
                open.push({tentative + heuristic(next_index), next_index});
            }
        }

        (void)f_score;
    }

    if (parent[goal_index] == -1) {
        return {};
    }

    std::vector<int> path;
    for (int current = goal_index; current != -1; current = parent[current]) {
        path.push_back(current);
        if (current == start_index) {
            break;
        }
    }
    if (path.empty() || path.back() != start_index) {
        return {};
    }
    std::reverse(path.begin(), path.end());
    return path;
}

bool NavigationModule::lineOfSight(int from_index, int to_index) const {
    const int width = static_cast<int>(map_.info.width);
    int x0 = from_index % width;
    int y0 = from_index / width;
    const int x1 = to_index % width;
    const int y1 = to_index / width;

    const int dx = std::abs(x1 - x0);
    const int dy = -std::abs(y1 - y0);
    const int sx = (x0 < x1) ? 1 : -1;
    const int sy = (y0 < y1) ? 1 : -1;
    int err = dx + dy;

    while (!(x0 == x1 && y0 == y1)) {
        const int e2 = 2 * err;
        if (e2 >= dy) {
            err += dy;
            x0 += sx;
        }
        if (e2 <= dx) {
            err += dx;
            y0 += sy;
        }
        if (!(x0 == x1 && y0 == y1) && !isTraversable(x0, y0)) {
            return false;
        }
    }
    return true;
}

bool NavigationModule::findNearestTraversable(
    int col, int row, int& out_col, int& out_row) const
{
    if (!map_ready_) {
        return false;
    }

    const int width = static_cast<int>(map_.info.width);
    const int height = static_cast<int>(map_.info.height);
    const double resolution = map_.info.resolution;
    const int max_radius = static_cast<int>(std::ceil(goal_search_radius_ / resolution));

    bool found = false;
    double best_dist_sq = std::numeric_limits<double>::max();

    for (int dr = -max_radius; dr <= max_radius; ++dr) {
        for (int dc = -max_radius; dc <= max_radius; ++dc) {
            const int nc = col + dc;
            const int nr = row + dr;
            if (nc < 0 || nc >= width || nr < 0 || nr >= height) {
                continue;
            }
            if (!isTraversable(nc, nr)) {
                continue;
            }
            const double dist_sq = static_cast<double>(dc * dc + dr * dr);
            if (dist_sq < best_dist_sq) {
                best_dist_sq = dist_sq;
                out_col = nc;
                out_row = nr;
                found = true;
            }
        }
    }

    return found;
}

bool NavigationModule::isTraversable(int col, int row) const {
    if (!map_ready_ || !inBounds(col, row)) {
        return false;
    }
    const std::size_t index =
        static_cast<std::size_t>(row) * map_.info.width + col;
    if (blocked_[index] == 0) {
        // 未知格 (-1) 也允许通行，但在 A* 里会加额外代价。
        // 否则 slam_toolbox 单帧地图里射线之间的未知缝隙会把房间切碎，
        // 机器人永远规划不出路径（激光安全保护负责防止真的撞墙）。
        return true;
    }
    // 膨胀区通常不可通行，但机器人当前所在位置附近要放行，
    // 否则一旦靠墙（或地图有噪声）就永远规划不出路径。
    if (start_clear_cells_ > 0 && plan_robot_col_ >= 0 && plan_robot_row_ >= 0) {
        const int dc = col - plan_robot_col_;
        const int dr = row - plan_robot_row_;
        if (dc * dc + dr * dr <= start_clear_cells_ * start_clear_cells_) {
            return true;
        }
    }
    return false;
}

bool NavigationModule::inBounds(int col, int row) const {
    return col >= 0 && row >= 0 &&
           col < static_cast<int>(map_.info.width) &&
           row < static_cast<int>(map_.info.height);
}

int NavigationModule::cellIndex(int col, int row) const {
    return row * static_cast<int>(map_.info.width) + col;
}

void NavigationModule::cellCenter(int index, double& x, double& y) const {
    const int width = static_cast<int>(map_.info.width);
    const int col = index % width;
    const int row = index / width;
    x = map_.info.origin.position.x +
        (col + 0.5) * map_.info.resolution;
    y = map_.info.origin.position.y +
        (row + 0.5) * map_.info.resolution;
}

bool NavigationModule::worldToCell(double x, double y, int& col, int& row) const {
    col = static_cast<int>(
        std::floor((x - map_.info.origin.position.x) / map_.info.resolution));
    row = static_cast<int>(
        std::floor((y - map_.info.origin.position.y) / map_.info.resolution));
    return inBounds(col, row);
}

void NavigationModule::publishPath() {
    if (path_.empty()) {
        return;
    }
    nav_msgs::msg::Path msg;
    msg.header = path_.front().header;
    msg.header.stamp = node_->now();
    msg.poses = path_;
    path_pub_->publish(msg);
}

void NavigationModule::publishCommand(double linear, double angular) {
    geometry_msgs::msg::TwistStamped msg;
    msg.header.stamp = node_->now();
    msg.header.frame_id = "base_link";
    msg.twist.linear.x = linear;
    msg.twist.angular.z = angular;
    cmd_pub_->publish(msg);
}

void NavigationModule::publishStop() {
    publishCommand(0.0, 0.0);
}
