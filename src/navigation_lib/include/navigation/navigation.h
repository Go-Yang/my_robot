#ifndef NAVIGATION_NAVIGATION_H
#define NAVIGATION_NAVIGATION_H

#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "sensor_interface/sensor_node.h"

/**
 * 基于占据栅格的全局导航。
 *
 * 数据流：
 *   /map ──> 障碍膨胀 ──> A* ──> /plan
 *   /goal_pose ─────────────┘
 *   /odom ──> 路径跟踪 ──> /cmd_vel (geometry_msgs/TwistStamped)
 *
 * 约定：
 * - 地图与机器人位姿都在同一坐标系（当前为 odom），不做 TF 变换。
 * - 未知栅格视为不可通行；目标点落在障碍/未知区域时会自动吸附到最近的可通行格。
 * - 周期性重规划（默认 2 s），地图更新后也会触发重规划。
 */
class NavigationModule {
public:
    /**
     * @param node   导航节点
     * @param sensor 已经 init() 过的传感器模块，用于取最新 odom
     */
    void init(const rclcpp::Node::SharedPtr& node,
              const std::shared_ptr<sensor_interface::SensorNode>& sensor);

private:
    void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
    void goalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
    void controlCallback();

    bool planPath(const geometry_msgs::msg::PoseStamped& goal);
    std::vector<int> searchAStar(int start_index, int goal_index) const;
    bool lineOfSight(int from_index, int to_index) const;

    /// 机器人位姿（map 系）。地图与里程计同系时直接用 odom，否则查 TF。
    bool getRobotPose(double& x, double& y, double& yaw) const;

    bool findNearestTraversable(int col, int row, int& out_col, int& out_row) const;
    bool isTraversable(int col, int row) const;
    bool inBounds(int col, int row) const;

    int cellIndex(int col, int row) const;
    void cellCenter(int index, double& x, double& y) const;
    bool worldToCell(double x, double y, int& col, int& row) const;

    void publishPath();
    void publishCommand(double linear, double angular);
    void publishStop();

    rclcpp::Node::SharedPtr node_;
    std::shared_ptr<sensor_interface::SensorNode> sensor_;

    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_pub_;
    rclcpp::TimerBase::SharedPtr control_timer_;

    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    // 参数
    double robot_radius_{0.25};       // 障碍膨胀半径（m）
    double start_clear_radius_{0.30}; // 规划时允许机器人附近这段膨胀区通行（脱困用）
    double safety_distance_{0.40};    // 激光前向安全距离（m），低于此值只转向不前进
    double slow_distance_{1.00};      // 前方障碍小于此距离时线性减速（m）
    double unknown_penalty_{2.0};     // 未知格额外代价倍数（0 表示与空闲同价）
    double goal_tolerance_{0.15};     // 到达判定距离（m）
    double max_linear_speed_{0.30};   // m/s（仿真里可超过 burger 硬件上限）
    double max_angular_speed_{0.9};   // rad/s（降低角速度，保证 SLAM 扫描匹配稳定）
    double lookahead_distance_{0.32}; // 前视距离（m）
    double control_rate_{20.0};       // Hz
    double replan_period_{1.0};       // s
    double goal_search_radius_{1.5};  // 目标吸附搜索半径（m）
    std::string map_topic_{"/map"};
    std::string goal_topic_{"/goal_pose"};
    std::string cmd_vel_topic_{"/cmd_vel"};
    std::string plan_topic_{"/plan"};

    // 地图与障碍膨胀
    nav_msgs::msg::OccupancyGrid map_;
    std::vector<uint8_t> blocked_;  // 1 = 占据或落在膨胀区内
    bool map_ready_{false};

    // 规划起点附近临时放行的范围（避免被地图噪声/膨胀区困住）
    int plan_robot_col_{-1};
    int plan_robot_row_{-1};
    int start_clear_cells_{0};

    // 目标与路径（只在导航节点回调中访问，回调默认互斥）
    bool have_goal_{false};
    geometry_msgs::msg::PoseStamped goal_;
    std::vector<geometry_msgs::msg::PoseStamped> path_;
    std::size_t path_index_{0};
    rclcpp::Time last_plan_time_;
    rclcpp::Time next_plan_attempt_;

    // 受阻避让状态（防止在障碍前左右来回抖动）
    bool was_blocked_{false};
    int consecutive_blocks_{0};
    rclcpp::Time blocked_since_;
    rclcpp::Time avoid_until_;
    rclcpp::Time last_block_time_;
    double avoid_linear_{0.0};
    double avoid_angular_{0.0};
};

#endif  // NAVIGATION_NAVIGATION_H
