# my_robot 参数列表

按“机器人本体 → 各节点 ROS 参数 → slam_toolbox → launch 参数”整理。
默认值以当前代码/配置为准；**大部分参数在节点 init 时读取一次，修改后需要重启仿真**。

---

## 1. 机器人本体（TurtleBot 4 Lite 默认 / TurtleBot3 回退）

### TurtleBot 4 Lite（默认，`robot:=turtlebot4_lite`）

模型文件：[urdf/turtlebot4_lite_nocam.urdf.xacro](../src/my_robot_bringup/urdf/turtlebot4_lite_nocam.urdf.xacro)；
后处理脚本：[scripts/tb4_urdf.py](../src/my_robot_bringup/scripts/tb4_urdf.py)。

| 项目 | 值 | 说明 |
|---|---|---|
| 底盘 | iRobot Create3，2 轮差速 + 前万向轮 | |
| 轮距 / 轮半径 | 0.233 m / 0.03575 m | `irobot_create_control/config/control.yaml` |
| 里程计频率 / frame | 62 Hz，`odom → base_link` | `diff_drive_controller` |
| 激光更新率 | **10 Hz** | `world_lidar.py` CPU 射线投射，不依赖 gz 渲染 |
| 激光采样数 | 720 | 0.5° 角分辨率，0 ~ 2π（以底盘 +x 为 0） |
| 激光量程 | 0.164 ~ 12.0 m | 与 RPLIDAR A1M8 一致 |
| 激光坐标系/安装位 | `rplidar_link`，相对 base_link `(0.0039, 0, 0.1395)` | link/TF 保留，合成扫描按 0° 对齐 |
| IMU | 62 Hz，frame `turtlebot4/imu_link/imu` | 由 gz 桥接补充 |
| 默认出生位姿 | `(-3.5, 2.0, 0.0)`，yaw=0 | Room A |
| 底盘控制器速度上限 | 0.46 m/s，1.9 rad/s | 导航实际限制：TB4/odom_grid 与 slam_toolbox 均为 **0.15 / 0.5** |
| 仿真裁剪 | 去掉 OAK-D、11 个悬崖/红外 gpu_lidar、相机支架网格 | 见脚本头部说明 |

> TB4 已经不用 gz 的 gpu_lidar（激光由 `world_lidar.py` 合成），server 端不需要
> 强制软件渲染；但 gz GUI 和 RViz 需要 `LIBGL_ALWAYS_SOFTWARE=1` 才能在无 3D
> 加速的 VM 上正常显示。launch 里按进程单独设置。

### TurtleBot3（回退，`robot:=turtlebot3`）

模型文件：[models/turtlebot3_burger/model.sdf](../src/my_robot_bringup/models/turtlebot3_burger/model.sdf)

| 项目 | 值 | 说明 |
|---|---|---|
| 底盘碰撞盒 | 0.14 × 0.14 × 0.14 m | base_link |
| 轮距 | 0.160 m | DiffDrive |
| 轮半径 | 0.033 m | DiffDrive |
| 最大线加速度 | 1.0 m/s² | DiffDrive |
| 里程计发布频率 | 30 Hz | DiffDrive |
| 激光更新率 | **10 Hz** | 自定义（原版 5 Hz） |
| 激光采样数 | 360 | 0 ~ 2π |
| 激光量程 | **0.12 ~ 12.0 m** | 自定义（原版 0.12 ~ 3.5 m） |
| 激光分辨率/噪声 | 0.015 m / σ=0.01 m | gpu_lidar |
| 激光坐标系/安装位 | `base_scan`，相对 base_link `(-0.032, 0, 0.171)` | |
| IMU | 200 Hz，frame `imu_link` | |
| 默认出生位姿 | `(-3.5, 2.0, 0.01)`，yaw=0 | Room A |
| 规划用机器人半径 | 0.25 m | 导航膨胀半径，不是物理尺寸 |
| 底盘硬件速度上限 | ≈0.22 m/s，≈2.84 rad/s | burger 官方指标；仿真空闲时可超 |

> 改 TB3 激光参数就改 `model.sdf`；TB4 的模型适配集中在 `tb4_urdf.py` 和
> `urdf/` 下，改完需要重启仿真（`robot_state_publisher` 每次启动重新生成）。

---

## 2. `sensor_node`（`sensor_interface_lib`）

| 参数 | 默认 | 说明 |
|---|---|---|
| `scan_topic` | `/scan` | 激光话题 |
| `odom_topic` | `/odom` | 里程计话题 |
| `imu_topic` | `/imu` | IMU 话题 |

### `world_lidar`（TB4 默认，`my_robot_bringup/scripts/world_lidar.py`）

| 参数 | 默认 | 说明 |
|---|---|---|
| `world_file` | 必填 | world SDF 绝对路径 |
| `pose_topic` | `/sim_ground_truth_pose` | Gazebo 真值位姿（nav_msgs/Odometry） |
| `scan_topic` | `/scan` | 输出激光 |
| `frame_id` | `rplidar_link` | 扫描坐标系 |
| `rate` | `10.0` | Hz |
| `num_rays` | `720` | 角度 0 ~ 2π，0 = 底盘 +x（0.5° 角分辨率） |
| `range_min` / `range_max` | `0.164` / `12.0` | m |
| `lidar_height` | `0.1395` | 只选取与该高度相交的 box |

> 这是**仿真专用**的 CPU 射线投射：解析 world 里的静态 box 墙，结合真值位姿
> 生成 LaserScan，绕开 llvmpipe 下不可靠的 gpu_lidar。只适用于当前这种由 box
> 墙组成的世界，不模拟动态障碍。`scan_filter.py` 保留作为恢复 gz 雷达时的备选。

订阅后缓存最新帧；每 5 s 打印一次接收速率。

---

## 3. `mapping_node`（`mapping_lib`）

### 3.1 通用（facade）

| 参数 | 默认 | 说明 |
|---|---|---|
| `mapping_backend` | `odom_grid` | `odom_grid` / `slam_toolbox` / `custom_slam` |

### 3.2 `odom_grid` 后端

| 参数 | 默认 | 说明 |
|---|---|---|
| `resolution` | `0.05` | m / 格 |
| `width` / `height` | `400` / `400` | 格数（20 m × 20 m） |
| `origin_x` / `origin_y` | `-10.0` / `-10.0` | 地图左下角（odom 系） |
| `map_frame` / `odom_frame` | `map` / `odom` | 坐标系 |
| `map_topic` | `/map` | 发布话题 |
| `free_log_odds` | `-0.45` | 空闲更新量 |
| `occupied_log_odds` | `0.9` | 占据更新量 |
| `min_log_odds` / `max_log_odds` | `-3.0` / `3.5` | 截断范围 |
| `occupied_threshold` | `0.4` | 占据/空闲判定阈值 |
| `update_rate` / `publish_rate` | `20` / `2` | Hz |

> 无回波（`inf` 或超过 `range_max`）的射线会**直接跳过**，不再按最大量程标记空闲。
> 否则 TB4 软渲染雷达的坏帧会把已建好的墙擦掉。

### 3.3 `slam_toolbox` 后端

| 参数 | 默认 | 说明 |
|---|---|---|
| `slam_map_topic` | `/map` | 订阅外部 slam_toolbox 的地图并缓存 |

实际建图参数在 [config/slam_toolbox.yaml](../src/my_robot_bringup/config/slam_toolbox.yaml)，见第 5 节。

### 3.4 `custom_slam` 后端

自研 SLAM 占位，没有可调参数；选中会打印警告且不产生地图。

---

## 4. `navigation_node`（`navigation_lib`）

| 参数 | 默认 | 说明 |
|---|---|---|
| `robot_radius` | `0.25` | 障碍膨胀半径（m） |
| `start_clear_radius` | `0.30` | 起点附近放行膨胀区（脱困） |
| `safety_distance` | `0.40` | 激光前向安全距离，低于此值停止前进并转向 |
| `slow_distance` | `1.00` | 前方障碍小于此距离时线性减速 |
| `unknown_penalty` | `2.0` | 未知格额外代价倍数（0 = 与空闲同价） |
| `goal_tolerance` | `0.15` | 到达判定距离（m） |
| `max_linear_speed` | `0.30` | m/s |
| `max_angular_speed` | `0.90` | rad/s |
| `lookahead_distance` | `0.32` | 前视距离（m） |
| `control_rate` | `20.0` | 控制频率（Hz） |
| `replan_period` | `1.0` | 重规划周期（s） |
| `goal_search_radius` | `1.5` | 目标吸附搜索半径（m） |
| `nav_map_topic` | `/map` | 输入地图 |
| `goal_topic` | `/goal_pose` | 目标输入 |
| `plan_topic` | `/plan` | 路径输出 |
| `cmd_vel_topic` | `/cmd_vel` | 速度输出（`TwistStamped`） |

> **bringup 会覆盖速度**：`mapping_backend:=slam_toolbox` 时传
> `0.15 / 0.50`（和 TB4 odom_grid 相同）；TB3 的 odom_grid 传 `0.30 / 0.90`。
> TB4 另外把 `slow_distance` 缩到 0.60 m，避免在门洞里长期处于减速区。

内部固定行为（不是参数）：

- 8 邻接 A*，禁止斜穿墙角；路径做视线简化
- 航向误差 >0.6 rad 时原地转向
- 连续受阻 3 次放弃当前目标；持续受阻 2 s 后倒车脱困
- 激光数据缺失或超过 1 s 未更新时停车（不拿旧扫描盲开）
- 无回波射线不参与安全距离判断
- 前方/后方扇面完全没有有效回波时按“受阻”处理（TB4 会丢扇面，不能当成空旷）
- `safety_distance` 保持 0.40 m；超过 0.45 m 时 TB4 会被 1 m 门洞的门框挡住、反复“受阻”并放弃目标
- 接近目标：0.4 m 内减速，最低保留 40% 速度；前方障碍减速最低保留 35%
- `frontier_explorer` 的 `progress_timeout` 由 bringup 设为 15 s（低速时 8 s 容易误拉黑）

---

## 5. `frontier_explorer`（`navigation_lib` 可执行文件）

| 参数 | 默认 | 说明 |
|---|---|---|
| `map_topic` | `/map` | 输入地图 |
| `goal_topic` | `/goal_pose` | 输出目标 |
| `odom_topic` | `/odom` | 机器人里程计 |
| `explore_period` | `1.0` | 决策周期（s） |
| `min_frontier_size` | `10` | 最小边界簇（格） |
| `min_known_cells` | `500` | 已知格少于该值时不判定“探索完成” |
| `min_target_distance` | `2.5` | 优先选至少这么远的边界（m） |
| `heading_weight` | `0.6` | 朝向偏差权重（越大越倾向保持当前朝向） |
| `goal_clearance` | `0.35` | 目标离障碍最小距离（m） |
| `goal_reached_distance` | `0.30` | 到达判定（m） |
| `progress_timeout` | `8.0` | 无进展超时（s） |
| `goal_timeout` | `180.0` | 单目标总时限（s） |
| `cooldown` | `0.5` | 到达后等待地图更新（s） |
| `blacklist_radius` | `0.75` | 已访问/失败点黑名单半径（m） |

---

## 6. `slam_toolbox`（外部节点）

配置文件：[config/slam_toolbox.yaml](../src/my_robot_bringup/config/slam_toolbox.yaml)

| 参数 | 值 | 说明 |
|---|---|---|
| `odom_frame` / `map_frame` / `base_frame` | `odom` / `map` / `base_footprint` | 坐标系 |
| `scan_topic` | `/scan` | 激光 |
| `mode` | `mapping` | 建图模式 |
| `resolution` | `0.05` | 地图分辨率 |
| `min_laser_range` / `max_laser_range` | `0.12` / `12.0` | 与自定义激光一致 |
| `minimum_time_interval` | `0.05` | 10 Hz 扫描全部处理 |
| `map_update_interval` | `1.0` | 地图发布周期（s） |
| `transform_timeout` | `0.2` | TF 等待（s） |
| `minimum_travel_distance` / `minimum_travel_heading` | `0.05` / `0.05` | 每 5 cm / 0.05 rad 建节点；全 0 节点过密，易累积双线 |
| `loop_search_maximum_distance` | `6.0` | 回环搜索半径（m），原 3.0 在 10×8 房间里常够不到 |
| `use_scan_matching` | `true` | 扫描匹配 |
| `do_loop_closing` | `true` | 回环 |
| `correlation_search_space_dimension` | `0.5` | 平移搜索范围（调大会在空地图上崩溃） |
| `coarse_search_angle_offset` | `0.349` | 粗搜索角度范围 |
| `stack_size_to_use` | `40000000` | 序列化大地图用 |

启动顺序由 bringup 控制：等第一帧 `/scan` → 5 s → 启动 slam_toolbox →
3 s → `activate_node.sh` 带重试激活。

---

## 7. Launch 参数（`sim_turtlebot4.launch.py` / `sim.launch.py` / `bringup.launch.py`）

| 参数 | 默认 | 说明 |
|---|---|---|
| `robot` | `turtlebot4_lite` | `turtlebot4_lite` / `turtlebot3` |
| `tb4_model` | `lite` | `lite` / `standard` |
| `world` | `worlds/room_world.world` | 10 m × 8 m 四室 + 走廊 |
| `model` | 环境变量 `TURTLEBOT3_MODEL`，否则 `burger` | TurtleBot3 型号（仅回退时生效） |
| `headless` | `false` | 只起 Gazebo server |
| `rviz` | `false` | 启动 RViz2 |
| `software_rendering` | `true` | 只对 TB3 生效；TB4 不要开（会段错误） |
| `explore` | `false` | 启动前沿自动探索 |
| `mapping_backend` | `odom_grid` | 建图后端 |
| `slam_params_file` | `config/slam_toolbox.yaml` | slam 参数文件 |
| `use_sim_time` | `true` | 使用仿真时钟 |
| `x_pose` / `y_pose` / `yaw` | `-3.5` / `2.0` / `0.0` | 初始位姿 |

---

## 8. 参数如何生效

- 节点参数在 `init()` 里用 `declare_parameter` 读取一次，保存为成员变量。
  **`ros2 param set` 只改变参数服务器，不会更新已缓存的成员，需要重启节点/仿真。**
- 带 `main_node` 的 launch 使用 `/**` 参数覆盖，因此 `max_linear_speed` 等
  会作用于进程内的 `navigation_node`；`mapping_backend` 只被 `mapping_node` 声明。
- 例：临时改导航速度（仿真单独起，主节点单独起并覆盖参数）：

  ```bash
  # 终端 1：只起 TB4 仿真
  ros2 launch my_robot_bringup sim_turtlebot4.launch.py headless:=true
  # （TurtleBot3 回退：ros2 launch my_robot_bringup sim.launch.py headless:=true）

  # 终端 2：主节点，参数作用于进程内所有节点
  ros2 run my_robot_project main_node --ros-args \
    -p use_sim_time:=true \
    -p mapping_backend:=odom_grid \
    -p max_linear_speed:=0.20 \
    -p max_angular_speed:=0.8
  ```

- 建图后端切换：

  ```bash
  ./sim.sh rviz:=true                                    # odom_grid
  ./sim.sh mapping_backend:=slam_toolbox rviz:=true      # slam_toolbox
  ```

- TB3 激光模型改 `models/turtlebot3_burger/model.sdf`、TB4 改
  `urdf/*.urdf.xacro` 或 `scripts/tb4_urdf.py` 后，需要
  `colcon build --symlink-install --packages-select my_robot_bringup` 并重启
  （TB4 的 URDF 由 `robot_state_publisher` 在每次 launch 时重新生成）。

---

## 9. 调参建议

| 现象 | 优先调整 |
|---|---|
| 长廊/弱特征环境叠图 | 降 `max_linear_speed` / `max_angular_speed`；加走廊特征；考虑 Cartographer |
| 探索太慢 | 提高速度/`min_target_distance`；减少 `cooldown` |
| 探索总在墙边打转 | 提高 `goal_clearance`、`min_frontier_size`；检查 `blacklist_radius` |
| 机器人顶墙抖动 | 提高 `safety_distance` / `slow_distance`；降低速度 |
| SLAM 启动地图 0×0 | 机器人需先移动 5 cm（`minimum_travel_distance=0.05`）；再检查 slam 是否 active |
| 地图/激光显示错位 | 检查 RViz Fixed Frame=`map`，TF 是否只有一套 |
