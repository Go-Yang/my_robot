# my_robot

基于 ROS 2 的机器人项目工作空间（colcon workspace）。

## 环境

本项目当前适配的实测环境：

| 项 | 版本 |
|---|---|
| 系统 | Ubuntu 24.04 (noble) |
| ROS 2 | **Jazzy**（`/opt/ros/jazzy`） |
| Gazebo | Gazebo Sim 8.11.0（Harmonic），随 `ros-jazzy-ros-gz` 的 vendor 包提供 |
| 仿真机器人 | TurtleBot3（`ros-jazzy-turtlebot3*`） |

> Ubuntu 24.04 上 **不支持** ROS 2 Humble，也装不了 Gazebo Classic（gazebo11 已 EOL）。
> 仿真用的是新一代 Gazebo Sim（`gz sim`），走 `ros_gz` 桥接，不是 Classic。

## 目录结构

```
my_robot/
├── build.sh              # 清理 + 完整构建 + 检查包
├── run.sh                # 直接运行主节点
├── sim.sh                # 自动 source 环境并启动仿真 + 主节点
├── build_and_run.sh      # 构建 + 运行
├── make.sh               # 分档构建：lib / core <包名> / main
└── src/
    ├── my_robot_msgs/            # 消息包（公共依赖）
    │   └── msg/MyCustomMessage.msg
    │
    ├── robot_control_lib/        # 控制库
    │   ├── include/robot_control/control.h
    │   └── src/control.cpp
    │
    ├── sensor_interface_lib/     # 传感器接收库（scan / odom / imu）
    │   ├── include/sensor_interface/sensor_node.h
    │   └── src/node/
    │       ├── sensor_node.cpp
    │       └── sensor_monitor_main.cpp   # sensor_monitor 自检节点
    │
    ├── mapping_lib/              # 建图 facade：odom_grid / slam_toolbox / custom_slam
    ├── navigation_lib/           # A* 全局规划 + 路径跟踪 + 前沿自动探索
    │
    ├── my_robot_bringup/         # 仿真环境、机器人、launch、rviz、桥接
    │   ├── worlds/room_world.world       # 四室 + 走廊测试环境
    │   ├── models/                       # 预留自定义模型
    │   ├── launch/sim.launch.py          # Gazebo + TurtleBot3 + ros_gz 桥接
    │   ├── launch/bringup.launch.py      # 仿真 + 主节点
    │   ├── config/bridge.yaml
    │   └── rviz/my_robot.rviz
    │
    └── my_robot_project/         # 顶层调度，链接上面各库
        ├── CMakeLists.txt
        ├── package.xml
        └── src/main.cpp
```

每个库都把公开头文件放在 `include/<包名>/` 下并导出 CMake target，`src/` 下**只放实现文件**——
同名头文件不要在两处各放一份，`#include "xxx.h"` 会优先命中 `src/` 那份，容易和下游用到的不一致。

## 构建与运行

```bash
cd ~/workspace/my_robot

./build.sh            # 清理 + 构建全部 + 验证包可见
./run.sh              # 运行主节点（需先构建过）
./build_and_run.sh    # 构建并直接运行

# 分档构建
./make.sh lib                 # 只编译各库（跳过 my_robot_project / my_robot_bringup）
./make.sh core mapping_lib    # 单独编译某个库及其依赖
./make.sh main                # 编译 my_robot_project + my_robot_bringup 并运行
```

脚本会从**自身所在位置**推导工作空间根目录，因此可以从任意目录调用。

手动构建等价于：

```bash
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
ros2 run my_robot_project main_node
```

> ⚠️ source 顺序必须是**先 ROS、后工作空间**，反了工作空间的路径会被覆盖掉。

主节点启动三个节点：`/sensor_node`（订阅 `/scan`、`/odom`、`/imu` 并缓存最新数据）、
`/mapping_node`（消费缓存数据建图，发布 `/map`）和 `/navigation_node`（订阅 `/map`、
`/goal_pose`，发布 `/plan` 与 `/cmd_vel`）。三者共用一个 `MultiThreadedExecutor`。

## 仿真与传感器自检（第一阶段）

### 一键启动

```bash
./build.sh
./sim.sh                          # 自动 source 环境 + Gazebo GUI + 主节点
./sim.sh headless:=true
./sim.sh rviz:=true
./sim.sh --show-args              # 查看全部 launch 参数
```

`sim.sh` 内部会先 source ROS 与 `install/setup.bash`，无需手动处理环境。
手动启动等价于：

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch my_robot_bringup bringup.launch.py        # 仿真 + 主节点
ros2 launch my_robot_bringup sim.launch.py            # 只起仿真
```

常用 launch 参数：

| 参数 | 默认 | 说明 |
|---|---|---|
| `world` | 本包 `worlds/room_world.world` | world 文件绝对路径 |
| `model` | 环境变量 `TURTLEBOT3_MODEL`，未设置时为 `burger` | `burger` / `waffle` / `waffle_pi` |
| `headless` | `false` | `true` 时只起 Gazebo server（无 GUI） |
| `rviz` | `false` | 同时启动 RViz2 |
| `x_pose` / `y_pose` / `yaw` | `-3.5` / `2.0` / `0.0` | 初始位姿，默认在 Room A |
| `software_rendering` | `true` | 强制 Mesa 软件渲染；关闭 3D 加速时必需，开启 3D 后可设 `false` |
| `explore` | `false` | 启动前沿自动探索 |
| `mapping_backend` | `odom_grid` | 建图后端：`odom_grid` / `slam_toolbox` / `custom_slam` |
| `slam_params_file` | 本包 `config/slam_toolbox.yaml` | slam_toolbox 参数文件 |

### 测试环境

`room_world.world` 是一个 10 m × 8 m、四室 + 走廊的室内场景，门洞宽 1 m，
全部用简单 box 搭建，不依赖 Fuel，离线可用。默认出生点在 Room A `(-3.5, 2.0)`。

### 传感器话题

仿真启动后由 `ros_gz_bridge` 提供：

| 话题 | 类型 | 频率（实测） |
|---|---|---|
| `/scan` | `sensor_msgs/msg/LaserScan` | ≈10 Hz，360 点，量程 0.12–12 m（自定义模型） |
| `/odom` | `nav_msgs/msg/Odometry` | ≈50 Hz |
| `/imu` | `sensor_msgs/msg/Imu` | ≈200 Hz |
| `/clock`、`/tf`、`/joint_states` | — | 由 Gazebo 提供 |

`/cmd_vel` 的类型是 **`geometry_msgs/msg/TwistStamped`**（Jazzy 版 TurtleBot3 的约定），
后续控制/导航模块发布速度指令时注意不要用成 `Twist`。

> 本项目的 `my_robot_bringup/models/turtlebot3_burger/` 里放了一份自定义模型，
> 把激光改成 **12 m / 10 Hz**（原版 3.5 m / 5 Hz），用于改善长廊里的 SLAM 可观测性。
> `sim.launch.py` 会优先使用本包 models/ 下的模型；没有时回退到系统安装的版本。

### 自检

不启动主节点，单独确认传感器是否正常：

```bash
ros2 run sensor_interface_lib sensor_monitor
```

会打印每类数据的第一帧信息，以及每 5 秒的接收速率统计。
`main_node` 里的 `SensorNode` 使用同一套订阅逻辑，只缓存最新一帧供建图/导航取用。

### 虚拟机渲染说明

开发机（VMware，8 vCPU / 16 GB）已关闭 VMware 3D 加速，默认走 **Mesa llvmpipe
软件渲染**（`software_rendering:=true`）。如果开着 3D 加速却用软件渲染会浪费性能，
此时可以传 `software_rendering:=false`；反过来，没有 3D 加速却走硬件渲染时
Gazebo 的 EGL 会初始化失败，表现为 `/scan` 一直没数据（日志里是
`libEGL warning: egl: failed to create dri2 screen`）。

## 建图（第二阶段）：可插拔后端

`mapping_lib` 通过统一接口 `MappingBackend`
（[mapping_backend.h](src/mapping_lib/include/mapping/mapping_backend.h)）支持多种后端，
由参数 `mapping_backend` 选择：

```bash
./sim.sh rviz:=true                                    # 默认 odom_grid
./sim.sh mapping_backend:=slam_toolbox rviz:=true      # 外部 slam_toolbox（扫描匹配 + 回环）
./sim.sh mapping_backend:=custom_slam rviz:=true       # 自研 SLAM 占位（待实现）
```

接口只有 `init(node, sensor)`、`getMap()`、`ready()`、`mapFrame()`。
自研后端自己发布 `/map`；外部 SLAM 后端由独立节点发布，`mapping_lib` 只订阅缓存。
所有后端都把地图放在 `map` 坐标系下：`odom_grid` 广播静态恒等 `map -> odom`，
`slam_toolbox` 自己发布动态 `map -> odom`。

### odom_grid（默认，自研）

- 数据流：`/scan` + `/odom` → log-odds 逆传感器模型 → Bresenham 射线投射 → `/map`
- 地图 400 × 400 格、0.05 m/格（20 m × 20 m），原点 `(-10, -10)`，QoS transient local
- 地图 frame=`map`，同时发布静态 `map -> odom`（纯里程计建图没有定位修正）
- 每 5 秒打印已处理帧数与空闲/占据格数

参数（`mapping_node`）：

| 参数 | 默认 | 说明 |
|---|---|---|
| `resolution` | `0.05` | 栅格分辨率（m） |
| `width` / `height` | `400` / `400` | 栅格尺寸（格） |
| `origin_x` / `origin_y` | `-10.0` / `-10.0` | 地图左下角在 odom 中的位置 |
| `map_topic` | `/map` | 发布话题 |
| `map_frame` / `odom_frame` | `map` / `odom` | 坐标系 |
| `free_log_odds` | `-0.45` | 空闲更新量 |
| `occupied_log_odds` | `0.9` | 占据更新量 |
| `min_log_odds` / `max_log_odds` | `-3.0` / `3.5` | log-odds 截断范围 |
| `occupied_threshold` | `0.4` | 占据/空闲判定阈值 |
| `update_rate` / `publish_rate` | `20` / `2` | 建图处理 / 地图发布频率（Hz） |

### slam_toolbox（外部 SLAM，已接入）

`mapping_backend:=slam_toolbox` 时，bringup 会启动
`slam_toolbox/async_slam_toolbox_node`，配置在
[slam_toolbox.yaml](src/my_robot_bringup/config/slam_toolbox.yaml)：

- 输入 `/scan` + TF（`odom -> base_footprint -> base_scan`）
- 输出 `/map` 和动态 `map -> odom`，含扫描匹配、回环检测、位姿图优化
- 地图会裁剪到已知区域；探索器把“栅格越界”也当作未知边界
- `mapping_lib` 的 `SlamToolboxBackend` 只订阅 `/map` 缓存，对外接口与 odom_grid 一致
- 启动顺序：先等第一帧 `/scan`，再由带重试的脚本 configure/activate；高负载下
  官方 launch 的一次性生命周期激活可能超时，节点会停在 inactive
- `minimum_travel_distance` / `minimum_travel_heading` 设为 `0`：机器人静止时也处理扫描。
  这两个值若大于 0，启动阶段机器人没动，地图会一直保持 0×0
- 自动降速：`mapping_backend:=slam_toolbox` 时 bringup 会把导航速度限制为
  **0.15 m/s / 0.5 rad/s**，减少长廊等弱约束环境里的匹配漂移

> slam_toolbox 刚启动时地图来自少量扫描，射线之间会有细小的未知缝隙。
> 导航允许未知格通行（`unknown_penalty` 加代价），配合激光安全距离，
> 机器人可以先动起来把地图补全。

> **叠图/墙体双线排查**：最常见原因是同时跑了两个仿真实例——两个 `/map`
> 和两个 `map -> odom` 会一起发布。`sim.sh` 启动前会检查并拒绝重复启动。
> 其次是 SLAM 扫描匹配失败：快速原地转向时相邻两帧转过的角度太大，
> 已把 `max_angular_speed` 降到 0.9 rad/s。仍出现双线时用 `headless:=true`
> 提高实时因子，或临时降低 `max_linear_speed` / `max_angular_speed`。

### custom_slam（自研 SLAM 占位）

接口骨架在 [custom_slam_backend.cpp](src/mapping_lib/src/custom_slam_backend.cpp)，
后续在这里实现扫描匹配 / 回环 / 位姿图优化，并发布 `/map` 与 `map -> odom`。
当前选中该后端会打印警告且不产生地图。

### 查看地图

```bash
./sim.sh rviz:=true
ros2 run sensor_interface_lib sensor_monitor    # 另开终端看传感器
```

RViz 的 Map display 已默认打开，Fixed Frame 是 `map`。

### 保存地图

地图只存在于 `/map` 话题里，**仿真退出后就没有了**，需要保存成文件：

```bash
./save_map.sh                    # maps/room_map.pgm + room_map.yaml
./save_map.sh maps/house_2       # 自定义输出前缀
```

`save_map.sh` 必须在仿真/建图运行时执行（脚本会检查 `/map` 话题）。
输出是 nav2 标准的 PGM + YAML，后续可以用 `nav2_map_server` 的 map_server 加载。
地图坐标系是 `map`。

### 自动探索建图

不想手动遥控时，加 `explore:=true`：

```bash
./sim.sh explore:=true rviz:=true
./sim.sh mapping_backend:=slam_toolbox explore:=true rviz:=true    # 用 SLAM 后端探索
```

`frontier_explorer` 节点会：

1. 在 `/map` 中找“本身空闲、8 邻域有未知格或越界”的前沿，按 8 邻接聚类；
2. 过滤小簇、离墙太近（`goal_clearance`）以及黑名单内的格子；
3. 优先选离机器人至少 `min_target_distance`（默认 2.5 m）的边界格，发 `/goal_pose`；
4. 到达后等 0.5 s 让地图更新；8 s 内没有靠近目标 0.15 m（或超过 180 s 总时限）则拉黑该点；
5. 没有可用边界时打印“探索完成”并停止。

RViz 中 `/frontiers` 显示绿色候选边界点，红色球是当前探索目标，`/plan` 是导航路径。
节点参数：`min_frontier_size`、`min_known_cells`、`min_target_distance`（默认 2.5 m）、
`heading_weight`（默认 0.6，越大会更倾向于保持当前朝向）、`goal_clearance`、
`goal_reached_distance`、`goal_timeout`、`progress_timeout`、`cooldown`、`blacklist_radius`、
`explore_period`。

实测（odom_grid）：从 Room A 出生点出发，自动探索四个房间 + 走廊约 8 分钟
（早期 VMware 软渲染环境）后打印“探索完成”；最终地图约 3 万空闲栅格、2.4 千占据栅格，
全程 odom 与 Gazebo 真值漂移约 3 cm。把 VM 调到 8 vCPU 后，软渲染下也能接近实时。

> 自动探索和键盘遥控都会发 `/cmd_vel`，不要同时使用。
>
> Gazebo GUI 和 RViz 仍会占用一部分渲染资源。追求建图速度时用 `headless:=true`，
> 需要看地图时再加 `rviz:=true`。

当前限制：odom_grid 不做扫描匹配/回环（仿真里程计足够准）；TurtleBot3 激光相对底盘
约 2 cm 的平移未补偿；slam_toolbox 后端在狭窄走廊里的自动探索目标仍可能超时，需要
继续调参。

## 栅格导航（第三阶段）

`navigation_lib` 基于 `/map` 做全局规划与路径跟踪，数据流：

```
/map ──> 障碍膨胀 ──> A* ──> /plan (nav_msgs/Path)
/goal_pose ─────────────────┘
/odom ──> TF(map->odom) ──> 路径跟踪 ──> /cmd_vel (geometry_msgs/TwistStamped)
```

- 障碍按 `robot_radius` 膨胀，8 邻接 A*，禁止斜穿墙角
- 未知栅格可通行，但按 `unknown_penalty` 加代价；目标是障碍/未知区时自动吸附到最近可通行格
- 机器人位姿通过 TF 从 `odom` 变换到地图的 `map` 系（odom_grid 用静态恒等变换，slam_toolbox 用动态变换）
- 路径做视线简化，跟踪时按 `lookahead_distance` 选取前视点
- 航向误差大于 0.6 rad 时原地转向，接近目标自动减速
- 每 `replan_period` 秒重规划一次，到达 `goal_tolerance` 内判定成功并停车
- 激光前向安全距离内只转向不前进，防止顶着墙走导致里程计漂移

发目标（RViz 里用 **2D Goal Pose** 工具，或命令行）：

```bash
ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: map}, pose: {position: {x: 1.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}"
```

导航参数（在 `navigation_node` 上声明）：

| 参数 | 默认 | 说明 |
|---|---|---|
| `robot_radius` | `0.25` | 障碍膨胀半径（m） |
| `start_clear_radius` | `0.30` | 规划时放行机器人附近这段膨胀区（脱困） |
| `safety_distance` | `0.40` | 激光前向安全距离，太近只转向不前进 |
| `slow_distance` | `1.00` | 前方障碍小于此距离时线性减速（m） |
| `unknown_penalty` | `2.0` | 未知格额外代价倍数（0 表示与空闲同价） |
| `goal_tolerance` | `0.15` | 到达判定距离（m） |
| `max_linear_speed` / `max_angular_speed` | `0.30` / `0.9` | 速度上限（m/s、rad/s；0.30 超过 burger 硬件上限，仅仿真用，真机改回 0.22） |
| `lookahead_distance` | `0.32` | 前视距离（m） |
| `control_rate` | `20.0` | 控制频率（Hz） |
| `replan_period` | `1.0` | 重规划周期（s） |
| `goal_search_radius` | `1.5` | 目标吸附搜索半径（m） |
| `nav_map_topic` | `/map` | 输入地图 |
| `goal_topic` / `plan_topic` / `cmd_vel_topic` | `/goal_pose` / `/plan` / `/cmd_vel` | 接口话题 |

实测（四室环境）：从出生点导航到 Room A 内的 `(1.0, 0.0)`，以及穿过 1 m 门洞到走廊
`(0.5, -2.2)`，均能规划并到达；后者在终点前自动旋转对准门洞。

当前限制：没有动态避障和恢复行为（卡住时需要重新发目标）；未知格是“乐观可通行”，
只靠激光安全距离兜底，不做完整的局部代价地图。

> ⚠️ 里程计依赖轮子不打滑。如果控制层持续顶着墙走，仿真里程计会快速漂移并毁掉地图，
> 所以控制器加了激光前向安全距离（`safety_distance`），太近时只原地转向；
> 同时把速度降到 0.15 m/s。实测 3 分钟探索后 odom 与 Gazebo 真值误差约 5 cm。

## TurtleBot3 官方仿真（备查）

> 本项目的 `my_robot_bringup` 不要求该环境变量（未设置时默认 `burger`，可用 `model:=` 覆盖）。
> 下面这条只针对 TurtleBot3 官方 launch：**必须先设置机器人型号**，否则会因
> `os.environ['TURTLEBOT3_MODEL']` 取不到值直接失败：

```bash
source /opt/ros/jazzy/setup.bash
export TURTLEBOT3_MODEL=burger        # 可选：burger / waffle / waffle_pi
```

建议写进 `~/.bashrc`。

### 官方 launch 启动

```bash
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py     # 官方世界
ros2 launch turtlebot3_gazebo empty_world.launch.py          # 空世界
ros2 launch turtlebot3_gazebo turtlebot3_house.launch.py     # 室内场景
```

启动后可用的关键话题：`/scan`、`/odom`、`/imu`、`/joint_states`、`/tf`、`/cmd_vel`。

### 键盘遥控（另开一个终端）

```bash
source /opt/ros/jazzy/setup.bash
export TURTLEBOT3_MODEL=burger
ros2 run turtlebot3_teleop teleop_keyboard
```

### 可视化与导航（可选）

```bash
ros2 launch turtlebot3_bringup rviz2.launch.py

# 建图（SLAM）
ros2 launch turtlebot3_cartographer cartographer.launch.py use_sim_time:=true

# 导航（自带 map.pgm / map.yaml）
ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=true
```

> nav2 和 cartographer 的 `use_sim_time` 默认是 `false`，跑仿真时必须显式传 `use_sim_time:=true`，
> 否则 TF 时间戳对不上。

### ⚠️ nav2 必须先给初始位姿

TurtleBot3 自带的 `param/burger.yaml` 里是 `set_initial_pose: false`（第 41 行），
也就是说 **AMCL 不会自己发布 `map→odom`**，必须由你告诉它机器人在哪。
而 nav2 的代价地图又必须在启动时拿到 `map→odom` 才能激活，于是形成死锁：

```
amcl 不给 map→odom  →  global_costmap 激活超时（约 75 秒）
                     →  lifecycle_manager_navigation 放弃 bringup
                     →  planner_server / bt_navigator 等全部停在 inactive
```

**正常流程**：nav2 起来后**尽快**（75 秒内）在 rviz 里点 **2D Pose Estimate**，
点一下机器人实际所在的 `(-2.0, -0.5)` 并拖出朝向。AMCL 收到后立刻发布 `map→odom`，
剩余节点会自动激活完成。

**如果错过了这个窗口**，生命周期管理器已经退出，重新调 `manage_nodes` 也没用
（它会试图从 `active` 状态的 `controller_server` 重新 configure，直接报
`No transition matching 1 found for current state active` 而放弃）。
此时按依赖顺序手动激活即可：

```bash
for n in planner_server bt_navigator behavior_server velocity_smoother \
         collision_monitor waypoint_follower; do
  ros2 lifecycle set /$n activate
done
```

`planner_server` 激活时会连带激活它内部的 `global_costmap`，所以顺序不能乱。

### 实测结论

本项目在 Ubuntu 24.04 + Jazzy + Gazebo Sim 8.11.0（VMware，早期软渲染环境）下实测通过：

- **nav2 全栈**：`NavigateToPose` 从 `(-1.95, -0.51)` 导航至 `(0.5, -0.5)`，
  17.2 秒到达，`error_code: 0`，0 次恢复行为。
- **Cartographer**：`map→odom` 以约 20 Hz 发布，地图正常增量构建
  （123×115 @ 0.05 m，14145 格中约 5900 格为已知占据/空闲）。

> 软渲染下 rviz 可能刷 `Message Filter dropping message ... queue is full`
> 和 GLSL 链接警告，都是 llvmpipe 性能不足导致的显示层问题，不影响导航与建图。
> 若出现较多，用 `headless:=true` 降低负载，或把 RViz 的显示订阅改为 Best Effort。

## 注意事项

- **`gz` 命令依赖 ROS 环境**：Gazebo 是作为 `ros-jazzy-ros-gz` 的 vendor 包装在
  `/opt/ros/jazzy/opt/` 下的，不在系统 PATH 中，需先 `source /opt/ros/jazzy/setup.bash`。
  **不要**再从 OSRF 源安装独立的 `gz-harmonic`，会与 vendored 版本冲突。
- **渲染**：当前 VM 已关闭 3D 加速，默认强制 Mesa 软件渲染（`software_rendering:=true`）。
  若重新开启 VMware 3D 加速，可传 `software_rendering:=false` 使用硬件渲染；
  两种模式都不要混用，否则 `/scan` 可能没有数据。
- 退出仿真后若有残留的 `gz sim` 进程，**不要**用 `pkill -f 'gz sim'` ——
  这个模式会匹配到执行它的那条 shell 命令本身，把自己的终端一起杀掉。
  先列 PID 再按 PID 杀：
  ```bash
  ps -eo pid,cmd --no-headers | grep 'gz sim' | grep -v grep | awk '{print $1}' | xargs -r kill
  ```

## 项目进度

- [x] 第一阶段：机器人模型 + 环境加载 + 传感器数据接收
- [x] 第二阶段：占据栅格建图（`mapping_lib`，发布 `/map`）
- [x] 建图后端抽象 + slam_toolbox 接入（`mapping_backend:=slam_toolbox`）
- [x] 第三阶段：栅格导航（`navigation_lib`，A* + 路径跟踪）
- [x] 附加：前沿自动探索（`frontier_explorer`，`explore:=true`）
- [ ] 第四阶段：房间分区 + 拓扑图（`room_segment_lib`、`my_robot_msgs`）
