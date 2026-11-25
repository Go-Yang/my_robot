# my_robot 项目架构

ROS 2 Jazzy + Gazebo Sim（Harmonic）的室内机器人工作空间。当前实现：

- 仿真环境、TurtleBot4 Lite（默认）/ TurtleBot3（回退）模型、传感器接收
- 可插拔建图后端：自研占据栅格（`odom_grid`）、外部 `slam_toolbox`、自研 SLAM 占位
- 自研栅格导航（A* + 路径跟踪 + 激光安全/脱困）
- 前沿自动探索
- 地图保存（PGM + YAML）

房间分区 / 拓扑图（`room_segment_lib`）尚未开始。

所有节点/launch/机器人本体的可调参数见 [PARAMETERS.md](PARAMETERS.md)。

---

## 1. 包与分层

```
my_robot_project/              # 顶层可执行 main_node：组装并 spin 各模块
├── sensor_interface_lib/      # 传感器缓存（/scan /odom /imu）+ sensor_monitor
├── mapping_lib/               # 建图 facade + 可插拔后端
│   └── sensor_interface_lib
├── navigation_lib/            # 导航 + 前沿探索
│   └── sensor_interface_lib
├── robot_control_lib/         # 控制模块占位（当前未启用）
│   └── my_robot_msgs
└── my_robot_msgs/             # 消息包（目前只有 MyCustomMessage 占位）

my_robot_bringup/              # 纯运行时资源包：world/models/launch/config/rviz/scripts
```

| 包 | 作用 | 关键依赖 |
|---|---|---|
| `my_robot_project` | `main_node` 进程，创建 `sensor_node` / `mapping_node` / `navigation_node`，共用一个 `MultiThreadedExecutor` | 下面四个库 |
| `sensor_interface_lib` | 订阅并缓存最新 `/scan`、`/odom`、`/imu`；提供线程安全 getter；`sensor_monitor` 自检节点 | `rclcpp`、`sensor_msgs`、`nav_msgs` |
| `mapping_lib` | `MappingModule` facade + `MappingBackend` 接口 + 三个后端；发布或缓存 `/map` | `tf2_ros`、`nav_msgs`、`sensor_interface_lib` |
| `navigation_lib` | `NavigationModule`（A* + 跟踪 + 安全）、`frontier_explorer` 可执行文件 | `tf2_ros`、`nav_msgs`、`visualization_msgs` |
| `robot_control_lib` | 控制占位，当前 `main.cpp` 未启用 | `std_msgs`、`my_robot_msgs` |
| `my_robot_msgs` | 消息生成包；目前只有 `MyCustomMessage.msg` | `rosidl_default_generators` |
| `my_robot_bringup` | Gazebo world、模型、launch、桥接/RViz/slam 配置、辅助脚本 | 运行时依赖 TurtleBot4 / TurtleBot3 / ros_gz / slam_toolbox |

> `my_robot_bringup` 在 `package.xml` 里 `exec_depend` 了 `my_robot_project`。
> `make.sh lib` 因此会跳过 bringup。

---

## 2. 运行时进程与节点

```
┌────────────────────────── main_node 进程 ──────────────────────────┐
│                                                                    │
│  /sensor_node ──缓存──▶ /mapping_node ──▶ /map                      │
│       │                    ▲                                       │
│       │                    │ 读缓存                                 │
│       └──────────▶ /navigation_node ──▶ /plan、/cmd_vel             │
│                          ▲                                         │
└──────────────────────────┼─────────────────────────────────────────┘
                           │ /map、/goal_pose、TF
        ┌──────────────────┴──────────────────┐
        │                                     │
 /frontier_explorer（可选）             /slam_toolbox（可选后端）
        │                                     │
        └──────────▶ /goal_pose               └──▶ /map、map→odom

外部进程：gz sim、ros_gz_bridge、robot_state_publisher、rviz2
```

- `main_node` 内的三个节点共享一个 `MultiThreadedExecutor`；同一节点默认回调组互斥，
  `SensorNode` 的数据缓存用 mutex 保护，可被 mapping/navigation 并发读取。
- `frontier_explorer` 是独立进程（`navigation_lib` 里的可执行文件），通过 `/goal_pose`
  复用导航，不直接控制底盘。
- `slam_toolbox` 是外部 lifecycle 节点，由 bringup 在收到第一帧 `/scan` 后启动。

---

## 3. 数据流

### 3.1 传感器与桥接

```
TB4: world + /sim_ground_truth_pose ──world_lidar(CPU 射线投射)──▶ /scan
TB3: Gazebo 雷达 ──ros_gz_bridge─────────────────────────────────▶ /scan
     /odom /imu /clock /tf /joint_states ─────────────────────────▶ SensorNode 缓存最新帧
```

- `/cmd_vel` 类型是 **`geometry_msgs/msg/TwistStamped`**（TB4 与 Jazzy 版 TB3 都是）。
  TB4 由 `motion_control` 转发给 `diff_drive_controller`；TB3 经桥接转成
  `gz.msgs.Twist` 驱动 DiffDrive。
- TB4 Lite：RPLIDAR A1M8，**12 m / 10 Hz / 640 点**，`/odom`、`/tf`、
  `/joint_states` 由 `diff_drive_controller` / `joint_state_broadcaster` 直接发布；
  `/imu` 由 gz 桥接补上。`SensorNode` 会丢弃全 inf/全 range_min 的坏帧。
- TB3（回退）：自定义 burger 模型，激光 **12 m / 10 Hz**，360 点。

### 3.2 建图

```
odom_grid 后端：
  SensorNode 缓存 ──▶ log-odds 逆传感器模型 ──▶ Bresenham 射线投射 ──▶ /map
                      （frame=map，发布静态 map→odom 恒等变换）

slam_toolbox 后端：
  /scan + TF ──▶ slam_toolbox（扫描匹配/回环/位姿图）──▶ /map + 动态 map→odom
                 MappingModule 的 SlamToolboxBackend 只订阅 /map 缓存
```

### 3.3 导航与探索

```
/map + /goal_pose + (odom 位姿经 TF 转到 map) ──▶ A* + 路径跟踪 + 安全保护
                                                     └──▶ /plan、/cmd_vel

/map ──▶ frontier_explorer（前沿检测/聚类/黑名单/进展超时）──▶ /goal_pose、/frontiers
```

---

## 4. 话题与坐标系

| 话题 | 类型 | 方向 | 说明 |
|---|---|---|---|
| `/scan` | `sensor_msgs/LaserScan` | 桥接→ROS | 12 m / 10 Hz，frame=`base_scan` |
| `/odom` | `nav_msgs/Odometry` | 桥接→ROS | 50 Hz，frame=`odom` |
| `/imu` | `sensor_msgs/Imu` | 桥接→ROS | 200 Hz |
| `/clock`、`/tf`、`/tf_static`、`/joint_states` | — | 桥接/RSP→ROS | 仿真时钟与 TF |
| `/cmd_vel` | `geometry_msgs/TwistStamped` | ROS→桥接 | 导航输出 |
| `/map` | `nav_msgs/OccupancyGrid` | 建图→ROS | transient local |
| `/map_metadata` | `nav_msgs/MapMetaData` | slam_toolbox | 可选 |
| `/goal_pose` | `geometry_msgs/PoseStamped` | 探索/RViz→导航 | frame=`map` |
| `/plan` | `nav_msgs/Path` | 导航→RViz | 规划路径 |
| `/frontiers` | `visualization_msgs/MarkerArray` | 探索→RViz | 边界点/当前目标 |

TF 链：

```
map ──(odom_grid: 静态恒等 / slam_toolbox: 动态)──▶ odom ──▶ base_footprint ──▶ base_link ──▶ base_scan
```

- `odom → base_footprint` 由 Gazebo DiffDrive 插件经桥接发布。
- `base_footprint → base_link → base_scan` 由 `robot_state_publisher` 发布（固定关节在 `/tf_static`）。
- 导航和探索都通过 TF 把 odom 位姿转换到地图的 `map` 系，不假设两系相同。
- RViz Fixed Frame 是 `map`。

---

## 5. 建图后端抽象

接口：[`mapping/mapping_backend.h`](../src/mapping_lib/include/mapping/mapping_backend.h)

```cpp
class MappingBackend {
  virtual std::string name() const = 0;
  virtual void init(node, sensor) = 0;
  virtual nav_msgs::msg::OccupancyGrid::SharedPtr getMap() const = 0;
  virtual bool ready() const = 0;
  virtual std::string mapFrame() const = 0;
};
```

[`MappingModule`](../src/mapping_lib/include/mapping/mapping.h) 按参数 `mapping_backend` 选择后端：

| 值 | 实现 | 行为 |
|---|---|---|
| `odom_grid`（默认） | [odom_grid_backend.cpp](../src/mapping_lib/src/odom_grid_backend.cpp) | 自研栅格建图，发布 `/map` + 静态 `map→odom` |
| `slam_toolbox` | [slam_toolbox_backend.cpp](../src/mapping_lib/src/slam_toolbox_backend.cpp) | 外部 slam_toolbox 发布 `/map`/`map→odom`，本类只缓存 |
| `custom_slam` | [custom_slam_backend.cpp](../src/mapping_lib/src/custom_slam_backend.cpp) | 自研 SLAM 占位，选中会打印警告且不产图 |

新增后端：实现接口 → 在 [`mapping.cpp`](../src/mapping_lib/src/mapping.cpp) 的工厂里注册 →
启动参数传 `mapping_backend:=<名字>`。导航、探索、RViz 不需要改。

---

## 6. 启动流程

### `sim_turtlebot4.launch.py`（TB4 仿真，默认）

1. `gz sim -r -s`（headless 时只起 server；`headless:=false` 再起 GUI）
2. 自定义 `robot_state_publisher`（由 `scripts/tb4_urdf.py` 生成裁剪后的 TB4 URDF）
3. 时钟桥 + `/imu` 桥；`world_lidar.py` 用 world 几何 + 真值位姿在 CPU 上
   生成 `/scan`；`create3_ros_gz_bridge.launch.py` 桥接底盘话题
4. 5 s 后 `ros_gz_sim create` 生成 TB4
5. 8 s 后启动 `create3_nodes.launch.py`（`motion_control`、controller spawner）
   和 `create3_gz_nodes.launch.py`（`sensors_node` / 位姿转发）
6. 可选 RViz2

TB4 模型适配（去掉 OAK-D、去掉全部 gpu_lidar、去重 Sensors 系统）集中在
`scripts/tb4_urdf.py`。激光由 `scripts/world_lidar.py` 用 world 静态 box 几何
+ `/sim_ground_truth_pose` 做 CPU 2D 射线投射，不再依赖 gz 的 Ogre 渲染。
`LIBGL_ALWAYS_SOFTWARE=1` 只给 gz GUI / RViz 两个显示进程单独设置。

### `sim.launch.py`（TurtleBot3 回退仿真）

1. `gz sim -r -s`（headless 时只起 server；`headless:=false` 再起 GUI）
2. `robot_state_publisher`
3. `ros_gz_bridge`（`config/bridge.yaml`）
4. 3 s 后 `ros_gz_sim create` 生成机器人
5. 可选 RViz2

模型选择：优先 `my_robot_bringup/models/turtlebot3_<model>/model.sdf`，
没有则回退系统 `turtlebot3_gazebo`。本仓库提供了 12 m / 10 Hz 的 burger 模型。

### `bringup.launch.py`（仿真 + 主节点 + 可选探索/SLAM）

- 按 `robot` 参数包含 `sim_turtlebot4.launch.py`（默认）或 `sim.launch.py`
- 启动 `main_node`，传 `mapping_backend`、`use_sim_time`，以及在 SLAM 模式下
  更低的 `max_linear_speed=0.15`、`max_angular_speed=0.5`
- `slam_toolbox` 模式下：
  1. `wait_for_scan.py` 等到第一帧 `/scan` 退出
  2. 等 5 s 后启动 `slam_toolbox/online_async_launch.py`（`autostart:=false`）
  3. 再等 3 s，`activate_node.sh` 带重试地 configure + activate
  - 原因：与仿真同时启动时 slam_toolbox 可能停在 0×0 地图；高负载下官方一次性
    生命周期激活也可能超时
- `explore:=true` 时启动 `frontier_explorer`

### 常用参数

| 参数 | 默认 | 说明 |
|---|---|---|
| `robot` | `turtlebot4_lite` | `turtlebot4_lite` / `turtlebot3` |
| `tb4_model` | `lite` | `lite` / `standard` |
| `world` | `worlds/room_world.world` | 10 m × 8 m 四室 + 走廊 |
| `model` | 环境变量或 `burger` | TurtleBot3 型号（`robot:=turtlebot3` 时生效） |
| `headless` | `false` | 只起 Gazebo server |
| `rviz` | `false` | 启动 RViz2 |
| `software_rendering` | `true` | 只对 TB3 生效；TB4 不要强制软件渲染（会段错误） |
| `explore` | `false` | 前沿自动探索 |
| `mapping_backend` | `odom_grid` | `odom_grid` / `slam_toolbox` / `custom_slam` |
| `slam_params_file` | `config/slam_toolbox.yaml` | slam_toolbox 参数 |

---

## 7. 构建与运行脚本

| 脚本 | 作用 |
|---|---|
| `build.sh` | 清理 `build/install/log` + 全量构建 + 检查包 |
| `make.sh lib` | 只编译各库（跳过 `my_robot_project` 和 `my_robot_bringup`） |
| `make.sh core <包>` | 编译指定包及其依赖 |
| `make.sh main` | 编译 `my_robot_project` + `my_robot_bringup` 并运行主节点 |
| `sim.sh [launch 参数]` | 自动 source 环境、检查重复实例，启动 `bringup.launch.py` |
| `run.sh` | 直接运行 `main_node`（不启动仿真） |
| `build_and_run.sh` | 清理 + 全量构建 + 运行 `main_node` |
| `save_map.sh [前缀]` | 把当前 `/map` 保存成 PGM + YAML（需仿真在跑） |

常用：

```bash
./build.sh
./sim.sh headless:=true mapping_backend:=slam_toolbox explore:=true rviz:=true
./save_map.sh maps/room_map
```

---

## 8. 并发模型

- `main_node`：`MultiThreadedExecutor` + 三个节点；`SensorNode` 的缓存由 mutex 保护。
- `MappingModule` 的更新/发布定时器在 `mapping_node` 上，同一回调组串行。
- `NavigationModule` 的控制定时器、地图/目标回调在 `navigation_node` 上串行；
  位姿查询通过 `tf2_ros::Buffer`。
- `frontier_explorer` 独立进程，1 Hz 定时器做前沿计算/选目标。
- `slam_toolbox` 独立进程，内部有自己的线程。

---

## 9. 当前限制与后续

- `odom_grid`：无扫描匹配/回环，依赖轮式里程计；靠激光安全保护和降速减少漂移。
- `slam_toolbox`：长平行墙走廊仍有几何退化，可能墙线加粗/叠图；增加走廊特征或接
  Cartographer 是更有效的方向。
- 导航：未知格“可通行但加代价”，没有完整局部代价地图/动态避障；连续受阻 3 次会
  放弃当前目标。
- 探索：只适合静态环境；没有主动回环/重访策略。
- `robot_control_lib` 仍是占位，`main.cpp` 未启用。
- `room_segment_lib` 未创建；`Region` / `TopologyGraph` / `GetRoomPath` 尚未定义。
- `custom_slam` 后端只有骨架，后续在这里实现扫描匹配/回环/位姿图。

---

## 10. 目录树

```
my_robot/
├── build.sh  make.sh  sim.sh  run.sh  build_and_run.sh  save_map.sh
├── maps/                          # save_map.sh 输出（PGM + YAML）
├── docs/ARCHITECTURE.md
└── src/
    ├── my_robot_msgs/
    ├── robot_control_lib/
    ├── sensor_interface_lib/
    │   ├── include/sensor_interface/sensor_node.h
    │   └── src/node/{sensor_node.cpp, sensor_monitor_main.cpp}
    ├── mapping_lib/
    │   ├── include/mapping/{mapping.h, mapping_backend.h,
    │   │                    odom_grid_backend.h, slam_toolbox_backend.h,
    │   │                    custom_slam_backend.h}
    │   └── src/{mapping.cpp, odom_grid_backend.cpp,
    │            slam_toolbox_backend.cpp, custom_slam_backend.cpp}
    ├── navigation_lib/
    │   ├── include/navigation/navigation.h
    │   └── src/{navigation.cpp, frontier_explorer.cpp}
    ├── my_robot_project/src/main.cpp
    └── my_robot_bringup/
        ├── launch/{sim_turtlebot4.launch.py, sim.launch.py, bringup.launch.py}
        ├── worlds/room_world.world
        ├── urdf/turtlebot4_{lite,standard}_nocam.urdf.xacro   # TB4 模型副本
        ├── models/turtlebot3_burger/{model.sdf, model.config}   # TB3 回退
        ├── config/{bridge.yaml, slam_toolbox.yaml}
        ├── rviz/my_robot.rviz
        └── scripts/{wait_for_scan.py, activate_node.sh, tb4_urdf.py, world_lidar.py, scan_filter.py}
```
