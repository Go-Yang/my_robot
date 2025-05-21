# my_robot
My robot project based on ros2  

/my_robot                  <-- ROS 2 工作空间（colcon workspace）
├── src                    <-- colcon 识别的 ROS 2 源码目录
│   ├── my_robot_msgs      <-- 消息包（公共包，所有库和主项目依赖它）
│   │   ├── CMakeLists.txt
│   │   ├── package.xml
│   │   └── msg
│   │       └── MyCustomMessage.msg
│   │
│   ├── robot_control_lib  <-- 控制库（独立维护）
│   │   ├── CMakeLists.txt
│   │   ├── package.xml
│   │   ├── src
│   │   │   └── control.cpp
│   │   └── include
│   │       └── robot_control
│   │           └── control.h
│   │
│   ├── sensor_interface_lib  <-- 传感器接口库（独<!--  -->立维护）
│   │   ├── CMakeLists.txt
│   │   ├── package.xml
│   │   ├── src
│   │   │   └── node                    # ROS2节点实现
│   │   │   │   └── sensor_node.cpp     # 节点主逻辑
│   │   │   │   
│   │   │   └── core                    # 核心业务逻辑
│   │   │       └──  sensor.cpp         # 实现文件
│   │   │       └──  sensor.hpp         # 内部头文件（不对外暴露）
│   │   │   
│   │   └── include
│   │       └── sensor_interface
│   │           └── sensor_node.h       # 对外暴露的节点API（可选）
│   │
│   ├── mapping_lib  <-- 地图构建库（独立维护）
│   │   ├── CMakeLists.txt
│   │   ├── package.xml
│   │   ├── src
│   │   │   └── mapping.cpp
│   │   └── include
│   │       └── mapping
│   │           └── mapping.h
│   │
│   ├── navigation_lib  <-- 导航库（独立维护）
│   │   ├── CMakeLists.txt
│   │   ├── package.xml
│   │   ├── src
│   │   │   └── navigation.cpp
│   │   └── include
│   │       └── navigation
│   │           └── navigation.h
│   │
│   ├── my_robot_project  <-- 最高调度层（负责调度所有库）
│   │   ├── CMakeLists.txt
│   │   ├── package.xml
│   │   ├── src
│   │   │   └── main.cpp
│   │
└── build/ install/ log/   <-- colcon 编译生成的目录（无需手动创建）

colcon build --packages-select my_robot_msgs
mapping_lib
robot_control_lib
colcon build --packages-select  sensor_interface_lib
navigation_lib
colcon build --packages-select my_robot_project

<!-- ros2 launch gazebo_ros gazebo.launch.py -->

ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py

ros2 run turtlebot3_teleop teleop_keyboard

ros2 launch turtlebot3_bringup rviz2.launch.py