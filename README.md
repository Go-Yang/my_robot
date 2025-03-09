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
│   ├── sensor_interface_lib  <-- 传感器接口库（独立维护）
│   │   ├── CMakeLists.txt
│   │   ├── package.xml
│   │   ├── src
│   │   │   └── sensor.cpp
│   │   └── include
│   │       └── sensor_interface
│   │           └── sensor.h
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
sensor_interface_lib
navigation_lib
my_robot_project