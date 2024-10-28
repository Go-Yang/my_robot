cmake_minimum_required(VERSION 3.5)
project(my_robot_project)

# 查找所需的 ROS 2 组件
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

# 包含头文件目录
include_directories(
  include
  lib/robot_control_lib/include
  lib/sensor_interface_lib/include
  lib/mapping_lib/include
  lib/navigation_lib/include
)

# 添加库
add_subdirectory(lib/robot_control_lib)
add_subdirectory(lib/sensor_interface_lib)
add_subdirectory(lib/mapping_lib)
add_subdirectory(lib/navigation_lib)

# 添加可执行文件
add_executable(main_node src/main.cpp)
ament_target_dependencies(main_node rclcpp std_msgs)

# 链接库
target_link_libraries(main_node
  robot_control_lib
  sensor_interface_lib
  mapping_lib
  navigation_lib
)

# 安装目标
install(TARGETS
  main_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
