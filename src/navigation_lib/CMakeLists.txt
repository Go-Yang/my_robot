cmake_minimum_required(VERSION 3.8)
project(navigation_lib)

# 查找依赖
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(sensor_interface_lib REQUIRED)

# 添加库
add_library(${PROJECT_NAME} SHARED
  src/navigation.cpp
)

# 包含头文件目录
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# 链接依赖
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  nav_msgs
  geometry_msgs
  tf2_ros
  sensor_interface_lib
)

# 自动前沿探索节点（独立进程，通过 /goal_pose 复用上面的导航）
add_executable(frontier_explorer
  src/frontier_explorer.cpp
)
ament_target_dependencies(frontier_explorer
  rclcpp
  nav_msgs
  geometry_msgs
  visualization_msgs
  tf2_ros
)

# 安装库
install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS frontier_explorer
  DESTINATION lib/${PROJECT_NAME}
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
  rclcpp
  nav_msgs
  geometry_msgs
  tf2_ros
  sensor_interface_lib
)

ament_package()
