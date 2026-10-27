cmake_minimum_required(VERSION 3.8)
project(sensor_interface_lib)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(my_robot_msgs REQUIRED)

# 库：传感器接收
add_library(${PROJECT_NAME} SHARED
  src/node/sensor_node.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  std_msgs
  nav_msgs
  sensor_msgs
  my_robot_msgs
)

# 独立自检节点
add_executable(sensor_monitor
  src/node/sensor_monitor_main.cpp
)
target_link_libraries(sensor_monitor ${PROJECT_NAME})
ament_target_dependencies(sensor_monitor rclcpp)

# 安装
install(
  DIRECTORY include/
  DESTINATION include/
)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS sensor_monitor
  DESTINATION lib/${PROJECT_NAME}
)

# 导出配置
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
  rclcpp
  std_msgs
  nav_msgs
  sensor_msgs
  my_robot_msgs
)

ament_package()
