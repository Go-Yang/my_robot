cmake_minimum_required(VERSION 3.5)  # 推荐使用ROS2 Humble/Iron要求的3.8+
project(sensor_interface_lib)


# 2. 查找依赖
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(my_robot_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

# 3. 添加库
add_library(${PROJECT_NAME} SHARED
  src/node/sensor_node.cpp
)

# 4. 包含目录（更安全的配置）
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  # 核心代码目录（如果头文件在src/core中）
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src/core>
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  std_msgs
  sensor_msgs
  my_robot_msgs
)

# 6. 安装规则（增强兼容性）
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
  # INCLUDES DESTINATION include  # 自动包含头文件
)

# 7. 导出配置（更完整）
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
  rclcpp
  std_msgs
  sensor_msgs
  my_robot_msgs
)

ament_package()