cmake_minimum_required(VERSION 3.5)
project(mapping_lib)

# 查找依赖
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(my_robot_msgs REQUIRED)

# 添加库
add_library(${PROJECT_NAME} SHARED
  src/mapping.cpp
)

# 包含头文件目录
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  # 添加ROS接口目录
  ${rclcpp_INCLUDE_DIRS}
  ${std_msgs_INCLUDE_DIRS}
)

# 链接依赖
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  std_msgs
  my_robot_msgs
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

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
  rclcpp
  std_msgs  # 必须添加
  my_robot_msgs
)

ament_package()
