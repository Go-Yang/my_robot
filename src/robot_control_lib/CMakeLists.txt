# cmake_minimum_required(VERSION 3.5)
# project(robot_control_lib)

# # 查找依赖
# find_package(ament_cmake REQUIRED)
# find_package(rclcpp REQUIRED)
# find_package(my_robot_msgs REQUIRED)

# # 添加库
# add_library(${PROJECT_NAME} src/control.cpp)

# # 包含头文件目录
# target_include_directories(${PROJECT_NAME} PUBLIC
#   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#   $<INSTALL_INTERFACE:include>
# )

# # 链接依赖
# target_link_libraries(${PROJECT_NAME}
#   rclcpp::rclcpp
#   my_robot_msgs::my_robot_msgs
# )

# # 安装库
# install(TARGETS ${PROJECT_NAME}
#   ARCHIVE DESTINATION lib
#   LIBRARY DESTINATION lib
#   RUNTIME DESTINATION bin
# )

# # 安装头文件
# install(DIRECTORY include/
#   DESTINATION include
# )

# ament_package()
cmake_minimum_required(VERSION 3.8)
project(robot_control_lib)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(my_robot_msgs REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/control.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  my_robot_msgs
)

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
ament_export_dependencies(rclcpp my_robot_msgs)

ament_package()
