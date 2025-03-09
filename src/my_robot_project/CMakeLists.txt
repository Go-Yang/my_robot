cmake_minimum_required(VERSION 3.5)
project(my_robot_project)

# 查找依赖
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(my_robot_msgs REQUIRED)

# 添加子库
add_subdirectory(../robot_control_lib ${CMAKE_CURRENT_BINARY_DIR}/robot_control_lib)
add_subdirectory(../sensor_interface_lib ${CMAKE_CURRENT_BINARY_DIR}/sensor_interface_lib)
add_subdirectory(../mapping_lib ${CMAKE_CURRENT_BINARY_DIR}/mapping_lib)
add_subdirectory(../navigation_lib ${CMAKE_CURRENT_BINARY_DIR}/navigation_lib)

# 添加主函数可执行文件
add_executable(main_node src/main.cpp)

# # 在编译时添加头文件路径
# target_include_directories(main_node
#   PRIVATE
#     ${CMAKE_CURRENT_SOURCE_DIR}/../robot_control_lib/include  # 添加控制库头文件
#     ${CMAKE_CURRENT_SOURCE_DIR}/../sensor_interface_lib/include  # 添加传感器库头文件
#     ${CMAKE_CURRENT_SOURCE_DIR}/../mapping_lib/include  # 添加地图库头文件
#     ${CMAKE_CURRENT_SOURCE_DIR}/../navigation_lib/include  # 添加导航库头文件
# )

# 链接依赖
target_link_libraries(main_node
  ${rclcpp_LIBRARIES}
  robot_control_lib
  sensor_interface_lib
  mapping_lib
  navigation_lib
)

# 链接消息
ament_target_dependencies(main_node
  rclcpp
  my_robot_msgs  # 确保链接了自定义消息
)

# 安装目标
install(TARGETS
  main_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
