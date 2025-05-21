cmake_minimum_required(VERSION 3.5)
project(my_robot_project)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(my_robot_msgs REQUIRED)
find_package(robot_control_lib REQUIRED)
find_package(sensor_interface_lib REQUIRED)
find_package(mapping_lib REQUIRED)
find_package(navigation_lib REQUIRED)

add_executable(main_node src/main.cpp)

target_link_libraries(main_node
  ${rclcpp_LIBRARIES}
  robot_control_lib::robot_control_lib
  sensor_interface_lib::sensor_interface_lib
  mapping_lib::mapping_lib
  navigation_lib::navigation_lib
)

ament_target_dependencies(main_node
  rclcpp
  my_robot_msgs
  robot_control_lib
  sensor_interface_lib
  mapping_lib
  navigation_lib
)

install(TARGETS main_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
