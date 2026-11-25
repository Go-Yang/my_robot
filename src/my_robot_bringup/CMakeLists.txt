cmake_minimum_required(VERSION 3.8)
project(my_robot_bringup)

find_package(ament_cmake REQUIRED)

# 纯 launch / 资源包，没有需要编译的代码
install(DIRECTORY
  launch
  config
  worlds
  models
  urdf
  rviz
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/wait_for_scan.py
  scripts/activate_node.sh
  scripts/tb4_urdf.py
  scripts/scan_filter.py
  scripts/world_lidar.py
  DESTINATION share/${PROJECT_NAME}/scripts
)

ament_package()
