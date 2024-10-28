cmake_minimum_required(VERSION 3.5)
project(navigation_lib)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

include_directories(include ${rclcpp_INCLUDE_DIRS})

add_library(${PROJECT_NAME} SHARED src/navigation.cpp)

ament_target_dependencies(${PROJECT_NAME} rclcpp)

install(TARGETS
  ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_package()
