# cmake_minimum_required(VERSION 3.5)
# project(my_robot_msgs)

# # 查找依赖
# find_package(ament_cmake REQUIRED)
# find_package(rosidl_default_generators REQUIRED)

# # 生成消息接口
# rosidl_generate_interfaces(${PROJECT_NAME}
#   "msg/MyCustomMessage.msg"
# )

# # 导出库
# ament_export_libraries(${PROJECT_NAME} ${rosidl_generate_interfaces_TARGETS})

# # 安装和导出
# ament_package()
cmake_minimum_required(VERSION 3.8)
project(my_robot_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/MyCustomMessage.msg"
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
