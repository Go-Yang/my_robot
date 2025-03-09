cmake_minimum_required(VERSION 3.5)
project(mapping_lib)

# 查找依赖
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(my_robot_msgs REQUIRED)

# 添加库
add_library(${PROJECT_NAME} src/mapping.cpp)

# 包含头文件目录
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# 链接依赖
target_link_libraries(${PROJECT_NAME}
  rclcpp::rclcpp
  my_robot_msgs::my_robot_msgs
)

# 安装库
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# 安装头文件
install(DIRECTORY include/
  DESTINATION include
)

ament_package()
# cmake_minimum_required(VERSION 3.8)
# project(mapping_lib)

# find_package(ament_cmake REQUIRED)
# find_package(rclcpp REQUIRED)
# find_package(my_robot_msgs REQUIRED)

# add_library(${PROJECT_NAME} SHARED
#   src/mapping.cpp
# )

# target_include_directories(${PROJECT_NAME} PUBLIC
#   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#   $<INSTALL_INTERFACE:include>
# )

# ament_target_dependencies(${PROJECT_NAME}
#   rclcpp
#   my_robot_msgs
# )

# install(
#   DIRECTORY include/
#   DESTINATION include
# )

# install(
#   TARGETS ${PROJECT_NAME}
#   EXPORT export_${PROJECT_NAME}
#   LIBRARY DESTINATION lib
#   ARCHIVE DESTINATION lib
#   RUNTIME DESTINATION bin
# )

# ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
# ament_export_dependencies(rclcpp my_robot_msgs)

# ament_package()

主要区别：
add_library 定义：

第一段代码：使用 add_library(${PROJECT_NAME} src/mapping.cpp) 创建一个普通的静态或共享库。默认情况下会创建一个静态库。你还没有指定 SHARED 或 STATIC，因此默认是静态库。
第二段代码：使用 add_library(${PROJECT_NAME} SHARED src/mapping.cpp) 创建一个共享库（.so 文件）。如果你希望库是动态加载的共享库，你可以使用这种方式。
ament_target_dependencies 和 target_link_libraries：

第一段代码：使用 target_link_libraries(${PROJECT_NAME} rclcpp::rclcpp my_robot_msgs::my_robot_msgs) 来手动链接依赖的库。这是指定依赖项并链接库的方式。
第二段代码：使用 ament_target_dependencies(${PROJECT_NAME} rclcpp my_robot_msgs) 来让 ament 自动处理依赖项的链接。这是 ROS2 推荐的更简洁的方式，避免了手动指定依赖的路径和库。
安装配置：

第一段代码：使用 install(TARGETS ${PROJECT_NAME} ...) 和 install(DIRECTORY include/ ...) 来手动安装目标库和头文件。
第二段代码：也是采用了相似的安装命令，但 install 语句被注释掉了。它的 install 语法没有太大差别，主要在导出依赖和库的配置上。
ament_export_targets 和 ament_export_dependencies：

第一段代码：没有使用这些导出功能，表示库不希望暴露目标给其他依赖的库。
第二段代码：使用了 ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET) 和 ament_export_dependencies(rclcpp my_robot_msgs) 来显式导出库目标和依赖项。这是为了将此库暴露给其他依赖的 ROS2 包，并允许他们链接这个库。
CMake 最小版本：

第一段代码：使用 cmake_minimum_required(VERSION 3.5)。
第二段代码：使用 cmake_minimum_required(VERSION 3.8)，这表明第二段代码需要一个较高版本的 CMake（3.8 及以上），而第一段代码则适配于较旧的 CMake 版本。
总结：
第一段代码是更简洁和现代的方式，推荐使用。它更加灵活且易于维护。
第二段代码是早期版本的配置方式，使用了 ament_target_dependencies 和导出功能，这对库的共享和导出有帮助，但通常来说，如果没有其他 ROS2 包需要依赖它，可以省略这些部分。
如果你的库不需要暴露给其他 ROS2 包（例如，它仅作为本地库使用），你可以保持第一段代码并简化它。如果需要更严格的包导出和依赖处理，你可以使用第二段代码。