#!/bin/bash

# 设置工作空间路径
WORKSPACE=~/my_robot

# 清理工作空间
echo "Cleaning workspace..."
rm -rf $WORKSPACE/build/ $WORKSPACE/install/ $WORKSPACE/log/

# 构建工作空间
echo "Building workspace..."
# colcon build --symlink-install --base-path $WORKSPACE
colcon build --packages-select my_robot_msgs
colcon build --packages-select mapping_lib
colcon build --packages-select robot_control_lib
colcon build --packages-select navigation_lib
colcon build --packages-select sensor_interface_lib
# build last
colcon build --packages-select my_robot_project 

# 设置环境变量
echo "Setting up environment..."
source /opt/ros/humble/setup.bash   # 先 source ROS 环境，保证 colcon 能正常工作
source $WORKSPACE/install/setup.bash  # 再 source 工作空间环境

# 检查包是否存在
echo "Checking if the package exists..."
ros2 pkg list | grep my_robot_project

# 运行节点
echo "Running main_node..."
# ros2 run my_robot_project main_node
# 或者直接运行编译出来的可执行文件（不推荐，环境变量可能不完整）
./install/my_robot_project/lib/my_robot_project/main_node

# 提示用户操作结束
echo "Build and run completed!"
