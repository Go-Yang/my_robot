#!/bin/bash

# 设置工作空间路径
WORKSPACE=~/my_robot

# 清理工作空间
echo "Cleaning workspace..."
rm -rf $WORKSPACE/build/ $WORKSPACE/install/ $WORKSPACE/log/

# 构建工作空间
echo "Building workspace..."
colcon build --symlink-install --base-path $WORKSPACE

# 设置环境变量
echo "Setting up environment..."
source $WORKSPACE/install/setup.bash
source /opt/ros/foxy/setup.bash  # 根据你的 ROS 版本调整

# 检查包是否存在
echo "Checking if the package exists..."
ros2 pkg list | grep my_robot_project

# 运行节点
echo "Running main_node..."
# ros2 run my_robot_project main_node
./install/my_robot_project/lib/my_robot_project/main_node

# 提示用户操作结束
echo "Build and run completed!"
