#!/bin/bash

# 工作空间根目录（取脚本所在目录）
WORKSPACE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# 清理工作空间
echo "Cleaning workspace..."
rm -rf "$WORKSPACE/build/" "$WORKSPACE/install/" "$WORKSPACE/log/"

# 设置环境变量（必须先 source ROS，再 source 工作空间）
echo "Setting up ROS environment..."
source /opt/ros/jazzy/setup.bash

# 构建工作空间
echo "Building workspace..."
cd "$WORKSPACE"
colcon build --symlink-install

source "$WORKSPACE/install/setup.bash"

# 检查包是否存在
echo "Checking if the package exists..."
ros2 pkg list | grep my_robot_project

# 运行节点
echo "Running main_node..."
ros2 run my_robot_project main_node

# 提示用户操作结束
echo "Build and run completed!"
