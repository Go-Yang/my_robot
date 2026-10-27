#!/bin/bash

# 工作空间根目录（取脚本所在目录）
WORKSPACE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# 设置环境变量（必须先 source ROS，再 source 工作空间，顺序反了工作空间会被覆盖）
echo "Setting up environment..."
source /opt/ros/jazzy/setup.bash
source "$WORKSPACE/install/setup.bash"

# 检查包是否存在
echo "Checking if the package exists..."
ros2 pkg list | grep my_robot_project

# 运行节点
echo "Running main_node..."
ros2 run my_robot_project main_node
