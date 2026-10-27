#!/bin/bash

# 工作空间根目录（取脚本所在目录，项目放在任意路径都不用改）
WORKSPACE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# 清理工作空间
echo "Cleaning workspace..."
rm -rf "$WORKSPACE/build/" "$WORKSPACE/install/" "$WORKSPACE/log/"

# 设置环境变量（必须先 source ROS，再 source 工作空间）
echo "Setting up ROS environment..."
source /opt/ros/jazzy/setup.bash

# 构建工作空间（colcon 会自动按依赖顺序构建，无需逐个 --packages-select）
echo "Building workspace..."
cd "$WORKSPACE"
colcon build --symlink-install

# 设置工作空间环境变量
source "$WORKSPACE/install/setup.bash"

# 检查包是否存在
echo "Checking if the package exists..."
ros2 pkg list | grep my_robot_project

echo "Build completed!"
