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
source $WORKSPACE/install/setup.bash
source /opt/ros/humble/setup.bash  # 根据你的 ROS 版本调整

# 检查包是否存在
echo "Checking if the package exists..."
ros2 pkg list | grep my_robot_project
