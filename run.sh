#!/bin/bash

# 设置环境变量
echo "Setting up environment..."
source install/setup.bash
source /opt/ros/humble/setup.bash  # 根据你的 ROS 版本调整

# 检查包是否存在
echo "Checking if the package exists..."
ros2 pkg list | grep my_robot_project

# 运行节点
echo "Running main_node..."
# ros2 run my_robot_project main_node
./install/my_robot_project/lib/my_robot_project/main_node

# 提示用户操作结束
echo "Build and run completed!"
