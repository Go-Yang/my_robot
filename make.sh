#!/bin/bash

# 设置工作空间路径
WORKSPACE=~/my_robot

# 获取用户输入的构建类型
BUILD_TYPE=$1

# 定义帮助信息
function print_help() {
  echo "Usage: $0 [lib | core | main]"
  echo "  lib     : 编译所有包（不包含 my_robot_main）"
  echo "  core    : 单独编译某个核心包（例如 robot_control_lib）"
  echo "  main : 仅编译 my_robot_main 包"
  exit 1
}

# 清理函数
function clean_workspace() {
  echo "Cleaning workspace..."
  rm -rf $WORKSPACE/build/ $WORKSPACE/instlib/ $WORKSPACE/log/
}

# 构建函数
function build_lib_except_main() {
  echo "Building lib packages except my_robot_main..."
  colcon build --symlink-instlib \
    --base-paths $WORKSPACE/src \
    --packages-skip my_robot_main
}

function build_core_package() {
  PACKAGE=$2
  if [ -z "$PACKAGE" ]; then
    echo "Please specify the package name to build, e.g., $0 core robot_control_lib"
    exit 1
  fi
  echo "Building package: $PACKAGE..."
  colcon build --symlink-instlib \
    --base-paths $WORKSPACE/src \
    --packages-select $PACKAGE
}

function build_main_package() {
  echo "Building my_robot_main only..."
  colcon build --symlink-instlib \
    --base-paths $WORKSPACE/src \
    --packages-select my_robot_main
}

function run_main_node() {
  echo "Setting up environment..."
  source /opt/ros/humble/setup.bash
  source $WORKSPACE/instlib/setup.bash

  echo "Running main_node..."
  ros2 run my_robot_main main_node
}

# 判断用户输入
case "$BUILD_TYPE" in
  lib)
    clean_workspace
    build_lib_except_main
    ;;
  core)
    clean_workspace
    build_core_package $@
    ;;
  main)
    build_main_package
    run_main_node
    ;;
  *)
    print_help
    ;;
esac

# 提示
echo "Build script completed."
exit 0
