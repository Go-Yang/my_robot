#!/bin/bash

# 工作空间根目录（取脚本所在目录）
WORKSPACE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# 获取用户输入的构建类型
BUILD_TYPE=$1

# 定义帮助信息
function print_help() {
  echo "Usage: $0 [lib | core | main]"
  echo "  lib   : 编译所有库（跳过 my_robot_project 与 my_robot_bringup）"
  echo "  core  : 单独编译某个核心包及其依赖（例如 core robot_control_lib）"
  echo "  main  : 仅编译 my_robot_project + my_robot_bringup 并运行"
  exit 1
}

# 清理函数
function clean_workspace() {
  echo "Cleaning workspace..."
  rm -rf "$WORKSPACE/build/" "$WORKSPACE/install/" "$WORKSPACE/log/"
}

# 构建函数
function build_lib_except_main() {
  echo "Building lib packages except my_robot_project and my_robot_bringup..."
  cd "$WORKSPACE"
  # bringup 依赖 my_robot_project（package.xml 的 exec_depend），
  # 跳过 main 时 colcon 会因找不到它而拒绝构建 bringup，所以两个一起跳过。
  colcon build --symlink-install \
    --base-paths "$WORKSPACE/src" \
    --packages-skip my_robot_project my_robot_bringup
}

function build_core_package() {
  PACKAGE=$2
  if [ -z "$PACKAGE" ]; then
    echo "Please specify the package name to build, e.g., $0 core robot_control_lib"
    exit 1
  fi
  echo "Building package: $PACKAGE (含其依赖)..."
  cd "$WORKSPACE"
  # 用 --packages-up-to 而非 --packages-select：前面清过 workspace，
  # 被选包的依赖（如 my_robot_msgs）也一并重建，否则会因找不到依赖而失败
  colcon build --symlink-install \
    --base-paths "$WORKSPACE/src" \
    --packages-up-to "$PACKAGE"
}

function build_main_package() {
  echo "Building my_robot_project and my_robot_bringup..."
  cd "$WORKSPACE"
  colcon build --symlink-install \
    --base-paths "$WORKSPACE/src" \
    --packages-select my_robot_project my_robot_bringup
}

function run_main_node() {
  echo "Setting up environment..."
  source /opt/ros/jazzy/setup.bash
  source "$WORKSPACE/install/setup.bash"

  echo "Running main_node..."
  ros2 run my_robot_project main_node
}

# 判断用户输入
case "$BUILD_TYPE" in
  lib)
    clean_workspace
    build_lib_except_main
    ;;
  core)
    clean_workspace
    build_core_package "$@"
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
