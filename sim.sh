#!/bin/bash

# 一键启动仿真 + 主节点（内部自动 source ROS 与工作空间）
#
# 用法：
#   ./sim.sh                          # Gazebo GUI + 仿真 + 主节点
#   ./sim.sh headless:=true           # 只跑 Gazebo server
#   ./sim.sh rviz:=true               # 同时打开 RViz
#   ./sim.sh explore:=true            # 前沿自动探索建图
#   ./sim.sh mapping_backend:=slam_toolbox rviz:=true   # 用 slam_toolbox 建图（默认 odom_grid）
#   ./sim.sh model:=waffle            # 换机器人型号
#   ./sim.sh --show-args              # 只查看可用参数

# 工作空间根目录（取脚本所在目录，项目放在任意路径都不用改）
WORKSPACE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# 必须先 source ROS，再 source 工作空间
source /opt/ros/jazzy/setup.bash

if [ ! -f "$WORKSPACE/install/setup.bash" ]; then
  echo "工作空间尚未构建，请先运行：$WORKSPACE/build.sh" >&2
  exit 1
fi
source "$WORKSPACE/install/setup.bash"

# make.sh lib 会跳过 my_robot_bringup，这里显式检查一下
if ! ros2 pkg prefix my_robot_bringup >/dev/null 2>&1; then
  echo "找不到 my_robot_bringup，请先运行：$WORKSPACE/build.sh（或 ./make.sh main）" >&2
  exit 1
fi

# 防止两个仿真实例同时跑：它们会同时发布 /map 和 map->odom，RViz 会叠图
EXISTING=$(ps -eo pid,cmd --no-headers | grep -E 'my_robot_project/main_node|gz sim -r -s' | grep -v grep | awk '{print $1}' | tr '\n' ' ')
if [ -n "${EXISTING// /}" ]; then
  echo "检测到已有仿真在运行（PID: $EXISTING）。" >&2
  echo "两个实例会同时发布 /map 和 map->odom，导致叠图；请先停止旧实例（Ctrl-C 或 kill 上面的 PID）再启动。" >&2
  exit 1
fi

# 其余参数原样传给 launch
exec ros2 launch my_robot_bringup bringup.launch.py "$@"
