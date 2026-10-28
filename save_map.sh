#!/bin/bash

# 把当前 /map 保存成 PGM + YAML（必须在仿真/建图运行时执行）
#
# 用法：
#   ./save_map.sh                     # 保存到 maps/room_map.pgm / .yaml
#   ./save_map.sh maps/house_2        # 自定义输出前缀
#
# 说明：
# - odom_grid 和 slam_toolbox 后端都发布 /map，脚本不区分后端。
# - 保存的是当前已建好的地图；仿真退出后 /map 就没了，需要重新建图。

# 工作空间根目录（取脚本所在目录）
WORKSPACE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

PREFIX="${1:-$WORKSPACE/maps/room_map}"

source /opt/ros/jazzy/setup.bash
if [ ! -f "$WORKSPACE/install/setup.bash" ]; then
  echo "工作空间尚未构建，请先运行：$WORKSPACE/build.sh" >&2
  exit 1
fi
source "$WORKSPACE/install/setup.bash"

if ! ros2 topic list 2>/dev/null | grep -qx '/map'; then
  echo "当前没有 /map 话题，请先在另一个终端启动仿真/建图。" >&2
  exit 1
fi

mkdir -p "$(dirname "$PREFIX")"

echo "Saving map to ${PREFIX}.pgm / .yaml ..."
exec ros2 run nav2_map_server map_saver_cli -f "$PREFIX" \
  --ros-args -p map_subscribe_transient_local:=true -p save_map_timeout:=10.0
