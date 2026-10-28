#!/bin/bash
# 带重试地把一个 lifecycle 节点 configure + activate 到 active。
# 用法：activate_node.sh <节点名>
# 背景：负载高时 slam_toolbox 的 change_state 服务响应可能超时，
# 官方 launch 的一次性生命周期事件会失败，节点停在 inactive。

NODE="${1:-/slam_toolbox}"

for i in $(seq 1 30); do
  state=$(ros2 lifecycle get "$NODE" 2>/dev/null | awk '{print $1}')
  if [ "$state" = "active" ]; then
    echo "[activate] $NODE 已激活"
    exit 0
  fi
  if [ "$state" = "unconfigured" ]; then
    ros2 lifecycle set "$NODE" configure >/dev/null 2>&1
  elif [ "$state" = "inactive" ]; then
    ros2 lifecycle set "$NODE" activate >/dev/null 2>&1
  fi
  sleep 1
done

echo "[activate] 无法激活 $NODE（最后状态: ${state:-unknown}）" >&2
exit 1
