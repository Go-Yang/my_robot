#!/usr/bin/env python3
"""生成 TurtleBot 4 的 URDF，并针对本项目的建图/导航做裁剪。

官方 irobot_create_description 在 gazebo:=ignition 下的仿真模型有几处会
让本项目的 odom_grid 建图失效或在这台无 3D 加速的开发机上表现异常：

1. 所有 gpu_lidar（RPLIDAR + 悬崖 + 红外）都去掉。激光改由
   `world_lidar.py` 用 world 几何 + Gazebo 真值位姿做 CPU 2D 射线投射，
   不再依赖 gz 的 Ogre 渲染（llvmpipe 下 gpu_lidar 会长时间只输出 inf）。
2. 模型自带一个 Sensors 系统插件（render_engine=ogre），和 world 里的
   Sensors 系统（ogre2）重复；一并去掉。
3. 相机不用，去掉相机支架的视觉/碰撞网格（link/TF 保留）。
4. RPLIDAR link 保留给 RViz/TF；合成扫描按 0° 对齐底盘 +x，360 点、
   10 Hz，由 world_lidar.py 发布。

裁剪后只影响仿真外观/仿真传感器，不改变底盘运动学和 RPLIDAR 的
LaserScan 接口。

用法（供 robot_state_publisher 的 Command 调用）：
    tb4_urdf.py xacro <urdf.xacro> gazebo:=ignition
"""

import subprocess
import sys
import xml.etree.ElementTree as ET

# 全部 gpu_lidar 都去掉：激光改由 world_lidar.py 用 CPU 射线投射生成，
# gz 不再需要任何渲染类传感器。
KEEP_LIDARS = set()


def main() -> int:
    if len(sys.argv) < 2:
        print('usage: tb4_urdf.py <command> [args...]', file=sys.stderr)
        return 2

    result = subprocess.run(sys.argv[1:], capture_output=True, text=True)
    if result.returncode != 0:
        sys.stderr.write(result.stderr)
        return result.returncode

    root = ET.fromstring(result.stdout)

    removed = 0
    for parent in root.iter():
        for child in list(parent):
            if (child.tag == 'sensor'
                    and child.get('type') == 'gpu_lidar'
                    and child.get('name') not in KEEP_LIDARS):
                parent.remove(child)
                removed += 1

    # create3 的 URDF 自带一个模型级 Sensors 系统插件（render_engine=ogre），
    # 而 world 里已经有 Sensors 系统（ogre2）。两个系统会对同一个雷达各发布
    # 一路数据：ogre2 那路正常，ogre 那路全是 range_min/inf，两路交错到达
    # /scan，建图就废了。去掉模型级的 Sensors 插件，只保留 world 的。
    for parent in root.iter():
        for child in list(parent):
            if (child.tag == 'plugin'
                    and (child.get('name') == 'gz::sim::systems::Sensors'
                         or 'sensors-system' in child.get('filename', ''))):
                parent.remove(child)

    # 相机支架（OAK-D bracket）的视觉网格在雷达正后方 9 cm 处，会把后方
    # 约 90° 的射线整段挡成 inf。本项目不用相机，去掉这个支架的视觉/碰撞
    # 网格；雷达自己的网格保留（渲染场景需要 link 有 visual 才稳定）。
    for link in root.iter('link'):
        if link.get('name') == 'oakd_camera_bracket':
            for child in list(link):
                if child.tag in ('visual', 'collision'):
                    link.remove(child)

    # 注意：成功时不要往 stderr 写任何东西，launch 的 Command 会把
    # stderr 非空当成失败。
    _ = removed
    sys.stdout.write(ET.tostring(root, encoding='unicode'))
    return 0


if __name__ == '__main__':
    sys.exit(main())
