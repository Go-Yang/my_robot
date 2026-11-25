#!/usr/bin/env python3
"""用 world 几何 + Gazebo 真值位姿生成 2D LaserScan（TB4 仿真专用）。

这台开发机没有 3D 加速，gz 的 gpu_lidar 走 Mesa llvmpipe 时会长时间只输出
inf（实测连续 100 s 没有一帧有效数据），靠它无法建图/导航。这个节点改成
CPU 2D 射线投射：

  - 启动时解析 world SDF 里的静态 box 墙（地面 plane 忽略）
  - 订阅 /sim_ground_truth_pose（nav_msgs/Odometry，Gazebo 真值位姿）
  - 10 Hz 发布 360 点 LaserScan 到 /scan

扫描角度以机器人底盘 +x 为 0，frame_id 用 rplidar_link（它相对 base_link
偏航为 0）。建图/导航拿到的数据是稳定的，不再依赖 gz 的渲染。

参数：
    world_file        world SDF 绝对路径
    pose_topic        默认 /sim_ground_truth_pose
    scan_topic        默认 /scan
    frame_id          默认 rplidar_link
    rate              默认 10.0 Hz
    num_rays          默认 720（0.5° 角分辨率，比真实 A1M8 的 1° 更密，
                      用于减少远处墙体采样空隙）
    range_min/max     默认 0.164 / 12.0 m
    lidar_height      默认 0.1395 m（相对 base_link）
"""

import math
import re
import xml.etree.ElementTree as ET

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import LaserScan


def _local(tag: str) -> str:
    return tag.split('}')[-1]


def _parse_pose(text: str):
    values = [float(v) for v in (text or '0 0 0 0 0 0').split()]
    while len(values) < 6:
        values.append(0.0)
    return values[:6]


def _compose_2d(parent, child):
    """parent/child = (x, y, yaw)。返回复合变换。"""
    px, py, pyaw = parent
    cx, cy, cyaw = child
    return (px + math.cos(pyaw) * cx - math.sin(pyaw) * cy,
            py + math.sin(pyaw) * cx + math.cos(pyaw) * cy,
            pyaw + cyaw)


def _iter_boxes(world_file):
    """解析 world 里的静态 box 碰撞体，返回 (cx, cy, half_x, half_y, yaw, zmin, zmax)。"""
    text = open(world_file, 'r', errors='replace').read()
    # 这个 world 的注释里画了 ASCII 图，包含 "--"，XML 解析器会报错；
    # 先去掉注释再解析。
    text = re.sub(r'<!--.*?-->', '', text, flags=re.S)
    root = ET.fromstring(text)

    world = None
    for elem in root.iter():
        if _local(elem.tag) == 'world':
            world = elem
            break
    if world is None:
        return []

    boxes = []
    for model in world:
        if _local(model.tag) != 'model':
            continue
        mpose = _parse_pose(
            (model.find('pose').text if model.find('pose') is not None else None))
        for link in model:
            if _local(link.tag) != 'link':
                continue
            lpose = _parse_pose(
                (link.find('pose').text if link.find('pose') is not None else None))
            for collision in link:
                if _local(collision.tag) != 'collision':
                    continue
                cpose = _parse_pose(
                    (collision.find('pose').text
                     if collision.find('pose') is not None else None))
                size = collision.find('geometry/box/size')
                if size is None or size.text is None:
                    continue
                sx, sy, sz = [float(v) for v in size.text.split()]

                x, y, yaw = _compose_2d(
                    (mpose[0], mpose[1], mpose[5]),
                    (lpose[0], lpose[1], lpose[5]))
                x, y, yaw = _compose_2d(
                    (x, y, yaw), (cpose[0], cpose[1], cpose[5]))

                z = mpose[2] + lpose[2] + cpose[2]
                boxes.append((x, y, sx / 2.0, sy / 2.0, yaw,
                              z - sz / 2.0, z + sz / 2.0))
    return boxes


def _ray_box(ox, oy, dx, dy, box):
    """返回射线与 box 的最近相交距离，没有则 None。"""
    cx, cy, hx, hy, yaw, _, _ = box
    # 转到 box 局部坐标系
    c, s = math.cos(-yaw), math.sin(-yaw)
    rx, ry = ox - cx, oy - cy
    lx = c * rx - s * ry
    ly = s * rx + c * ry
    ldx = c * dx - s * dy
    ldy = s * dx + c * dy

    tmin, tmax = -math.inf, math.inf
    for origin, direction, extent in ((lx, ldx, hx), (ly, ldy, hy)):
        if abs(direction) < 1e-12:
            if origin < -extent or origin > extent:
                return None
            continue
        t1 = (-extent - origin) / direction
        t2 = (extent - origin) / direction
        if t1 > t2:
            t1, t2 = t2, t1
        tmin = max(tmin, t1)
        tmax = min(tmax, t2)
        if tmin > tmax:
            return None
    if tmax < 0.0:
        return None
    return tmin if tmin > 0.0 else tmax


class WorldLidar(Node):
    def __init__(self) -> None:
        super().__init__('world_lidar')
        world_file = self.declare_parameter('world_file', '').value
        pose_topic = self.declare_parameter(
            'pose_topic', '/sim_ground_truth_pose').value
        scan_topic = self.declare_parameter('scan_topic', '/scan').value
        self.frame_id = self.declare_parameter(
            'frame_id', 'rplidar_link').value
        self.rate = float(self.declare_parameter('rate', 10.0).value)
        self.num_rays = int(self.declare_parameter('num_rays', 720).value)
        self.range_min = float(self.declare_parameter(
            'range_min', 0.164).value)
        self.range_max = float(self.declare_parameter(
            'range_max', 12.0).value)
        self.lidar_height = float(self.declare_parameter(
            'lidar_height', 0.1395).value)

        if not world_file:
            raise RuntimeError('world_lidar: 必须提供 world_file 参数')

        all_boxes = _iter_boxes(world_file)
        self.boxes = []
        # 只保留与雷达扫描平面相交的 box（lidar_height 相对 base_link；
        # 机器人 base_link 在真值里的 z 约为 0，这里用 0 做近似）。
        for box in all_boxes:
            if box[5] <= self.lidar_height <= box[6]:
                self.boxes.append(box)

        self.pose = None
        self.last_pose_stamp = None

        qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=QoSReliabilityPolicy.RELIABLE)
        self.pub = self.create_publisher(LaserScan, scan_topic, qos)
        # 真值位姿由 gz 桥接/pose_republisher 发布，通常是 BEST_EFFORT；
        # 订阅端也用 BEST_EFFORT 才能兼容两种 QoS。
        pose_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=QoSReliabilityPolicy.BEST_EFFORT)
        self.sub = self.create_subscription(
            Odometry, pose_topic, self.pose_callback, pose_qos)

        self.timer = self.create_timer(
            1.0 / max(self.rate, 1.0), self.publish_scan)
        self.get_logger().info(
            f'world_lidar: world={world_file}, boxes={len(self.boxes)}, '
            f'rays={self.num_rays}, rate={self.rate} Hz, '
            f'scan_topic={scan_topic}, frame={self.frame_id}')

    def pose_callback(self, msg: Odometry) -> None:
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self.pose = (p.x, p.y, yaw, p.z)
        self.last_pose_stamp = msg.header.stamp

    def publish_scan(self) -> None:
        if self.pose is None:
            return

        x, y, yaw, z = self.pose
        scan = LaserScan()
        scan.header.stamp = self.get_clock().now().to_msg()
        scan.header.frame_id = self.frame_id
        scan.angle_min = 0.0
        scan.angle_increment = 2.0 * math.pi / self.num_rays
        scan.angle_max = 2.0 * math.pi - scan.angle_increment
        scan.time_increment = 0.0
        scan.scan_time = 1.0 / max(self.rate, 1.0)
        scan.range_min = self.range_min
        scan.range_max = self.range_max

        ranges = []
        for i in range(self.num_rays):
            angle = yaw + i * scan.angle_increment
            dx, dy = math.cos(angle), math.sin(angle)
            nearest = math.inf
            for box in self.boxes:
                t = _ray_box(x, y, dx, dy, box)
                if t is not None and t < nearest:
                    nearest = t
            if nearest == math.inf:
                ranges.append(math.inf)
            else:
                ranges.append(
                    min(max(nearest, self.range_min), self.range_max))

        scan.ranges = ranges
        self.pub.publish(scan)


def main() -> None:
    rclpy.init()
    node = WorldLidar()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
