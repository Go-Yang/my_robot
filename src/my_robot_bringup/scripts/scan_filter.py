#!/usr/bin/env python3
"""过滤 gz GPU 雷达的坏帧，再发布到 /scan。

这台开发机没有 3D 加速，gz 的 gpu_lidar 走 Mesa llvmpipe，会间歇性只渲染
出部分扇面（一部分 range 是 inf），偶尔整帧都是 inf。部分帧里的有效射线
仍然有用，全部丢掉会让激光频率低到无法导航；所以只丢弃**有效点太少**的帧，
部分帧照常转发。导航侧已经处理缺失扇面（前方无回波时按受阻处理），建图
侧也跳过了 inf 射线，两边都不会把缺失数据当成空旷。

参数：
    input_topic       默认 /scan_raw
    output_topic      默认 /scan
    min_valid_points  默认 120（360 点模型里少于 120 个有效点就丢弃）
"""

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import LaserScan


class ScanFilter(Node):
    def __init__(self) -> None:
        super().__init__('scan_filter')
        self.input_topic = self.declare_parameter(
            'input_topic', '/scan_raw').value
        self.output_topic = self.declare_parameter(
            'output_topic', '/scan').value
        self.min_valid_points = int(self.declare_parameter(
            'min_valid_points', 120).value)

        qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=QoSReliabilityPolicy.RELIABLE)

        self.pub = self.create_publisher(LaserScan, self.output_topic, qos)
        self.sub = self.create_subscription(
            LaserScan, self.input_topic, self.callback, qos)

        self.dropped = 0
        self.forwarded = 0
        self.get_logger().info(
            f'scan_filter: {self.input_topic} -> {self.output_topic}, '
            f'min_valid_points={self.min_valid_points}')

    def callback(self, msg: LaserScan) -> None:
        total = len(msg.ranges)
        if total == 0:
            return

        valid = 0
        for r in msg.ranges:
            if math.isfinite(r) and r > msg.range_min:
                valid += 1

        if valid < self.min_valid_points:
            self.dropped += 1
            self.get_logger().warn(
                f'丢弃坏扫描帧：仅 {valid}/{total} 个有效点'
                f'（累计丢弃 {self.dropped} 帧）',
                throttle_duration_sec=5.0)
            return

        self.forwarded += 1
        self.pub.publish(msg)


def main() -> None:
    rclpy.init()
    node = ScanFilter()
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
