#!/usr/bin/env python3
"""等第一帧 /scan 到达后退出，用于在仿真就绪后再启动 slam_toolbox。"""
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan


class WaitForScan(Node):
    def __init__(self):
        super().__init__('wait_for_scan')
        self.done = False
        self.create_subscription(
            LaserScan, '/scan', self.on_scan, qos_profile_sensor_data)

    def on_scan(self, msg):
        if not self.done:
            self.done = True
            self.get_logger().info(
                f'收到 /scan（{len(msg.ranges)} 点），仿真已就绪')


def main():
    rclpy.init()
    node = WaitForScan()
    while rclpy.ok() and not node.done:
        rclpy.spin_once(node, timeout_sec=0.5)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
