#!/usr/bin/env python3
"""
订阅 /scan 话题并打印激光扫描数据中的一个元素
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


class ScanSubscriber(Node):
    def __init__(self):
        super().__init__('scan_subscriber')

        # 创建订阅者，订阅 /scan 话题
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10  # QoS 队列深度
        )
        self.subscription  # 防止未使用变量警告
        self.get_logger().info('Scan subscriber node started, waiting for /scan messages...')

    def scan_callback(self, msg: LaserScan):
        """
        激光扫描数据回调函数
        """
        # 检查 ranges 数组是否有数据
        if len(msg.ranges) > 0:
            # 打印数组中间位置的元素（通常是机器人正前方的距离）
            middle_index = 270
            distance = msg.ranges[middle_index]

            self.get_logger().info(
                f'Ranges array length: {len(msg.ranges)}, '
                f'Angle element [index {middle_index}]: {distance:.3f} m'
            )
        else:
            self.get_logger().warn('Received empty ranges array')


def main(args=None):
    rclpy.init(args=args)

    scan_subscriber = ScanSubscriber()

    try:
        rclpy.spin(scan_subscriber)
    except KeyboardInterrupt:
        pass
    finally:
        scan_subscriber.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
