#!/usr/bin/env python3
"""
@Author: huangshiheng@agibot.com
@Date: 2025-11-06
@Description: Python node to subscribe to motor_pos topic
"""

import rclpy
from rclpy.node import Node
import sys
import os

# 动态获取项目根路径
script_dir = os.path.dirname(os.path.abspath(__file__))
project_root = os.path.dirname(os.path.dirname(script_dir))
python_packages_path = os.path.join(project_root, "build/install/local/lib/python3.10/dist-packages")

sys.path.append(python_packages_path)

from omnihand_pro_node_msgs.msg import MotorPos


class MotorPosSubscriber(Node):
    def __init__(self):
        super().__init__('motor_pos_subscriber')
        
        # 创建订阅器
        self.subscription = self.create_subscription(
            MotorPos,
            '/agihand/omnihand/left/motor_pos',
            self.motor_pos_callback,
            10
        )
        self.subscription  # prevent unused variable warning

        self.get_logger().info('Motor Position Subscriber Node started')

    def motor_pos_callback(self, msg):
        """处理接收到的MotorPos消息"""
        self.get_logger().info(
            f'Received motor positions: {msg.pos}, '
            f'frame_id: {msg.header.frame_id}, '
            f'timestamp: {msg.header.stamp.sec}.{msg.header.stamp.nanosec}'
        )

def main(args=None):
    rclpy.init(args=args)
    
    motor_pos_subscriber = MotorPosSubscriber()
    
    try:
        rclpy.spin(motor_pos_subscriber)
    except KeyboardInterrupt:
        pass
    
    motor_pos_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
