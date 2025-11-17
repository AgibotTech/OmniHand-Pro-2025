#!/usr/bin/env python3
"""
@Author: huangshiheng@agibot.com
@Date: 2025-11-06
@Description: Python node to subscribe left hand motor velocity feedback
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

from omnihand_node_msgs.msg import MotorVel


class LeftMotorVelSubscriber(Node):
    def __init__(self):
        super().__init__('left_motor_vel_subscriber')
        
        # 创建左手电机速度反馈订阅器
        self.subscription = self.create_subscription(
            MotorVel,
            '/agihand/omnihand_pro/left/motor_vel',
            self.motor_vel_callback,
            10
        )
        self.subscription

        self.get_logger().info('Left Motor Velocity Subscriber Node started')

    def motor_vel_callback(self, msg):
        """处理电机速度反馈消息"""
        self.get_logger().info(
            f'Received left motor velocity feedback: {msg.vels}'
        )
        
        # 这里可以添加速度反馈处理逻辑
        # 例如：运动监控、安全检查、数据记录等


def main(args=None):
    rclpy.init(args=args)
    
    left_motor_vel_subscriber = LeftMotorVelSubscriber()
    
    try:
        rclpy.spin(left_motor_vel_subscriber)
    except KeyboardInterrupt:
        pass
    
    left_motor_vel_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
