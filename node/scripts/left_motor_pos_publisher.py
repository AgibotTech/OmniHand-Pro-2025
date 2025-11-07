#!/usr/bin/env python3
"""
@Author: huangshiheng@agibot.com
@Date: 2025-11-06
@Description: Python node to publish left hand motor position commands
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


class LeftMotorPosPublisher(Node):
    def __init__(self):
        super().__init__('left_motor_pos_publisher')
        
        # 创建左手电机位置命令发布器
        self.left_motor_pos_cmd_publisher = self.create_publisher(
            MotorPos,
            '/agihand/omnihand/left/motor_pos_cmd',
            10
        )
        
        # 创建定时器，每100ms发布一次命令
        self.timer = self.create_timer(0.1, self.publish_left_motor_pos_cmd)
        
        # 位置命令数据
        self.position_counter = 0
        
        self.get_logger().info('Left Motor Position Publisher Node started')

    def publish_left_motor_pos_cmd(self):
        """发布左手电机位置命令"""
        msg = MotorPos()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "left_hand_frame"
        
        # 生成示例位置命令数据 (可以根据实际需求修改)
        # 这里使用正弦波模拟动态位置命令，位置范围0-2000
        import math
        self.position_counter += 1

        # 12个电机的位置命令，范围0-2000
        msg.pos = [
            int(500 + 400 * math.sin(self.position_counter * 0.1)),
            int(800 + 300 * math.cos(self.position_counter * 0.1)),
            int(1000 + 400 * math.sin(self.position_counter * 0.15)),
            int(1200 + 300 * math.cos(self.position_counter * 0.12)),
            int(1500 + 200 * math.sin(self.position_counter * 0.08)),
            int(700 + 350 * math.cos(self.position_counter * 0.09)),
            int(900 + 250 * math.sin(self.position_counter * 0.13)),
            int(1100 + 250 * math.cos(self.position_counter * 0.11)),
            int(1300 + 300 * math.sin(self.position_counter * 0.14)),
            int(600 + 400 * math.cos(self.position_counter * 0.16)),
            int(1400 + 300 * math.sin(self.position_counter * 0.07)),
            int(1600 + 300 * math.cos(self.position_counter * 0.18))
        ]

        self.left_motor_pos_cmd_publisher.publish(msg)
        
        self.get_logger().debug(
            f'Published left motor position command: {msg.pos}'
        )

    def publish_custom_positions(self, positions):
        """发布自定义位置命令"""
        msg = MotorPos()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "left_hand_frame"
        msg.pos = positions
        
        self.left_motor_pos_cmd_publisher.publish(msg)
        self.get_logger().info(f'Published custom positions: {positions}')


def main(args=None):
    rclpy.init(args=args)
    
    left_motor_pos_publisher = LeftMotorPosPublisher()
    
    try:
        rclpy.spin(left_motor_pos_publisher)
    except KeyboardInterrupt:
        pass
    
    left_motor_pos_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
