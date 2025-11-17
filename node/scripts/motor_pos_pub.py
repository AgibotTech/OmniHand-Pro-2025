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
import time

# 动态获取项目根路径
script_dir = os.path.dirname(os.path.abspath(__file__))
project_root = os.path.dirname(os.path.dirname(script_dir))
python_packages_path = os.path.join(project_root, "build/install/local/lib/python3.10/dist-packages")

sys.path.append(python_packages_path)

from omnihand_node_msgs.msg import MotorPos


class LeftMotorPosPublisher(Node):
    def __init__(self):
        super().__init__('left_motor_pos_publisher')
        
        # 创建左手电机位置命令发布器
        self.left_motor_pos_cmd_publisher = self.create_publisher(
            MotorPos,
            '/agihand/omnihand_pro/left/motor_pos_cmd',
            10
        )
        
        # 创建定时器，每100ms发布一次命令
        self.timer = self.create_timer(4, self.publish_left_motor_pos_cmd)
        
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

        msg.pos = [0, 2081, 4094, 2029, 4094, 4094, 2048, 4094, 4000, 4094]
        self.left_motor_pos_cmd_publisher.publish(msg)
        time.sleep(0.5)

        msg.pos = [1500, 2081, 4094, 2029, 4094, 4094, 2048, 4094, 4000, 4094]
        self.left_motor_pos_cmd_publisher.publish(msg)
        time.sleep(0.5)

        msg.pos = [500, 2081, 4094, 2029, 4094, 4094, 2048, 4094, 4000, 4094]
        self.left_motor_pos_cmd_publisher.publish(msg)
        time.sleep(0.5)

        msg.pos = [1000, 2081, 4094, 2029, 4094, 4094, 2048, 4094, 4000, 4094]
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
