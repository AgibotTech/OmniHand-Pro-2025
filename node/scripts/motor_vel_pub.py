#!/usr/bin/env python3
"""
@Author: huangshiheng@agibot.com
@Date: 2025-11-06
@Description: Python node to publish left hand motor velocity commands
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


class LeftMotorVelPublisher(Node):
    def __init__(self):
        super().__init__('left_motor_vel_publisher')
        
        # 创建左手电机速度命令发布器
        self.left_motor_vel_cmd_publisher = self.create_publisher(
            MotorVel,
            '/agihand/omnihand_pro/left/motor_vel_cmd',
            100
        )
        # 创建定时器，每100ms发布一次命令
        self.timer = self.create_timer(1, self.publish_left_motor_vel_cmd)
        
        # 速度命令数据
        self.velocity_counter = 0
        
        self.get_logger().info('Left Motor Velocity Publisher Node started')

    def publish_left_motor_vel_cmd(self):
        """发布左手电机速度命令"""
        msg = MotorVel()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "left_hand_frame"
        
        # 生成示例速度命令数据 (可以根据实际需求修改)
        import math
        self.velocity_counter += 1

        msg.vels = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0]

        self.left_motor_vel_cmd_publisher.publish(msg)
        
        self.get_logger().debug(
            f'Published left motor velocity command: {msg.vels}'
        )


def main(args=None):
    rclpy.init(args=args)
    
    left_motor_vel_publisher = LeftMotorVelPublisher()
    
    try:
        rclpy.spin(left_motor_vel_publisher)
    except KeyboardInterrupt:
        pass
    
    left_motor_vel_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
