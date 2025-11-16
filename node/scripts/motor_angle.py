'''
Author: huangshiheng@agibot.com
Date: 2025-11-11 16:31:13
LastEditors: huangshiheng
LastEditTime: 2025-11-11 16:51:45
FilePath: /OmniHand-Pro-2025/node/scripts/motor_angle.py
Description: 

Copyright (c) 2025 by huangshiheng@agibot.com, All Rights Reserved. 
'''
#!/usr/bin/env python3
"""
@Author: huangshiheng@agibot.com
@Date: 2025-11-06
@Description: Python node to control and monitor motor angle (100Hz)
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

from omnihand_node_msgs.msg import MotorAngle


class MotorAngleNode(Node):
    def __init__(self, hand_side='left'):
        """
        初始化电机角度节点
        
        Args:
            hand_side: 'left' 或 'right'，表示左手或右手
        """
        super().__init__(f'{hand_side}_motor_angle_node')
        
        self.hand_side = hand_side
        
        # 创建电机角度反馈发布器 (100Hz)
        self.motor_angle_fb_publisher = self.create_publisher(
            MotorAngle,
            f'/agihand/omnihand_pro/{hand_side}/motor_angle_cmd',
            10
        )
        
        # 创建电机角度命令订阅器
        self.motor_angle_subscriber = self.create_subscription(
            MotorAngle,
            f'/agihand/omnihand_pro/{hand_side}/motor_angle',
            self.motor_angle_callback,
            100
        )
        
        # 创建100Hz定时器用于发布电机角度反馈
        # self.timer_100hz = self.create_timer(0.01, self.timer_10hz_callback)

        # 存储当前电机角度
        self.current_motor_angle = MotorAngle()

        # 上一秒的消息计数
        self.last_fb_count = 0
        self.last_cmd_count = 0

        self.get_logger().info(f'{hand_side.capitalize()} Motor Angle Node started')
        self.get_logger().info(f'Publishing to: /agihand/omnihand_pro/{hand_side}/motor_angle_cmd')
        self.get_logger().info(f'Subscribing to: /agihand/omnihand_pro/{hand_side}/motor_angle')

    def motor_angle_callback(self, msg):
        """处理电机角度命令消息"""
        angles_str = ', '.join([f'{t}' for t in msg.angles])
        self.get_logger().info(
            f'Receive {self.hand_side} hand: [{angles_str}]'
        )

    def timer_10hz_callback(self):
        """100Hz定时器回调，定期发布电机角度反馈"""
        self.publish_motor_angle_feedback()

    def publish_motor_angle_feedback(self):
        """发布电机角度反馈"""
        msg = MotorAngle()

        # self.motor_angle_fb_publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    
    # 可以通过命令行参数指定左手或右手，默认为左手
    hand_side = 'left'
    if len(sys.argv) > 1:
        hand_side = sys.argv[1]
    
    motor_angle_node = MotorAngleNode(hand_side)
    
    try:
        rclpy.spin(motor_angle_node)
    except KeyboardInterrupt:
        motor_angle_node.get_logger().info(
            f'Shutting down {hand_side.capitalize()} Motor Angle Node'
        )
    except Exception as e:
        motor_angle_node.get_logger().error(f'Error: {str(e)}')
    finally:
        motor_angle_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
