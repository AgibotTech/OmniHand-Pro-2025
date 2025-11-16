#!/usr/bin/env python3
"""
@Author: huangshiheng@agibot.com
@Date: 2025-11-06
@Description: Python node to control and monitor control mode
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

from omnihand_node_msgs.msg import ControlMode


class ControlModeNode(Node):
    def __init__(self, hand_side='left'):
        """
        初始化控制模式节点
        
        Args:
            hand_side: 'left' 或 'right'，表示左手或右手
        """
        super().__init__(f'{hand_side}_control_mode_node')
        
        self.hand_side = hand_side
        
        # 创建控制模式反馈发布器 (1Hz)
        self.control_mode_fb_publisher = self.create_publisher(
            ControlMode,
            f'/agihand/omnihand_pro/{hand_side}/control_mode_fb',
            10
        )
        
        # 创建控制模式命令订阅器
        self.control_mode_subscriber = self.create_subscription(
            ControlMode,
            f'/agihand/omnihand_pro/{hand_side}/control_mode',
            self.control_mode_callback,
            10
        )

        # 创建1Hz定时器用于发布控制模式反馈
        # self.timer_1hz = self.create_timer(1.0, self.timer_1hz_callback)
        
        # 存储当前控制模式
        self.current_control_mode = ControlMode()
        
        self.get_logger().info(f'{hand_side.capitalize()} Control Mode Node started')

    def control_mode_callback(self, msg):
        """处理控制模式命令消息"""
        self.get_logger().info(
            f'Received control mode command for {self.hand_side} hand: mode={msg.modes}'
        )
        
        # 更新当前控制模式
        self.current_control_mode = msg
        
        # 这里可以添加控制模式切换逻辑
        # 例如：调用底层驱动API切换控制模式

    def timer_1hz_callback(self):
        """1Hz定时器回调，定期发布控制模式反馈"""
        self.publish_control_mode_feedback()

    def publish_control_mode_feedback(self):
        """发布控制模式反馈"""
        msg = ControlMode()
        msg.mode = self.current_control_mode.mode

        self.control_mode_fb_publisher.publish(msg)
        self.get_logger().debug(
            f'Published control mode feedback for {self.hand_side} hand: mode={msg.mode}'
        )

def main(args=None):
    rclpy.init(args=args)
    
    # 可以通过命令行参数指定左手或右手，默认为左手
    hand_side = 'left'
    if len(sys.argv) > 1:
        hand_side = sys.argv[1]
    
    control_mode_node = ControlModeNode(hand_side)
    
    try:
        rclpy.spin(control_mode_node)
    except KeyboardInterrupt:
        pass
    
    control_mode_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
