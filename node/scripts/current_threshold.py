#!/usr/bin/env python3
"""
@Author: huangshiheng@agibot.com
@Date: 2025-11-06
@Description: Python node to control and monitor current threshold (1Hz)
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

from omnihand_node_msgs.msg import CurrentThreshold


class CurrentThresholdNode(Node):
    def __init__(self, hand_side='left'):
        """
        初始化电流阈值节点
        
        Args:
            hand_side: 'left' 或 'right'，表示左手或右手
        """
        super().__init__(f'{hand_side}_current_threshold_node')

        self.hand_side = hand_side

        # 创建电流阈值反馈发布器 (1Hz)
        self.current_threshold_fb_publisher = self.create_publisher(
            CurrentThreshold,
            f'/agihand/omnihand_pro/{hand_side}/current_threshold_cmd',
            10
        )

        # 创建电流阈值命令订阅器
        self.current_threshold_subscriber = self.create_subscription(
            CurrentThreshold,
            f'/agihand/omnihand_pro/{hand_side}/current_threshold',
            self.current_threshold_callback,
            10
        )

        # 创建1Hz定时器用于发布电流阈值反馈
        # self.timer_1hz = self.create_timer(1.0, self.timer_1hz_callback)

        # 存储当前电流阈值
        self.current_threshold = CurrentThreshold()

        self.get_logger().info(f'{hand_side.capitalize()} Current Threshold Node started')
        self.get_logger().info(f'Publishing to: /agihand/omnihand/{hand_side}/current_threshold')
        self.get_logger().info(f'Subscribing to: /agihand/omnihand/{hand_side}/current_threshold')

    def current_threshold_callback(self, msg):
        """处理电流阈值命令消息"""
        
        threshold_str = ', '.join([f'{t:.2f}' for t in msg.current_thresholds])
        self.get_logger().info(
            f'Received current threshold command for {self.hand_side} hand: [{threshold_str}]'
        )

    def timer_1hz_callback(self):
        """1Hz定时器回调，定期发布电流阈值反馈"""
        self.publish_current_threshold_feedback()

    def publish_current_threshold_feedback(self):
        """发布电流阈值反馈"""
        msg = CurrentThreshold()
        msg.current_thresholds = self.current_threshold.threshold.copy()
        
        self.current_threshold_fb_publisher.publish(msg)
        
        if self.get_logger().get_effective_level() <= rclpy.logging.LoggingSeverity.DEBUG:
            threshold_str = ', '.join([f'{t:.2f}' for t in msg.current_thresholds])
            self.get_logger().debug(
                f'Published current threshold feedback for {self.hand_side} hand: [{threshold_str}] mA'
            )


def main(args=None):
    rclpy.init(args=args)
    
    # 可以通过命令行参数指定左手或右手，默认为左手
    hand_side = 'left'
    if len(sys.argv) > 1:
        hand_side = sys.argv[1]
    
    current_threshold_node = CurrentThresholdNode(hand_side)
    
    try:
        rclpy.spin(current_threshold_node)
    except KeyboardInterrupt:
        current_threshold_node.get_logger().info(
            f'Shutting down {hand_side.capitalize()} Current Threshold Node'
        )
    except Exception as e:
        current_threshold_node.get_logger().error(f'Error: {str(e)}')
    finally:
        current_threshold_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
