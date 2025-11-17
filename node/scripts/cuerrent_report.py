'''
Author: huangshiheng@agibot.com
Date: 2025-11-11 16:31:13
LastEditors: huangshiheng
LastEditTime: 2025-11-11 16:34:25
FilePath: /OmniHand-Pro-2025/node/scripts/cuerrent_report.py
Description: 

Copyright (c) 2025 by huangshiheng@agibot.com, All Rights Reserved. 
'''
#!/usr/bin/env python3
"""
@Author: huangshiheng@agibot.com
@Date: 2025-11-06
@Description: Python node to subscribe hand current report feedback (1Hz)
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

from omnihand_node_msgs.msg import CurrentReport


class CurrentReportSubscriber(Node):
    def __init__(self, hand_side='left'):
        """
        初始化电流报告订阅节点
        
        Args:
            hand_side: 'left' 或 'right'，表示左手或右手
        """
        super().__init__(f'{hand_side}_current_report_subscriber')
        
        self.hand_side = hand_side
        
        # 创建电流报告反馈订阅器 (1Hz)
        self.current_report_fb_subscriber = self.create_subscription(
            CurrentReport,
            f'/agihand/omnihand_pro/{hand_side}/current_report',
            self.current_report_feedback_callback,
            10
        )

        self.get_logger().info(f'{hand_side.capitalize()} Current Report Subscriber Node started')
        self.get_logger().info(f'Subscribing to: /agihand/omnihand_pro/{hand_side}/current_report')

    def current_report_feedback_callback(self, msg):
        """处理电流报告反馈消息"""
        self.get_logger().info(
            f'Current report: [{msg.current_reports}]'
        )

def main(args=None):
    rclpy.init(args=args)
    
    # 可以通过命令行参数指定左手或右手，默认为左手
    hand_side = 'left'
    if len(sys.argv) > 1:
        hand_side = sys.argv[1]
    
    current_report_subscriber = CurrentReportSubscriber(hand_side)
    
    try:
        rclpy.spin(current_report_subscriber)
    except KeyboardInterrupt:
        current_report_subscriber.get_logger().info(
            f'Shutting down {hand_side.capitalize()} Current Report Subscriber Node'
        )
    except Exception as e:
        current_report_subscriber.get_logger().error(f'Error: {str(e)}')
    finally:
        current_report_subscriber.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
