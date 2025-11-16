#!/usr/bin/env python3
"""
@Author: huangshiheng@agibot.com
@Date: 2025-11-06
@Description: Python node to subscribe hand tactile sensor feedback (100Hz)
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

from omnihand_node_msgs.msg import TactileSensor


class TactileSensorSubscriber(Node):
    def __init__(self, hand_side='left'):
        """
        初始化触觉传感器订阅节点
        
        Args:
            hand_side: 'left' 或 'right'，表示左手或右手
        """
        super().__init__(f'{hand_side}_tactile_sensor_subscriber')
        
        self.hand_side = hand_side

        # 创建触觉传感器订阅器 (10Hz)
        self.tactile_sensor_subscriber = self.create_subscription(
            TactileSensor,
            f'/agihand/omnihand_pro/{hand_side}/tactile_sensor',
            self.tactile_sensor_feedback_callback,
            10
        )

        self.get_logger().info(f'{hand_side.capitalize()} Tactile Sensor Subscriber Node started')
        self.get_logger().info(f'Subscribing to: /agihand/omnihand_pro/{hand_side}/tactile_sensor')

    def tactile_sensor_feedback_callback(self, msg):
        """处理触觉传感器反馈消息"""
        for i, sensor_data in enumerate(msg.tactile_datas):
            self.get_logger().info(
                f'Sensor {i} pressure exceeds sensor_data: {sensor_data}'
            )

def main(args=None):
    rclpy.init(args=args)
    
    # 可以通过命令行参数指定左手或右手，默认为左手
    hand_side = 'left'
    if len(sys.argv) > 1:
        hand_side = sys.argv[1]
    
    tactile_sensor_subscriber = TactileSensorSubscriber(hand_side)
    
    try:
        rclpy.spin(tactile_sensor_subscriber)
    except KeyboardInterrupt:
        tactile_sensor_subscriber.get_logger().info(
            f'Shutting down {hand_side.capitalize()} Tactile Sensor Subscriber Node'
        )
    except Exception as e:
        tactile_sensor_subscriber.get_logger().error(f'Error: {str(e)}')
    finally:
        tactile_sensor_subscriber.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
