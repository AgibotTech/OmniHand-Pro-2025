/*
 * @Author: huangshiheng@agibot.com
 * @Date: 2025-11-06 14:15:12
 * @LastEditors: error: git config user.name & please set dead value or install git
 * @LastEditTime: 2025-11-07 16:21:47
 * @FilePath: /OmniHand-Pro-2025/node/src/main.cpp
 * @Description: 
 * 
 * Copyright (c) 2025 by huangshiheng@agibot.com, All Rights Reserved. 
 */

#include "rclcpp/rclcpp.hpp"
#include "hand_node.h"

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  // 创建多个节点
  auto left_hand_node = std::make_shared<omnihand_pro::OmniHandProNode>(1, 0, EHandType::eLeft);
  auto right_hand_node = std::make_shared<omnihand_pro::OmniHandProNode>(2, 1, EHandType::eRight);

  // 创建多线程执行器
  rclcpp::executors::MultiThreadedExecutor executor;

  // 将节点添加到执行器
  executor.add_node(left_hand_node);
  executor.add_node(right_hand_node);

  // 同时运行所有节点
  executor.spin();

  rclcpp::shutdown();
}