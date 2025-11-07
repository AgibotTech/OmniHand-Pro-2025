/*
 * @Author: huangshiheng@agibot.com
 * @Date: 2025-11-06 14:15:12
 * @LastEditors: error: git config user.name & please set dead value or install git
 * @LastEditTime: 2025-11-06 19:51:31
 * @FilePath: /OmniHand-Pro-2025/node/src/main.cpp
 * @Description: 
 * 
 * Copyright (c) 2025 by huangshiheng@agibot.com, All Rights Reserved. 
 */

#include "rclcpp/rclcpp.hpp"
#include "hand_node.h"

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<omnihand_pro::OmniHandProNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}