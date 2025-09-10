// Copyright (c) 2025, Agibot Co., Ltd.
// OmniHand Pro 2025 SDK is licensed under Mulan PSL v2.

#include <chrono>
#include <iostream>
#include <thread>
#include "c_agibot_hand.h"

void positionControlDemo() {
  try {
    AgibotHandO12 hand(1, EHandType::eLeft);

    // 设置单个关节位置
    hand.SetJointMotorPosi(8, 1000);
    std::cout << "设置关节10位置为1000" << std::endl;
    std::this_thread::sleep_for(std::chrono::seconds(1));

    // 获取单个关节位置
    short pos = hand.GetJointMotorPosi(8);
    std::cout << "关节8当前位置: " << pos << std::endl;
    std::this_thread::sleep_for(std::chrono::seconds(2));

    // 批量设置所有关节位置
    std::vector<short> positions = {2000, 2000, 2000, 2000, 2000, 2000, 2000, 2000, 2000, 2000, 2000, 0};
    hand.SetAllJointMotorPosi(positions);
    std::cout << "批量设置所有关节位置完成" << std::endl;
    std::this_thread::sleep_for(std::chrono::seconds(1));

    // 批量获取所有关节位置
    std::vector<short> allPositions = hand.GetAllJointMotorPosi();
    std::cout << "所有关节位置: ";
    for (size_t i = 0; i < allPositions.size(); ++i) {
      std::cout << allPositions[i];
      if (i < allPositions.size() - 1) std::cout << ", ";
    }
    std::cout << std::endl;

  } catch (const std::exception& e) {
    std::cerr << "位置控制错误: " << e.what() << std::endl;
  }
}

int main() {
  std::cout << "OmniHand Pro 2025 C++ SDK 功能演示" << std::endl;

  positionControlDemo();

  std::cout << "演示完成!" << std::endl;

  return 0;
}