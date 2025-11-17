/*
 * @Author: huangshiheng@agibot.com
 * @Date: 2025-11-06 17:29:45
 * @LastEditors: huangshiheng
 * @LastEditTime: 2025-11-11 17:06:28
 * @FilePath: /OmniHand-Pro-2025/node/src/hand_node.h
 * @Description: 
 * 
 * Copyright (c) 2025 by huangshiheng@agibot.com, All Rights Reserved. 
 */

#pragma once

#include "rclcpp/rclcpp.hpp"
#include "omnihand_pro_node_msgs/msg/control_mode.hpp"
#include "omnihand_pro_node_msgs/msg/current_period.hpp"
#include "omnihand_pro_node_msgs/msg/current_report.hpp"
#include "omnihand_pro_node_msgs/msg/current_threshold.hpp"
#include "omnihand_pro_node_msgs/msg/error_period.hpp"
#include "omnihand_pro_node_msgs/msg/motor_angle.hpp"
#include "omnihand_pro_node_msgs/msg/motor_error_report.hpp"
#include "omnihand_pro_node_msgs/msg/motor_pos.hpp"
#include "omnihand_pro_node_msgs/msg/motor_vel.hpp"
#include "omnihand_pro_node_msgs/msg/tactile_sensor.hpp"
#include "omnihand_pro_node_msgs/msg/tactile_sensor_data.hpp"
#include "omnihand_pro_node_msgs/msg/temperature_period.hpp"
#include "omnihand_pro_node_msgs/msg/temperature_report.hpp"


#include "c_agibot_hand.h"

namespace omnihand_pro {

class OmniHandProNode : public rclcpp::Node {
 public:
  OmniHandProNode(uint8_t device_id, uint8_t canfd_id, EHandType hand_type);
  ~OmniHandProNode();

 private:
  // Publishers
  rclcpp::Publisher<omnihand_pro_node_msgs::msg::ControlMode>::SharedPtr control_mode_publisher_;             // 1HZ

  rclcpp::Publisher<omnihand_pro_node_msgs::msg::CurrentReport>::SharedPtr current_report_publisher_;         // 1HZ
  rclcpp::Publisher<omnihand_pro_node_msgs::msg::CurrentThreshold>::SharedPtr current_threshold_publisher_;   // 1HZ
  rclcpp::Publisher<omnihand_pro_node_msgs::msg::MotorErrorReport>::SharedPtr motor_error_report_publisher_;  // 1HZ
  rclcpp::Publisher<omnihand_pro_node_msgs::msg::MotorPos>::SharedPtr motor_pos_publisher_;                   // 10HZ
  rclcpp::Publisher<omnihand_pro_node_msgs::msg::MotorVel>::SharedPtr motor_vel_publisher_;                   // 10HZ
  rclcpp::Publisher<omnihand_pro_node_msgs::msg::TactileSensor>::SharedPtr tactile_sensor_publisher_;         // 10HZ
  rclcpp::Publisher<omnihand_pro_node_msgs::msg::TemperatureReport>::SharedPtr temperature_report_publisher_; // 1HZ
  rclcpp::Publisher<omnihand_pro_node_msgs::msg::MotorAngle>::SharedPtr motor_angle_publisher_;

  // Subscribers
  rclcpp::Subscription<omnihand_pro_node_msgs::msg::ControlMode>::SharedPtr control_mode_subscriber_;
  rclcpp::Subscription<omnihand_pro_node_msgs::msg::CurrentThreshold>::SharedPtr current_threshold_subscriber_;
  rclcpp::Subscription<omnihand_pro_node_msgs::msg::MotorErrorReport>::SharedPtr motor_error_report_subscriber_;
  rclcpp::Subscription<omnihand_pro_node_msgs::msg::MotorPos>::SharedPtr motor_pos_subscriber_;
  rclcpp::Subscription<omnihand_pro_node_msgs::msg::MotorVel>::SharedPtr motor_vel_subscriber_;
  rclcpp::Subscription<omnihand_pro_node_msgs::msg::MotorAngle>::SharedPtr motor_angle_subscriber_;

  // Callback functions
  void control_mode_callback(const omnihand_pro_node_msgs::msg::ControlMode::SharedPtr msg);
  void current_report_callback(const omnihand_pro_node_msgs::msg::CurrentReport::SharedPtr msg);
  void current_threshold_callback(const omnihand_pro_node_msgs::msg::CurrentThreshold::SharedPtr msg);
  void motor_pos_callback(const omnihand_pro_node_msgs::msg::MotorPos::SharedPtr msg);
  void motor_vel_callback(const omnihand_pro_node_msgs::msg::MotorVel::SharedPtr msg);
  void motor_angle_callback(const omnihand_pro_node_msgs::msg::MotorAngle::SharedPtr msg);


  // Publisher functions
  void publish_control_mode();
  void publish_current_report();
  void publish_current_threshold();
  void publish_motor_error_report();
  void publish_motor_pos();
  void publish_motor_vel();
  void publish_tactile_sensor();
  void publish_temperature_report();
  void publish_motor_angle();

  // Timer callback functions
  void timer_1hz_callback();   // 1Hz消息发布
  void timer_10hz_callback(); // 10Hz消息发布

  // Timers for different frequencies
  rclcpp::TimerBase::SharedPtr timer_1hz_;   // 1Hz timer
  rclcpp::TimerBase::SharedPtr timer_10hz_; // 10Hz timer

  std::shared_ptr<AgibotHandO12> agibot_hand_;

  std::mutex mutex_;
};

}  // namespace omnihand_pro