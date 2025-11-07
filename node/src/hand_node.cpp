/*
 * @Author: huangshiheng@agibot.com
 * @Date: 2025-11-06 17:29:45
 * @Description: OmniHand Pro Node implementation
 * 
 * Copyright (c) 2025 by huangshiheng@agibot.com, All Rights Reserved. 
 */

#include "hand_node.h"

namespace omnihand_pro {

OmniHandProNode::OmniHandProNode() : Node("omnihand_pro_node") {
  agibot_hand_ = std::make_shared<AgibotHandO12>(1, EHandType::eLeft);

  bool is_left = true;
  std::string topic_prefix = "";
  if (is_left) {
    topic_prefix = "/agihand/omnihand/left/";
  } else {
    topic_prefix = "/agihand/omnihand/right/";
  }

  // Initialize Publishers
  control_mode_publisher_ = this->create_publisher<omnihand_pro_node_msgs::msg::ControlMode>(topic_prefix + "control_mode", 1);
  current_period_publisher_ = this->create_publisher<omnihand_pro_node_msgs::msg::CurrentPeriod>(topic_prefix + "current_period", 1);
  current_report_publisher_ = this->create_publisher<omnihand_pro_node_msgs::msg::CurrentReport>(topic_prefix + "current_report", 1);
  current_threshold_publisher_ = this->create_publisher<omnihand_pro_node_msgs::msg::CurrentThreshold>(topic_prefix + "current_threshold", 1);
  error_period_publisher_ = this->create_publisher<omnihand_pro_node_msgs::msg::ErrorPeriod>(topic_prefix + "error_period", 1);
  motor_error_report_publisher_ = this->create_publisher<omnihand_pro_node_msgs::msg::MotorErrorReport>(topic_prefix + "motor_error_report", 1);
  motor_pos_publisher_ = this->create_publisher<omnihand_pro_node_msgs::msg::MotorPos>(topic_prefix + "motor_pos", 100);
  motor_vel_publisher_ = this->create_publisher<omnihand_pro_node_msgs::msg::MotorVel>(topic_prefix + "motor_vel", 100);
  tactile_sensor_publisher_ = this->create_publisher<omnihand_pro_node_msgs::msg::TactileSensor>(topic_prefix + "tactile_sensor", 100);
  temperature_period_publisher_ = this->create_publisher<omnihand_pro_node_msgs::msg::TemperaturePeriod>(topic_prefix + "temperature_period", 1);
  temperature_report_publisher_ = this->create_publisher<omnihand_pro_node_msgs::msg::TemperatureReport>(topic_prefix + "temperature_report", 1);

  // Initialize Subscribers
  control_mode_subscriber_ = this->create_subscription<omnihand_pro_node_msgs::msg::ControlMode>(
    topic_prefix + "control_mode_cmd", 1, std::bind(&OmniHandProNode::control_mode_callback, this, std::placeholders::_1));
  current_period_subscriber_ = this->create_subscription<omnihand_pro_node_msgs::msg::CurrentPeriod>(
    topic_prefix + "current_period_cmd", 1, std::bind(&OmniHandProNode::current_period_callback, this, std::placeholders::_1));
  current_report_subscriber_ = this->create_subscription<omnihand_pro_node_msgs::msg::CurrentReport>(
    topic_prefix + "current_report_cmd", 1, std::bind(&OmniHandProNode::current_report_callback, this, std::placeholders::_1));
  current_threshold_subscriber_ = this->create_subscription<omnihand_pro_node_msgs::msg::CurrentThreshold>(
    topic_prefix + "current_threshold_cmd", 1, std::bind(&OmniHandProNode::current_threshold_callback, this, std::placeholders::_1));
  error_period_subscriber_ = this->create_subscription<omnihand_pro_node_msgs::msg::ErrorPeriod>(
    topic_prefix + "error_period_cmd", 1, std::bind(&OmniHandProNode::error_period_callback, this, std::placeholders::_1));
  motor_error_report_subscriber_ = this->create_subscription<omnihand_pro_node_msgs::msg::MotorErrorReport>(
    topic_prefix + "motor_error_report_cmd", 1, std::bind(&OmniHandProNode::motor_error_report_callback, this, std::placeholders::_1));
  motor_pos_subscriber_ = this->create_subscription<omnihand_pro_node_msgs::msg::MotorPos>(
    topic_prefix + "motor_pos_cmd", 100, std::bind(&OmniHandProNode::motor_pos_callback, this, std::placeholders::_1));
  motor_vel_subscriber_ = this->create_subscription<omnihand_pro_node_msgs::msg::MotorVel>(
    topic_prefix + "motor_vel_cmd", 100, std::bind(&OmniHandProNode::motor_vel_callback, this, std::placeholders::_1));
  tactile_sensor_subscriber_ = this->create_subscription<omnihand_pro_node_msgs::msg::TactileSensor>(
    topic_prefix + "tactile_sensor_cmd", 100, std::bind(&OmniHandProNode::tactile_sensor_callback, this, std::placeholders::_1));
  temperature_period_subscriber_ = this->create_subscription<omnihand_pro_node_msgs::msg::TemperaturePeriod>(
    topic_prefix + "temperature_period_cmd", 1, std::bind(&OmniHandProNode::temperature_period_callback, this, std::placeholders::_1));
  temperature_report_subscriber_ = this->create_subscription<omnihand_pro_node_msgs::msg::TemperatureReport>(
    topic_prefix + "temperature_report_cmd", 1, std::bind(&OmniHandProNode::temperature_report_callback, this, std::placeholders::_1));

  // Initialize timers for different frequencies
  timer_1hz_ = this->create_wall_timer(
    std::chrono::milliseconds(1000), // 1Hz = 1000ms
    std::bind(&OmniHandProNode::timer_1hz_callback, this));

  timer_100hz_ = this->create_wall_timer(
    std::chrono::milliseconds(10), // 100Hz = 10ms
    std::bind(&OmniHandProNode::timer_100hz_callback, this));

  RCLCPP_INFO(this->get_logger(), "OmniHand Pro Node initialized");
}

OmniHandProNode::~OmniHandProNode() {
  RCLCPP_INFO(this->get_logger(), "OmniHand Pro Node destroyed");
}

// Callback implementations
void OmniHandProNode::control_mode_callback(const omnihand_pro_node_msgs::msg::ControlMode::SharedPtr msg) {
  RCLCPP_INFO(this->get_logger(), "Received control mode command");
}

void OmniHandProNode::current_period_callback(const omnihand_pro_node_msgs::msg::CurrentPeriod::SharedPtr msg) {
  RCLCPP_INFO(this->get_logger(), "Received current period command");
}

void OmniHandProNode::current_report_callback(const omnihand_pro_node_msgs::msg::CurrentReport::SharedPtr msg) {
  RCLCPP_INFO(this->get_logger(), "Received current report command");
}

void OmniHandProNode::current_threshold_callback(const omnihand_pro_node_msgs::msg::CurrentThreshold::SharedPtr msg) {
  RCLCPP_INFO(this->get_logger(), "Received current threshold command");
}

void OmniHandProNode::error_period_callback(const omnihand_pro_node_msgs::msg::ErrorPeriod::SharedPtr msg) {
  RCLCPP_INFO(this->get_logger(), "Received error period command");
}

void OmniHandProNode::motor_error_report_callback(const omnihand_pro_node_msgs::msg::MotorErrorReport::SharedPtr msg) {
  RCLCPP_INFO(this->get_logger(), "Received motor error report command");
}

void OmniHandProNode::motor_pos_callback(const omnihand_pro_node_msgs::msg::MotorPos::SharedPtr msg) {
  RCLCPP_INFO(this->get_logger(), "Received motor position command with %zu positions", msg->pos.size());
}

void OmniHandProNode::motor_vel_callback(const omnihand_pro_node_msgs::msg::MotorVel::SharedPtr msg) {
  RCLCPP_INFO(this->get_logger(), "Received motor velocity command");
}

void OmniHandProNode::tactile_sensor_callback(const omnihand_pro_node_msgs::msg::TactileSensor::SharedPtr msg) {
  RCLCPP_INFO(this->get_logger(), "Received tactile sensor command");
}

void OmniHandProNode::temperature_period_callback(const omnihand_pro_node_msgs::msg::TemperaturePeriod::SharedPtr msg) {
  RCLCPP_INFO(this->get_logger(), "Received temperature period command");
}

void OmniHandProNode::temperature_report_callback(const omnihand_pro_node_msgs::msg::TemperatureReport::SharedPtr msg) {
  RCLCPP_INFO(this->get_logger(), "Received temperature report command");
}

// Timer callback implementations
void OmniHandProNode::timer_1hz_callback() {
  // 发布所有1Hz消息
  publish_control_mode();
  publish_current_period();
  publish_current_report();
  publish_current_threshold();
  publish_error_period();
  publish_motor_error_report();
  publish_temperature_period();
  publish_temperature_report();
}

void OmniHandProNode::timer_100hz_callback() {
  // 发布所有100Hz消息
  publish_motor_pos();
  publish_motor_vel();
  publish_tactile_sensor();
}

// Publisher implementations
void OmniHandProNode::publish_control_mode() {
  auto msg = omnihand_pro_node_msgs::msg::ControlMode();
  msg.header.stamp = this->now();
  msg.header.frame_id = "control_frame";
  control_mode_publisher_->publish(msg);
}

void OmniHandProNode::publish_current_period() {
  auto msg = omnihand_pro_node_msgs::msg::CurrentPeriod();
  msg.header.stamp = this->now();
  msg.header.frame_id = "current_frame";
  current_period_publisher_->publish(msg);
}

void OmniHandProNode::publish_current_report() {
  auto msg = omnihand_pro_node_msgs::msg::CurrentReport();
  msg.header.stamp = this->now();
  msg.header.frame_id = "current_frame";
  current_report_publisher_->publish(msg);
}

void OmniHandProNode::publish_current_threshold() {
  auto msg = omnihand_pro_node_msgs::msg::CurrentThreshold();
  msg.header.stamp = this->now();
  msg.header.frame_id = "current_frame";
  current_threshold_publisher_->publish(msg);
}

void OmniHandProNode::publish_error_period() {
  auto msg = omnihand_pro_node_msgs::msg::ErrorPeriod();
  msg.header.stamp = this->now();
  msg.header.frame_id = "error_frame";
  error_period_publisher_->publish(msg);
}

void OmniHandProNode::publish_motor_error_report() {
  auto msg = omnihand_pro_node_msgs::msg::MotorErrorReport();
  msg.header.stamp = this->now();
  msg.header.frame_id = "motor_frame";
  motor_error_report_publisher_->publish(msg);
}

void OmniHandProNode::publish_motor_pos() {
  auto msg = omnihand_pro_node_msgs::msg::MotorPos();
  msg.header.stamp = this->now();
  msg.header.frame_id = "motor_frame";
  msg.pos = {100, 200, 300, 400, 500};
  motor_pos_publisher_->publish(msg);
}

void OmniHandProNode::publish_motor_vel() {
  auto msg = omnihand_pro_node_msgs::msg::MotorVel();
  msg.header.stamp = this->now();
  msg.header.frame_id = "motor_frame";
  motor_vel_publisher_->publish(msg);
}

void OmniHandProNode::publish_tactile_sensor() {
  auto msg = omnihand_pro_node_msgs::msg::TactileSensor();
  msg.header.stamp = this->now();
  msg.header.frame_id = "tactile_frame";
  tactile_sensor_publisher_->publish(msg);
}

void OmniHandProNode::publish_temperature_period() {
  auto msg = omnihand_pro_node_msgs::msg::TemperaturePeriod();
  msg.header.stamp = this->now();
  msg.header.frame_id = "temperature_frame";
  temperature_period_publisher_->publish(msg);
}

void OmniHandProNode::publish_temperature_report() {
  auto msg = omnihand_pro_node_msgs::msg::TemperatureReport();
  msg.header.stamp = this->now();
  msg.header.frame_id = "temperature_frame";
  temperature_report_publisher_->publish(msg);
}

} // namespace omnihand_pro