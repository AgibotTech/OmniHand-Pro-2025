// Copyright (c) 2025, Agibot Co., Ltd.
// OmniHand Pro 2025 SDK is licensed under Mulan PSL v2.

/**
 * @file c_agibot_hand.h
 * @brief O12灵巧手类
 * @author WSJ
 * @date 25-8-1
 **/

#ifndef C_AGIBOT_HAND_H
#define C_AGIBOT_HAND_H

#include <queue>

#include "can_bus_device/c_can_bus_device.h"
#include "kinematics_solver/kinematics_solver.h"
#include "proto.h"

#include "export_symbols.h"

#define DEFAULT_DEVICE_ID 0x01

/**
 * @brief O12灵巧手类
 */
class AGIBOT_EXPORT AgibotHandO12 {
 public:
  /**
   * @brief 构造函数
   * @param device_id 设备Id
   * @param hand_type 手型(左手/右手)
   */
  explicit AgibotHandO12(unsigned char device_id, EHandType hand_type);

  ~AgibotHandO12();

  /**
   * @brief 获取厂家信息
   * @return 厂家信息数据结构体
   */
  VendorInfo GetVendorInfo();

  /**
   * @brief 获取设备信息
   * @return 设备信息结构体
   */
  DeviceInfo GetDeviceInfo();

  /**
   * @brief 设置设备Id
   * @param device_id 设备Id
   */
  void SetDeviceId(unsigned char device_id);

  /**
   * @brief 设置单个关节电机位置
   * @param joint_motor_index 关节电机索引
   * @param posi 电机位置:(0~2000)
   */
  void SetJointMotorPosi(unsigned char joint_motor_index, short posi);

  /**
   * @brief 获取单个关节电机位置
   * @param joint_motor_index
   * @return
   */
  short GetJointMotorPosi(unsigned char joint_motor_index);

  /**
   * @brief 批量设置所有关节电机位置
   * @note 注意要提供完整的12个关节电机的位置数据
   * @param vec_posi
   */
  void SetAllJointMotorPosi(std::vector<short> vec_posi);

  /**
   * @brief 批量获取所有关节电机位置
   * @return
   */
  std::vector<short> GetAllJointMotorPosi();

  /**
   * @brief 设置单个关节的关节角
   * @param joint_motor_index 关节电机索引
   * @param angle 关节角度(单位：弧度)
   */
  void SetJointAngle(unsigned char joint_motor_index, double angle);

  /**
   * @brief 获取单个关节的关节角
   * @param joint_motor_index
   * @return
   */
  double GetJointAngle(unsigned char joint_motor_index);

  /**
   * @brief 批量设置所有关节的关节角
   * @note 注意要提供完整的12个关节电机的关节角数据
   * @param vec_angle
   */
  void SetAllJointAngles(std::vector<double> vec_angle);

  /**
   * @brief 批量获取所有关节的关节角
   * @return
   */
  std::vector<double> GetAllJointAngles();

  /**
   * @brief 设置单个关节电机力矩
   * @param joint_motor_index
   * @param torque
   */
  void SetJointMotorTorque(unsigned char joint_motor_index, short torque);

  /**
   * @brief 获取单个关节电机力矩
   * @param joint_motor_index
   * @return
   */
  short GetJointMotorTorque(unsigned char joint_motor_index);

  /**
   * @brief 批量设置所有关节电机力矩
   * @param vec_torque
   */
  void SetAllJointMotorTorque(std::vector<short> vec_torque);

  /**
   * @brief 批量获取所有关节电机力矩
   * @return
   */
  std::vector<short> GetAllJointMotorTorque();

  /**
   * @brief 设置单个关节电机速度
   * @param joint_motor_index
   * @param velo
   */
  void SetJointMotorVelo(unsigned char joint_motor_index, short velo);

  /**
   * @brief 获取单个关节电机速度
   * @param joint_motor_index
   * @return
   */
  short GetJointMotorVelo(unsigned char joint_motor_index);

  /**
   * @brief 批量设置所有关节电机速度
   * @param vec_velo
   */
  void SetAllJointMotorVelo(std::vector<short> vec_velo);

  /**
   * @brief
   * @return 批量获取所有关节电机速度
   */
  std::vector<short> GetAllJointMotorVelo();

  /**
   * @brief 获取指定手指的触觉传感器数据
   * @param eFinger
   * @return
   */
  TouchSensorData GetTouchSensorData(EFinger eFinger);

  /**
   * @brief 设置单个关节电机控制模式
   * @param joint_motor_index
   * @param mode
   */
  void SetControlMode(unsigned char joint_motor_index, EControlMode mode);

  /**
   * @brief 获取单个关节电机控制模式
   * @param joint_motor_index
   * @return
   */
  EControlMode GetControlMode(unsigned char joint_motor_index);

  /**
   * @brief 批量设置所有关节电机控制模式
   * @param vec_ctrl_mode
   */
  void SetAllControlMode(std::vector<unsigned char> vec_ctrl_mode);

  /**
   * @brief 批量获取所有关节电机控制模式
   * @return
   */
  std::vector<unsigned char> GetAllControlMode();

  /**
   * @brief 设置单个关节电机电流阈值
   * @param joint_motor_index
   * @param current_threshold
   */
  void SetCurrentThreshold(unsigned char joint_motor_index, short current_threshold);

  /**
   * @brief 获取单个关节电机电流阈值
   * @param joint_motor_index
   * @return
   */
  short GetCurrentThreshold(unsigned char joint_motor_index);

  /**
   * @brief 批量设置所有关节电机电流阈值
   * @param vec_current_threshold
   */
  void SetAllCurrentThreshold(std::vector<short> vec_current_threshold);

  /**
   * @brief 批量获取所有关节电机电流阈值
   * @return
   */
  std::vector<short> GetAllCurrentThreshold();

  /**
   * @brief 混合控制
   * @param vec_mix_ctrl
   */
  void MixCtrlJointMotor(std::vector<MixCtrl> vec_mix_ctrl);

  /**
   * @brief 获取单个关节电机错误上报
   * @param joint_motor_index
   * @return
   */
  JointMotorErrorReport GetErrorReport(unsigned char joint_motor_index);

  /**
   * @brief 获取所有关节电机错误上报
   * @return
   */
  std::vector<JointMotorErrorReport> GetAllErrorReport();

  /**
   * @brief 设置单个关节电机错误上报周期
   * @param joint_motor_index
   * @param period
   */
  void SetErrorReportPeriod(unsigned char joint_motor_index, unsigned short period);

  /**
   * @brief 批量设置所有关节电机错误上报周期
   * @param vec_period
   */
  void SetAllErrorReportPeriod(std::vector<unsigned short> vec_period);

  /**
   * @brief 获取单个关节电机温度上报
   * @param joint_motor_index
   * @return
   */
  unsigned short GetTemperatureReport(unsigned char joint_motor_index);

  /**
   * @brief 批量获取所有关节电机温度上报
   * @return
   */
  std::vector<unsigned short> GetAllTemperatureReport();

  /**
   * @brief 设置单个关节电机温度上报周期
   * @param joint_motor_index
   * @param period
   */
  void SetTemperReportPeriod(unsigned char joint_motor_index, unsigned short period);

  /**
   * @brief 设置所有关节电机温度上报周期
   * @param vec_period
   */
  void SetAllTemperReportPeriod(std::vector<unsigned short> vec_period);

  /**
   * @brief 获取单个关节电机电流上报
   * @param joint_motor_index
   * @return
   */
  short GetCurrentReport(unsigned char joint_motor_index);

  /**
   * @brief 批量获取所有关节电机电流上报
   * @return
   */
  std::vector<unsigned short> GetAllCurrentReport();

  /**
   * @brief 设置单个关节电机电流上报周期
   * @param joint_motor_index
   * @param period
   */
  void SetCurrentReportPeriod(unsigned char joint_motor_index, unsigned short period);

  /**
   * @brief 批量设置所有关节电机电流上报周期
   * @param vec_period
   */
  void SetAllCurrentReportPeriod(std::vector<unsigned short> vec_period);

  /**
   * @brief 显示发送接收数据细节
   * @param show
   */
  void ShowDataDetails(bool show) const;

 private:
  /**
   * @brief 接收处理回调函数
   * @param frame 帧数据
   */
  void ProcessMsg(CanfdFrame frame);

  /**
   * @brief 判断请求应答Id是否匹配
   * @param req_id 请求Id
   * @param rep_id 应答Id
   * @return true~匹配;false~不匹配
   */
  bool JudgeMsgMatch(unsigned int req_id, unsigned int rep_id);

  /**
   * @brief 获得请求Id的匹配应答Id
   * @details 一一对应
   * @param req_id 请求Id
   * @return 对应的应答Id
   */
  unsigned int GetMatchedRepId(unsigned int req_id);

  /**
   * @brief 更新固件
   * @details TODO 测试
   * @param file_name 固件文件名
   */
  void UpdateFirmware(std::string file_name);

  /**
   * @brief 发送升级固件分包
   * @details TODO 测试
   */
  void SendPackage();

  /**
   * @brief 获取升级结果
   * @details TODO 测试
   */
  void GetUpgradeResult();

  /**
   * @brief CANFD设备
   */
  std::unique_ptr<CanBusDeviceBase> canfd_device_;

  /**
   * @brief 温度上报数据
   */
  std::mutex mutex_temper_report_;
  std::vector<unsigned short> vec_temper_report_{};

  /**
   * @brief 电流上报数据
   */
  std::mutex mutex_current_report_;
  std::vector<unsigned short> vec_current_report_{};

  /**
   * @brief 升级固件分包
   */
  std::queue<std::array<char, 2048> > que_packages_;

  /**
   * @brief 设备Id
   */
  unsigned char device_id_{};

  /**
   * @brief 是否为左手
   */
  bool is_left_hand_{true};

  /**
   * @brief O12运动学求解器
   */
  std::unique_ptr<omnihandProSDK::O12KinematicsSolver> kinematics_solver_ptr_;
};

#endif  // C_AGIBOT_HAND_H