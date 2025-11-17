// Copyright (c) 2025, Agibot Co., Ltd.
// OmniHand Pro 2025 SDK is licensed under Mulan PSL v2.

/**
 * @file c_agibot_hand.cpp
 * @brief
 * @author WSJ
 * @date 25-8-1
 **/

#include "c_agibot_hand.h"

#include <cstring>
#include <fstream>
#include <iostream>

#ifdef ZLG_USBCANFD_SDK
  #include "can_bus_device/zlg_usb_canfd/c_zlg_usbcanfd_sdk.h"
#endif

#ifdef SOCKET_CAN
  #include "can_bus_device/socket_can/c_can_bus_device_socket_can.h"
#endif

#define CANID_WRITE_FLAG 0x01
#define CANID_READ_FLAG 0x00
#define CANID_PRODUCT_ID 0x01

#define DEGREE_OF_FREEDOM 12

AgibotHandO12::AgibotHandO12(unsigned char device_id, unsigned char canfd_id, EHandType hand_type)
    : device_id_(device_id) {
  is_left_hand_ = hand_type == EHandType::eLeft;

#ifdef ZLG_USBCANFD_SDK
  canfd_device_ = std::make_unique<ZlgUsbcanfdSDK>(canfd_id);
#endif

#ifdef SOCKET_CAN
  canfd_device_ = std::make_unique<CanBusDeviceSocketCan>();
#endif

  init_success_ = canfd_device_->Init();
  canfd_device_->SetCallback(std::bind(&AgibotHandO12::ProcessMsg, this, std::placeholders::_1));
  canfd_device_->SetCalcuMatchRepId(std::bind(&AgibotHandO12::GetMatchedRepId, this, std::placeholders::_1));
  canfd_device_->SetMsgMatchJudge(std::bind(&AgibotHandO12::JudgeMsgMatch, this, std::placeholders::_1, std::placeholders::_2));

  kinematics_solver_ptr_ = std::make_unique<omnihandProSDK::O12KinematicsSolver>(is_left_hand_);
}

AgibotHandO12::~AgibotHandO12() {
}

void AgibotHandO12::SetJointMotorPosi(unsigned char joint_motor_index, int16_t posi) {
  if (joint_motor_index > 0 && joint_motor_index <= DEGREE_OF_FREEDOM) {
    UnCanId unCanId{};
    unCanId.st_can_Id_.device_id_ = device_id_;
    unCanId.st_can_Id_.rw_flag_ = CANID_WRITE_FLAG;
    unCanId.st_can_Id_.product_id_ = CANID_PRODUCT_ID;
    unCanId.st_can_Id_.msg_type_ = static_cast<unsigned char>(EMsgType::ePosiCtrl);
    unCanId.st_can_Id_.msg_id_ = joint_motor_index;

    CanfdFrame posiReqFrame{};
    posiReqFrame.can_id_ = unCanId.ui_can_id_;
    posiReqFrame.len_ = CANFD_MAX_DATA_LENGTH;
    memcpy(posiReqFrame.data_, &posi, sizeof(posi));
    try {
      CanfdFrame posiRepFrame = canfd_device_->SendRequestSynch(posiReqFrame);
    } catch (std::exception& ex) {
      std::cerr << ex.what() << std::endl;
    }
  } else {
    std::cerr << "[Error]: 无效关节电机ID参数" << std::dec << static_cast<unsigned int>(joint_motor_index) << " 正确范围：1～" << DEGREE_OF_FREEDOM << "." << std::endl;
    return;
  }
}

int16_t AgibotHandO12::GetJointMotorPosi(unsigned char joint_motor_index) {
  if (joint_motor_index > 0 && joint_motor_index <= DEGREE_OF_FREEDOM) {
    UnCanId unCanId{};
    unCanId.st_can_Id_.device_id_ = device_id_;
    unCanId.st_can_Id_.rw_flag_ = CANID_READ_FLAG;
    unCanId.st_can_Id_.product_id_ = CANID_PRODUCT_ID;
    unCanId.st_can_Id_.msg_type_ = static_cast<unsigned char>(EMsgType::ePosiCtrl);
    unCanId.st_can_Id_.msg_id_ = joint_motor_index;

    int16_t posi{};

    CanfdFrame posiReqFrame{};
    posiReqFrame.can_id_ = unCanId.ui_can_id_;
    posiReqFrame.len_ = CANFD_MAX_DATA_LENGTH;

    try {
      CanfdFrame posiRepFrame = canfd_device_->SendRequestSynch(posiReqFrame);
      memcpy(&posi, posiRepFrame.data_, sizeof(posi));
    } catch (std::exception& ex) {
      std::cerr << ex.what() << std::endl;
    }

    return posi;
  } else {
    std::cerr << "[Error]: 无效关节电机ID参数" << std::dec << static_cast<unsigned int>(joint_motor_index) << " 正确范围：1～" << DEGREE_OF_FREEDOM << "." << std::endl;
    return 0;
  }
}

void AgibotHandO12::SetAllJointMotorPosi(std::vector<int16_t> vec_posi) {
  if (vec_posi.size() != DEGREE_OF_FREEDOM) {
    std::cerr << "[Error]: 无效参数，需与主动自由度数量 " << std::dec << DEGREE_OF_FREEDOM << " 相匹配." << std::endl;
    return;
  }

  UnCanId unCanId{};
  unCanId.st_can_Id_.device_id_ = device_id_;
  unCanId.st_can_Id_.rw_flag_ = CANID_WRITE_FLAG;
  unCanId.st_can_Id_.product_id_ = CANID_PRODUCT_ID;
  unCanId.st_can_Id_.msg_type_ = static_cast<unsigned char>(EMsgType::ePosiCtrl);
  unCanId.st_can_Id_.msg_id_ = 0x00;

  CanfdFrame posiReqFrame{};
  posiReqFrame.can_id_ = unCanId.ui_can_id_;
  posiReqFrame.len_ = CANFD_MAX_DATA_LENGTH;
  memcpy(posiReqFrame.data_, vec_posi.data(), vec_posi.size() * sizeof(int16_t));
  try {
    CanfdFrame posiRepFrame = canfd_device_->SendRequestSynch(posiReqFrame);
  } catch (std::exception& ex) {
    std::cerr << ex.what() << std::endl;
  }
}

std::vector<int16_t> AgibotHandO12::GetAllJointMotorPosi() {
  UnCanId unCanId{};
  unCanId.st_can_Id_.device_id_ = device_id_;
  unCanId.st_can_Id_.rw_flag_ = CANID_READ_FLAG;
  unCanId.st_can_Id_.product_id_ = CANID_PRODUCT_ID;
  unCanId.st_can_Id_.msg_type_ = static_cast<unsigned char>(EMsgType::ePosiCtrl);
  unCanId.st_can_Id_.msg_id_ = 0x00;

  CanfdFrame posiReqFrame{};
  posiReqFrame.can_id_ = unCanId.ui_can_id_;
  posiReqFrame.len_ = CANFD_MAX_DATA_LENGTH;

  try {
    CanfdFrame posiRepFrame = canfd_device_->SendRequestSynch(posiReqFrame);
    return std::vector<int16_t>{reinterpret_cast<int16_t*>(posiRepFrame.data_), reinterpret_cast<int16_t*>(posiRepFrame.data_ + posiRepFrame.len_)};
  } catch (std::exception& ex) {
    std::cerr << ex.what() << std::endl;
    return {};
  }
}
#if !DISABLE_FUNC
void AgibotHandO12::SetActiveJointAngle(unsigned char joint_motor_index, double angle) {
  if (joint_motor_index > 0 && joint_motor_index <= DEGREE_OF_FREEDOM) {
    // 获取当前所有关节角度
    std::vector<double> current_angles = GetAllActiveJointAngles();
    for (auto a : current_angles) {
      std::cout << " | " << a << std::endl;
    }

    // 更新指定关节的角度
    current_angles[joint_motor_index - 1] = angle;

    // 转换为电机位置并设置
    std::vector<int> motor_positions = kinematics_solver_ptr_->ConvertJoint2Actuator(current_angles);
    SetJointMotorPosi(joint_motor_index, static_cast<int16_t>(motor_positions[joint_motor_index - 1]));
  } else {
    std::cerr << "[Error]: 无效关节电机ID参数" << std::dec << static_cast<unsigned int>(joint_motor_index) << " 正确范围：1～" << DEGREE_OF_FREEDOM << "." << std::endl;
    return;
  }
}

double AgibotHandO12::GetActiveJointAngle(unsigned char joint_motor_index) {
  if (joint_motor_index > 0 && joint_motor_index <= DEGREE_OF_FREEDOM) {
    // 获取当前电机位置
    int16_t motor_posi = GetJointMotorPosi(joint_motor_index);

    // 获取所有电机位置
    std::vector<int16_t> all_motor_posi = GetAllJointMotorPosi();

    // 转换为int向量供运动学求解器使用
    std::vector<int> motor_positions(all_motor_posi.begin(), all_motor_posi.end());

    // 转换为关节角度
    std::vector<double> joint_angles = kinematics_solver_ptr_->ConvertActuator2Joint(motor_positions);

    return joint_angles[joint_motor_index - 1];
  } else {
    std::cerr << "[Error]: 无效关节电机ID参数" << std::dec << static_cast<unsigned int>(joint_motor_index) << " 正确范围：1～" << DEGREE_OF_FREEDOM << "." << std::endl;
    return 0.0;
  }
}
#endif

void AgibotHandO12::SetAllActiveJointAngles(std::vector<double> vec_angle) {
  if (vec_angle.size() != DEGREE_OF_FREEDOM) {
    std::cerr << "[Error]: 无效参数，需与主动自由度数量 " << std::dec << DEGREE_OF_FREEDOM << " 相匹配." << std::endl;
    return;
  }

  // 使用运动学求解器转换关节角度为电机位置
  std::vector<int> motor_positions = kinematics_solver_ptr_->ConvertJoint2Actuator(vec_angle);

  // 转换为short向量
  std::vector<int16_t> motor_posi_short(motor_positions.begin(), motor_positions.end());

  // 设置所有电机位置
  SetAllJointMotorPosi(motor_posi_short);
}

std::vector<double> AgibotHandO12::GetAllActiveJointAngles() {
  // 获取所有电机位置
  std::vector<int16_t> motor_posi = GetAllJointMotorPosi();

  // 转换为int向量供运动学求解器使用
  std::vector<int> motor_positions(motor_posi.begin(), motor_posi.end());

  // 使用运动学求解器转换电机位置为关节角度
  return kinematics_solver_ptr_->ConvertActuator2Joint(motor_positions);
}

std::vector<double> AgibotHandO12::GetAllJointAngles() {
  std::vector<double> active_joint_angles = GetAllActiveJointAngles();
  return kinematics_solver_ptr_->GetAllJointPos(active_joint_angles);
  ;
}

#if !DISABLE_FUNC
void AgibotHandO12::SetJointMotorTorque(unsigned char joint_motor_index, int16_t torque) {
  if (joint_motor_index > 0 && joint_motor_index <= DEGREE_OF_FREEDOM) {
    UnCanId unCanId{};
    unCanId.st_can_Id_.device_id_ = device_id_;
    unCanId.st_can_Id_.rw_flag_ = CANID_WRITE_FLAG;
    unCanId.st_can_Id_.product_id_ = CANID_PRODUCT_ID;
    unCanId.st_can_Id_.msg_type_ = static_cast<unsigned char>(EMsgType::eTorqueCtrl);
    unCanId.st_can_Id_.msg_id_ = joint_motor_index;

    CanfdFrame torqueReqFrame{};
    torqueReqFrame.can_id_ = unCanId.ui_can_id_;
    torqueReqFrame.len_ = CANFD_MAX_DATA_LENGTH;
    memcpy(torqueReqFrame.data_, &torque, sizeof(torque));
    try {
      CanfdFrame torqueRepFrame = canfd_device_->SendRequestSynch(torqueReqFrame);
    } catch (std::exception& ex) {
      std::cerr << ex.what() << std::endl;
    }
  } else {
    std::cerr << "[Error]: 无效关节电机ID参数" << std::dec << static_cast<unsigned int>(joint_motor_index) << " 正确范围：1～" << DEGREE_OF_FREEDOM << "." << std::endl;
    return;
  }
}

int16_t AgibotHandO12::GetJointMotorTorque(unsigned char joint_motor_index) {
  if (joint_motor_index > 0 && joint_motor_index <= DEGREE_OF_FREEDOM) {
    UnCanId unCanId{};
    unCanId.st_can_Id_.device_id_ = device_id_;
    unCanId.st_can_Id_.rw_flag_ = CANID_READ_FLAG;
    unCanId.st_can_Id_.product_id_ = CANID_PRODUCT_ID;
    unCanId.st_can_Id_.msg_type_ = static_cast<unsigned char>(EMsgType::eTorqueCtrl);
    unCanId.st_can_Id_.msg_id_ = joint_motor_index;

    int16_t torque{};

    CanfdFrame torqueReqFrame{};
    torqueReqFrame.can_id_ = unCanId.ui_can_id_;
    torqueReqFrame.len_ = CANFD_MAX_DATA_LENGTH;

    try {
      CanfdFrame torqueRepFrame = canfd_device_->SendRequestSynch(torqueReqFrame);
      memcpy(&torque, torqueRepFrame.data_, sizeof(torque));
    } catch (std::exception& ex) {
      std::cerr << ex.what() << std::endl;
    }

    return torque;
  } else {
    std::cerr << "[Error]: 无效关节电机ID参数" << std::dec << static_cast<unsigned int>(joint_motor_index) << " 正确范围：1～" << DEGREE_OF_FREEDOM << "." << std::endl;
    return 0;
  }
}

void AgibotHandO12::SetAllJointMotorTorque(std::vector<int16_t> vec_torque) {
  if (vec_torque.size() != DEGREE_OF_FREEDOM) {
    std::cerr << "[Error]: 无效参数，需与主动自由度数量 " << std::dec << DEGREE_OF_FREEDOM << " 相匹配." << std::endl;
    return;
  }

  UnCanId unCanId{};
  unCanId.st_can_Id_.device_id_ = device_id_;
  unCanId.st_can_Id_.rw_flag_ = CANID_WRITE_FLAG;
  unCanId.st_can_Id_.product_id_ = CANID_PRODUCT_ID;
  unCanId.st_can_Id_.msg_type_ = static_cast<unsigned char>(EMsgType::eTorqueCtrl);
  unCanId.st_can_Id_.msg_id_ = 0x00;

  CanfdFrame torqueReqFrame{};
  torqueReqFrame.can_id_ = unCanId.ui_can_id_;
  torqueReqFrame.len_ = CANFD_MAX_DATA_LENGTH;
  memcpy(torqueReqFrame.data_, vec_torque.data(), vec_torque.size() * sizeof(int16_t));
  try {
    CanfdFrame torqueRepFrame = canfd_device_->SendRequestSynch(torqueReqFrame);
  } catch (std::exception& ex) {
    std::cerr << ex.what() << std::endl;
  }
}

std::vector<int16_t> AgibotHandO12::GetAllJointMotorTorque() {
  UnCanId unCanId{};
  unCanId.st_can_Id_.device_id_ = device_id_;
  unCanId.st_can_Id_.rw_flag_ = CANID_READ_FLAG;
  unCanId.st_can_Id_.product_id_ = CANID_PRODUCT_ID;
  unCanId.st_can_Id_.msg_type_ = static_cast<unsigned char>(EMsgType::eTorqueCtrl);
  unCanId.st_can_Id_.msg_id_ = 0x00;

  CanfdFrame torqueReqFrame{};
  torqueReqFrame.can_id_ = unCanId.ui_can_id_;
  torqueReqFrame.len_ = CANFD_MAX_DATA_LENGTH;

  try {
    CanfdFrame torqueRepFrame = canfd_device_->SendRequestSynch(torqueReqFrame);
    return std::vector<int16_t>{(int16_t*)torqueRepFrame.data_, (int16_t*)(torqueRepFrame.data_ + torqueRepFrame.len_)};
  } catch (std::exception& ex) {
    std::cerr << ex.what() << std::endl;
    return {};
  }
}
#endif

void AgibotHandO12::SetJointMotorVelo(unsigned char joint_motor_index, int16_t velo) {
  if (joint_motor_index > 0 && joint_motor_index <= DEGREE_OF_FREEDOM) {
    UnCanId unCanId{};
    unCanId.st_can_Id_.device_id_ = device_id_;
    unCanId.st_can_Id_.rw_flag_ = CANID_WRITE_FLAG;
    unCanId.st_can_Id_.product_id_ = CANID_PRODUCT_ID;
    unCanId.st_can_Id_.msg_type_ = static_cast<unsigned char>(EMsgType::eVeloCtrl);
    unCanId.st_can_Id_.msg_id_ = joint_motor_index;

    CanfdFrame veloReqFrame{};
    veloReqFrame.can_id_ = unCanId.ui_can_id_;
    veloReqFrame.len_ = CANFD_MAX_DATA_LENGTH;
    memcpy(veloReqFrame.data_, &velo, sizeof(velo));
    try {
      CanfdFrame veloRepFrame = canfd_device_->SendRequestSynch(veloReqFrame);
    } catch (std::exception& ex) {
      std::cerr << ex.what() << std::endl;
    }
  } else {
    std::cerr << "[Error]: 无效关节电机ID参数" << std::dec << static_cast<unsigned int>(joint_motor_index) << " 正确范围：1～" << DEGREE_OF_FREEDOM << "." << std::endl;
    return;
  }
}

int16_t AgibotHandO12::GetJointMotorVelo(unsigned char joint_motor_index) {
  if (joint_motor_index > 0 && joint_motor_index <= DEGREE_OF_FREEDOM) {
    UnCanId unCanId{};
    unCanId.st_can_Id_.device_id_ = device_id_;
    unCanId.st_can_Id_.rw_flag_ = CANID_READ_FLAG;
    unCanId.st_can_Id_.product_id_ = CANID_PRODUCT_ID;
    unCanId.st_can_Id_.msg_type_ = static_cast<unsigned char>(EMsgType::eVeloCtrl);
    unCanId.st_can_Id_.msg_id_ = joint_motor_index;

    int16_t velo{};

    CanfdFrame veloReqFrame{};
    veloReqFrame.can_id_ = unCanId.ui_can_id_;
    veloReqFrame.len_ = CANFD_MAX_DATA_LENGTH;

    try {
      CanfdFrame veloRepFrame = canfd_device_->SendRequestSynch(veloReqFrame);
      memcpy(&velo, veloRepFrame.data_, sizeof(velo));
    } catch (std::exception& ex) {
      std::cerr << ex.what() << std::endl;
    }

    return velo;
  } else {
    std::cerr << "[Error]: 无效关节电机ID参数" << std::dec << static_cast<unsigned int>(joint_motor_index) << " 正确范围：1～" << DEGREE_OF_FREEDOM << "." << std::endl;
    return 0;
  }
}

void AgibotHandO12::SetAllJointMotorVelo(std::vector<int16_t> vec_velo) {
  if (vec_velo.size() != DEGREE_OF_FREEDOM) {
    std::cerr << "[Error]: 无效参数，需与主动自由度数量 " << std::dec << DEGREE_OF_FREEDOM << " 相匹配." << std::endl;
    return;
  }

  UnCanId unCanId{};
  unCanId.st_can_Id_.device_id_ = device_id_;
  unCanId.st_can_Id_.rw_flag_ = CANID_WRITE_FLAG;
  unCanId.st_can_Id_.product_id_ = CANID_PRODUCT_ID;
  unCanId.st_can_Id_.msg_type_ = static_cast<unsigned char>(EMsgType::eVeloCtrl);
  unCanId.st_can_Id_.msg_id_ = 0x00;

  CanfdFrame veloReqFrame{};
  veloReqFrame.can_id_ = unCanId.ui_can_id_;
  veloReqFrame.len_ = CANFD_MAX_DATA_LENGTH;
  memcpy(veloReqFrame.data_, vec_velo.data(), vec_velo.size() * sizeof(int16_t));
  try {
    CanfdFrame veloRepFrame = canfd_device_->SendRequestSynch(veloReqFrame);
  } catch (std::exception& ex) {
    std::cerr << ex.what() << std::endl;
  }
}

std::vector<int16_t> AgibotHandO12::GetAllJointMotorVelo() {
  UnCanId unCanId{};
  unCanId.st_can_Id_.device_id_ = device_id_;
  unCanId.st_can_Id_.rw_flag_ = CANID_READ_FLAG;
  unCanId.st_can_Id_.product_id_ = CANID_PRODUCT_ID;
  unCanId.st_can_Id_.msg_type_ = static_cast<unsigned char>(EMsgType::eVeloCtrl);
  unCanId.st_can_Id_.msg_id_ = 0x00;

  CanfdFrame veloReqFrame{};
  veloReqFrame.can_id_ = unCanId.ui_can_id_;
  veloReqFrame.len_ = CANFD_MAX_DATA_LENGTH;

  try {
    CanfdFrame veloRepFrame = canfd_device_->SendRequestSynch(veloReqFrame);
    return std::vector<int16_t>{reinterpret_cast<int16_t*>(veloRepFrame.data_), reinterpret_cast<int16_t*>(veloRepFrame.data_ + veloRepFrame.len_)};
  } catch (std::exception& ex) {
    std::cerr << ex.what() << std::endl;
    return {};
  }
}

TactileSensorData AgibotHandO12::GetTactileSensorData(EFinger eFinger) {
  TactileSensorData tactileSensorData{};

  UnCanId unCanId{};
  unCanId.st_can_Id_.device_id_ = device_id_;
  unCanId.st_can_Id_.rw_flag_ = CANID_READ_FLAG;
  unCanId.st_can_Id_.product_id_ = CANID_PRODUCT_ID;
  unCanId.st_can_Id_.msg_type_ = static_cast<unsigned char>(EMsgType::eTactileSensor);
  unCanId.st_can_Id_.msg_id_ = static_cast<unsigned char>(eFinger);

  CanfdFrame tactileSensorDataReq{};
  tactileSensorDataReq.can_id_ = unCanId.ui_can_id_;
  tactileSensorDataReq.len_ = CANFD_MAX_DATA_LENGTH;
  try {
    CanfdFrame tactileSensorDataRep = canfd_device_->SendRequestSynch(tactileSensorDataReq);
    memcpy(&tactileSensorData, tactileSensorDataRep.data_, sizeof(tactileSensorData));
  } catch (std::exception& ex) {
    std::cerr << ex.what() << std::endl;
  }

  return tactileSensorData;
}

void AgibotHandO12::SetControlMode(unsigned char joint_motor_index, EControlMode mode) {
  if (joint_motor_index > 0 && joint_motor_index <= DEGREE_OF_FREEDOM) {
    UnCanId unCanId{};
    unCanId.st_can_Id_.device_id_ = device_id_;
    unCanId.st_can_Id_.rw_flag_ = CANID_WRITE_FLAG;
    unCanId.st_can_Id_.product_id_ = CANID_PRODUCT_ID;
    unCanId.st_can_Id_.msg_type_ = static_cast<unsigned char>(EMsgType::eCtrlMode);
    unCanId.st_can_Id_.msg_id_ = joint_motor_index;

    CanfdFrame ctlModeReq{};
    ctlModeReq.can_id_ = unCanId.ui_can_id_;
    ctlModeReq.len_ = CANFD_MAX_DATA_LENGTH;
    unsigned char ucMode = static_cast<unsigned char>(mode);
    memcpy(ctlModeReq.data_, &ucMode, sizeof(ucMode));
    try {
      CanfdFrame ctlModeRep = canfd_device_->SendRequestSynch(ctlModeReq);
    } catch (std::exception& ex) {
      std::cerr << ex.what() << std::endl;
    }
  } else {
    std::cerr << "[Error]: 无效关节电机ID参数" << std::dec << static_cast<unsigned int>(joint_motor_index) << " 正确范围：1～" << DEGREE_OF_FREEDOM << "." << std::endl;
    return;
  }
}

EControlMode AgibotHandO12::GetControlMode(unsigned char joint_motor_index) {
  if (joint_motor_index > 0 && joint_motor_index <= DEGREE_OF_FREEDOM) {
    UnCanId unCanId{};
    unCanId.st_can_Id_.device_id_ = device_id_;
    unCanId.st_can_Id_.rw_flag_ = CANID_READ_FLAG;
    unCanId.st_can_Id_.product_id_ = CANID_PRODUCT_ID;
    unCanId.st_can_Id_.msg_type_ = static_cast<unsigned char>(EMsgType::eCtrlMode);
    unCanId.st_can_Id_.msg_id_ = joint_motor_index;

    CanfdFrame ctlModeReq{};
    ctlModeReq.can_id_ = unCanId.ui_can_id_;
    ctlModeReq.len_ = CANFD_MAX_DATA_LENGTH;
    try {
      CanfdFrame ctlModeRep = canfd_device_->SendRequestSynch(ctlModeReq);
      return static_cast<EControlMode>(ctlModeRep.data_[0] & 0x07);
    } catch (std::exception& ex) {
      std::cerr << ex.what() << std::endl;
      return EControlMode::eUnknown;
    }
  } else {
    std::cerr << "[Error]: 无效关节电机ID参数" << std::dec << static_cast<unsigned int>(joint_motor_index) << " 正确范围：1～" << DEGREE_OF_FREEDOM << "." << std::endl;
    return EControlMode::eUnknown;
  }
}

void AgibotHandO12::SetAllControlMode(std::vector<unsigned char> vec_ctrl_mode) {
  if (vec_ctrl_mode.size() != DEGREE_OF_FREEDOM) {
    std::cerr << "[Error]: 无效参数，需与主动自由度数量 " << std::dec << DEGREE_OF_FREEDOM << " 相匹配." << std::endl;
    return;
  }

  UnCanId unCanId{};
  unCanId.st_can_Id_.device_id_ = device_id_;
  unCanId.st_can_Id_.rw_flag_ = CANID_WRITE_FLAG;
  unCanId.st_can_Id_.product_id_ = CANID_PRODUCT_ID;
  unCanId.st_can_Id_.msg_type_ = static_cast<unsigned char>(EMsgType::eCtrlMode);
  unCanId.st_can_Id_.msg_id_ = 0x00;

  CanfdFrame ctlModeReq{};
  ctlModeReq.can_id_ = unCanId.ui_can_id_;
  ctlModeReq.len_ = vec_ctrl_mode.size() * sizeof(unsigned char);
  memcpy(ctlModeReq.data_, vec_ctrl_mode.data(), vec_ctrl_mode.size() * sizeof(unsigned char));
  try {
    CanfdFrame ctlModeRep = canfd_device_->SendRequestSynch(ctlModeReq);
  } catch (std::exception& ex) {
    std::cerr << ex.what() << std::endl;
  }
}

std::vector<unsigned char> AgibotHandO12::GetAllControlMode() {
  UnCanId unCanId{};
  unCanId.st_can_Id_.device_id_ = device_id_;
  unCanId.st_can_Id_.rw_flag_ = CANID_READ_FLAG;
  unCanId.st_can_Id_.product_id_ = CANID_PRODUCT_ID;
  unCanId.st_can_Id_.msg_type_ = static_cast<unsigned char>(EMsgType::eCtrlMode);
  unCanId.st_can_Id_.msg_id_ = 0x00;

  CanfdFrame ctlModeReq{};
  ctlModeReq.can_id_ = unCanId.ui_can_id_;
  ctlModeReq.len_ = CANFD_MAX_DATA_LENGTH;
  try {
    CanfdFrame ctlModeRep = canfd_device_->SendRequestSynch(ctlModeReq);
    return std::vector<unsigned char>(ctlModeRep.data_, ctlModeRep.data_ + ctlModeRep.len_);
  } catch (std::exception& ex) {
    std::cerr << ex.what() << std::endl;
    return {};
  }
}

void AgibotHandO12::SetCurrentThreshold(unsigned char joint_motor_index, int16_t current_threshold) {
  if (joint_motor_index > 0 && joint_motor_index <= DEGREE_OF_FREEDOM) {
    UnCanId unCanId{};
    unCanId.st_can_Id_.device_id_ = device_id_;
    unCanId.st_can_Id_.rw_flag_ = CANID_WRITE_FLAG;
    unCanId.st_can_Id_.product_id_ = CANID_PRODUCT_ID;
    unCanId.st_can_Id_.msg_type_ = static_cast<unsigned char>(EMsgType::eCurrentThreshold);
    unCanId.st_can_Id_.msg_id_ = joint_motor_index;

    CanfdFrame currentThreshReqFrame{};
    currentThreshReqFrame.can_id_ = unCanId.ui_can_id_;
    currentThreshReqFrame.len_ = CANFD_MAX_DATA_LENGTH;
    memcpy(currentThreshReqFrame.data_, &current_threshold, sizeof(current_threshold));
    try {
      CanfdFrame currentThreshRepFrame = canfd_device_->SendRequestSynch(currentThreshReqFrame);
    } catch (std::exception& ex) {
      std::cerr << ex.what() << std::endl;
    }
  } else {
    std::cerr << "[Error]: 无效关节电机ID参数" << std::dec << static_cast<unsigned int>(joint_motor_index) << " 正确范围：1～" << DEGREE_OF_FREEDOM << "." << std::endl;
    return;
  }
}

int16_t AgibotHandO12::GetCurrentThreshold(unsigned char joint_motor_index) {
  if (joint_motor_index > 0 && joint_motor_index <= DEGREE_OF_FREEDOM) {
    UnCanId unCanId{};
    unCanId.st_can_Id_.device_id_ = device_id_;
    unCanId.st_can_Id_.rw_flag_ = CANID_READ_FLAG;
    unCanId.st_can_Id_.product_id_ = CANID_PRODUCT_ID;
    unCanId.st_can_Id_.msg_type_ = static_cast<unsigned char>(EMsgType::eCurrentThreshold);
    unCanId.st_can_Id_.msg_id_ = joint_motor_index;

    int16_t currentThreshold{};

    CanfdFrame currentThreshReqFrame{};
    currentThreshReqFrame.can_id_ = unCanId.ui_can_id_;
    currentThreshReqFrame.len_ = CANFD_MAX_DATA_LENGTH;

    try {
      CanfdFrame currentThreshRepFrame = canfd_device_->SendRequestSynch(currentThreshReqFrame);
      memcpy(&currentThreshold, currentThreshRepFrame.data_, sizeof(currentThreshold));
    } catch (std::exception& ex) {
      std::cerr << ex.what() << std::endl;
    }

    return currentThreshold;
  } else {
    std::cerr << "[Error]: 无效关节电机ID参数" << std::dec << static_cast<unsigned int>(joint_motor_index) << " 正确范围：1～" << DEGREE_OF_FREEDOM << "." << std::endl;
    return {};
  }
}

void AgibotHandO12::SetAllCurrentThreshold(std::vector<int16_t> vec_current_threshold) {
  if (vec_current_threshold.size() != DEGREE_OF_FREEDOM) {
    std::cerr << "[Error]: 无效参数，需与主动自由度数量 " << std::dec << DEGREE_OF_FREEDOM << " 相匹配." << std::endl;
    return;
  }

  UnCanId unCanId{};
  unCanId.st_can_Id_.device_id_ = device_id_;
  unCanId.st_can_Id_.rw_flag_ = CANID_WRITE_FLAG;
  unCanId.st_can_Id_.product_id_ = CANID_PRODUCT_ID;
  unCanId.st_can_Id_.msg_type_ = static_cast<unsigned char>(EMsgType::eCurrentThreshold);
  unCanId.st_can_Id_.msg_id_ = 0x00;

  CanfdFrame currentThreshReq{};
  currentThreshReq.can_id_ = unCanId.ui_can_id_;
  currentThreshReq.len_ = CANFD_MAX_DATA_LENGTH;
  memcpy(currentThreshReq.data_, vec_current_threshold.data(), vec_current_threshold.size() * sizeof(int16_t));
  try {
    CanfdFrame currentThreshRep = canfd_device_->SendRequestSynch(currentThreshReq);
  } catch (std::exception& ex) {
    std::cerr << ex.what() << std::endl;
  }
}

std::vector<int16_t> AgibotHandO12::GetAllCurrentThreshold() {
  UnCanId unCanId{};
  unCanId.st_can_Id_.device_id_ = device_id_;
  unCanId.st_can_Id_.rw_flag_ = CANID_READ_FLAG;
  unCanId.st_can_Id_.product_id_ = CANID_PRODUCT_ID;
  unCanId.st_can_Id_.msg_type_ = static_cast<unsigned char>(EMsgType::eCurrentThreshold);
  unCanId.st_can_Id_.msg_id_ = 0x00;

  CanfdFrame currentThreshReq{};
  currentThreshReq.can_id_ = unCanId.ui_can_id_;
  currentThreshReq.len_ = CANFD_MAX_DATA_LENGTH;
  try {
    CanfdFrame currentThreshRep = canfd_device_->SendRequestSynch(currentThreshReq);
    return std::vector<int16_t>{reinterpret_cast<int16_t*>(currentThreshRep.data_), reinterpret_cast<int16_t*>(currentThreshRep.data_ + currentThreshRep.len_)};
  } catch (std::exception& ex) {
    std::cerr << ex.what() << std::endl;
    return {};
  }
}

void AgibotHandO12::MixCtrlJointMotor(std::vector<MixCtrl> vec_mix_ctrl) {
  if (vec_mix_ctrl.size() > 0) {
    auto ctrlMode = EControlMode(vec_mix_ctrl[0].ctrl_mode_);

    if (ctrlMode == EControlMode::ePosiTorque || ctrlMode == EControlMode::eVeloTorque) {
      if (vec_mix_ctrl.size() > 12) {
        std::cerr << "[Error]: 无效参数，需与主动自由度数量 " << std::dec << DEGREE_OF_FREEDOM << " 相匹配." << std::endl;
        return;
      }
    } else if (ctrlMode == EControlMode::ePosiVeloTorque) {
      if (vec_mix_ctrl.size() > 8) {
        std::cerr << "[Error]: 无效参数，位置速度力控模式最多一次性下发8个关节目标信息." << std::endl;
        return;
      }
    }

    UnCanId unCanId{};
    unCanId.st_can_Id_.device_id_ = device_id_;
    unCanId.st_can_Id_.rw_flag_ = CANID_WRITE_FLAG;
    unCanId.st_can_Id_.product_id_ = CANID_PRODUCT_ID;
    unCanId.st_can_Id_.msg_type_ = static_cast<unsigned char>(EMsgType::eMixCtrl);
    unCanId.st_can_Id_.msg_id_ = 0x00;

    CanfdFrame mixCtrlReq{};
    mixCtrlReq.can_id_ = unCanId.ui_can_id_;
    mixCtrlReq.len_ = CANFD_MAX_DATA_LENGTH;
    unsigned char* head = mixCtrlReq.data_;
    for (auto& mixCtrl : vec_mix_ctrl) {
      if (ctrlMode == EControlMode::ePosiTorque) {
        memcpy(head, &mixCtrl, sizeof(unsigned char));
        head += sizeof(unsigned char);
        memcpy(head, &mixCtrl.tgt_posi_.value(), sizeof(int16_t));
        head += sizeof(int16_t);
        memcpy(head, &mixCtrl.tgt_torque_.value(), sizeof(int16_t));
        head += sizeof(int16_t);
      } else if (ctrlMode == EControlMode::eVeloTorque) {
        memcpy(head, &mixCtrl, sizeof(unsigned char));
        head += sizeof(unsigned char);
        memcpy(head, &mixCtrl.tgt_velo_.value(), sizeof(int16_t));
        head += sizeof(int16_t);
        memcpy(head, &mixCtrl.tgt_torque_.value(), sizeof(int16_t));
        head += sizeof(int16_t);
      } else if (ctrlMode == EControlMode::ePosiVeloTorque) {
        memcpy(head, &mixCtrl, sizeof(unsigned char));
        head += sizeof(unsigned char);
        memcpy(head, &mixCtrl.tgt_posi_.value(), sizeof(int16_t));
        head += sizeof(int16_t);
        memcpy(head, &mixCtrl.tgt_velo_.value(), sizeof(int16_t));
        head += sizeof(int16_t);
        memcpy(head, &mixCtrl.tgt_torque_.value(), sizeof(int16_t));
        head += sizeof(int16_t);
      } else {
        return;
      }
    }

    try {
      CanfdFrame mixCtrlRep = canfd_device_->SendRequestSynch(mixCtrlReq);
    } catch (std::exception& ex) {
      std::cerr << ex.what() << std::endl;
    }
  }
}

JointMotorErrorReport AgibotHandO12::GetErrorReport(unsigned char joint_motor_index) {
  if (joint_motor_index > 0 && joint_motor_index <= DEGREE_OF_FREEDOM) {
    UnCanId unCanId{};
    unCanId.st_can_Id_.device_id_ = device_id_;
    unCanId.st_can_Id_.rw_flag_ = CANID_READ_FLAG;
    unCanId.st_can_Id_.product_id_ = CANID_PRODUCT_ID;
    unCanId.st_can_Id_.msg_type_ = static_cast<unsigned char>(EMsgType::eErrorReport);
    unCanId.st_can_Id_.msg_id_ = joint_motor_index;

    JointMotorErrorReport errReport{};

    CanfdFrame errReportReq{};
    errReportReq.can_id_ = unCanId.ui_can_id_;
    errReportReq.len_ = CANFD_MAX_DATA_LENGTH;
    try {
      CanfdFrame errReportRep = canfd_device_->SendRequestSynch(errReportReq);
      memcpy(&errReport, errReportRep.data_, sizeof(errReport));
    } catch (std::exception& ex) {
      std::cerr << ex.what() << std::endl;
    }

    return errReport;
  } else {
    std::cerr << "[Error]: 无效关节电机ID参数" << std::dec << static_cast<unsigned int>(joint_motor_index) << " 正确范围：1～" << DEGREE_OF_FREEDOM << "." << std::endl;
    return {};
  }
}

std::vector<JointMotorErrorReport> AgibotHandO12::GetAllErrorReport() {
  UnCanId unCanId{};
  unCanId.st_can_Id_.device_id_ = device_id_;
  unCanId.st_can_Id_.rw_flag_ = CANID_READ_FLAG;
  unCanId.st_can_Id_.product_id_ = CANID_PRODUCT_ID;
  unCanId.st_can_Id_.msg_type_ = static_cast<unsigned char>(EMsgType::eErrorReport);
  unCanId.st_can_Id_.msg_id_ = 0x00;

  CanfdFrame errReportReq{};
  errReportReq.can_id_ = unCanId.ui_can_id_;
  errReportReq.len_ = CANFD_MAX_DATA_LENGTH;
  try {
    CanfdFrame errReportRep = canfd_device_->SendRequestSynch(errReportReq);
    return std::vector<JointMotorErrorReport>{(JointMotorErrorReport*)errReportRep.data_, (JointMotorErrorReport*)(errReportRep.data_ + errReportRep.len_)};
  } catch (std::exception& ex) {
    std::cerr << ex.what() << std::endl;
    return {};
  }
}

void AgibotHandO12::SetErrorReportPeriod(unsigned char joint_motor_index, uint16_t period) {
  if (joint_motor_index > 0 && joint_motor_index <= DEGREE_OF_FREEDOM) {
    UnCanId unCanId{};
    unCanId.st_can_Id_.device_id_ = device_id_;
    unCanId.st_can_Id_.rw_flag_ = CANID_WRITE_FLAG;
    unCanId.st_can_Id_.product_id_ = CANID_PRODUCT_ID;
    unCanId.st_can_Id_.msg_type_ = static_cast<unsigned char>(EMsgType::eErrorReport);
    unCanId.st_can_Id_.msg_id_ = joint_motor_index;

    CanfdFrame errReportReq{};
    errReportReq.can_id_ = unCanId.ui_can_id_;
    errReportReq.len_ = CANFD_MAX_DATA_LENGTH;
    memcpy(errReportReq.data_, &period, sizeof(period));
    try {
      canfd_device_->SendRequestWithoutReply(errReportReq);
    } catch (std::exception& ex) {
      std::cerr << ex.what() << std::endl;
    }
  } else {
    std::cerr << "[Error]: 无效关节电机ID参数" << std::dec << static_cast<unsigned int>(joint_motor_index) << " 正确范围：1～" << DEGREE_OF_FREEDOM << "." << std::endl;
    return;
  }
}

void AgibotHandO12::SetAllErrorReportPeriod(std::vector<uint16_t> vec_period) {
  if (vec_period.size() != DEGREE_OF_FREEDOM) {
    std::cerr << "[Error]: 无效参数，需与主动自由度数量 " << std::dec << DEGREE_OF_FREEDOM << " 相匹配." << std::endl;
    return;
  }

  UnCanId unCanId{};
  unCanId.st_can_Id_.device_id_ = device_id_;
  unCanId.st_can_Id_.rw_flag_ = CANID_WRITE_FLAG;
  unCanId.st_can_Id_.product_id_ = CANID_PRODUCT_ID;
  unCanId.st_can_Id_.msg_type_ = static_cast<unsigned char>(EMsgType::eErrorReport);
  unCanId.st_can_Id_.msg_id_ = 0x00;

  CanfdFrame errReportReq{};
  errReportReq.can_id_ = unCanId.ui_can_id_;
  errReportReq.len_ = CANFD_MAX_DATA_LENGTH;
  memcpy(errReportReq.data_, vec_period.data(), sizeof(uint16_t) * vec_period.size());
  try {
    CanfdFrame errReportRep = canfd_device_->SendRequestSynch(errReportReq);
  } catch (std::exception& ex) {
    std::cerr << ex.what() << std::endl;
  }
}

uint16_t AgibotHandO12::GetTemperatureReport(unsigned char joint_motor_index) {
  if (joint_motor_index > 0 && joint_motor_index <= DEGREE_OF_FREEDOM) {
    std::lock_guard<std::mutex> lockGuard(mutex_temper_report_);
    if (vec_temper_report_.size() == DEGREE_OF_FREEDOM) {
      return vec_temper_report_[joint_motor_index];
    } else {
      return {};
    }
  } else {
    std::cerr << "[Error]: 无效关节电机ID参数" << std::dec << static_cast<unsigned int>(joint_motor_index) << " 正确范围：1～" << DEGREE_OF_FREEDOM << "." << std::endl;
    return {};
  }
}

std::vector<uint16_t> AgibotHandO12::GetAllTemperatureReport() {
  std::lock_guard<std::mutex> lockGuard(mutex_temper_report_);
  return vec_temper_report_;
}

void AgibotHandO12::SetTemperReportPeriod(unsigned char joint_motor_index, uint16_t period) {
  if (joint_motor_index > 0 && joint_motor_index <= DEGREE_OF_FREEDOM) {
    UnCanId unCanId{};
    unCanId.st_can_Id_.device_id_ = device_id_;
    unCanId.st_can_Id_.rw_flag_ = CANID_WRITE_FLAG;
    unCanId.st_can_Id_.product_id_ = CANID_PRODUCT_ID;
    unCanId.st_can_Id_.msg_type_ = static_cast<unsigned char>(EMsgType::eTemperatureReport);
    unCanId.st_can_Id_.msg_id_ = joint_motor_index;

    CanfdFrame temperReportReq{};
    temperReportReq.can_id_ = unCanId.ui_can_id_;
    temperReportReq.len_ = CANFD_MAX_DATA_LENGTH;
    memcpy(&temperReportReq.data_, &period, sizeof(period));
    try {
      CanfdFrame temperReportRep = canfd_device_->SendRequestSynch(temperReportReq);
    } catch (std::exception& ex) {
      std::cerr << ex.what() << std::endl;
    }
  } else {
    std::cerr << "[Error]: 无效关节电机ID参数" << std::dec << static_cast<unsigned int>(joint_motor_index) << " 正确范围：1～" << DEGREE_OF_FREEDOM << "." << std::endl;
    return;
  }
}

void AgibotHandO12::SetAllTemperReportPeriod(std::vector<uint16_t> vec_period) {
  if (vec_period.size() != DEGREE_OF_FREEDOM) {
    std::cerr << "[Error]: 无效参数，需与主动自由度数量 " << std::dec << DEGREE_OF_FREEDOM << " 相匹配." << std::endl;
    return;
  }

  UnCanId unCanId{};
  unCanId.st_can_Id_.device_id_ = device_id_;
  unCanId.st_can_Id_.rw_flag_ = CANID_WRITE_FLAG;
  unCanId.st_can_Id_.product_id_ = CANID_PRODUCT_ID;
  unCanId.st_can_Id_.msg_type_ = static_cast<unsigned char>(EMsgType::eTemperatureReport);
  unCanId.st_can_Id_.msg_id_ = 0x00;

  CanfdFrame temperReportReq{};
  temperReportReq.can_id_ = unCanId.ui_can_id_;
  temperReportReq.len_ = CANFD_MAX_DATA_LENGTH;
  for (int i = 0; i < DEGREE_OF_FREEDOM; i++) {
    memcpy(temperReportReq.data_ + i * sizeof(uint16_t), &vec_period[i], sizeof(uint16_t));
  }
  // memcpy(&temperReportReq.data_, vec_period.data(), vec_period.size() * sizeof(uint16_t));
  try {
    canfd_device_->SendRequestWithoutReply(temperReportReq);
  } catch (std::exception& ex) {
    std::cerr << ex.what() << std::endl;
  }
}

int16_t AgibotHandO12::GetCurrentReport(unsigned char joint_motor_index) {
  if (joint_motor_index > 0 && joint_motor_index <= DEGREE_OF_FREEDOM) {
    std::lock_guard<std::mutex> lockGuard(mutex_current_report_);
    if (vec_current_report_.size() == DEGREE_OF_FREEDOM) {
      return vec_current_report_[joint_motor_index];
    } else {
      return {};
    }
  } else {
    std::cerr << "[Error]: 无效关节电机ID参数" << std::dec << static_cast<unsigned int>(joint_motor_index) << " 正确范围：1～" << DEGREE_OF_FREEDOM << "." << std::endl;
    return {};
  }
}

std::vector<uint16_t> AgibotHandO12::GetAllCurrentReport() {
  std::lock_guard<std::mutex> lockGuard(mutex_current_report_);
  return vec_current_report_;
}

void AgibotHandO12::SetCurrentReportPeriod(unsigned char joint_motor_index, uint16_t period) {
  if (joint_motor_index > 0 && joint_motor_index <= DEGREE_OF_FREEDOM) {
    UnCanId unCanId{};
    unCanId.st_can_Id_.device_id_ = device_id_;
    unCanId.st_can_Id_.rw_flag_ = CANID_WRITE_FLAG;
    unCanId.st_can_Id_.product_id_ = CANID_PRODUCT_ID;
    unCanId.st_can_Id_.msg_type_ = static_cast<unsigned char>(EMsgType::eCurrentReport);
    unCanId.st_can_Id_.msg_id_ = joint_motor_index;

    CanfdFrame currentReportReq{};
    currentReportReq.can_id_ = unCanId.ui_can_id_;
    currentReportReq.len_ = CANFD_MAX_DATA_LENGTH;
    memcpy(&currentReportReq.data_, &period, sizeof(period));
    try {
      canfd_device_->SendRequestWithoutReply(currentReportReq);
    } catch (std::exception& ex) {
      std::cerr << ex.what() << std::endl;
    }
  } else {
    std::cerr << "[Error]: 无效关节电机ID参数" << std::dec << static_cast<unsigned int>(joint_motor_index) << " 正确范围：1～" << DEGREE_OF_FREEDOM << "." << std::endl;
    return;
  }
}

void AgibotHandO12::SetAllCurrentReportPeriod(std::vector<uint16_t> vec_period) {
  if (vec_period.size() != DEGREE_OF_FREEDOM) {
    std::cerr << "[Error]: 无效参数，需与主动自由度数量 " << std::dec << DEGREE_OF_FREEDOM << " 相匹配." << std::endl;
    return;
  }

  UnCanId unCanId{};
  unCanId.st_can_Id_.device_id_ = device_id_;
  unCanId.st_can_Id_.rw_flag_ = CANID_WRITE_FLAG;
  unCanId.st_can_Id_.product_id_ = CANID_PRODUCT_ID;
  unCanId.st_can_Id_.msg_type_ = static_cast<unsigned char>(EMsgType::eCurrentReport);
  unCanId.st_can_Id_.msg_id_ = 0x00;

  CanfdFrame currentReportReq{};
  currentReportReq.can_id_ = unCanId.ui_can_id_;
  currentReportReq.len_ = CANFD_MAX_DATA_LENGTH;
  for (int i = 0; i < DEGREE_OF_FREEDOM; i++) {
    memcpy(currentReportReq.data_ + i * sizeof(uint16_t), &vec_period[i], sizeof(uint16_t));
  }
  // memcpy(&currentReportReq.data_, vec_period.data(), vec_period.size() * sizeof(uint16_t));
  try {
    canfd_device_->SendRequestWithoutReply(currentReportReq);
  } catch (std::exception& ex) {
    std::cerr << ex.what() << std::endl;
  }
}

void AgibotHandO12::ProcessMsg(CanfdFrame frame) {
  if ((frame.can_id_ & 0xFFFF0000) == 0x00210000) {
    // unsigned char* head = frame.data_;
    // std::vector<uint16_t> vec_temper_report{};
    //
    // for (int i = 0;i < DEGREE_OF_FREEDOM;i++) {
    //   uint16_t temperReport{};
    //   memcpy(&temperReport, &frame.data_[i*sizeof(temperReport)], sizeof(temperReport));
    //   vec_temper_report.push_back(temperReport);
    // }

    std::vector<uint16_t> vec_temper_report{reinterpret_cast<uint16_t*>(frame.data_),
                                            reinterpret_cast<uint16_t*>(frame.data_ + frame.len_)};
    {
      std::lock_guard<std::mutex> lockGuard(mutex_temper_report_);
      vec_temper_report_ = vec_temper_report;
    }
  } else if ((frame.can_id_ & 0xFFFF0000) == 0x00220000) {
    // std::vector<uint16_t> vec_current_report{};
    //
    // for (int i = 0;i < DEGREE_OF_FREEDOM;i++) {
    //   uint16_t currentReport{};
    //   memcpy(&currentReport, &frame.data_[i*sizeof(currentReport)], sizeof(currentReport));
    //   vec_current_report.push_back(currentReport);
    // }

    std::vector<uint16_t> vec_current_report{reinterpret_cast<uint16_t*>(frame.data_),
                                             reinterpret_cast<uint16_t*>(frame.data_ + frame.len_)};
    {
      std::lock_guard<std::mutex> lockGuard(mutex_current_report_);
      vec_current_report_ = vec_current_report;
    }
  } else if ((frame.can_id_ & 0xFFFF0000) == 0x00200000) {
    std::vector<JointMotorErrorReport> vecErrorReport{reinterpret_cast<JointMotorErrorReport*>(frame.data_),
                                                      reinterpret_cast<JointMotorErrorReport*>(frame.data_ + frame.len_)};
    for (int index = 0; index < vecErrorReport.size(); index++) {
      JointMotorErrorReport errReport = vecErrorReport[index];
      if (errReport.stalled_) {
        std::cerr << "[Error]: " << std::dec << index + 1 << " 号关节电机 堵转！" << std::endl;
      }

      if (errReport.overheat_) {
        std::cerr << "[Error]: " << std::dec << index + 1 << " 号关节电机 过温！" << std::endl;
      }

      if (errReport.over_current_) {
        std::cerr << "[Error]: " << std::dec << index + 1 << " 号关节电机 过流！" << std::endl;
      }

      if (errReport.motor_except_) {
        std::cerr << "[Error]: " << std::dec << index + 1 << " 号关节电机 电机异常！" << std::endl;
      }

      if (errReport.commu_except_) {
        std::cerr << "[Error]: " << std::dec << index + 1 << " 号关节电机 通讯异常！" << std::endl;
      }
    }
  } else if ((frame.can_id_ & 0x00FF0000) == 0x00200000) {
    JointMotorErrorReport errReport{};
    memcpy(&errReport, frame.data_, sizeof(errReport));
    if (errReport.stalled_) {
      std::cerr << "[Error]: " << std::dec << ((frame.can_id_ & 0xFF000000) >> 24) << " 号关节电机 堵转！" << std::endl;
    }

    if (errReport.overheat_) {
      std::cerr << "[Error]: " << std::dec << ((frame.can_id_ & 0xFF000000) >> 24) << " 号关节电机 过温！" << std::endl;
    }

    if (errReport.over_current_) {
      std::cerr << "[Error]: " << std::dec << ((frame.can_id_ & 0xFF000000) >> 24) << " 号关节电机 过流！" << std::endl;
    }

    if (errReport.motor_except_) {
      std::cerr << "[Error]: " << std::dec << ((frame.can_id_ & 0xFF000000) >> 24) << " 号关节电机 电机异常！" << std::endl;
    }

    if (errReport.commu_except_) {
      std::cerr << "[Error]: " << std::dec << ((frame.can_id_ & 0xFF000000) >> 24) << " 号关节电机 通讯异常！" << std::endl;
    }
  } else if ((frame.can_id_ & 0xFFFF0000) == 0x01F20000) {
    // 收到了OTA升级应答
    SendPackage();
  } else if ((frame.can_id_ & 0xFFFF0000) == 0x02F20000) {
    // 收到了OTA数据传输应答
    que_packages_.pop();  // 收到应答后再弹出
    if (que_packages_.empty()) {
      // 发送完成请求
      CanfdFrame finishReq{};
      finishReq.can_id_ = 0x13F10101;
      finishReq.len_ = CANFD_MAX_DATA_LENGTH;
      try {
        canfd_device_->SendRequestWithoutReply(finishReq);  // 不等待应答，直接发送一包共32帧，然后等待应答才发最后一帧
      } catch (std::exception& ex) {
        std::cerr << ex.what() << std::endl;
      }
    } else {
      SendPackage();  // 再发送下一包
    }
  } else if ((frame.can_id_ & 0xFFFF0000) == 0x03F20000) {
    // 收到了OTA完成应答
    // 发送重启请求
    CanfdFrame restartReq{};
    restartReq.can_id_ = 0x14F10101;
    restartReq.len_ = CANFD_MAX_DATA_LENGTH;
    try {
      canfd_device_->SendRequestWithoutReply(restartReq);  // 不等待应答，直接发送一包共32帧，然后等待应答才发最后一帧
    } catch (std::exception& ex) {
      std::cerr << ex.what() << std::endl;
    }
  } else if ((frame.can_id_ & 0xFFFF0000) == 0x04F20000) {
    // 收到了OTA重启应答
    // TODO 发送结果请求，一秒发送一次等待灵巧手重启
  } else if ((frame.can_id_ & 0xFFFF0000) == 0x05F20000) {
    // 收到了OTA结构应答
    // TODO 提示用户OTA升级完成
  }
}

VendorInfo AgibotHandO12::GetVendorInfo() {
  UnCanId unCanId{};
  unCanId.st_can_Id_.device_id_ = device_id_;
  unCanId.st_can_Id_.rw_flag_ = CANID_READ_FLAG;
  unCanId.st_can_Id_.product_id_ = CANID_PRODUCT_ID;
  unCanId.st_can_Id_.msg_type_ = static_cast<unsigned char>(EMsgType::eVendorInfo);
  unCanId.st_can_Id_.msg_id_ = 0x00;

  CanfdFrame vendorInfoReq{};
  vendorInfoReq.can_id_ = unCanId.ui_can_id_;
  vendorInfoReq.len_ = CANFD_MAX_DATA_LENGTH;

  try {
    CanfdFrame rep = canfd_device_->SendRequestSynch(vendorInfoReq);

    VendorInfo info;
    info.productModel = std::string(reinterpret_cast<char*>(rep.data_), 16);
    info.productSeqNum = std::string(reinterpret_cast<char*>(rep.data_) + 16, 14);

    // handware version：4bytes [major_, minor_, patch_, res_]
    memcpy(&info.hardwareVersion, rep.data_ + 30, sizeof(Version));

    // software version：4bytes [major_, minor_, patch_, res_]
    memcpy(&info.softwareVersion, rep.data_ + 34, sizeof(Version));

    memcpy(&info.voltage, rep.data_ + 38, sizeof(info.voltage));
    memcpy(&info.dof, rep.data_ + 40, sizeof(info.dof));

    return info;
  } catch (std::exception& ex) {
    std::cerr << ex.what() << std::endl;
    return {};
  }
}

DeviceInfo AgibotHandO12::GetDeviceInfo() {
  UnCanId unCanId{};
  unCanId.st_can_Id_.device_id_ = device_id_;
  unCanId.st_can_Id_.rw_flag_ = CANID_READ_FLAG;
  unCanId.st_can_Id_.product_id_ = CANID_PRODUCT_ID;
  unCanId.st_can_Id_.msg_type_ = static_cast<unsigned char>(EMsgType::eDeviceInfo);
  unCanId.st_can_Id_.msg_id_ = 0x00;

  CanfdFrame vendorInfoReq{};
  vendorInfoReq.can_id_ = unCanId.ui_can_id_;
  vendorInfoReq.len_ = CANFD_MAX_DATA_LENGTH;

  try {
    CanfdFrame rep = canfd_device_->SendRequestSynch(vendorInfoReq);

    DeviceInfo deviceInfo;
    deviceInfo.deviceId = rep.data_[0];
    memcpy(&deviceInfo.commuParams, rep.data_ + 1, sizeof(CommuParams));

    return deviceInfo;
  } catch (std::exception& ex) {
    std::cerr << ex.what() << std::endl;
    return DeviceInfo{};
  }
}

void AgibotHandO12::SetDeviceId(unsigned char device_id) {
  UnCanId unCanId{};
  unCanId.st_can_Id_.device_id_ = device_id_;
  unCanId.st_can_Id_.rw_flag_ = CANID_WRITE_FLAG;
  unCanId.st_can_Id_.product_id_ = CANID_PRODUCT_ID;
  unCanId.st_can_Id_.msg_type_ = static_cast<unsigned char>(EMsgType::eDeviceInfo);
  unCanId.st_can_Id_.msg_id_ = 0x01;

  CanfdFrame deviceIdReq{};
  deviceIdReq.can_id_ = unCanId.ui_can_id_;
  deviceIdReq.len_ = CANFD_MAX_DATA_LENGTH;
  memcpy(&deviceIdReq.data_, &device_id, sizeof(device_id));
  try {
    CanfdFrame rep = canfd_device_->SendRequestSynch(deviceIdReq);
    unsigned char deviceIdRep{};
    memcpy(&deviceIdRep, rep.data_, sizeof(deviceIdRep));
    if (deviceIdRep == device_id) {
      device_id_ = device_id;
    }
  } catch (std::exception& ex) {
    std::cerr << ex.what() << std::endl;
  }
}

void AgibotHandO12::UpdateFirmware(std::string file_name) {
  std::ifstream file(file_name, std::ios::binary);
  if (!file.is_open()) {
    std::cerr << "[Error] Failed to read " << file_name << std::endl;
  }

  file.seekg(0, std::ios::end);
  std::streamoff fileSize = file.tellg();

  file.seekg(0, std::ios::beg);
  char data[fileSize]{};
  file.read(data, fileSize);

  int packageNum = fileSize % 2048 == 0 ? fileSize / 2048 : fileSize / 2048 + 1;
  // 每个包均为2K，不足的0xFF补齐，一包32帧canfd_frame，每次发一包（连续无应答发32帧），收到一包应答后再发下一包，
  int head = 0;
  for (int pIndex = 0; pIndex < packageNum; pIndex++) {
    std::array<char, 2048> packageData;
    packageData.fill(0xFF);
    if (pIndex == packageNum - 1) {
      std::copy(data + head, data + fileSize, packageData.begin());
    } else {
      std::copy_n(data + head, 2048, packageData.begin());
    }

    head += 2048;
    que_packages_.push(packageData);
  }

  CanfdFrame updateReq{};
  updateReq.can_id_ = 0x11F10101;
  updateReq.len_ = CANFD_MAX_DATA_LENGTH;
  // TODO 可以RPC的采用一来一回阻塞式接口
  try {
    OTAUpgradeReq upgradeReq{};
    upgradeReq.firmware_length_ = fileSize;
    upgradeReq.package_num_ = packageNum;
    memcpy(&updateReq.data_, &upgradeReq, sizeof(OTAUpgradeReq));
    CanfdFrame updateRep = canfd_device_->SendRequestSynch(updateReq);  // 不等待应答，直接发送一包共32帧，然后等待应答才发最后一帧
    // TODO 成功的话往下执行
    unsigned int upgradeRepValue{};
    memcpy(&upgradeRepValue, &updateRep.data_, 4);
  } catch (std::exception& ex) {
    std::cerr << ex.what() << std::endl;
  }
}

void AgibotHandO12::SendPackage() {
  std::array<char, 2048> packageData = que_packages_.front();

  CanfdFrame packageReq{};
  packageReq.can_id_ = 0x12F10101;
  packageReq.len_ = CANFD_MAX_DATA_LENGTH;
  try {
    canfd_device_->SendRequestWithoutReply(packageReq);  // 不等待应答，直接发送一包共32帧，然后等待应答才发最后一帧
  } catch (std::exception& ex) {
    std::cerr << ex.what() << std::endl;
  }

  for (int fIndex = 0; fIndex < 32; fIndex++) {
    CanfdFrame frame{};
    frame.can_id_ = 0x10F00101;
    frame.len_ = CANFD_MAX_DATA_LENGTH;
    memcpy(&frame.data_, packageData.data() + fIndex * CANFD_MAX_DATA_LENGTH, CANFD_MAX_DATA_LENGTH);
    try {
      canfd_device_->SendRequestWithoutReply(frame);
    } catch (std::exception& ex) {
      std::cerr << ex.what() << std::endl;
    }
  }
}

void AgibotHandO12::GetUpgradeResult() {
  CanfdFrame resultReq{};
  resultReq.can_id_ = 0x15F10101;
  resultReq.len_ = CANFD_MAX_DATA_LENGTH;
  for (int i = 0; i < 10; i++) {
    try {
      CanfdFrame resultRep = canfd_device_->SendRequestSynch(resultReq);
      // TODO 判别升级结果
      break;
    } catch (std::exception& ex) {
      std::cerr << ex.what() << "请求失败等待下一次重试" << std::endl;
    }
  }
}

bool AgibotHandO12::JudgeMsgMatch(unsigned int req_id, unsigned int rep_id) {
  if ((req_id & 0x1FFFFF7F) == (rep_id & 0x1FFFFF7F)) {
    return true;
  } else if ((req_id & 0x1FFF7F7F ^ 0x10F10000) == (rep_id & 0x1FFF7F7F ^ 0x00F20000)) {
    return true;
  } else {
    return false;
  }
}

unsigned int AgibotHandO12::GetMatchedRepId(unsigned int req_id) {
  if ((req_id & 0x00FF0000) == 0x00F10000) {
    return req_id & 0x0F00FFFF | 0x00F20000;
  } else {
    return req_id & 0xFFFFFF7F;
  }
}

void AgibotHandO12::ShowDataDetails(bool show) const {
  canfd_device_->ShowDataDetails(show);
}