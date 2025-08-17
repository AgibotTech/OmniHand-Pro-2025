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

AgibotHandO12::AgibotHandO12(unsigned char device_id)
    : device_id_(device_id) {
#ifdef ZLG_USBCANFD_SDK
  canfd_device_ = std::make_unique<ZlgUsbcanfdSDK>();
#endif

#ifdef SOCKET_CAN
  canfd_device_ = std::make_unique<CanBusDeviceSocketCan>();
#endif
  canfd_device_->SetCallback(std::bind(&AgibotHandO12::ProcessMsg, this, std::placeholders::_1));
  canfd_device_->SetCalcuMatchRepId(std::bind(&AgibotHandO12::GetMatchedRepId, this, std::placeholders::_1));
  canfd_device_->SetMsgMatchJudge(std::bind(&AgibotHandO12::JudgeMsgMatch, this, std::placeholders::_1, std::placeholders::_2));
}

AgibotHandO12::~AgibotHandO12() {
}

void AgibotHandO12::SetJointMotorPosi(unsigned char joint_motor_index, short posi) {
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
}

short AgibotHandO12::GetJointMotorPosi(unsigned char joint_motor_index) {
  UnCanId unCanId{};
  unCanId.st_can_Id_.device_id_ = device_id_;
  unCanId.st_can_Id_.rw_flag_ = CANID_READ_FLAG;
  unCanId.st_can_Id_.product_id_ = CANID_PRODUCT_ID;
  unCanId.st_can_Id_.msg_type_ = static_cast<unsigned char>(EMsgType::ePosiCtrl);
  unCanId.st_can_Id_.msg_id_ = joint_motor_index;

  short posi{};

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
}

void AgibotHandO12::SetAllJointMotorPosi(std::vector<short> vec_posi) {
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
  memcpy(posiReqFrame.data_, vec_posi.data(), vec_posi.size() * sizeof(short));
  try {
    CanfdFrame posiRepFrame = canfd_device_->SendRequestSynch(posiReqFrame);
  } catch (std::exception& ex) {
    std::cerr << ex.what() << std::endl;
  }
}

std::vector<short> AgibotHandO12::GetAllJointMotorPosi() {
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
    return std::vector<short>{(short*)posiRepFrame.data_, (short*)(posiRepFrame.data_ + posiRepFrame.len_)};
  } catch (std::exception& ex) {
    std::cerr << ex.what() << std::endl;
    return {};
  }
}

void AgibotHandO12::SetJointMotorTorque(unsigned char joint_motor_index, short torque) {
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
}

short AgibotHandO12::GetJointMotorTorque(unsigned char joint_motor_index) {
  UnCanId unCanId{};
  unCanId.st_can_Id_.device_id_ = device_id_;
  unCanId.st_can_Id_.rw_flag_ = CANID_READ_FLAG;
  unCanId.st_can_Id_.product_id_ = CANID_PRODUCT_ID;
  unCanId.st_can_Id_.msg_type_ = static_cast<unsigned char>(EMsgType::eTorqueCtrl);
  unCanId.st_can_Id_.msg_id_ = joint_motor_index;

  short torque{};

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
}

void AgibotHandO12::SetAllJointMotorTorque(std::vector<short> vec_torque) {
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
  memcpy(torqueReqFrame.data_, vec_torque.data(), vec_torque.size() * sizeof(short));
  try {
    CanfdFrame torqueRepFrame = canfd_device_->SendRequestSynch(torqueReqFrame);
  } catch (std::exception& ex) {
    std::cerr << ex.what() << std::endl;
  }
}

std::vector<short> AgibotHandO12::GetAllJointMotorTorque() {
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
    return std::vector<short>{(short*)torqueRepFrame.data_, (short*)(torqueRepFrame.data_ + torqueRepFrame.len_)};
  } catch (std::exception& ex) {
    std::cerr << ex.what() << std::endl;
    return {};
  }
}

void AgibotHandO12::SetJointMotorVelo(unsigned char joint_motor_index, short velo) {
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
}

short AgibotHandO12::GetJointMotorVelo(unsigned char joint_motor_index) {
  UnCanId unCanId{};
  unCanId.st_can_Id_.device_id_ = device_id_;
  unCanId.st_can_Id_.rw_flag_ = CANID_READ_FLAG;
  unCanId.st_can_Id_.product_id_ = CANID_PRODUCT_ID;
  unCanId.st_can_Id_.msg_type_ = static_cast<unsigned char>(EMsgType::eVeloCtrl);
  unCanId.st_can_Id_.msg_id_ = joint_motor_index;

  short velo{};

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
}

void AgibotHandO12::SetAllJointMotorVelo(std::vector<short> vec_velo) {
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
  memcpy(veloReqFrame.data_, vec_velo.data(), vec_velo.size() * sizeof(short));
  try {
    CanfdFrame veloRepFrame = canfd_device_->SendRequestSynch(veloReqFrame);
  } catch (std::exception& ex) {
    std::cerr << ex.what() << std::endl;
  }
}

std::vector<short> AgibotHandO12::GetAllJointMotorVelo() {
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
    return std::vector<short>{(short*)veloRepFrame.data_, (short*)(veloRepFrame.data_ + veloRepFrame.len_)};
  } catch (std::exception& ex) {
    std::cerr << ex.what() << std::endl;
    return {};
  }
}

TouchSensorData AgibotHandO12::GetTouchSensorData(EFinger eFinger) {
  TouchSensorData touchSensorData{};

  UnCanId unCanId{};
  unCanId.st_can_Id_.device_id_ = device_id_;
  unCanId.st_can_Id_.rw_flag_ = CANID_READ_FLAG;
  unCanId.st_can_Id_.product_id_ = CANID_PRODUCT_ID;
  unCanId.st_can_Id_.msg_type_ = static_cast<unsigned char>(EMsgType::eTouchSensor);
  unCanId.st_can_Id_.msg_id_ = static_cast<unsigned char>(eFinger);

  CanfdFrame touchSensorDataReq{};
  touchSensorDataReq.can_id_ = unCanId.ui_can_id_;
  touchSensorDataReq.len_ = CANFD_MAX_DATA_LENGTH;
  try {
    CanfdFrame touchSensorDataRep = canfd_device_->SendRequestSynch(touchSensorDataReq);
    memcpy(&touchSensorData, touchSensorDataRep.data_, sizeof(touchSensorData));
  } catch (std::exception& ex) {
    std::cerr << ex.what() << std::endl;
  }

  return touchSensorData;
}

void AgibotHandO12::SetControlMode(unsigned char joint_motor_index, EControlMode mode) {
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
}

EControlMode AgibotHandO12::GetControlMode(unsigned char joint_motor_index) {
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

void AgibotHandO12::SetCurrentThreshold(unsigned char joint_motor_index, short current_threshold) {
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
}

short AgibotHandO12::GetCurrentThreshold(unsigned char joint_motor_index) {
  UnCanId unCanId{};
  unCanId.st_can_Id_.device_id_ = device_id_;
  unCanId.st_can_Id_.rw_flag_ = CANID_READ_FLAG;
  unCanId.st_can_Id_.product_id_ = CANID_PRODUCT_ID;
  unCanId.st_can_Id_.msg_type_ = static_cast<unsigned char>(EMsgType::eCurrentThreshold);
  unCanId.st_can_Id_.msg_id_ = joint_motor_index;

  short currentThreshold{};

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
}

void AgibotHandO12::SetAllCurrentThreshold(std::vector<short> vec_current_threshold) {
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
  memcpy(currentThreshReq.data_, vec_current_threshold.data(), vec_current_threshold.size() * sizeof(short));
  try {
    CanfdFrame currentThreshRep = canfd_device_->SendRequestSynch(currentThreshReq);
  } catch (std::exception& ex) {
    std::cerr << ex.what() << std::endl;
  }
}

std::vector<short> AgibotHandO12::GetAllCurrentThreshold() {
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
    return std::vector<short>{(short*)currentThreshRep.data_, (short*)(currentThreshRep.data_ + currentThreshRep.len_)};
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
        memcpy(head, &mixCtrl.tgt_posi_.value(), sizeof(short));
        head += sizeof(short);
        memcpy(head, &mixCtrl.tgt_torque_.value(), sizeof(short));
        head += sizeof(short);
      } else if (ctrlMode == EControlMode::eVeloTorque) {
        memcpy(head, &mixCtrl, sizeof(unsigned char));
        head += sizeof(unsigned char);
        memcpy(head, &mixCtrl.tgt_velo_.value(), sizeof(short));
        head += sizeof(short);
        memcpy(head, &mixCtrl.tgt_torque_.value(), sizeof(short));
        head += sizeof(short);
      } else if (ctrlMode == EControlMode::ePosiVeloTorque) {
        memcpy(head, &mixCtrl, sizeof(unsigned char));
        head += sizeof(unsigned char);
        memcpy(head, &mixCtrl.tgt_posi_.value(), sizeof(short));
        head += sizeof(short);
        memcpy(head, &mixCtrl.tgt_velo_.value(), sizeof(short));
        head += sizeof(short);
        memcpy(head, &mixCtrl.tgt_torque_.value(), sizeof(short));
        head += sizeof(short);
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

void AgibotHandO12::SetErrorReportPeriod(unsigned char joint_motor_index, unsigned short period) {
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
}

void AgibotHandO12::SetAllErrorReportPeriod(std::vector<unsigned short> vec_period) {
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
  memcpy(errReportReq.data_, vec_period.data(), sizeof(unsigned short) * vec_period.size());
  try {
    CanfdFrame errReportRep = canfd_device_->SendRequestSynch(errReportReq);
  } catch (std::exception& ex) {
    std::cerr << ex.what() << std::endl;
  }
}

unsigned short AgibotHandO12::GetTemperatureReport(unsigned char joint_motor_index) {
  if (joint_motor_index > 0 && joint_motor_index <= 12) {
    std::lock_guard<std::mutex> lockGuard(mutex_temper_report_);
    return vec_temper_report_[joint_motor_index];
  } else {
    return 0;
  }
}

std::vector<unsigned short> AgibotHandO12::GetAllTemperatureReport() {
  std::lock_guard<std::mutex> lockGuard(mutex_temper_report_);
  return vec_temper_report_;
}

void AgibotHandO12::SetTemperReportPeriod(unsigned char joint_motor_index, unsigned short period) {
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
}

void AgibotHandO12::SetAllTemperReportPeriod(std::vector<unsigned short> vec_period) {
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
    memcpy(temperReportReq.data_ + i * sizeof(unsigned short), &vec_period[i], sizeof(unsigned short));
  }
  // memcpy(&temperReportReq.data_, vec_period.data(), vec_period.size() * sizeof(unsigned short));
  try {
    canfd_device_->SendRequestWithoutReply(temperReportReq);
  } catch (std::exception& ex) {
    std::cerr << ex.what() << std::endl;
  }
}

short AgibotHandO12::GetCurrentReport(unsigned char joint_motor_index) {
  if (joint_motor_index > 0 && joint_motor_index <= 12) {
    std::lock_guard<std::mutex> lockGuard(mutex_current_report_);
    return vec_current_report_[joint_motor_index];
  } else {
    return 0;
  }
}

std::vector<unsigned short> AgibotHandO12::GetAllCurrentReport() {
  std::lock_guard<std::mutex> lockGuard(mutex_current_report_);
  return vec_current_report_;
}

void AgibotHandO12::SetCurrentReportPeriod(unsigned char joint_motor_index, unsigned short period) {
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
}

void AgibotHandO12::SetAllCurrentReportPeriod(std::vector<unsigned short> vec_period) {
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
    memcpy(currentReportReq.data_ + i * sizeof(unsigned short), &vec_period[i], sizeof(unsigned short));
  }
  // memcpy(&currentReportReq.data_, vec_period.data(), vec_period.size() * sizeof(unsigned short));
  try {
    canfd_device_->SendRequestWithoutReply(currentReportReq);
  } catch (std::exception& ex) {
    std::cerr << ex.what() << std::endl;
  }
}

void AgibotHandO12::ProcessMsg(CanfdFrame frame) {
  if ((frame.can_id_ & 0xFFFF0000) == 0x00210000) {
    // unsigned char* head = frame.data_;
    // std::vector<unsigned short> vec_temper_report{};
    //
    // for (int i = 0;i < DEGREE_OF_FREEDOM;i++) {
    //   unsigned short temperReport{};
    //   memcpy(&temperReport, &frame.data_[i*sizeof(temperReport)], sizeof(temperReport));
    //   vec_temper_report.push_back(temperReport);
    // }

    std::vector<unsigned short> vec_temper_report{reinterpret_cast<unsigned short*>(frame.data_),
                                                  reinterpret_cast<unsigned short*>(frame.data_ + frame.len_)};
    {
      std::lock_guard<std::mutex> lockGuard(mutex_temper_report_);
      vec_temper_report_ = vec_temper_report;
    }
  } else if ((frame.can_id_ & 0xFFFF0000) == 0x00220000) {
    // std::vector<unsigned short> vec_current_report{};
    //
    // for (int i = 0;i < DEGREE_OF_FREEDOM;i++) {
    //   unsigned short currentReport{};
    //   memcpy(&currentReport, &frame.data_[i*sizeof(currentReport)], sizeof(currentReport));
    //   vec_current_report.push_back(currentReport);
    // }

    std::vector<unsigned short> vec_current_report{reinterpret_cast<unsigned short*>(frame.data_),
                                                   reinterpret_cast<unsigned short*>(frame.data_ + frame.len_)};
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

std::string AgibotHandO12::GetVendorInfo() {
  UnCanId unCanId{};
  unCanId.st_can_Id_.device_id_ = device_id_;
  unCanId.st_can_Id_.rw_flag_ = CANID_READ_FLAG;
  unCanId.st_can_Id_.product_id_ = CANID_PRODUCT_ID;
  unCanId.st_can_Id_.msg_type_ = static_cast<unsigned char>(EMsgType::eVendorInfo);
  unCanId.st_can_Id_.msg_id_ = 0x00;

  std::string productModel;
  std::string productSeqNum;
  Version hardwareVersion{};
  Version softwareVersion{};
  short voltage;
  unsigned char dof;

  CanfdFrame vendorInfoReq{};
  vendorInfoReq.can_id_ = unCanId.ui_can_id_;
  vendorInfoReq.len_ = CANFD_MAX_DATA_LENGTH;
  try {
    CanfdFrame rep = canfd_device_->SendRequestSynch(vendorInfoReq);
    productModel = std::string((char*)rep.data_, 16);
    productSeqNum = std::string((char*)rep.data_ + 16, 14);
    memcpy(&hardwareVersion, rep.data_ + 30, sizeof(hardwareVersion));
    memcpy(&softwareVersion, rep.data_ + 34, sizeof(softwareVersion));
    memcpy(&voltage, rep.data_ + 38, sizeof(voltage));
    memcpy(&dof, rep.data_ + 40, sizeof(dof));

    std::stringstream sstream;
    sstream << "产品型号：" << productModel << " 产品序列号：" << productSeqNum
            << "\n硬件版本：" << (unsigned int)hardwareVersion.major_ << "." << (unsigned int)hardwareVersion.minor_ << "." << (unsigned int)hardwareVersion.patch_
            << " 软件版本：" << (unsigned int)softwareVersion.major_ << "." << (unsigned int)softwareVersion.minor_ << "." << (unsigned int)softwareVersion.patch_
            << "\n供电电压：" << voltage << "mV"
            << " 主动自由度：" << (unsigned int)dof << std::endl;
    return sstream.str();
  } catch (std::exception& ex) {
    std::cerr << ex.what() << std::endl;
    return {};
  }
}

std::string AgibotHandO12::GetDeviceInfo() {
  UnCanId unCanId{};
  unCanId.st_can_Id_.device_id_ = device_id_;
  unCanId.st_can_Id_.rw_flag_ = CANID_READ_FLAG;
  unCanId.st_can_Id_.product_id_ = CANID_PRODUCT_ID;
  unCanId.st_can_Id_.msg_type_ = static_cast<unsigned char>(EMsgType::eDeviceInfo);
  unCanId.st_can_Id_.msg_id_ = 0x00;

  unsigned char deviceId{};
  CommuParams commuParams{};

  std::vector<std::string> vecBitrate = {"125Kbps", "500Kbps", "1Mbps", "5Mbps"};
  std::vector<std::string> vecSamplePoint = {"75.0%", "80.0%", "87.5%"};

  CanfdFrame vendorInfoReq{};
  vendorInfoReq.can_id_ = unCanId.ui_can_id_;
  vendorInfoReq.len_ = CANFD_MAX_DATA_LENGTH;
  try {
    CanfdFrame rep = canfd_device_->SendRequestSynch(vendorInfoReq);
    deviceId = rep.data_[0];
    memcpy(&commuParams, rep.data_ + 1, sizeof(commuParams));

    std::stringstream sstream;
    sstream << "设备ID：" << (unsigned int)deviceId
            << "\n仲裁域波特率：" << vecBitrate[commuParams.bitrate_] << "."
            << "仲裁域采样点：" << vecSamplePoint[commuParams.sample_point_]
            << "\n数据域波特率：" << vecBitrate[commuParams.dbitrate_] << "."
            << "数据域采样点：" << vecSamplePoint[commuParams.dsample_point_]
            << std::endl;

    return sstream.str();
  } catch (std::exception& ex) {
    std::cerr << ex.what() << std::endl;
    return {};
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