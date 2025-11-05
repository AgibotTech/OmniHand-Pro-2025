// Copyright (c) 2025, Agibot Co., Ltd.
// OmniHand Pro 2025 SDK is licensed under Mulan PSL v2.

/**
 * @file proto.h
 * @brief
 * @author agiuser
 * @date 25-8-7
 **/

#ifndef PROTO_H
#define PROTO_H

#include <optional>
#include <sstream>
#include <string>
#include <vector>
#include "export_symbols.h"

#pragma pack(push, 1)

/**
 * @brief 报文CanId
 */
struct AGIBOT_EXPORT CanId {
  unsigned char device_id_ : 7;
  unsigned char rw_flag_ : 1;
  unsigned char product_id_ : 7;
  unsigned char res1 : 1;
  unsigned char msg_type_;
  unsigned char msg_id_;
};

/**
 * @brief 联合体CanId
 */
union AGIBOT_EXPORT UnCanId {
  CanId st_can_Id_;
  unsigned int ui_can_id_;

  UnCanId() {
    ui_can_id_ = 0;
  }
};

enum class AGIBOT_EXPORT EFinger : unsigned char {
  eThumb = 0x01,
  eIndex = 0x02,
  eMiddle = 0x03,
  eRing = 0x04,
  eLittle = 0x05,
};

/**
 * @brief 触觉传感器数据
 */
struct AGIBOT_EXPORT TactileSensorData {
  unsigned char online_state_;          //: 1~传感器在线;0~传感器不在线
  unsigned short channel_value_[9];     // 各通道值
  unsigned short normal_force_;         // 法向力:(0-3000,0.1N)
  unsigned short tangent_force_;        // 切向力
  unsigned short tangent_force_angle_;  // 切向力角度，指尖向上为0度，顺时针旋转:(0~359)
  unsigned char capa_approach_[4];      // 自电容接近
};

enum class AGIBOT_EXPORT EHandType : unsigned char {
  eLeft = 0,
  eRight = 1,
  eUnknown = 10
};

enum class AGIBOT_EXPORT EControlMode : unsigned char {
  ePosi = 0,
  eVelo = 1,
  eTorque = 2,
  ePosiTorque = 3,
  eVeloTorque = 4,
  ePosiVeloTorque = 5,
  eUnknown = 10
};

struct AGIBOT_EXPORT JointMotorErrorReport {
  unsigned char stalled_ : 1;
  unsigned char overheat_ : 1;
  unsigned char over_current_ : 1;
  unsigned char motor_except_ : 1;
  unsigned char commu_except_ : 1;
  unsigned char res1_ : 3;
  unsigned char res2_;
};

enum class AGIBOT_EXPORT EMsgType : unsigned char {
  eVendorInfo = 0x01,
  eDeviceInfo = 0x02,
  eCurrentThreshold = 0x03,
  eTactileSensor = 0x05,
  eCtrlMode = 0x10,
  eTorqueCtrl = 0x11,
  eVeloCtrl = 0x12,
  ePosiCtrl = 0x13,
  eMixCtrl = 0x14,
  eErrorReport = 0x20,
  eTemperatureReport = 0x21,
  eCurrentReport = 0x22,
};

struct AGIBOT_EXPORT Version {
  unsigned char major_;
  unsigned char minor_;
  unsigned char patch_;
  unsigned char res_;
};

struct AGIBOT_EXPORT VendorInfo {
  std::string productModel;   // 产品型号
  std::string productSeqNum;  // 产品序列号
  Version hardwareVersion;    // 硬件版本
  Version softwareVersion;    // 软件版本
  int16_t voltage;            // 供电电压(mV)
  unsigned char dof;          // 主动自由度

  std::string toString() const {
    std::stringstream sstream;
    sstream << "Product Model: " << productModel
            << "\nSerial Number: " << productSeqNum
            << "\nHardware Version: " << static_cast<unsigned int>(hardwareVersion.major_)
            << "." << static_cast<unsigned int>(hardwareVersion.minor_)
            << "." << static_cast<unsigned int>(hardwareVersion.patch_)
            << "\nSoftware Version: " << static_cast<unsigned int>(softwareVersion.major_)
            << "." << static_cast<unsigned int>(softwareVersion.minor_)
            << "." << static_cast<unsigned int>(softwareVersion.patch_)
            << "\nSupply Voltage: " << voltage << "mV"
            << "\nActive Degrees of Freedom: " << static_cast<unsigned int>(dof);
    return sstream.str();
  }
};

struct AGIBOT_EXPORT CommuParams {
  unsigned char bitrate_;
  unsigned char sample_point_;
  unsigned char dbitrate_;
  unsigned char dsample_point_;
};

struct AGIBOT_EXPORT DeviceInfo {
  unsigned char deviceId;   // 设备ID
  CommuParams commuParams;  // 通信参数

  std::string toString() const {
    std::vector<std::string> vecBitrate = {"125Kbps", "500Kbps", "1Mbps", "5Mbps"};
    std::vector<std::string> vecSamplePoint = {"75.0%", "80.0%", "87.5%"};

    std::stringstream sstream;
    sstream << "Device ID: " << static_cast<unsigned int>(deviceId)
            << "\nArbitration Bitrate: " << vecBitrate[commuParams.bitrate_]
            << "\nArbitration Sample Point: " << vecSamplePoint[commuParams.sample_point_]
            << "\nData Bitrate: " << vecBitrate[commuParams.dbitrate_]
            << "\nData Sample Point: " << vecSamplePoint[commuParams.dsample_point_];
    return sstream.str();
  }
};

struct AGIBOT_EXPORT OTAUpgradeReq {
  unsigned int firmware_length_;
  unsigned short package_num_;
  unsigned short res1_;
  unsigned int res2_;
  unsigned int res3_;
};

struct AGIBOT_EXPORT OTAUpgradeRep {
  unsigned int result_;
};

struct AGIBOT_EXPORT OTATransmitReq {
  unsigned short package_index_;
  unsigned short crc_;
};

struct AGIBOT_EXPORT OTATransmitRep {
  unsigned int result_;
};

struct AGIBOT_EXPORT OTAFinishReq {
  unsigned int res_{};
};

struct AGIBOT_EXPORT OTAFinishRep {
  unsigned int result_{};
};

struct AGIBOT_EXPORT OTARestartReq {
  unsigned int delay_{};
};

struct AGIBOT_EXPORT OTARestartRep {
  unsigned int result_{};
};

struct AGIBOT_EXPORT OTAResultReq {
  unsigned int result_;
};

struct AGIBOT_EXPORT OTAResultRep {
  unsigned int result_;
};

struct AGIBOT_EXPORT OTAExitReq {
  unsigned int code_;
};

struct AGIBOT_EXPORT OTAExitRep {
  unsigned int result_;
};

#pragma pack(pop)

struct AGIBOT_EXPORT MixCtrl {
  unsigned char joint_index_ : 5 {};
  unsigned char ctrl_mode_ : 3 {};
  std::optional<short> tgt_posi_;
  std::optional<short> tgt_velo_;
  std::optional<short> tgt_torque_;
};

#endif  // PROTO_H
