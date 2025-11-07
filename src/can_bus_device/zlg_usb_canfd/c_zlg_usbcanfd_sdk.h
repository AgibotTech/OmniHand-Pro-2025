// Copyright (c) 2025, Agibot Co., Ltd.
// OmniHand Pro 2025 SDK is licensed under Mulan PSL v2.

/**
 * @file c_zlg_usbcanfd_sdk.h
 * @brief
 * @author WSJ
 * @date 25-7-28
 **/

#ifndef C_ZLG_USBCANFD_SDK_H
#define C_ZLG_USBCANFD_SDK_H

#include "../c_can_bus_device.h"

/**
 * @brief 基于周立功usbcanfd的SDK的CAN Device类
 */
class ZlgUsbcanfdSDK : public CanBusDeviceBase {
 public:
  ZlgUsbcanfdSDK(uint8_t device_id);

  ~ZlgUsbcanfdSDK() override;

  int OpenDevice() override;

  int CloseDevice() override;

  void RecvFrame() override;

  int SendFrame(unsigned int id, unsigned char* data, unsigned char length) override;

  bool Init() override;

 private:
  uint8_t device_id_;
};

#endif  // C_ZLG_USBCANFD_SDK_H
