// Copyright (c) 2025, Agibot Co., Ltd.
// OmniHand Pro 2025 SDK is licensed under Mulan PSL v2.

/**
 * @file c_can_bus_device_socket_can.h
 * @brief 基于socketCAN的CAN总线设备类
 * @author WSJ
 * @date 25-7-31
 **/

#ifndef C_CAN_BUS_DEVICE_SOCKET_CAN_H
#define C_CAN_BUS_DEVICE_SOCKET_CAN_H

/*需要root权限方可执行*/
#define CMD_CAN0_SET_PARAMS "sudo ip link set can0 type can bitrate 1000000 sample-point 0.8 dbitrate 5000000 dsample-point 0.8 fd on"
#define CMD_CAN0_UP "sudo ifconfig can0 up"
#define CMD_CAN0_DOWN "sudo ifconfig can0 down"

#include "../c_can_bus_device.h"

/**
 * @brief 基于socketCAN的CAN总线设备类
 */
class CanBusDeviceSocketCan : public CanBusDeviceBase {
 public:
  CanBusDeviceSocketCan();

  ~CanBusDeviceSocketCan() final;

  int OpenDevice() override;

  int CloseDevice() override;

  int SendFrame(unsigned int id, unsigned char* data, unsigned char length) override;

  void RecvFrame() override;

 private:
  /**
   * @brief 套接字
   */
  int fd_sock_{};
};

#endif  // C_CAN_BUS_DEVICE_SOCKET_CAN_H
