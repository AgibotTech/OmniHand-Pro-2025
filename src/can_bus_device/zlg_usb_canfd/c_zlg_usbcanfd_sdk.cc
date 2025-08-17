// Copyright (c) 2025, Agibot Co., Ltd.
// OmniHand Pro 2025 SDK is licensed under Mulan PSL v2.

/**
 * @file c_zlg_usbcanfd_sdk.cpp
 * @brief
 * @author AgiUser
 * @date 25-7-28
 **/

#include "c_zlg_usbcanfd_sdk.h"

#include <cstring>
#include <iomanip>
#include <iostream>
#include <sstream>

#include "zcan.h"

typedef unsigned int UINT;
typedef unsigned char U8;
typedef unsigned int U32;
#define msleep(ms) usleep((ms)*1000)

#define DEVICE_TYPE_USBCANFD 33  // 设备类型
#define DEVICE_INDEX 0
#define CAN_MAX_CHANNELS 1  // 100mini的最大通道数量为1
#define CHANNEL_INDEX 0
#define RX_WAIT_TIME 100
#define RX_BUFF_SIZE 1000

ZlgUsbcanfdSDK::ZlgUsbcanfdSDK() {
  if (ZlgUsbcanfdSDK::OpenDevice() == -1) {
    return;
  }

  pthread_ = new std::thread(&ZlgUsbcanfdSDK::RecvFrame, this);
}

ZlgUsbcanfdSDK::~ZlgUsbcanfdSDK() {
  RequestInterrupt();

  if (pthread_ != nullptr) {
    pthread_->join();
    delete pthread_;
  }

  ZlgUsbcanfdSDK::CloseDevice();
}

int ZlgUsbcanfdSDK::OpenDevice() {
  /*打开设备*/
  if (VCI_OpenDevice(DEVICE_TYPE_USBCANFD, DEVICE_INDEX, 0)) {
    std::cout << "[INFO]: Open device usbcanfd successfully." << std::endl;
  } else {
    std::cout << "[ERROR]: Open device usbcanfd failed!" << std::endl;
    return -1;
  }

  /*初始化并启动通道*/
  for (int i = 0; i < CAN_MAX_CHANNELS; i++) {
    ZCAN_INIT init;  // TODO 初始化数据根据zcanpro的波特率计算器得出
    init.clk = 60000000;
    init.mode = 0;

    init.aset.tseg1 = 46;  // 仲裁域 1M
    init.aset.tseg2 = 11;
    init.aset.sjw = 3;
    init.aset.smp = 0;
    init.aset.brp = 0;

    init.dset.tseg1 = 7;  // 数据域 5M
    init.dset.tseg2 = 2;
    init.dset.sjw = 1;
    init.dset.smp = 0;
    init.dset.brp = 0;

    /*初始化通道*/
    if (VCI_InitCAN(DEVICE_TYPE_USBCANFD, DEVICE_INDEX, i, &init)) {
      std::cout << "[INFO]: Init canfd successfully." << std::endl;
    } else {
      std::cout << "[ERROR]: Init canfd failed!" << std::endl;
      return -1;
    }

    // TODO 终端电阻是否要设置
    /*终端电阻*/
    U32 on = 1;
    if (!VCI_SetReference(DEVICE_TYPE_USBCANFD, DEVICE_INDEX, i, CMD_CAN_TRES, &on)) {
      std::cout << "[ERROR: CMD_CAN_TRES failed!" << std::endl;
    }

    // TODO 合并接收是否要设置
    /*合并接收*/
    int isMerge = 0;
    if (i == 0) {
      if (!VCI_SetReference(DEVICE_TYPE_USBCANFD, DEVICE_INDEX, i, ZCAN_CMD_SET_CHNL_RECV_MERGE, &isMerge)) {
        std::cout << "[ERROR]: ZCAN_CMD_SET_CHNL_RECV_MERGE failed!" << std::endl;
      }
    }

    /*启动通道*/
    if (VCI_StartCAN(DEVICE_TYPE_USBCANFD, DEVICE_INDEX, i)) {
      std::cout << "[INFO]: Start canfd successfully." << std::endl;
    } else {
      std::cout << "[ERROR]: Start canfd failed!" << std::endl;
      return -1;
    }
  }

  return 0;
}

int ZlgUsbcanfdSDK::CloseDevice() {
  for (int i = 0; i < CAN_MAX_CHANNELS; i++) {
    if (VCI_ResetCAN(DEVICE_TYPE_USBCANFD, DEVICE_INDEX, i)) {
      std::cout << "[INFO]: Reset canfd successfully." << std::endl;
    } else {
      std::cout << "[ERROR]: Reset canfd failed!" << std::endl;
    }
  }

  if (VCI_CloseDevice(DEVICE_TYPE_USBCANFD, DEVICE_INDEX)) {
    std::cout << "[INFO]: Close canfd successfully." << std::endl;
  } else {
    std::cout << "[ERROR]: Close canfd failed!" << std::endl;
    return -1;
  }

  return 0;
}

void ZlgUsbcanfdSDK::RecvFrame() {
  ZCAN_FD_MSG canfd_data[RX_BUFF_SIZE];

  while (!IsInterruptRequested()) {
    memset(canfd_data, 0, sizeof(canfd_data));
    int recvCount = VCI_ReceiveFD(DEVICE_TYPE_USBCANFD, DEVICE_INDEX, 0, canfd_data, RX_BUFF_SIZE, RX_WAIT_TIME);
    for (int i = 0; i < recvCount; i++) {
      CanfdFrame rep{};
      rep.can_id_ = canfd_data[i].hdr.id;
      rep.len_ = canfd_data[i].hdr.len;
      memcpy(rep.data_, canfd_data[i].dat, rep.len_);

      if (show_data_details_.load()) {
        std::cout << "RCV: " << rep;
      }

      {
        bool matchedReq = false;
        std::lock_guard<std::mutex> lockGuard(mutex_reqRep_);
        for (auto reqId : uset_req_) {
          if (msg_match_judge_) {
            matchedReq = msg_match_judge_(reqId, rep.can_id_);
            if (matchedReq) {
              umap_rep_[rep.can_id_] = rep;
              break;
            }
          }
        }

        if (matchedReq) {
          continue;
        }
      }

      if (callback_) {
        callback_(rep);
      }
    }
  }
}

int ZlgUsbcanfdSDK::SendFrame(unsigned int id, unsigned char* data, unsigned char length) {
  ZCAN_FD_MSG canfd_msg;
  memset(&canfd_msg, 0, sizeof(canfd_msg));

  canfd_msg.hdr.inf.txm = 0;  // 0-正常发送
  canfd_msg.hdr.inf.fmt = 1;  // 0-CAN, 1-CANFD
  canfd_msg.hdr.inf.sdf = 0;  // 0-数据帧 CANFD只有数据帧!
  canfd_msg.hdr.inf.sef = 1;  // 0-标准帧, 1-扩展帧
  canfd_msg.hdr.inf.brs = 1;  // canfd 加速
  // canfd_msg.hdr.inf.echo  = 1;   // 发送回显

  canfd_msg.hdr.id = id;
  canfd_msg.hdr.chn = 0;
  canfd_msg.hdr.len = 64;  // 数据长度

  // TODO 注意非法长度
  memcpy(canfd_msg.dat, data, length);

  int sndRet = VCI_TransmitFD(DEVICE_TYPE_USBCANFD, DEVICE_INDEX, 0, &canfd_msg, 1);
  if (sndRet == 1) {
    return 0;
  } else {
    return -1;
  }
}
