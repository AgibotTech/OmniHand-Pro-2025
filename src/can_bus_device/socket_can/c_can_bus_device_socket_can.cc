// Copyright (c) 2025, Agibot Co., Ltd.
// OmniHand Pro 2025 SDK is licensed under Mulan PSL v2.

/**
 * @file c_can_bus_device_socket_can.cpp
 * @brief
 * @author agiuser
 * @date 25-7-31
 **/

#include "c_can_bus_device_socket_can.h"

#include <cstring>
#include <iostream>

#include <fcntl.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

CanBusDeviceSocketCan::CanBusDeviceSocketCan() {
  /*打开设备*/
  if (CanBusDeviceSocketCan::OpenDevice() == -1) {
    return;
  }

  /*启动接收帧线程*/
  pthread_ = new std::thread(&CanBusDeviceSocketCan::RecvFrame, this);
}

CanBusDeviceSocketCan::~CanBusDeviceSocketCan() {
  /*请求中止*/
  RequestInterrupt();

  /*等待线程释放*/
  if (pthread_ != nullptr) {
    pthread_->join();
    delete pthread_;
  }

  /*关闭设备*/
  CanBusDeviceSocketCan::CloseDevice();
}

int CanBusDeviceSocketCan::OpenDevice() {
  // TODO 发送和接收的CAN socket需要区分开吗？单CAN socket也可以双向通信
  /*创建socket*/
  fd_sock_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);

  /*设置为非阻塞*/
  int flags = fcntl(fd_sock_, F_GETFL, 0);
  fcntl(fd_sock_, F_SETFL, flags | O_NONBLOCK);

  /*指定can0设备，获取设备索引*/
  struct ifreq ifr {};
  strcpy(ifr.ifr_name, "can0");
  ioctl(fd_sock_, SIOCGIFINDEX, &ifr);

  /*地址*/
  struct sockaddr_can addr {};
  addr.can_family = AF_CAN;
  addr.can_ifindex = ifr.ifr_ifindex;

  /*使能CANFD*/
  int enableFD = 1;
  setsockopt(fd_sock_, SOL_CAN_RAW, CAN_RAW_FD_FRAMES, &enableFD, sizeof(enableFD));

  /*
  int rcvOwn = 1;
  setsockopt(m_fd_sock,SOL_CAN_RAW,CAN_RAW_RECV_OWN_MSGS,&rcvOwn,sizeof(rcvOwn));

  int rcvTimeout = 1;
  setsockopt(m_fd_sock,SOL_CAN_RAW,SO_RCVTIMEO,&rcvTimeout,sizeof(rcvTimeout));

  //设置一组接收过滤规则，received_can_id & can_mask = can_id & can_mask
  struct can_filter filters[3];
  filters[0].can_id = 0;
  filters[0].can_mask = CAN_EFF_MASK;   //扩展帧
  setsockopt(m_fd_sock,SOL_CAN_RAW,CAN_RAW_FILTER,&filters,sizeof(filters));
  */

  int bindRes = bind(fd_sock_, (struct sockaddr *)&addr, sizeof(addr));
  return bindRes;
}

int CanBusDeviceSocketCan::CloseDevice() {
  /*关闭套接字*/
  return close(fd_sock_);
}

void CanBusDeviceSocketCan::RecvFrame() {
  while (!IsInterruptRequested()) {
    canfd_frame frame{};
    int ret = read(fd_sock_, &frame, sizeof(frame));  // read接收阻塞式会导致线程无法释放
    if (ret > 0) {
      CanfdFrame rep{};
      rep.can_id_ = frame.can_id & CAN_EFF_MASK;
      rep.len_ = frame.len;
      memcpy(rep.data_, frame.data, rep.len_);

      if (show_data_details_.load()) {
        std::cout << "RCV: " << rep;
      }

      /*与已有等待中的请求相匹配时先保存下来，后续RPC操作进行匹配返回请求的应答结果，未匹配的视为主动上报信息，直接调用预先设定的回调函数进行处理*/
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

int CanBusDeviceSocketCan::SendFrame(unsigned int id, unsigned char *data, unsigned char length) {
  canfd_frame frame{};
  frame.can_id = id | CAN_EFF_FLAG;  // 实际发送id需要和CAN_EFF_FLAG进行或运算才能正确发送（candump可监控）
  frame.len = length;
  memcpy(frame.data, data, length);

  int sndRet = write(fd_sock_, &frame, sizeof(frame));
  if (sndRet > 0) {
    return 0;
  } else {
    return -1;
  }
}