// Copyright (c) 2025, Agibot Co., Ltd.
// OmniHand Pro 2025 SDK is licensed under Mulan PSL v2.

/**
 * @file c_can_bus_device.h
 * @brief CAN总线设备基类
 * @author WSJ
 * @date 25-7-27
 **/

#ifndef C_CAN_BUS_DEVICE_H
#define C_CAN_BUS_DEVICE_H

#include <atomic>
#include <functional>
#include <iomanip>
#include <mutex>
#include <thread>
#include <unordered_set>

#define CANFD_MAX_DATA_LENGTH 64  // CANFD的最大数据长度

/**
 * @brief 自定义规范化的CAN帧结构
 */
struct CanfdFrame {
  /**
   * @brief CAN ID
   */
  unsigned int can_id_{};

  /**
   * @brief 长度
   */
  unsigned char len_{};

  /**
   * @brief 数据
   */
  unsigned char data_[CANFD_MAX_DATA_LENGTH]{};

  friend std::ostream& operator<<(std::ostream& os, const CanfdFrame& frame) {
    os << "CANID: 0x" << std::hex << std::setw(8) << std::setfill('0') << (frame.can_id_ & 0x1FFFFFFF)
       << "\n\t\tDATA: 0x";
    for (int i = 0; i < frame.len_; i++) {
      os << std::hex << std::setw(2) << std::setfill('0')
         << static_cast<unsigned int>(frame.data_[i]);
    }
    os << std::endl;
    return os;
  }
};

/**
 * @brief CAN总线设备基类
 */
class CanBusDeviceBase {
 public:
  /**
   * @brief 构造函数
   */
  CanBusDeviceBase() = default;

  /**
   * @brief 析构函数
   */
  virtual ~CanBusDeviceBase() = default;

  /**
   * @brief 打开设备
   * @return 操作结果：0~成功;-1~失败
   */
  virtual int OpenDevice() = 0;

  /**
   * @brief 关闭设备
   * @return 操作结果：0~成功;-1~失败
   */
  virtual int CloseDevice() = 0;

  /**
   * @brief 请求中止
   */
  void RequestInterrupt() { is_interrupted_.store(true); }

  /**
   * @brief 是否被请求中止
   * @return : true~已被请求中止;false~未被请求中止
   */
  bool IsInterruptRequested() { return is_interrupted_.load(); }

  /**
   * @brief 设置接收处理回调函数
   * @param callback 接收处理回调函数
   */
  void SetCallback(std::function<void(CanfdFrame)> callback) { callback_ = callback; }

  /**
   * @brief 设置报文匹配判断函数
   * @param msg_matched_judge
   */
  void SetMsgMatchJudge(std::function<bool(unsigned int, unsigned int)> msg_matched_judge);

  /**
   * @brief 设置匹配应答Id计算函数
   * @param calcu_match_rep_id
   */
  void SetCalcuMatchRepId(std::function<unsigned int(unsigned int)> calcu_match_rep_id) {
    calcu_match_rep_id_ = calcu_match_rep_id;
  }

  /**
   * @brief 接收帧
   */
  virtual void RecvFrame() = 0;

  /**
   * @brief 发送帧
   * @param id CAN ID
   * @param data 实际报文数据
   * @param length 有效数据长度
   * @return 发送结果:0~成功;-1~失败
   */
  virtual int SendFrame(unsigned int id, unsigned char* data, unsigned char length) = 0;

  /**
   * @brief 发送请求
   * @details 失败时会抛出异常
   * @param req 请求帧
   * @return 应答帧
   */
  CanfdFrame SendRequestSynch(const CanfdFrame& req);

  /**
   * @brief 发送无应答请求
   * @details 失败时不抛出异常
   * @param req 请求帧
   * @return 发送结果:0~成功;-1~失败
   */
  int SendRequestWithoutReply(const CanfdFrame& req);

  /**
   * @brief 设置是否显示发送接收数据细节
   * @param show
   */
  void ShowDataDetails(bool show) {
    show_data_details_.store(show);
  }

 protected:
  /**
   * @brief 请求中止标志位
   */
  std::atomic<bool> is_interrupted_{false};

  /**
   * @brief 接收处理回调函数
   */
  std::function<void(CanfdFrame)> callback_{};

  /**
   * @brief 报文匹配判断函数
   */
  std::function<bool(unsigned int, unsigned int)> msg_match_judge_{};

  /**
   * @brief 匹配应答Id计算函数
   */
  std::function<unsigned int(unsigned int)> calcu_match_rep_id_{};

  /**
   * @brief 线程
   */
  std::thread* pthread_{nullptr};

  /**
   * @brief 请求应答
   */
  std::mutex mutex_reqRep_;
  std::unordered_set<unsigned int> uset_req_;
  std::unordered_map<unsigned int, CanfdFrame> umap_rep_;

  /**
   * @brief 是否显示发送接收数据细节
   */
  std::atomic<bool> show_data_details_{false};
};

#endif  // C_CAN_BUS_DEVICE_H
