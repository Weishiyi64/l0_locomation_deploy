#pragma once

#include <linux/can.h>
#include <linux/can/raw.h>
#include <pthread.h>
#include <sched.h>

#include <array>
#include <atomic>
#include <chrono>
#include <ctime>
#include <functional>
#include <iostream>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include "rclcpp/rclcpp.hpp"

namespace can {

// 兼容 Frame 结构体
struct Frame {
  uint32_t id;                  // CAN ID
  uint8_t dlc;                  // 数据长度 (0-8)
  std::array<uint8_t, 8> data;  // 数据字节
  bool is_extended;             // 是否为扩展帧
  bool is_rtr;                  // 是否为远程传输请求
  bool is_error;                // 是否为错误帧

  Frame() : id(0), dlc(0), is_extended(false), is_rtr(false), is_error(false) { data.fill(0); }
};

}  // namespace can

namespace socketcan_wrapper {

using FrameCallback = std::function<void(const can::Frame&)>;

// SocketCAN 配置参数
struct SocketCANConfig {
  bool set_realtime_priority;
  std::string thread_scheduling_method;
  int thread_priority;
  int socket_rcvbuf_size;
  bool enable_error_frame_filter;

  SocketCANConfig()
      : set_realtime_priority(false),
        thread_scheduling_method("SCHED_FIFO"),
        thread_priority(99),
        socket_rcvbuf_size(1024 * 1024),
        enable_error_frame_filter(true) {}
};

class SocketCANManager {
 public:
  SocketCANManager();
  ~SocketCANManager();

  // 初始化 SocketCAN 接口
  bool init(const std::string& device, bool loopback = false);

  // 发送 CAN 帧
  // 参数:
  //   can_id: CAN ID
  //   data: 数据指针（可为 nullptr，将填充 0）
  //   dlc: 数据长度（0-8）
  //   is_extended: 是否为扩展帧（默认 false 为标准帧）
  bool send(uint32_t can_id, const uint8_t* data, uint8_t dlc, bool is_extended = false);

  // 设置接收回调
  void setReceiveCallback(FrameCallback callback);

  // 关闭接口
  void shutdown();

  // 获取设备名称
  const std::string& getDevice() const { return device_; }

 private:
  int socket_fd_;                      // Socket 文件描述符
  std::string device_;                 // CAN 设备名称
  std::atomic<bool> running_;          // 接收线程运行标志
  std::thread receive_thread_;         // 接收线程
  FrameCallback receive_callback_;     // 用户回调函数
  mutable std::mutex callback_mutex_;  // 保护 receive_callback_ 的互斥锁
  SocketCANConfig config_;             // 配置参数
  std::chrono::steady_clock::time_point last_send_error_log_time_;

  // 接收线程主函数
  void receiveThreadFunc();

  // 将 Linux can_frame 转换为 can::Frame
  can::Frame convertFrame(const struct can_frame& kernel_frame);

  // 配置 socket 选项
  bool configureSocket(bool loopback);

  // 加载配置（使用默认值）
  bool loadConfig();

  // 获取调度策略名称
  inline std::string getSchedulerPolicyName(int policy) {
    switch (policy)
    {
      case SCHED_OTHER:
        return "SCHED_OTHER";
      case SCHED_FIFO:
        return "SCHED_FIFO";
      case SCHED_RR:
        return "SCHED_RR";
#ifdef SCHED_BATCH
      case SCHED_BATCH:
        return "SCHED_BATCH";
#endif
#ifdef SCHED_IDLE
      case SCHED_IDLE:
        return "SCHED_IDLE";
#endif
      default:
        return "UNKNOWN";
    }
  }

  // 获取调度策略
  inline int getSchedulerPolicy(const std::string& method) {
    if (method == "SCHED_OTHER") return SCHED_OTHER;
    if (method == "SCHED_FIFO") return SCHED_FIFO;
    if (method == "SCHED_RR") return SCHED_RR;
#ifdef SCHED_BATCH
    if (method == "SCHED_BATCH") return SCHED_BATCH;
#endif
#ifdef SCHED_IDLE
    if (method == "SCHED_IDLE") return SCHED_IDLE;
#endif
    return SCHED_OTHER;  // default fallback
  }

  // 打印线程调度信息
  inline void printThreadSchedulerInfo(pthread_t thread, const std::string& threadName) {
    int policy;
    struct sched_param param;

    if (pthread_getschedparam(thread, &policy, &param) == 0)
    {
      printf("\033[35m[SocketCAN] %s current scheduler: %s, priority: %d\033[0m\n", threadName.c_str(),
             getSchedulerPolicyName(policy).c_str(), param.sched_priority);

      if (policy == SCHED_RR)
      {
        struct timespec ts;
        if (sched_rr_get_interval(0, &ts) == 0)
        {
          printf("\033[35m[SocketCAN] %s RR time slice: %ld.%03ld ms\033[0m\n", threadName.c_str(),
                 ts.tv_sec * 1000 + ts.tv_nsec / 1000000, (ts.tv_nsec % 1000000) / 1000);
        }
      }
    }
  }
};

}  // namespace socketcan_wrapper
