#include "socketcan_wrapper/socketcan_manager.h"

#include <fcntl.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cerrno>
#include <cstring>

// Simple logging macros that don't depend on ROS
#define LOG_INFO(fmt, ...)                                         \
  do                                                               \
  {                                                                \
    printf("\033[32m[SocketCAN] " fmt "\033[0m\n", ##__VA_ARGS__); \
  } while (0)

#define LOG_WARN(fmt, ...)                                         \
  do                                                               \
  {                                                                \
    printf("\033[33m[SocketCAN] " fmt "\033[0m\n", ##__VA_ARGS__); \
  } while (0)

#define LOG_ERROR(fmt, ...)                                                 \
  do                                                                        \
  {                                                                         \
    fprintf(stderr, "\033[31m[SocketCAN] " fmt "\033[0m\n", ##__VA_ARGS__); \
  } while (0)

namespace socketcan_wrapper {

SocketCANManager::SocketCANManager() : socket_fd_(-1), running_(false) {
  // 加载配置（使用默认值）
  loadConfig();
}

SocketCANManager::~SocketCANManager() { shutdown(); }

bool SocketCANManager::init(const std::string& device, bool loopback) {
  device_ = device;

  // 创建 SocketCAN socket
  socket_fd_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
  if (socket_fd_ < 0)
  {
    LOG_ERROR("Failed to create SocketCAN socket for device %s: %s", device_.c_str(), strerror(errno));
    return false;
  }

  // 配置 socket 选项
  if (!configureSocket(loopback))
  {
    close(socket_fd_);
    socket_fd_ = -1;
    return false;
  }

  // 获取网络接口索引
  struct ifreq ifr;
  std::strncpy(ifr.ifr_name, device_.c_str(), IFNAMSIZ - 1);
  ifr.ifr_name[IFNAMSIZ - 1] = '\0';

  if (ioctl(socket_fd_, SIOCGIFINDEX, &ifr) < 0)
  {
    LOG_ERROR("Failed to get interface index for %s: %s", device_.c_str(), strerror(errno));
    close(socket_fd_);
    socket_fd_ = -1;
    return false;
  }

  // 绑定 socket 到 CAN 接口
  struct sockaddr_can addr;
  std::memset(&addr, 0, sizeof(addr));
  addr.can_family = AF_CAN;
  addr.can_ifindex = ifr.ifr_ifindex;

  if (bind(socket_fd_, (struct sockaddr*)&addr, sizeof(addr)) < 0)
  {
    LOG_ERROR("Failed to bind socket to %s: %s", device_.c_str(), strerror(errno));
    close(socket_fd_);
    socket_fd_ = -1;
    return false;
  }

  // 启动接收线程
  running_ = true;
  receive_thread_ = std::thread(&SocketCANManager::receiveThreadFunc, this);

  LOG_INFO("SocketCAN initialized on device: %s (native implementation)", device_.c_str());
  return true;
}

bool SocketCANManager::configureSocket(bool loopback) {
  // 设置 loopback 模式
  int loopback_val = loopback ? 1 : 0;
  if (setsockopt(socket_fd_, SOL_CAN_RAW, CAN_RAW_LOOPBACK, &loopback_val, sizeof(loopback_val)) < 0)
  {
    LOG_WARN("Failed to set CAN_RAW_LOOPBACK: %s", strerror(errno));
  }

  // 启用错误帧接收
  if (config_.enable_error_frame_filter)
  {
    can_err_mask_t err_mask = CAN_ERR_MASK;
    if (setsockopt(socket_fd_, SOL_CAN_RAW, CAN_RAW_ERR_FILTER, &err_mask, sizeof(err_mask)) < 0)
    {
      LOG_WARN("Failed to set CAN_RAW_ERR_FILTER: %s", strerror(errno));
    }
  }

  // 增大接收缓冲区以提高实时性能
  int rcvbuf_size = config_.socket_rcvbuf_size;
  if (setsockopt(socket_fd_, SOL_SOCKET, SO_RCVBUF, &rcvbuf_size, sizeof(rcvbuf_size)) < 0)
  {
    LOG_WARN("Failed to set SO_RCVBUF: %s", strerror(errno));
  }

  return true;
}

bool SocketCANManager::loadConfig() {
  // 使用默认配置值（不依赖ROS参数）
  try
  {
    // 设置默认的实时调度配置
    config_.set_realtime_priority = true;
    config_.thread_scheduling_method = "SCHED_FIFO";
    config_.thread_priority = 99;

    // 设置默认的性能优化参数
    config_.socket_rcvbuf_size = 8388608;  // 8MB
    config_.enable_error_frame_filter = true;

    return true;
  } catch (const std::exception& e)
  {
    LOG_ERROR("Error in loadConfig: %s", e.what());
    return false;
  }
}

bool SocketCANManager::send(uint32_t can_id, const uint8_t* data, uint8_t dlc, bool is_extended) {
  if (socket_fd_ < 0)
  {
    LOG_ERROR("SocketCAN not initialized");
    return false;
  }

  if (dlc > 8)
  {
    LOG_ERROR("Invalid DLC: %d (must be 0-8)", dlc);
    return false;
  }

  struct can_frame frame;
  std::memset(&frame, 0, sizeof(frame));

  // 设置 CAN ID
  frame.can_id = can_id;
  if (is_extended)
  {
    frame.can_id |= CAN_EFF_FLAG;  // 扩展帧标志
  }

  // 设置数据长度
  frame.can_dlc = dlc;

  // 复制数据
  if (data != nullptr && dlc > 0)
  {
    std::memcpy(frame.data, data, dlc);
  }

  // 发送帧
  ssize_t nbytes = write(socket_fd_, &frame, sizeof(frame));
  if (nbytes != sizeof(frame))
  {
    const auto now = std::chrono::steady_clock::now();
    if (last_send_error_log_time_.time_since_epoch().count() == 0 ||
        now - last_send_error_log_time_ >= std::chrono::seconds(1))
    {
      LOG_WARN("Failed to send CAN frame on %s: %s", device_.c_str(), strerror(errno));
      last_send_error_log_time_ = now;
    }
    return false;
  }

  return true;
}

void SocketCANManager::setReceiveCallback(FrameCallback callback) {
  std::lock_guard<std::mutex> lock(callback_mutex_);
  receive_callback_ = std::move(callback);
}

void SocketCANManager::shutdown() {
  // 停止接收线程
  running_ = false;

  // 关闭 socket（这会解除 read() 阻塞）
  if (socket_fd_ >= 0)
  {
    ::shutdown(socket_fd_, SHUT_RDWR);
    close(socket_fd_);
    socket_fd_ = -1;
  }

  // 等待接收线程结束
  if (receive_thread_.joinable())
  {
    receive_thread_.join();
  }

  LOG_INFO("SocketCAN on %s shutdown complete", device_.c_str());
}

void SocketCANManager::receiveThreadFunc() {
  LOG_INFO("\033[35m[SocketCAN %s] Receive thread started\033[0m", device_.c_str());

  // 设置线程为实时调度
  if (config_.set_realtime_priority)
  {
    sched_param sched{};
    sched.sched_priority = config_.thread_priority;
    int policy = getSchedulerPolicy(config_.thread_scheduling_method);

    if (pthread_setschedparam(pthread_self(), policy, &sched) != 0)
    {
      LOG_WARN("\033[35m[SocketCAN %s] Failed to set realtime priority\033[0m", device_.c_str());
    }
    else
    {
      std::string thread_name = "[SocketCAN " + device_ + "]";
      printThreadSchedulerInfo(pthread_self(), thread_name);
    }
  }

  struct can_frame kernel_frame;

  while (running_)
  {
    // 阻塞读取 CAN 帧
    // 关键：每次循环都在栈上分配新的 kernel_frame，确保数据独立
    ssize_t nbytes = read(socket_fd_, &kernel_frame, sizeof(kernel_frame));

    if (nbytes < 0)
    {
      if (errno == EINTR || errno == EAGAIN)
      {
        continue;  // 被信号中断或非阻塞模式无数据
      }
      if (running_)
      {  // 只在运行时报错
        LOG_ERROR("Error reading from SocketCAN %s: %s", device_.c_str(), strerror(errno));
      }
      break;
    }

    if (nbytes != sizeof(kernel_frame))
    {
      LOG_WARN("Incomplete CAN frame read: %zd bytes", nbytes);
      continue;
    }

    // 转换为 can::Frame 格式
    // 关键：convertFrame 创建一个新的独立拷贝
    can::Frame user_frame = convertFrame(kernel_frame);

    // 调用用户回调
    // 关键：传递的是按值传递（拷贝），不是引用
    // 关键：拷贝callback后在锁外调用，避免持锁时间过长
    FrameCallback cb_copy;
    {
      std::lock_guard<std::mutex> lock(callback_mutex_);
      cb_copy = receive_callback_;
    }

    if (cb_copy)
    {
      cb_copy(user_frame);
    }
  }

  LOG_INFO("SocketCAN receive thread stopped for %s", device_.c_str());
}

can::Frame SocketCANManager::convertFrame(const struct can_frame& kernel_frame) {
  can::Frame frame;

  // 提取 CAN ID（去除标志位）
  if (kernel_frame.can_id & CAN_ERR_FLAG)
  {
    // 错误帧
    frame.id = kernel_frame.can_id & CAN_ERR_MASK;
    frame.is_error = true;
    frame.is_extended = false;
    frame.is_rtr = false;
  }
  else if (kernel_frame.can_id & CAN_EFF_FLAG)
  {
    // 扩展帧
    frame.id = kernel_frame.can_id & CAN_EFF_MASK;
    frame.is_extended = true;
    frame.is_error = false;
    frame.is_rtr = (kernel_frame.can_id & CAN_RTR_FLAG) != 0;
  }
  else
  {
    // 标准帧
    frame.id = kernel_frame.can_id & CAN_SFF_MASK;
    frame.is_extended = false;
    frame.is_error = false;
    frame.is_rtr = (kernel_frame.can_id & CAN_RTR_FLAG) != 0;
  }

  // 复制数据长度
  frame.dlc = kernel_frame.can_dlc;

  // 复制数据
  for (size_t i = 0; i < 8; ++i)
  {
    frame.data[i] = kernel_frame.data[i];
  }

  return frame;
}

}  // namespace socketcan_wrapper
