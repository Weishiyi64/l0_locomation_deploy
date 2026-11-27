/*
 * Copyright (c) 2026 HPMicro
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "hpm_transport/transport.hpp"
#include "hpm_imu/imu.hpp"
#include "hpm_ultrasonic/ultrasonic.hpp"
#include "hpm_motor/motor.hpp"
#include "hpm_tactile/tactile.hpp"
#include "hpm_battery/battery.hpp"
#include "hpm_display/display.hpp"
#include "hpm_system/version.hpp"

#include "motor_can_log.hpp"
#include <ament_index_cpp/get_package_prefix.hpp>
#include <rclcpp/rclcpp.hpp>

#include <algorithm>
#include <array>
#include <atomic>
#include <cerrno>
#include <chrono>
#include <cstdio>
#include <cstdint>
#include <cstring>
#include <fcntl.h>
#include <functional>
#include <memory>
#include <sched.h>
#include <stdexcept>
#include <string>
#include <termios.h>
#include <thread>
#include <utility>
#include <unistd.h>
#include <vector>

class TactileUartReceiver
{
public:
  using Callback = std::function<void(const frameTransport::Frame &)>;

  TactileUartReceiver(std::string device, int baud, int reconnect_ms, Callback callback)
  : device_(std::move(device)), baud_(baud), reconnect_ms_(reconnect_ms), callback_(std::move(callback)),
    running_(true), thread_(&TactileUartReceiver::run, this)
  {
  }

  ~TactileUartReceiver()
  {
    stop();
  }

  void stop()
  {
    if (running_.exchange(false)) {
      if (thread_.joinable()) {
        thread_.join();
      }
    }
  }

private:
  static speed_t baud_constant(int baud)
  {
    switch (baud) {
      case 115200: return B115200;
      case 230400: return B230400;
      case 460800: return B460800;
#ifdef B921600
      case 921600: return B921600;
#endif
      default: return 0;
    }
  }

  int open_port() const
  {
    const speed_t speed = baud_constant(baud_);
    if (speed == 0) {
      return -1;
    }
    const int fd = ::open(device_.c_str(), O_RDONLY | O_NOCTTY);
    if (fd < 0) {
      return -1;
    }
    termios config{};
    if (tcgetattr(fd, &config) != 0) {
      ::close(fd);
      return -1;
    }
    cfmakeraw(&config);
    config.c_cflag |= CLOCAL | CREAD;
    config.c_cflag &= static_cast<tcflag_t>(~CSTOPB);
    config.c_cflag &= static_cast<tcflag_t>(~PARENB);
    config.c_cflag &= static_cast<tcflag_t>(~CRTSCTS);
    config.c_cc[VMIN] = 0;
    config.c_cc[VTIME] = 1;
    if (cfsetispeed(&config, speed) != 0 || cfsetospeed(&config, speed) != 0 ||
      tcsetattr(fd, TCSANOW, &config) != 0) {
      ::close(fd);
      return -1;
    }
    tcflush(fd, TCIFLUSH);
    return fd;
  }

  void parse()
  {
    constexpr std::array<uint8_t, 4> sync{{0x0AU, 0x4DU, 0x50U, 0x48U}};
    while (true) {
      const auto it = std::search(buffer_.begin(), buffer_.end(), sync.begin(), sync.end());
      if (it == buffer_.end()) {
        if (buffer_.size() > sync.size() - 1U) {
          buffer_.erase(buffer_.begin(), buffer_.end() - (sync.size() - 1U));
        }
        return;
      }
      buffer_.erase(buffer_.begin(), it);
      if (buffer_.size() < frameTransport::kFrameWireSize) {
        return;
      }
      frameTransport::Frame frame;
      if (frameTransport::frame_deserialize(
          frame, buffer_.data(), frameTransport::kFrameWireSize) == frameTransport::FrameStatus::Ok) {
        callback_(frame);
        buffer_.erase(buffer_.begin(), buffer_.begin() + frameTransport::kFrameWireSize);
      } else {
        buffer_.erase(buffer_.begin());
      }
    }
  }

  void run()
  {
    std::array<uint8_t, 256> read_buffer{};
    while (running_) {
      const int fd = open_port();
      if (fd < 0) {
        std::this_thread::sleep_for(std::chrono::milliseconds(reconnect_ms_));
        continue;
      }
      buffer_.clear();
      while (running_) {
        const ssize_t count = ::read(fd, read_buffer.data(), read_buffer.size());
        if (count > 0) {
          buffer_.insert(buffer_.end(), read_buffer.begin(), read_buffer.begin() + count);
          if (buffer_.size() > 4096U) {
            buffer_.erase(buffer_.begin(), buffer_.end() - 3);
          }
          parse();
        } else if (count < 0 && errno != EINTR) {
          break;
        }
      }
      ::close(fd);
      if (running_) {
        std::this_thread::sleep_for(std::chrono::milliseconds(reconnect_ms_));
      }
    }
  }

  std::string device_;
  int baud_;
  int reconnect_ms_;
  Callback callback_;
  std::atomic<bool> running_;
  std::thread thread_;
  std::vector<uint8_t> buffer_;
};

class HpmSystemNode : public rclcpp::Node
{
public:
  HpmSystemNode()
  : Node("hpm_system", rclcpp::NodeOptions().start_parameter_services(true))
  {
    rcl_interfaces::msg::ParameterDescriptor startup;
    startup.read_only = true;
    const bool realtime = declare_parameter<bool>("set_realtime_priority", false, startup);
    const auto policy = declare_parameter<std::string>("thread_scheduling_method", "SCHED_FIFO", startup);
    const auto priority = declare_parameter<int>("thread_priority", 99, startup);
    if (realtime) {
      const int expected = policy == "SCHED_FIFO" ? SCHED_FIFO : policy == "SCHED_RR" ? SCHED_RR : -1;
      sched_param actual{};
      if (expected < 0 || sched_getscheduler(0) != expected || sched_getparam(0, &actual) != 0 ||
          actual.sched_priority != priority) {
        throw std::runtime_error("Realtime scheduling mismatch: start via hpm_system.launch.py with rtprio permission");
      }
      RCLCPP_INFO(get_logger(), "Realtime scheduling active: %s priority %d", policy.c_str(), actual.sched_priority);
    }
    frameTransport::HpmFrameTransport::Config config;
    config.board_ip = declare_parameter<std::string>("board_ip", "192.168.1.10");
    config.port = static_cast<uint16_t>(declare_parameter<int>("port", 5001));
    config.enable_udp = declare_parameter<bool>("enable_udp", false);
    config.reconnect_ms = declare_parameter<int>("reconnect_ms", 1000);
    const bool launch_imu = declare_parameter<bool>("launch_imu", true);
    const bool launch_ultrasonic = declare_parameter<bool>("launch_ultrasonic", false);
    const bool launch_tactile = declare_parameter<bool>("launch_tactile", false);
    const bool launch_battery = declare_parameter<bool>("launch_battery", false);
    const bool enable_tactile_uart = declare_parameter<bool>("enable_tactile_uart", false);
    const std::string tactile_uart_device =
      declare_parameter<std::string>("tactile_uart_device", "/dev/ttyUSB0");
    const int tactile_uart_baud = declare_parameter<int>("tactile_uart_baud", 921600);
    const int tactile_uart_reconnect_ms =
      declare_parameter<int>("tactile_uart_reconnect_ms", 1000);

    transport_ = std::make_unique<frameTransport::HpmFrameTransport>(config);
    const bool launch_motor = declare_parameter<bool>("launch_motor", false);

    if (launch_imu) {
      imu_ = std::make_unique<hpmImu::HpmImu>(*this);
    }
    if (launch_ultrasonic) {
      ultrasonic_ = std::make_unique<hpmUltrasonic::HpmUltrasonic>(*this);
    }
    if (launch_tactile) {
      tactile_ = std::make_unique<hpmTactile::HpmTactile>(*this);
    }
    if (launch_motor) {
      motor_ = std::make_unique<hpmMotor::HpmMotor>(*this, *transport_);
    }
    if (declare_parameter<bool>("motor_log_enabled", false)) {
      try {
        auto dir = declare_parameter<std::string>("motor_log_dir", "");
        const auto shard = declare_parameter<int64_t>("motor_log_shard_bytes", 64LL << 20);
        const auto quota = declare_parameter<int64_t>("motor_log_total_bytes", 512LL * 1024 * 1024);
        if (shard <= 0 || quota < shard) throw std::invalid_argument("invalid motor log capacity");
        if (dir.empty()) {
          std::filesystem::path prefix = ament_index_cpp::get_package_prefix("hpm_system");
          // Colcon isolated prefixes end in install/hpm_system; merged prefixes end in install.
          if (prefix.filename() == "hpm_system") prefix = prefix.parent_path();
          dir = (prefix.parent_path() / "motor_log").string();
        }
        std::vector<hpmMotor::MotorDefinition> definitions;
        if (motor_) {
          definitions = motor_->definitions();
        } else {
          const auto path = declare_parameter<std::string>("motor_config_file", "");
          if (!path.empty()) definitions = hpmMotor::load_motor_config(path);
        }
        motor_log_ = std::make_unique<MotorCanLog>(dir, shard, quota,
          std::move(definitions), get_logger());
        if (motor_) {
          motor_->set_topic_tx_callback([this](const auto &frame, uint32_t id, hpmMotor::TxTopic topic) {
            motor_log_->tx(frame, id, std::move(topic));
          });
        }
        transport_->set_motor_tx_callback([this](const auto &frame, uint32_t id) {
          motor_log_->tx(frame, id);
        });
        RCLCPP_INFO(get_logger(), "Motor CAN log directory: %s", dir.c_str());
      } catch (const std::exception &e) {
        RCLCPP_WARN(get_logger(), "Motor CAN recording disabled: %s", e.what());
      }
    }
    if (launch_battery) {
      battery_ = std::make_unique<hpmBattery::HpmBattery>(*this);
    }
    transport_->set_battery_snapshot_callback(
      [this](const frameTransport::BatterySnapshot &snapshot) {
        if (battery_) battery_->handle_battery_snapshot(snapshot);
      });
    transport_->set_rcvcallback([this](const frameTransport::Frame & frame) {
      if (motor_log_) motor_log_->rx(frame);
      handle_frame(frame);
    });
    transport_->set_motor_snapshot_callback(
      [this](const frameTransport::MotorSnapshot & snapshot) {
        if (motor_log_) {
          motor_log_->snapshot(snapshot, motor_.get());
        } else if (motor_) {
          motor_->handle_motor_snapshot(snapshot);
        }
      });
    const std::string board_ip = config.board_ip;
    const uint16_t port = config.port;
    const int reconnect_ms = config.reconnect_ms;
    transport_->set_frame_gap_callback([this]() {
      if (imu_) {
        imu_->reset();
      }
    });
    transport_->set_udp_state_callback(
      [this, board_ip, port, reconnect_ms](frameTransport::UdpLinkState state, int err) {
        const char * err_text = (err == 0) ? "no error" : std::strerror(err);
        switch (state) {
          case frameTransport::UdpLinkState::Disabled:
            break;
          case frameTransport::UdpLinkState::Registering:
            RCLCPP_INFO(get_logger(), "UDP connecting to %s:%u",
                        board_ip.c_str(), port);
            break;
          case frameTransport::UdpLinkState::Ready:
            RCLCPP_INFO(get_logger(), "UDP connected to %s:%u",
                        board_ip.c_str(), port);
            break;
          case frameTransport::UdpLinkState::Busy:
            RCLCPP_WARN(
              get_logger(),
              "UDP board %s:%u is busy; retry in %d ms",
              board_ip.c_str(), port, reconnect_ms);
            break;
          case frameTransport::UdpLinkState::Stale:
            RCLCPP_WARN(
              get_logger(),
              "UDP disconnected from %s:%u; registering a new session",
              board_ip.c_str(), port);
            break;
          case frameTransport::UdpLinkState::OtaBusy:
            RCLCPP_WARN(
              get_logger(),
              "UDP business transport paused while OTA is active on %s:%u",
              board_ip.c_str(), port);
            break;
          case frameTransport::UdpLinkState::SocketError:
            RCLCPP_WARN(get_logger(), "UDP socket error for %s:%u: %s; retry in %d ms",
                        board_ip.c_str(), port, err_text, reconnect_ms);
            break;
        }
      });
    if (enable_tactile_uart && tactile_) {
      tactile_uart_ = std::make_unique<TactileUartReceiver>(
        tactile_uart_device, tactile_uart_baud, tactile_uart_reconnect_ms,
        [this](const frameTransport::Frame & frame) { handle_frame(frame); });
      RCLCPP_INFO(
        get_logger(), "tactile UART enabled: %s @ %d baud",
        tactile_uart_device.c_str(), tactile_uart_baud);
    }
    if (!config.enable_udp) {
      RCLCPP_INFO(get_logger(), "UDP disabled; use enable_udp:=true to connect");
    }
    transport_->start();
    display_ = hpmDisplay::HpmDisplay::try_create(*this);
  }

  ~HpmSystemNode() override
  {
    display_.reset();
    if (tactile_uart_) {
      tactile_uart_->stop();
    }
    if (transport_) {
      transport_->stop();
    }
  }

private:
  void handle_frame(const frameTransport::Frame & frame)
  {
    if (imu_) {
      imu_->handle_frame(frame);
    }
    if (ultrasonic_) {
      ultrasonic_->handle_frame(frame);
    }
    if (tactile_) {
      tactile_->handle_frame(frame);
    }
    if (motor_) {
      motor_->handle_frame(frame);
    }
  }

  std::unique_ptr<MotorCanLog> motor_log_;
  std::unique_ptr<frameTransport::HpmFrameTransport> transport_;
  std::unique_ptr<hpmImu::HpmImu> imu_;
  std::unique_ptr<hpmUltrasonic::HpmUltrasonic> ultrasonic_;
  std::unique_ptr<hpmTactile::HpmTactile> tactile_;
  std::unique_ptr<hpmMotor::HpmMotor> motor_;
  std::unique_ptr<hpmBattery::HpmBattery> battery_;
  std::unique_ptr<TactileUartReceiver> tactile_uart_;
  std::unique_ptr<hpmDisplay::HpmDisplay> display_;
};

int main(int argc, char ** argv)
{
  std::printf("[VERSION] hpm_system_node version=%s\n", hpm_system::APP_VERSION);
  std::fflush(stdout);
  rclcpp::init(argc, argv);
  auto node = std::make_shared<HpmSystemNode>();
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 2U);
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
