#include <cerrno>
#include <chrono>
#include <cstring>
#include <fstream>
#include <filesystem>
#include <functional>
#include <stdexcept>
#include <string>
#include <cstdlib>
#include <csignal>
#include <fcntl.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <unistd.h>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/empty.hpp"
#include "AppVersion.h"

namespace
{
constexpr char kDefaultBroadcastName[] = "LUVRUMI";

std::string trim(const std::string & value)
{
  const auto first = value.find_first_not_of(" \t\r\n");
  if (first == std::string::npos) {
    return {};
  }
  const auto last = value.find_last_not_of(" \t\r\n");
  return value.substr(first, last - first + 1);
}

std::filesystem::path user_config_path()
{
  const char * home = std::getenv("HOME");
  if (home == nullptr || *home == '\0') {
    throw std::runtime_error("HOME is not set; cannot locate provisioning configuration");
  }
  return std::filesystem::path(home) / ".config/luvbot_provisioning/luvbot_provisioning.conf";
}

std::string read_broadcast_name()
{
  auto config_path = std::filesystem::path(
    ament_index_cpp::get_package_share_directory("luvbot_provisioning")) /
    "config/luvbot_provisioning.conf";
  try {
    std::error_code ec;
    const auto writable_path = user_config_path();
    if (std::filesystem::exists(writable_path, ec) && !ec) {
      config_path = writable_path;
    }
  } catch (const std::exception &) {
    // Use the installed configuration when no user config directory is available.
  }
  std::ifstream config(config_path);
  if (!config) {
    return kDefaultBroadcastName;
  }

  std::string line;
  while (std::getline(config, line)) {
    line = trim(line);
    if (line.empty() || line.front() == '#') {
      continue;
    }
    const auto separator = line.find('=');
    if (separator == std::string::npos) {
      continue;
    }
    if (trim(line.substr(0, separator)) == "broadcast_name") {
      auto name = trim(line.substr(separator + 1));
      return name.empty() ? kDefaultBroadcastName : name;
    }
  }
  return kDefaultBroadcastName;
}

void write_broadcast_name(const std::string & name)
{
  const auto path = user_config_path();
  std::filesystem::create_directories(path.parent_path());
  const auto temporary_path = path.string() + ".tmp";
  {
    std::ofstream config(temporary_path, std::ios::trunc);
    if (!config || !(config << "broadcast_name=" << name << '\n')) {
      throw std::runtime_error("Cannot write configuration: " + temporary_path);
    }
  }
  std::error_code ec;
  std::filesystem::rename(temporary_path, path, ec);
  if (ec) {
    std::filesystem::remove(temporary_path);
    throw std::runtime_error("Cannot replace configuration: " + ec.message());
  }
}
}  // namespace

class ProvisionOnce : public rclcpp::Node
{
public:
  ProvisionOnce()
  : Node("luvbot_provision_once")
  {
    configure_name_only_ = declare_parameter<bool>("configure_name_only", false);
    wait_for_trigger_ = declare_parameter<bool>("wait_for_trigger", false);
    device_sn_ = declare_parameter<std::string>("device_sn", "RUMI-TEST-007");
    if (configure_name_only_ && wait_for_trigger_) {
      throw std::runtime_error("configure_name_only and wait_for_trigger are mutually exclusive");
    }
    state_publisher_ = create_publisher<std_msgs::msg::String>(
      "/luvbot/provision_state", rclcpp::QoS(1).reliable().transient_local());
    if (wait_for_trigger_) {
      trigger_subscription_ = create_subscription<std_msgs::msg::Empty>(
        "/luvbot/provision_trigger", rclcpp::QoS(1).reliable().durability_volatile(),
        [this](std_msgs::msg::Empty::ConstSharedPtr) {
          if (child_pid_ > 0) {
            RCLCPP_WARN(get_logger(), "Provisioning already active; ignoring duplicate trigger");
            return;
          }
          start_session();
        });
    }
    auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local();
    publisher_ = create_publisher<std_msgs::msg::String>("/luvbot/provision_result", qos);
    if (configure_name_only_) {
      name_subscription_ = create_subscription<std_msgs::msg::String>(
        "/luvbot/set_bluetooth_name", 10,
        std::bind(&ProvisionOnce::on_name_requested, this, std::placeholders::_1));
    }
    reap_timer_ = create_wall_timer(
      std::chrono::milliseconds(250), std::bind(&ProvisionOnce::reap_child, this));
  }

  ~ProvisionOnce() override
  {
    stop_daemon();
  }

  void start()
  {
    if (configure_name_only_) {
      RCLCPP_INFO(get_logger(), "Waiting for a Bluetooth name; provisioning is disabled in this mode");
      return;
    }
    if (wait_for_trigger_) {
      publish_state("idle");
      RCLCPP_INFO(get_logger(), "Waiting for /luvbot/provision_trigger (std_msgs/Empty)");
      return;
    }
    start_session();
  }

private:
  void publish_state(const std::string & state)
  {
    std_msgs::msg::String message;
    message.data = state;
    state_publisher_->publish(message);
  }

  void finish_session(const std::string & state)
  {
    publish_state(state);
    if (!wait_for_trigger_) {
      rclcpp::shutdown();
    }
  }

  void start_session()
  {
    if (device_sn_.empty() || device_sn_.size() > 64 ||
        device_sn_.find_first_not_of("ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789_-") != std::string::npos) {
      RCLCPP_ERROR(get_logger(), "device_sn must contain 1-64 ASCII letters, digits, _ or -");
      finish_session("error");
      return;
    }
    result_published_ = false;
    daemon_started_ = false;
    daemon_output_.clear();
    std::string broadcast_name;
    try {
      broadcast_name = read_broadcast_name();
    } catch (const std::exception & error) {
      RCLCPP_ERROR(get_logger(), "%s", error.what());
      finish_session("error");
      return;
    }

    start_daemon(broadcast_name);
  }

  void start_daemon(const std::string & broadcast_name)
  {
    std::error_code ec;
    const auto self = std::filesystem::read_symlink("/proc/self/exe", ec);
    if (ec) {
      RCLCPP_ERROR(get_logger(), "Cannot resolve bridge executable: %s", ec.message().c_str());
      finish_session("error");
      return;
    }
    const auto binary = (self.parent_path() / "luvbot_provd").string();
    RCLCPP_INFO(get_logger(), "Starting %s with Bluetooth name '%s', device_sn='%s'",
      binary.c_str(), broadcast_name.c_str(), device_sn_.c_str());
    int output_pipe[2];
    if (pipe(output_pipe) < 0) {
      RCLCPP_ERROR(get_logger(), "pipe failed: %s", std::strerror(errno));
      finish_session("error");
      return;
    }
    const pid_t child = fork();
    if (child < 0) {
      RCLCPP_ERROR(get_logger(), "fork failed: %s", std::strerror(errno));
      close(output_pipe[0]);
      close(output_pipe[1]);
      finish_session("error");
      return;
    }
    if (child == 0) {
      if (setsid() < 0) {
        _exit(126);
      }
      close(output_pipe[0]);
      if (dup2(output_pipe[1], STDOUT_FILENO) < 0) {
        _exit(126);
      }
      close(output_pipe[1]);
      if (geteuid() == 0) {
        execl(binary.c_str(), binary.c_str(), "--name", broadcast_name.c_str(), "--sn", device_sn_.c_str(),
          static_cast<char *>(nullptr));
      } else {
        execlp("sudo", "sudo", "-n", "--", binary.c_str(), "--name", broadcast_name.c_str(), "--sn", device_sn_.c_str(),
          static_cast<char *>(nullptr));
      }
      _exit(127);
    }
    close(output_pipe[1]);
    child_pid_ = child;
    output_fd_ = output_pipe[0];
    const int flags = fcntl(output_fd_, F_GETFL, 0);
    if (flags < 0 || fcntl(output_fd_, F_SETFL, flags | O_NONBLOCK) < 0) {
      RCLCPP_ERROR(get_logger(), "Cannot make daemon output nonblocking: %s", std::strerror(errno));
      stop_daemon();
      finish_session("error");
      return;
    }
    publish_state("starting");
  }

  void stop_daemon()
  {
    if (child_pid_ <= 0) {
      close_output_pipe();
      return;
    }
    // sudo forwards SIGTERM to the privileged daemon, which stops its nmcli group.
    kill(child_pid_, SIGTERM);
    int status = 0;
    pid_t result = 0;
    for (int attempt = 0; attempt < 30; ++attempt) {
      result = waitpid(child_pid_, &status, WNOHANG);
      if (result == child_pid_ || (result < 0 && errno != EINTR)) {
        break;
      }
      usleep(100000);
    }
    if (result == 0 || (result < 0 && errno == EINTR)) {
      kill(child_pid_, SIGKILL);
      do {
        result = waitpid(child_pid_, &status, 0);
      } while (result < 0 && errno == EINTR);
    }
    if (result < 0) {
      RCLCPP_ERROR(get_logger(), "waitpid failed: %s", std::strerror(errno));
    }
    child_pid_ = -1;
    close_output_pipe();
  }

  void close_output_pipe()
  {
    if (output_fd_ >= 0) {
      close(output_fd_);
      output_fd_ = -1;
    }
  }

  void publish_success()
  {
    if (result_published_) {
      return;
    }
    std_msgs::msg::String message;
    message.data = "ok";
    publisher_->publish(message);
    result_published_ = true;
    publish_state("wifi_connected");
    RCLCPP_INFO(get_logger(), "Provisioning succeeded; published luvbot/provision_result=ok");
  }

  void handle_daemon_line(const std::string & line)
  {
    if (line == "[RESULT] ok") {
      publish_success();
      return;
    }
    if (line == "[PROV] luvbot provisioning daemon started") {
      daemon_started_ = true;
      publish_state("provisioning");
    } else if (line.rfind("[ADV] ", 0) == 0 && line.find(" advertising ON") != std::string::npos) {
      publish_state("advertising");
      RCLCPP_INFO(get_logger(), "BLE advertising is ready; waiting until WiFi connects (no idle timeout)");
    }
    if (!line.empty()) {
      RCLCPP_INFO(get_logger(), "%s", line.c_str());
    }
  }

  void read_daemon_output()
  {
    if (output_fd_ < 0) {
      return;
    }
    char buffer[512];
    while (true) {
      const auto count = read(output_fd_, buffer, sizeof(buffer));
      if (count > 0) {
        daemon_output_.append(buffer, static_cast<std::size_t>(count));
        continue;
      }
      if (count < 0 && errno == EINTR) {
        continue;
      }
      if (count == 0) {
        close_output_pipe();
      }
      break;
    }

    std::size_t newline;
    while ((newline = daemon_output_.find('\n')) != std::string::npos) {
      const auto line = daemon_output_.substr(0, newline);
      daemon_output_.erase(0, newline + 1);
      handle_daemon_line(line);
    }
  }

  void on_name_requested(const std_msgs::msg::String::SharedPtr message)
  {
    const auto name = trim(message->data);
    if (name.empty() || name.find_first_of("\r\n") != std::string::npos ||
        name.find('\0') != std::string::npos) {
      RCLCPP_WARN(get_logger(), "Ignoring empty or invalid Bluetooth name request");
      return;
    }
    try {
      write_broadcast_name(name);
    } catch (const std::exception & error) {
      RCLCPP_ERROR(get_logger(), "Failed to save Bluetooth name: %s", error.what());
      return;
    }
    RCLCPP_INFO(get_logger(), "Saved Bluetooth name '%s'", name.c_str());
    rclcpp::shutdown();
  }

  void reap_child()
  {
    read_daemon_output();
    if (child_pid_ <= 0) {
      return;
    }
    int status = 0;
    const auto result = waitpid(child_pid_, &status, WNOHANG);
    if (result == 0 || (result < 0 && errno == EINTR)) {
      return;
    }
    child_pid_ = -1;
    if (result < 0) {
      RCLCPP_ERROR(get_logger(), "waitpid failed: %s", std::strerror(errno));
      close_output_pipe();
      finish_session("error");
      return;
    }
    // The child can write its result between the first pipe read and waitpid.
    read_daemon_output();
    close_output_pipe();
    if (!daemon_output_.empty()) {
      handle_daemon_line(daemon_output_);
      daemon_output_.clear();
    }
    const int code = WIFEXITED(status) ? WEXITSTATUS(status) : 128 + WTERMSIG(status);
    if (code == 0) {
      publish_success();
      finish_session("wifi_connected");
    } else if (!daemon_started_ && !result_published_) {
      RCLCPP_ERROR(get_logger(),
        "Provisioning daemon could not start (exit=%d). Check the preceding error. "
        "If sudo requires a password, run: ros2 run luvbot_provisioning configure_provisioning_sudo.sh; "
        "then publish the trigger again. BLE advertising has not started.", code);
      finish_session("start_failed");
    } else {
      RCLCPP_WARN(get_logger(), "luvbot_provd exited with code %d", code);
      finish_session(result_published_ ? "wifi_connected" : "failed");
    }
  }

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_, state_publisher_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr trigger_subscription_;
  bool wait_for_trigger_{false};
  std::string device_sn_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr name_subscription_;
  rclcpp::TimerBase::SharedPtr reap_timer_;
  std::string daemon_output_;
  pid_t child_pid_{-1};
  int output_fd_{-1};
  bool result_published_{false};
  bool daemon_started_{false};
  bool configure_name_only_{false};
};

int main(int argc, char ** argv)
{
  l0::printAppVersion("luvbot_bridge");
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ProvisionOnce>();
  node->start();
  if (rclcpp::ok()) {
    rclcpp::spin(node);
  }
  rclcpp::shutdown();
  return 0;
}
