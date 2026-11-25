#include <mosquitto.h>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <fstream>
#include <iomanip>
#include <locale>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <sstream>
#include <stdexcept>
#include <string>

#include "luvbot.pb.h"

namespace {
using SteadyClock = std::chrono::steady_clock;

// TEMPORARY: delete this function when a real voltage source is connected.
double simulatedVoltage(double seconds) {
  constexpr double kPi = 3.14159265358979323846;
  return 48.0 + 2.0 * std::sin(2.0 * kPi * seconds / 20.0);
}

void requireSuccess(int result, const char *operation) {
  if (result != MOSQ_ERR_SUCCESS)
  {
    throw std::runtime_error(std::string(operation) + ": " + mosquitto_strerror(result));
  }
}

void requireReadable(const std::string &path) {
  if (!std::ifstream(path).good())
  {
    throw std::runtime_error("Cannot read certificate/key file: " + path);
  }
}

std::string sessionId() {
  std::ifstream input("/proc/sys/kernel/random/uuid");
  std::string value;
  if (!(input >> value))
  {
    throw std::runtime_error("Cannot read Linux session UUID; run this node on the Ubuntu robot");
  }
  return value;
}
}  // namespace

class VoltageMqttTestNode : public rclcpp::Node {
 public:
  VoltageMqttTestNode()
      : Node("voltage_mqtt_test"),
        device_id_(declare_parameter<std::string>("device_id", "LUV2026090001")),
        session_(sessionId()),
        start_(SteadyClock::now()) {
    const auto host = declare_parameter<std::string>("host", "mqtt.luvbotics.com");
    auto cert_dir = declare_parameter<std::string>("cert_dir", "");
    if (cert_dir.empty())
    {
      cert_dir = ament_index_cpp::get_package_share_directory("luvbot_cloud_bridge") + "/certs";
    }
    const auto ca_file = declare_parameter<std::string>("ca_file", "/etc/ssl/certs/ca-certificates.crt");
    if (device_id_.empty() ||
        device_id_.find_first_not_of("ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789_-") !=
            std::string::npos)
    {
      throw std::runtime_error("device_id must contain only letters, digits, _ or -");
    }
    const auto certificate = cert_dir + "/" + device_id_ + ".crt";
    const auto key = cert_dir + "/" + device_id_ + ".key";
    requireReadable(ca_file);
    requireReadable(certificate);
    requireReadable(key);
    event_topic_ = "luvbot/" + device_id_ + "/up/event";
    status_topic_ = "luvbot/" + device_id_ + "/up/status";
    command_topic_ = "luvbot/" + device_id_ + "/down/command";

    client_.reset(mosquitto_new(device_id_.c_str(), false, this));
    if (!client_)
    {
      throw std::runtime_error("mosquitto_new failed");
    }
    auto *client = client_.get();
    requireSuccess(mosquitto_int_option(client, MOSQ_OPT_PROTOCOL_VERSION, MQTT_PROTOCOL_V311), "MQTT 3.1.1");
    requireSuccess(mosquitto_tls_set(client, ca_file.c_str(), nullptr, certificate.c_str(), key.c_str(), nullptr),
                   "TLS configuration");
    requireSuccess(mosquitto_tls_insecure_set(client, false), "TLS hostname verification");
    requireSuccess(mosquitto_reconnect_delay_set(client, 1, 30, true), "Reconnect delay");
    requireSuccess(mosquitto_will_set(client, status_topic_.c_str(), 0, nullptr, 1, false), "Offline will");
    mosquitto_connect_callback_set(client, [](mosquitto *, void *object, int result) {
      auto *self = static_cast<VoltageMqttTestNode *>(object);
      self->connected_.store(result == 0);
      if (result == 0)
      {
        self->announce_online_.store(true);
        RCLCPP_INFO(self->get_logger(), "MQTT connected; publishing to %s", self->event_topic_.c_str());
        const int subscription_result =
            mosquitto_subscribe(self->client_.get(), nullptr, self->command_topic_.c_str(), 1);
        if (subscription_result != MOSQ_ERR_SUCCESS)
        {
          RCLCPP_ERROR(self->get_logger(), "Command subscription failed: %s", mosquitto_strerror(subscription_result));
        }
      }
      else
      {
        RCLCPP_ERROR(self->get_logger(), "MQTT CONNACK rejected: %s", mosquitto_connack_string(result));
      }
    });
    mosquitto_disconnect_callback_set(client, [](mosquitto *, void *object, int result) {
      auto *self = static_cast<VoltageMqttTestNode *>(object);
      self->connected_.store(false);
      if (result != 0)
      {
        RCLCPP_WARN(self->get_logger(), "MQTT disconnected: %s; reconnecting", mosquitto_strerror(result));
      }
    });
    mosquitto_publish_callback_set(client, [](mosquitto *, void *object, int) {
      auto *self = static_cast<VoltageMqttTestNode *>(object);
      self->pending_.fetch_sub(1);
      self->acknowledged_.fetch_add(1);
    });
    mosquitto_message_callback_set(client, [](mosquitto *, void *object, const mosquitto_message *message) {
      auto *self = static_cast<VoltageMqttTestNode *>(object);
      self->receiveCommand(message);
    });
    mosquitto_subscribe_callback_set(client, [](mosquitto *, void *object, int, int count, const int *qos) {
      auto *self = static_cast<VoltageMqttTestNode *>(object);
      if (count != 1 || !qos || qos[0] == 0x80)
      {
        RCLCPP_ERROR(self->get_logger(), "Broker rejected command subscription: %s", self->command_topic_.c_str());
        return;
      }
      RCLCPP_INFO(self->get_logger(), "Command subscription confirmed: %s QoS=%d (receive only)",
                  self->command_topic_.c_str(), qos[0]);
    });
    mosquitto_log_callback_set(client, [](mosquitto *, void *object, int level, const char *message) {
      if (level == MOSQ_LOG_ERR)
      {
        auto *self = static_cast<VoltageMqttTestNode *>(object);
        RCLCPP_ERROR(self->get_logger(), "MQTT: %s", message);
      }
    });

    // Create ROS resources before starting the thread, so construction failure cannot leave it running.
    timer_ = create_wall_timer(std::chrono::milliseconds(100), [this]() { tick(); });
    RCLCPP_WARN(get_logger(),
                "SIMULATION: 46-50 V / 20 s cycle / 10 Hz MQTT, event=test.battery_voltage. "
                "Stop other clients using device ID %s. No robot actions are executed.",
                device_id_.c_str());
    requireSuccess(mosquitto_connect_async(client, host.c_str(), 8883, 60), "Connect setup");
    requireSuccess(mosquitto_loop_start(client), "Network thread");
    loop_started_ = true;
  }

  ~VoltageMqttTestNode() override {
    timer_->cancel();
    // Best effort only: the broker's will covers unexpected connection loss.
    if (connected_.load())
    {
      publishStatus(false);
    }
    mosquitto_disconnect(client_.get());
    if (loop_started_)
    {
      mosquitto_loop_stop(client_.get(), true);
    }
  }

 private:
  void receiveCommand(const mosquitto_message *message) {
    if (!message || !message->topic || command_topic_ != message->topic)
    {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "Ignoring message outside this device's command topic");
      return;
    }
    // A volume command is small; bound parsing and log output for untrusted payloads.
    if (!message->payload || message->payloadlen <= 0 || message->payloadlen > 4096)
    {
      RCLCPP_WARN(get_logger(), "Rejected command: payload must contain 1-4096 Protobuf bytes");
      return;
    }
    luvbot::v1::Envelope frame;
    if (!frame.ParseFromArray(message->payload, message->payloadlen))
    {
      RCLCPP_WARN(get_logger(), "Rejected command: invalid Protobuf Envelope");
      return;
    }
    if (frame.device_id() != device_id_ || frame.msg_id().empty() || !frame.has_command())
    {
      RCLCPP_WARN(get_logger(), "Rejected command: device_id mismatch, missing msg_id or missing command");
      return;
    }
    const auto &command = frame.command();
    if (command.cmd() != "volume.set")
    {
      RCLCPP_INFO(get_logger(), "Ignoring unsupported command: %s", frame.ShortDebugString().c_str());
      return;
    }
    const std::string *volume = nullptr;
    for (const auto &param : command.params())
    {
      if (param.key() == "volume")
      {
        if (volume)
        {
          RCLCPP_WARN(get_logger(), "Rejected volume.set: duplicate volume parameter");
          return;
        }
        volume = &param.value();
      }
    }
    if (!volume || volume->empty() || volume->find_first_not_of("0123456789") != std::string::npos)
    {
      RCLCPP_WARN(get_logger(), "Rejected volume.set: volume must be a nonempty decimal integer string");
      return;
    }
    RCLCPP_INFO(get_logger(), "RECEIVE ONLY volume.set volume=%s; no volume change or business reply. frame={%s}",
                volume->c_str(), frame.ShortDebugString().c_str());
  }

  luvbot::v1::Envelope envelope() {
    luvbot::v1::Envelope result;
    const auto sequence = ++sequence_;
    result.set_device_id(device_id_);
    result.set_msg_id("voltage-test-" + session_ + "-" + std::to_string(sequence));
    result.set_seq(sequence);
    result.set_ts(
        std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch())
            .count());
    return result;
  }

  bool publish(const std::string &topic, const luvbot::v1::Envelope &frame) {
    // At most 20 QoS 1 messages awaiting ACK; no unbounded offline telemetry queue.
    if (!connected_.load() || pending_.load() >= 20)
    {
      ++skipped_;
      return false;
    }
    std::string bytes;
    if (!frame.SerializeToString(&bytes))
    {
      RCLCPP_ERROR(get_logger(), "Protobuf serialization failed");
      return false;
    }
    pending_.fetch_add(1);  // Reserve before publish: the network thread may ACK immediately.
    const int result = mosquitto_publish(client_.get(), nullptr, topic.c_str(), static_cast<int>(bytes.size()),
                                         bytes.data(), 1, false);
    if (result != MOSQ_ERR_SUCCESS)
    {
      pending_.fetch_sub(1);
      RCLCPP_ERROR(get_logger(), "Publish failed: %s", mosquitto_strerror(result));
      return false;
    }
    ++submitted_;
    return true;
  }

  bool publishStatus(bool online) {
    auto frame = envelope();
    auto *status = frame.mutable_status_report();
    status->set_online(online);
    if (!online)
    {
      status->set_reason("normal_shutdown");
    }
    return publish(status_topic_, frame);
  }

  void tick() {
    const auto now = SteadyClock::now();
    const double seconds = std::chrono::duration<double>(now - start_).count();
    const double voltage = simulatedVoltage(seconds);  // Replace only this source for real voltage.
    if (connected_.load())
    {
      if (announce_online_.exchange(false) || now >= next_heartbeat_)
      {
        if (publishStatus(true))
        {
          next_heartbeat_ = now + std::chrono::seconds(30);
        }
        else
        {
          announce_online_.store(true);
        }
      }
      auto frame = envelope();
      auto *event = frame.mutable_event_report();
      event->set_event("test.battery_voltage");
      event->set_level(1);
      std::ostringstream detail;
      detail.imbue(std::locale::classic());
      detail << std::fixed << std::setprecision(3) << "{\"voltage\":" << voltage
             << ",\"unit\":\"V\",\"simulated\":true}";
      event->set_detail(detail.str());
      publish(event_topic_, frame);
    }
    else
    {
      ++skipped_;
    }
    if (now >= next_log_)
    {
      RCLCPP_INFO(get_logger(),
                  "SIM voltage=%.3f V connected=%s queued_total=%llu broker_ack_total=%llu pending=%d skipped=%llu",
                  voltage, connected_.load() ? "yes" : "no", static_cast<unsigned long long>(submitted_),
                  static_cast<unsigned long long>(acknowledged_.load()), pending_.load(),
                  static_cast<unsigned long long>(skipped_));
      next_log_ = now + std::chrono::seconds(5);
    }
  }

  std::string device_id_, event_topic_, status_topic_, command_topic_, session_;
  SteadyClock::time_point start_, next_heartbeat_{}, next_log_{};
  std::atomic<bool> connected_{false}, announce_online_{false};
  std::atomic<int> pending_{0};
  std::atomic<std::uint64_t> acknowledged_{0};
  std::uint64_t submitted_{0}, skipped_{0};
  std::uint32_t sequence_{0};
  bool loop_started_{false};
  rclcpp::TimerBase::SharedPtr timer_;
  // The destructor stops the network thread before this client and callback state are destroyed.
  std::unique_ptr<mosquitto, decltype(&mosquitto_destroy)> client_{nullptr, &mosquitto_destroy};
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  const int result = mosquitto_lib_init();
  if (result != MOSQ_ERR_SUCCESS)
  {
    std::fprintf(stderr, "mosquitto_lib_init: %s\n", mosquitto_strerror(result));
    rclcpp::shutdown();
    return 1;
  }
  int exit_code = 0;
  try
  { rclcpp::spin(std::make_shared<VoltageMqttTestNode>()); } catch (const std::exception &error)
  {
    std::fprintf(stderr, "voltage_mqtt_test: %s\n", error.what());
    exit_code = 1;
  }
  mosquitto_lib_cleanup();
  rclcpp::shutdown();
  return exit_code;
}
