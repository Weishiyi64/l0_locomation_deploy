#include <mosquitto.h>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <fstream>
#include <deque>
#include <mutex>
#include <unordered_map>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <stdexcept>
#include <string>
#include <unordered_set>

#include "luvbot.pb.h"
#include "face_validation.hpp"
#include "face_store.hpp"
#include "voice_store.hpp"
#include "health_report.hpp"
#include "map_bridge.hpp"
#include <luvbot_control_interfaces/srv/set_custom_wake_word.hpp>
#include <luvbot_control_interfaces/srv/update_users.hpp>
#include <luvbot_control_interfaces/srv/map_control.hpp>
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"
#include "AppVersion.h"

namespace {
using SteadyClock = std::chrono::steady_clock;
constexpr int kMaxCommandPayloadBytes = 64 * 1024;

std::string logText(const std::string& value) {
  // Keep server-supplied text on one bounded line, including invalid UTF-8.
  return cloud::Json(value.substr(0, 256)).dump(-1, ' ', false, cloud::Json::error_handler_t::replace) +
         (value.size() > 256 ? " [truncated]" : "");
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

class CloudBridgeNode : public rclcpp::Node {
  using UsersService = luvbot_control_interfaces::srv::UpdateUsers;
  using MapControl = luvbot_control_interfaces::srv::MapControl;
  struct UsersUpdate {
    rclcpp::Client<UsersService>::SharedPtr client;
    luvbot::v1::Envelope request;
    bool active{false}, timed_out{false};
    SteadyClock::time_point started{};
  };

 public:
  CloudBridgeNode()
      : Node("luvbot_cloud_bridge"),
        device_id_(declare_parameter<std::string>("device_id", "RUMI-TEST-007")),
        session_(sessionId()) {
    host_ = declare_parameter<std::string>("host", "mqtt.luvbotics.com");
    if (host_.empty()) throw std::runtime_error("host must not be empty");
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
    property_topic_ = "luvbot/" + device_id_ + "/up/property";
    status_topic_ = "luvbot/" + device_id_ + "/up/status";
    command_topic_ = "luvbot/" + device_id_ + "/down/command";
    reply_topic_ = "luvbot/" + device_id_ + "/up/reply";
    event_topic_ = "luvbot/" + device_id_ + "/up/event";
    map_bridge_.reset(new cloud::MapBridge(*this,
        [this](const luvbot::v1::Envelope& frame, int code, const std::string& message) {
          reply(frame, code, message, true);
        },
        [this](const luvbot::v1::Envelope& frame, int* mid) { return publish(event_topic_, frame, mid); },
        [this]() { return envelope(); }));
    local_map_service_ = create_service<MapControl>("/luvbot/cloud/map_start",
        [this](std::shared_ptr<rmw_request_id_t> header, std::shared_ptr<MapControl::Request> request) {
          if (request->action != MapControl::Request::ACTION_MAPPING ||
              (request->index != MapControl::Request::MAP_INDEX_1 && request->index != MapControl::Request::MAP_INDEX_2))
          {
            localMapReply(header, MapControl::Response::ERROR_INVALID_REQUEST, "Use action=1 and index=1/2");
            return;
          }
          if (!local_map_requests_.empty())
          {
            localMapReply(header, MapControl::Response::ERROR_SERVICE_DOWN, "Local map start already awaiting brain response");
            return;
          }
          auto frame = envelope();
          frame.set_msg_id("local-map-" + session_ + "-" + std::to_string(frame.seq()));
          const auto map_id = request->map_id.empty()
              ? "map-local-" + session_ + "-" + std::to_string(frame.seq()) : request->map_id;
          auto* command = frame.mutable_command();
          command->set_cmd("map.start");
          auto* map = command->add_params();
          map->set_key("mapId");
          map->set_value(map_id);
          auto* index = command->add_params();
          index->set_key("mapIndex");
          index->set_value(std::to_string(request->index));
          local_map_requests_.emplace(frame.msg_id(), header);
          RCLCPP_INFO(get_logger(), "[MAP] local start injected msg_id=%s mapId=%s; using normal brain/upload workflow",
                      frame.msg_id().c_str(), logText(map_id).c_str());
          processCommand(frame);
        });
    wake_timeout_sec_ = declare_parameter("wake_timeout_sec", 10.0);
    if (!std::isfinite(wake_timeout_sec_) || wake_timeout_sec_ <= 0.0)
      throw std::runtime_error("wake_timeout_sec must be finite and positive");
    wake_client_ = create_client<WakeService>("/wake/set_custom_keyword");
    RCLCPP_INFO(get_logger(), "[RENAME] ready service=/wake/set_custom_keyword type=luvbot_control_interfaces/srv/SetCustomWakeWord timeout_sec=%.3f",
                wake_timeout_sec_);
    face_.client = create_client<UsersService>("/algorithm/face_perception/update_users");
    RCLCPP_INFO(get_logger(), "[FACE] ready service=/algorithm/face_perception/update_users type=luvbot_control_interfaces/srv/UpdateUsers path=%s timeout_sec=10",
                cloud::kFaceUsersPath);
    voice_.client = create_client<UsersService>("/algorithm/voice_perception/update_users");
    RCLCPP_INFO(get_logger(), "[VOICE] ready service=/algorithm/voice_perception/update_users type=luvbot_control_interfaces/srv/UpdateUsers path=%s timeout_sec=10",
                cloud::kVoiceUsersPath);
    health_.reset(new cloud::HealthReport(*this));
    emotion_sub_ = create_subscription<std_msgs::msg::String>(
        "/algorithm/brain/eyes_expression", rclcpp::QoS(10).reliable().durability_volatile(),
        [this](std_msgs::msg::String::ConstSharedPtr message) {
          if (message->data.size() > 4096) return;
          const auto value = cloud::Json::parse(message->data, nullptr, false);
          if (!value.is_object() || !value.contains("timestamp_ms") ||
              !value["timestamp_ms"].is_number_integer() || value["timestamp_ms"] <= 0 ||
              !value.contains("emotion_id") || !value["emotion_id"].is_string() ||
              value["emotion_id"].get_ref<const std::string&>().size() != 3 ||
              value["emotion_id"].get_ref<const std::string&>().compare(0, 2, "E0") != 0 ||
              value["emotion_id"].get_ref<const std::string&>()[2] < '1' ||
              value["emotion_id"].get_ref<const std::string&>()[2] > '8')
          {
            RCLCPP_WARN(get_logger(), "Invalid eyes_expression JSON; expected E01-E08; keeping last emotion code");
            return;
          }
          emotion_code_ = value["emotion_id"].get<std::string>();
        });
    personality_sub_ = create_subscription<std_msgs::msg::String>(
        "/algorithm/brain/personality", rclcpp::QoS(10).reliable().durability_volatile(),
        [this](std_msgs::msg::String::ConstSharedPtr message) { receivePersonality(message->data); });

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
      auto *self = static_cast<CloudBridgeNode *>(object);
      self->connected_.store(result == 0);
      if (result == 0)
      {
        self->announce_online_.store(true);
        RCLCPP_INFO(self->get_logger(), "MQTT connected; publishing status/heartbeats to %s", self->status_topic_.c_str());
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
      auto *self = static_cast<CloudBridgeNode *>(object);
      self->connected_.store(false);
      if (result != 0)
      {
        RCLCPP_WARN(self->get_logger(), "MQTT disconnected: %s; reconnecting", mosquitto_strerror(result));
      }
    });
    mosquitto_publish_callback_set(client, [](mosquitto *, void *object, int mid) {
      auto *self = static_cast<CloudBridgeNode *>(object);
      std::string request_id;
      {
        std::lock_guard<std::mutex> lock(self->rename_ack_mutex_);
        const auto it = self->rename_publish_ids_.find(mid);
        if (it != self->rename_publish_ids_.end())
        {
          request_id = std::move(it->second);
          self->rename_publish_ids_.erase(it);
        }
      }
      self->pending_.fetch_sub(1);
      self->map_bridge_->acknowledge(mid);
      if (!request_id.empty())
        RCLCPP_INFO(self->get_logger(), "[RENAME] reply_puback msg_id=%s mqtt_mid=%d; broker acknowledged, server business processing not confirmed",
                    logText(request_id).c_str(), mid);
    });
    mosquitto_message_callback_set(client, [](mosquitto *, void *object, const mosquitto_message *message) {
      auto *self = static_cast<CloudBridgeNode *>(object);
      self->receiveCommand(message);
    });
    mosquitto_subscribe_callback_set(client, [](mosquitto *, void *object, int, int count, const int *qos) {
      auto *self = static_cast<CloudBridgeNode *>(object);
      if (count != 1 || !qos || qos[0] == 0x80)
      {
        RCLCPP_ERROR(self->get_logger(), "Broker rejected command subscription: %s", self->command_topic_.c_str());
        return;
      }
      RCLCPP_INFO(self->get_logger(), "Command subscription confirmed: %s QoS=%d",
                  self->command_topic_.c_str(), qos[0]);
    });
    mosquitto_log_callback_set(client, [](mosquitto *, void *object, int level, const char *message) {
      if (level == MOSQ_LOG_ERR)
      {
        auto *self = static_cast<CloudBridgeNode *>(object);
        RCLCPP_ERROR(self->get_logger(), "MQTT: %s", message);
      }
    });

    cloud_connected_pub_ = create_publisher<std_msgs::msg::Bool>(
        "/luvbot/cloud_connected", rclcpp::QoS(1).reliable().transient_local());
    std_msgs::msg::Bool initial_state;
    initial_state.data = false;
    cloud_connected_pub_->publish(initial_state);
    // Receive live results only: a bridge restart must not replay an old provisioning success.
    provision_sub_ = create_subscription<std_msgs::msg::String>(
        "/luvbot/provision_result", rclcpp::QoS(10).reliable().durability_volatile(),
        [this](std_msgs::msg::String::ConstSharedPtr message) {
          if (message->data != "ok") return;
          RCLCPP_INFO(get_logger(), "BLE Wi-Fi connected; MQTT will connect/reconnect and send normal status/heartbeats");
          announce_online_.store(true);
          next_connect_attempt_ = SteadyClock::time_point{};
        });
    timer_ = create_wall_timer(std::chrono::milliseconds(100), [this]() { tick(); });
    // Defer first connect: no Wi-Fi/DNS at startup must not terminate the ROS node.
    RCLCPP_INFO(get_logger(), "MQTT bridge ready; device=%s heartbeat/property=1 Hz health=10 s; fixed properties for server integration",
                device_id_.c_str());
  }

  ~CloudBridgeNode() override {
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
  void localMapReply(const std::shared_ptr<rmw_request_id_t>& header, uint16_t error, const std::string& message) {
    MapControl::Response response;
    response.accepted = error == MapControl::Response::ERROR_NONE;
    response.error_code = error;
    response.message = message;
    try { local_map_service_->send_response(*header, response); }
    catch (const std::exception& failure) { RCLCPP_WARN(get_logger(), "[MAP] local reply failed: %s", failure.what()); }
  }

  void receiveCommand(const mosquitto_message *message) {
    if (!message || !message->topic || command_topic_ != message->topic)
    {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "Ignoring message outside this device's command topic");
      return;
    }
    // Bound parsing and log output for untrusted command payloads.
    if (!message->payload || message->payloadlen <= 0 || message->payloadlen > kMaxCommandPayloadBytes)
    {
      RCLCPP_WARN(get_logger(), "Rejected command: payload must contain 1-%d Protobuf bytes (received %d)",
                  kMaxCommandPayloadBytes, message->payloadlen);
      return;
    }
    luvbot::v1::Envelope frame;
    if (!frame.ParseFromArray(message->payload, message->payloadlen))
    {
      RCLCPP_WARN(get_logger(), "Rejected command: invalid Protobuf Envelope");
      return;
    }
    const bool rename = frame.has_command() && frame.command().cmd() == "device.rename";
    if (frame.has_command() && cloud::MapBridge::handles(frame.command().cmd()))
      RCLCPP_INFO(get_logger(), "[MAP] received cmd=%s msg_id=%s device_id=%s params=%d",
                  frame.command().cmd().c_str(), logText(frame.msg_id()).c_str(),
                  logText(frame.device_id()).c_str(), frame.command().params_size());
    if (rename)
      RCLCPP_INFO(get_logger(), "[RENAME] received topic=%s msg_id=%s device_id=%s trace_id=%s params=%d payload_bytes=%d",
                  command_topic_.c_str(), logText(frame.msg_id()).c_str(), logText(frame.device_id()).c_str(),
                  logText(frame.command().trace_id()).c_str(), frame.command().params_size(), message->payloadlen);
    if (frame.device_id() != device_id_ || frame.msg_id().empty() || !frame.has_command())
    {
      if (rename)
        RCLCPP_WARN(get_logger(), "[RENAME] rejected msg_id=%s: invalid envelope; expected device_id=%s, nonempty msg_id and command",
                    logText(frame.msg_id()).c_str(), device_id_.c_str());
      RCLCPP_WARN(get_logger(), "Rejected command: device_id mismatch, missing msg_id or missing command");
      return;
    }
    std::lock_guard<std::mutex> lock(command_mutex_);
    if (commands_.size() >= 32)
    {
      if (rename)
        RCLCPP_WARN(get_logger(), "[RENAME] rejected msg_id=%s: command queue full size=%zu; not executed",
                    logText(frame.msg_id()).c_str(), commands_.size());
      RCLCPP_WARN(get_logger(), "Command queue full; command not executed (server may retry)");
      return;
    }
    if (rename)
      RCLCPP_INFO(get_logger(), "[RENAME] queued msg_id=%s queue_size=%zu",
                  logText(frame.msg_id()).c_str(), commands_.size() + 1);
    commands_.push_back(std::move(frame));
  }

  void receiveOnly(const luvbot::v1::Envelope& frame) {
    const auto &command = frame.command();
    if (command.cmd() == "family.sync")
    {
      const char *required_key = "members";
      const std::string *required_value = nullptr;
      std::unordered_set<std::string> keys;
      for (const auto &param : command.params())
      {
        if (!keys.insert(param.key()).second)
        {
          RCLCPP_WARN(get_logger(), "Rejected %s: duplicate parameter key", command.cmd().c_str());
          return;
        }
        if (param.key() == required_key) required_value = &param.value();
      }
      if (!required_value || required_value->empty())
      {
        RCLCPP_WARN(get_logger(), "Rejected %s: %s must be a nonempty string",
                    command.cmd().c_str(), required_key);
        return;
      }
      auto header = frame;
      header.mutable_command()->clear_params();
      RCLCPP_INFO(get_logger(),
                  "RECEIVE ONLY %s params=%d payload_bytes=%d; no business execution or reply. envelope={%s}",
                  command.cmd().c_str(), command.params_size(), static_cast<int>(frame.ByteSizeLong()),
                  header.ShortDebugString().c_str());
      for (const auto &param : command.params())
      {
        if (param.value().size() <= 256 && param.key().size() <= 64)
        {
          RCLCPP_INFO(get_logger(), "RECEIVE ONLY %s param={%s}",
                      command.cmd().c_str(), param.ShortDebugString().c_str());
        }
        else
        {
          luvbot::v1::KeyValue key;
          key.set_key(param.key().substr(0, 64));
          RCLCPP_INFO(get_logger(), "RECEIVE ONLY %s param={%s} key_bytes=%zu value_bytes=%zu; full value at DEBUG",
                      command.cmd().c_str(), key.ShortDebugString().c_str(),
                      param.key().size(), param.value().size());
        }
      }
      RCLCPP_DEBUG(get_logger(), "RECEIVE ONLY %s frame={%s}",
                   command.cmd().c_str(), frame.ShortDebugString().c_str());
      return;
    }
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

  void reply(const luvbot::v1::Envelope& request, int code, const std::string& message,
             bool remember = false) {
    if (cloud::MapBridge::handles(request.command().cmd()))
      RCLCPP_INFO(get_logger(), "[MAP] reply cmd=%s msg_id=%s code=%d message=%s",
                  request.command().cmd().c_str(), logText(request.msg_id()).c_str(), code, logText(message).c_str());
    const std::string local_prefix = "local-map-" + session_ + "-";
    if (request.msg_id().compare(0, local_prefix.size(), local_prefix) == 0)
    {
      const auto it = local_map_requests_.find(request.msg_id());
      if (it != local_map_requests_.end())
      {
        const uint16_t error = code == 0 ? MapControl::Response::ERROR_NONE :
            code == 1003 ? MapControl::Response::ERROR_INVALID_REQUEST :
            (code == 1005 || code == 1006) ? MapControl::Response::ERROR_SERVICE_DOWN :
                                           MapControl::Response::ERROR_STEP_FAILED;
        localMapReply(it->second, error, message + " (" + request.msg_id() + ")");
        local_map_requests_.erase(it);
      }
      return;  // Local injection has no cloud command ID; never send a synthetic command_reply upstream.
    }
    auto result = envelope();
    auto* body = result.mutable_command_reply();
    body->set_msg_id(request.msg_id());
    body->set_cmd(request.command().cmd());
    body->set_code(code);
    body->set_message(message);
    if (remember)
    {
      if (completed_.find(request.msg_id()) == completed_.end())
      {
        if (completed_.size() >= 128)
        {
          RCLCPP_INFO(get_logger(), "Command cache evicted msg_id=%s", logText(completed_order_.front()).c_str());
          completed_.erase(completed_order_.front());
          completed_order_.pop_front();
        }
        completed_order_.push_back(request.msg_id());
      }
      completed_[request.msg_id()] = {request.command().SerializeAsString(), result};
    }
    if (replies_.size() < 128)
    {
      if (request.command().cmd() == "device.rename")
        RCLCPP_INFO(get_logger(), "[RENAME] reply_queued msg_id=%s code=%d message=%s cached=%s reply_queue_size=%zu",
                    logText(request.msg_id()).c_str(), code, logText(message).c_str(),
                    remember ? "true" : "false", replies_.size() + 1);
      replies_.push_back(std::move(result));
    }
    else
    {
      if (request.command().cmd() == "device.rename")
        RCLCPP_ERROR(get_logger(), "[RENAME] reply_queue_full msg_id=%s code=%d message=%s cached=%s",
                     logText(request.msg_id()).c_str(), code, logText(message).c_str(), remember ? "true" : "false");
      RCLCPP_ERROR(get_logger(), "Reply queue full; cached results remain available for duplicate commands");
    }
  }

  void processBiometric(const luvbot::v1::Envelope& frame, bool voice) {
    auto& update = voice ? voice_ : face_;
    const char* label = voice ? "VOICE" : "FACE";
    const std::string kind = voice ? "voice" : "face";
    const char* path = voice ? cloud::kVoiceUsersPath : cloud::kFaceUsersPath;
    const auto old = completed_.find(frame.msg_id());
    if (old != completed_.end())
    {
      if (old->second.command == frame.command().SerializeAsString())
        replies_.push_back(old->second.reply);
      else reply(frame, 1006, "msg_id reused with different command");
      return;
    }
    const auto& other = voice ? face_ : voice_;
    if (other.active && other.request.msg_id() == frame.msg_id())
    {
      reply(frame, 1006, "msg_id reused with different command");
      return;
    }
    if (update.active)
    {
      if (update.request.msg_id() != frame.msg_id())
        reply(frame, 1006, kind + " service busy; database unchanged");
      else if (update.request.command().SerializeAsString() != frame.command().SerializeAsString())
        reply(frame, 1006, "msg_id reused with different command");
      return;
    }
    const char* stage = "validation";
    try
    {
      const auto incoming = cloud::parameters(frame.command());
      const auto params = voice ? cloud::voiceParameters(incoming) : cloud::faceParameters(incoming);
      stage = "save";
      if (voice) cloud::saveVoice(params);
      else cloud::saveFace(params);
      RCLCPP_INFO(get_logger(), "[%s] saved msg_id=%s path=%s; notifying business service",
                  label, logText(frame.msg_id()).c_str(), path);
      if (!update.client->service_is_ready())
      {
        reply(frame, 1006, kind + " database saved; service unavailable");
        return;
      }
      stage = "service call";
      auto request = std::make_shared<UsersService::Request>();
      request->path = path;
      update.request = frame;
      update.active = true;
      update.timed_out = false;
      update.started = SteadyClock::now();
      const auto pending = update.client->async_send_request(request,
          [this, &update, label, kind](rclcpp::Client<UsersService>::SharedFuture future) {
            try
            {
              const auto response = future.get();
              RCLCPP_INFO(get_logger(), "[%s] service_response msg_id=%s success=%s message=%s late=%s",
                          label, logText(update.request.msg_id()).c_str(), response->success ? "true" : "false",
                          logText(response->message).c_str(), update.timed_out ? "true" : "false");
              if (response->success && response->message == "0")
                reply(update.request, 0, "0", true);
              else if (!response->success && response->message == "1001")
                reply(update.request, 1001, "1001", true);
              else if (!response->success && response->message == "1002")
                reply(update.request, 1002, "1002", true);
              else if (!response->success && response->message == "1003")
                reply(update.request, 1003, "1003", true);
              else reply(update.request, 1006, "invalid " + kind + " service response", true);
            }
            catch (const std::exception&)
            {
              reply(update.request, 1006, kind + " service response failed; outcome unconfirmed", true);
            }
            update.active = false;
            update.request.Clear();
          });
      RCLCPP_INFO(get_logger(), "[%s] service_pending msg_id=%s ros_request_id=%lld",
                  label, logText(frame.msg_id()).c_str(), static_cast<long long>(pending.request_id));
    }
    catch (const std::exception&)
    {
      update.active = false;
      update.request.Clear();
      // JSON exceptions can contain input fragments; do not log biometric data.
      const std::string message = kind + " " + stage + " failed";
      RCLCPP_WARN(get_logger(), "[%s] failed msg_id=%s stage=%s", label, logText(frame.msg_id()).c_str(), stage);
      reply(frame, 1006, message);
    }
  }

  void processCommand(const luvbot::v1::Envelope& frame) {
    const auto& command = frame.command();
    if (cloud::MapBridge::handles(command.cmd()))
    {
      const auto old = completed_.find(frame.msg_id());
      if (old != completed_.end())
      {
        if (old->second.command == command.SerializeAsString()) replies_.push_back(old->second.reply);
        else reply(frame, 1003, "msg_id reused with different command");
      }
      else map_bridge_->command(frame);
      return;
    }
    if (command.cmd() == "member.face.train")
    {
      processBiometric(frame, false);
      return;
    }
    if (command.cmd() == "member.voice.train")
    {
      processBiometric(frame, true);
      return;
    }
    if (command.cmd() != "device.rename")
    {
      receiveOnly(frame);
      return;
    }
    if (command.cmd() == "device.rename")
    {
      RCLCPP_INFO(get_logger(), "[RENAME] processing msg_id=%s params=%d",
                  logText(frame.msg_id()).c_str(), command.params_size());
      const auto old = completed_.find(frame.msg_id());
      if (old != completed_.end())
      {
        if (old->second.command == command.SerializeAsString())
        {
          RCLCPP_INFO(get_logger(), "[RENAME] cached_reply msg_id=%s code=%d message=%s; reusing result without service call",
                      logText(frame.msg_id()).c_str(), old->second.reply.command_reply().code(),
                      logText(old->second.reply.command_reply().message()).c_str());
          replies_.push_back(old->second.reply);
        }
        else reply(frame, 1001, "msg_id reused with different command");
        return;
      }
      if (rename_active_ && rename_request_.msg_id() == frame.msg_id())
      {
        RCLCPP_INFO(get_logger(), "[RENAME] duplicate_in_progress msg_id=%s timed_out=%s; no additional service call",
                    logText(frame.msg_id()).c_str(), rename_timed_out_ ? "true" : "false");
        if (rename_request_.command().SerializeAsString() != command.SerializeAsString())
          reply(frame, 1001, "msg_id reused with different command");
        return;
      }
    }
    try
    {
      const auto params = cloud::parameters(command);
      const auto& name = cloud::required(params, "name");
      RCLCPP_INFO(get_logger(), "[RENAME] validated msg_id=%s keyword=%s keyword_bytes=%zu",
                  logText(frame.msg_id()).c_str(), logText(name).c_str(), name.size());
      if (rename_active_)
      {
        RCLCPP_WARN(get_logger(), "[RENAME] busy msg_id=%s active_msg_id=%s timed_out=%s",
                    logText(frame.msg_id()).c_str(), logText(rename_request_.msg_id()).c_str(),
                    rename_timed_out_ ? "true" : "false");
        reply(frame, 1001, "wake service busy");
        return;
      }
      if (!wake_client_->service_is_ready())
      {
        RCLCPP_WARN(get_logger(), "[RENAME] service_unavailable msg_id=%s service=/wake/set_custom_keyword",
                    logText(frame.msg_id()).c_str());
        reply(frame, 1001, "wake service unavailable", true);
        return;
      }
      auto request = std::make_shared<WakeService::Request>();
      request->keyword = name;
      rename_request_ = frame;
      rename_active_ = true;
      rename_timed_out_ = false;
      rename_started_ = SteadyClock::now();
      const auto generation = ++rename_generation_;
      RCLCPP_INFO(get_logger(), "[RENAME] service_call msg_id=%s keyword=%s timeout_sec=%.3f",
                  logText(frame.msg_id()).c_str(), logText(name).c_str(), wake_timeout_sec_);
      const auto pending = wake_client_->async_send_request(request,
          [this, generation, msg_id = frame.msg_id()](rclcpp::Client<WakeService>::SharedFuture future) {
            if (!rename_active_ || generation != rename_generation_)
            {
              RCLCPP_WARN(get_logger(), "[RENAME] stale_response_ignored msg_id=%s", logText(msg_id).c_str());
              return;
            }
            if (rename_timed_out_)
            {
              RCLCPP_WARN(get_logger(), "[RENAME] late_response msg_id=%s; reporting the final result", logText(msg_id).c_str());
            }
            try
            {
              const auto response = future.get();
              RCLCPP_INFO(get_logger(), "[RENAME] service_response msg_id=%s result=%u evaluation=%s elapsed_ms=%.1f",
                          logText(msg_id).c_str(), static_cast<unsigned>(response->result),
                          logText(response->evaluation).c_str(),
                          std::chrono::duration<double, std::milli>(SteadyClock::now() - rename_started_).count());
              reply(rename_request_, response->result, response->evaluation, true);
            }
            catch (const std::exception& error)
            {
              RCLCPP_WARN(get_logger(), "[RENAME] service_response_failed msg_id=%s error=%s",
                          logText(msg_id).c_str(), logText(error.what()).c_str());
              reply(rename_request_, 1001, "wake service response failed", true);
            }
            rename_active_ = false;
            rename_request_.Clear();
            RCLCPP_INFO(get_logger(), "[RENAME] finished msg_id=%s; ready for next rename", logText(msg_id).c_str());
          });
      RCLCPP_INFO(get_logger(), "[RENAME] service_pending msg_id=%s ros_request_id=%lld",
                  logText(frame.msg_id()).c_str(), static_cast<long long>(pending.request_id));
    }
    catch (const std::exception& error)
    {
      if (command.cmd() == "device.rename")
        RCLCPP_WARN(get_logger(), "[RENAME] validation_or_call_failed msg_id=%s error=%s",
                    logText(frame.msg_id()).c_str(), logText(error.what()).c_str());
      if (rename_active_ && rename_request_.msg_id() == frame.msg_id())
      {
        rename_active_ = false;
        rename_request_.Clear();
      }
      reply(frame, 1001, error.what(), true);
    }
  }

  void processCommands(SteadyClock::time_point now) {
    for (auto* update : {&face_, &voice_})
    {
      if (update->active && !update->timed_out && now - update->started >= std::chrono::seconds(10))
      {
        reply(update->request, 1006, "biometric service timeout; outcome unconfirmed", true);
        // The server may still read this path: keep its database until it responds.
        update->timed_out = true;
        RCLCPP_WARN(get_logger(), "Biometric timeout msg_id=%s; retaining database until outstanding response",
                    logText(update->request.msg_id()).c_str());
      }
    }
    if (rename_active_ && !rename_timed_out_ &&
        std::chrono::duration<double>(now - rename_started_).count() >= wake_timeout_sec_)
    {
      RCLCPP_WARN(get_logger(), "[RENAME] timeout msg_id=%s elapsed_ms=%.1f; outcome unconfirmed, waiting for outstanding response",
                  logText(rename_request_.msg_id()).c_str(),
                  std::chrono::duration<double, std::milli>(now - rename_started_).count());
      reply(rename_request_, 1001, "wake service timeout; outcome unconfirmed", true);
      // Timeout cannot cancel the voice server. Retain this one pending request until it retires.
      rename_timed_out_ = true;
    }
    // Reserve room for outstanding face, voice and rename responses.
    for (unsigned i = 0; i < 8 && replies_.size() < 125; ++i)
    {
      luvbot::v1::Envelope command;
      {
        std::lock_guard<std::mutex> lock(command_mutex_);
        if (commands_.empty()) break;
        command = std::move(commands_.front());
        commands_.pop_front();
      }
      processCommand(command);
    }
    while (!replies_.empty() && publish(reply_topic_, replies_.front())) replies_.pop_front();
  }

  void receivePersonality(const std::string& data) {
    if (data.size() > 4096)
    {
      RCLCPP_WARN(get_logger(), "[PERSONALITY] rejected: JSON exceeds 4096 bytes");
      return;
    }
    const auto value = cloud::Json::parse(data, nullptr, false);
    if (!value.is_object() || !value.contains("published_at_ms") ||
        !value["published_at_ms"].is_number_integer() || value["published_at_ms"] <= 0 ||
        !value.contains("date") || !value["date"].is_string() ||
        !value.contains("values") || !value["values"].is_object())
    {
      RCLCPP_WARN(get_logger(), "[PERSONALITY] rejected: expected published_at_ms, date and values");
      return;
    }
    const auto date = value["date"].get<std::string>();
    bool valid_date = date.size() == 10;
    for (size_t i = 0; valid_date && i < date.size(); ++i)
      valid_date = (i == 4 || i == 7) ? date[i] == '-' : date[i] >= '0' && date[i] <= '9';
    if (valid_date)
    {
      const int year = std::stoi(date.substr(0, 4)), month = std::stoi(date.substr(5, 2));
      const int day = std::stoi(date.substr(8, 2));
      const bool leap = year % 4 == 0 && (year % 100 != 0 || year % 400 == 0);
      const int days[] = {31, leap ? 29 : 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};
      valid_date = year > 0 && month >= 1 && month <= 12 && day >= 1 && day <= days[month - 1];
    }
    if (!valid_date)
    {
      RCLCPP_WARN(get_logger(), "[PERSONALITY] rejected: invalid YYYY-MM-DD date");
      return;
    }
    const char* keys[] = {"O", "C", "E", "A", "N"};
    const char* names[] = {"openness", "conscientiousness", "extraversion", "agreeableness", "neuroticism"};
    cloud::Json ocean = cloud::Json::object();
    const auto& values = value["values"];
    for (size_t i = 0; i < 5; ++i)
    {
      if (!values.contains(keys[i]) || !values[keys[i]].is_number() ||
          !std::isfinite(values[keys[i]].get<double>()) || values[keys[i]] < 0 || values[keys[i]] > 100)
      {
        RCLCPP_WARN(get_logger(), "[PERSONALITY] rejected: values must contain O/C/E/A/N in [0,100]");
        return;
      }
      ocean[names[i]] = values[keys[i]].get<double>() / 100.0;
    }
    const auto published = value["published_at_ms"].get<uint64_t>();
    if (date < personality_date_ || (date == personality_date_ && published <= personality_published_ms_)) return;
    personality_date_ = date;
    personality_published_ms_ = published;
    personality_value_ = ocean.dump();
  }

  luvbot::v1::Envelope propertyEnvelope() {
    auto frame = envelope();
    // shortcut: fixed server-integration properties; restore live telemetry after integration.
    auto* property = frame.mutable_property_report();
    property->set_fw_version("1.0.3");
    property->set_battery(86);
    property->set_rssi(-58);
    property->set_uptime(3600);
    property->set_temperature(365);
    property->set_net_type("wifi");
    return frame;
  }

  luvbot::v1::Envelope envelope() {
    luvbot::v1::Envelope result;
    const auto sequence = ++sequence_;
    result.set_device_id(device_id_);
    result.set_msg_id("cloud-" + session_ + "-" + std::to_string(sequence));
    result.set_seq(sequence);
    result.set_ts(
        std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch())
            .count());
    return result;
  }

  bool publish(const std::string &topic, const luvbot::v1::Envelope &frame, int* published_mid = nullptr) {
    const bool rename = frame.has_command_reply() && frame.command_reply().cmd() == "device.rename";
    const bool connected = connected_.load();
    const int inflight = pending_.load();
    // At most 20 QoS 1 messages awaiting ACK; no unbounded offline telemetry queue.
    if (!connected || inflight >= 20)
    {
      if (rename)
      {
        const std::string reason = connected ? "inflight_limit" : "disconnected";
        const auto waiting = frame.msg_id() + ":" + reason;
        if (deferred_rename_reply_ != waiting)
        {
          RCLCPP_WARN(get_logger(), "[RENAME] reply_waiting msg_id=%s reason=%s mqtt_inflight=%d; retained for retry",
                      logText(frame.command_reply().msg_id()).c_str(), reason.c_str(), inflight);
          deferred_rename_reply_ = waiting;
        }
      }
      return false;
    }
    std::string bytes;
    if (!frame.SerializeToString(&bytes))
    {
      if (rename)
        RCLCPP_ERROR(get_logger(), "[RENAME] reply_serialization_failed msg_id=%s",
                     logText(frame.command_reply().msg_id()).c_str());
      RCLCPP_ERROR(get_logger(), "Protobuf serialization failed");
      return false;
    }
    // Hold across publish/registration so an immediate QoS 1 PUBACK cannot overtake its log entry.
    std::unique_lock<std::mutex> ack_lock(rename_ack_mutex_, std::defer_lock);
    if (rename) ack_lock.lock();
    pending_.fetch_add(1);  // Reserve before publish: the network thread may ACK immediately.
    int mid = 0;
    const int result = mosquitto_publish(client_.get(), &mid, topic.c_str(), static_cast<int>(bytes.size()),
                                         bytes.data(), 1, false);
    if (result != MOSQ_ERR_SUCCESS)
    {
      pending_.fetch_sub(1);
      if (rename)
        RCLCPP_ERROR(get_logger(), "[RENAME] reply_publish_failed msg_id=%s error=%s; retained for retry",
                     logText(frame.command_reply().msg_id()).c_str(), mosquitto_strerror(result));
      RCLCPP_ERROR(get_logger(), "Publish failed: %s", mosquitto_strerror(result));
      return false;
    }
    if (rename)
    {
      rename_publish_ids_[mid] = frame.command_reply().msg_id();
      deferred_rename_reply_.clear();
      RCLCPP_INFO(get_logger(), "[RENAME] reply_submitted msg_id=%s code=%d message=%s topic=%s mqtt_mid=%d payload_bytes=%zu; awaiting broker PUBACK",
                  logText(frame.command_reply().msg_id()).c_str(), frame.command_reply().code(),
                  logText(frame.command_reply().message()).c_str(), topic.c_str(), mid, bytes.size());
    }
    if (published_mid) *published_mid = mid;
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

  void tryStartNetwork(SteadyClock::time_point now) {
    if (loop_started_ || now < next_connect_attempt_) return;
    next_connect_attempt_ = now + std::chrono::seconds(5);
    int result = mosquitto_connect_async(client_.get(), host_.c_str(), 8883, 60);
    if (result == MOSQ_ERR_SUCCESS)
    {
      result = mosquitto_loop_start(client_.get());
      loop_started_ = result == MOSQ_ERR_SUCCESS;
    }
    if (!loop_started_)
    {
      RCLCPP_WARN(get_logger(), "MQTT initial connection unavailable: %s; retrying in 5 s",
                  mosquitto_strerror(result));
    }
    // After loop_start, libmosquitto handles dropped connections with its 1-30 s backoff.
  }

  void tick() {
    const auto now = SteadyClock::now();
    tryStartNetwork(now);
    processCommands(now);
    map_bridge_->tick();
    const bool connected = connected_.load();
    if (connected != last_cloud_connected_)
    {
      std_msgs::msg::Bool message;
      message.data = connected;
      cloud_connected_pub_->publish(message);
      last_cloud_connected_ = connected;
    }
    if (connected_.load())
    {
      if (announce_online_.exchange(false))
      {
        next_heartbeat_ = now;
        next_property_ = now;
        next_health_ = now;
      }
      if (now >= next_heartbeat_)
      {
        if (publishStatus(true))
        {
          next_heartbeat_ += std::chrono::seconds(1);
          if (next_heartbeat_ <= now) next_heartbeat_ = now + std::chrono::seconds(1);
        }
      }
      if (now >= next_property_)
      {
        auto frame = propertyEnvelope();
        auto* property = frame.mutable_property_report();
        if (!emotion_code_.empty())
        {
          auto* mood = property->add_metrics();
          mood->set_key("mood");
          mood->set_str_value(emotion_code_);
        }
        if (publish(property_topic_, frame))
        {
          next_property_ += std::chrono::seconds(1);
          if (next_property_ <= now) next_property_ = now + std::chrono::seconds(1);
        }
      }
      if (!personality_value_.empty() && personality_value_ != personality_sent_)
      {
        auto frame = propertyEnvelope();
        auto* metric = frame.mutable_property_report()->add_metrics();
        metric->set_key("personality");
        metric->set_str_value(personality_value_);
        if (publish(property_topic_, frame))
        {
          personality_sent_ = personality_value_;
          RCLCPP_INFO(get_logger(), "[PERSONALITY] submitted date=%s published_at_ms=%llu msg_id=%s",
                      personality_date_.c_str(), static_cast<unsigned long long>(personality_published_ms_),
                      frame.msg_id().c_str());
        }
      }
      if (now >= next_health_)
      {
        try
        {
          auto frame = envelope();
          auto* event = frame.mutable_event_report();
          const auto data = health_->sample();
          event->set_event("robot.health");
          event->set_level(data["emergency"]["active"] == true ? 3 : 1);
          event->set_detail(data.dump());
          publish(event_topic_, frame);
        }
        catch (const std::exception& error)
        {
          RCLCPP_WARN(get_logger(), "Health snapshot unavailable: %s", error.what());
        }
        next_health_ = now + std::chrono::seconds(10);
      }
    }
  }

  using WakeService = luvbot_control_interfaces::srv::SetCustomWakeWord;
  UsersUpdate face_, voice_;
  struct Completed { std::string command; luvbot::v1::Envelope reply; };
  std::mutex command_mutex_;
  std::mutex rename_ack_mutex_;
  std::unordered_map<int, std::string> rename_publish_ids_;
  std::string deferred_rename_reply_;
  std::deque<luvbot::v1::Envelope> commands_, replies_;
  std::unordered_map<std::string, Completed> completed_;
  std::deque<std::string> completed_order_;
  luvbot::v1::Envelope rename_request_;
  bool rename_active_{false}, rename_timed_out_{false};
  uint64_t rename_generation_{0};
  double wake_timeout_sec_{10.0};
  SteadyClock::time_point rename_started_{}, next_health_{};
  std::unique_ptr<cloud::HealthReport> health_;
  std::unique_ptr<cloud::MapBridge> map_bridge_;
  rclcpp::Service<MapControl>::SharedPtr local_map_service_;
  std::unordered_map<std::string, std::shared_ptr<rmw_request_id_t>> local_map_requests_;
  rclcpp::Client<WakeService>::SharedPtr wake_client_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr emotion_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr personality_sub_;
  std::string personality_date_, personality_value_, personality_sent_;
  uint64_t personality_published_ms_{0};
  std::string reply_topic_, event_topic_, emotion_code_;
  std::string device_id_, property_topic_, status_topic_, command_topic_, session_, host_;
  SteadyClock::time_point next_heartbeat_{}, next_property_{}, next_connect_attempt_{};
  bool last_cloud_connected_{false};
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr cloud_connected_pub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr provision_sub_;
  std::atomic<bool> connected_{false}, announce_online_{false};
  std::atomic<int> pending_{0};
  std::uint32_t sequence_{0};
  bool loop_started_{false};
  rclcpp::TimerBase::SharedPtr timer_;
  // The destructor stops the network thread before this client and callback state are destroyed.
  std::unique_ptr<mosquitto, decltype(&mosquitto_destroy)> client_{nullptr, &mosquitto_destroy};
};

int main(int argc, char **argv) {
  l0::printAppVersion("cloud_bridge_node");
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
  { rclcpp::spin(std::make_shared<CloudBridgeNode>()); } catch (const std::exception &error)
  {
    std::fprintf(stderr, "luvbot_cloud_bridge: %s\n", error.what());
    exit_code = 1;
  }
  mosquitto_lib_cleanup();
  rclcpp::shutdown();
  return exit_code;
}
