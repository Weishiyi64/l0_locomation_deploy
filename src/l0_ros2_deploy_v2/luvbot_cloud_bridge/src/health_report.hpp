#pragma once

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cctype>
#include <stdexcept>
#include <unordered_set>
#include <cstdint>
#include <dirent.h>
#include <fstream>
#include <limits>
#include <string>
#include <sys/statvfs.h>
#include <vector>

#include <nlohmann/json.hpp>
#include <rclcpp/rclcpp.hpp>
#include <luvbot_hardware_interfaces/msg/motor_state_array.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <robot_interfaces/msg/policy_observation.hpp>

namespace cloud {
using Json = nlohmann::json;
using Clock = std::chrono::steady_clock;

inline double ageMs(Clock::time_point time) {
  return std::chrono::duration<double, std::milli>(Clock::now() - time).count();
}

class SystemResources {
 public:
  SystemResources(double hot, double cool) : hot_(hot), cool_(cool) {
    if (!std::isfinite(hot) || !std::isfinite(cool) || cool >= hot)
      throw std::invalid_argument("health temperature thresholds require finite recover < overheat");
  }

  Json sample() {
    Json out = {{"cpu_temperature_c", nullptr}, {"cpu_usage_pct", nullptr}, {"memory_usage_pct", nullptr},
                {"disk_available_bytes", nullptr}, {"overheated", nullptr}};
    std::ifstream cpu("/proc/stat");
    std::string label;
    std::array<uint64_t, 8> ticks{};
    bool valid_cpu = bool(cpu >> label) && label == "cpu";
    for (auto& tick : ticks) valid_cpu = valid_cpu && bool(cpu >> tick);
    if (valid_cpu)
    {
      uint64_t total = 0;
      for (auto tick : ticks) total += tick;
      const auto idle = ticks[3] + ticks[4];
      if (cpu_seen_ && total > total_ && idle >= idle_ && idle - idle_ <= total - total_)
        out["cpu_usage_pct"] = 100.0 * (1.0 - double(idle - idle_) / double(total - total_));
      total_ = total;
      idle_ = idle;
      cpu_seen_ = true;
    }
    else cpu_seen_ = false;
    std::ifstream memory("/proc/meminfo");
    uint64_t total_kb = 0, available_kb = 0, value;
    bool available_seen = false;
    std::string unit;
    while (memory >> label >> value >> unit)
    {
      if (label == "MemTotal:") total_kb = value;
      if (label == "MemAvailable:") { available_kb = value; available_seen = true; }
    }
    if (available_seen && total_kb > 0 && available_kb <= total_kb)
      out["memory_usage_pct"] = 100.0 * double(total_kb - available_kb) / double(total_kb);
    struct statvfs disk{};
    if (statvfs("/", &disk) == 0)
      out["disk_available_bytes"] = uint64_t(disk.f_bavail) * uint64_t(disk.f_frsize);
    double temperature = -std::numeric_limits<double>::infinity();
    double soc_temperature = temperature;
    if (auto* directory = opendir("/sys/class/thermal"))
    {
      while (auto* entry = readdir(directory))
      {
        const std::string name(entry->d_name);
        if (name.compare(0, 12, "thermal_zone") != 0) continue;
        const auto root = "/sys/class/thermal/" + name + "/";
        std::ifstream type_file(root + "type"), temperature_file(root + "temp");
        std::string type;
        double millidegrees;
        if (!(type_file >> type) || !(temperature_file >> millidegrees)) continue;
        std::transform(type.begin(), type.end(), type.begin(), [](unsigned char c) { return std::tolower(c); });
        if (!std::isfinite(millidegrees)) continue;
        if (type.find("cpu") != std::string::npos || type.find("bigcore") != std::string::npos ||
            type.find("littlecore") != std::string::npos || type.find("cluster") != std::string::npos)
          temperature = std::max(temperature, millidegrees / 1000.0);
        else if (type.find("soc") != std::string::npos)
          soc_temperature = std::max(soc_temperature, millidegrees / 1000.0);
      }
      closedir(directory);
    }
    if (!std::isfinite(temperature)) temperature = soc_temperature;
    if (std::isfinite(temperature))
    {
      if (temperature >= hot_) overheated_ = true;
      else if (temperature <= cool_) overheated_ = false;
      out["cpu_temperature_c"] = temperature;
      out["overheated"] = overheated_;
    }
    return out;
  }

 private:
  double hot_, cool_;
  uint64_t total_{0}, idle_{0};
  bool cpu_seen_{false}, overheated_{false};
};

class HealthReport {
 public:
  explicit HealthReport(rclcpp::Node& node)
      : resources_(node.declare_parameter("health.overheat_c", 85.0),
                   node.declare_parameter("health.recover_c", 80.0)) {
    const auto sensor_qos = rclcpp::SensorDataQoS();
    const auto status_qos = rclcpp::QoS(1).reliable().durability_volatile();
    const auto safety_qos = rclcpp::QoS(1).reliable().transient_local();
    subscriptions_.push_back(node.create_subscription<std_msgs::msg::String>(
        "/system/motion_controller/status", status_qos,
        [this](std_msgs::msg::String::ConstSharedPtr msg) { receiveModule(msg->data, controller_, true); }));
    subscriptions_.push_back(node.create_subscription<std_msgs::msg::String>(
        "/hardware_status/fault_detail", status_qos,
        [this](std_msgs::msg::String::ConstSharedPtr msg) { receiveModule(msg->data, hardware_, false); }));
    subscriptions_.push_back(node.create_subscription<luvbot_hardware_interfaces::msg::MotorStateArray>(
        "/actuator/motor/state", sensor_qos, [this](luvbot_hardware_interfaces::msg::MotorStateArray::ConstSharedPtr msg) {
          for (const auto& state : msg->states)
          {
            const auto index = state.route.motor_type;
            if (index >= motors_.size()) continue;
            auto& motor = motors_[index];
            motor.received = Clock::now();
            motor.valid = state.valid && std::isfinite(state.q) && std::isfinite(state.dq) && std::isfinite(state.tau);
            motor.error = state.error_code;
          }
        }));
    subscriptions_.push_back(node.create_subscription<sensor_msgs::msg::Imu>(
        "/imu/data", sensor_qos, [this](sensor_msgs::msg::Imu::ConstSharedPtr msg) {
          imu_received_ = Clock::now();
          const auto& q = msg->orientation;
          const double norm = q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w;
          imu_valid_ = std::isfinite(norm) && norm >= 1e-12 &&
              std::isfinite(msg->angular_velocity.x) && std::isfinite(msg->angular_velocity.y) &&
              std::isfinite(msg->angular_velocity.z);
        }));
    subscriptions_.push_back(node.create_subscription<robot_interfaces::msg::PolicyObservation>(
        "/control/policy/observation", rclcpp::QoS(1).best_effort().durability_volatile(),
        [this](robot_interfaces::msg::PolicyObservation::ConstSharedPtr msg) {
          observation_received_ = Clock::now();
          inference_enabled_ = msg->inference_enabled;
        }));
    subscribeBool(node, "/hardware_status/motors_ready", safety_qos, motors_ready_);
    subscribeBool(node, "/system/safety/hardware_fault", safety_qos, hardware_fault_);
    subscribeBool(node, "/system/safety/emergency_stop", safety_qos, hardware_stop_);
  }

  Json sample() {
    const auto controller = module(controller_), hardware = module(hardware_);
    Json out = {{"schema_version", 1}, {"timestamp_ms", std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::system_clock::now().time_since_epoch()).count()},
        {"emergency", {{"active", nullptr}, {"controller", nullptr}, {"hardware", nullptr}}},
        {"mode", nullptr}, {"motors", {{"expected", nullptr}, {"normal", nullptr}, {"abnormal", nullptr}}},
        {"imu", nullptr}, {"policy", nullptr}, {"system", resources_.sample()}};
    if (!controller.is_null())
    {
      out["mode"] = controller.at("mode");
      out["emergency"]["controller"] = controller.at("emergency");
      out["policy"] = controller.at("policy");
      out["controller_sample_age_ms"] = controller.at("sample_age_ms").get<double>() + ageMs(controller_.received);
      auto& remaining = out["policy"]["valid_for_ms"];
      if (remaining.is_number())
      {
        remaining = std::max(0.0, remaining.get<double>() - ageMs(controller_.received));
        if (remaining == 0 && out["policy"]["valid"] == true) out["policy"]["valid"] = false;
      }
      auto& age = out["policy"]["last_update_age_ms"];
      if (age.is_number()) age = age.get<double>() + ageMs(controller_.received);
    }
    if (!hardware.is_null())
    {
      out["hardware_sample_age_ms"] = hardware.at("sample_age_ms").get<double>() + ageMs(hardware_.received);
      out["emergency"]["hardware"] = hardware.at("protection");
      out["imu"] = hardware.at("imu");
      auto& imu_age = out["imu"]["last_update_age_ms"];
      if (imu_age.is_number()) imu_age = imu_age.get<double>() + ageMs(hardware_.received);
      if (out["imu"]["valid"] == true && imu_age.is_number() &&
          imu_age.get<double>() > out["imu"]["timeout_ms"].get<double>()) out["imu"]["valid"] = false;
      out["imu"]["raw_valid"] = fresh(imu_received_, hardware.at("imu").at("timeout_ms").get<double>()) ?
          Json(imu_valid_) : Json(nullptr);
      const auto& enabled = hardware.at("enabled_motors");
      Json abnormal = Json::array();
      std::size_t normal = 0;
      for (const auto& entry : enabled)
      {
        const auto index = entry.at("index").get<std::size_t>();
        if (index >= motors_.size()) continue;
        const auto& motor = motors_[index];
        const bool live = fresh(motor.received, hardware.at("motor_feedback_timeout_ms").get<double>());
        if (live && motor.valid && motor.error == 0 && !entry.at("fault").get<bool>() &&
            entry.at("position_ready").get<bool>()) ++normal;
        else
          abnormal.push_back({{"index", index}, {"joint", entry.at("joint")},
              {"error_code", live ? Json(motor.error) : Json(nullptr)},
              {"latched_error_code", entry.at("error_code")},
              {"reason", !live ? Json("feedback_unavailable") :
                  (entry.at("fault") == true ? entry.at("reason") :
                    (motor.error != 0 ? Json("motor_error") :
                      (!motor.valid ? Json("invalid_feedback") : entry.at("reason"))))}});
      }
      out["motors"] = {{"expected", enabled.size()}, {"normal", normal}, {"abnormal", abnormal}};
    }
    const auto active = [](const Json& state) -> Json {
      return state.is_object() ? state.at("active") : Json(nullptr);
    };
    const auto control_stop = active(out["emergency"]["controller"]), hard_stop = active(out["emergency"]["hardware"]);
    if (control_stop == true || hard_stop == true) out["emergency"]["active"] = true;
    else if (control_stop == false && hard_stop == false) out["emergency"]["active"] = false;
    // Latched Bool topics have no heartbeat. Only expose them alongside a live module snapshot.
    out["motors"]["ready"] = hardware.is_null() ? Json(nullptr) : motors_ready_;
    out["emergency"]["hardware_fault_signal"] = hardware.is_null() ? Json(nullptr) : hardware_fault_;
    out["emergency"]["hardware_stop_signal"] = hardware.is_null() ? Json(nullptr) : hardware_stop_;
    out["policy_observation_live"] = fresh(observation_received_, 3000);
    out["inference_enabled"] = fresh(observation_received_, 3000) ? Json(inference_enabled_) : Json(nullptr);
    return out;
  }

 private:
  struct Module { Json value; Clock::time_point received{}; };
  struct Motor { bool valid{false}; uint16_t error{0}; Clock::time_point received{}; };
  static bool fresh(Clock::time_point time, double timeout_ms) {
    return time != Clock::time_point{} && ageMs(time) <= timeout_ms;
  }
  static void receiveModule(const std::string& text, Module& target, bool controller) {
    // Treat malformed module snapshots as unavailable, rather than losing the entire health report.
    try
    {
      if (text.size() > 65536) throw std::invalid_argument("oversized status");
      auto value = Json::parse(text);
      if (!value.is_object() || value.at("schema_version") != 1 ||
          !value.at("sample_age_ms").is_number() || value.at("sample_age_ms").get<double>() < 0)
        throw std::invalid_argument("invalid status header");
      const auto check_stop = [](const Json& state) {
        if (!state.at("active").is_boolean() || !state.at("reasons").is_array())
          throw std::invalid_argument("invalid protection state");
        for (const auto& reason : state.at("reasons"))
          if (!reason.is_string()) throw std::invalid_argument("invalid protection reason");
      };
      const auto check_age = [](const Json& age) {
        if (!age.is_null() && (!age.is_number() || age.get<double>() < 0))
          throw std::invalid_argument("invalid update age");
      };
      if (controller)
      {
        const auto mode = value.at("mode").get<std::string>();
        if (mode != "standby" && mode != "motion" && mode != "calibration" && mode != "emergency")
          throw std::invalid_argument("invalid mode");
        check_stop(value.at("emergency"));
        const auto& policy = value.at("policy");
        if (!policy.at("source").is_string() || !policy.at("required").is_boolean() ||
            (!policy.at("valid").is_null() && !policy.at("valid").is_boolean()))
          throw std::invalid_argument("invalid policy state");
        check_age(policy.at("last_update_age_ms"));
        check_age(policy.at("valid_for_ms"));
      }
      else
      {
        check_stop(value.at("protection"));
        if (!value.at("motor_feedback_timeout_ms").is_number() ||
            value.at("motor_feedback_timeout_ms").get<double>() <= 0)
          throw std::invalid_argument("invalid motor timeout");
        const auto& imu = value.at("imu");
        if (!imu.at("enabled").is_boolean() || !imu.at("timeout_ms").is_number() ||
            imu.at("timeout_ms").get<double>() <= 0 ||
            (!imu.at("valid").is_null() && !imu.at("valid").is_boolean()))
          throw std::invalid_argument("invalid IMU state");
        check_age(imu.at("last_update_age_ms"));
        const auto& motors = value.at("enabled_motors");
        if (!motors.is_array() || motors.size() > 256) throw std::invalid_argument("invalid motor roster");
        std::unordered_set<int> indices;
        for (const auto& motor : motors)
        {
          if (!motor.at("index").is_number_integer() || motor.at("index") < 0 || motor.at("index") >= 256)
            throw std::invalid_argument("invalid motor index");
          const int index = motor.at("index").get<int>();
          if (index < 0 || index >= 256 || !indices.insert(index).second ||
              !motor.at("joint").is_string() || !motor.at("fault").is_boolean() ||
              !motor.at("position_ready").is_boolean() || !motor.at("error_code").is_number_integer() ||
              !motor.at("reason").is_string()) throw std::invalid_argument("invalid motor status");
        }
      }
      const auto received = Clock::now();
      const auto now_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(received.time_since_epoch()).count();
      if (!value.at("sample_monotonic_ns").is_number_integer() ||
          value.at("sample_monotonic_ns") <= 0 || value.at("sample_monotonic_ns") > now_ns)
        throw std::invalid_argument("invalid local monotonic sample timestamp");
      const double sampled_age = double(now_ns - value.at("sample_monotonic_ns").get<int64_t>()) / 1e6;
      const double transport_age = std::max(0.0, sampled_age - value.at("sample_age_ms").get<double>());
      value["sample_age_ms"] = sampled_age;
      auto& state = value.at(controller ? "policy" : "imu");
      if (state.at("last_update_age_ms").is_number())
        state["last_update_age_ms"] = state["last_update_age_ms"].get<double>() + transport_age;
      if (controller && state.at("valid_for_ms").is_number())
        state["valid_for_ms"] = std::max(0.0, state["valid_for_ms"].get<double>() - transport_age);
      target.value = std::move(value);
      target.received = received;
    }
    catch (const std::exception&) { target = Module{}; }
  }
  static Json module(const Module& source) {
    if (!fresh(source.received, 3000) || !source.value.contains("sample_age_ms") ||
        !source.value["sample_age_ms"].is_number()) return nullptr;
    const double age = source.value["sample_age_ms"].get<double>();
    if (age < 0 || age + ageMs(source.received) > 3000) return nullptr;
    return source.value;
  }
  void subscribeBool(rclcpp::Node& node, const std::string& topic, const rclcpp::QoS& qos, Json& target) {
    auto* value = &target;
    subscriptions_.push_back(node.create_subscription<std_msgs::msg::Bool>(topic, qos,
        [value](std_msgs::msg::Bool::ConstSharedPtr msg) { *value = msg->data; }));
  }
  SystemResources resources_;
  Module controller_, hardware_;
  std::array<Motor, 256> motors_{};
  Clock::time_point imu_received_{}, observation_received_{};
  bool imu_valid_{false}, inference_enabled_{false};
  Json motors_ready_, hardware_fault_, hardware_stop_;
  std::vector<rclcpp::SubscriptionBase::SharedPtr> subscriptions_;
};
}  // namespace cloud
