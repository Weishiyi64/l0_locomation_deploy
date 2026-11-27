#include "rl_controller_wbc.h"

#include <algorithm>
#include <cmath>
#include <functional>
#include <stdexcept>

namespace {
std::string quoted(const std::string& value) {
  static const char hex[] = "0123456789abcdef";
  std::string out = "\"";
  for (unsigned char c : value) {
    if (c == '"' || c == '\\') { out += '\\'; out += static_cast<char>(c); }
    else if (c < 32) { out += "\\u00"; out += hex[c >> 4]; out += hex[c & 15]; }
    else out += static_cast<char>(c);
  }
  return out + '"';
}
void remember(std::deque<std::string>& ids, const std::string& id) {
  if (std::find(ids.begin(), ids.end(), id) == ids.end()) ids.push_back(id);
  if (ids.size() > 256) ids.pop_front();
}
}  // namespace

void RLWbc::initializeJsonReference() {
  node_->get_parameter("wbc_json_base_action", json_base_action_);
  if (json_base_action_.empty()) return;
  if (!lora_assets_ || !motions_.count(json_base_action_) ||
      !lora_assets_->action(json_base_action_).json_only ||
      lora_assets_->action(json_base_action_).parameter_source != "lora" || json_base_action_ != "UpperBody_Enhanced")
    throw std::runtime_error("text tracking requires the JSON-only UpperBody_Enhanced LoRA");
  std::string mapping_file;
  if (!node_->get_parameter("mapping_file", mapping_file) || mapping_file.empty())
    throw std::runtime_error("JSON WBC requires the robot mapping_file");
  json_joint_limits_ = wbcJsonJointLimits(mapping_file, joint_names_);
  std::string standing_config;
  if (!node_->get_parameter("standing_config", standing_config) || standing_config.empty())
    throw std::runtime_error("Text tracking requires standing_config");
  const auto standing = YAML::LoadFile(standing_config);
  standing_root_height_m_ = standing["standing_root_height_m"].as<double>();
  if (!std::isfinite(standing_root_height_m_) || standing_root_height_m_ <= 0.05 || standing_root_height_m_ >= 2.0)
    throw std::runtime_error("invalid configured standing_root_height_m");
  const auto legs = standing["standing_leg_joint_angles"];
  if (!legs.IsMap() || legs.size() != 12) throw std::runtime_error("Text standing config requires 12 named leg angles");
  for (std::size_t i = 0; i < 12; ++i) {
    if (!legs[joint_names_[i]]) throw std::runtime_error("Missing text standing joint: " + joint_names_[i]);
    const float angle = legs[joint_names_[i]].as<float>();
    if (!std::isfinite(angle) || angle < json_joint_limits_[i].first || angle > json_joint_limits_[i].second)
      throw std::runtime_error("Text standing target outside mechanical limits: " + joint_names_[i]);
    json_standing_joints_[i] = angle;
  }
  RCLCPP_INFO(node_->get_logger(), "Text JSON reference uses configured standing: root_height=%.3f; no reference model",
              standing_root_height_m_);
  json_status_pub_ = node_->create_publisher<std_msgs::msg::String>("/text_motion/wbc_status", 20);
  json_callback_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  json_cancel_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  rclcpp::SubscriptionOptions options;
  options.callback_group = json_callback_group_;
  json_sub_ = node_->create_subscription<std_msgs::msg::String>("/text_motion/json",
      rclcpp::QoS(1).reliable().durability_volatile(),
      std::bind(&RLWbc::jsonReferenceCallback, this, std::placeholders::_1), options);
  options.callback_group = json_cancel_group_;
  json_cancel_sub_ = node_->create_subscription<std_msgs::msg::String>("/text_motion/cancel", 10,
      std::bind(&RLWbc::jsonCancelCallback, this, std::placeholders::_1), options);
  RCLCPP_INFO(node_->get_logger(), "JSON tracking enabled: shared Base; LoRA=%s; /text_motion/json",
              json_base_action_.c_str());
}

void RLWbc::publishJsonStatus(const std::string& id, const char* state, const std::string& detail) {
  std_msgs::msg::String message;
  message.data = "{\"request_id\":" + quoted(id) + ",\"state\":" + quoted(state) +
      ",\"detail\":" + quoted(detail) + "}";
  json_status_pub_->publish(message);
}

void RLWbc::jsonReferenceCallback(const std_msgs::msg::String::ConstSharedPtr msg) {
  std::string id;
  try {
    if (msg->data.empty() || msg->data.size() > 4U * 1024U * 1024U)
      throw std::runtime_error("JSON must contain 1 byte to 4 MiB");
    const auto header = YAML::Load(msg->data);
    id = header["request_id"].as<std::string>("");
    if (header["wbc_base_action"].as<std::string>("") != json_base_action_)
      throw std::runtime_error("JSON tracking action mismatch");
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (std::find(json_cancelled_ids_.begin(), json_cancelled_ids_.end(), id) != json_cancelled_ids_.end()) {
        publishJsonStatus(id, "cancelled", "cancelled before JSON admission");
        return;
      }
      if (std::find(json_recent_ids_.begin(), json_recent_ids_.end(), id) != json_recent_ids_.end()) {
        publishJsonStatus(id, "duplicate", "JSON request already handled");
        return;
      }
    }
    std::unique_lock<std::mutex> loading_lock(loading_mutex_, std::try_to_lock);
    if (!loading_lock.owns_lock()) throw std::runtime_error("another WBC preparation is loading");
    auto reference = std::make_shared<WbcJsonReference>(header, joint_names_, json_joint_limits_,
        json_standing_joints_, standing_root_height_m_);
    auto motion = std::make_unique<Motion>();
    motion->robot_cfg_path_ = lora_assets_->action(json_base_action_).config;
    loadConfig(*motion, motion->robot_cfg_path_);
    motion->motion_id_ = reference->id();
    motion->json_reference_ = reference;
    motion->reference_num_frames_ = static_cast<int>(reference->size());
    motion->reference_root_body_index_ = 0;
    initializeModel(*motion);  // Reuse policy ABI checks; no reference model is created.
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (active_ || prepared_ || preparing_) throw std::runtime_error("WBC is preparing/prepared/active");
      if (std::find(json_cancelled_ids_.begin(), json_cancelled_ids_.end(), id) != json_cancelled_ids_.end()) {
        publishJsonStatus(id, "cancelled", "cancelled before JSON admission");
        return;
      }
      if (std::find(json_recent_ids_.begin(), json_recent_ids_.end(), id) != json_recent_ids_.end()) {
        publishJsonStatus(id, "duplicate", "JSON request already handled");
        return;
      }
      for (const auto& entry : motions_) {
        const auto& previous = entry.second;
        if (previous->json_reference_ && !previous->json_consumed_ && !previous->json_reference_->expired())
          throw std::runtime_error("a complete JSON is already waiting for its WBC command");
      }
      // Retire prior dynamic references only while idle and under the same
      // mutex as runLoop/prepare. Ordinary LoRA motions remain preloaded.
      for (auto it = motions_.begin(); it != motions_.end();) {
        if (it->second->json_reference_) {
          if (current_ == it->second.get()) current_ = nullptr;
          it = motions_.erase(it);
        } else ++it;
      }
      if (reference->expired()) throw std::runtime_error("JSON expired before registration");
      if (motions_.count(id)) throw std::runtime_error("JSON request ID collides with a fixed WBC action");
      remember(json_recent_ids_, id);
      motions_.emplace(id, std::move(motion));
    }
    publishJsonStatus(id, "ready", "complete JSON registered; tracking LoRA selected; no actuator command sent");
  } catch (const std::exception& error) {
    publishJsonStatus(id, "rejected", error.what());
  }
}

void RLWbc::jsonCancelCallback(const std_msgs::msg::String::ConstSharedPtr msg) {
  const auto& id = msg->data;
  if (id.size() > 32 || id.compare(0, 2, "T_") != 0) return;
  std::lock_guard<std::mutex> lock(mutex_);
  remember(json_cancelled_ids_, id);
  const auto motion = motions_.find(id);
  if (motion != motions_.end()) motion->second->json_consumed_ = true;
  const bool executing = current_ && current_->motion_id_ == id && (active_ || prepared_ || preparing_);
  publishJsonStatus(id, executing ? "cancelling" : "cancelled",
      executing ? "controller owns WBC to AMP handoff" : "cancelled before WBC activation");
}
