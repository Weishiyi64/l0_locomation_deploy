#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <thread>
#include <unordered_set>

#include "LeggedReal.h"

namespace legged {
namespace {
constexpr double kZeroPositionTolerance = 0.005;
}  // namespace

bool LeggedReal::zeroMotors(const std::vector<std::string>& joint_names) {
  std::lock_guard<std::mutex> lock(lifecycle_mutex_);
  if (!calibration_mode_.load() || lifecycle_active_ || lifecycle_fault_.load() ||
      zero_pending_ || zero_failed_ || !motor_mapping_)
  {
    RCLCPP_ERROR(nh_->get_logger(),
                 "[CALIBRATION] Zeroing blocked: enter calibration, wait for motor lifecycle completion, "
                 "and resolve any pending/failed batch");
    return false;
  }
  std::vector<uint8_t> indices;
  std::unordered_set<std::string> seen;
  for (const auto& name : joint_names)
  {
    const int index = motor_mapping_->indexOfJoint(name);
    if (index < 0 || index > std::numeric_limits<uint8_t>::max() || !seen.insert(name).second ||
        !motor_mapping_->entries()[index].enabled)
    {
      RCLCPP_ERROR(nh_->get_logger(), "[CALIBRATION] Invalid, duplicate or disabled joint: %s", name.c_str());
      return false;
    }
    indices.push_back(static_cast<uint8_t>(index));
  }
  if (indices.empty() || !zero_client_->service_is_ready())
  {
    RCLCPP_ERROR(nh_->get_logger(), "[CALIBRATION] No enabled joints selected or HPM service %s is unavailable",
                 zero_client_->get_service_name());
    return false;
  }

  zero_pending_indices_ = indices;
  zero_queued_indices_ = std::move(indices);
  // Unsent motors cannot be confirmed by feedback arriving during the queue delay.
  zero_feedback_after_ns_.assign(motor_mapping_->size(), std::numeric_limits<int64_t>::max());
  zero_pending_ = true;
  RCLCPP_INFO(nh_->get_logger(), "[CALIBRATION] Queued zero for %zu motors, at least 100 ms apart; zero-gain MIT continues",
              zero_queued_indices_.size());
  return true;
}

void LeggedReal::zeroTick() {
  using SetZero = luvbot_hardware_interfaces::srv::MotorSetZero;
  std::lock_guard<std::mutex> lock(lifecycle_mutex_);
  if (!zero_pending_ || zero_failed_ || !calibration_mode_.load() ||
      lifecycle_active_ || lifecycle_fault_.load() || zero_request_pending_)
    return;
  if (zero_queued_indices_.empty())
  {
    if (zero_pending_indices_.empty())
    {
      zero_pending_ = false;
      zero_failed_ = false;
      RCLCPP_INFO(nh_->get_logger(), "[CALIBRATION] Batch position confirmed from raw motor feedback");
    }
    return;
  }
  if (std::chrono::steady_clock::now() < zero_next_send_) return;

  const auto i = zero_queued_indices_.front();
  try
  {
    if (!zero_client_->service_is_ready()) throw std::runtime_error("HPM set_zero service is unavailable");
    auto request = std::make_shared<SetZero::Request>();
    request->motor_types.push_back(i);
    {
      // Recompute turns from feedback after the motor has reset its encoder position.
      std::lock_guard<std::mutex> feedback_lock(feedback_mutex_);
      feedback_wrap_[i] = system_command_arbiter::FeedbackWrapState{};
      feedback_updated_[i] = false;
      latest_feedback_[i].valid = false;
    }
    // The response releases HPM's single-batch slot; only raw q confirms the zero.
    zero_client_->async_send_request(request, [this](rclcpp::Client<SetZero>::SharedFuture) {
      std::lock_guard<std::mutex> lock(lifecycle_mutex_);
      zero_request_pending_ = false;
    });
    zero_request_pending_ = true;
    zero_feedback_after_ns_[i] = nh_->now().nanoseconds();
    zero_next_send_ = std::chrono::steady_clock::now() + std::chrono::milliseconds(100);
    zero_queued_indices_.erase(zero_queued_indices_.begin());
  } catch (const std::exception& error)
  {
    zero_pending_ = false;
    zero_failed_ = true;
    zero_pending_indices_.clear();
    zero_queued_indices_.clear();
    RCLCPP_ERROR(nh_->get_logger(), "[CALIBRATION] Request failed for %s (HPM index %u): %s; inspect before restarting",
                 joint_names_[i].c_str(), static_cast<unsigned>(i), error.what());
    return;
  }
  RCLCPP_INFO(nh_->get_logger(), "[CALIBRATION] Requested zero for %s (HPM index %u)", joint_names_[i].c_str(),
              static_cast<unsigned>(i));
  if (motor_mapping_->entries()[i].family == system_command_arbiter::MotorFamily::LS)
    RCLCPP_WARN(nh_->get_logger(), "[CALIBRATION] %s (HPM index %u): LS电机需要重启来保存参数",
                joint_names_[i].c_str(), static_cast<unsigned>(i));
  RCLCPP_INFO(nh_->get_logger(), "[CALIBRATION] Waiting for new raw motor q=0 within %.6f rad; zero-gain MIT continues",
              kZeroPositionTolerance);
}

bool LeggedReal::zeroMotorsByTag(const std::string& tag) {
  if (!motor_mapping_) return false;
  if (tag != "left_leg" && tag != "right_leg" && tag != "left_arm" && tag != "right_arm" && tag != "waist" &&
      tag != "neck")
  {
    RCLCPP_ERROR(nh_->get_logger(), "[CALIBRATION] Unknown joint group: %s", tag.c_str());
    return false;
  }
  std::vector<std::string> selected;
  for (std::size_t i = 0; i < joint_names_.size(); ++i)
  {
    const auto& name = joint_names_[i];
    const bool left = name.compare(0, 5, "left_") == 0;
    const bool right = name.compare(0, 6, "right_") == 0;
    const bool leg = name.find("hip") != std::string::npos || name.find("knee") != std::string::npos ||
                     name.find("ankle") != std::string::npos;
    const bool arm = name.find("shoulder") != std::string::npos || name.find("elbow") != std::string::npos ||
                     name.find("wrist") != std::string::npos;
    const bool head = name.find("neck") != std::string::npos || name.find("head") != std::string::npos;
    const bool matches =
        (tag == "left_leg" && left && leg) || (tag == "right_leg" && right && leg) ||
        (tag == "left_arm" && left && arm) || (tag == "right_arm" && right && arm) ||
        (tag == "neck" && head) || (tag == "waist" && name.find("waist") != std::string::npos);
    if (matches && motor_mapping_->entries()[i].enabled) selected.push_back(name);
  }
  if (selected.empty())
  {
    RCLCPP_ERROR(nh_->get_logger(), "[CALIBRATION] No enabled joints in group %s", tag.c_str());
    return false;
  }
  return zeroMotors(selected);
}

void LeggedReal::verifyZeroFeedback(const luvbot_hardware_interfaces::msg::MotorStateArray& msg) {
  if (!calibration_mode_.load()) return;
  std::lock_guard<std::mutex> lock(lifecycle_mutex_);
  if (!zero_pending_ || msg.stamp.sec < 0 || msg.stamp.nanosec >= 1000000000U) return;
  const auto stamp_ns = rclcpp::Time(msg.stamp).nanoseconds();
  for (auto it = zero_pending_indices_.begin(); it != zero_pending_indices_.end();)
  {
    const auto i = *it;
    if (stamp_ns <= zero_feedback_after_ns_[i])
    {
      ++it;
      continue;
    }
    const auto state = std::find_if(msg.states.begin(), msg.states.end(), [i](const auto& sample) {
      return sample.route.motor_type == i;
    });
    if (state == msg.states.end())
    {
      ++it;
      continue;
    }
    if (state->valid && std::isfinite(state->q) && std::abs(state->q) <= kZeroPositionTolerance)
    {
      RCLCPP_INFO(nh_->get_logger(),
                  "[CALIBRATION] %s (HPM index %u): POSITION_CONFIRMED motor_q=%.6f target_q=0 tolerance=%.6f rad",
                  joint_names_[i].c_str(), static_cast<unsigned>(i), state->q, kZeroPositionTolerance);
      it = zero_pending_indices_.erase(it);
    }
    else
    {
      RCLCPP_INFO_THROTTLE(nh_->get_logger(), *nh_->get_clock(), 1000,
                           "[CALIBRATION] %s (HPM index %u): waiting for raw q=0, motor_q=%.6f valid=%s tolerance=%.6f rad",
                           joint_names_[i].c_str(), static_cast<unsigned>(i), state->q,
                           state->valid ? "true" : "false", kZeroPositionTolerance);
      ++it;
    }
  }
}

bool LeggedReal::resetAllMotors() {
  RCLCPP_WARN(nh_->get_logger(), "Motor reset is not supported by the topic motor interface");
  return false;
}

bool LeggedReal::enableAllMotors() { return requestAllMotorsEnabled(true); }
bool LeggedReal::disableAllMotors() { return requestAllMotorsEnabled(false); }

bool LeggedReal::disableMotorsBeforeShutdown() {
  if (!lifecycle_timer_) return true;
  lifecycle_timer_->cancel();
  RCLCPP_INFO(nh_->get_logger(),
               "Stopping hardware bridge: disabling all configured motors; zero-gain MIT continues until process exit");
  if (!disableAllMotors()) return false;
  while (true)
  {
    lifecycleTick();
    {
      std::lock_guard<std::mutex> lock(lifecycle_mutex_);
      if (!lifecycle_active_ || enable_publisher_->get_subscription_count() == 0U) break;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  if (lifecycle_active_ || enable_publisher_->get_subscription_count() == 0U)
  {
    RCLCPP_ERROR(nh_->get_logger(), "Shutdown disable delivery unconfirmed: HPM enable subscriber unavailable");
    return false;
  }
  if (!enable_publisher_->wait_for_all_acked(std::chrono::milliseconds(500)))
  {
    RCLCPP_ERROR(nh_->get_logger(), "Shutdown disable delivery timed out after 500 ms; motor state is unconfirmed");
    return false;
  }
  RCLCPP_INFO(nh_->get_logger(), "Shutdown disable acknowledged by DDS; this is not motor execution confirmation");
  return true;
}

bool LeggedReal::set_calibration_mode(bool enabled) {
  std::lock_guard<std::mutex> lock(lifecycle_mutex_);
  if (!enabled && (zero_pending_ || zero_failed_))
  {
    RCLCPP_ERROR(nh_->get_logger(), "[CALIBRATION] Exit blocked by pending or failed zeroing");
    return false;
  }
  calibration_mode_.store(enabled);
  return true;
}

}  // namespace legged
