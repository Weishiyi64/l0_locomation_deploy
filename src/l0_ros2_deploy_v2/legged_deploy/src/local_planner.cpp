
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <mutex>
#include <cstring>
#include <functional>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/string.hpp>
#include <stdexcept>
#include <string>
#include <vector>
#include <utility>

#include "base/first_order_low_pass.h"
#include "robot_interfaces/msg/policy_observation.hpp"
#include "inference_model.h"
#include "AppVersion.h"

namespace {
constexpr std::size_t kHistoryLength = 5;
constexpr std::size_t kStateSize = 11;
constexpr std::size_t kInputSize = kHistoryLength * kStateSize;
constexpr std::size_t kTaskSize = 5;
constexpr std::size_t kObservationSize = 2631;
constexpr std::size_t kPlannerInputSize = 2582;
constexpr double kDefaultTimeoutSec = 0.5;
constexpr auto kInferencePeriod = std::chrono::milliseconds(200);
constexpr float kPi = 3.1415927f;
constexpr float kMinLinearSpeed = 0.3f;
constexpr float kMaxLinearSpeed = 0.8f;
constexpr float kMaxReverseLinearSpeed = 0.4f;
constexpr float kDefaultAngularSpeed = 1.57f;
constexpr float kMaxRequestedAngularSpeed = 1.57f;
constexpr float kMinFaceAngularSpeed = 0.5f;
constexpr float kMaxAngularSpeed = 1.57f;
constexpr const char* kCommandFrame = "device0/base_link";

double wrapAngle(double angle) { return std::atan2(std::sin(angle), std::cos(angle)); }

std::array<float, kTaskSize> actorTask(const std::array<float, kTaskSize>& task) {
  const bool nav = task[0] != 0.0f || task[1] != 0.0f || task[2] != 0.0f;
  const bool face = task[3] != 0.0f || task[4] != 0.0f;
  if (nav && !face)
  {
    const float linear_speed =
        task[2] == 0.0f ? kMinLinearSpeed : std::clamp(task[2], kMinLinearSpeed, kMaxLinearSpeed);
    return {task[0], task[1], linear_speed, 0.0f, 0.0f};
  }
  if (!face || nav) return {};

  const float error = static_cast<float>(wrapAngle(task[3]));
  const float requested_turn_speed = std::abs(task[4]);
  const float turn_speed = requested_turn_speed == 0.0f
                               ? kDefaultAngularSpeed
                               : std::clamp(requested_turn_speed, 0.0f, kMaxRequestedAngularSpeed);
  const float fade = std::clamp((1.0f - std::abs(error)) / 0.7382006f, 0.0f, 1.0f);
  const float low_speed = std::min(turn_speed, kMinFaceAngularSpeed);
  const float magnitude = turn_speed - (turn_speed - low_speed) * std::expm1(3.0f * fade) / 19.085537f;
  const float omega_star = std::abs(error) <= kPi / 18.0f ? 0.0f : std::copysign(magnitude, error);
  return {0.0f, 0.0f, 0.0f, error, omega_star};
}

}  // namespace

class LocalPlanner final : public rclcpp::Node {
 public:
  LocalPlanner() : Node("local_planner") {
    check_sensor_timestamps_ = declare_parameter<bool>("nav_check_sensor_timestamps", true);
    if (check_sensor_timestamps_)
      RCLCPP_INFO(get_logger(), "Navigation sensor timestamp checks enabled");
    else
      RCLCPP_WARN(get_logger(), "Navigation sensor timestamp checks DISABLED: source age ignored; "
                               "input/result watchdogs, HPM IMU and command/reference checks retained");
    declare_parameter<std::string>("model_path", "");
    declare_parameter<double>("input_timeout", kDefaultTimeoutSec);
    declare_parameter<double>("sensor_timeout", 0.1);
    declare_parameter<double>("imu_filter_alpha", 0.8);
    timeout_sec_ = get_parameter("input_timeout").as_double();
    sensor_timeout_sec_ = get_parameter("sensor_timeout").as_double();
    const double imu_filter_alpha = get_parameter("imu_filter_alpha").as_double();
    if (!std::isfinite(timeout_sec_) || timeout_sec_ <= 0.0)
      throw std::runtime_error("input_timeout must be finite and positive");
    if (!std::isfinite(sensor_timeout_sec_) || sensor_timeout_sec_ <= 0.0)
      throw std::runtime_error("sensor_timeout must be finite and positive");
    if (!std::isfinite(imu_filter_alpha) || imu_filter_alpha < 0.0 || imu_filter_alpha > 1.0)
      throw std::runtime_error("imu_filter_alpha must be finite and in [0, 1]");
    imu_filter_.setAlpha(imu_filter_alpha);

    const std::string onnx_path = get_parameter("model_path").as_string();
    if (onnx_path.empty()) throw std::runtime_error("model_path is required");
    model_ = loadPlannerModel(onnx_path);
    const std::vector<std::vector<std::size_t>> input_shapes{{1, 2631}, {1, 1, 128}};
    const std::vector<std::vector<std::size_t>> output_shapes{{1, 2}, {1, 1, 128}};
    if (model_->inputCount() != 2 || model_->outputCount() != 2 || model_->inputName(0) != "observations" ||
        model_->inputName(1) != "hidden_state" || model_->outputName(0) != "actions" ||
        model_->outputName(1) != "next_hidden_state") throw std::runtime_error("inference planner tensor names/count mismatch");
    for (unsigned i = 0; i < 2; ++i)
      for (bool input : {true, false})
      {
        const auto dims = input ? model_->inputShape(i) : inferenceOutputShape(*model_, i);
        const auto& shape = input ? input_shapes[i] : output_shapes[i];
        if (dims != shape || !isFloatingTensor(*model_, i, input))
          throw std::runtime_error("inference planner tensor shape/type mismatch");
      }
    // Receive the latest input even while the independent inference timer is running.
    command_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    inference_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::SubscriptionOptions command_options;
    command_options.callback_group = command_group_;
    command_sub_ = create_subscription<std_msgs::msg::Float32MultiArray>(
        "/planner_input", rclcpp::QoS(1), std::bind(&LocalPlanner::commandCallback, this, std::placeholders::_1),
        command_options);
    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
        "/imu/data", rclcpp::SensorDataQoS(), std::bind(&LocalPlanner::imuCallback, this, std::placeholders::_1));
    inference_sub_ = create_subscription<robot_interfaces::msg::PolicyObservation>(
        "control/policy/observation", rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile(),
        std::bind(&LocalPlanner::inferenceEnabledCallback, this, std::placeholders::_1));
    command_pub_ = create_publisher<geometry_msgs::msg::TwistStamped>("control/cmd_vel", rclcpp::QoS(10));
    status_pub_ = create_publisher<std_msgs::msg::String>("/local_planner/state", rclcpp::QoS(10));
    stop_reason_pub_ = create_publisher<std_msgs::msg::String>("/local_planner/stop_reason", rclcpp::QoS(1));
    publish_timer_ = create_wall_timer(std::chrono::milliseconds(20), std::bind(&LocalPlanner::publish, this));
    invalidateResult();
    inference_timer_ = create_wall_timer(kInferencePeriod, std::bind(&LocalPlanner::infer, this),
                                         inference_group_);
    RCLCPP_INFO(get_logger(), "inference planner ready: %s, 5 Hz inference from latest cached input, 50 Hz held output; "
                "linear [-0.4, 0.8] m/s, angular +/-1.57 rad/s",
                inferenceModelPath(onnx_path).c_str());
  }

 private:
  using SteadyClock = std::chrono::steady_clock;

  void resetMemory() {
    ++generation_;
    hidden_.fill(0.0f);
    history_.fill(0.0f);
    previous_velocity_.fill(0.0f);
    history_ready_ = false;
  }

  void invalidateResult() {
    const auto current = now();
    if (result_valid_ || safe_stamp_.nanoseconds() <= 0 || current.nanoseconds() < safe_stamp_.nanoseconds())
      safe_stamp_ = current;
    resetMemory();
    result_.fill(0.0);
    result_valid_ = false;
  }

  bool clockMovedBackwards() {
    const int64_t current = now().nanoseconds();
    if (last_ros_time_ns_ > 0 && current < last_ros_time_ns_)
    {
      last_ros_time_ns_ = current;
      command_received_ = state_received_ = false;
      last_imu_stamp_ns_ = 0;
      imu_filter_.reset();
      invalidateResult();
      RCLCPP_WARN(get_logger(), "ROS clock moved backwards; planner state reset");
      return true;
    }
    last_ros_time_ns_ = current;
    return false;
  }

  bool ready() const { return readinessFailure().empty(); }

  std::string readinessFailure() const {
    if (!active_) return "policy inference is disabled";
    if (!reference_active_) return "navigation reference is inactive";
    if (reference_deadline_ns_ <= 0 || now().nanoseconds() >= reference_deadline_ns_)
      return "navigation reference expired";
    const double sensor_age = (now().nanoseconds()-sensor_stamp_ns_)*1e-9;
    if (check_sensor_timestamps_ && (sensor_stamp_ns_ <= 0 || sensor_age < 0. || sensor_age > timeout_sec_))
      return "source sensor measurement is stale";
    if (std::chrono::duration<double>(SteadyClock::now() - activation_time_).count() > timeout_sec_) return "policy activation is stale";
    if (!command_received_) return command_error_.empty() ? "planner input has not been received" : command_error_;
    if (!state_received_) return imu_error_.empty() ? "IMU data has not been received" : imu_error_;

    const auto current = SteadyClock::now();
    if (std::chrono::duration<double>(current - command_time_).count() > timeout_sec_) return "planner input is stale";
    if (std::chrono::duration<double>(current - state_time_).count() > sensor_timeout_sec_) return "IMU data is stale";
    return {};
  }

  void inferenceEnabledCallback(const robot_interfaces::msg::PolicyObservation::ConstSharedPtr msg) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (msg->header.stamp.sec < 0 || msg->header.stamp.nanosec >= 1000000000U) {
      active_ = false; invalidateResult(); return;
    }
    const bool next_active = msg->inference_enabled;
    if (next_active != active_) invalidateResult();
    const auto stamp = rclcpp::Time(msg->header.stamp).nanoseconds();
    const double age = (now().nanoseconds()-stamp)*1e-9;
    if (stamp <= 0 || age < 0. || age > timeout_sec_) { active_ = false; invalidateResult(); return; }
    active_ = next_active;
    activation_time_ = SteadyClock::now();
    if (reference_active_)
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
          "[controller_velocity] inference_enabled=%d source_age_ms=%.1f "
          "vx=%.3f vy=%.3f wz=%.3f imu_wz=%.3f",
          static_cast<int>(msg->inference_enabled), age*1000., msg->velocity_command.linear.x,
          msg->velocity_command.linear.y, msg->velocity_command.angular.z, msg->imu.angular_velocity.z);
  }

  void imuCallback(const sensor_msgs::msg::Imu::ConstSharedPtr msg) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (clockMovedBackwards()) return;
    if (msg->header.stamp.sec < 0 || msg->header.stamp.nanosec >= 1000000000U) {
      state_received_ = false; invalidateResult(); return;
    }
    const std::array<double, 4> q{{msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w}};
    const std::array<double, 3> gyro{{msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z}};
    const std::array<double, 3> acceleration{
        {msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z}};
    const double norm = std::sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
    if (!std::all_of(q.begin(), q.end(), [](double value) { return std::isfinite(value); }) ||
        !std::all_of(gyro.begin(), gyro.end(), [](double value) { return std::isfinite(value); }) ||
        !std::all_of(acceleration.begin(), acceleration.end(), [](double value) { return std::isfinite(value); }) ||
        !std::isfinite(norm) || norm < 1e-8)
    {
      state_received_ = false;
      imu_error_ = "IMU data is non-finite or has an invalid quaternion";
      invalidateResult();
      return;
    }
    const int64_t stamp_ns = rclcpp::Time(msg->header.stamp).nanoseconds();
    if (stamp_ns <= 0 || (now().nanoseconds() - stamp_ns) * 1e-9 < 0.0 ||
        (now().nanoseconds() - stamp_ns) * 1e-9 > sensor_timeout_sec_ || (last_imu_stamp_ns_ > 0 && stamp_ns < last_imu_stamp_ns_))
    {
      state_received_ = false;
      imu_error_ = "IMU timestamp is zero or moved backwards";
      last_imu_stamp_ns_ = 0;
      imu_filter_.reset();
      invalidateResult();
      return;
    }
    last_imu_stamp_ns_ = stamp_ns;
    const double x = q[0] / norm;
    const double y = q[1] / norm;
    const double z = q[2] / norm;
    const double w = q[3] / norm;
    const auto& filtered_gyro = imu_filter_.update(gyro);
    omega_ = {static_cast<float>(filtered_gyro[0]), static_cast<float>(filtered_gyro[1]),
              static_cast<float>(filtered_gyro[2])};
    gravity_ = {static_cast<float>(2.0 * (w * y - x * z)), static_cast<float>(-2.0 * (y * z + w * x)),
                static_cast<float>(2.0 * (x * x + y * y) - 1.0)};
    state_time_ = SteadyClock::now();
    state_received_ = true;
    imu_error_.clear();
  }

  void commandCallback(const std_msgs::msg::Float32MultiArray::ConstSharedPtr msg) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (clockMovedBackwards()) return;
    try
    {
      if (msg->data.size() != kPlannerInputSize ||
          !std::all_of(msg->data.begin(), msg->data.end(), [](float value) { return std::isfinite(value); }))
        throw std::runtime_error("expected task[5] + path[16] + metric depth[2560] + freshness[1]");
      std::string token, reference_token;
      int64_t sensor_stamp = 0, reference_deadline = 0;
      bool reference_active = false;
      const auto positive_ns = [](const std::string& value) {
        if (value.empty() || value.find_first_not_of("0123456789") != std::string::npos)
          throw std::runtime_error("Invalid planner provenance timestamp");
        return std::stoll(value);
      };
      const std::string sensor_prefix = "sensor_stamp_ns:", deadline_prefix = "reference_deadline_ns:";
      for (const auto& dim : msg->layout.dim) {
        if (dim.label.compare(0, 5, "goal:") == 0) token = dim.label;
        if (dim.label.compare(0, 10, "reference:") == 0) reference_token = dim.label;
        if (dim.label.compare(0, sensor_prefix.size(), sensor_prefix) == 0)
          sensor_stamp = positive_ns(dim.label.substr(sensor_prefix.size()));
        if (dim.label.compare(0, deadline_prefix.size(), deadline_prefix) == 0)
          reference_deadline = positive_ns(dim.label.substr(deadline_prefix.size()));
        if (dim.label == "reference_active:1") reference_active = true;
      }
      reference_active_ = reference_active;
      if (!reference_active) {
        command_received_ = false;
        command_error_ = "navigation reference is inactive";
        invalidateResult();
        return;
      }
      if (token.empty() || reference_token.empty()) throw std::runtime_error("Planner task/reference identity is missing");
      if (reference_deadline <= now().nanoseconds()) throw std::runtime_error("Navigation reference expired");
      if (msg->data.back() != 1.0f) throw std::runtime_error("Depth or odometry is stale");
      const double sensor_age = (now().nanoseconds()-sensor_stamp)*1e-9;
      if (check_sensor_timestamps_ && (sensor_stamp <= 0 || sensor_age < 0. || sensor_age > timeout_sec_))
        throw std::runtime_error("Stale source sensor measurement");
      sensor_stamp_ns_ = sensor_stamp;
      if (token != goal_token_) { invalidateResult(); goal_token_ = token; }
      reference_deadline_ns_ = reference_deadline;

      for (std::size_t i = 0; i < kTaskSize; ++i) task_[i] = msg->data[i];
      const bool nav = task_[0] != 0.0f || task_[1] != 0.0f || task_[2] != 0.0f;
      const bool face = task_[3] != 0.0f || task_[4] != 0.0f;
      if (nav && face) throw std::runtime_error("Nav and Face fields must not be mixed");
      if (nav)
        task_[2] = task_[2] == 0.0f ? kMinLinearSpeed : std::clamp(task_[2], kMinLinearSpeed, kMaxLinearSpeed);
      else if (face)
      {
        task_[3] = static_cast<float>(wrapAngle(task_[3]));
        const float requested_turn_speed = std::abs(task_[4]);
        task_[4] = requested_turn_speed == 0.0f ? kDefaultAngularSpeed
                                                : std::clamp(requested_turn_speed, 0.0f, kMaxRequestedAngularSpeed);
      }
      for (std::size_t i = 0; i < preview_.size(); ++i) preview_[i] = msg->data[kTaskSize + i];
      std::copy(msg->data.begin() + 21, msg->data.begin() + 2581, depth_.begin());
      if (!nav && std::any_of(preview_.begin(), preview_.end(), [](float value) { return value != 0.0f; }))
        throw std::runtime_error("Face/Hold preview must be zero");

      command_time_ = SteadyClock::now();
      reference_token_ = reference_token;
      command_received_ = true;
      command_error_.clear();
      std_msgs::msg::String status;
      status.data = nav ? "Nav" : face ? "Face" : "Hold";
      status_pub_->publish(status);
    } catch (const std::exception& error)
    {
      command_received_ = false;
      command_error_ = error.what();
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000, "Invalid planner input: %s", error.what());
      invalidateResult();
    }
  }

  void infer() {
    std::unique_lock<std::mutex> lock(mutex_);
    if (clockMovedBackwards()) return;
    if (!ready()) { invalidateResult(); return; }
    const auto generation = generation_;
    const auto reference_token = reference_token_;
    const auto processed = actorTask(task_);
    std::array<float, kStateSize> frame{};
    std::copy(omega_.begin(), omega_.end(), frame.begin());
    std::copy(gravity_.begin(), gravity_.end(), frame.begin() + 3);
    std::copy(previous_velocity_.begin(), previous_velocity_.end(), frame.begin() + 6);
    std::copy(processed.begin() + 2, processed.end(), frame.begin() + 8);
    auto history = history_;
    if (!history_ready_)
      for (std::size_t i = 0; i < 5; ++i) std::copy(frame.begin(), frame.end(), history.begin() + i * kStateSize);
    else
    {
      std::move(history.begin() + kStateSize, history.end(), history.begin());
      std::copy(frame.begin(), frame.end(), history.end() - kStateSize);
    }
    std::array<float, kObservationSize> observations{};
    std::copy(history.begin(), history.end(), observations.begin());
    std::copy(preview_.begin(), preview_.end(), observations.begin() + 55);
    for (std::size_t i = 0; i < depth_.size(); ++i)
      observations[71+i] = depth_[i] > 0.f ? std::clamp(depth_[i], .2f, 5.f) / 5.f : 0.f;
    const auto hidden = hidden_;
    const auto started = SteadyClock::now();
    // Schedule from this start, so delayed timer dispatch does not cause a catch-up burst.
    inference_timer_->reset();
    const auto input_received = command_time_;
    const auto source_stamp = sensor_stamp_ns_;
    const auto reference_deadline = reference_deadline_ns_;
    const auto stamp = now();
    lock.unlock();
    bool success = false;
    try { success = model_->run({observations.data(), hidden.data()}, {observations.size(), hidden.size()}); }
    catch (const std::exception& error) { RCLCPP_ERROR(get_logger(), "Planner inference exception: %s", error.what()); }
    lock.lock();
    const auto completed = SteadyClock::now();
    const double inference_ms = std::chrono::duration<double, std::milli>(completed-started).count();
    const bool late = completed-started > kInferencePeriod;
    if (late) {
      inference_timer_->reset();
      RCLCPP_ERROR(get_logger(), "[planner_timeout] inference_ms=%.3f budget_ms=200; result discarded, no catch-up inference",
                   inference_ms);
    }
    if (generation != generation_ || reference_token != reference_token_) return;
    const double source_age = (now().nanoseconds()-source_stamp)*1e-9;
    if (!success || !ready() || now().nanoseconds() >= reference_deadline ||
        std::chrono::duration<double>(completed-input_received).count() > timeout_sec_ ||
        (check_sensor_timestamps_ && (source_stamp <= 0 || source_age < 0. || source_age > timeout_sec_)) || late)
    {
      invalidateResult();
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Planner inference failed, stale or late: %s", model_->lastError().c_str());
      return;
    }
    const auto& actions = model_->output(0);
    const bool nav = processed[0] != 0.f || processed[1] != 0.f || processed[2] != 0.f;
    const bool face = processed[3] != 0.f || processed[4] != 0.f;
    const float vx = nav ? std::clamp(processed[2]+actions[0], -kMaxReverseLinearSpeed, kMaxLinearSpeed) : 0.f;
    // Face turns keep a minimum command after residuals; a zero reference still stops.
    const float min_wz = face && processed[4] >= 0.f ? 0.f : -kMaxAngularSpeed;
    const float max_wz = face && processed[4] <= 0.f ? 0.f : kMaxAngularSpeed;
    float wz = nav || face ? std::clamp(processed[4]+actions[1], min_wz, max_wz) : 0.f;
    if (face && processed[4] != 0.f)
      wz = std::copysign(std::max(std::abs(wz), kMinFaceAngularSpeed), processed[4]);
    result_ = {vx, 0., wz};
    previous_velocity_ = {vx, wz};
    history_ = history;
    history_ready_ = true;
    std::copy(model_->output(1).begin(), model_->output(1).end(), hidden_.begin());
    result_sensor_stamp_ns_ = source_stamp;
    result_reference_deadline_ns_ = reference_deadline;
    result_stamp_ = stamp;
    result_time_ = started;
    result_valid_ = true;
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
        "[planner_command] mode=%s goal_body=(%.3f,%.3f) reference_vx=%.3f reference_wz=%.3f "
        "residual_vx=%.3f residual_wz=%.3f issued_vx=%.3f issued_wz=%.3f "
        "heading_error=%.3f imu_wz=%.3f inference_ms=%.1f",
        nav ? "Nav" : face ? "Face" : "Hold", processed[0], processed[1], processed[2], processed[4],
        actions[0], actions[1], vx, wz, processed[3], frame[2],
        std::chrono::duration<double, std::milli>(SteadyClock::now()-started).count());
  }

  void publish() {
    std::lock_guard<std::mutex> lock(mutex_);
    clockMovedBackwards();
    auto failure = readinessFailure();
    if (failure.empty() && result_valid_) {
      if (std::chrono::duration<double>(SteadyClock::now()-result_time_).count() > timeout_sec_)
        failure = "planner result is stale";
      else if (check_sensor_timestamps_ && (now().nanoseconds()-result_sensor_stamp_ns_)*1e-9 > timeout_sec_)
        failure = "planner result source sensor measurement is stale";
      else if (now().nanoseconds() >= result_reference_deadline_ns_)
        failure = "planner result navigation reference expired";
    }
    if (!failure.empty())
    {
      invalidateResult();
      if (active_ && reference_active_)
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Planner stopped: %s",
                             failure.c_str());
    }
    if (!result_valid_ && failure.empty()) failure = "waiting for a valid planner result";
    if (!stop_reason_logged_ || failure != last_stop_reason_) {
      stop_reason_logged_ = true;
      last_stop_reason_ = failure;
      RCLCPP_INFO(get_logger(), "[planner_stop] reason=%s timestamp_checks=%d source_age_ms=%.1f "
                  "input_receive_age_ms=%.1f result_age_ms=%.1f reference_active=%d",
                  failure.empty() ? "ready" : failure.c_str(), static_cast<int>(check_sensor_timestamps_),
                  sensor_stamp_ns_ > 0 ? (now().nanoseconds()-sensor_stamp_ns_)*1e-6 : -1.,
                  command_received_ ? std::chrono::duration<double, std::milli>(SteadyClock::now()-command_time_).count() : -1.,
                  result_time_.time_since_epoch().count() != 0 ?
                      std::chrono::duration<double, std::milli>(SteadyClock::now()-result_time_).count() : -1.,
                  static_cast<int>(reference_active_));
    }
    std_msgs::msg::String stop_reason;
    stop_reason.data = failure;
    stop_reason_pub_->publish(stop_reason);
    if (!active_) return;
    geometry_msgs::msg::TwistStamped message;
    // A safety zero is distinct from a computed result. Neither cached stamp is refreshed by retransmission.
    message.header.stamp = result_valid_ ? result_stamp_ : safe_stamp_;
    message.header.frame_id = kCommandFrame;
    if (result_valid_) {
      message.twist.linear.x = result_[0];
      message.twist.linear.y = result_[1];
      message.twist.angular.z = result_[2];
    }
    command_pub_->publish(message);
  }

  double timeout_sec_{kDefaultTimeoutSec};
  bool check_sensor_timestamps_{true}, stop_reason_logged_{false};
  std::string last_stop_reason_;
  double sensor_timeout_sec_{0.1};
  bool active_{false};
  bool reference_active_{false};
  bool command_received_{false};
  bool state_received_{false};
  bool history_ready_{false};
  bool result_valid_{false};
  std::string command_error_;
  std::string imu_error_;
  int64_t last_ros_time_ns_{0};
  int64_t last_imu_stamp_ns_{0};
  int64_t sensor_stamp_ns_{0}, result_sensor_stamp_ns_{0};
  int64_t reference_deadline_ns_{0}, result_reference_deadline_ns_{0};
  FirstOrderLowPass3 imu_filter_;
  std::array<float, 3> omega_{};
  std::array<float, 3> gravity_{};
  std::array<float, kTaskSize> task_{};
  std::array<float, 16> preview_{};
  std::array<float, 2560> depth_{};
  std::array<float, 128> hidden_{};
  std::mutex mutex_;
  std::uint64_t generation_{0};
  std::string goal_token_, reference_token_;
  rclcpp::CallbackGroup::SharedPtr command_group_;
  rclcpp::CallbackGroup::SharedPtr inference_group_;
  SteadyClock::time_point activation_time_{};
  std::array<float, 2> previous_velocity_{};
  std::array<float, kInputSize> history_{};
  std::array<double, 3> result_{};
  rclcpp::Time result_stamp_{0, 0, RCL_ROS_TIME};
  rclcpp::Time safe_stamp_{0, 0, RCL_ROS_TIME};
  SteadyClock::time_point command_time_{};
  SteadyClock::time_point state_time_{};
  SteadyClock::time_point result_time_{};
  std::unique_ptr<InferenceModel> model_;
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr command_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<robot_interfaces::msg::PolicyObservation>::SharedPtr inference_sub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr command_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr stop_reason_pub_;
  rclcpp::TimerBase::SharedPtr publish_timer_;
  rclcpp::TimerBase::SharedPtr inference_timer_;
};

int main(int argc, char** argv) {
  l0::printAppVersion("local_planner");
  rclcpp::init(argc, argv);
  try
  {
    auto node = std::make_shared<LocalPlanner>();
    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 2);
    executor.add_node(node);
    executor.spin();
  } catch (const std::exception& error)
  {
    RCLCPP_FATAL(rclcpp::get_logger("local_planner"), "Planner startup failed: %s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
