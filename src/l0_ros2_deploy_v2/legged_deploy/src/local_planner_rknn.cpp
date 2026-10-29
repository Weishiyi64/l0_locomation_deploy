#include <openssl/sha.h>
#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <functional>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <iomanip>
#include <iterator>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sstream>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/string.hpp>
#include <stdexcept>
#include <string>
#include <vector>

#include "base/first_order_low_pass.h"
#include "luvbot_control_interfaces/msg/policy_observation.hpp"
#include "rknn_model.h"

namespace {
constexpr std::size_t kHistoryLength = 5;
constexpr std::size_t kStateSize = 17;
constexpr std::size_t kInputSize = kHistoryLength * kStateSize;
constexpr std::size_t kTaskSize = 5;
constexpr std::size_t kPlannerInputSize = 9;
constexpr double kDefaultTimeoutSec = 0.5;
constexpr float kPi = 3.1415927f;
constexpr float kMinLinearSpeed = 0.3f;
constexpr float kMaxLinearSpeed = 0.4f;
constexpr float kDefaultAngularSpeed = 1.57f;
constexpr float kMaxRequestedAngularSpeed = 1.57f;
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
  const float low_speed = std::min(turn_speed, 0.3f);
  const float magnitude = turn_speed - (turn_speed - low_speed) * std::expm1(3.0f * fade) / 19.085537f;
  const float omega_star = std::abs(error) <= kPi / 12.0f ? 0.0f : std::copysign(magnitude, error);
  return {0.0f, 0.0f, 0.0f, error, omega_star};
}

std::string sha256File(const std::string& path) {
  std::ifstream stream(path, std::ios::binary);
  if (!stream) throw std::runtime_error("cannot open file for SHA256: " + path);
  std::vector<unsigned char> data((std::istreambuf_iterator<char>(stream)), std::istreambuf_iterator<char>());
  unsigned char digest[SHA256_DIGEST_LENGTH];
  SHA256(data.data(), data.size(), digest);
  std::ostringstream result;
  result << std::hex << std::setfill('0');
  for (unsigned char byte : digest) result << std::setw(2) << static_cast<unsigned int>(byte);
  return result.str();
}

void validateManifest(const std::string& onnx_path, const std::string& rknn_path) {
  const YAML::Node manifest = YAML::LoadFile(rknn_path + ".json");
  if (manifest["format_version"].as<int>(0) != 2 || manifest["target_platform"].as<std::string>("") != "rk3588" ||
      manifest["inference_partition"].as<std::string>("") != "actor_mlp")
    throw std::runtime_error("planner RKNN manifest has an unsupported format or target");
  if (manifest["rknn_sha256"].as<std::string>("") != sha256File(rknn_path) ||
      manifest["onnx_sha256"].as<std::string>("") != sha256File(onnx_path))
    throw std::runtime_error("planner model SHA256 does not match its manifest");
  const YAML::Node verification = manifest["verification"];
  if (!verification["performed"].as<bool>(false) || verification["samples"].as<int>(0) < 20)
    throw std::runtime_error("planner RKNN manifest requires at least 20 verified samples");
  const YAML::Node invariants = verification["invariants"];
  if (!invariants["nav_vy_zero"].as<bool>(false) || !invariants["face_vx_vy_zero"].as<bool>(false) ||
      !invariants["hold_all_zero"].as<bool>(false))
    throw std::runtime_error("planner RKNN manifest is missing output interlock verification");
  const YAML::Node inputs = manifest["inputs"];
  if (!inputs.IsSequence() || inputs.size() != 1 || inputs[0]["name"].as<std::string>("") != "state_history" ||
      inputs[0]["shape"].as<std::vector<int>>() != std::vector<int>({1, 5, 17}))
    throw std::runtime_error("planner manifest input must be state_history[1,5,17]");
  bool actions_found = false;
  for (const auto& output : manifest["outputs"])
    actions_found = actions_found || (output["name"].as<std::string>("") == "actions" &&
                                      output["shape"].as<std::vector<int>>() == std::vector<int>({1, 2}));
  if (!actions_found) throw std::runtime_error("planner manifest output must contain actions[1,2]");
  const YAML::Node contract = YAML::Load(manifest["metadata"]["l0_arrival_contract"].as<std::string>(""));
  if (contract["version"].as<int>(0) != 8 || contract["state_dim"].as<int>(0) != 17 ||
      contract["actor_history_length"].as<int>(0) != 5 ||
      contract["policy_architecture"].as<std::string>("") != "mlp_history" ||
      contract["previous_input"].as<std::string>("") != "issued_velocity" || contract["planner_hz"].as<int>(0) != 5 ||
      !contract["depth_shape"].IsNull())
    throw std::runtime_error("planner model contract is not the verified 5 Hz state-only v8 contract");
}
}  // namespace

class LocalPlannerRknn final : public rclcpp::Node {
 public:
  LocalPlannerRknn() : Node("local_planner") {
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
    const std::string rknn_path = rknnModelPath(onnx_path);
    validateManifest(onnx_path, rknn_path);
    model_ = std::make_unique<RknnModel>(rknn_path);
    if (model_->inputCount() != 1 || model_->inputName(0) != "state_history" || model_->inputElements(0) != kInputSize)
      throw std::runtime_error("planner RKNN input must be state_history[1,5,17]");
    actions_output_ = model_->outputIndex("actions");
    if (actions_output_ < 0 || model_->outputElements(static_cast<std::size_t>(actions_output_)) != 2)
      throw std::runtime_error("planner RKNN output must be actions[1,2]");

    command_sub_ = create_subscription<std_msgs::msg::Float32MultiArray>(
        "/planner_input", rclcpp::QoS(1), std::bind(&LocalPlannerRknn::commandCallback, this, std::placeholders::_1));
    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
        "/imu/data", rclcpp::SensorDataQoS(), std::bind(&LocalPlannerRknn::imuCallback, this, std::placeholders::_1));
    inference_sub_ = create_subscription<luvbot_control_interfaces::msg::PolicyObservation>(
        "control/policy/observation", rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile(),
        std::bind(&LocalPlannerRknn::inferenceEnabledCallback, this, std::placeholders::_1));
    command_pub_ = create_publisher<geometry_msgs::msg::TwistStamped>("control/cmd_vel", rclcpp::QoS(10));
    status_pub_ = create_publisher<std_msgs::msg::String>("/local_planner/state", rclcpp::QoS(10));
    publish_timer_ = create_wall_timer(std::chrono::milliseconds(20), std::bind(&LocalPlannerRknn::publish, this));
    invalidateResult();
    RCLCPP_INFO(get_logger(), "RKNN planner ready: %s, 5 Hz input, 50 Hz held output",
                rknnModelPath(onnx_path).c_str());
  }

 private:
  using SteadyClock = std::chrono::steady_clock;

  void resetMemory() {
    history_.fill(0.0f);
    previous_velocity_.fill(0.0f);
    history_ready_ = false;
  }

  void invalidateResult() {
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

  bool ready() const {
    const auto current = SteadyClock::now();
    return active_ && command_received_ && state_received_ &&
           std::chrono::duration<double>(current - command_time_).count() <= timeout_sec_ &&
           std::chrono::duration<double>(current - state_time_).count() <= sensor_timeout_sec_;
  }

  std::string readinessFailure() const {
    if (!command_received_) return command_error_.empty() ? "planner input has not been received" : command_error_;
    if (!state_received_) return imu_error_.empty() ? "IMU data has not been received" : imu_error_;

    const auto current = SteadyClock::now();
    if (std::chrono::duration<double>(current - command_time_).count() > timeout_sec_) return "planner input is stale";
    if (std::chrono::duration<double>(current - state_time_).count() > sensor_timeout_sec_) return "IMU data is stale";
    return {};
  }

  void inferenceEnabledCallback(const luvbot_control_interfaces::msg::PolicyObservation::ConstSharedPtr msg) {
    const bool next_active = msg->inference_enabled;
    if (next_active != active_) invalidateResult();
    active_ = next_active;
  }

  void imuCallback(const sensor_msgs::msg::Imu::ConstSharedPtr msg) {
    if (clockMovedBackwards()) return;
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
    if (stamp_ns <= 0 || (last_imu_stamp_ns_ > 0 && stamp_ns < last_imu_stamp_ns_))
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
    if (clockMovedBackwards()) return;
    try
    {
      if (msg->data.size() != kPlannerInputSize ||
          !std::all_of(msg->data.begin(), msg->data.end(), [](float value) { return std::isfinite(value); }))
        throw std::runtime_error("expected atomic task[5] + future waypoints[4]");

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
      if (!nav && std::any_of(preview_.begin(), preview_.end(), [](float value) { return value != 0.0f; }))
        throw std::runtime_error("Face/Hold preview must be zero");

      command_time_ = SteadyClock::now();
      command_received_ = true;
      command_error_.clear();
      std_msgs::msg::String status;
      status.data = nav ? "Nav" : face ? "Face" : "Hold";
      status_pub_->publish(status);
      if (!nav && !face)
      {
        resetMemory();
        result_.fill(0.0);
        result_stamp_ = now();
        result_time_ = SteadyClock::now();
        result_valid_ = true;
      }
      else
      {
        infer();
      }
    } catch (const std::exception& error)
    {
      command_received_ = false;
      command_error_ = error.what();
      RCLCPP_ERROR(get_logger(), "Invalid planner input: %s", error.what());
      invalidateResult();
    }
  }

  void infer() {
    if (!ready())
    {
      invalidateResult();
      return;
    }

    std::array<float, kStateSize> state{};
    std::copy(omega_.begin(), omega_.end(), state.begin());
    std::copy(gravity_.begin(), gravity_.end(), state.begin() + 3);
    std::copy(previous_velocity_.begin(), previous_velocity_.end(), state.begin() + 6);
    const auto processed_task = actorTask(task_);
    std::copy(processed_task.begin(), processed_task.end(), state.begin() + 8);
    std::copy(preview_.begin(), preview_.end(), state.begin() + 13);
    if (!history_ready_)
    {
      for (std::size_t i = 0; i < kHistoryLength; ++i)
        std::copy(state.begin(), state.end(), history_.begin() + i * kStateSize);
      history_ready_ = true;
    }
    else
    {
      std::copy(history_.begin() + kStateSize, history_.end(), history_.begin());
      std::copy(state.begin(), state.end(), history_.end() - kStateSize);
    }

    const auto started = SteadyClock::now();
    if (!model_->run(history_.data(), history_.size()))
    {
      invalidateResult();
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000, "RKNN planner failed: %s", model_->lastError().c_str());
      return;
    }
    const auto& actions = model_->output(static_cast<std::size_t>(actions_output_));
    if (actions.size() != 2 ||
        !std::all_of(actions.begin(), actions.end(), [](float value) { return std::isfinite(value); }))
    {
      invalidateResult();
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000, "RKNN planner returned invalid actions");
      return;
    }
    const bool nav = processed_task[0] != 0.0f || processed_task[1] != 0.0f || processed_task[2] != 0.0f;
    const bool face = processed_task[3] != 0.0f || processed_task[4] != 0.0f;
    const float raw_vx = processed_task[2] + actions[0];
    const float raw_wz = processed_task[4] + actions[1];
    const float vx = nav ? std::clamp(raw_vx, -kMaxLinearSpeed, kMaxLinearSpeed) : 0.0f;
    const float wz = (nav || face) ? std::clamp(raw_wz, -kMaxAngularSpeed, kMaxAngularSpeed) : 0.0f;
    result_ = {vx, 0.0, wz};
    previous_velocity_ = {vx, wz};
    result_stamp_ = now();
    result_time_ = SteadyClock::now();
    result_valid_ = true;
    const double elapsed_ms = std::chrono::duration<double, std::milli>(result_time_ - started).count();
    if (elapsed_ms > 50.0)
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "RKNN planner inference took %.1f ms", elapsed_ms);
  }

  void publish() {
    if (clockMovedBackwards()) return;
    if (!active_) return;

    const std::string failure = readinessFailure();
    if (!failure.empty())
    {
      invalidateResult();
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000, "%s; suppressing planner velocity output",
                            failure.c_str());
      return;
    }

    if (!result_valid_) return;
    if (std::chrono::duration<double>(SteadyClock::now() - result_time_).count() > timeout_sec_)
    {
      invalidateResult();
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000,
                            "planner result is stale; suppressing planner velocity output");
      return;
    }

    geometry_msgs::msg::TwistStamped message;
    message.header.stamp = result_stamp_;
    message.header.frame_id = kCommandFrame;
    message.twist.linear.x = result_[0];
    message.twist.linear.y = result_[1];
    message.twist.angular.z = result_[2];
    command_pub_->publish(message);
  }

  double timeout_sec_{kDefaultTimeoutSec};
  double sensor_timeout_sec_{0.1};
  bool active_{false};
  bool command_received_{false};
  bool state_received_{false};
  bool history_ready_{false};
  bool result_valid_{false};
  std::string command_error_;
  std::string imu_error_;
  int actions_output_{-1};
  int64_t last_ros_time_ns_{0};
  int64_t last_imu_stamp_ns_{0};
  FirstOrderLowPass3 imu_filter_;
  std::array<float, 3> omega_{};
  std::array<float, 3> gravity_{};
  std::array<float, kTaskSize> task_{};
  std::array<float, 4> preview_{};
  std::array<float, 2> previous_velocity_{};
  std::array<float, kInputSize> history_{};
  std::array<double, 3> result_{};
  rclcpp::Time result_stamp_{0, 0, RCL_ROS_TIME};
  SteadyClock::time_point command_time_{};
  SteadyClock::time_point state_time_{};
  SteadyClock::time_point result_time_{};
  std::unique_ptr<RknnModel> model_;
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr command_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<luvbot_control_interfaces::msg::PolicyObservation>::SharedPtr inference_sub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr command_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::TimerBase::SharedPtr publish_timer_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  try
  { rclcpp::spin(std::make_shared<LocalPlannerRknn>()); } catch (const std::exception& error)
  {
    RCLCPP_FATAL(rclcpp::get_logger("local_planner"), "Planner startup failed: %s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
