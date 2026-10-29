#include "system_command_arbiter/motor_mapping.hpp"

#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <unordered_map>
#include <unordered_set>
#include <utility>

namespace system_command_arbiter {
namespace {

using MotorRoute = luvbot_hardware_interfaces::msg::MotorRoute;

uint64_t physicalRouteKey(const MotorRoute& route) {
  return (static_cast<uint64_t>(route.bus_index) << 32U) | route.motor_id;
}

bool sameRoute(const MotorRoute& lhs, const MotorRoute& rhs) {
  return lhs.motor_type == rhs.motor_type && lhs.bus_index == rhs.bus_index && lhs.motor_id == rhs.motor_id;
}

double requiredFinite(const YAML::Node& node, const char* key, const std::string& context) {
  if (!node[key] || node[key].IsNull())
    throw std::runtime_error(context + "." + key + " is not configured; complete the motor mapping before enabling");
  const double value = node[key].as<double>();
  if (!std::isfinite(value)) throw std::runtime_error(context + "." + key + " must be finite");
  return value;
}

MotorLimits loadLimits(const YAML::Node& model, const std::string& name) {
  MotorLimits limits;
  limits.p_min = requiredFinite(model, "p_min", name);
  limits.p_max = requiredFinite(model, "p_max", name);
  limits.v_min = requiredFinite(model, "v_min", name);
  limits.v_max = requiredFinite(model, "v_max", name);
  limits.t_min = requiredFinite(model, "t_min", name);
  limits.t_max = requiredFinite(model, "t_max", name);
  limits.kp_min = requiredFinite(model, "kp_min", name);
  limits.kp_max = requiredFinite(model, "kp_max", name);
  limits.kd_min = requiredFinite(model, "kd_min", name);
  limits.kd_max = requiredFinite(model, "kd_max", name);
  limits.rotation_direction = requiredFinite(model, "rotation_direction", name);
  if (limits.p_min > limits.p_max || limits.v_min > limits.v_max || limits.t_min > limits.t_max ||
      limits.kp_min > limits.kp_max || limits.kd_min > limits.kd_max || std::abs(limits.rotation_direction) != 1.0)
    throw std::runtime_error("invalid limits for model " + name);
  return limits;
}

}  // namespace

uint8_t MotorMapping::motorTypeForFamily(const std::string& family) {
  if (family == "DM") return MotorRoute::MOTOR_TYPE_DM;
  if (family == "RS") return MotorRoute::MOTOR_TYPE_RS;
  if (family == "EC") return MotorRoute::MOTOR_TYPE_ENC;
  if (family == "MR") return MotorRoute::MOTOR_TYPE_MTE;
  if (family == "LS") return MotorRoute::MOTOR_TYPE_LS;
  throw std::runtime_error("unsupported motor family: " + family);
}

MotorMapping::MotorMapping(const std::string& path) {
  const YAML::Node root = YAML::LoadFile(path);
  const YAML::Node models = root["models"];
  const YAML::Node motors = root["motors"];
  if (!models || !models.IsMap()) throw std::runtime_error("mapping must contain a models map");
  if (!motors || !motors.IsSequence() || motors.size() == 0U || motors.size() > 64U)
    throw std::runtime_error("mapping must contain 1..64 motors");

  struct Model {
    uint8_t type{};
    MotorLimits limits;
  };
  std::unordered_map<std::string, Model> parsed_models;
  for (const auto& item : models)
  {
    const std::string name = item.first.as<std::string>();
    parsed_models.emplace(
        name, Model{motorTypeForFamily(item.second["family"].as<std::string>()), loadLimits(item.second, name)});
  }

  std::unordered_set<uint64_t> physical_routes;
  entries_.reserve(motors.size());
  for (const auto& motor : motors)
  {
    MotorMappingEntry entry;
    entry.joint_name = motor["joint_name"].as<std::string>();
    entry.enabled = motor["enabled"].as<bool>(false);
    const std::string model_name = motor["model"].as<std::string>();
    const auto model = parsed_models.find(model_name);
    if (model == parsed_models.end()) throw std::runtime_error("unknown model for " + entry.joint_name);
    entry.route.motor_type = model->second.type;
    entry.route.bus_index = motor["bus_index"].as<uint8_t>();
    entry.route.motor_id = motor["motor_id"].as<uint32_t>();
    entry.zero_offset = requiredFinite(motor, "zero_offset", entry.joint_name);
    entry.motor_direction = requiredFinite(motor, "motor_direction", entry.joint_name);
    entry.reduction = requiredFinite(motor, "reduction", entry.joint_name);
    entry.joint_lower = requiredFinite(motor, "joint_lower", entry.joint_name);
    entry.joint_upper = requiredFinite(motor, "joint_upper", entry.joint_name);
    entry.limits = model->second.limits;

    if (entry.joint_name.empty() || entry.joint_name.size() > 64U || std::abs(entry.motor_direction) != 1.0 ||
        entry.reduction <= 0.0 || entry.joint_lower > entry.joint_upper)
      throw std::runtime_error("invalid mapping for " + entry.joint_name);
    if (!physical_routes.emplace(physicalRouteKey(entry.route)).second)
      throw std::runtime_error("duplicate motor route for " + entry.joint_name);
    if (!joint_indices_.emplace(entry.joint_name, entries_.size()).second)
      throw std::runtime_error("duplicate joint name: " + entry.joint_name);
    if (entry.enabled) ++enabled_size_;
    entries_.push_back(std::move(entry));
  }
}

const MotorMappingEntry* MotorMapping::findJoint(const std::string& joint_name) const {
  const auto index = joint_indices_.find(joint_name);
  return index == joint_indices_.end() ? nullptr : &entries_[index->second];
}

int MotorMapping::indexOfJoint(const std::string& joint_name) const {
  const auto index = joint_indices_.find(joint_name);
  return index == joint_indices_.end() ? -1 : static_cast<int>(index->second);
}

int MotorMapping::indexOfRoute(const MotorRoute& route) const {
  for (std::size_t i = 0; i < entries_.size(); ++i)
    if (entries_[i].enabled && sameRoute(entries_[i].route, route)) return static_cast<int>(i);
  return -1;
}

luvbot_hardware_interfaces::msg::ActuatorMotorCommand MotorMapping::commandFor(const MotorMappingEntry& mapping,
                                                                               double q, double dq, double tau,
                                                                               double kp, double kd,
                                                                               double wrap_offset) const {
  if (!mapping.enabled) throw std::runtime_error("motor is disabled for " + mapping.joint_name);
  if (!std::isfinite(q) || !std::isfinite(dq) || !std::isfinite(tau) || !std::isfinite(kp) || !std::isfinite(kd) ||
      !std::isfinite(wrap_offset))
    throw std::runtime_error("non-finite command for " + mapping.joint_name);
  const double direction = mapping.motor_direction * mapping.limits.rotation_direction;
  const double reduction_squared = mapping.reduction * mapping.reduction;
  const double joint_q = std::clamp(q, mapping.joint_lower, mapping.joint_upper);

  luvbot_hardware_interfaces::msg::ActuatorMotorCommand command;
  command.route = mapping.route;
  command.q =
      static_cast<float>(std::clamp((joint_q - wrap_offset - mapping.zero_offset) * direction * mapping.reduction,
                                    mapping.limits.p_min, mapping.limits.p_max));
  command.dq =
      static_cast<float>(std::clamp(dq * direction * mapping.reduction, mapping.limits.v_min, mapping.limits.v_max));
  command.tau =
      static_cast<float>(std::clamp(tau * direction / mapping.reduction, mapping.limits.t_min, mapping.limits.t_max));
  command.kp = static_cast<float>(std::clamp(kp / reduction_squared, mapping.limits.kp_min, mapping.limits.kp_max));
  command.kd = static_cast<float>(std::clamp(kd / reduction_squared, mapping.limits.kd_min, mapping.limits.kd_max));
  return command;
}

double MotorMapping::rawJointPosition(const MotorMappingEntry& mapping, double motor_q) const {
  const double direction = mapping.motor_direction * mapping.limits.rotation_direction;
  return motor_q * direction / mapping.reduction + mapping.zero_offset;
}

double MotorMapping::initialWrapOffset(const MotorMappingEntry& mapping, double motor_q, double tolerance) const {
  if (!std::isfinite(motor_q) || !std::isfinite(tolerance) || tolerance < 0.0)
    throw std::runtime_error("invalid initial feedback for " + mapping.joint_name);
  const double raw = rawJointPosition(mapping, motor_q);
  const double full_turn = 2.0 * std::acos(-1.0);
  if (!std::isfinite(raw) || motor_q < mapping.limits.p_min || motor_q > mapping.limits.p_max)
    throw std::runtime_error("initial motor position outside protocol limits for " + mapping.joint_name);
  const double lower = mapping.joint_lower - tolerance;
  const double upper = mapping.joint_upper + tolerance;
  if (raw >= lower && raw <= upper) return 0.0;
  const double first_turn = std::ceil((lower - raw) / full_turn);
  const double last_turn = std::floor((upper - raw) / full_turn);
  if (first_turn != last_turn)
    throw std::runtime_error("no unique feedback turn within joint limits for " + mapping.joint_name);
  const double offset = first_turn * full_turn;
  if (!std::isfinite(offset) || raw + offset < lower || raw + offset > upper)
    throw std::runtime_error("invalid feedback turn for " + mapping.joint_name);
  return offset;
}

JointFeedback MotorMapping::feedbackFor(const MotorMappingEntry& mapping,
                                        const luvbot_hardware_interfaces::msg::MotorState& state,
                                        double wrap_offset) const {
  const double direction = mapping.motor_direction * mapping.limits.rotation_direction;
  JointFeedback feedback;
  feedback.q = rawJointPosition(mapping, state.q) + wrap_offset;
  feedback.dq = state.dq * direction / mapping.reduction;
  feedback.tau = state.tau * direction * mapping.reduction;
  feedback.error = state.error_code;
  feedback.operating_mode = state.operating_mode;
  feedback.valid = state.valid && std::isfinite(state.q) && state.q >= mapping.limits.p_min &&
                   state.q <= mapping.limits.p_max && std::isfinite(feedback.q) && std::isfinite(feedback.dq) &&
                   std::isfinite(feedback.tau) && feedback.q >= mapping.joint_lower - kJointPositionTolerance &&
                   feedback.q <= mapping.joint_upper + kJointPositionTolerance;
  return feedback;
}

JointFeedback MotorMapping::updateFeedback(const MotorMappingEntry& mapping,
                                           const luvbot_hardware_interfaces::msg::MotorState& state,
                                           FeedbackWrapState& wrap) const {
  auto feedback = feedbackFor(mapping, state, wrap.offset);
  if (state.error_code == 0U)
    wrap.error_samples = 0U;
  else if (state.error_code != wrap.error_code)
    wrap.error_samples = 1U;
  else
    wrap.error_samples = std::min(wrap.error_samples + 1U, kMotorErrorFaultSamples);
  wrap.error_code = state.error_code;
  const bool persistent_error = wrap.error_samples >= kMotorErrorFaultSamples;
  if (wrap.fault || !state.valid || persistent_error || !std::isfinite(state.q) || !std::isfinite(state.dq) ||
      !std::isfinite(state.tau) || state.q < mapping.limits.p_min || state.q > mapping.limits.p_max)
  {
    wrap.stable_samples = 0U;
    wrap.fault = wrap.fault || wrap.locked || persistent_error;
    feedback.valid = false;
    return feedback;
  }

  if (state.error_code != 0U)
  {
    // A transient error must neither confirm/rebase a turn nor hide invalid active feedback.
    if (wrap.locked && !feedback.valid)
    {
      wrap.stable_samples = 0U;
      wrap.fault = true;
    }
    else
    {
      if (!wrap.ready()) wrap.stable_samples = 0U;
      feedback.error_pending = true;
      feedback.initializing = !wrap.ready();
    }
    feedback.valid = false;
    return feedback;
  }

  // Only startup/damping may change turns; powered control must never silently rebase a target.
  if (!wrap.locked && (wrap.stable_samples == 0U || !feedback.valid))
  {
    try
    {
      const double offset = initialWrapOffset(mapping, state.q);
      if (offset != wrap.offset) wrap.stable_samples = 0U;
      wrap.offset = offset;
      feedback = feedbackFor(mapping, state, wrap.offset);
    } catch (const std::exception&)
    {
      wrap.stable_samples = 0U;
      feedback.valid = false;
      feedback.initializing = true;
      return feedback;
    }
  }
  if (!feedback.valid)
  {
    wrap.stable_samples = 0U;
    wrap.fault = wrap.locked;
    feedback.initializing = !wrap.locked;
    return feedback;
  }
  wrap.stable_samples = std::min(wrap.stable_samples + 1U, 3U);
  feedback.valid = wrap.ready();
  feedback.initializing = !feedback.valid;
  return feedback;
}

}  // namespace system_command_arbiter
