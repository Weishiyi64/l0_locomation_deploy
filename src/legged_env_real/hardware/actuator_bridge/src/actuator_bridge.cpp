#include "actuator_bridge/actuator_bridge.h"

#include <socketcan_wrapper/can_error_parser.h>
#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <boost/bind.hpp>
#include <chrono>
#include <cmath>
#include <typeinfo>
#include <unordered_map>
#include <unordered_set>
#include <utility>

#include "actuator_bridge/config_loader.h"
#include "actuator_bridge/coordinate_mapping.h"
#include "actuator_bridge/driver_registry.h"
#include "actuator_bridge/drivers/dm_driver.h"
#include "actuator_bridge/drivers/ec_driver.h"
#include "actuator_bridge/drivers/motorevo_driver.h"
#include "actuator_bridge/drivers/rs_driver.h"

namespace {
void ensureDriversRegistered() {
  static bool done = false;
  if (done) return;
  auto& reg = actuator_bridge::DriverRegistry::instance();
  reg.registerFactory("RS", []() { return std::make_shared<actuator_bridge::RSDriver>(); });
  reg.registerFactory("DM", []() { return std::make_shared<actuator_bridge::DMDriver>(); });
  reg.registerFactory("EC", []() { return std::make_shared<actuator_bridge::ECDriver>(); });
  reg.registerFactory("MR", []() { return std::make_shared<actuator_bridge::MotorevoDriver>(); });
  done = true;
}

// Fallback order for legacy launches that do not pass RobotCfg.joint_order_mj.
int fallbackJointRank(const std::string& name) {
  static const char* const kOrder[] = {
      "left_hip_pitch_joint",      "left_hip_roll_joint",      "left_hip_yaw_joint",      "left_knee_joint",
      "left_ankle_pitch_joint",    "left_ankle_roll_joint",    "right_hip_pitch_joint",   "right_hip_roll_joint",
      "right_hip_yaw_joint",       "right_knee_joint",         "right_ankle_pitch_joint", "right_ankle_roll_joint",
      "left_shoulder_pitch_joint", "left_shoulder_roll_joint", "left_elbow_roll_joint",   "right_shoulder_pitch_joint",
      "right_shoulder_roll_joint", "right_elbow_roll_joint",   "neck_yaw_joint",          "neck_pitch_joint",
      "neck_roll_joint",
  };
  for (size_t i = 0; i < sizeof(kOrder) / sizeof(kOrder[0]); ++i)
  {
    if (name == kOrder[i]) return static_cast<int>(i);
  }
  return 100000;
}

bool loadJointOrderMjFromRobotConfig(const std::shared_ptr<rclcpp::Node>& node,
                                     std::vector<std::string>& joint_order_mj) {
  joint_order_mj.clear();

  std::string robot_cfg_path;
  if (!node->get_parameter("robot_cfg_path", robot_cfg_path) || robot_cfg_path.empty())
  {
    RCLCPP_INFO(node->get_logger(), "ActuatorBridge: robot_cfg_path not set, using actuator config order");
    return true;
  }

  YAML::Node root;
  try
  { root = YAML::LoadFile(robot_cfg_path); } catch (const YAML::Exception& e)
  {
    RCLCPP_ERROR(node->get_logger(), "ActuatorBridge: failed to load RobotCfg YAML '%s': %s", robot_cfg_path.c_str(),
                 e.what());
    return false;
  }

  const YAML::Node robot_cfg = root["RobotCfg"];
  if (!robot_cfg || !robot_cfg.IsMap())
  {
    RCLCPP_ERROR(node->get_logger(), "ActuatorBridge: RobotCfg missing or invalid in '%s'", robot_cfg_path.c_str());
    return false;
  }

  const YAML::Node joint_order = robot_cfg["joint_order_mj"];
  if (!joint_order || !joint_order.IsSequence())
  {
    RCLCPP_ERROR(node->get_logger(), "ActuatorBridge: RobotCfg.joint_order_mj missing or invalid in '%s'",
                 robot_cfg_path.c_str());
    return false;
  }

  std::unordered_set<std::string> seen;
  for (std::size_t i = 0; i < joint_order.size(); ++i)
  {
    if (!joint_order[i].IsScalar())
    {
      RCLCPP_ERROR(node->get_logger(), "ActuatorBridge: RobotCfg.joint_order_mj[%zu] must be a string", i);
      return false;
    }
    std::string name = joint_order[i].as<std::string>();
    if (name.empty())
    {
      RCLCPP_ERROR(node->get_logger(), "ActuatorBridge: RobotCfg.joint_order_mj[%zu] is empty", i);
      return false;
    }
    if (!seen.insert(name).second)
    {
      RCLCPP_ERROR(node->get_logger(), "ActuatorBridge: duplicate joint '%s' in RobotCfg.joint_order_mj", name.c_str());
      return false;
    }
    joint_order_mj.push_back(std::move(name));
  }

  if (joint_order_mj.empty())
  {
    RCLCPP_ERROR(node->get_logger(), "ActuatorBridge: RobotCfg.joint_order_mj is empty in '%s'",
                 robot_cfg_path.c_str());
    return false;
  }

  RCLCPP_INFO(node->get_logger(), "ActuatorBridge: using RobotCfg.joint_order_mj from '%s' (%zu joints)",
              robot_cfg_path.c_str(), joint_order_mj.size());
  return true;
}
}  // namespace

namespace actuator_bridge {

ActuatorBridge::ActuatorBridge(std::shared_ptr<rclcpp::Node> node) : node_(std::move(node)) {}

ActuatorBridge::~ActuatorBridge() {
  if (shutdown_disable_all_)
  {
    disableAll();
  }
  for (auto& bus : buses_)
  {
    if (bus.can_manager)
    {
      bus.can_manager->shutdown();
    }
  }
  RCLCPP_INFO(node_->get_logger(), "\033[35m[ActuatorBridge] Shutting down complete\033[0m");
}

bool ActuatorBridge::loadFromParams() {
  ensureDriversRegistered();
  std::vector<BusConfig> buses_config;
  // load configuration file path
  std::string cfg_path;
  if (!node_->get_parameter("config_file", cfg_path))
  {
    RCLCPP_ERROR(node_->get_logger(), "parameter 'config_file' not set");
    return false;
  }
  if (!ConfigLoader::loadBusesConfig(cfg_path, buses_config))
  {
    return false;
  }
  return buildFromBusesConfig(buses_config);
}

std::vector<std::string> ActuatorBridge::motorNames() const {
  std::vector<std::string> names;
  names.reserve(flat_.size());
  for (const auto& flat_motor : flat_)
  {
    names.push_back(flat_motor.name);
  }
  return names;
}

int ActuatorBridge::indexOf(const std::string& name) const {
  // 先查找name
  auto name_result = name_to_flat_.find(name);
  if (name_result != name_to_flat_.end())
  {
    return name_result->second;
  }

  // 再查找alias
  auto alias_result = alias_to_flat_.find(name);
  if (alias_result != alias_to_flat_.end())
  {
    return alias_result->second;
  }

  return -1;
}

std::vector<std::string> ActuatorBridge::getJointNamesByTag(const std::string& tag) const {
  std::vector<std::string> result;
  if (tag.empty())
  {
    return result;  // 空tag不匹配任何关节
  }

  for (size_t i = 0; i < flat_.size(); ++i)
  {
    if (tags_[i] == tag)
    {
      result.push_back(flat_[i].name);
    }
  }

  return result;
}

void ActuatorBridge::sendFrame(Bus& bus, const can_msgs::msg::Frame& frame) {
  if (!bus.can_manager) return;

  bool success = bus.can_manager->send(frame.id, frame.data.data(), frame.dlc, /*is_extended=*/frame.is_extended);
  if (!success)
  {
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000, "Failed to send CAN frame on bus %s (ID=0x%X)",
                         bus.can_device.c_str(), frame.id);
  }
}

int ActuatorBridge::ensureBus(const std::string& vendor, const std::string& can_device) {
  auto existing_bus = bus_index_by_key_.find(can_device);
  if (existing_bus != bus_index_by_key_.end())
  {
    int existing_bus_idx = existing_bus->second;
    if (buses_[existing_bus_idx].drivers.find(vendor) == buses_[existing_bus_idx].drivers.end())
    {
      auto new_driver = DriverRegistry::instance().create(vendor);
      if (!new_driver)
      {
        throw std::runtime_error("unsupported vendor: " + vendor);
      }
      buses_[existing_bus_idx].drivers[vendor] = new_driver;
      RCLCPP_INFO(node_->get_logger(), "ActuatorBridge: added vendor=%s to existing bus %d (device=%s)", vendor.c_str(),
                  existing_bus_idx, can_device.c_str());
    }
    return existing_bus_idx;
  }

  Bus new_bus;
  new_bus.can_device = can_device;
  auto new_driver = DriverRegistry::instance().create(vendor);
  if (!new_driver)
  {
    throw std::runtime_error("unsupported vendor: " + vendor);
  }
  new_bus.drivers[vendor] = new_driver;

  int new_bus_idx = static_cast<int>(buses_.size());
  RCLCPP_INFO(node_->get_logger(), "ActuatorBridge: created bus %d for vendor=%s, device=%s", new_bus_idx,
              vendor.c_str(), can_device.c_str());

  buses_.push_back(std::move(new_bus));
  bus_index_by_key_[can_device] = new_bus_idx;
  return new_bus_idx;
}

bool ActuatorBridge::buildFromBusesConfig(const std::vector<BusConfig>& buses_config) {
  // 清空现有数据
  buses_.clear();
  bus_index_by_key_.clear();
  name_to_bus_.clear();
  name_to_id_.clear();
  flat_.clear();
  name_to_flat_.clear();
  alias_to_flat_.clear();
  states_.clear();
  pending_cmds_.clear();
  has_pending_cmd_.clear();
  sent_.clear();
  receive_.clear();
  zero_offset_.clear();
  motor_direction_.clear();
  reduction_.clear();
  rotation_direction_.clear();
  joint_lower_limit_.clear();
  joint_upper_limit_.clear();
  feedback_wrap_offset_.clear();
  feedback_wrap_initialized_.clear();
  tags_.clear();
  mode_.clear();

  // 构建总线和执行器
  for (const auto& bus_config : buses_config)
  {
    for (const auto& actuator : bus_config.actuators)
    {
      int bus_idx = ensureBus(bus_config.vendor, actuator.can_device);

      buses_[bus_idx].actuators.push_back(actuator);
      name_to_bus_[actuator.name] = bus_idx;
      name_to_id_[actuator.name] = actuator.id;

      // 设置限制参数到对应 vendor 的驱动
      buses_[bus_idx].drivers.at(bus_config.vendor)->setLimits(actuator.id, actuator.limits);

      if (actuator.mode == ActuatorMode::DISABLE)
      {
        RCLCPP_INFO(node_->get_logger(), "Actuator '%s' registered as virtual motor (mode: disable)",
                    actuator.name.c_str());
      }
      else if (actuator.mode == ActuatorMode::FAKE)
      {
        RCLCPP_INFO(node_->get_logger(), "Actuator '%s' registered as fake motor (mode: fake)", actuator.name.c_str());
      }
    }
  }

  auto add_flat_entry = [&](const ActuatorSpec& actuator_spec, size_t bus_index) -> bool {
    FlatIndex new_flat_entry;
    new_flat_entry.name = actuator_spec.name;
    new_flat_entry.vendor = actuator_spec.vendor;
    new_flat_entry.id = actuator_spec.id;
    new_flat_entry.bus = static_cast<int>(bus_index);
    int new_flat_idx = static_cast<int>(flat_.size());
    name_to_flat_[new_flat_entry.name] = new_flat_idx;
    flat_.push_back(std::move(new_flat_entry));

    // 可选配置
    if (!actuator_spec.alias.empty())
    {
      if (alias_to_flat_.count(actuator_spec.alias))
      {
        RCLCPP_ERROR(node_->get_logger(), "Duplicate alias '%s' for joint '%s'", actuator_spec.alias.c_str(),
                     actuator_spec.name.c_str());
        return false;
      }
      alias_to_flat_[actuator_spec.alias] = new_flat_idx;
    }
    return true;
  };

  std::vector<std::string> joint_order_mj;
  if (!loadJointOrderMjFromRobotConfig(node_, joint_order_mj))
  {
    return false;
  }

  if (!joint_order_mj.empty())
  {
    std::unordered_map<std::string, std::pair<size_t, size_t>> actuator_index_by_name;
    for (size_t b = 0; b < buses_.size(); ++b)
    {
      for (size_t a = 0; a < buses_[b].actuators.size(); ++a)
      {
        const std::string& name = buses_[b].actuators[a].name;
        if (!actuator_index_by_name.emplace(name, std::make_pair(b, a)).second)
        {
          RCLCPP_ERROR(node_->get_logger(), "Duplicate actuator name detected while building flat index: %s",
                       name.c_str());
          return false;
        }
      }
    }

    std::unordered_set<std::string> ordered_names;
    for (const auto& joint_name : joint_order_mj)
    {
      if (!ordered_names.insert(joint_name).second)
      {
        RCLCPP_ERROR(node_->get_logger(), "Duplicate joint '%s' in RobotCfg.joint_order_mj", joint_name.c_str());
        return false;
      }

      const auto actuator_it = actuator_index_by_name.find(joint_name);
      if (actuator_it == actuator_index_by_name.end())
      {
        RCLCPP_ERROR(node_->get_logger(),
                     "ActuatorBridge: RobotCfg.joint_order_mj contains '%s', but actuator config has no matching "
                     "actuator",
                     joint_name.c_str());
        return false;
      }

      const size_t bus_index = actuator_it->second.first;
      const size_t actuator_index = actuator_it->second.second;
      if (!add_flat_entry(buses_[bus_index].actuators[actuator_index], bus_index)) return false;
    }

    for (const auto& item : actuator_index_by_name)
    {
      if (ordered_names.find(item.first) == ordered_names.end())
      {
        RCLCPP_ERROR(node_->get_logger(),
                     "ActuatorBridge: actuator config contains '%s', but RobotCfg.joint_order_mj does not",
                     item.first.c_str());
        return false;
      }
    }
  }
  else
  {
    for (auto& bus : buses_)
    {
      std::stable_sort(bus.actuators.begin(), bus.actuators.end(), [](const ActuatorSpec& a, const ActuatorSpec& b) {
        return fallbackJointRank(a.name) < fallbackJointRank(b.name);
      });
    }

    // 构建扁平索引
    for (size_t b = 0; b < buses_.size(); ++b)
    {
      for (const auto& actuator_spec : buses_[b].actuators)
      {
        if (!add_flat_entry(actuator_spec, b)) return false;
      }
    }
  }

  // 初始化运行态容器
  states_.resize(flat_.size());
  pending_cmds_.resize(flat_.size());
  has_pending_cmd_.assign(flat_.size(), false);
  sent_.assign(flat_.size(), false);
  receive_.assign(flat_.size(), false);

  // 初始化校准参数
  zero_offset_.assign(flat_.size(), 0.0f);
  motor_direction_.assign(flat_.size(), 0.0f);
  reduction_.assign(flat_.size(), 0.0f);
  rotation_direction_.assign(flat_.size(), 0.0f);
  joint_lower_limit_.assign(flat_.size(), 0.0f);
  joint_upper_limit_.assign(flat_.size(), 0.0f);
  feedback_wrap_offset_.assign(flat_.size(), 0.0f);
  feedback_wrap_initialized_.assign(flat_.size(), false);
  tags_.assign(flat_.size(), "");
  mode_.assign(flat_.size(), ActuatorMode::ENABLE);

  // 对齐扁平索引
  for (size_t i = 0; i < flat_.size(); ++i)
  {
    const std::string& name = flat_[i].name;
    int bus_idx = flat_[i].bus;

    for (const auto& spec : buses_[bus_idx].actuators)
    {
      if (spec.name == name)
      {
        zero_offset_[i] = spec.zero_offset;
        motor_direction_[i] = spec.motor_direction;
        reduction_[i] = spec.reduction;
        rotation_direction_[i] = spec.limits.rotation_direction;
        joint_lower_limit_[i] = spec.joint_lower_limit;
        joint_upper_limit_[i] = spec.joint_upper_limit;
        tags_[i] = spec.tag;
        mode_[i] = spec.mode;
        break;
      }
    }
  }

  // 初始化 SocketCAN 接口
  for (size_t b = 0; b < buses_.size(); ++b)
  {
    auto& bus = buses_[b];

    // 检查这个总线上是否有任何启用的电机
    bool has_enabled_motor = false;
    for (const auto& actuator : bus.actuators)
    {
      if (actuator.mode == ActuatorMode::ENABLE)
      {
        has_enabled_motor = true;
        break;
      }
    }

    // 如果所有电机都被禁用，跳过 SocketCAN 初始化
    if (!has_enabled_motor)
    {
      RCLCPP_INFO(node_->get_logger(),
                  "Bus %zu: All actuators disabled (device=%s) - skipping "
                  "SocketCAN initialization",
                  b, bus.can_device.c_str());
      continue;
    }

    // 初始化 SocketCAN
    bus.can_manager = std::make_shared<socketcan_wrapper::SocketCANManager>();
    if (!bus.can_manager->init(bus.can_device, false))
    {
      RCLCPP_ERROR(node_->get_logger(), "Failed to initialize SocketCAN for bus %zu (device: %s)", b,
                   bus.can_device.c_str());
    }

    // 设置接收回调
    int bus_idx = static_cast<int>(b);
    bus.can_manager->setReceiveCallback(
        [this, bus_idx](const can::Frame& frame) { this->CANRxCallback(frame, bus_idx); });

    std::string vendors_str;
    for (const auto& [vendor_name, drv] : bus.drivers)
    {
      if (!vendors_str.empty()) vendors_str += ",";
      vendors_str += vendor_name;
    }
    RCLCPP_INFO(node_->get_logger(), "Bus %zu: SocketCAN initialized (device=%s, vendors=[%s], actuators=%zu)", b,
                bus.can_device.c_str(), vendors_str.c_str(), bus.actuators.size());
  }

  RCLCPP_INFO(node_->get_logger(), "ActuatorBridge: loaded %zu actuators on %zu buses", flat_.size(), buses_.size());
  for (size_t i = 0; i < flat_.size(); ++i)
  {
    const auto& flat_motor = flat_[i];
    RCLCPP_INFO(node_->get_logger(), "  [%zu] %s: bus=%d, id=0x%X, vendor=%s, can_device=%s", i,
                flat_motor.name.c_str(), flat_motor.bus, flat_motor.id, flat_motor.vendor.c_str(),
                buses_[flat_motor.bus].can_device.c_str());
  }

  return true;
}

bool ActuatorBridge::enableAll() {
  double WaitSec = 0.1;
  int MaxRounds = 40;

  for (int round = 0; round < MaxRounds; ++round)
  {
    std::vector<size_t> need_retry;
    need_retry.reserve(flat_.size());
    for (size_t i = 0; i < flat_.size(); ++i)
    {
      if (mode_[i] != ActuatorMode::ENABLE) continue;
      JointState js = getJointState(i);
      if (js.state == 1) continue;
      need_retry.push_back(i);
    }
    if (need_retry.empty())
    {
      RCLCPP_INFO(node_->get_logger(), "All motors enabled successfully");
      return true;
    }
    if (round > 0)
    {
      for (size_t idx : need_retry)
      {
        RCLCPP_WARN(node_->get_logger(), "Re-enabling motor '%s' (round %d): previous verify failed",
                    flat_[idx].name.c_str(), round);
      }
    }
    for (size_t idx : need_retry)
    {
      setReceiveFlag(idx, false);
      enable(idx);
    }
    rclcpp::sleep_for(std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(WaitSec)));
    for (size_t idx : need_retry)
    {
      bool rx = getReceiveFlag(idx);
      JointState js = getJointState(idx);
      bool ok = rx && (js.state == 1);
      if (!ok)
      {
        if (!rx)
        {
          RCLCPP_WARN(node_->get_logger(), "Motor '%s': no CAN feedback in %.1fs window", flat_[idx].name.c_str(),
                      WaitSec);
        }
        else
        {
          RCLCPP_WARN(node_->get_logger(), "Motor '%s': state=0x%X (expected state=1 for enabled)",
                      flat_[idx].name.c_str(), js.state);
        }
      }
    }
  }
  RCLCPP_ERROR(node_->get_logger(), "enableAll() failed: not all motors reached enabled state within %d rounds",
               MaxRounds);
  return false;
}

void ActuatorBridge::disableAll() {
  for (size_t i = 0; i < flat_.size(); ++i)
  {
    disable(i);
  }

  for (size_t i = 0; i < flat_.size(); ++i)
  {
    if (mode_[i] != ActuatorMode::ENABLE) continue;
    {
      std::lock_guard<std::mutex> lock(states_mutex_);
      states_[i].state = 0;
    }
    setReceiveFlag(i, false);
  }
}

void ActuatorBridge::resetAll() {
  for (size_t i = 0; i < flat_.size(); ++i)
  {
    reset(i);
  }
}

void ActuatorBridge::zeroAll() {
  for (size_t i = 0; i < flat_.size(); ++i)
  {
    zero(i);
  }
}

void ActuatorBridge::enable(size_t idx) {
  const FlatIndex* target_motor = getFlatIndex(idx);
  if (!target_motor) return;

  // 仅对真实电机发送指令
  if (mode_[idx] != ActuatorMode::ENABLE)
  {
    return;
  }

  auto& bus = buses_[target_motor->bus];
  auto& drv = bus.drivers.at(target_motor->vendor);
  can_msgs::msg::Frame frame_to_send;
  drv->makeEnable(target_motor->id, frame_to_send);
  sendFrame(bus, frame_to_send);
}

void ActuatorBridge::disable(size_t idx) {
  const FlatIndex* target_motor = getFlatIndex(idx);
  if (!target_motor) return;

  // 仅对真实电机发送指令
  if (mode_[idx] != ActuatorMode::ENABLE)
  {
    return;
  }

  auto& bus = buses_[target_motor->bus];
  auto& drv = bus.drivers.at(target_motor->vendor);
  can_msgs::msg::Frame frame_to_send;
  drv->makeDisable(target_motor->id, frame_to_send);
  sendFrame(bus, frame_to_send);
}

void ActuatorBridge::reset(size_t idx) {
  const FlatIndex* target_motor = getFlatIndex(idx);
  if (!target_motor) return;

  feedback_wrap_offset_[idx] = 0.0f;
  feedback_wrap_initialized_[idx] = false;

  // 仅对真实电机发送指令
  if (mode_[idx] != ActuatorMode::ENABLE)
  {
    return;
  }

  auto& bus = buses_[target_motor->bus];
  auto& drv = bus.drivers.at(target_motor->vendor);
  if (!drv->supportsReset()) return;

  can_msgs::msg::Frame frame_to_send;
  drv->makeReset(target_motor->id, frame_to_send);
  sendFrame(bus, frame_to_send);
}

void ActuatorBridge::zero(size_t idx) {
  const FlatIndex* target_motor = getFlatIndex(idx);
  if (!target_motor) return;

  // 仅对真实电机发送指令
  if (mode_[idx] != ActuatorMode::ENABLE)
  {
    return;
  }

  auto& bus = buses_[target_motor->bus];
  auto& drv = bus.drivers.at(target_motor->vendor);
  can_msgs::msg::Frame frame_to_send;
  drv->makeZero(target_motor->id, frame_to_send);
  sendFrame(bus, frame_to_send);
}

void ActuatorBridge::setCommand(size_t idx, const JointCmd& cmd) {
  // disable 模式：完全跳过，不存储命令
  if (mode_[idx] == ActuatorMode::DISABLE)
  {
    return;
  }

  if (idx >= pending_cmds_.size() || idx >= has_pending_cmd_.size()) return;

  std::lock_guard<std::mutex> lock(cmd_mutex_);
  pending_cmds_[idx] = cmd;
  has_pending_cmd_[idx] = true;

  // fake 模式：立即将指令值透传为反馈状态
  if (mode_[idx] == ActuatorMode::FAKE)
  {
    JointState fake_state;
    fake_state.pos = cmd.pos_d;
    fake_state.vel = cmd.vel_d;
    fake_state.tau = cmd.tau_d;
    fake_state.state = static_cast<uint16_t>(0x1);
    fake_state.error = static_cast<uint16_t>(0x0);
    std::lock_guard<std::mutex> lock_state(states_mutex_);
    states_[idx] = fake_state;
  }
}

void ActuatorBridge::flush() {
  for (size_t i = 0; i < flat_.size(); ++i)
  {
    // disable 模式：完全跳过
    if (mode_[i] == ActuatorMode::DISABLE)
    {
      continue;
    }

    // fake 模式：无需发送 CAN，标记发送完成并立即置接收标志
    if (mode_[i] == ActuatorMode::FAKE)
    {
      bool has_cmd = false;
      {
        std::lock_guard<std::mutex> lock(cmd_mutex_);
        has_cmd = (i < has_pending_cmd_.size()) && has_pending_cmd_[i];
        if (has_cmd) has_pending_cmd_[i] = false;
      }
      if (has_cmd)
      {
        {
          std::lock_guard<std::mutex> lock(sent_mutex_);
          sent_[i] = true;
        }
        {
          std::lock_guard<std::mutex> lock(receive_mutex_);
          receive_[i] = true;
        }
      }
      continue;
    }
    bool has_cmd = false;
    JointCmd cmd;

    // 读取待发送命令
    {
      std::lock_guard<std::mutex> lock(cmd_mutex_);
      if (i >= has_pending_cmd_.size() || !has_pending_cmd_[i]) continue;
      has_cmd = has_pending_cmd_[i];
      cmd = pending_cmds_[i];
    }

    if (!has_cmd) continue;

    const auto& target_motor = flat_[i];
    auto& bus = buses_[target_motor.bus];
    auto& drv = bus.drivers.at(target_motor.vendor);

    MotorCmd motor_cmd;
    motor_cmd.name = target_motor.name;
    motor_cmd.id = target_motor.id;

    // 关节 -> 电机坐标变换
    if (!CoordinateTransformer::transformCommand(cmd, motor_cmd, zero_offset_[i], motor_direction_[i], reduction_[i],
                                                 rotation_direction_[i], feedback_wrap_offset_[i]))
    {
      RCLCPP_ERROR_THROTTLE(node_->get_logger(), *node_->get_clock(), 1.0,
                            "Invalid transform parameters for motor %s (motor_direction=%.3f, "
                            "reduction=%.3f, rotation_direction=%.3f)",
                            target_motor.name.c_str(), motor_direction_[i], reduction_[i], rotation_direction_[i]);
      std::lock_guard<std::mutex> lock(cmd_mutex_);
      has_pending_cmd_[i] = false;
      continue;
    }

    // 编码并发送最新一帧命令
    can_msgs::msg::Frame frame_to_send;
    drv->encodeCommand(motor_cmd, frame_to_send);
    frame_to_send.header.stamp = node_->get_clock()->now();

    sendFrame(bus, frame_to_send);

    // 更新标志位
    {
      std::lock_guard<std::mutex> lock(sent_mutex_);
      sent_[i] = true;
    }
    {
      std::lock_guard<std::mutex> lock(receive_mutex_);
      receive_[i] = false;
    }
    {
      std::lock_guard<std::mutex> lock(cmd_mutex_);
      has_pending_cmd_[i] = false;
    }
  }
}

void ActuatorBridge::CANRxCallback(const can::Frame& frame, int bus_idx) {
  if (bus_idx < 0 || bus_idx >= static_cast<int>(buses_.size())) return;

  auto& bus = buses_[bus_idx];

  // 转换为 can_msgs::msg::Frame
  // 注意：frame 已经在 SocketCANManager 层被深拷贝，这里是安全的
  can_msgs::msg::Frame can_frame;
  can_frame.id = frame.id;
  can_frame.dlc = frame.dlc;
  can_frame.is_extended = frame.is_extended;
  can_frame.is_rtr = frame.is_rtr;
  can_frame.is_error = frame.is_error;
  std::copy(frame.data.begin(), frame.data.end(), can_frame.data.begin());
  can_frame.header.stamp = node_->get_clock()->now();

  auto frame_ptr = std::make_shared<can_msgs::msg::Frame>(can_frame);

  // 过滤错误帧
  if (frame_ptr->is_error)
  {
    //     std::string error_details =
    //     socketcan_wrapper::parseCanRxErrorFrame(frame_ptr);
    //     RCLCPP_ERROR_THROTTLE(node_->get_logger(), *node_->get_clock(), 0.1,
    //     "[%s] CAN Error Frame - ID=0x%X, extended=%d, dlc=%d, data=[%02X %02X
    //     %02X %02X %02X %02X %02X %02X], Details: %s",
    //                       bus.can_device.c_str(), frame_ptr->id,
    //                       frame_ptr->is_extended, frame_ptr->dlc,
    //                       frame_ptr->data[0], frame_ptr->data[1],
    //                       frame_ptr->data[2], frame_ptr->data[3],
    //                       frame_ptr->data[4], frame_ptr->data[5],
    //                       frame_ptr->data[6], frame_ptr->data[7],
    //                       error_details.c_str());
    return;
  }

  // 通过帧特征匹配对应can_device上的关节名称
  std::string matched_vendor;
  std::string matched_name;
  int motor_id = -1;

  if (frame_ptr->is_extended)
  {
    const int type = (int)((frame_ptr->id & 0x3F000000) >> 24);
    if (type == 0x2 && bus.drivers.count("RS"))
    {
      int rs_id = (int)((frame_ptr->id & 0xFF00) >> 8);
      for (const auto& actuator_spec : bus.actuators)
      {
        if (actuator_spec.vendor == "RS" && actuator_spec.id == rs_id)
        {
          motor_id = rs_id;
          matched_vendor = "RS";
          matched_name = actuator_spec.name;
          break;
        }
      }
    }
  }
  else if (frame_ptr->dlc == 8)
  {
    if (bus.drivers.count("DM"))
    {
      int dm_id = (int)(frame_ptr->id) - 0x10;  // MST_ID = ID + 0x10
      for (const auto& actuator_spec : bus.actuators)
      {
        if (actuator_spec.vendor == "DM" && actuator_spec.id == dm_id)
        {
          motor_id = dm_id;
          matched_vendor = "DM";
          matched_name = actuator_spec.name;
          break;
        }
      }
    }
    if (matched_vendor.empty() && bus.drivers.count("MR"))
    {
      uint8_t motorevo_status = frame_ptr->data[0];
      if (motorevo_status <= 0x05)
      {
        int motorevo_id = (int)(frame_ptr->id);
        for (const auto& actuator_spec : bus.actuators)
        {
          if (actuator_spec.vendor == "MR" && actuator_spec.id == motorevo_id)
          {
            motor_id = motorevo_id;
            matched_vendor = "MR";
            matched_name = actuator_spec.name;
            break;
          }
        }
      }
    }
    if (matched_vendor.empty() && bus.drivers.count("EC"))
    {
      uint8_t ack_status = frame_ptr->data[0] >> 5;
      if (ack_status == 1)
      {
        int ec_id = (int)(frame_ptr->id);
        for (const auto& actuator_spec : bus.actuators)
        {
          if (actuator_spec.vendor == "EC" && actuator_spec.id == ec_id)
          {
            motor_id = ec_id;
            matched_vendor = "EC";
            matched_name = actuator_spec.name;
            break;
          }
        }
      }
    }
  }

  if (matched_vendor.empty() || motor_id < 0)
  {
    // RS 上电帧（type=0x0）静默忽略
    if (bus.drivers.count("RS") && frame_ptr->is_extended)
    {
      const int type = (int)((frame_ptr->id & 0x3F000000) >> 24);
      if (type == 0x0) return;
    }
    // RS 故障反馈帧（type=0x15）静默忽略
    if (bus.drivers.count("RS") && frame_ptr->is_extended)
    {
      const int type = (int)((frame_ptr->id & 0x3F000000) >> 24);
      if (type == 0x15) return;
    }
    // EC 调零反馈帧（id=0x7FF & dlc=4）静默忽略
    if (bus.drivers.count("EC") && frame_ptr->id == 0x7FF)
    {
      if (frame_ptr->dlc == 4) return;
    }
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1.0,
                         "Failed to parse CAN frame from %s: ID=0x%X, extended=%d, dlc=%d, data=[%02X %02X %02X %02X "
                         "%02X %02X %02X %02X]",
                         bus.can_device.c_str(), frame_ptr->id, frame_ptr->is_extended, frame_ptr->dlc,
                         frame_ptr->data[0], frame_ptr->data[1], frame_ptr->data[2], frame_ptr->data[3],
                         frame_ptr->data[4], frame_ptr->data[5], frame_ptr->data[6], frame_ptr->data[7]);
    return;
  }

  MotorState received_motor_state;
  received_motor_state.id = motor_id;
  received_motor_state.vendor = matched_vendor;
  received_motor_state.name = matched_name;
  bus.drivers.at(matched_vendor)->parseFeedback(*frame_ptr, received_motor_state);

  // 查找电机在扁平列表中的位置
  auto flat_search = name_to_flat_.find(received_motor_state.name);
  if (flat_search == name_to_flat_.end())
  {
    RCLCPP_ERROR_THROTTLE(node_->get_logger(), *node_->get_clock(), 1.0,
                          "Motor name '%s' (ID=0x%X) found in bus config but not in flat index.",
                          received_motor_state.name.c_str(), received_motor_state.id);
    return;
  }
  size_t flat_idx = static_cast<size_t>(flat_search->second);

  JointState joint_state;

  if (!feedback_wrap_initialized_[flat_idx])
  {
    constexpr float joint_tolerance = 0.1f;  // rad
    float raw_joint_pos = 0.0f;
    if (CoordinateTransformer::transformRawFeedbackPosition(raw_joint_pos, received_motor_state, zero_offset_[flat_idx],
                                                            motor_direction_[flat_idx], reduction_[flat_idx],
                                                            rotation_direction_[flat_idx]))
    {
      if (joint_lower_limit_[flat_idx] > joint_upper_limit_[flat_idx])
      {
        RCLCPP_ERROR(node_->get_logger(),
                     "CoordinateTransformer: invalid joint limits [%.3f, %.3f], keeping raw position for startup wrap",
                     joint_lower_limit_[flat_idx], joint_upper_limit_[flat_idx]);
      }
      else
      {
        if (raw_joint_pos < joint_lower_limit_[flat_idx] - joint_tolerance)
        {
          feedback_wrap_offset_[flat_idx] = static_cast<float>(2.0 * M_PI);
        }
        else if (raw_joint_pos > joint_upper_limit_[flat_idx] + joint_tolerance)
        {
          feedback_wrap_offset_[flat_idx] = static_cast<float>(-2.0 * M_PI);
        }
      }
    }

    feedback_wrap_initialized_[flat_idx] = true;
  }

  if (!CoordinateTransformer::transformFeedback(joint_state, received_motor_state, zero_offset_[flat_idx],
                                                motor_direction_[flat_idx], reduction_[flat_idx],
                                                rotation_direction_[flat_idx], feedback_wrap_offset_[flat_idx]))
  {
    RCLCPP_ERROR_THROTTLE(
        node_->get_logger(), *node_->get_clock(), 1.0,
        "Invalid transform parameters for motor %s (motor_direction=%.3f, reduction=%.3f, rotation_direction=%.3f)",
        received_motor_state.name.c_str(), motor_direction_[flat_idx], reduction_[flat_idx],
        rotation_direction_[flat_idx]);
    return;
  }

  // disable 模式：强制反馈为零；fake 模式
  if (mode_[flat_idx] == ActuatorMode::DISABLE)
  {
    joint_state.pos = 0.0f;
    joint_state.vel = 0.0f;
    joint_state.tau = 0.0f;
    joint_state.state = static_cast<uint16_t>(0x1);
    joint_state.error = static_cast<uint16_t>(0x0);
  }

  // 仅对 enable/disable 模式更新 states_
  if (mode_[flat_idx] != ActuatorMode::FAKE)
  {
    std::lock_guard<std::mutex> lock_state(states_mutex_);
    states_[flat_idx] = joint_state;
  }

  {
    std::lock_guard<std::mutex> lock_receive(receive_mutex_);
    receive_[flat_idx] = true;
  }
}

}  // namespace actuator_bridge
