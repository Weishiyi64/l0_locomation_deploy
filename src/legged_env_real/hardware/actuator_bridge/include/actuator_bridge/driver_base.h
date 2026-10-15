#pragma once
#include <can_msgs/msg/frame.hpp>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <unordered_map>

#include "actuator_bridge/config_types.h"
#include "actuator_bridge/driver_types.h"

namespace actuator_bridge {

class DriverBase {
 public:
  virtual ~DriverBase() = default;

  virtual std::string vendor() const = 0;

  virtual void encodeCommand(const MotorCmd& cmd, can_msgs::msg::Frame& out) = 0;

  virtual bool parseFeedback(const can_msgs::msg::Frame& in, MotorState& out) = 0;

  virtual void makeEnable(int id, can_msgs::msg::Frame& out) = 0;
  virtual void makeDisable(int id, can_msgs::msg::Frame& out) = 0;
  virtual void makeZero(int id, can_msgs::msg::Frame& out) = 0;
  virtual void makeReset(int id, can_msgs::msg::Frame& out) = 0;

  virtual bool supportsReset() const { return false; }

  virtual void setLimits(int id, const Limits& lim) { limits_[id] = lim; }
  virtual Limits getLimits(int id) const {
    auto it = limits_.find(id);
    if (it == limits_.end())
    {
      RCLCPP_ERROR(rclcpp::get_logger("driver_base"), "%s: No matching limits found for actuator id %d",
                   vendor().c_str(), id);
      return Limits{};
    }
    return it->second;
  }

 protected:
  std::unordered_map<int, Limits> limits_;
};

}  // namespace actuator_bridge
