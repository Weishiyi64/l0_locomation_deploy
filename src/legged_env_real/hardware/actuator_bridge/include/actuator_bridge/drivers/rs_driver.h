#pragma once
#include "actuator_bridge/driver_base.h"

namespace actuator_bridge {

class RSDriver : public DriverBase {
 public:
  RSDriver() = default;
  std::string vendor() const override { return "RS"; }
  void encodeCommand(const MotorCmd& cmd, can_msgs::msg::Frame& out) override;
  bool parseFeedback(const can_msgs::msg::Frame& in, MotorState& out) override;
  void makeEnable(int id, can_msgs::msg::Frame& out) override;
  void makeDisable(int id, can_msgs::msg::Frame& out) override;
  void makeZero(int id, can_msgs::msg::Frame& out) override;
  void makeReset(int id, can_msgs::msg::Frame& out) override;
  bool supportsReset() const override { return true; }
};

}  // namespace actuator_bridge
