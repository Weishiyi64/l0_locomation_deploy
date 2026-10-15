#include "actuator_bridge/drivers/ec_driver.h"

#include <algorithm>

#include "actuator_bridge/driver_registry.h"

namespace {

inline int float_to_uint(float x, float x_min, float x_max, int bits) {
  float span = x_max - x_min;
  float offset = x_min;
  if (x > x_max)
    x = x_max;
  else if (x < x_min)
    x = x_min;
  return (int)((x - offset) * ((float)((1 << bits) - 1)) / span);
}

inline float uint_to_float(uint16_t x, float x_max, float x_min, int bits) {
  float span = x_max - x_min;
  float offset = x_min;
  return ((float)x) * span / ((float)((1 << bits) - 1)) + offset;
}

}  // namespace

namespace actuator_bridge {

void ECDriver::encodeCommand(const MotorCmd& cmd, can_msgs::msg::Frame& out) {
  can_msgs::msg::Frame f;
  f.is_extended = false;
  f.is_rtr = false;
  f.dlc = 8;
  f.id = cmd.id;

  Limits lim = getLimits(cmd.id);
  int kp_int = float_to_uint(cmd.kp, lim.kp_min, lim.kp_max, 12);
  int kd_int = float_to_uint(cmd.kd, lim.kd_min, lim.kd_max, 9);
  int pos_int = float_to_uint(cmd.pos_d, lim.p_min, lim.p_max, 16);
  int spd_int = float_to_uint(cmd.vel_d, lim.v_min, lim.v_max, 12);
  int tor_int = float_to_uint(cmd.tau_d, lim.t_min, lim.t_max, 12);

  f.data.fill(0);
  f.data[0] = 0x00 | (kp_int >> 7);
  f.data[1] = ((kp_int & 0x7F) << 1) | ((kd_int & 0x100) >> 8);
  f.data[2] = kd_int & 0xFF;
  f.data[3] = pos_int >> 8;
  f.data[4] = pos_int & 0xFF;
  f.data[5] = spd_int >> 4;
  f.data[6] = (spd_int & 0x0F) << 4 | (tor_int >> 8);
  f.data[7] = tor_int & 0xFF;

  out = f;
}

bool ECDriver::parseFeedback(const can_msgs::msg::Frame& in, MotorState& out) {
  uint16_t pos_hex = (in.data[1] << 8) | in.data[2];
  uint16_t spd_hex = (in.data[3] << 4) | ((in.data[4] & 0xF0) >> 4);
  uint16_t cur_hex = ((in.data[4] & 0x0F) << 8) | in.data[5];
  Limits lim = getLimits(out.id);
  out.pos = uint_to_float(pos_hex, lim.p_max, lim.p_min, 16);
  out.vel = uint_to_float(spd_hex, lim.v_max, lim.v_min, 12);
  out.tau = uint_to_float(cur_hex, lim.cur_max, lim.cur_min, 12) * lim.kt;

  out.motor_temp = (in.data[6] - 50.0) / 2.0;
  out.mos_temp = (in.data[7] - 50.0) / 2.0;

  const uint16_t ec_state = static_cast<uint16_t>(0x1);
  const uint16_t ec_error = static_cast<uint16_t>(in.data[0] & 0x1F);

  // ec电机无使/失能概念, 总是处于使能状态, state=1代表使能, state=0xF代表故障
  if (ec_error != 0)
    out.state = static_cast<uint16_t>(0xF);
  else
    out.state = ec_state;

  // error=0代表正常, error>0代表故障
  out.error = ec_error;
  out.raw = in;

  return true;
}

void ECDriver::makeEnable(int id, can_msgs::msg::Frame& out) {
  // EC motor does not require enable command
  can_msgs::msg::Frame f;
  f.is_extended = false;
  f.is_rtr = false;
  f.dlc = 3;
  f.id = id;
  f.data.fill(0);
  f.data[0] = 0x61;
  f.data[1] = 0x00;
  f.data[2] = 0x00;
  out = f;
}

void ECDriver::makeDisable(int id, can_msgs::msg::Frame& out) {
  // EC motor uses zero current command for disable
  can_msgs::msg::Frame f;
  f.is_extended = false;
  f.is_rtr = false;
  f.dlc = 3;
  f.id = id;
  f.data.fill(0);
  f.data[0] = 0x61;
  f.data[1] = 0x00;
  f.data[2] = 0x00;
  out = f;
}

void ECDriver::makeZero(int id, can_msgs::msg::Frame& out) {
  can_msgs::msg::Frame f;
  f.is_extended = false;
  f.is_rtr = false;
  f.dlc = 4;
  f.id = 0x7FF;
  f.data.fill(0);
  f.data[0] = id >> 8;
  f.data[1] = id & 0xFF;
  f.data[2] = 0x00;
  f.data[3] = 0x03;
  out = f;
}

void ECDriver::makeReset(int id, can_msgs::msg::Frame& out) {
  // EC motor uses zero current command for reset
  can_msgs::msg::Frame f;
  f.is_extended = false;
  f.is_rtr = false;
  f.dlc = 3;
  f.id = id;
  f.data.fill(0);
  f.data[0] = 0x61;
  f.data[1] = 0x00;
  f.data[2] = 0x00;
  out = f;
}

}  // namespace actuator_bridge

// static registrar to auto-register ECDriver
namespace {
struct _ec_reg {
  _ec_reg() {
    actuator_bridge::DriverRegistry::instance().registerFactory(
        "EC", []() { return std::make_shared<actuator_bridge::ECDriver>(); });
  }
} _ec_reg_instance;
}  // namespace
