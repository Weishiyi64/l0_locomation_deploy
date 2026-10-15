#include "actuator_bridge/drivers/dm_driver.h"

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

void DMDriver::encodeCommand(const MotorCmd& cmd, can_msgs::msg::Frame& out) {
  can_msgs::msg::Frame f;
  f.is_extended = false;
  f.is_rtr = false;
  f.dlc = 8;
  f.id = cmd.id;

  Limits lim = getLimits(cmd.id);
  uint16_t pos_d_tmp = float_to_uint(cmd.pos_d, lim.p_min, lim.p_max, 16);
  uint16_t vel_d_tmp = float_to_uint(cmd.vel_d, lim.v_min, lim.v_max, 12);
  uint16_t tau_d_tmp = float_to_uint(cmd.tau_d, lim.t_min, lim.t_max, 12);
  uint16_t kp_tmp = float_to_uint(cmd.kp, lim.kp_min, lim.kp_max, 12);
  uint16_t kd_tmp = float_to_uint(cmd.kd, lim.kd_min, lim.kd_max, 12);

  f.data.fill(0);
  f.data[0] = (pos_d_tmp >> 8);
  f.data[1] = (pos_d_tmp & 0xFF);
  f.data[2] = ((vel_d_tmp >> 4) & 0xFF);
  f.data[3] = ((((vel_d_tmp & 0xF) << 4) & 0xFF) | ((kp_tmp >> 8) & 0xFF));
  f.data[4] = (kp_tmp & 0xFF);
  f.data[5] = ((kd_tmp >> 4) & 0xFF);
  f.data[6] = ((((kd_tmp & 0xF) << 4) & 0xFF) | ((tau_d_tmp >> 8) & 0xFF));
  f.data[7] = (tau_d_tmp & 0xFF);

  out = f;
}

bool DMDriver::parseFeedback(const can_msgs::msg::Frame& in, MotorState& out) {
  int id = out.id;
  uint16_t pos_hex = (uint16_t)(in.data[2] | (in.data[1] << 8));
  uint16_t vel_hex = (uint16_t)((in.data[4] >> 4) | (in.data[3] << 4));
  uint16_t t_hex = (uint16_t)((in.data[5] | (in.data[4] & 0x0F) << 8));
  Limits lim = getLimits(id);
  out.pos = uint_to_float(pos_hex, lim.p_max, lim.p_min, 16);
  out.vel = uint_to_float(vel_hex, lim.v_max, lim.v_min, 12);
  out.tau = uint_to_float(t_hex, lim.t_max, lim.t_min, 12);
  out.mos_temp = in.data[6];
  out.motor_temp = in.data[7];
  const uint16_t dm_state = static_cast<uint16_t>(in.data[0]) >> 4;
  // state=0代表失能, state=1代表使能, state=0xF代表故障
  out.state = (dm_state <= 1) ? dm_state : static_cast<uint16_t>(0xF);
  // error=0代表正常, error>0代表故障
  out.error = (dm_state > 1) ? dm_state : static_cast<uint16_t>(0x0);
  out.raw = in;

  return true;
}

void DMDriver::makeEnable(int id, can_msgs::msg::Frame& out) {
  can_msgs::msg::Frame f;
  f.is_extended = false;
  f.is_rtr = false;
  f.dlc = 8;
  f.id = id;
  f.data.fill(0xFF);
  f.data[7] = 0xFC;
  out = f;
}

void DMDriver::makeDisable(int id, can_msgs::msg::Frame& out) {
  can_msgs::msg::Frame f;
  f.is_extended = false;
  f.is_rtr = false;
  f.dlc = 8;
  f.id = id;
  f.data.fill(0xFF);
  f.data[7] = 0xFD;
  out = f;
}

void DMDriver::makeZero(int id, can_msgs::msg::Frame& out) {
  can_msgs::msg::Frame f;
  f.is_extended = false;
  f.is_rtr = false;
  f.dlc = 8;
  f.id = id;
  f.data.fill(0xFF);
  f.data[7] = 0xFE;
  out = f;
}

void DMDriver::makeReset(int id, can_msgs::msg::Frame& out) {
  can_msgs::msg::Frame f;
  f.is_extended = false;
  f.is_rtr = false;
  f.dlc = 8;
  f.id = id;
  f.data.fill(0xFF);
  f.data[7] = 0xFB;
  out = f;
}

}  // namespace actuator_bridge

// static registrar to auto-register DMDriver
namespace {
struct _dm_reg {
  _dm_reg() {
    actuator_bridge::DriverRegistry::instance().registerFactory(
        "DM", []() { return std::make_shared<actuator_bridge::DMDriver>(); });
  }
} _dm_reg_instance;
}  // namespace
