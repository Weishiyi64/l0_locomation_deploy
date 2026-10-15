#include "actuator_bridge/drivers/motorevo_driver.h"

#include <memory>

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

void make_state_frame(int id, uint8_t command, can_msgs::msg::Frame& out) {
  can_msgs::msg::Frame f;
  f.is_extended = false;
  f.is_rtr = false;
  f.dlc = 8;
  f.id = id;
  f.data.fill(0xFF);
  f.data[7] = command;
  out = f;
}

}  // namespace

namespace actuator_bridge {

void MotorevoDriver::encodeCommand(const MotorCmd& cmd, can_msgs::msg::Frame& out) {
  can_msgs::msg::Frame f;
  f.is_extended = false;
  f.is_rtr = false;
  f.dlc = 8;
  f.id = cmd.id;

  Limits lim = getLimits(cmd.id);
  uint16_t pos_d_tmp = float_to_uint(cmd.pos_d, lim.p_min, lim.p_max, 16);
  uint16_t vel_d_tmp = float_to_uint(cmd.vel_d, lim.v_min, lim.v_max, 12);
  uint16_t kp_tmp = float_to_uint(cmd.kp, lim.kp_min, lim.kp_max, 12);
  uint16_t kd_tmp = float_to_uint(cmd.kd, lim.kd_min, lim.kd_max, 12);
  uint16_t tau_d_tmp = float_to_uint(cmd.tau_d, lim.t_min, lim.t_max, 12);

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

bool MotorevoDriver::parseFeedback(const can_msgs::msg::Frame& in, MotorState& out) {
  uint16_t pos_hex = (uint16_t)((in.data[1] << 8) | in.data[2]);
  uint16_t vel_hex = (uint16_t)((in.data[3] << 4) | ((in.data[4] & 0xF0) >> 4));
  uint16_t cur_hex = (uint16_t)(((in.data[4] & 0x0F) << 8) | in.data[5]);
  Limits lim = getLimits(out.id);
  out.pos = uint_to_float(pos_hex, lim.p_max, lim.p_min, 16);
  out.vel = uint_to_float(vel_hex, lim.v_max, lim.v_min, 12);
  out.tau = uint_to_float(cur_hex, lim.cur_max, lim.cur_min, 12) * lim.kt;
  out.error = static_cast<uint16_t>(in.data[6]);
  out.mos_temp = static_cast<float>(in.data[7]);
  out.motor_temp = 0.0f;

  const uint16_t status = static_cast<uint16_t>(in.data[0]);
  if (out.error != 0)
    out.state = static_cast<uint16_t>(0xF);
  else if (status == 0)
    out.state = static_cast<uint16_t>(0x0);
  else if (status <= 5)
    out.state = static_cast<uint16_t>(0x1);
  else
    out.state = static_cast<uint16_t>(0xF);

  out.raw = in;
  return true;
}

void MotorevoDriver::makeEnable(int id, can_msgs::msg::Frame& out) { make_state_frame(id, 0xFC, out); }

void MotorevoDriver::makeDisable(int id, can_msgs::msg::Frame& out) { make_state_frame(id, 0xFD, out); }

void MotorevoDriver::makeZero(int id, can_msgs::msg::Frame& out) { make_state_frame(id, 0xFE, out); }

void MotorevoDriver::makeReset(int id, can_msgs::msg::Frame& out) { make_state_frame(id, 0xFD, out); }

}  // namespace actuator_bridge

namespace {
struct _motorevo_reg {
  _motorevo_reg() {
    actuator_bridge::DriverRegistry::instance().registerFactory(
        "MR", []() { return std::make_shared<actuator_bridge::MotorevoDriver>(); });
  }
} _motorevo_reg_instance;
}  // namespace
