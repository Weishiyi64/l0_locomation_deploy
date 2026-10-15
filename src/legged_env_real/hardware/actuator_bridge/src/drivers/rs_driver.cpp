#include "actuator_bridge/drivers/rs_driver.h"

#include <cstring>

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

inline float byte_to_float(const uint8_t* bytedata) {
  uint32_t data =
      (uint32_t)bytedata[7] << 24 | (uint32_t)bytedata[6] << 16 | (uint32_t)bytedata[5] << 8 | (uint32_t)bytedata[4];
  float f;
  std::memcpy(&f, &data, 4);
  return f;
}
}  // namespace

namespace actuator_bridge {

void RSDriver::encodeCommand(const MotorCmd& cmd, can_msgs::msg::Frame& out) {
  can_msgs::msg::Frame f;
  f.is_extended = true;
  f.is_rtr = false;
  f.dlc = 8;
  const uint32_t MIT_Type = 0x01;

  Limits lim = getLimits(cmd.id);
  uint32_t id = (MIT_Type << 24) | ((uint32_t)float_to_uint(cmd.tau_d, lim.t_min, lim.t_max, 16) << 8) |
                (uint32_t)(cmd.id & 0xFF);
  f.id = id;
  uint16_t p = (uint16_t)float_to_uint(cmd.pos_d, lim.p_min, lim.p_max, 16);
  uint16_t v = (uint16_t)float_to_uint(cmd.vel_d, lim.v_min, lim.v_max, 16);
  uint16_t kp = (uint16_t)float_to_uint(cmd.kp, lim.kp_min, lim.kp_max, 16);
  uint16_t kd = (uint16_t)float_to_uint(cmd.kd, lim.kd_min, lim.kd_max, 16);
  f.data[0] = (p >> 8) & 0xFF;
  f.data[1] = p & 0xFF;
  f.data[2] = (v >> 8) & 0xFF;
  f.data[3] = v & 0xFF;
  f.data[4] = (kp >> 8) & 0xFF;
  f.data[5] = kp & 0xFF;
  f.data[6] = (kd >> 8) & 0xFF;
  f.data[7] = kd & 0xFF;

  out = f;
}

bool RSDriver::parseFeedback(const can_msgs::msg::Frame& in, MotorState& out) {
  auto u16 = [&](int idx_hi, int idx_lo) { return (uint16_t)((in.data[idx_hi] << 8) | in.data[idx_lo]); };
  uint16_t p_hex = u16(0, 1);
  uint16_t v_hex = u16(2, 3);
  uint16_t t_hex = u16(4, 5);
  Limits lim = getLimits(out.id);
  out.pos = (float)((lim.p_max - lim.p_min) * (float)p_hex / ((1 << 16) - 1) + lim.p_min);
  out.vel = (float)((lim.v_max - lim.v_min) * (float)v_hex / ((1 << 16) - 1) + lim.v_min);
  out.tau = (float)((lim.t_max - lim.t_min) * (float)t_hex / ((1 << 16) - 1) + lim.t_min);
  out.motor_temp = 0.1f * u16(6, 7);
  out.mos_temp = 0.0f;
  const uint16_t rs_state = static_cast<uint16_t>((in.id & 0xC00000) >> 22);
  const uint16_t rs_error = static_cast<uint16_t>((in.id & 0x3F0000) >> 16);
  // state=0代表失能, state=1代表使能, state=2代表标定, state=0xF代表故障
  if (rs_state == 0)
    out.state = static_cast<uint16_t>(0x0);
  else if (rs_state == 2)
    out.state = static_cast<uint16_t>(0x1);
  else if (rs_state == 1)
    out.state = static_cast<uint16_t>(0x2);
  else if (rs_error != 0)
    out.state = static_cast<uint16_t>(0xF);
  else
    out.state = rs_state;

  // error=0代表正常, error>0代表故障
  out.error = rs_error;
  out.raw = in;

  return true;
}

void RSDriver::makeEnable(int id, can_msgs::msg::Frame& out) {
  const uint32_t Communication_Type_MotorEnable = 0x03;
  can_msgs::msg::Frame f;
  f.is_extended = true;
  f.is_rtr = false;
  f.dlc = 8;
  f.id = (Communication_Type_MotorEnable << 24) | (0x00 << 8) | (uint32_t)(id & 0xFF);
  f.data.fill(0);
  out = f;
}

void RSDriver::makeDisable(int id, can_msgs::msg::Frame& out) {
  const uint32_t Communication_Type_MotorStop = 0x04;
  can_msgs::msg::Frame f;
  f.is_extended = true;
  f.is_rtr = false;
  f.dlc = 8;
  f.id = (Communication_Type_MotorStop << 24) | (0x00 << 8) | (uint32_t)(id & 0xFF);
  f.data.fill(0);
  out = f;
}

void RSDriver::makeZero(int id, can_msgs::msg::Frame& out) {
  const uint32_t Communication_Type_SetPosZero = 0x06;
  can_msgs::msg::Frame f;
  f.is_extended = true;
  f.is_rtr = false;
  f.dlc = 8;
  f.id = (Communication_Type_SetPosZero << 24) | (0x00 << 8) | (uint32_t)(id & 0xFF);
  f.data.fill(0);
  f.data[0] = 1;
  out = f;
}

void RSDriver::makeReset(int id, can_msgs::msg::Frame& out) {
  // RS 协议无单独Reset功能,Reset功能与MotorStop重叠
  const uint32_t Communication_Type_MotorStop = 0x04;
  can_msgs::msg::Frame f;
  f.is_extended = true;
  f.is_rtr = false;
  f.dlc = 8;
  f.id = (Communication_Type_MotorStop << 24) | (0x00 << 8) | (uint32_t)(id & 0xFF);
  f.data.fill(0);
  f.data[0] = 1;
  out = f;
}

}  // namespace actuator_bridge

// static registrar to auto-register RSDriver
namespace {
struct _rs_reg {
  _rs_reg() {
    actuator_bridge::DriverRegistry::instance().registerFactory(
        "RS", []() { return std::make_shared<actuator_bridge::RSDriver>(); });
  }
} _rs_reg_instance;
}  // namespace
