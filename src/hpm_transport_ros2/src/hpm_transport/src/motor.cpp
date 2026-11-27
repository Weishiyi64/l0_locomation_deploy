#include "hpm_transport/motor.hpp"

#include <algorithm>
#include <cstdint>
#include <cstring>
#include <stdexcept>
#define MOTOR_UNUSED_FUNC (void)1;

namespace frameTransport {

namespace {
bool valid_range(float low, float high)
{
  return std::isfinite(low) && std::isfinite(high) && low < high &&
         std::isfinite(high - low);
}
} // namespace

bool MotorLimits::valid(bool needs_current) const
{
  return valid_range(p_min, p_max) && valid_range(v_min, v_max) &&
         valid_range(kp_min, kp_max) && valid_range(kd_min, kd_max) &&
         valid_range(t_min, t_max) && (!needs_current || valid_range(i_min, i_max));
}

const MotorLimits * MotorProtocolConfig::find(MotorType type, MotorModel model) const
{
  switch (type) {
    case MotorType::MOTOR_TYPE_DM: return &dm;
    case MotorType::MOTOR_TYPE_ENC: return &enc;
    case MotorType::MOTOR_TYPE_RS: return &rs;
    case MotorType::MOTOR_TYPE_LS: return &ls;
    case MotorType::MOTOR_TYPE_MR:
      switch (model) {
        case MotorModel::MR_PA43_10_40: return &mr_pa43_10_40;
        case MotorModel::MR_PA43_10_20_25: return &mr_pa43_10_20_25;
        case MotorModel::MR_PA35_10_8: return &mr_pa35_10_8;
        default: return nullptr;
      }
    default: return nullptr;
  }
}

namespace {

uint16_t float_to_uint(float x, float x_min, float x_max, uint8_t bits) {
  const float s = std::clamp(x, x_min, x_max);
  const uint32_t maxv = (1U << bits) - 1U;
  return static_cast<uint16_t>(
      (s - x_min) / (x_max - x_min) * static_cast<float>(maxv) + 0.5f);
}

uint16_t float_to_uint_trunc(float x, float x_min, float x_max, uint8_t bits) {
  const float s = std::clamp(x, x_min, x_max);
  const uint32_t maxv = (1U << bits) - 1U;
  return static_cast<uint16_t>((s - x_min) / (x_max - x_min) *
                               static_cast<float>(maxv));
}


int8_t mr_unpack_param(const uint8_t *data_in, uint8_t &motor_id_out,
                        uint16_t &addr_out, uint32_t &value_out) {
  if (data_in[6] != 0x00U || data_in[7] != 0xFFU) {
    return 1;
  }
  motor_id_out = data_in[0];
  addr_out = data_in[1];
  value_out = static_cast<uint32_t>(data_in[2]) |
              (static_cast<uint32_t>(data_in[3]) << 8U) |
              (static_cast<uint32_t>(data_in[4]) << 16U) |
              (static_cast<uint32_t>(data_in[5]) << 24U);
  return 0;
}

float uint_to_float(uint16_t x, float x_min, float x_max, uint8_t bits) {
  const uint32_t maxv = (1U << bits) - 1U;
  return x_min +
         (x_max - x_min) * static_cast<float>(x) / static_cast<float>(maxv);
}

uint32_t rs_canid(uint8_t type, uint8_t motor_id) {
  return (static_cast<uint32_t>(type) << 24) | (motor_id & 0xFFu);
}

} // namespace

uint32_t motor_rs_pack_canid_mit(const MotorProtocolConfig &config_in, uint8_t motor_id_in, float t_ff_in) {
  const auto &limits = config_in.rs;
  if (!limits.valid()) {
    throw std::invalid_argument("RS motor protocol limits are missing or invalid");
  }
  const uint16_t torque = float_to_uint(t_ff_in, limits.t_min, limits.t_max, 16);
  return (0x1u << 24) | (static_cast<uint32_t>(torque) << 8) |
         (motor_id_in & 0xFFu);
}

int8_t motor_rs_unpack_canid_mit(uint32_t can_id_in, mitfb_t &out_out) {
  if (((can_id_in >> 24) & 0x1Fu) != 0x2u) {
    return 1;
  }
  out_out.motor_id = static_cast<uint8_t>((can_id_in >> 8) & 0xFFu);
  out_out.err = static_cast<uint8_t>((can_id_in >> 16) & 0x3Fu);
  out_out.mode = static_cast<uint8_t>((can_id_in >> 22) & 0x3u);
  out_out.enabled_known = out_out.mode < 3U;
  out_out.enabled = out_out.mode == 2U;  // RS run (as opposed to reset/calibration).
  return 0;
}

uint32_t motor_rs_pack_canid_enable(uint8_t motor_id_in) {
  return rs_canid(0x3, motor_id_in);
}

uint32_t motor_rs_pack_canid_disable(uint8_t motor_id_in) {
  return rs_canid(0x4, motor_id_in);
}

uint32_t motor_rs_pack_canid_setzero(uint8_t motor_id_in) {
  return rs_canid(0x6, motor_id_in);
}

uint32_t motor_rs_pack_canid_reset(uint8_t motor_id_in) {
  return rs_canid(0x4, motor_id_in);
}

uint32_t motor_rs_pack_canid_regw(uint8_t motor_id_in) {
  return rs_canid(0x12, motor_id_in);
}

uint32_t motor_rs_pack_canid_regr(uint8_t motor_id_in) {
  return rs_canid(0x11, motor_id_in);
}

int8_t motor_rs_unpack_canid_regr(uint32_t can_id_in, uint8_t &motor_id_out,
                                  bool &ok_out) {
  motor_id_out = 0U;
  ok_out = false;
  if (((can_id_in >> 24) & 0x1Fu) != 0x11u) {
    return 1;
  }
  motor_id_out = static_cast<uint8_t>((can_id_in >> 8) & 0xFFu);
  ok_out = (((can_id_in >> 16) & 0xFFu) == 0U);
  return 0;
}

uint32_t motor_rs_pack_canid_regs(uint8_t motor_id_in) {
  return rs_canid(0x16, motor_id_in);
}

uint32_t motor_rs_pack_canid_getid(uint8_t motor_id_in) {
  return rs_canid(0x0, motor_id_in);
}

int8_t motor_rs_unpack_canid_getid(uint32_t can_id_in, uint16_t &motor_id_out,
                                   bool &ok_out) {
  motor_id_out = 0U;
  ok_out = false;
  if (((can_id_in >> 24) & 0x1Fu) != 0x0u || (can_id_in & 0xFFu) != 0xFEu) {
    return 1;
  }
  motor_id_out = static_cast<uint16_t>((can_id_in >> 8) & 0xFFFFu);
  ok_out = true;
  return 0;
}

uint32_t motor_rs_pack_canid_setid(uint8_t old_motor_id_in,
                                   uint8_t new_motor_id_in) {
  return (0x7u << 24) | (static_cast<uint32_t>(new_motor_id_in) << 16) |
         (old_motor_id_in & 0xFFu);
}

int8_t motor_rs_unpack_canid_setid(uint32_t can_id_in, uint16_t &motor_id_out,
                                   bool &ok_out) {
  return motor_rs_unpack_canid_getid(can_id_in, motor_id_out, ok_out);
}

uint32_t motor_rs_pack_canid_resetid(uint8_t old_motor_id_in) {
  return motor_rs_pack_canid_setid(old_motor_id_in, 0x01);
}

int8_t motor_rs_unpack_canid_resetid(uint32_t can_id_in, uint16_t &motor_id_out,
                                     bool &ok_out) {
  return motor_rs_unpack_canid_setid(can_id_in, motor_id_out, ok_out);
}

int8_t motor_pack_mit(const MotorProtocolConfig &config_in, MotorType type_in, const mitctrl_t &params_in,
                      std::array<uint8_t, 8> &out_out) {

  const MotorLimits * selected = config_in.find(type_in, params_in.model);
  if (selected == nullptr || !selected->valid(type_in == MotorType::MOTOR_TYPE_ENC ||
      type_in == MotorType::MOTOR_TYPE_LS) || !std::isfinite(config_in.deg_to_rad) ||
      config_in.deg_to_rad <= 0.0f) {
    return 1;
  }
  const auto &limits = *selected;
  switch (type_in) {
  case MotorType::MOTOR_TYPE_DM: {
    const uint16_t p = float_to_uint(params_in.p_des, limits.p_min, limits.p_max, 16);
    const uint16_t v = float_to_uint(params_in.v_des, limits.v_min, limits.v_max, 12);
    const uint16_t kp = float_to_uint(params_in.kp, limits.kp_min, limits.kp_max, 12);
    const uint16_t kd = float_to_uint(params_in.kd, limits.kd_min, limits.kd_max, 12);
    const uint16_t t = float_to_uint(params_in.t_ff, limits.t_min, limits.t_max, 12);

    out_out[0] = static_cast<uint8_t>(p >> 8U);
    out_out[1] = static_cast<uint8_t>(p & 0xFFU);
    out_out[2] = static_cast<uint8_t>(v >> 4U);
    out_out[3] =
        static_cast<uint8_t>(((v & 0x0FU) << 4U) | ((kp >> 8U) & 0x0FU));
    out_out[4] = static_cast<uint8_t>(kp & 0xFFU);
    out_out[5] = static_cast<uint8_t>(kd >> 4U);
    out_out[6] =
        static_cast<uint8_t>(((kd & 0x0FU) << 4U) | ((t >> 8U) & 0x0FU));
    out_out[7] = static_cast<uint8_t>(t & 0xFFU);
    return 0;
  }
  case MotorType::MOTOR_TYPE_RS: {
    /* RS: 只 pack p/v/kp/kd 四个 u16 BE; 扭矩走扩展 ID, 不写 data */
    const uint16_t p = float_to_uint(params_in.p_des, limits.p_min, limits.p_max, 16);
    const uint16_t v = float_to_uint(params_in.v_des, limits.v_min, limits.v_max, 16);
    const uint16_t kp = float_to_uint(params_in.kp, limits.kp_min, limits.kp_max, 16);
    const uint16_t kd = float_to_uint(params_in.kd, limits.kd_min, limits.kd_max, 16);

    out_out[0] = static_cast<uint8_t>(p >> 8U);
    out_out[1] = static_cast<uint8_t>(p & 0xFFU);
    out_out[2] = static_cast<uint8_t>(v >> 8U);
    out_out[3] = static_cast<uint8_t>(v & 0xFFU);
    out_out[4] = static_cast<uint8_t>(kp >> 8U);
    out_out[5] = static_cast<uint8_t>(kp & 0xFFU);
    out_out[6] = static_cast<uint8_t>(kd >> 8U);
    out_out[7] = static_cast<uint8_t>(kd & 0xFFU);
    return 0;
  }
  case MotorType::MOTOR_TYPE_ENC: {
    const uint16_t kp =
        float_to_uint_trunc(params_in.kp, limits.kp_min, limits.kp_max, 12);
    const uint16_t kd =
        float_to_uint_trunc(params_in.kd, limits.kd_min, limits.kd_max, 9);
    const uint16_t p =
        float_to_uint_trunc(params_in.p_des, limits.p_min, limits.p_max, 16);
    const uint16_t v =
        float_to_uint_trunc(params_in.v_des, limits.v_min, limits.v_max, 12);
    const uint16_t t =
        float_to_uint_trunc(params_in.t_ff, limits.t_min, limits.t_max, 12);

    out_out[0] = static_cast<uint8_t>(kp >> 7U);
    out_out[1] =
        static_cast<uint8_t>(((kp & 0x7FU) << 1U) | ((kd >> 8U) & 0x01U));
    out_out[2] = static_cast<uint8_t>(kd & 0xFFU);
    out_out[3] = static_cast<uint8_t>(p >> 8U);
    out_out[4] = static_cast<uint8_t>(p & 0xFFU);
    out_out[5] = static_cast<uint8_t>(v >> 4U);
    out_out[6] =
        static_cast<uint8_t>(((v & 0x0FU) << 4U) | ((t >> 8U) & 0x0FU));
    out_out[7] = static_cast<uint8_t>(t & 0xFFU);
    return 0;
  }
  case MotorType::MOTOR_TYPE_MR: {
    const uint16_t p =
        float_to_uint_trunc(params_in.p_des, limits.p_min, limits.p_max, 16);
    const uint16_t v =
        float_to_uint_trunc(params_in.v_des, limits.v_min, limits.v_max, 12);
    const uint16_t kp =
        float_to_uint_trunc(params_in.kp, limits.kp_min, limits.kp_max, 12);
    const uint16_t kd =
        float_to_uint_trunc(params_in.kd, limits.kd_min, limits.kd_max, 12);
    const uint16_t t =
        float_to_uint_trunc(params_in.t_ff, limits.t_min, limits.t_max, 12);

    out_out[0] = static_cast<uint8_t>(p >> 8U);
    out_out[1] = static_cast<uint8_t>(p & 0xFFU);
    out_out[2] = static_cast<uint8_t>(v >> 4U);
    out_out[3] =
        static_cast<uint8_t>(((v & 0x0FU) << 4U) | ((kp >> 8U) & 0x0FU));
    out_out[4] = static_cast<uint8_t>(kp & 0xFFU);
    out_out[5] = static_cast<uint8_t>(kd >> 4U);
    out_out[6] =
        static_cast<uint8_t>(((kd & 0x0FU) << 4U) | ((t >> 8U) & 0x0FU));
    out_out[7] = static_cast<uint8_t>(t & 0xFFU);
    return 0;
  }
  case MotorType::MOTOR_TYPE_LS: {
    /* mode3 | kp12 | kd9 | p16 | v12 | t12, MIT mode=0x00 */
    const uint16_t kp =
        float_to_uint_trunc(params_in.kp, limits.kp_min, limits.kp_max, 12);
    const uint16_t kd =
        float_to_uint_trunc(params_in.kd, limits.kd_min, limits.kd_max, 9);
    const uint16_t p =
        float_to_uint_trunc(params_in.p_des, limits.p_min, limits.p_max, 16);
    const uint16_t v =
        float_to_uint_trunc(params_in.v_des, limits.v_min, limits.v_max, 12);
    const uint16_t t =
        float_to_uint_trunc(params_in.t_ff, limits.t_min, limits.t_max, 12);
    constexpr uint8_t mode = 0x00U;

    out_out[0] =
        static_cast<uint8_t>((mode << 5U) | ((kp >> 7U) & 0x1FU));
    out_out[1] =
        static_cast<uint8_t>(((kp & 0x7FU) << 1U) | ((kd >> 8U) & 0x01U));
    out_out[2] = static_cast<uint8_t>(kd & 0xFFU);
    out_out[3] = static_cast<uint8_t>(p >> 8U);
    out_out[4] = static_cast<uint8_t>(p & 0xFFU);
    out_out[5] = static_cast<uint8_t>(v >> 4U);
    out_out[6] =
        static_cast<uint8_t>(((v & 0x0FU) << 4U) | ((t >> 8U) & 0x0FU));
    out_out[7] = static_cast<uint8_t>(t & 0xFFU);
    return 0;
  }
  default:
    return 1;
  }
}

int8_t motor_unpack_mit(const MotorProtocolConfig &config_in, MotorType type_in, const uint8_t *data_in,
                        mitfb_t &out_out, MotorModel model_in) {
  if (data_in == nullptr) {
    return 1;
  }

  const MotorLimits * selected = config_in.find(type_in, model_in);
  if (selected == nullptr || !selected->valid(type_in == MotorType::MOTOR_TYPE_ENC ||
      type_in == MotorType::MOTOR_TYPE_LS) || !std::isfinite(config_in.deg_to_rad) ||
      config_in.deg_to_rad <= 0.0f) {
    return 1;
  }
  const auto &limits = *selected;
  out_out.enabled_known = false;
  out_out.enabled = false;
  switch (type_in) {
  case MotorType::MOTOR_TYPE_DM: {
    const uint16_t pos = static_cast<uint16_t>((data_in[1] << 8U) | data_in[2]);
    const uint16_t vel =
        static_cast<uint16_t>((data_in[3] << 4U) | (data_in[4] >> 4U));
    const uint16_t tau =
        static_cast<uint16_t>(((data_in[4] & 0x0FU) << 8U) | data_in[5]);

    out_out.motor_id = static_cast<uint8_t>(data_in[0] & 0x0FU);
    out_out.err = static_cast<uint8_t>(data_in[0] >> 4U);
    out_out.enabled_known = out_out.err <= 1U;
    out_out.enabled = out_out.err == 1U;
    out_out.pos = uint_to_float(pos, limits.p_min, limits.p_max, 16);
    out_out.vel = uint_to_float(vel, limits.v_min, limits.v_max, 12);
    out_out.torque = uint_to_float(tau, limits.t_min, limits.t_max, 12);
    out_out.t_mos = data_in[6];
    out_out.t_rotor = data_in[7];
    return 0;
  }
  case MotorType::MOTOR_TYPE_RS: {
    const uint16_t pos = static_cast<uint16_t>((data_in[0] << 8U) | data_in[1]);
    const uint16_t vel = static_cast<uint16_t>((data_in[2] << 8U) | data_in[3]);
    const uint16_t tau = static_cast<uint16_t>((data_in[4] << 8U) | data_in[5]);
    const uint16_t temp_x10 =
        static_cast<uint16_t>((data_in[6] << 8U) | data_in[7]);

    out_out.pos = uint_to_float(pos, limits.p_min, limits.p_max, 16);
    out_out.vel = uint_to_float(vel, limits.v_min, limits.v_max, 16);
    out_out.torque = uint_to_float(tau, limits.t_min, limits.t_max, 16);
    out_out.t_mos =
        static_cast<uint8_t>(std::min<uint16_t>(255U, temp_x10 / 10U));
    out_out.t_rotor = 0U;
    return 0;
  }
  case MotorType::MOTOR_TYPE_ENC: {
    const uint16_t pos = static_cast<uint16_t>((data_in[1] << 8U) | data_in[2]);
    const uint16_t vel =
        static_cast<uint16_t>((data_in[3] << 4U) | (data_in[4] >> 4U));
    const uint16_t cur =
        static_cast<uint16_t>(((data_in[4] & 0x0FU) << 8U) | data_in[5]);

    out_out.motor_id = static_cast<uint8_t>(data_in[0] >> 5U);
    out_out.err = static_cast<uint8_t>(data_in[0] & 0x1FU);
    out_out.pos = uint_to_float(pos, limits.p_min, limits.p_max, 16);
    out_out.vel = uint_to_float(vel, limits.v_min, limits.v_max, 12);
    out_out.torque = uint_to_float(cur, limits.i_min, limits.i_max, 12);
    out_out.t_rotor = static_cast<uint8_t>(
        std::max(0, (static_cast<int>(data_in[6]) - 50) / 2));
    out_out.t_mos = static_cast<uint8_t>(
        std::max(0, (static_cast<int>(data_in[7]) - 50) / 2));
    return 0;
  }
  case MotorType::MOTOR_TYPE_MR: {
    const uint16_t pos = static_cast<uint16_t>((data_in[0] << 8U) | data_in[1]);
    const uint16_t vel =
        static_cast<uint16_t>((data_in[2] << 4U) | (data_in[3] >> 4U));
    const uint16_t tau =
        static_cast<uint16_t>(((data_in[3] & 0x0FU) << 8U) | data_in[4]);
    const uint16_t status =
        static_cast<uint16_t>((data_in[6] << 8U) | data_in[7]);

    out_out.mode = 0xFFU;
    out_out.enabled_known = true;
    out_out.enabled = (status & 0x0001U) != 0U;
    out_out.pos = uint_to_float(pos, limits.p_min, limits.p_max, 16);
    out_out.vel = uint_to_float(vel, limits.v_min, limits.v_max, 12);
    out_out.torque = uint_to_float(tau, limits.t_min, limits.t_max, 12);
    out_out.winding_temperature = static_cast<float>(data_in[5]) - 40.0f;
    out_out.err = status & 0xFFFEU;
    out_out.t_mos = 0U;
    out_out.t_rotor = 0U;
    return 0;
  }
  case MotorType::MOTOR_TYPE_LS: {
    out_out.mode = static_cast<uint8_t>(data_in[0] >> 5U);
    out_out.err = static_cast<uint8_t>(data_in[0] & 0x1FU);
    if (out_out.mode == 1U) {
      out_out.enabled = false;
      if (config_in.ls_type1_new_firmware) {
        // New LS type 1: bits 0..2 fault, bit 3 reserved, bit 4 enabled.
        out_out.err = static_cast<uint8_t>(data_in[0] & 0x07U);
        out_out.enabled_known = true;
        out_out.enabled = (data_in[0] & 0x10U) != 0U;
      }
      const uint16_t pos =
          static_cast<uint16_t>((data_in[1] << 8U) | data_in[2]);
      const uint16_t vel =
          static_cast<uint16_t>((data_in[3] << 4U) | (data_in[4] >> 4U));
      const uint16_t iq =
          static_cast<uint16_t>(((data_in[4] & 0x0FU) << 8U) | data_in[5]);
      out_out.pos = uint_to_float(pos, limits.p_min, limits.p_max, 16);
      out_out.vel = uint_to_float(vel, limits.v_min, limits.v_max, 12);
      out_out.torque = uint_to_float(iq, limits.i_min, limits.i_max, 12);
      out_out.t_rotor = data_in[6];
      out_out.t_mos = data_in[7];
      return 0;
    }
    if (out_out.mode == 2U) {
      uint32_t u = (static_cast<uint32_t>(data_in[1]) << 24U) |
                   (static_cast<uint32_t>(data_in[2]) << 16U) |
                   (static_cast<uint32_t>(data_in[3]) << 8U) |
                   static_cast<uint32_t>(data_in[4]);
      float pos_deg = 0.0f;
      std::memcpy(&pos_deg, &u, sizeof(pos_deg));
      const int16_t iq_cA = static_cast<int16_t>(
          static_cast<uint16_t>((data_in[5] << 8U) | data_in[6]));
      out_out.pos = pos_deg * config_in.deg_to_rad;
      out_out.vel = 0.0f;
      out_out.torque = static_cast<float>(iq_cA) / 100.0f;
      /* ponytail: t_rotor 是 uint8，type2 温度<0 夹到 0 */
      out_out.t_rotor =
          static_cast<uint8_t>(data_in[7] >= 25U ? data_in[7] - 25U : 0U);
      out_out.t_mos = 0U;
      return 0;
    }
    return 1;
  }
  default:
    return 1;
  }
}

void motor_decode_err(MotorType type, uint16_t err, std::string &out) {
  out = "未知";

  switch (type) {
  case MotorType::MOTOR_TYPE_DM:
    switch (err) {
    case 0x0:
      out = "失能";
      break;
    case 0x1:
      out = "使能";
      break;
    case 0x8:
      out = "超压";
      break;
    case 0x9:
      out = "欠压";
      break;
    case 0xA:
      out = "过电流";
      break;
    case 0xB:
      out = "MOS过温";
      break;
    case 0xC:
      out = "线圈过温";
      break;
    case 0xD:
      out = "通讯丢失";
      break;
    case 0xE:
      out = "过载";
      break;
    default:
      break;
    }
    break;
  case MotorType::MOTOR_TYPE_RS: {
    static const char *const names[] = {"欠压故障",   "三相电流故障", "过温",
                                        "磁编码故障", "堵转过载故障", "未标定"};
    if (err == 0U) {
      out = "无错误";
      break;
    }
    out.clear();
    for (uint8_t i = 0U; i < 6U; ++i) {
      if ((err & (1U << i)) == 0U) {
        continue;
      }
      if (!out.empty()) {
        out += '+';
      }
      out += names[i];
    }
    if (out.empty()) {
      out = "未知";
    }
    break;
  }
  case MotorType::MOTOR_TYPE_ENC:
    switch (err) {
    case 0x0:
      out = "无错误";
      break;
    case 0x1:
      out = "电机过热";
      break;
    case 0x2:
      out = "电机过流";
      break;
    case 0x3:
      out = "电机电压过高";
      break;
    case 0x4:
      out = "电机电压过低";
      break;
    case 0x5:
      out = "电机编码器错误";
      break;
    case 0x6:
      out = "电机刹车电压过高";
      break;
    case 0x7:
      out = "DRV 驱动错误";
      break;
    default:
      break;
    }
    break;
  case MotorType::MOTOR_TYPE_MR: {
    struct ErrorBit {
      uint16_t mask;
      const char *name;
    };
    static const ErrorBit errors[] = {
        {1U << 1U, "直流母线过压"}, {1U << 2U, "相电流过流"},
        {1U << 3U, "电机线圈过温"}, {1U << 4U, "电机超速"},
        {1U << 8U, "电机堵转"},     {1U << 10U, "驱动板过温"},
        {1U << 11U, "直流母线欠压"}, {1U << 12U, "位置超限"},
        {1U << 13U, "CAN通讯超时"},
    };
    if (err == 0U) {
      out = "无故障";
      break;
    }
    out.clear();
    uint16_t known = 0U;
    for (const auto &error : errors) {
      known |= error.mask;
      if ((err & error.mask) == 0U) {
        continue;
      }
      if (!out.empty()) {
        out += '+';
      }
      out += error.name;
    }
    if ((err & static_cast<uint16_t>(~known)) != 0U) {
      if (!out.empty()) {
        out += '+';
      }
      out += "未知故障";
    }
    break;
  }
  case MotorType::MOTOR_TYPE_LS:
    switch (err) {
    case 0x0:
      out = "无错误";
      break;
    case 0x1:
      out = "电机过热";
      break;
    case 0x2:
      out = "电机过流";
      break;
    case 0x3:
      out = "电机电压过低";
      break;
    case 0x4:
      out = "电机编码器错误";
      break;
    case 0x6:
      out = "电机刹车电压过高";
      break;
    default:
      break;
    }
    break;
  default:
    break;
  }
}

// enc电机调0需要传入motor_id
int8_t motor_pack_setzero(MotorType type_in, uint16_t motor_id_in,
                          std::array<uint8_t, 8> &out_out) {

  switch (type_in) {
  case MotorType::MOTOR_TYPE_DM:
    out_out = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFE};
    return 0;
  case MotorType::MOTOR_TYPE_RS:
    (void)motor_id_in;
    out_out = {0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    return 0;
  case MotorType::MOTOR_TYPE_ENC:
    out_out = {};
    out_out[0] = static_cast<uint8_t>(motor_id_in >> 8U);
    out_out[1] = static_cast<uint8_t>(motor_id_in & 0xFFU);
    out_out[2] = 0x00U;
    out_out[3] = 0x03U;
    return 0;
  case MotorType::MOTOR_TYPE_MR:
    out_out = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFE};
    return 0;
  case MotorType::MOTOR_TYPE_LS:
    out_out = {};
    out_out[0] = static_cast<uint8_t>(motor_id_in >> 8U);
    out_out[1] = static_cast<uint8_t>(motor_id_in & 0xFFU);
    out_out[2] = 0x00U;
    out_out[3] = 0x03U;
    return 0;
  default:
    return 1;
  }
}

int8_t motor_unpack_setzero(const MotorProtocolConfig &config_in, MotorType type_in, const uint8_t *data_in,
                            uint16_t &motor_id_out, mitfb_t &out_out,
                            bool &ok_out, MotorModel model_in) {
  motor_id_out = 0U;
  ok_out = false;
  out_out = {};
  if (data_in == nullptr) {
    return 1;
  }

  switch (type_in) {
  case MotorType::MOTOR_TYPE_DM:
    return motor_unpack_mit(config_in, type_in, data_in, out_out);
  case MotorType::MOTOR_TYPE_RS:
    return motor_unpack_mit(config_in, type_in, data_in, out_out);
  case MotorType::MOTOR_TYPE_ENC:
    motor_id_out = static_cast<uint16_t>((data_in[0] << 8U) | data_in[1]);
    ok_out = (data_in[2] == 0x01U && data_in[3] == 0x03U);
    return 0;
  case MotorType::MOTOR_TYPE_MR:
    if (motor_unpack_mit(config_in, type_in, data_in, out_out, model_in) != 0) {
      return 1;
    }
    ok_out = (out_out.err == 0U);
    return 0;
  case MotorType::MOTOR_TYPE_LS:
    motor_id_out = static_cast<uint16_t>((data_in[0] << 8U) | data_in[1]);
    ok_out = (data_in[2] == 0x01U && data_in[3] == 0x03U);
    return 0;
  default:
    return 1;
  }
}

int8_t motor_pack_enable(MotorType type_in, uint16_t motor_id_in,
                         std::array<uint8_t, 8> &out_out) {

  switch (type_in) {
  case MotorType::MOTOR_TYPE_DM:
    out_out = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC};
    return 0;
  case MotorType::MOTOR_TYPE_RS:
    out_out = {};
    return 0;
  case MotorType::MOTOR_TYPE_ENC:
    out_out = {0x61, 0x00, 0x00};
    return 0;
  case MotorType::MOTOR_TYPE_MR:
    out_out = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC};
    return 0;
  case MotorType::MOTOR_TYPE_LS:
    out_out = {};
    out_out[0] = static_cast<uint8_t>(motor_id_in >> 8U);
    out_out[1] = static_cast<uint8_t>(motor_id_in & 0xFFU);
    out_out[2] = 0x00U;
    out_out[3] = 0x06U;
    return 0;
  default:
    return 1;
  }
}

int8_t motor_unpack_enable(const MotorProtocolConfig &config_in, MotorType type_in, const uint8_t *data_in,
                           uint16_t &motor_id_out, mitfb_t &out_out,
                           bool &ok_out, MotorModel model_in) {
  motor_id_out = 0U;
  ok_out = false;
  out_out = {};
  if (data_in == nullptr) {
    return 1;
  }

  switch (type_in) {
  case MotorType::MOTOR_TYPE_DM:
    return motor_unpack_mit(config_in, type_in, data_in, out_out);
  case MotorType::MOTOR_TYPE_RS:
    return motor_unpack_mit(config_in, type_in, data_in, out_out);
  case MotorType::MOTOR_TYPE_ENC:
    return motor_unpack_mit(config_in, type_in, data_in, out_out);
  case MotorType::MOTOR_TYPE_MR:
    if (motor_unpack_mit(config_in, type_in, data_in, out_out, model_in) != 0) {
      return 1;
    }
    ok_out = out_out.enabled && (out_out.err == 0U);
    return 0;
  case MotorType::MOTOR_TYPE_LS:
    motor_id_out = static_cast<uint16_t>((data_in[0] << 8U) | data_in[1]);
    ok_out = (data_in[2] == 0x01U && data_in[3] == 0x06U);
    return 0;
  default:
    return 1;
  }
}

int8_t motor_pack_disable(MotorType type_in, uint16_t motor_id_in,
                          std::array<uint8_t, 8> &out_out) {

  switch (type_in) {
  case MotorType::MOTOR_TYPE_DM:
    out_out = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFD};
    return 0;
  case MotorType::MOTOR_TYPE_RS:
    out_out = {};
    return 0;
  case MotorType::MOTOR_TYPE_ENC:
    out_out = {0x61, 0x00, 0x00};
    return 0;
  case MotorType::MOTOR_TYPE_MR:
    out_out = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFD};
    return 0;
  case MotorType::MOTOR_TYPE_LS:
    out_out = {};
    out_out[0] = static_cast<uint8_t>(motor_id_in >> 8U);
    out_out[1] = static_cast<uint8_t>(motor_id_in & 0xFFU);
    out_out[2] = 0x00U;
    out_out[3] = 0x07U;
    return 0;
  default:
    return 1;
  }
}

int8_t motor_unpack_disable(const MotorProtocolConfig &config_in, MotorType type_in, const uint8_t *data_in,
                            uint16_t &motor_id_out, mitfb_t &out_out,
                            bool &ok_out, MotorModel model_in) {
  motor_id_out = 0U;
  ok_out = false;
  out_out = {};
  if (data_in == nullptr) {
    return 1;
  }

  switch (type_in) {
  case MotorType::MOTOR_TYPE_DM:
    return motor_unpack_mit(config_in, type_in, data_in, out_out);
  case MotorType::MOTOR_TYPE_RS:
    return motor_unpack_mit(config_in, type_in, data_in, out_out);
  case MotorType::MOTOR_TYPE_ENC:
    return motor_unpack_mit(config_in, type_in, data_in, out_out);
  case MotorType::MOTOR_TYPE_MR:
    if (motor_unpack_mit(config_in, type_in, data_in, out_out, model_in) != 0) {
      return 1;
    }
    ok_out = !out_out.enabled && (out_out.err == 0U);
    return 0;
  case MotorType::MOTOR_TYPE_LS:
    motor_id_out = static_cast<uint16_t>((data_in[0] << 8U) | data_in[1]);
    ok_out = (data_in[2] == 0x01U && data_in[3] == 0x07U);
    return 0;
  default:
    return 1;
  }
}

int8_t motor_pack_reset(MotorType type_in, uint16_t motor_id_in,
                        std::array<uint8_t, 8> &out_out) {

  switch (type_in) {
  case MotorType::MOTOR_TYPE_DM:
    out_out = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFB};
    return 0;
  case MotorType::MOTOR_TYPE_RS:
    out_out = {0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    return 0;
  case MotorType::MOTOR_TYPE_ENC:
    out_out = {0x61, 0x00, 0x00};
    return 0;
  case MotorType::MOTOR_TYPE_MR:
    out_out = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFB};
    return 0;
  case MotorType::MOTOR_TYPE_LS:
    out_out = {};
    out_out[0] = static_cast<uint8_t>(motor_id_in >> 8U);
    out_out[1] = static_cast<uint8_t>(motor_id_in & 0xFFU);
    out_out[2] = 0x00U;
    out_out[3] = 0x08U;
    return 0;
  default:
    return 1;
  }
}

int8_t motor_unpack_reset(const MotorProtocolConfig &config_in, MotorType type_in, const uint8_t *data_in,
                          uint16_t &motor_id_out, mitfb_t &out_out,
                          bool &ok_out, MotorModel model_in) {
  motor_id_out = 0U;
  ok_out = false;
  out_out = {};
  if (data_in == nullptr) {
    return 1;
  }

  switch (type_in) {
  case MotorType::MOTOR_TYPE_DM:
    return motor_unpack_mit(config_in, type_in, data_in, out_out);
  case MotorType::MOTOR_TYPE_RS:
    return motor_unpack_mit(config_in, type_in, data_in, out_out);
  case MotorType::MOTOR_TYPE_ENC:
    return motor_unpack_mit(config_in, type_in, data_in, out_out);
  case MotorType::MOTOR_TYPE_MR:
    if (motor_unpack_mit(config_in, type_in, data_in, out_out, model_in) != 0) {
      return 1;
    }
    ok_out = (out_out.err == 0U);
    return 0;
  case MotorType::MOTOR_TYPE_LS:
    motor_id_out = static_cast<uint16_t>((data_in[0] << 8U) | data_in[1]);
    ok_out = (data_in[2] == 0x01U && data_in[3] == 0x08U);
    return 0;
  default:
    return 1;
  }
}

int8_t motor_pack_regr(MotorType type_in, uint8_t motor_id_in, uint16_t addr_in,
                       std::array<uint8_t, 8> &out_out) {
  switch (type_in) {
  case MotorType::MOTOR_TYPE_DM:
    out_out = {};
    out_out[0] = motor_id_in;
    out_out[1] = 0x00U;
    out_out[2] = 0x33U;
    out_out[3] = static_cast<uint8_t>(addr_in & 0xFFU);
    return 0;
  case MotorType::MOTOR_TYPE_RS:
    (void)motor_id_in;
    out_out = {};
    out_out[0] = static_cast<uint8_t>(addr_in & 0xFFU);
    out_out[1] = static_cast<uint8_t>((addr_in >> 8U) & 0xFFU);
    return 0;
  case MotorType::MOTOR_TYPE_ENC:
    MOTOR_UNUSED_FUNC
    return 1;
  case MotorType::MOTOR_TYPE_MR:
    (void)motor_id_in;
    if (addr_in > 0xFFU) {
      return 1;
    }
    out_out = {0x67, static_cast<uint8_t>(addr_in & 0xFFU),
               0x00, 0x00,
               0x00, 0x00,
               0x04, 0x76};
    return 0;
  case MotorType::MOTOR_TYPE_LS:
    MOTOR_UNUSED_FUNC
    return 1;
  default:
    return 1;
  }
}

int8_t motor_unpack_regr(MotorType type_in, const uint8_t *data_in,
                         uint8_t &motor_id_out, uint16_t &addr_out,
                         uint32_t &value_out) {
  if (data_in == nullptr) {
    return 1;
  }

  switch (type_in) {
  case MotorType::MOTOR_TYPE_DM:
    motor_id_out = data_in[0];
    addr_out = data_in[3];
    value_out = static_cast<uint32_t>(data_in[4]) |
                (static_cast<uint32_t>(data_in[5]) << 8U) |
                (static_cast<uint32_t>(data_in[6]) << 16U) |
                (static_cast<uint32_t>(data_in[7]) << 24U);
    return 0;
  case MotorType::MOTOR_TYPE_RS:
    motor_id_out = 0U;
    addr_out = static_cast<uint16_t>(data_in[0] |
                                     (static_cast<uint16_t>(data_in[1]) << 8U));
    value_out = static_cast<uint32_t>(data_in[4]) |
                (static_cast<uint32_t>(data_in[5]) << 8U) |
                (static_cast<uint32_t>(data_in[6]) << 16U) |
                (static_cast<uint32_t>(data_in[7]) << 24U);
    return 0;
  case MotorType::MOTOR_TYPE_ENC:
    MOTOR_UNUSED_FUNC
    return 1;
  case MotorType::MOTOR_TYPE_MR:
    return mr_unpack_param(data_in, motor_id_out, addr_out, value_out);
  case MotorType::MOTOR_TYPE_LS:
    MOTOR_UNUSED_FUNC
    return 1;
  default:
    return 1;
  }
}

int8_t motor_pack_regw(MotorType type_in, uint8_t motor_id_in, uint16_t addr_in,
                       uint32_t data_in, std::array<uint8_t, 8> &out_out) {

  switch (type_in) {
  case MotorType::MOTOR_TYPE_DM:
    out_out = {};
    out_out[0] = motor_id_in;
    out_out[1] = 0x00U;
    out_out[2] = 0x55U;
    out_out[3] = static_cast<uint8_t>(addr_in & 0xFFU);
    out_out[4] = static_cast<uint8_t>(data_in & 0xFFU);
    out_out[5] = static_cast<uint8_t>((data_in >> 8U) & 0xFFU);
    out_out[6] = static_cast<uint8_t>((data_in >> 16U) & 0xFFU);
    out_out[7] = static_cast<uint8_t>((data_in >> 24U) & 0xFFU);
    return 0;
  case MotorType::MOTOR_TYPE_RS:
    (void)motor_id_in;
    out_out = {};
    out_out[0] = static_cast<uint8_t>(addr_in & 0xFFU);
    out_out[1] = static_cast<uint8_t>((addr_in >> 8U) & 0xFFU);
    out_out[4] = static_cast<uint8_t>(data_in & 0xFFU);
    out_out[5] = static_cast<uint8_t>((data_in >> 8U) & 0xFFU);
    out_out[6] = static_cast<uint8_t>((data_in >> 16U) & 0xFFU);
    out_out[7] = static_cast<uint8_t>((data_in >> 24U) & 0xFFU);
    return 0;
  case MotorType::MOTOR_TYPE_ENC:
    MOTOR_UNUSED_FUNC;
    return 1;
  case MotorType::MOTOR_TYPE_MR:
    (void)motor_id_in;
    if (addr_in > 0xFFU) {
      return 1;
    }
    out_out = {0x67,
               static_cast<uint8_t>(addr_in & 0xFFU),
               static_cast<uint8_t>(data_in & 0xFFU),
               static_cast<uint8_t>((data_in >> 8U) & 0xFFU),
               static_cast<uint8_t>((data_in >> 16U) & 0xFFU),
               static_cast<uint8_t>((data_in >> 24U) & 0xFFU),
               0x15,
               0x76};
    return 0;
  case MotorType::MOTOR_TYPE_LS:
    MOTOR_UNUSED_FUNC
    return 1;
  default:
    return 1;
  }
}

int8_t motor_unpack_regw(MotorType type_in, const uint8_t *data_in,
                         uint8_t &motor_id_out, uint16_t &addr_out,
                         uint32_t &value_out) {
  if (data_in == nullptr) {
    return 1;
  }

  switch (type_in) {
  case MotorType::MOTOR_TYPE_DM:
    motor_id_out = data_in[0];
    addr_out = data_in[3];
    value_out = static_cast<uint32_t>(data_in[4]) |
                (static_cast<uint32_t>(data_in[5]) << 8U) |
                (static_cast<uint32_t>(data_in[6]) << 16U) |
                (static_cast<uint32_t>(data_in[7]) << 24U);
    return 0;
  case MotorType::MOTOR_TYPE_RS:
    return 1;
  case MotorType::MOTOR_TYPE_ENC:
    MOTOR_UNUSED_FUNC;
    return 1;
  case MotorType::MOTOR_TYPE_MR:
    return mr_unpack_param(data_in, motor_id_out, addr_out, value_out);
  case MotorType::MOTOR_TYPE_LS:
    MOTOR_UNUSED_FUNC
    return 1;
  default:
    return 1;
  }
}

int8_t motor_pack_regs(MotorType type_in, uint8_t motor_id_in, uint16_t addr_in,
                       std::array<uint8_t, 8> &out_out) {
  switch (type_in) {
  case MotorType::MOTOR_TYPE_DM:
    out_out = {};
    out_out[0] = motor_id_in;
    out_out[1] = 0x00U;
    out_out[2] = 0xAAU;
    out_out[3] = static_cast<uint8_t>(addr_in & 0xFFU);
    return 0;
  case MotorType::MOTOR_TYPE_RS:
    (void)motor_id_in;
    (void)addr_in;
    out_out = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08};
    return 0;
  case MotorType::MOTOR_TYPE_ENC:
    MOTOR_UNUSED_FUNC;
    return 1;
  case MotorType::MOTOR_TYPE_MR:
    (void)motor_id_in;
    (void)addr_in;
    out_out = {0x67, 0x00,
               0x00, 0x00,
               0x00, 0x00,
               0x15, 0x76};
    return 0;
  case MotorType::MOTOR_TYPE_LS:
    MOTOR_UNUSED_FUNC
    return 1;
  default:
    return 1;
  }
}

int8_t motor_unpack_regs(MotorType type_in, const uint8_t *data_in,
                         uint8_t &motor_id_out, uint16_t &addr_out,
                         uint32_t &value_out) {
  if (data_in == nullptr) {
    return 1;
  }

  switch (type_in) {
  case MotorType::MOTOR_TYPE_DM:
    motor_id_out = data_in[0];
    addr_out = data_in[3];
    value_out = 0U;
    return 0;
  case MotorType::MOTOR_TYPE_RS:
    return 1;
  case MotorType::MOTOR_TYPE_ENC:
    MOTOR_UNUSED_FUNC
    return 1;
  case MotorType::MOTOR_TYPE_MR:
    if (mr_unpack_param(data_in, motor_id_out, addr_out, value_out) != 0) {
      return 1;
    }
    return (addr_out == 0U) ? 0 : 1;
  case MotorType::MOTOR_TYPE_LS:
    MOTOR_UNUSED_FUNC
    return 1;
  default:
    return 1;
  }
}
int8_t motor_pack_getid(MotorType type_in, std::array<uint8_t, 8> &out_out) {
  switch (type_in) {
  case MotorType::MOTOR_TYPE_DM:
    MOTOR_UNUSED_FUNC
    return 1;
  case MotorType::MOTOR_TYPE_RS:
    out_out = {};
    return 0;
  case MotorType::MOTOR_TYPE_ENC:
    out_out = {};
    out_out[0] = 0xFFU;
    out_out[1] = 0xFFU;
    out_out[2] = 0x00U;
    out_out[3] = 0x82U;
    return 0;
  case MotorType::MOTOR_TYPE_MR:
    return motor_pack_regr(type_in, 0U, 36U, out_out);
  case MotorType::MOTOR_TYPE_LS:
    out_out = {};
    out_out[0] = 0xFFU;
    out_out[1] = 0xFFU;
    out_out[2] = 0x00U;
    out_out[3] = 0x82U;
    return 0;
  default:
    return 1;
  }
}

int8_t motor_unpack_getid(MotorType type_in, const uint8_t *data_in,
                          uint16_t &motor_id_out, bool &ok_out) {
  motor_id_out = 0U;
  ok_out = false;
  if (data_in == nullptr) {
    return 1;
  }

  switch (type_in) {
  case MotorType::MOTOR_TYPE_DM:
    MOTOR_UNUSED_FUNC
    return 1;
  case MotorType::MOTOR_TYPE_RS:
    return 1;
  case MotorType::MOTOR_TYPE_ENC:
    if (data_in[0] == 0xFFU && data_in[1] == 0xFFU && data_in[2] == 0x01U) {
      motor_id_out = static_cast<uint16_t>((data_in[3] << 8U) | data_in[4]);
      ok_out = true;
    }
    return 0;
  case MotorType::MOTOR_TYPE_MR:
    {
      uint8_t response_id = 0U;
      uint16_t addr = 0U;
      uint32_t value = 0U;
      if (mr_unpack_param(data_in, response_id, addr, value) != 0 ||
          addr != 36U || value == 0U || value > 0xFFU ||
          response_id != static_cast<uint8_t>(value)) {
        return 1;
      }
      motor_id_out = static_cast<uint16_t>(value);
      ok_out = true;
      return 0;
    }
  case MotorType::MOTOR_TYPE_LS:
    if (data_in[0] == 0xFFU && data_in[1] == 0xFFU && data_in[2] == 0x01U) {
      motor_id_out = static_cast<uint16_t>((data_in[3] << 8U) | data_in[4]);
      ok_out = true;
    }
    return 0;
  default:
    return 1;
  }
}

int8_t motor_pack_setid(MotorType type_in, uint16_t old_motor_id_in,
                        uint16_t new_motor_id_in,
                        std::array<uint8_t, 8> &out_out) {
  switch (type_in) {
  case MotorType::MOTOR_TYPE_DM:
    MOTOR_UNUSED_FUNC
    return 1;
  case MotorType::MOTOR_TYPE_RS:
    (void)old_motor_id_in;
    (void)new_motor_id_in;
    out_out = {};
    return 0;
  case MotorType::MOTOR_TYPE_ENC:
    out_out = {};
    out_out[0] = static_cast<uint8_t>(old_motor_id_in >> 8U);
    out_out[1] = static_cast<uint8_t>(old_motor_id_in & 0xFFU);
    out_out[2] = 0x00U;
    out_out[3] = 0x04U;
    out_out[4] = static_cast<uint8_t>(new_motor_id_in >> 8U);
    out_out[5] = static_cast<uint8_t>(new_motor_id_in & 0xFFU);
    return 0;
  case MotorType::MOTOR_TYPE_MR:
    (void)old_motor_id_in;
    if (new_motor_id_in == 0U || new_motor_id_in > 0xFFU) {
      return 1;
    }
    return motor_pack_regw(type_in, 0U, 36U,
                           static_cast<uint32_t>(new_motor_id_in), out_out);
  case MotorType::MOTOR_TYPE_LS:
    out_out = {};
    out_out[0] = static_cast<uint8_t>(old_motor_id_in >> 8U);
    out_out[1] = static_cast<uint8_t>(old_motor_id_in & 0xFFU);
    out_out[2] = 0x00U;
    out_out[3] = 0x04U;
    out_out[4] = static_cast<uint8_t>(new_motor_id_in >> 8U);
    out_out[5] = static_cast<uint8_t>(new_motor_id_in & 0xFFU);
    return 0;
  default:
    return 1;
  }
}

int8_t motor_unpack_setid(MotorType type_in, const uint8_t *data_in,
                          uint16_t &motor_id_out, bool &ok_out) {
  motor_id_out = 0U;
  ok_out = false;
  if (data_in == nullptr) {
    return 1;
  }

  switch (type_in) {
  case MotorType::MOTOR_TYPE_DM:
    MOTOR_UNUSED_FUNC;
    return 1;
  case MotorType::MOTOR_TYPE_RS:
    return 1;
  case MotorType::MOTOR_TYPE_ENC:
    motor_id_out = static_cast<uint16_t>((data_in[0] << 8U) | data_in[1]);
    ok_out = (data_in[2] == 0x01U && data_in[3] == 0x04U);
    return 0;
  case MotorType::MOTOR_TYPE_MR:
    {
      uint8_t response_id = 0U;
      uint16_t addr = 0U;
      uint32_t value = 0U;
      if (mr_unpack_param(data_in, response_id, addr, value) != 0 ||
          addr != 36U || value == 0U || value > 0xFFU) {
        return 1;
      }
      motor_id_out = static_cast<uint16_t>(value);
      ok_out = true;
      return 0;
    }
  case MotorType::MOTOR_TYPE_LS:
    motor_id_out = static_cast<uint16_t>((data_in[0] << 8U) | data_in[1]);
    ok_out = (data_in[2] == 0x01U && data_in[3] == 0x04U);
    return 0;
  default:
    return 1;
  }
}

int8_t motor_pack_resetid(MotorType type_in, uint16_t old_motor_id_in,
                          std::array<uint8_t, 8> &out_out) {
  switch (type_in) {
  case MotorType::MOTOR_TYPE_DM:
    MOTOR_UNUSED_FUNC;
    return motor_pack_setid(MotorType::MOTOR_TYPE_DM, old_motor_id_in, 0x01,
                            out_out);
  case MotorType::MOTOR_TYPE_RS:
    return motor_pack_setid(MotorType::MOTOR_TYPE_RS, old_motor_id_in, 0x01,
                            out_out);
  case MotorType::MOTOR_TYPE_ENC:
    out_out = {};
    out_out[0] = 0x7FU;
    out_out[1] = 0x7FU;
    out_out[2] = 0x00U;
    out_out[3] = 0x05U;
    out_out[4] = 0x7FU;
    out_out[5] = 0x7FU;
    return 0;
  case MotorType::MOTOR_TYPE_MR:
    MOTOR_UNUSED_FUNC;
    return motor_pack_setid(MotorType::MOTOR_TYPE_MR, old_motor_id_in, 0x01,
                            out_out);
  case MotorType::MOTOR_TYPE_LS:
    out_out = {};
    out_out[0] = 0x7FU;
    out_out[1] = 0x7FU;
    out_out[2] = 0x00U;
    out_out[3] = 0x05U;
    out_out[4] = 0x7FU;
    out_out[5] = 0x7FU;
    return 0;
  default:
    return 1;
  }
}

int8_t motor_unpack_resetid(MotorType type_in, const uint8_t *data_in,
                            uint16_t &motor_id_out, bool &ok_out) {
  motor_id_out = 0U;
  ok_out = false;
  if (data_in == nullptr) {
    return 1;
  }

  switch (type_in) {
  case MotorType::MOTOR_TYPE_DM:
    MOTOR_UNUSED_FUNC
    return motor_unpack_setid(MotorType::MOTOR_TYPE_DM, data_in, motor_id_out,
                              ok_out);
  case MotorType::MOTOR_TYPE_RS:
    return 1;
  case MotorType::MOTOR_TYPE_ENC:
    ok_out =
        (data_in[0] == 0x7FU && data_in[1] == 0x7FU && data_in[2] == 0x01U &&
         data_in[3] == 0x05U && data_in[4] == 0x7FU && data_in[5] == 0x7FU);
    return 0;
  case MotorType::MOTOR_TYPE_MR:
    if (motor_unpack_setid(type_in, data_in, motor_id_out, ok_out) != 0 ||
        motor_id_out != 1U) {
      ok_out = false;
      return 1;
    }
    return 0;
  case MotorType::MOTOR_TYPE_LS:
    ok_out =
        (data_in[0] == 0x7FU && data_in[1] == 0x7FU && data_in[2] == 0x01U &&
         data_in[3] == 0x05U && data_in[4] == 0x7FU && data_in[5] == 0x7FU);
    return 0;
  default:
    return 1;
  }
}

} // namespace frameTransport
