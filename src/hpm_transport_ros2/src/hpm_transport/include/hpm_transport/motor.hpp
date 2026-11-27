#ifndef HPM_TRANSPORT__MOTOR_HPP_
#define HPM_TRANSPORT__MOTOR_HPP_

#include <array>
#include <cmath>
#include <cstdint>
#include <string>

namespace frameTransport
{

enum class MotorType : uint8_t
{
  MOTOR_TYPE_UNKNOWN = 0,
  MOTOR_TYPE_DM,
  MOTOR_TYPE_RS,
  MOTOR_TYPE_ENC,
  MOTOR_TYPE_MR,
  MOTOR_TYPE_LS,
};

enum class MotorModel : uint8_t
{
  UNKNOWN = 0,
  MR_PA43_10_40,
  MR_PA43_10_20_25,
  MR_PA35_10_8,
};

enum class MotorCommand : uint8_t
{
  MOTOR_CMD_UNKNOWN = 0,
  MOTOR_CMD_CTRL,
  MOTOR_CMD_ENABLE,
  MOTOR_CMD_DISABLE,
  MOTOR_CMD_SETZERO,
  MOTOR_CMD_RESET,
  MOTOR_CMD_WRITEREG,
  MOTOR_CMD_READREG,
  MOTOR_CMD_SAVEREG,
  MOTOR_CMD_GETID,
  MOTOR_CMD_SETID,
  MOTOR_CMD_RESETID,
  MOTOR_CMD_READ_LS_ERROR,
};

constexpr uint8_t kLsAlarmQuery = 0x24U;

// Values are loaded from motors.yaml; zero-initialized ranges are invalid.
struct MotorLimits
{
  float p_min{}, p_max{};
  float v_min{}, v_max{};
  float kp_min{}, kp_max{};
  float kd_min{}, kd_max{};
  float t_min{}, t_max{};
  float i_min{}, i_max{};
  bool valid(bool needs_current = false) const;
};

struct MotorProtocolConfig
{
  MotorLimits dm, enc, rs, ls;
  MotorLimits mr_pa43_10_40, mr_pa43_10_20_25, mr_pa35_10_8;
  float deg_to_rad{};
  bool ls_type1_new_firmware{false};

  const MotorLimits * find(MotorType type, MotorModel model = MotorModel::UNKNOWN) const;
};

typedef struct 
{
  float p_des{0.0f};
  float v_des{0.0f};
  float kp{0.0f};
  float kd{0.0f};
  float t_ff{0.0f};
  MotorModel model{MotorModel::UNKNOWN};
} mitctrl_t;

inline const mitctrl_t mit_dummy{};

typedef struct 
{
  uint8_t motor_id{0U};
  uint16_t err{0U};
  uint8_t mode{0U}; 
  bool enabled{false};
  bool enabled_known{false};
  float pos{0.0f};
  float vel{0.0f};
  float torque{0.0f};
  float winding_temperature{0.0f};
  uint8_t t_mos{0U};
  uint8_t t_rotor{0U};
} mitfb_t;

struct MotorRx
{
  MotorCommand command{MotorCommand::MOTOR_CMD_UNKNOWN};
  uint8_t can_port{0};
  uint32_t can_id{0};
  mitfb_t mit{};
  uint16_t motor_id{0};
  uint16_t addr{0};
  uint32_t value{0};
  bool ok{false};
};

/* pack/unpack 返回值: 0 = 正常, 1 = 错误(不支持的类型/空指针) */
/* 形参约定: 输入形参 *_in, 输出形参 *_out */
/* RS 29-bit CAN ID: motor_rs_{pack|unpack}_canid_<cmd> */
uint32_t motor_rs_pack_canid_mit(const MotorProtocolConfig & config_in, uint8_t motor_id_in, float t_ff_in);
int8_t motor_rs_unpack_canid_mit(uint32_t can_id_in, mitfb_t & out_out);
uint32_t motor_rs_pack_canid_enable(uint8_t motor_id_in);
uint32_t motor_rs_pack_canid_disable(uint8_t motor_id_in);
uint32_t motor_rs_pack_canid_setzero(uint8_t motor_id_in);
uint32_t motor_rs_pack_canid_reset(uint8_t motor_id_in);
uint32_t motor_rs_pack_canid_regw(uint8_t motor_id_in);
uint32_t motor_rs_pack_canid_regr(uint8_t motor_id_in);
int8_t motor_rs_unpack_canid_regr(uint32_t can_id_in, uint8_t & motor_id_out, bool & ok_out);
uint32_t motor_rs_pack_canid_regs(uint8_t motor_id_in);
uint32_t motor_rs_pack_canid_getid(uint8_t motor_id_in);
int8_t motor_rs_unpack_canid_getid(uint32_t can_id_in, uint16_t & motor_id_out, bool & ok_out);
uint32_t motor_rs_pack_canid_setid(uint8_t old_motor_id_in, uint8_t new_motor_id_in);
int8_t motor_rs_unpack_canid_setid(uint32_t can_id_in, uint16_t & motor_id_out, bool & ok_out);
uint32_t motor_rs_pack_canid_resetid(uint8_t old_motor_id_in);
int8_t motor_rs_unpack_canid_resetid(uint32_t can_id_in, uint16_t & motor_id_out, bool & ok_out);

int8_t motor_pack_mit(const MotorProtocolConfig & config_in, MotorType type_in, const mitctrl_t & params_in, std::array<uint8_t, 8> & out_out);
int8_t motor_pack_setzero(MotorType type_in, uint16_t motor_id_in, std::array<uint8_t, 8> & out_out);
int8_t motor_pack_enable(MotorType type_in, uint16_t motor_id_in, std::array<uint8_t, 8> & out_out);
int8_t motor_pack_disable(MotorType type_in, uint16_t motor_id_in, std::array<uint8_t, 8> & out_out);
int8_t motor_pack_reset(MotorType type_in, uint16_t motor_id_in, std::array<uint8_t, 8> & out_out);
int8_t motor_pack_regr(MotorType type_in, uint8_t motor_id_in, uint16_t addr_in, std::array<uint8_t, 8> & out_out);
int8_t motor_pack_regw(MotorType type_in, uint8_t motor_id_in, uint16_t addr_in, uint32_t data_in, std::array<uint8_t, 8> & out_out);
int8_t motor_pack_regs(MotorType type_in, uint8_t motor_id_in, uint16_t addr_in, std::array<uint8_t, 8> & out_out);
int8_t motor_pack_getid(MotorType type_in, std::array<uint8_t, 8> & out_out);
int8_t motor_pack_setid(MotorType type_in, uint16_t old_motor_id_in, uint16_t new_motor_id_in, std::array<uint8_t, 8> & out_out);
int8_t motor_pack_resetid(MotorType type_in, uint16_t old_motor_id_in, std::array<uint8_t, 8> & out_out);


int8_t motor_unpack_mit(const MotorProtocolConfig & config_in, MotorType type_in, const uint8_t * data_in, mitfb_t & out_out,
                        MotorModel model_in = MotorModel::UNKNOWN);
int8_t motor_unpack_setzero(const MotorProtocolConfig & config_in, MotorType type_in, const uint8_t * data_in, uint16_t & motor_id_out, mitfb_t & out_out, bool & ok_out,
                            MotorModel model_in = MotorModel::UNKNOWN);
int8_t motor_unpack_enable(const MotorProtocolConfig & config_in, MotorType type_in, const uint8_t *data_in, uint16_t & motor_id_out, mitfb_t &out_out, bool & ok_out,
                           MotorModel model_in = MotorModel::UNKNOWN);
int8_t motor_unpack_disable(const MotorProtocolConfig & config_in, MotorType type_in, const uint8_t *data_in, uint16_t & motor_id_out, mitfb_t &out_out, bool & ok_out,
                            MotorModel model_in = MotorModel::UNKNOWN);
int8_t motor_unpack_reset(const MotorProtocolConfig & config_in, MotorType type_in, const uint8_t *data_in, uint16_t & motor_id_out, mitfb_t &out_out, bool & ok_out,
                          MotorModel model_in = MotorModel::UNKNOWN);
int8_t motor_unpack_regr(MotorType type_in, const uint8_t * data_in, uint8_t & motor_id_out, uint16_t & addr_out, uint32_t & value_out);
int8_t motor_unpack_regw(MotorType type_in, const uint8_t * data_in, uint8_t & motor_id_out, uint16_t & addr_out, uint32_t & value_out);
int8_t motor_unpack_regs(MotorType type_in, const uint8_t * data_in, uint8_t & motor_id_out, uint16_t & addr_out, uint32_t & value_out);
int8_t motor_unpack_getid(MotorType type_in, const uint8_t * data_in, uint16_t & motor_id_out, bool & ok_out);
int8_t motor_unpack_setid(MotorType type_in, const uint8_t * data_in, uint16_t & motor_id_out, bool & ok_out);
int8_t motor_unpack_resetid(MotorType type_in, const uint8_t * data_in, uint16_t & motor_id_out, bool & ok_out);

void motor_decode_err(MotorType type, uint16_t err, std::string & out);

}  // namespace frameTransport

#endif  // HPM_TRANSPORT__MOTOR_HPP_
