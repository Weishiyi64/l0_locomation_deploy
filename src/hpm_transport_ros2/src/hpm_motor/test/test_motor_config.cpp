#include "hpm_motor/motor.hpp"

#include <gtest/gtest.h>
#include <yaml-cpp/yaml.h>
#include "hpm_transport/motor_config.hpp"

#include <filesystem>
#include <fstream>
#include <string>

namespace
{

std::string write_config(const std::string & contents, const std::string & name)
{
  const auto path = std::filesystem::temp_directory_path() / name;
  auto config = YAML::LoadFile(HPM_TEST_MOTOR_CONFIG);
  config["motors"] = YAML::Load(contents)["motors"];
  std::ofstream(path) << config;
  return path.string();
}

TEST(MotorConfig, LoadsAllProtocols)
{
  const auto path = write_config(
    "motors:\n"
    "  dm: {index: 4, motor_type: DM, bus_index: 0, motor_id: 1}\n"
    "  rs: {index: 0, motor_type: RS, bus_index: 1, motor_id: 2}\n"
    "  enc: {index: 3, motor_type: ENC, bus_index: 2, motor_id: 3}\n"
    "  mr: {index: 1, motor_type: MR, model: PA43-10-40, bus_index: 3, motor_id: 4}\n"
    "  ls: {index: 2, motor_type: LS, bus_index: 4, motor_id: 5}\n"
    "  mr35: {index: 5, motor_type: MR, model: PA35-10-8, bus_index: 3, motor_id: 6}\n",
    "hpm_motor_all_protocols.yaml");
  const auto motors = hpmMotor::load_motor_config(path);
  EXPECT_EQ(motors.size(), 6U);
  EXPECT_EQ(motors[0].index, 4U);
  EXPECT_EQ(motors[3].index, 1U);
  EXPECT_EQ(motors[3].name, "mr");
  EXPECT_EQ(motors[3].bus_index, 3U);
  EXPECT_EQ(motors[3].motor_id, 4U);
  EXPECT_EQ(motors[3].model, frameTransport::MotorModel::MR_PA43_10_40);
  EXPECT_EQ(motors[5].model, frameTransport::MotorModel::MR_PA35_10_8);
}

TEST(MotorConfig, ParsesHexMotorId)
{
  const auto path = write_config(
    "motors:\n  left_hip: {index: 7, motor_type: MR, model: PA43-10-20.25, bus_index: 4, motor_id: 0x01}\n",
    "hpm_motor_hex_id.yaml");
  const auto motors = hpmMotor::load_motor_config(path);
  EXPECT_EQ(motors.size(), 1U);
  EXPECT_EQ(motors[0].index, 7U);
  EXPECT_EQ(motors[0].name, "left_hip");
  EXPECT_EQ(motors[0].motor_id, 1U);
}

TEST(MotorConfig, RejectsDuplicateRoute)
{
  const auto path = write_config(
    "motors:\n"
    "  first: {index: 0, motor_type: MR, model: PA43-10-20.25, bus_index: 4, motor_id: 1}\n"
    "  second: {index: 1, motor_type: DM, bus_index: 4, motor_id: 1}\n",
    "hpm_motor_duplicate.yaml");
  EXPECT_THROW(hpmMotor::load_motor_config(path), std::runtime_error);
}

TEST(MotorConfig, RejectsInvalidProtocolAndBus)
{
  const auto protocol_path = write_config(
    "motors:\n  bad: {index: 0, motor_type: OTHER, bus_index: 0, motor_id: 1}\n",
    "hpm_motor_bad_protocol.yaml");
  EXPECT_THROW(hpmMotor::load_motor_config(protocol_path), std::runtime_error);

  const auto bus_path = write_config(
    "motors:\n  bad: {index: 0, motor_type: MR, model: PA43-10-20.25, bus_index: 8, motor_id: 1}\n",
    "hpm_motor_bad_bus.yaml");
  EXPECT_THROW(hpmMotor::load_motor_config(bus_path), std::runtime_error);
}

TEST(MotorConfig, RejectsDuplicateAndOutOfRangeIndex)
{
  const auto duplicate_path = write_config(
    "motors:\n"
    "  first: {index: 3, motor_type: MR, model: PA43-10-20.25, bus_index: 4, motor_id: 1}\n"
    "  second: {index: 3, motor_type: DM, bus_index: 5, motor_id: 2}\n",
    "hpm_motor_duplicate_index.yaml");
  EXPECT_THROW(hpmMotor::load_motor_config(duplicate_path), std::runtime_error);

  const auto range_path = write_config(
    "motors:\n  bad: {index: 256, motor_type: MR, model: PA43-10-20.25, bus_index: 4, motor_id: 1}\n",
    "hpm_motor_bad_index.yaml");
  EXPECT_THROW(hpmMotor::load_motor_config(range_path), std::runtime_error);
}

TEST(MotorConfig, RequiresSupportedModelForMr)
{
  const auto missing = write_config(
    "motors:\n  missing_model: {index: 0, motor_type: MR, bus_index: 0, motor_id: 1}\n",
    "hpm_motor_missing_mr_model.yaml");
  try {
    (void)hpmMotor::load_motor_config(missing);
    FAIL() << "missing MR model was accepted";
  } catch (const std::runtime_error & error) {
    EXPECT_NE(std::string(error.what()).find("missing_model"), std::string::npos);
  }

  const auto unsupported = write_config(
    "motors:\n  bad_model: {index: 0, motor_type: MR, model: PA43-10-30, bus_index: 0, motor_id: 1}\n",
    "hpm_motor_unsupported_mr_model.yaml");
  try {
    (void)hpmMotor::load_motor_config(unsupported);
    FAIL() << "unsupported MR model was accepted";
  } catch (const std::runtime_error & error) {
    EXPECT_NE(std::string(error.what()).find("bad_model"), std::string::npos);
  }
}

TEST(MotorConfig, RejectsMissingAndInvalidProtocolValues)
{
  const auto check = [](const YAML::Node & config, const std::string & field) {
    const auto path = std::filesystem::temp_directory_path() / "hpm_invalid_protocols.yaml";
    std::ofstream(path) << config;
    try {
      (void)hpmMotor::load_motor_config(path.string());
      FAIL() << "invalid config accepted: " << field;
    } catch (const std::runtime_error & error) {
      EXPECT_NE(std::string(error.what()).find(path.string()), std::string::npos);
      EXPECT_NE(std::string(error.what()).find(field), std::string::npos);
    }
    std::filesystem::remove(path);
  };
  auto config = YAML::LoadFile(HPM_TEST_MOTOR_CONFIG);
  config.remove("motor_units");
  check(config, "motor_units");
  config = YAML::LoadFile(HPM_TEST_MOTOR_CONFIG);
  config.remove("motor_protocols");
  check(config, "motor_protocols");
  for (const char * protocol : {"DM", "ENC", "RS", "LS"}) {
    for (const char * key : {"p_min", "p_max", "v_min", "v_max", "kp_min", "kp_max",
                            "kd_min", "kd_max", "t_min", "t_max"}) {
      config = YAML::LoadFile(HPM_TEST_MOTOR_CONFIG);
      config["motor_protocols"][protocol].remove(key);
      check(config, std::string("motor_protocols.") + protocol + "." + key);
    }
  }
  for (const char * protocol : {"ENC", "LS"}) {
    for (const char * key : {"i_min", "i_max"}) {
      config = YAML::LoadFile(HPM_TEST_MOTOR_CONFIG);
      config["motor_protocols"][protocol].remove(key);
      check(config, std::string("motor_protocols.") + protocol + "." + key);
    }
  }
  for (const char * model : {"PA43-10-40", "PA43-10-20.25", "PA35-10-8"}) {
    config = YAML::LoadFile(HPM_TEST_MOTOR_CONFIG);
    config["motor_protocols"]["MR"]["models"].remove(model);
    check(config, std::string("motor_protocols.MR.models.") + model);
  }
  for (const char * invalid : {".nan", ".inf", "-.inf", "wrong", "[]", "{}", "null"}) {
    config = YAML::LoadFile(HPM_TEST_MOTOR_CONFIG);
    config["motor_protocols"]["DM"]["p_min"] = YAML::Load(invalid);
    check(config, "motor_protocols.DM.p_min");
  }
  for (float invalid : {0.0f, -1.0f}) {
    config = YAML::LoadFile(HPM_TEST_MOTOR_CONFIG);
    config["motor_units"]["deg_to_rad"] = invalid;
    check(config, "motor_units.deg_to_rad");
  }
  for (float invalid : {12.5f, 13.0f}) {
    config = YAML::LoadFile(HPM_TEST_MOTOR_CONFIG);
    config["motor_protocols"]["DM"]["p_min"] = invalid;
    check(config, "motor_protocols.DM.p");
  }
}

TEST(MotorConfig, ChangedYamlDrivesAllCodecsAndReplyDecoding)
{
  using namespace frameTransport;
  auto root = YAML::LoadFile(HPM_TEST_MOTOR_CONFIG);
  const auto change = [](YAML::Node limits) {
    limits["p_min"] = -2.0f; limits["p_max"] = 6.0f;
    limits["v_min"] = -4.0f; limits["v_max"] = 12.0f;
    limits["kp_min"] = 2.0f; limits["kp_max"] = 10.0f;
    limits["kd_min"] = 3.0f; limits["kd_max"] = 11.0f;
    limits["t_min"] = -6.0f; limits["t_max"] = 10.0f;
  };
  for (const char * protocol : {"DM", "ENC", "RS", "LS"}) {
    change(root["motor_protocols"][protocol]);
  }
  for (const char * model : {"PA43-10-40", "PA43-10-20.25", "PA35-10-8"}) {
    change(root["motor_protocols"]["MR"]["models"][model]);
  }
  for (const char * protocol : {"ENC", "LS"}) {
    root["motor_protocols"][protocol]["i_min"] = -8.0f;
    root["motor_protocols"][protocol]["i_max"] = 24.0f;
  }
  root["motor_units"]["deg_to_rad"] = 0.25f;
  const auto path = std::filesystem::temp_directory_path() / "hpm_changed_protocols.yaml";
  std::ofstream(path) << root;
  MotorProtocolConfig config;
  (void)hpmMotor::load_motor_config(path.string(), &config);
  HpmFrameTransport transport(HpmFrameTransport::Config{});
  transport.set_motor_protocol_config(config);
  for (const auto type : {MotorType::MOTOR_TYPE_DM, MotorType::MOTOR_TYPE_ENC,
                         MotorType::MOTOR_TYPE_RS, MotorType::MOTOR_TYPE_LS,
                         MotorType::MOTOR_TYPE_MR}) {
    for (const auto model : {MotorModel::MR_PA43_10_40, MotorModel::MR_PA43_10_20_25,
                            MotorModel::MR_PA35_10_8}) {
      mitctrl_t command{6.0f, 12.0f, 10.0f, 11.0f, 10.0f, model};
      std::array<uint8_t, 8> bytes{};
      ASSERT_EQ(motor_pack_mit(config, type, command, bytes), 0);
      std::array<uint8_t, 8> expected;
      expected.fill(0xFF);
      if (type == MotorType::MOTOR_TYPE_LS || type == MotorType::MOTOR_TYPE_ENC) {
        expected[0] = 0x1F;
      }
      EXPECT_EQ(bytes, expected);

      Frame frame;
      frame_init(frame);
      frame.can_id = type == MotorType::MOTOR_TYPE_RS ? 0x02000100 : 0x301;
      frame.data.fill(0xFF);
      if (type == MotorType::MOTOR_TYPE_LS) frame.data[0] = 0x20;
      MotorRx feedback;
      ASSERT_TRUE(transport.unpack_motorcmd(type, MotorCommand::MOTOR_CMD_CTRL,
                                           frame, feedback, model));
      EXPECT_FLOAT_EQ(feedback.mit.pos, 6.0f);
      EXPECT_FLOAT_EQ(feedback.mit.vel, 12.0f);
      EXPECT_FLOAT_EQ(feedback.mit.torque,
        type == MotorType::MOTOR_TYPE_ENC || type == MotorType::MOTOR_TYPE_LS ? 24.0f : 10.0f);
      // DM/RS and MR wrappers must forward the same YAML values.
      if (type != MotorType::MOTOR_TYPE_ENC && type != MotorType::MOTOR_TYPE_LS) {
        for (auto reply : {motor_unpack_enable, motor_unpack_disable,
                           motor_unpack_reset, motor_unpack_setzero}) {
          uint16_t id{};
          bool ok{};
          mitfb_t fb;
          ASSERT_EQ(reply(config, type, frame.data.data(), id, fb, ok, model), 0);
          EXPECT_FLOAT_EQ(fb.pos, 6.0f);
          EXPECT_FLOAT_EQ(fb.vel, 12.0f);
          EXPECT_FLOAT_EQ(fb.torque, 10.0f);
        }
      }
    }
  }
  EXPECT_EQ(motor_rs_pack_canid_mit(config, 1, 10.0f), 0x01FFFF01U);
  EXPECT_EQ(motor_rs_pack_canid_mit(config, 1, -6.0f), 0x01000001U);
  const uint8_t ls_degrees[8] = {0x40, 0x43, 0x34, 0, 0, 0, 0, 25}; // 180 degrees
  mitfb_t fb;
  ASSERT_EQ(motor_unpack_mit(config, MotorType::MOTOR_TYPE_LS, ls_degrees, fb), 0);
  EXPECT_FLOAT_EQ(fb.pos, 45.0f);
  std::filesystem::remove(path);
}

TEST(MotorConfig, LsFirmwareSwitchOnlyChangesTypeOneStatus)
{
  using namespace frameTransport;
  MotorProtocolConfig config;
  (void)hpmMotor::load_motor_config(HPM_TEST_MOTOR_CONFIG, &config);
  EXPECT_FALSE(config.ls_type1_new_firmware);
  HpmFrameTransport transport(HpmFrameTransport::Config{});
  Frame frame;
  frame_init(frame);
  frame.can_id = 0x301;
  frame.data = {{0x20, 0x12, 0x34, 0x56, 0x78, 0x9A, 71, 83}};
  for (const bool new_firmware : {false, true}) {
    config.ls_type1_new_firmware = new_firmware;
    transport.set_motor_protocol_config(config);
    for (uint8_t status = 0; status < 32; ++status) {
      frame.data[0] = 0x20U | status;
      MotorRx feedback;
      ASSERT_TRUE(transport.unpack_motorcmd(
        MotorType::MOTOR_TYPE_LS, MotorCommand::MOTOR_CMD_CTRL, frame, feedback));
      EXPECT_EQ(feedback.motor_id, 0x301U);
      EXPECT_EQ(feedback.mit.mode, 1U);
      EXPECT_EQ(feedback.mit.err, new_firmware ? status & 7U : status);
      EXPECT_EQ(feedback.mit.enabled_known, new_firmware);
      EXPECT_EQ(feedback.mit.enabled, new_firmware && (status & 16U) != 0U);
      EXPECT_FLOAT_EQ(feedback.mit.pos,
        config.ls.p_min + 0x1234 * (config.ls.p_max - config.ls.p_min) / 65535.0f);
      EXPECT_FLOAT_EQ(feedback.mit.vel,
        config.ls.v_min + 0x567 * (config.ls.v_max - config.ls.v_min) / 4095.0f);
      EXPECT_FLOAT_EQ(feedback.mit.torque,
        config.ls.i_min + 0x89A * (config.ls.i_max - config.ls.i_min) / 4095.0f);
      EXPECT_EQ(feedback.mit.t_rotor, 71U);
      EXPECT_EQ(feedback.mit.t_mos, 83U);
    }
    // Type 2 still uses all five error bits and its existing payload layout.
    frame.data = {{0x57, 0x43, 0x34, 0, 0, 0x03, 0xE8, 50}};
    MotorRx feedback;
    ASSERT_TRUE(transport.unpack_motorcmd(
      MotorType::MOTOR_TYPE_LS, MotorCommand::MOTOR_CMD_CTRL, frame, feedback));
    EXPECT_EQ(feedback.mit.mode, 2U);
    EXPECT_EQ(feedback.mit.err, 23U);
    EXPECT_FALSE(feedback.mit.enabled_known);
    EXPECT_FALSE(feedback.mit.enabled);
    EXPECT_FLOAT_EQ(feedback.mit.pos, 180.0f * config.deg_to_rad);
    EXPECT_FLOAT_EQ(feedback.mit.vel, 0.0f);
    EXPECT_FLOAT_EQ(feedback.mit.torque, 10.0f);
    EXPECT_EQ(feedback.mit.t_rotor, 25U);
    EXPECT_EQ(feedback.mit.t_mos, 0U);
    frame.data = {{0x20, 0x12, 0x34, 0x56, 0x78, 0x9A, 71, 83}};
  }
}

TEST(MotorConfig, RsRunModeAndMrStatusReportDriveEnable)
{
  using namespace frameTransport;
  mitfb_t fb;
  for (unsigned mode = 0; mode < 4; ++mode) {
    ASSERT_EQ(motor_rs_unpack_canid_mit(0x02001300U | (mode << 22), fb), 0);
    EXPECT_EQ(fb.enabled_known, mode < 3);
    EXPECT_EQ(fb.enabled, mode == 2);
  }
  MotorProtocolConfig config;
  (void)hpmMotor::load_motor_config(HPM_TEST_MOTOR_CONFIG, &config);
  uint8_t data[8] = {0x7F, 0xFF, 0x7F, 0xF8, 0x0B, 0x49, 0, 1};
  for (const uint8_t status : {1, 0}) {
    data[7] = status;
    ASSERT_EQ(motor_unpack_mit(config, MotorType::MOTOR_TYPE_MR, data, fb,
                              MotorModel::MR_PA43_10_20_25), 0);
    EXPECT_TRUE(fb.enabled_known);
    EXPECT_EQ(fb.enabled, status == 1);
  }
}

TEST(MotorConfig, LsFirmwareStartupParameterConfiguresTransport)
{
  using namespace frameTransport;
  auto context = std::make_shared<rclcpp::Context>();
  context->init(0, nullptr);
  for (const std::string version : {"", "old", "new", "invalid"}) {
    rclcpp::NodeOptions options;
    options.context(context);
    options.parameter_overrides({rclcpp::Parameter("motor_config_file", HPM_TEST_MOTOR_CONFIG)});
    if (!version.empty()) {
      options.append_parameter_override("ls_firmware_version", version);
    }
    rclcpp::Node node("ls_firmware_test", options);
    HpmFrameTransport transport(HpmFrameTransport::Config{});
    if (version == "invalid") {
      EXPECT_THROW(hpmMotor::HpmMotor(node, transport), std::invalid_argument);
      continue;
    }
    hpmMotor::HpmMotor motor(node, transport);
    EXPECT_EQ(node.get_parameter("ls_firmware_version").as_string(),
      version.empty() ? "old" : version);
    EXPECT_TRUE(node.describe_parameter("ls_firmware_version").read_only);
    EXPECT_FALSE(node.set_parameter(rclcpp::Parameter(
      "ls_firmware_version", version == "new" ? "old" : "new")).successful);
    Frame frame;
    frame_init(frame);
    frame.can_id = 1;
    frame.data = {{0x30, 0x12, 0x34, 0x56, 0x78, 0x9A, 71, 83}};
    MotorRx feedback;
    ASSERT_TRUE(transport.unpack_motorcmd(
      MotorType::MOTOR_TYPE_LS, MotorCommand::MOTOR_CMD_CTRL, frame, feedback));
    EXPECT_EQ(feedback.mit.err, version == "new" ? 0U : 16U);
    EXPECT_EQ(feedback.mit.enabled, version == "new");
  }
  context->shutdown("LS firmware parameter check complete");
}

TEST(MotorConfig, UnconfiguredTransportAndCodecsRejectMotorOperations)
{
  using namespace frameTransport;
  HpmFrameTransport transport(HpmFrameTransport::Config{});
  Frame frame;
  MotorRx feedback;
  EXPECT_FALSE(transport.unpack_motorcmd(MotorType::MOTOR_TYPE_DM,
    MotorCommand::MOTOR_CMD_CTRL, frame, feedback));
  EXPECT_FALSE(transport.send_motorcmd(MotorType::MOTOR_TYPE_DM,
    MotorCommand::MOTOR_CMD_ENABLE, 1, 0));
  EXPECT_FALSE(transport.send_motor_controls({{MotorType::MOTOR_TYPE_DM, 1, 0, {}}}));
  EXPECT_FALSE(transport.send_motor_enables({{MotorType::MOTOR_TYPE_DM, 1, 0, true}}));
  EXPECT_FALSE(transport.send_motor_id_write(MotorType::MOTOR_TYPE_RS, 1, 2, 0));
  EXPECT_THROW(transport.set_motor_protocol_config(MotorProtocolConfig{}), std::invalid_argument);
  std::array<uint8_t, 8> bytes{};
  EXPECT_NE(motor_pack_mit(MotorProtocolConfig{}, MotorType::MOTOR_TYPE_DM, {}, bytes), 0);
  EXPECT_THROW(motor_rs_pack_canid_mit(MotorProtocolConfig{}, 1, 0), std::invalid_argument);
}

}  // namespace
