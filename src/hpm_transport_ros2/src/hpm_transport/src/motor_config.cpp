/*
 * Copyright (c) 2026 HPMicro
 * SPDX-License-Identifier: BSD-3-Clause
 */
#include "hpm_transport/motor_config.hpp"

#include <yaml-cpp/yaml.h>
#include <stdexcept>
#include <utility>

namespace frameTransport
{
namespace
{
float read_float(const YAML::Node & node, const std::string & key,
                 const std::string & field)
{
  try {
    const float value = node[key].as<float>();
    if (std::isfinite(value)) {
      return value;
    }
  } catch (const YAML::Exception &) {
    // Include the complete field path rather than only a YAML line number.
  }
  throw std::runtime_error(field + "." + key + " must be a finite number");
}

MotorLimits read_limits(const YAML::Node & node, const std::string & field, bool current)
{
  MotorLimits limits;
  const auto range = [&](const char * name, float & low, float & high) {
    const std::string prefix(name);
    low = read_float(node, prefix + "_min", field);
    high = read_float(node, prefix + "_max", field);
    if (!(low < high) || !std::isfinite(high - low)) {
      throw std::runtime_error(field + "." + prefix + " requires min < max and a finite span");
    }
  };
  range("p", limits.p_min, limits.p_max);
  range("v", limits.v_min, limits.v_max);
  range("kp", limits.kp_min, limits.kp_max);
  range("kd", limits.kd_min, limits.kd_max);
  range("t", limits.t_min, limits.t_max);
  if (current) {
    range("i", limits.i_min, limits.i_max);
  }
  return limits;
}
}  // namespace

void validate_motor_protocol_config(const MotorProtocolConfig & config)
{
  for (const auto & entry : {
      std::make_pair("DM", &config.dm), std::make_pair("ENC", &config.enc),
      std::make_pair("RS", &config.rs), std::make_pair("LS", &config.ls),
      std::make_pair("MR.models.PA43-10-40", &config.mr_pa43_10_40),
      std::make_pair("MR.models.PA43-10-20.25", &config.mr_pa43_10_20_25),
      std::make_pair("MR.models.PA35-10-8", &config.mr_pa35_10_8)}) {
    if (!entry.second->valid(entry.second == &config.enc || entry.second == &config.ls)) {
      throw std::invalid_argument(std::string("invalid motor_protocols.") + entry.first);
    }
  }
  if (!std::isfinite(config.deg_to_rad) || config.deg_to_rad <= 0.0f) {
    throw std::invalid_argument("motor_units.deg_to_rad must be positive and finite");
  }
}

MotorProtocolConfig parse_motor_protocol_config(const YAML::Node & root, const std::string & path)
{
  try {
    MotorProtocolConfig config;
    config.deg_to_rad = read_float(root["motor_units"], "deg_to_rad", "motor_units");
    if (config.deg_to_rad <= 0.0f) {
      throw std::runtime_error("motor_units.deg_to_rad must be positive");
    }
    const auto protocols = root["motor_protocols"];
    config.dm = read_limits(protocols["DM"], "motor_protocols.DM", false);
    config.enc = read_limits(protocols["ENC"], "motor_protocols.ENC", true);
    config.rs = read_limits(protocols["RS"], "motor_protocols.RS", false);
    config.ls = read_limits(protocols["LS"], "motor_protocols.LS", true);
    const auto models = protocols["MR"]["models"];
    config.mr_pa43_10_40 = read_limits(models["PA43-10-40"],
      "motor_protocols.MR.models.PA43-10-40", false);
    config.mr_pa43_10_20_25 = read_limits(models["PA43-10-20.25"],
      "motor_protocols.MR.models.PA43-10-20.25", false);
    config.mr_pa35_10_8 = read_limits(models["PA35-10-8"],
      "motor_protocols.MR.models.PA35-10-8", false);
    return config;
  } catch (const std::exception & error) {
    throw std::runtime_error(path + ": " + error.what());
  }
}

MotorProtocolConfig load_motor_protocol_config(const std::string & path)
{
  YAML::Node root;
  try {
    root = YAML::LoadFile(path);
  } catch (const YAML::Exception & error) {
    throw std::runtime_error(path + ": " + error.what());
  }
  return parse_motor_protocol_config(root, path);
}
}  // namespace frameTransport
