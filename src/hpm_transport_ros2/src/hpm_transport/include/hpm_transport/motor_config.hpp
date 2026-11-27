/*
 * Copyright (c) 2026 HPMicro
 * SPDX-License-Identifier: BSD-3-Clause
 */
#ifndef HPM_TRANSPORT__MOTOR_CONFIG_HPP_
#define HPM_TRANSPORT__MOTOR_CONFIG_HPP_

#include "hpm_transport/motor.hpp"

namespace YAML { class Node; }

namespace frameTransport
{
// Throws with the config path and field name. No fallback values are supplied.
MotorProtocolConfig parse_motor_protocol_config(const YAML::Node & root, const std::string & path);
MotorProtocolConfig load_motor_protocol_config(const std::string & path);
void validate_motor_protocol_config(const MotorProtocolConfig & config);
}  // namespace frameTransport

#endif
