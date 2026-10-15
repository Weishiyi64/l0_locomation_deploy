#include "actuator_bridge/config_loader.h"

#include <yaml-cpp/yaml.h>

#include <rclcpp/rclcpp.hpp>
#include <unordered_set>

namespace actuator_bridge {

bool ConfigLoader::loadBusesConfig(const std::string& yaml_path, std::vector<BusConfig>& buses_config) {
  YAML::Node root;
  try
  { root = YAML::LoadFile(yaml_path); } catch (const YAML::Exception& e)
  {
    RCLCPP_ERROR(rclcpp::get_logger("config_loader"), "Failed to load YAML file '%s': %s", yaml_path.c_str(), e.what());
    return false;
  }

  if (!root["vendors"] || !root["vendors"].IsSequence())
  {
    RCLCPP_ERROR(rclcpp::get_logger("config_loader"), "YAML must contain a 'vendors' sequence");
    return false;
  }

  buses_config.clear();
  std::unordered_set<std::string> seen_names;

  for (std::size_t i = 0; i < root["vendors"].size(); ++i)
  {
    BusConfig bus_config;
    if (!parseBusConfig(root["vendors"][i], static_cast<int>(i), bus_config))
    {
      return false;
    }

    for (const auto& actuator : bus_config.actuators)
    {
      if (seen_names.find(actuator.name) != seen_names.end())
      {
        RCLCPP_ERROR(rclcpp::get_logger("config_loader"), "Duplicate actuator name detected: %s",
                     actuator.name.c_str());
        return false;
      }
      seen_names.insert(actuator.name);
    }

    buses_config.push_back(std::move(bus_config));
  }

  return true;
}

bool ConfigLoader::parseBusConfig(const YAML::Node& bus_node, int bus_index, BusConfig& bus_config) {
  if (!bus_node.IsMap())
  {
    RCLCPP_ERROR(rclcpp::get_logger("config_loader"), "bus[%d] must be a map", bus_index);
    return false;
  }

  if (!bus_node["vendor"] || !bus_node["vendor"].IsScalar())
  {
    RCLCPP_ERROR(rclcpp::get_logger("config_loader"), "bus[%d] missing or invalid 'vendor' field", bus_index);
    return false;
  }
  bus_config.vendor = bus_node["vendor"].as<std::string>();

  if (!bus_node["models"])
  {
    RCLCPP_ERROR(rclcpp::get_logger("config_loader"), "bus[%d] vendor=%s: missing 'models'", bus_index,
                 bus_config.vendor.c_str());
    return false;
  }
  std::unordered_map<std::string, Limits> model_limits;
  if (!parseModelLimits(bus_node["models"], model_limits))
  {
    RCLCPP_ERROR(rclcpp::get_logger("config_loader"), "bus[%d] vendor=%s: failed to parse 'models'", bus_index,
                 bus_config.vendor.c_str());
    return false;
  }

  if (!bus_node["actuators"] || !bus_node["actuators"].IsSequence() || bus_node["actuators"].size() == 0)
  {
    RCLCPP_ERROR(rclcpp::get_logger("config_loader"), "bus[%d] vendor=%s: 'actuators' must be a non-empty sequence",
                 bus_index, bus_config.vendor.c_str());
    return false;
  }

  const YAML::Node& actuators_node = bus_node["actuators"];
  for (std::size_t j = 0; j < actuators_node.size(); ++j)
  {
    ActuatorSpec actuator_spec;
    if (!parseActuatorConfig(actuators_node[j], bus_index, static_cast<int>(j), bus_config.vendor, model_limits,
                             actuator_spec))
    {
      return false;
    }
    bus_config.actuators.push_back(std::move(actuator_spec));
  }

  return true;
}

bool ConfigLoader::parseModelLimits(const YAML::Node& models_node,
                                    std::unordered_map<std::string, Limits>& model_limits) {
  if (!models_node.IsMap())
  {
    RCLCPP_ERROR(rclcpp::get_logger("config_loader"), "models must be a map");
    return false;
  }

  const char* required_numeric_keys[] = {"p_min",  "p_max",  "v_min",  "v_max",   "t_min",   "t_max", "kp_min",
                                         "kp_max", "kd_min", "kd_max", "cur_min", "cur_max", "kt"};

  for (auto it = models_node.begin(); it != models_node.end(); ++it)
  {
    std::string model_name = it->first.as<std::string>();
    const YAML::Node& model_yaml = it->second;

    if (!model_yaml.IsMap())
    {
      RCLCPP_ERROR(rclcpp::get_logger("config_loader"), "Model '%s' must be a map", model_name.c_str());
      return false;
    }

    for (const char* key : required_numeric_keys)
    {
      if (!model_yaml[key])
      {
        RCLCPP_ERROR(rclcpp::get_logger("config_loader"), "Model '%s' missing required field '%s'", model_name.c_str(),
                     key);
        return false;
      }
    }

    if (!model_yaml["rotation_direction"])
    {
      RCLCPP_ERROR(rclcpp::get_logger("config_loader"), "Model '%s' missing required field 'rotation_direction'",
                   model_name.c_str());
      return false;
    }
    if (!model_yaml["rotation_direction"].IsScalar())
    {
      RCLCPP_ERROR(rclcpp::get_logger("config_loader"),
                   "Model '%s': rotation_direction must be a string ('CW' or 'CCW')", model_name.c_str());
      return false;
    }

    Limits lim;
    lim.p_min = model_yaml["p_min"].as<double>();
    lim.p_max = model_yaml["p_max"].as<double>();
    lim.v_min = model_yaml["v_min"].as<double>();
    lim.v_max = model_yaml["v_max"].as<double>();
    lim.t_min = model_yaml["t_min"].as<double>();
    lim.t_max = model_yaml["t_max"].as<double>();
    lim.kp_min = model_yaml["kp_min"].as<double>();
    lim.kp_max = model_yaml["kp_max"].as<double>();
    lim.kd_min = model_yaml["kd_min"].as<double>();
    lim.kd_max = model_yaml["kd_max"].as<double>();
    lim.cur_min = model_yaml["cur_min"].as<double>();
    lim.cur_max = model_yaml["cur_max"].as<double>();
    lim.kt = model_yaml["kt"].as<double>();

    std::string rot_dir_str = model_yaml["rotation_direction"].as<std::string>();
    if (rot_dir_str == "CW")
    {
      lim.rotation_direction = -1.0f;
    }
    else if (rot_dir_str == "CCW")
    {
      lim.rotation_direction = 1.0f;
    }
    else
    {
      RCLCPP_ERROR(rclcpp::get_logger("config_loader"),
                   "Model '%s': invalid rotation_direction '%s', must be 'CW' or 'CCW'", model_name.c_str(),
                   rot_dir_str.c_str());
      return false;
    }

    model_limits[model_name] = lim;
  }

  return true;
}

bool ConfigLoader::parseActuatorConfig(const YAML::Node& actuator_yaml, int bus_index, int actuator_index,
                                       const std::string& vendor,
                                       const std::unordered_map<std::string, Limits>& model_limits,
                                       ActuatorSpec& actuator_spec) {
  if (!actuator_yaml.IsMap())
  {
    RCLCPP_ERROR(rclcpp::get_logger("config_loader"), "bus[%d] actuator[%d] must be a map", bus_index, actuator_index);
    return false;
  }

  if (!actuator_yaml["name"] || !actuator_yaml["name"].IsScalar())
  {
    RCLCPP_ERROR(rclcpp::get_logger("config_loader"), "bus[%d] actuator[%d] missing or invalid 'name'", bus_index,
                 actuator_index);
    return false;
  }
  actuator_spec.name = actuator_yaml["name"].as<std::string>();

  if (!actuator_yaml["id"] || !(actuator_yaml["id"].IsScalar()))
  {
    RCLCPP_ERROR(rclcpp::get_logger("config_loader"), "bus[%d] actuator[%d] missing or invalid 'id'", bus_index,
                 actuator_index);
    return false;
  }
  actuator_spec.id = actuator_yaml["id"].as<int>();

  const char* string_fields[] = {"model", "can_device"};
  for (const char* field : string_fields)
  {
    if (!actuator_yaml[field] || !actuator_yaml[field].IsScalar())
    {
      RCLCPP_ERROR(rclcpp::get_logger("config_loader"), "bus[%d] actuator[%d] (%s): missing or invalid '%s'", bus_index,
                   actuator_index, actuator_spec.name.c_str(), field);
      return false;
    }
  }
  actuator_spec.model = actuator_yaml["model"].as<std::string>();
  actuator_spec.can_device = actuator_yaml["can_device"].as<std::string>();

  if (model_limits.find(actuator_spec.model) == model_limits.end())
  {
    RCLCPP_ERROR(rclcpp::get_logger("config_loader"),
                 "bus[%d] actuator[%d] (%s): model '%s' not defined in models table", bus_index, actuator_index,
                 actuator_spec.name.c_str(), actuator_spec.model.c_str());
    return false;
  }

  const char* numeric_fields[] = {"reduction", "zero_offset", "motor_direction", "joint_lower_limit",
                                  "joint_upper_limit"};
  for (const char* field : numeric_fields)
  {
    if (!actuator_yaml[field])
    {
      RCLCPP_ERROR(rclcpp::get_logger("config_loader"), "bus[%d] actuator[%d] (%s): missing required '%s'", bus_index,
                   actuator_index, actuator_spec.name.c_str(), field);
      return false;
    }
  }

  // Parse numeric fields
  actuator_spec.reduction = actuator_yaml["reduction"].as<double>();
  actuator_spec.zero_offset = actuator_yaml["zero_offset"].as<double>();
  actuator_spec.joint_lower_limit = actuator_yaml["joint_lower_limit"].as<double>();
  actuator_spec.joint_upper_limit = actuator_yaml["joint_upper_limit"].as<double>();

  // motor_direction may be 1.0/-1.0 (float) in YAML; yaml-cpp .as<int>() fails
  // on float, use .as<double>()
  int dir_int = static_cast<int>(actuator_yaml["motor_direction"].as<double>());
  if (dir_int != 1 && dir_int != -1)
  {
    RCLCPP_ERROR(rclcpp::get_logger("config_loader"),
                 "bus[%d] actuator[%d] (%s): motor_direction must be 1 or -1, got %d", bus_index, actuator_index,
                 actuator_spec.name.c_str(), dir_int);
    return false;
  }
  actuator_spec.motor_direction = dir_int;

  if (actuator_spec.joint_lower_limit >= actuator_spec.joint_upper_limit)
  {
    RCLCPP_ERROR(rclcpp::get_logger("config_loader"),
                 "bus[%d] actuator[%d] (%s): joint_lower_limit (%.3f) must be "
                 "less than joint_upper_limit (%.3f)",
                 bus_index, actuator_index, actuator_spec.name.c_str(), actuator_spec.joint_lower_limit,
                 actuator_spec.joint_upper_limit);
    return false;
  }

  actuator_spec.vendor = vendor;
  actuator_spec.limits = model_limits.at(actuator_spec.model);

  // Parse mode (same semantics as ROS1: enable, disable, fake)
  if (!actuator_yaml["mode"] || !actuator_yaml["mode"].IsScalar())
  {
    RCLCPP_ERROR(rclcpp::get_logger("config_loader"),
                 "bus[%d] actuator[%d] (%s): missing required field 'mode' "
                 "(valid values: enable, disable, fake)",
                 bus_index, actuator_index, actuator_spec.name.c_str());
    return false;
  }
  {
    std::string mode_str = actuator_yaml["mode"].as<std::string>();
    if (mode_str == "enable")
    {
      actuator_spec.mode = ActuatorMode::ENABLE;
    }
    else if (mode_str == "disable")
    {
      actuator_spec.mode = ActuatorMode::DISABLE;
    }
    else if (mode_str == "fake")
    {
      actuator_spec.mode = ActuatorMode::FAKE;
    }
    else
    {
      RCLCPP_ERROR(rclcpp::get_logger("config_loader"),
                   "bus[%d] actuator[%d] (%s): invalid mode '%s' "
                   "(valid values: enable, disable, fake)",
                   bus_index, actuator_index, actuator_spec.name.c_str(), mode_str.c_str());
      return false;
    }
  }

  if (actuator_yaml["tag"] && actuator_yaml["tag"].IsScalar())
  {
    actuator_spec.tag = actuator_yaml["tag"].as<std::string>();
  }
  else
  {
    actuator_spec.tag = "";
    RCLCPP_WARN(rclcpp::get_logger("config_loader"), "bus[%d] actuator[%d] (%s): missing 'tag', using empty string",
                bus_index, actuator_index, actuator_spec.name.c_str());
  }

  if (actuator_yaml["alias"] && actuator_yaml["alias"].IsScalar())
  {
    actuator_spec.alias = actuator_yaml["alias"].as<std::string>();
  }
  else
  {
    actuator_spec.alias = "";
  }

  return true;
}

}  // namespace actuator_bridge
