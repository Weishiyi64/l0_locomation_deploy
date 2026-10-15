#pragma once

#include <yaml-cpp/yaml.h>

#include <string>
#include <unordered_map>
#include <vector>

#include "actuator_bridge/config_types.h"

namespace actuator_bridge {

class ConfigLoader {
 public:
  /**
   * @brief 从 YAML 文件加载总线配置
   * @param yaml_path YAML 配置文件路径
   * @param buses_config 输出：总线配置列表
   * @return 成功返回 true，失败返回 false
   */
  static bool loadBusesConfig(const std::string& yaml_path, std::vector<BusConfig>& buses_config);

 private:
  // Helpers for YAML parsing; similar functionality to legacy XML-RPC version
  static bool parseBusConfig(const YAML::Node& bus_node, int bus_index, BusConfig& bus_config);
  static bool parseModelLimits(const YAML::Node& models_node, std::unordered_map<std::string, Limits>& model_limits);
  static bool parseActuatorConfig(const YAML::Node& actuator_node, int bus_index, int actuator_index,
                                  const std::string& vendor,
                                  const std::unordered_map<std::string, Limits>& model_limits,
                                  ActuatorSpec& actuator_spec);
};

}  // namespace actuator_bridge
