#pragma once
#include <functional>
#include <memory>
#include <string>
#include <unordered_map>

#include "actuator_bridge/driver_base.h"

namespace actuator_bridge {

class DriverRegistry {
 public:
  using FactoryFn = std::function<std::shared_ptr<DriverBase>()>;

  static DriverRegistry& instance();

  void registerFactory(const std::string& vendor, FactoryFn fn);
  std::shared_ptr<DriverBase> create(const std::string& vendor) const;

 private:
  std::unordered_map<std::string, FactoryFn> map_;
};

class DriverRegistrar {
 public:
  DriverRegistrar(const std::string& vendor, DriverRegistry::FactoryFn fn) {
    DriverRegistry::instance().registerFactory(vendor, fn);
  }
};

}  // namespace actuator_bridge
