#ifndef POLICY_FACTORY_H
#define POLICY_FACTORY_H

// #include <ros/ros.h>
#include <signal.h>

#include <functional>
#include <map>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <vector>

// forward declare global shutdown flag defined in main
extern volatile sig_atomic_t g_shutdown_request;

// Minimal policy interface used by factory
class ControllerType {
 public:
  virtual ~ControllerType() = default;
  // Blocks until shutdown (typical implementation: loop while ros::ok() &&
  // !g_shutdown_request)
  virtual void spinUntilShutdown() = 0;
};

class RLControllerFactory {
 public:
  // using Creator =
  // std::function<std::unique_ptr<ControllerType>(ros::NodeHandle&)>;
  using Creator = std::function<std::unique_ptr<ControllerType>(rclcpp::Node::SharedPtr)>;

  static RLControllerFactory& instance() {
    static RLControllerFactory inst;
    return inst;
  }

  void registerController(const std::string& name, Creator c) {
    std::cout << " registerController  name: " << name << std::endl;
    creators_[name] = std::move(c);
  }

  // Return a list of registered policy names. Useful for diagnostics.
  std::vector<std::string> registeredControllers() const {
    std::vector<std::string> names;
    names.reserve(creators_.size());
    for (const auto& kv : creators_) names.push_back(kv.first);
    return names;
  }

  // std::unique_ptr<ControllerType> create(const std::string& name,
  // ros::NodeHandle& nh) {
  std::unique_ptr<ControllerType> create(const std::string& name, rclcpp::Node::SharedPtr nh) {
    auto it = creators_.find(name);
    if (it == creators_.end()) return nullptr;
    return it->second(nh);
  }

 private:
  RLControllerFactory() = default;
  std::map<std::string, Creator> creators_;
};

// helper registrar used by translation units to register policys
struct RLControllerRegistrar {
  RLControllerRegistrar(const std::string& name, RLControllerFactory::Creator c) {
    RLControllerFactory::instance().registerController(name, std::move(c));
  }
};

#endif
