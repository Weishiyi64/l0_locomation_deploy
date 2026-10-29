#pragma once
// #include <ros/ros.h>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <vector>

namespace legged {
class ParameterManager {
 public:
  // explicit ParameterManager(ros::NodeHandle nh): nh_(nh) {}
  explicit ParameterManager(rclcpp::Node::SharedPtr nh) : nh_(nh) {}

  bool loadVector(const std::string& key, std::vector<double>& out) {
    // if (!nh_.getParam(key, out)) {
    if (!nh_->get_parameter(key, out))
    {
      RCLCPP_ERROR_STREAM(nh_->get_logger(), "Parameter missing: " << key);
      return false;
    }
    return true;
  }
  bool loadDouble(const std::string& key, double& out) {
    // if (!nh_.getParam(key, out)) {
    if (!nh_->get_parameter(key, out))
    {
      RCLCPP_ERROR_STREAM(nh_->get_logger(), "Parameter missing: " << key);
      return false;
    }
    return true;
  }
  template <typename T>
  bool load(const std::string& key, T& out) {
    // if (!nh_.getParam(key, out)) {
    if (!nh_->get_parameter(key, out))
    {
      RCLCPP_ERROR_STREAM(nh_->get_logger(), "Parameter missing: " << key);
      return false;
    }
    return true;
  }

 private:
  // ros::NodeHandle nh_;
  rclcpp::Node::SharedPtr nh_;
};
}  // namespace legged
