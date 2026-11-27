#pragma once

#include <string>
#include <vector>
// #include <ros/ros.h>
#include <Eigen/Dense>
#include <rclcpp/rclcpp.hpp>

namespace order_mapping {

class OrderMapper {
 public:
  OrderMapper(rclcpp::Logger logger) : logger_(logger) {};
  ~OrderMapper() = default;

  void initialize(const std::vector<std::string>& joint_order_mj, const std::vector<std::string>& joint_order_isaac);

  void transformMjToIsaac(const Eigen::VectorXd& mj_vec, Eigen::VectorXd& isaac_vec) const;

  void mapJointObservation(const std::vector<double>& positions, const std::vector<double>& velocities,
                           const std::vector<double>& default_angles, Eigen::VectorXd& q, Eigen::VectorXd& dq) const;

  void transformIsaacToMj(const Eigen::VectorXd& isaac_vec, Eigen::VectorXd& mj_vec) const;

 private:
  std::vector<int> mj_to_isaac_;
  std::vector<int> isaac_to_mj_;
  rclcpp::Logger logger_;
};

}  // namespace order_mapping
