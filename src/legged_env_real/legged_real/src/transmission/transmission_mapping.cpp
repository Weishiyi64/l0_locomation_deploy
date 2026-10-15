#include "legged_real/transmission/transmission_mapping.h"

#include <rclcpp/rclcpp.hpp>

TransmissionMapping::TransmissionMapping(double direction_p, double direction_r, double offset_p, double offset_r)
    : direction_p_(direction_p), direction_r_(direction_r), offset_p_(offset_p), offset_r_(offset_r) {}

bool TransmissionMapping::calc_position_cmd(double theta_p_cmd_urdf, double theta_r_cmd_urdf) {
  theta_p_cmd_urdf_ = theta_p_cmd_urdf;
  theta_r_cmd_urdf_ = theta_r_cmd_urdf;

  urdf2model();

  if (!inverse_transmission_mapping())
  {
    RCLCPP_ERROR(rclcpp::get_logger("transmission_mapping"),
                 "[TransmissionMapping] Failed to update inverse transmission mapping, theta_p_cmd_urdf: %f, "
                 "theta_r_cmd_urdf: %f",
                 theta_p_cmd_urdf, theta_r_cmd_urdf);
    return false;
  }

  return true;
}

Eigen::Vector2d TransmissionMapping::get_position_cmd() { return Eigen::Vector2d(theta_u_cmd_, theta_l_cmd_); }

bool TransmissionMapping::calc_torque_cmd(double theta_p_cmd_urdf, double theta_r_cmd_urdf, double kp_p, double kp_r,
                                          double kd_p, double kd_r) {
  theta_p_cmd_urdf_ = theta_p_cmd_urdf;
  theta_r_cmd_urdf_ = theta_r_cmd_urdf;

  Kp.diagonal() = Eigen::Vector2d(kp_p, kp_r);
  Kd.diagonal() = Eigen::Vector2d(kd_p, kd_r);

  urdf2model();

  if (!pd_control_torque())
  {
    RCLCPP_ERROR(rclcpp::get_logger("transmission_mapping"),
                 "[TransmissionMapping] Failed to update PD control torque, theta_p_cmd_urdf: %f, theta_r_cmd_urdf: %f",
                 theta_p_cmd_urdf, theta_r_cmd_urdf);
    return false;
  }

  return true;
}

Eigen::Vector2d TransmissionMapping::get_torque_cmd() { return Eigen::Vector2d(tau_u_cmd_, tau_l_cmd_); }

bool TransmissionMapping::calc_state(double theta_u, double theta_l, double theta_u_dot, double theta_l_dot) {
  theta_u_ = theta_u;
  theta_l_ = theta_l;
  theta_u_dot_ = theta_u_dot;
  theta_l_dot_ = theta_l_dot;

  if (!transmission_mapping())
  {
    RCLCPP_ERROR(rclcpp::get_logger("transmission_mapping"),
                 "[TransmissionMapping] Failed to update transmission mapping, theta_u: %f, theta_l: %f, theta_u_dot: "
                 "%f, theta_l_dot: %f",
                 theta_u, theta_l, theta_u_dot, theta_l_dot);
    return false;
  }

  if (!jacobian_update())
  {
    RCLCPP_ERROR(rclcpp::get_logger("transmission_mapping"),
                 "[TransmissionMapping] Failed to update Jacobian matrix, theta_u: %f, theta_l: %f, theta_u_dot: %f, "
                 "theta_l_dot: %f",
                 theta_u, theta_l, theta_u_dot, theta_l_dot);
    return false;
  }

  if (!transmission_velocity_mapping())
  {
    RCLCPP_ERROR(rclcpp::get_logger("transmission_mapping"),
                 "[TransmissionMapping] Failed to update transmission velocity mapping, theta_u: %f, theta_l: %f, "
                 "theta_u_dot: %f, theta_l_dot: %f",
                 theta_u, theta_l, theta_u_dot, theta_l_dot);
    return false;
  }

  model2urdf();

  return true;
}

Eigen::Vector4d TransmissionMapping::get_state() {
  return Eigen::Vector4d(theta_p_urdf_, theta_r_urdf_, theta_p_dot_urdf_, theta_r_dot_urdf_);
}
