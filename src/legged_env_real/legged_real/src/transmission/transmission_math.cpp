#include <rclcpp/rclcpp.hpp>

#include "legged_real/transmission/transmission_mapping.h"

bool TransmissionMapping::urdf2model() {
  theta_p_cmd_model_ = (theta_p_cmd_urdf_ - offset_p_) * direction_p_;
  theta_r_cmd_model_ = (theta_r_cmd_urdf_ - offset_r_) * direction_r_;

  return true;
}

bool TransmissionMapping::model2urdf() {
  theta_p_urdf_ = (theta_p_model_ * direction_p_) + offset_p_;
  theta_r_urdf_ = (theta_r_model_ * direction_r_) + offset_r_;

  theta_p_dot_urdf_ = theta_p_dot_model_ * direction_p_;
  theta_r_dot_urdf_ = theta_r_dot_model_ * direction_r_;

  return true;
}

bool TransmissionMapping::inverse_transmission_mapping() {
  double _theta_u_cmd = theta_u_calc(theta_p_cmd_model_, theta_r_cmd_model_);
  double _theta_l_cmd = theta_l_calc(theta_p_cmd_model_, theta_r_cmd_model_);

  if (!std::isfinite(_theta_u_cmd) || !std::isfinite(_theta_l_cmd))
  {
    RCLCPP_ERROR(rclcpp::get_logger("transmission_math"),
                 "[TransmissionMath] Failed to calculate theta_u_cmd and theta_l_cmd, theta_p_cmd_model_: %f, "
                 "theta_r_cmd_model_: %f",
                 theta_p_cmd_model_, theta_r_cmd_model_);
    return false;
  }

  theta_u_cmd_ = _theta_u_cmd;
  theta_l_cmd_ = _theta_l_cmd;
  return true;
}

bool TransmissionMapping::transmission_mapping() {
  const int MAX_ITERATIONS = 100;
  const double CONVERGENCE_THRESHOLD = 1e-5;

  // Initialize with last successful solution
  double theta_p_model_guess = last_theta_p_;
  double theta_r_model_guess = last_theta_r_;

  for (int iter = 0; iter < MAX_ITERATIONS; ++iter)
  {
    // Calculate constraint equations
    double eq1_val = eq1_calc(theta_p_model_guess, theta_r_model_guess, theta_u_, theta_l_);
    double eq2_val = eq2_calc(theta_p_model_guess, theta_r_model_guess, theta_u_, theta_l_);

    // Check for numerical issues
    if (!std::isfinite(eq1_val) || !std::isfinite(eq2_val))
    {
      RCLCPP_ERROR(rclcpp::get_logger("transmission_math"),
                   "[TransmissionMath] Failed to calculate eq1_val and eq2_val, theta_p_model_guess: %f, "
                   "theta_r_model_guess: %f, theta_u_: %f, theta_l_: %f",
                   theta_p_model_guess, theta_r_model_guess, theta_u_, theta_l_);
      return false;
    }

    // Check convergence: constraint equations should be close to zero
    Eigen::Vector2d F_guess;
    F_guess << eq1_val, eq2_val;
    double error_norm = F_guess.norm();
    if (error_norm < CONVERGENCE_THRESHOLD)
    {
      // Successfully converged, update last solution
      last_theta_p_ = theta_p_model_guess;
      last_theta_r_ = theta_r_model_guess;
      theta_p_model_ = theta_p_model_guess;
      theta_r_model_ = theta_r_model_guess;
      return true;
    }

    // Calculate Jacobian
    double J_pr_guess_array[4];
    J_pr_calc(theta_p_model_guess, theta_r_model_guess, theta_u_, theta_l_, J_pr_guess_array);

    Eigen::Matrix2d J_pr_guess;
    J_pr_guess << J_pr_guess_array[0], J_pr_guess_array[2], J_pr_guess_array[1], J_pr_guess_array[3];

    // Check singularity
    double det = J_pr_guess.determinant();
    if (std::abs(det) < 1e-6)
    {
      RCLCPP_ERROR(rclcpp::get_logger("transmission_math"),
                   "[TransmissionMath] Failed to calculate J_pr_guess, theta_p_model_guess: %f, theta_r_model_guess: "
                   "%f, theta_u_: %f, theta_l_: %f",
                   theta_p_model_guess, theta_r_model_guess, theta_u_, theta_l_);
      return false;
    }

    // Newton-Raphson update
    Eigen::Vector2d delta = -J_pr_guess.inverse() * F_guess;

    theta_p_model_guess += delta(0);
    theta_r_model_guess += delta(1);

    // Check for numerical issues
    if (!std::isfinite(theta_p_model_guess) || !std::isfinite(theta_r_model_guess))
    {
      RCLCPP_ERROR(rclcpp::get_logger("transmission_math"),
                   "[TransmissionMath] Failed to calculate theta_p_model_guess and theta_r_model_guess, "
                   "theta_p_model_guess: %f, theta_r_model_guess: %f",
                   theta_p_model_guess, theta_r_model_guess);
      return false;
    }
  }

  // Failed to converge
  return false;
}

bool TransmissionMapping::jacobian_update() {
  double J_pr_array[4], J_ul_array[4];
  J_pr_calc(theta_p_model_, theta_r_model_, theta_u_, theta_l_, J_pr_array);
  J_ul_calc(theta_p_model_, theta_r_model_, theta_u_, theta_l_, J_ul_array);

  Eigen::Matrix2d J_pr, J_ul;
  J_pr << J_pr_array[0], J_pr_array[2], J_pr_array[1], J_pr_array[3];
  J_ul << J_ul_array[0], J_ul_array[2], J_ul_array[1], J_ul_array[3];

  // Check singularity
  double det_pr = J_pr.determinant();
  if (std::abs(det_pr) < 1e-6)
  {
    RCLCPP_ERROR(rclcpp::get_logger("transmission_math"),
                 "[TransmissionMath] Failed to calculate J_pr, theta_p_model_: %f, theta_r_model_: %f, theta_u_: %f, "
                 "theta_l_: %f, det_pr: %f",
                 theta_p_model_, theta_r_model_, theta_u_, theta_l_, det_pr);
    return false;
  }

  J_pr_ = J_pr;
  J_ul_ = J_ul;
  J_pr_inv_ = J_pr.inverse();
  J_ul_trans_ = J_ul.transpose();
  J_pr_inv_trans_ = J_pr_inv_.transpose();

  return true;
}

bool TransmissionMapping::transmission_velocity_mapping() {
  Eigen::Vector2d ul_vel(theta_u_dot_, theta_l_dot_);
  Eigen::Vector2d pr_vel = -J_pr_inv_ * J_ul_ * ul_vel;

  // Check for numerical issues
  if (!std::isfinite(pr_vel(0)) || !std::isfinite(pr_vel(1)))
  {
    RCLCPP_ERROR(rclcpp::get_logger("transmission_math"),
                 "[TransmissionMath] Failed to calculate theta_p_dot_model and theta_r_dot_model, theta_u_dot_: %f, "
                 "theta_l_dot_: %f",
                 theta_u_dot_, theta_l_dot_);
    return false;
  }

  theta_p_dot_model_ = pr_vel(0);
  theta_r_dot_model_ = pr_vel(1);

  return true;
}

bool TransmissionMapping::pd_control_torque() {
  Eigen::Vector2d cmd_pos_error(theta_p_cmd_model_ - theta_p_model_, theta_r_cmd_model_ - theta_r_model_);
  Eigen::Vector2d ul_dot(theta_u_dot_, theta_l_dot_);

  Eigen::Vector2d tau_cmd = -J_ul_trans_ * J_pr_inv_trans_ * (Kp * cmd_pos_error + Kd * J_pr_inv_ * J_ul_ * ul_dot);

  if (!std::isfinite(tau_cmd(0)) || !std::isfinite(tau_cmd(1)))
  {
    RCLCPP_ERROR(rclcpp::get_logger("transmission_math"),
                 "[TransmissionMath] Failed to calculate tau_u_cmd and tau_l_cmd, theta_p_cmd_model_: %f, "
                 "theta_r_cmd_model_: %f, theta_p_model_: %f, theta_r_model_: %f, theta_u_dot_: %f, theta_l_dot_: %f",
                 theta_p_cmd_model_, theta_r_cmd_model_, theta_p_model_, theta_r_model_, theta_u_dot_, theta_l_dot_);
    return false;
  }

  tau_u_cmd_ = tau_cmd(0);
  tau_l_cmd_ = tau_cmd(1);
  return true;
}
