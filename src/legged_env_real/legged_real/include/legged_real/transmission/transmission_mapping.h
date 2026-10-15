#pragma once

#include <Eigen/Dense>
#include <cmath>

class TransmissionMapping {
 public:
  TransmissionMapping(double direction_p, double direction_r, double offset_p, double offset_r);

  double theta_u_ = 0.0;
  double theta_l_ = 0.0;
  double theta_u_dot_ = 0.0;
  double theta_l_dot_ = 0.0;

  double theta_u_cmd_ = 0.0;
  double theta_l_cmd_ = 0.0;
  double tau_u_cmd_ = 0.0;
  double tau_l_cmd_ = 0.0;

  double theta_p_urdf_ = 0.0;
  double theta_r_urdf_ = 0.0;
  double theta_p_dot_urdf_ = 0.0;
  double theta_r_dot_urdf_ = 0.0;

  double theta_p_cmd_urdf_ = 0.0;
  double theta_r_cmd_urdf_ = 0.0;

  Eigen::DiagonalMatrix<double, 2> Kp;
  Eigen::DiagonalMatrix<double, 2> Kd;

  [[nodiscard]] bool calc_position_cmd(double theta_p_cmd_urdf, double theta_r_cmd_urdf);
  Eigen::Vector2d get_position_cmd();  // theta_u_cmd_, theta_l_cmd_
  [[nodiscard]] bool calc_torque_cmd(double theta_p_cmd_urdf, double theta_r_cmd_urdf, double kp_p, double kp_r,
                                     double kd_p, double kd_r);
  Eigen::Vector2d get_torque_cmd();  // tau_u_cmd_, tau_l_cmd_

  [[nodiscard]] bool calc_state(double theta_u, double theta_l, double theta_u_dot, double theta_l_dot);
  Eigen::Vector4d get_state();  // theta_p_urdf_, theta_r_urdf_, theta_p_dot_urdf_, theta_r_dot_urdf_

 private:
  // Link length physical constants
  static constexpr double l_OA = 0.018;
  // static constexpr double l_AB = 0.09865;  //y1_v2
  // static constexpr double l_BC = 0.060;  //y1_v2
  static constexpr double l_AB = 0.10865;  // y1_v3
  static constexpr double l_BC = 0.060;    // y1_v3
  static constexpr double l_OD = 0.022;
  static constexpr double l_DE = 0.013;
  static constexpr double l_EF = 0.011;
  static constexpr double l_BI = 0.030;
  static constexpr double l_IJ = 0.0165;
  // static constexpr double l_FJ = 0.0986; //y1_v2
  // static constexpr double l_HL = 0.1586; //y1_v2
  static constexpr double l_FJ = 0.1086;  // y1_v3
  static constexpr double l_HL = 0.1686;  // y1_v3

  // Last iteration results for Newton's method
  double last_theta_p_ = 0.0;
  double last_theta_r_ = 0.0;

  double direction_p_;
  double direction_r_;
  double offset_p_;
  double offset_r_;

  double theta_p_model_ = 0.0;
  double theta_r_model_ = 0.0;
  double theta_p_dot_model_ = 0.0;
  double theta_r_dot_model_ = 0.0;

  double theta_p_cmd_model_ = 0.0;
  double theta_r_cmd_model_ = 0.0;

  Eigen::Matrix2d J_pr_ = Eigen::Matrix2d::Zero();
  Eigen::Matrix2d J_ul_ = Eigen::Matrix2d::Zero();
  Eigen::Matrix2d J_pr_inv_ = Eigen::Matrix2d::Zero();
  Eigen::Matrix2d J_ul_trans_ = Eigen::Matrix2d::Zero();
  Eigen::Matrix2d J_pr_inv_trans_ = Eigen::Matrix2d::Zero();

  // urdf -> model
  bool urdf2model();

  // model -> urdf
  bool model2urdf();

  // theta_p, theta_r -> theta_u, theta_l
  bool inverse_transmission_mapping();

  // theta_u, theta_l -> theta_p, theta_r
  bool transmission_mapping();

  // jacobian matrix
  bool jacobian_update();

  // theta_u_dot, theta_l_dot -> theta_p_dot, theta_r_dot
  bool transmission_velocity_mapping();

  // Compute motor torques tau_u_cmd, tau_l_cmd from PD control law in task space
  bool pd_control_torque();

  // MATLAB Coder generated functions
  double theta_u_calc(double theta_p, double theta_r);
  double theta_l_calc(double theta_p, double theta_r);
  double eq1_calc(double theta_p, double theta_r, double theta_u, double theta_l);
  double eq2_calc(double theta_p, double theta_r, double theta_u, double theta_l);
  void J_pr_calc(double theta_p, double theta_r, double theta_u, double theta_l, double J_pr[4]);
  void J_ul_calc(double theta_p, double theta_r, double theta_u, double theta_l, double J_ul[4]);
};
