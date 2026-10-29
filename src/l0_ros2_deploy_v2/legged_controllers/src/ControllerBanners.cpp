#include "legged_controllers/ControllerBanners.h"

#include <cstddef>
#include <rclcpp/rclcpp.hpp>
#include <string>

namespace legged {
namespace {

constexpr std::size_t kBoxInnerWidth = 42;
constexpr const char* kBoxBorder = "+------------------------------------------+";

void logBlank(rclcpp::Logger logger) { RCLCPP_INFO(logger, " "); }

void logBorder(rclcpp::Logger logger) { RCLCPP_INFO(logger, "%s", kBoxBorder); }

void logBoxLine(rclcpp::Logger logger, const std::string& text = std::string(), bool centered = true) {
  std::string content = text;
  if (content.size() > kBoxInnerWidth)
  {
    content.resize(kBoxInnerWidth);
  }

  std::size_t left = 0;
  if (centered)
  {
    left = (kBoxInnerWidth - content.size()) / 2;
  }
  else
  {
    left = content.empty() ? (kBoxInnerWidth / 2) : 2;
    if (left + content.size() > kBoxInnerWidth) left = 0;
  }
  const std::size_t right = kBoxInnerWidth - content.size() - left;
  const std::string left_pad(left, ' ');
  const std::string right_pad(right, ' ');
  RCLCPP_INFO(logger, "|%s%s%s|", left_pad.c_str(), content.c_str(), right_pad.c_str());
}

}  // namespace

void printCalibrationBanner(rclcpp::Logger logger, const std::string& banner) {
  logBlank(logger);
  logBorder(logger);
  logBoxLine(logger);
  logBoxLine(logger, banner);
  logBoxLine(logger);
  logBorder(logger);
  logBlank(logger);
}

void printEmergencyStopBanner(rclcpp::Logger logger) {
  logBlank(logger);
  logBorder(logger);
  logBoxLine(logger);
  logBoxLine(logger, "EMERGENCY STOP ACTIVATED");
  logBoxLine(logger);
  logBorder(logger);
  logBlank(logger);
}

void printWalkModeBanner(rclcpp::Logger logger) {
  logBlank(logger);
  logBorder(logger);
  logBoxLine(logger);
  logBoxLine(logger, "WALK MODE ACTIVATED");
  logBoxLine(logger);
  logBorder(logger);
  logBlank(logger);
}

void printCalibrationEnabledBanner(rclcpp::Logger logger) {
  logBlank(logger);
  logBorder(logger);
  logBoxLine(logger);
  logBoxLine(logger, "CALIBRATION MODE ENABLED");
  logBoxLine(logger);
  logBoxLine(logger, "Use controller buttons to calibrate:", false);
  logBoxLine(logger, "- LB+LT: Left Arm", false);
  logBoxLine(logger, "- RB+RT: Right Arm", false);
  logBoxLine(logger, "- A+X: Left Leg", false);
  logBoxLine(logger, "- B+Y: Right Leg", false);
  logBoxLine(logger, "- BACK: Waist (if configured)", false);
  logBoxLine(logger, "- START: Neck", false);
  logBoxLine(logger, "Press LS+RS again to exit calibration", false);
  logBoxLine(logger);
  logBorder(logger);
  logBlank(logger);
}

void printCalibrationDisabledBanner(rclcpp::Logger logger) {
  logBlank(logger);
  logBorder(logger);
  logBoxLine(logger);
  logBoxLine(logger, "CALIBRATION MODE DISABLED");
  logBoxLine(logger);
  logBoxLine(logger, "Returning to Emergency Stop mode");
  logBoxLine(logger);
  logBorder(logger);
  logBlank(logger);
}

void logResetBanner(rclcpp::Logger logger) {
  RCLCPP_INFO(logger, " ");
  logBorder(logger);
  logBoxLine(logger);
  logBoxLine(logger, "CONTROLLER RESET");
  logBoxLine(logger);
  logBoxLine(logger, "Emergency cleared - System ready");
  logBoxLine(logger, "Policy state reset for walk");
  logBoxLine(logger);
  logBorder(logger);
  RCLCPP_INFO(logger, " ");
}

}  // namespace legged
