#pragma once

#include <rclcpp/logger.hpp>
#include <string>

namespace legged {

void printCalibrationBanner(rclcpp::Logger logger, const std::string& banner);
void printEmergencyStopBanner(rclcpp::Logger logger);
void printWalkModeBanner(rclcpp::Logger logger);
void printCalibrationEnabledBanner(rclcpp::Logger logger);
void printCalibrationDisabledBanner(rclcpp::Logger logger);
void logResetBanner(rclcpp::Logger logger);

}  // namespace legged
