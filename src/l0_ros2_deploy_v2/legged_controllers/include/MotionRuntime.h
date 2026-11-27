#pragma once

#include "MotionData.h"
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <robot_interfaces/action/request_motion.hpp>
#include <robot_interfaces/srv/motion_transaction.hpp>

namespace legged {
namespace motion {
using MotionAction = robot_interfaces::action::RequestMotion;
using Transaction = robot_interfaces::srv::MotionTransaction;
using Goal = MotionAction::Goal;
Json textRequest(const Goal& goal);
std::shared_ptr<rclcpp::Node> makeMotionBackend();
std::shared_ptr<rclcpp::Node> makeMotionFrontend(const std::string& role);
}  // namespace motion
}  // namespace legged
