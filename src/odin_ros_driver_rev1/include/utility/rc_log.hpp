/**
 * @file rc_log.hpp
 * @brief RC_LOG_* logging macros for ROS1/ROS2, without the full ROS API.
 *
 * ros_compat.hpp provides the version-agnostic Node/Publisher/Subscriber
 * wrappers and therefore has to include the complete ROS API (ros/ros.h or
 * rclcpp/rclcpp.hpp plus the sensor_msgs message headers). Translation units
 * that only emit log lines - the decoder backends and their factories - do not
 * need any of that, so they include this header instead: it pulls in the
 * logging headers alone.
 *
 * The macro names and their expansion are identical to what ros_compat.hpp
 * used to define; ros_compat.hpp now includes this file, so there is a single
 * definition site and switching a file between the two headers never changes
 * logging behaviour.
 */

#pragma once

// ROS2 is selected by the ODIN_ROS2 definition added by CMake. The two extra
// conditions keep the header usable when it is reached through ros_compat.hpp,
// whose version detection relies on a per-target ROS_VERSION_MAJOR definition
// or on rclcpp having been included first.
#if defined(ODIN_ROS2) || (defined(ROS_VERSION_MAJOR) && ROS_VERSION_MAJOR == 2) || \
    defined(RCLCPP__RCLCPP_HPP_)

#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>

#define RC_LOG_DEBUG(...) RCLCPP_DEBUG(rclcpp::get_logger("post_process"), __VA_ARGS__)
#define RC_LOG_INFO(...) RCLCPP_INFO(rclcpp::get_logger("post_process"), __VA_ARGS__)
#define RC_LOG_WARN(...) RCLCPP_WARN(rclcpp::get_logger("post_process"), __VA_ARGS__)
#define RC_LOG_ERROR(...) RCLCPP_ERROR(rclcpp::get_logger("post_process"), __VA_ARGS__)
#define RC_LOG_FATAL(...) RCLCPP_FATAL(rclcpp::get_logger("post_process"), __VA_ARGS__)
#define RC_LOG_INFO_STREAM(x) RCLCPP_INFO_STREAM(rclcpp::get_logger("post_process"), x)
#define RC_LOG_ERROR_STREAM(x) RCLCPP_ERROR_STREAM(rclcpp::get_logger("post_process"), x)

#else

#include <ros/console.h>

#define RC_LOG_DEBUG(...) ROS_DEBUG(__VA_ARGS__)
#define RC_LOG_INFO(...) ROS_INFO(__VA_ARGS__)
#define RC_LOG_WARN(...) ROS_WARN(__VA_ARGS__)
#define RC_LOG_ERROR(...) ROS_ERROR(__VA_ARGS__)
#define RC_LOG_FATAL(...) ROS_FATAL(__VA_ARGS__)
#define RC_LOG_INFO_STREAM(x) ROS_INFO_STREAM(x)
#define RC_LOG_ERROR_STREAM(x) ROS_ERROR_STREAM(x)

#endif
