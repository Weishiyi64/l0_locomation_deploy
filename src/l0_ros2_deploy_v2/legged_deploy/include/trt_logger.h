#ifndef TRT_LOGGER_H
#define TRT_LOGGER_H

#include <NvInfer.h>

#include <rclcpp/rclcpp.hpp>

// TensorRT logger that forwards messages to the ROS 2 logger.
// Shared by all controllers that use TRT inference.
struct TRTLogger : public nvinfer1::ILogger {
  rclcpp::Logger ros_logger = rclcpp::get_logger("TRTLogger");
  void log(Severity severity, const char* msg) noexcept override {
    switch (severity)
    {
      case Severity::kINTERNAL_ERROR:
      case Severity::kERROR:
        RCLCPP_ERROR(ros_logger, "[TRT] %s", msg);
        break;
      case Severity::kWARNING:
      default:
        break;
    }
  }
};

#endif  // TRT_LOGGER_H
