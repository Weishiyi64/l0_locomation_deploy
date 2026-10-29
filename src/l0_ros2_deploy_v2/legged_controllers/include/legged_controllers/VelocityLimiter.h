#pragma once
#include <geometry_msgs/msg/twist.h>
#include <std_msgs/msg/float32.h>

#include <Eigen/Core>
// #include <ros/ros.h>
#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <cmath>
#include <rclcpp/rclcpp.hpp>
#include <string>

namespace legged {
class VelocityLimiter {
 public:
  // 分段映射配置结构
  struct SegmentMapping {
    double threshold{1.0};
    double first_segment_min{0.0};
    double first_segment_max{1.0};
    double second_segment_min{1.0};
    double second_segment_max{1.0};
  };

  // 轴映射配置结构
  struct AxisMapping {
    SegmentMapping positive;
    SegmentMapping negative;
  };

  // 平滑处理参数
  struct SmoothParams {
    double rate_linear_x_positive{0.05};
    double rate_linear_x_negative{0.05};
    double rate_linear_y_positive{0.05};
    double rate_linear_y_negative{0.05};
    double rate_angular_z_positive{0.1};
    double rate_angular_z_negative{0.1};
  };

  // 完整配置结构
  struct JoyConfig {
    AxisMapping linear_x;
    AxisMapping linear_y;
    AxisMapping angular_z;
    double lt_multiplier_max{1.0};
    bool enable_smooth_transition{false};
    SmoothParams smooth_params;
  };

  VelocityLimiter(rclcpp::Node::SharedPtr nh) : nh_(nh) {
    last_ << 0, 0, 0;
    speed_multiplier_ = 1.0;
    lt_initialized_ = false;
    loadJoyConfig(nh);
  }

  void setSpeedMultiplier(double multiplier) { speed_multiplier_ = multiplier; }

  void updateSpeedMultiplierFromAxis(double lt_axis_value) {
    lt_axis_value = clampNormalizedInput(lt_axis_value);

    if (std::abs(lt_axis_value) > 0.1)
    {
      lt_initialized_ = true;
    }

    if (lt_initialized_)
    {
      // LT轴原始值范围：未按下=1.0，按到底=-1.0
      // 将 [1.0, -1.0] 映射到 [0.0, 1.0]
      double normalized_input = (1.0 - lt_axis_value) / 2.0;

      // 线性映射：[0, 1] → [1.0, lt_multiplier_max]
      // 未按下时1.0倍速，按到底时lt_multiplier_max倍速
      speed_multiplier_ = 1.0 + normalized_input * (joy_config_.lt_multiplier_max - 1.0);
      speed_multiplier_ = std::max(1.0, speed_multiplier_);
    }
  }

  geometry_msgs::msg::Twist filter(const geometry_msgs::msg::Twist& in) {
    geometry_msgs::msg::Twist out = in;

    const double input_vx = clampNormalizedInput(in.linear.x);
    const double input_vy = clampNormalizedInput(in.linear.y);
    const double input_wz = clampNormalizedInput(in.angular.z);

    // 应用分段映射
    double mapped_vx = applyAxisMapping(input_vx, joy_config_.linear_x);
    double mapped_vy = applyAxisMapping(input_vy, joy_config_.linear_y);
    double mapped_wz = applyAxisMapping(input_wz, joy_config_.angular_z);

    // LT加速倍数
    double current_multiplier = speed_multiplier_;
    mapped_vx *= current_multiplier;
    mapped_vx = clampToConfiguredAxisLimit(mapped_vx, joy_config_.linear_x, input_vx);

    // 应用平滑处理
    if (joy_config_.enable_smooth_transition)
    {
      smoothAxis(0, mapped_vx, joy_config_.smooth_params.rate_linear_x_positive,
                 joy_config_.smooth_params.rate_linear_x_negative);
      smoothAxis(1, mapped_vy, joy_config_.smooth_params.rate_linear_y_positive,
                 joy_config_.smooth_params.rate_linear_y_negative);
      smoothAxis(2, mapped_wz, joy_config_.smooth_params.rate_angular_z_positive,
                 joy_config_.smooth_params.rate_angular_z_negative);

      out.linear.x = last_[0];
      out.linear.y = last_[1];
      out.angular.z = last_[2];
    }
    else
    {
      out.linear.x = mapped_vx;
      out.linear.y = mapped_vy;
      out.angular.z = mapped_wz;
    }

    return out;
  }

 private:
  rclcpp::Node::SharedPtr nh_;

  static double clampValue(double value, double min_value, double max_value) {
    if (value < min_value) return min_value;
    if (value > max_value) return max_value;
    return value;
  }

  static double clampNormalizedInput(double input) { return clampValue(input, -1.0, 1.0); }

  static double effectiveThreshold(double threshold) { return clampValue(threshold, 0.0, 1.0); }

  static double configuredSegmentLimit(const SegmentMapping& mapping) {
    const double threshold = effectiveThreshold(mapping.threshold);
    double limit = std::max(std::abs(mapping.first_segment_min), std::abs(mapping.first_segment_max));
    if (threshold < 1.0 - 1e-6)
    {
      limit = std::max(limit, std::abs(mapping.second_segment_min));
      limit = std::max(limit, std::abs(mapping.second_segment_max));
    }
    return limit;
  }

  static double configuredAxisLimit(const AxisMapping& axis_mapping, double input) {
    if (input > 0.0)
    {
      return configuredSegmentLimit(axis_mapping.positive);
    }
    if (input < 0.0)
    {
      return configuredSegmentLimit(axis_mapping.negative);
    }
    return std::max(configuredSegmentLimit(axis_mapping.positive), configuredSegmentLimit(axis_mapping.negative));
  }

  static double clampToConfiguredAxisLimit(double value, const AxisMapping& axis_mapping, double input) {
    const double limit = configuredAxisLimit(axis_mapping, input);
    return clampValue(value, -limit, limit);
  }

  double applyAxisMapping(double input, const AxisMapping& axis_mapping) {
    input = clampNormalizedInput(input);
    if (input >= 0)
    {
      return applySegmentMapping(input, axis_mapping.positive);
    }
    else
    {
      double abs_input = std::abs(input);
      double mapped_abs = applySegmentMapping(abs_input, axis_mapping.negative);
      return -mapped_abs;
    }
  }

  double applySegmentMapping(double input, const SegmentMapping& mapping) {
    input = clampValue(input, 0.0, 1.0);
    const double threshold = effectiveThreshold(mapping.threshold);
    if (input <= threshold)
    {
      // 第一段：[0, threshold] -> [first_segment_min, first_segment_max]
      if (threshold <= 1e-6)
      {
        return mapping.first_segment_min;
      }
      double ratio = input / threshold;
      return mapping.first_segment_min + ratio * (mapping.first_segment_max - mapping.first_segment_min);
    }
    else
    {
      // 第二段：[threshold, 1] -> [second_segment_min, second_segment_max]
      double remaining = 1.0 - threshold;
      if (remaining <= 1e-6)
      {
        return mapping.second_segment_max;
      }
      double ratio = (input - threshold) / remaining;
      return mapping.second_segment_min + ratio * (mapping.second_segment_max - mapping.second_segment_min);
    }
  }

  void loadJoyConfig(rclcpp::Node::SharedPtr nh) {
    try
    {
      // ros::NodeHandle nh;
      // ("Loading joy config from parameter server");
      RCLCPP_INFO(nh_->get_logger(), "Loading joy config from parameter server");
      std::string joy_cfg_path("");
      try
      { nh->declare_parameter<std::string>("joy_cfg_path", ""); } catch (...)
      {}
      if (!nh->get_parameter("joy_cfg_path", joy_cfg_path) || joy_cfg_path.empty())
      {
        throw std::runtime_error("Required parameter 'joy_cfg_path' not found");
      }
      YAML::Node root = YAML::LoadFile(joy_cfg_path);
      YAML::Node jc = root["joystick_mapping"];

      // 解析joystick_mapping
      // loadAxisMappingFromParam(nh, "/joystick_mapping/linear_x",
      // joy_config_.linear_x); loadAxisMappingFromParam(nh,
      // "/joystick_mapping/linear_y", joy_config_.linear_y);
      // loadAxisMappingFromParam(nh, "/joystick_mapping/angular_z",
      // joy_config_.angular_z);

      loadAxisMappingFromParam(nh, jc["linear_x"], joy_config_.linear_x);
      loadAxisMappingFromParam(nh, jc["linear_y"], joy_config_.linear_y);
      loadAxisMappingFromParam(nh, jc["angular_z"], joy_config_.angular_z);

      // 解析LT加速倍数
      /*
      if (!nh.getParam("/joystick_mapping/lt_multiplier_max",
      joy_config_.lt_multiplier_max)) { ("Failed to get
      lt_multiplier_max, using default: 1.0"); joy_config_.lt_multiplier_max
      = 1.0;
      }
      */

      if (!jc["lt_multiplier_max"])
      {
        RCLCPP_WARN(nh->get_logger(), "Failed to get lt_multiplier_max, using default: 1.0");
        joy_config_.lt_multiplier_max = 1.0;
      }
      else
      {
        joy_config_.lt_multiplier_max = jc["lt_multiplier_max"].as<double>(0.0);
      }

      // 解析平滑处理配置
      /*
      if (!nh.getParam("/enable_smooth_transition",
      joy_config_.enable_smooth_transition)) { ("Failed to get
      enable_smooth_transition, using default: false");
        joy_config_.enable_smooth_transition = false;
      }
        */
      // YAML::Node enable_smooth_transition_node =
      // root["enable_smooth_transition"];
      if (!root["enable_smooth_transition"])
      {
        RCLCPP_WARN(nh->get_logger(), "Failed to get enable_smooth_transition, using default: false");
        joy_config_.enable_smooth_transition = false;
      }
      else
      {
        joy_config_.enable_smooth_transition = root["enable_smooth_transition"].as<bool>(false);
      }

      // 解析平滑参数
      /*
      if (!nh.getParam("/smooth_params/rate_linear_x",
      joy_config_.smooth_params.rate_linear_x)) { ("Failed to get
      rate_linear_x, using default: 0.05");
          joy_config_.smooth_params.rate_linear_x = 0.05;
      }
      if (!nh.getParam("/smooth_params/rate_linear_y",
      joy_config_.smooth_params.rate_linear_y)) { ("Failed to get
      rate_linear_y, using default: 0.05");
          joy_config_.smooth_params.rate_linear_y = 0.05;
      }
      if (!nh.getParam("/smooth_params/rate_angular_z",
      joy_config_.smooth_params.rate_angular_z)) { ("Failed to get
      rate_angular_z, using default: 0.1");
          joy_config_.smooth_params.rate_angular_z = 0.1;
      }
      */
      if (!root["smooth_params"]["rate_linear_x_positive"])
      {
        RCLCPP_WARN(nh->get_logger(), "Failed to get rate_linear_x_positive, using default: 0.05");
        joy_config_.smooth_params.rate_linear_x_positive = 0.05;
      }
      else
      {
        joy_config_.smooth_params.rate_linear_x_positive =
            root["smooth_params"]["rate_linear_x_positive"].as<double>(0.0);
      }

      if (!root["smooth_params"]["rate_linear_x_negative"])
      {
        RCLCPP_WARN(nh->get_logger(), "Failed to get rate_linear_x_negative, using default: 0.05");
        joy_config_.smooth_params.rate_linear_x_negative = 0.05;
      }
      else
      {
        joy_config_.smooth_params.rate_linear_x_negative =
            root["smooth_params"]["rate_linear_x_negative"].as<double>(0.0);
      }

      if (!root["smooth_params"]["rate_linear_y_positive"])
      {
        RCLCPP_WARN(nh->get_logger(), "Failed to get rate_linear_y, using default: 0.05");
        joy_config_.smooth_params.rate_linear_y_positive = 0.05;
      }
      else
      {
        joy_config_.smooth_params.rate_linear_y_positive =
            root["smooth_params"]["rate_linear_y_positive"].as<double>(0.0);
      }

      if (!root["smooth_params"]["rate_linear_y_negative"])
      {
        RCLCPP_WARN(nh->get_logger(), "Failed to get rate_linear_y_negative, using default: 0.05");
        joy_config_.smooth_params.rate_linear_y_negative = 0.05;
      }
      else
      {
        joy_config_.smooth_params.rate_linear_y_negative =
            root["smooth_params"]["rate_linear_y_negative"].as<double>(0.0);
      }

      if (!root["smooth_params"]["rate_angular_z_positive"])
      {
        RCLCPP_WARN(nh->get_logger(), "Failed to get rate_angular_z, using default: 0.1");
        joy_config_.smooth_params.rate_angular_z_positive = 0.1;
      }
      else
      {
        joy_config_.smooth_params.rate_angular_z_positive =
            root["smooth_params"]["rate_angular_z_positive"].as<double>(0.0);
      }

      if (!root["smooth_params"]["rate_angular_z_negative"])
      {
        RCLCPP_WARN(nh->get_logger(), "Failed to get rate_angular_z_negative, using default: 0.1");
        joy_config_.smooth_params.rate_angular_z_negative = 0.1;
      }
      else
      {
        joy_config_.smooth_params.rate_angular_z_negative =
            root["smooth_params"]["rate_angular_z_negative"].as<double>(0.0);
      }

      RCLCPP_INFO(nh->get_logger(), "Joy config loaded successfully from parameter server");
    } catch (const std::exception& e)
    {
      // ("Failed to load joy config: %s. Using default values.",
      // e.what());
      RCLCPP_WARN(nh->get_logger(), "Failed to load joy config: %s. Using default values.", e.what());
      setDefaultConfig();
    }
  }

  // 从参数服务器加载轴映射配置
  // void loadAxisMappingFromParam(ros::NodeHandle& nh, const std::string&
  // base_param, AxisMapping& axis_mapping) {
  void loadAxisMappingFromParam(rclcpp::Node::SharedPtr nh, const YAML::Node& base_node, AxisMapping& axis_mapping) {
    // 加载正向映射
    // loadSegmentMappingFromParam(nh, base_param + "/positive",
    // axis_mapping.positive);
    loadSegmentMappingFromParam(nh, base_node["positive"], axis_mapping.positive);
    // 加载负向映射
    // loadSegmentMappingFromParam(nh, base_param + "/negative",
    // axis_mapping.negative);
    loadSegmentMappingFromParam(nh, base_node["negative"], axis_mapping.negative);
  }

  // 从参数服务器加载分段映射配置
  // void loadSegmentMappingFromParam(ros::NodeHandle& nh, const std::string&
  // base_param, SegmentMapping& mapping) { void
  // loadSegmentMappingFromParam(rclcpp::Node::SharedPtr nh, const std::string&
  // base_param, SegmentMapping& mapping) {
  void loadSegmentMappingFromParam(rclcpp::Node::SharedPtr nh, const YAML::Node& base_node, SegmentMapping& mapping) {
    if (!base_node["threshold"])
    {
      RCLCPP_WARN(nh->get_logger(), "Failed to get threshold, using default: 1.0");
      mapping.threshold = 1.0;
    }
    else
    {
      mapping.threshold = base_node["threshold"].as<double>(0.0);
    }

    if (!base_node["first_segment_min"])
    {
      RCLCPP_WARN(nh->get_logger(), "Failed to get first_segment_min, using default: 0.0");
      mapping.first_segment_min = 0.0;
    }
    else
    {
      mapping.first_segment_min = base_node["first_segment_min"].as<double>(0.0);
    }

    if (!base_node["first_segment_max"])
    {
      RCLCPP_WARN(nh->get_logger(), "Failed to get first_segment_max, using default: 1.0");
      mapping.first_segment_max = 1.0;
    }
    else
    {
      mapping.first_segment_max = base_node["first_segment_max"].as<double>(0.0);
    }

    if (!base_node["second_segment_min"])
    {
      RCLCPP_WARN(nh->get_logger(), "Failed to get second_segment_min, using default: 1.0");
      mapping.second_segment_min = 1.0;
    }
    else
    {
      mapping.second_segment_min = base_node["second_segment_min"].as<double>(0.0);
    }

    if (!base_node["second_segment_max"])
    {
      RCLCPP_WARN(nh->get_logger(), "Failed to get second_segment_max, using default: 1.0");
      mapping.second_segment_max = 1.0;
    }
    else
    {
      mapping.second_segment_max = base_node["second_segment_max"].as<double>(0.0);
    }
  }

  // 设置默认配置
  void setDefaultConfig() {
    // 使用默认参数值
    joy_config_.enable_smooth_transition = false;
    joy_config_.smooth_params.rate_linear_x_positive = 0.05;
    joy_config_.smooth_params.rate_linear_x_negative = 0.05;
    joy_config_.smooth_params.rate_linear_y_positive = 0.05;
    joy_config_.smooth_params.rate_linear_y_negative = 0.05;
    joy_config_.smooth_params.rate_angular_z_positive = 0.1;
    joy_config_.smooth_params.rate_angular_z_negative = 0.1;

    // 设置默认的线性映射（threshold=1.0）
    auto setLinearMapping = [](SegmentMapping& mapping, double max_val) {
      mapping.threshold = 1.0;
      mapping.first_segment_min = 0.0;
      mapping.first_segment_max = max_val;
      mapping.second_segment_min = max_val;
      mapping.second_segment_max = max_val;
    };

    setLinearMapping(joy_config_.linear_x.positive, 1.0);
    setLinearMapping(joy_config_.linear_x.negative, 1.0);
    setLinearMapping(joy_config_.linear_y.positive, 1.0);
    setLinearMapping(joy_config_.linear_y.negative, 1.0);
    setLinearMapping(joy_config_.angular_z.positive, 1.0);
    setLinearMapping(joy_config_.angular_z.negative, 1.0);
  }

  void smoothAxis(int idx, double target, double positive_limit, double negative_limit) {
    double limit = (last_[idx] >= 0) ? positive_limit : negative_limit;
    double delta = target - last_[idx];
    if (delta > limit)
      delta = limit;
    else if (delta < -limit)
      delta = -limit;
    last_[idx] += delta;
  }

  JoyConfig joy_config_;
  Eigen::Vector3d last_;
  double speed_multiplier_;
  bool lt_initialized_;
};
}  // namespace legged
