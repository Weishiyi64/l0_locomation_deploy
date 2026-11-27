#pragma once
#include <sensor_msgs/msg/joint_state.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
// #include <ros/ros.h>
#include <iomanip>
#include <rclcpp/rclcpp.hpp>
#include <sstream>
#include <string>
#include <vector>

namespace legged {
class VisualizationPublisher {
 public:
  struct Params {
    std::string frame{"base_link"};
    double text_size{0.04};
    double y_stride{0.05};
    double z_base{0.5};
    double z_span{0.3};
    double lifetime{0.25};
  };

  VisualizationPublisher(rclcpp::Node::SharedPtr nh, const Params& p, bool enable_markers) : p_(p) {
    js_pub_ = nh->create_publisher<sensor_msgs::msg::JointState>("/system/joint/jointstates", rclcpp::QoS(10));
    if (enable_markers)
      marker_pub_ = nh->create_publisher<visualization_msgs::msg::MarkerArray>("/joint_rad_text", rclcpp::QoS(10));
  }

  // void publish(const sensor_msgs::JointState& js) {
  void publish(const sensor_msgs::msg::JointState& js) {
    js_pub_->publish(js);
    if (!marker_pub_) return;

    visualization_msgs::msg::MarkerArray arr;
    arr.markers.reserve(js.name.size());
    for (size_t i = 0; i < js.name.size(); ++i)
    {
      visualization_msgs::msg::Marker m;
      // m.header.stamp = ros::Time::now();
      m.header.stamp = rclcpp::Clock().now();
      m.header.frame_id = p_.frame;
      m.ns = "joint_rads";
      m.id = static_cast<int>(i);
      m.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
      m.action = visualization_msgs::msg::Marker::ADD;
      m.pose.position.x = 0.0;
      m.pose.position.y = static_cast<double>(i) * p_.y_stride;
      m.pose.position.z = p_.z_base;  //+ fmod(i * 0.02, p_.z_span);
      m.pose.orientation.w = 1.0;
      m.scale.z = p_.text_size;
      m.color.r = 1.0f;
      m.color.g = 1.0f;
      m.color.b = 1.0f;
      m.color.a = 1.0f;
      std::ostringstream ss;
      ss.setf(std::ios::fixed);
      ss << std::setprecision(3) << (i < js.position.size() ? js.position[i] : 0.0);
      m.text = js.name[i] + ": " + ss.str() + " rad";
      // m.lifetime = ros::Duration(p_.lifetime);
      m.lifetime = rclcpp::Duration::from_seconds(p_.lifetime);
      arr.markers.push_back(m);
    }
    marker_pub_->publish(arr);
  }

 private:
  Params p_;
  // ros::Publisher js_pub_;
  // ros::Publisher marker_pub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr js_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
};
}  // namespace legged
