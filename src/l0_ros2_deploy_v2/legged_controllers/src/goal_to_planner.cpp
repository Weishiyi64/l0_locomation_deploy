#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <functional>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <memory>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/string.hpp>
#include <string>
#include <visualization_msgs/msg/marker_array.hpp>

#include "FollowGoalTracker.h"

namespace {

constexpr double kOdomTimeoutSeconds = 0.5;
constexpr const char* kOdomTopic = "/manifold/ODIN2/device0/odometry";
constexpr const char* kOdomFrame = "device0/odom";
constexpr const char* kBaseFrame = "device0/base_link";
constexpr const char* kNavigationMarkerTopic = "/navigation_visualization";

class GoalToPlanner final : public rclcpp::Node {
 public:
  GoalToPlanner() : Node("goal_to_planner") {
    input_pub_ = create_publisher<std_msgs::msg::Float32MultiArray>("/planner_input", rclcpp::QoS(1));
    status_pub_ = create_publisher<std_msgs::msg::String>("/goal_to_planner/state", rclcpp::QoS(10));
    marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(kNavigationMarkerTopic,
                                                                         rclcpp::QoS(1).reliable().transient_local());
    goal_sub_ = create_subscription<std_msgs::msg::Float32MultiArray>(
        "/navigation_goal", rclcpp::QoS(1), std::bind(&GoalToPlanner::goalCallback, this, std::placeholders::_1));
    pose_goal_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
        "/goal_pose", rclcpp::QoS(1), std::bind(&GoalToPlanner::poseGoalCallback, this, std::placeholders::_1));
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        kOdomTopic, rclcpp::SensorDataQoS().keep_last(1),
        std::bind(&GoalToPlanner::odomCallback, this, std::placeholders::_1));
    timer_ = create_wall_timer(std::chrono::milliseconds(200), std::bind(&GoalToPlanner::tick, this));
    RCLCPP_INFO(get_logger(),
                "Absolute odom-goal bridge ready: immediate goal response, 5 Hz input/output, "
                "1.0 m three-point lookahead, "
                "default speeds 0.35 m/s and 1.57 rad/s, 0.5 s goal/odometry timeout; "
                "RViz markers on %s",
                kNavigationMarkerTopic);
  }

  void stop() {
    pose_goal_active_ = false;
    tracker_.reset();
    goal_marker_visible_ = false;
    publishNavigationMarkers();
    publish(legged_controllers::FollowGoalTracker::Output{});
  }

 private:
  static double steadySeconds() {
    return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
  }

  void publishNavigationMarkers() {
    visualization_msgs::msg::MarkerArray markers;
    const auto stamp = now();

    if (odom_received_)
    {
      visualization_msgs::msg::Marker robot;
      robot.header.frame_id = kOdomFrame;
      robot.header.stamp = stamp;
      robot.ns = "robot_pose";
      robot.id = 0;
      robot.type = visualization_msgs::msg::Marker::ARROW;
      robot.action = visualization_msgs::msg::Marker::ADD;
      robot.pose.position.x = pose_.x;
      robot.pose.position.y = pose_.y;
      robot.pose.position.z = 0.15;
      robot.pose.orientation.z = std::sin(0.5 * pose_.yaw);
      robot.pose.orientation.w = std::cos(0.5 * pose_.yaw);
      robot.scale.x = 0.5;
      robot.scale.y = 0.12;
      robot.scale.z = 0.12;
      robot.color.r = 0.1f;
      robot.color.g = 0.4f;
      robot.color.b = 1.0f;
      robot.color.a = 1.0f;
      markers.markers.push_back(robot);
    }

    visualization_msgs::msg::Marker goal;
    goal.header.frame_id = kOdomFrame;
    goal.header.stamp = stamp;
    goal.ns = "navigation_goal";
    goal.id = 0;
    goal.action = goal_marker_visible_ ? visualization_msgs::msg::Marker::ADD : visualization_msgs::msg::Marker::DELETE;
    if (goal_marker_visible_)
    {
      goal.type = visualization_msgs::msg::Marker::SPHERE;
      goal.pose.position.x = goal_marker_x_;
      goal.pose.position.y = goal_marker_y_;
      goal.pose.position.z = 0.15;
      goal.pose.orientation.w = 1.0;
      goal.scale.x = 0.3;
      goal.scale.y = 0.3;
      goal.scale.z = 0.3;
      goal.color.r = 0.1f;
      goal.color.g = 1.0f;
      goal.color.b = 0.1f;
      goal.color.a = 1.0f;
    }
    markers.markers.push_back(goal);
    marker_pub_->publish(markers);
  }

  void goalCallback(const std_msgs::msg::Float32MultiArray::ConstSharedPtr message) {
    pose_goal_active_ = false;
    const auto result = tracker_.accept(message->data, steadySeconds());
    if (result == legged_controllers::FollowGoalTracker::AcceptResult::Accepted)
    {
      goal_marker_x_ = message->data[0];
      goal_marker_y_ = message->data[1];
      goal_marker_visible_ = true;
      tick();
      timer_->reset();
      return;
    }
    if (result == legged_controllers::FollowGoalTracker::AcceptResult::Invalid)
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                           "navigation_goal must be six finite floats "
                           "[x_odom,y_odom,speed,target_rad,radspeed,visible], "
                           "with visible equal to 0 or 1");
    goal_marker_visible_ = false;
    publishNavigationMarkers();
    publish(legged_controllers::FollowGoalTracker::Output{});
  }

  void poseGoalCallback(const geometry_msgs::msg::PoseStamped::ConstSharedPtr message) {
    const auto& position = message->pose.position;
    const auto& orientation = message->pose.orientation;
    const std::array<double, 4> quaternion{{orientation.x, orientation.y, orientation.z, orientation.w}};
    const double norm = std::sqrt(quaternion[0] * quaternion[0] + quaternion[1] * quaternion[1] +
                                  quaternion[2] * quaternion[2] + quaternion[3] * quaternion[3]);
    if (message->header.frame_id != kOdomFrame || !std::isfinite(position.x) || !std::isfinite(position.y) ||
        !std::all_of(quaternion.begin(), quaternion.end(), [](double value) { return std::isfinite(value); }))
    {
      pose_goal_active_ = false;
      tracker_.reset();
      RCLCPP_WARN(get_logger(), "goal_pose must be a finite pose in device0/odom");
      return;
    }
    float heading = odom_received_ ? pose_.yaw : 0.0f;
    if (norm >= 1e-8)
    {
      const double x = quaternion[0] / norm;
      const double y = quaternion[1] / norm;
      const double z = quaternion[2] / norm;
      const double w = quaternion[3] / norm;
      heading = static_cast<float>(std::atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z)));
    }
    pose_goal_ = {static_cast<float>(position.x), static_cast<float>(position.y), 0.0f, heading, 0.0f, 1.0f};
    pose_goal_active_ = true;
    goal_marker_x_ = pose_goal_[0];
    goal_marker_y_ = pose_goal_[1];
    goal_marker_visible_ = true;
    tick();
    timer_->reset();
  }

  void odomCallback(const nav_msgs::msg::Odometry::ConstSharedPtr message) {
    const auto& position = message->pose.pose.position;
    const auto& orientation = message->pose.pose.orientation;
    const std::array<double, 4> quaternion{{orientation.x, orientation.y, orientation.z, orientation.w}};
    const double norm = std::sqrt(quaternion[0] * quaternion[0] + quaternion[1] * quaternion[1] +
                                  quaternion[2] * quaternion[2] + quaternion[3] * quaternion[3]);
    if (message->header.frame_id != kOdomFrame || message->child_frame_id != kBaseFrame || !std::isfinite(position.x) ||
        !std::isfinite(position.y) ||
        !std::all_of(quaternion.begin(), quaternion.end(), [](double value) { return std::isfinite(value); }) ||
        !std::isfinite(norm) || norm < 1e-8)
    {
      odom_received_ = false;
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                           "odometry must contain a finite device0/base_link pose in device0/odom");
      return;
    }

    const double x = quaternion[0] / norm;
    const double y = quaternion[1] / norm;
    const double z = quaternion[2] / norm;
    const double w = quaternion[3] / norm;
    const double yaw = std::atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z));
    const legged_controllers::FollowGoalTracker::Pose2D current_pose{
        static_cast<float>(position.x), static_cast<float>(position.y), static_cast<float>(yaw)};
    if (!std::isfinite(current_pose.x) || !std::isfinite(current_pose.y) || !std::isfinite(current_pose.yaw))
    {
      odom_received_ = false;
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "odometry pose exceeds planner numeric range");
      return;
    }
    const double receive_seconds = steadySeconds();
    pose_ = current_pose;
    odom_time_seconds_ = receive_seconds;
    odom_received_ = true;
  }

  void tick() {
    const double now_seconds = steadySeconds();
    if (pose_goal_active_)
    {
      const std::vector<float> goal(pose_goal_.begin(), pose_goal_.end());
      (void)tracker_.accept(goal, now_seconds);
    }
    const bool was_active = tracker_.active();
    const bool odom_fresh =
        odom_received_ && now_seconds >= odom_time_seconds_ && now_seconds - odom_time_seconds_ <= kOdomTimeoutSeconds;
    const auto output = tracker_.update(now_seconds, odom_fresh ? &pose_ : nullptr);
    if (was_active && !tracker_.active())
    {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "navigation_goal timed out; output Hold");
      goal_marker_visible_ = false;
    }
    else if (tracker_.active() && !odom_fresh)
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "odometry is unavailable or stale; output Hold");
    publishNavigationMarkers();
    publish(output);
  }

  void publish(const legged_controllers::FollowGoalTracker::Output& output) {
    std_msgs::msg::Float32MultiArray input;
    input.data.assign(output.data.begin(), output.data.end());
    input_pub_->publish(input);
    std_msgs::msg::String status;
    status.data = output.mode;
    status_pub_->publish(status);
  }

  legged_controllers::FollowGoalTracker tracker_;
  legged_controllers::FollowGoalTracker::Pose2D pose_;
  bool odom_received_{false};
  double odom_time_seconds_{0.0};
  bool goal_marker_visible_{false};
  float goal_marker_x_{0.0f};
  float goal_marker_y_{0.0f};
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr input_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr goal_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_goal_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::array<float, legged_controllers::FollowGoalTracker::kGoalSize> pose_goal_{};
  bool pose_goal_active_{false};
};

}  // namespace

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<GoalToPlanner>();
  rclcpp::spin(node);
  if (rclcpp::ok()) node->stop();
  rclcpp::shutdown();
  return 0;
}
