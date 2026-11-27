#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <csignal>
#include <functional>
#include <cstring>
#include <sensor_msgs/msg/image.hpp>
#include <memory>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <limits>
#include <unordered_set>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/string.hpp>
#include <string>
#include <visualization_msgs/msg/marker_array.hpp>

#include "FollowGoalTracker.h"
#include "AppVersion.h"

namespace {

volatile std::sig_atomic_t stop_requested = 0;
void requestStop(int) { stop_requested = 1; }

constexpr const char* kOdomTopic = "/planner/base_odometry";
constexpr const char* kOdomFrame = "device0/odom";
constexpr const char* kBaseFrame = "base_link";
constexpr const char* kNavigationMarkerTopic = "/navigation_visualization";

class GoalToPlanner final : public rclcpp::Node {
 public:
  using Action = luvbot_control_interfaces::action::FollowNavigationPath;
  using GoalHandle = rclcpp_action::ServerGoalHandle<Action>;

  GoalToPlanner() : Node("goal_to_planner"), session_(std::to_string(now().nanoseconds())) {
    check_sensor_timestamps_ = declare_parameter<bool>("nav_check_sensor_timestamps", true);
    reference_receive_timeout_ = declare_parameter<bool>("nav_reference_receive_timeout", false);
    if (reference_receive_timeout_)
      RCLCPP_WARN(get_logger(), "Navigation reference receive-time mode: each accepted path renews reference_timeout; "
                               "source stamp age/order ignored, increasing revisions still required");
    else
      RCLCPP_INFO(get_logger(), "Navigation reference deadline uses source path.header.stamp + reference_timeout");
    if (check_sensor_timestamps_)
      RCLCPP_INFO(get_logger(), "Navigation sensor timestamp checks enabled");
    else
      RCLCPP_WARN(get_logger(), "Navigation sensor timestamp checks DISABLED: latest received odometry/depth; "
                               "0.5s receive watchdog and navigation reference deadline retained");
    input_pub_ = create_publisher<std_msgs::msg::Float32MultiArray>("/planner_input", rclcpp::QoS(1));
    status_pub_ = create_publisher<std_msgs::msg::String>("/goal_to_planner/state", rclcpp::QoS(10));
    marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(kNavigationMarkerTopic,
                                                                         rclcpp::QoS(1).reliable().transient_local());
    action_server_ = rclcpp_action::create_server<Action>(
        this, "/control/navigation/follow_path",
        std::bind(&GoalToPlanner::handleGoal, this, std::placeholders::_1, std::placeholders::_2),
        std::bind(&GoalToPlanner::handleCancel, this, std::placeholders::_1),
        std::bind(&GoalToPlanner::handleAccepted, this, std::placeholders::_1));
    planner_stop_sub_ = create_subscription<std_msgs::msg::String>(
        "/local_planner/stop_reason", rclcpp::QoS(1),
        [this](std_msgs::msg::String::ConstSharedPtr msg) {
          planner_stop_reason_ = msg->data;
          planner_status_time_ = steadySeconds();
        });
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        kOdomTopic, rclcpp::SensorDataQoS().keep_last(1),
        std::bind(&GoalToPlanner::odomCallback, this, std::placeholders::_1));
    depth_sub_ = create_subscription<sensor_msgs::msg::Image>(
        "/planner/depth_fused", rclcpp::QoS(1).reliable().durability_volatile(),
        [this](sensor_msgs::msg::Image::ConstSharedPtr image) { depthCallback(*image); });
    timer_ = create_wall_timer(std::chrono::milliseconds(20), [this] { tick(); });
    RCLCPP_DEBUG(get_logger(),
                "FollowNavigationPath Action ready on /control/navigation/follow_path: "
                "8 forward path samples at 0.4m spacing, absolute odom final yaw, 10 degree tolerance");
  }

  void stop() {
    if (active_goal_) finish(Action::Result::STOPPED, "navigation bridge stopped");
    else publish(legged_controllers::FollowGoalTracker::Output{}, false);
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

  int64_t referenceDeadline(const Action::Goal& goal) const {
    const auto& stamp = goal.path.header.stamp;
    if (stamp.sec < 0 || stamp.nanosec >= 1000000000U || !std::isfinite(goal.reference_timeout) ||
        goal.reference_timeout <= 0.0) return 0;
    const int64_t ns = reference_receive_timeout_ ? now().nanoseconds() :
        static_cast<int64_t>(stamp.sec)*1000000000LL+stamp.nanosec;
    const long double deadline = ns+static_cast<long double>(goal.reference_timeout)*1e9L;
    if (ns <= 0 || deadline >= std::numeric_limits<int64_t>::max()) return 0;
    return static_cast<int64_t>(deadline);
  }

  bool referenceExpired() const {
    return active_goal_ && (now().nanoseconds() >= reference_deadline_ns_ ||
        (reference_receive_timeout_ && steadySeconds()-reference_received_time_ >= reference_->reference_timeout));
  }

  std::string validateReference(const Action::Goal& goal) const {
    if (cancel_requested_) return "cancellation is in progress";
    if (goal.task_id.empty() || retired_tasks_.count(goal.task_id) || goal.path_revision == 0)
      return "task_id is empty/finished or path_revision is zero; finished task IDs must not be reused";
    if (goal.path.header.frame_id != kOdomFrame || goal.path.points.empty())
      return "a nonempty device0/odom path is required";
    if (!std::isfinite(goal.speed) || !std::isfinite(goal.turn_speed) ||
        (goal.use_heading && !std::isfinite(goal.goal_yaw))) return "non-finite reference speed or final yaw";
    const int64_t stamp = static_cast<int64_t>(goal.path.header.stamp.sec)*1000000000LL+
        goal.path.header.stamp.nanosec;
    const auto current = now().nanoseconds();
    if (referenceDeadline(goal) <= current || (!reference_receive_timeout_ && stamp > current))
      return "invalid, future or expired reference stamp/timeout";
    double length = 0.0;
    for (std::size_t i = 0; i < goal.path.points.size(); ++i) {
      const auto& pos = goal.path.points[i];
      const std::array<double, 3> values{{pos.x, pos.y, pos.z}};
      if (!std::all_of(values.begin(), values.end(), [](double x) { return std::isfinite(x); }) ||
          std::abs(pos.x) > std::numeric_limits<float>::max()/16.0 ||
          std::abs(pos.y) > std::numeric_limits<float>::max()/16.0) return "invalid path point numeric range";
      if (i > 0) {
        const auto& prev = goal.path.points[i-1];
        length += std::hypot(pos.x-prev.x, pos.y-prev.y);
      }
    }
    if (!std::isfinite(length)) return "invalid path length";
    if (reference_ && goal.task_id == reference_->task_id) {
      if (goal.path_revision < reference_->path_revision) return "older path revision";
      if (goal.path_revision == reference_->path_revision &&
          !legged_controllers::FollowGoalTracker::sameReference(goal, *reference_))
        return "same revision has changed execution content";
      if (!reference_receive_timeout_ && stamp < reference_stamp_ns_)
        return "reference generation stamp moved backwards";
    }
    return {};
  }

  rclcpp_action::GoalResponse handleGoal(const rclcpp_action::GoalUUID&, std::shared_ptr<const Action::Goal> goal) {
    RCLCPP_DEBUG(get_logger(), "Navigation Action received: task_id=%s revision=%llu points=%zu",
                 goal->task_id.c_str(), static_cast<unsigned long long>(goal->path_revision),
                 goal->path.points.size());
    checkClock();
    if (!cancel_requested_ && referenceExpired())
      finish(Action::Result::REFERENCE_EXPIRED, "reference expired");
    const auto error = validateReference(*goal);
    if (!error.empty()) {
      RCLCPP_WARN(get_logger(), "Navigation reference rejected: %s", error.c_str());
      return rclcpp_action::GoalResponse::REJECT;
    }
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handleCancel(const std::shared_ptr<GoalHandle> handle) {
    if (handle != active_goal_) return rclcpp_action::CancelResponse::REJECT;
    cancel_requested_ = true;
    ++reference_generation_;
    publish(legged_controllers::FollowGoalTracker::Output{}, false);
    // rclcpp_action changes to CANCELING only after this callback returns.
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handleAccepted(const std::shared_ptr<GoalHandle> handle) {
    checkClock();
    if (!cancel_requested_ && referenceExpired())
      finish(Action::Result::REFERENCE_EXPIRED, "reference expired");
    const auto goal = handle->get_goal();
    const auto error = validateReference(*goal);
    if (!error.empty()) {
      auto result = std::make_shared<Action::Result>();
      result->task_id = goal->task_id; result->path_revision = goal->path_revision;
      result->code = Action::Result::STOPPED; result->message = error;
      handle->abort(result);
      return;
    }
    const bool new_task = !reference_ || goal->task_id != reference_->task_id;
    const bool new_revision = new_task || goal->path_revision != reference_->path_revision;
    if (active_goal_) finish(Action::Result::REPLACED, "replaced by accepted navigation reference", new_task, false);
    if (new_task) ++task_epoch_;
    if (new_revision) ++reference_generation_;
    active_goal_ = handle;
    reference_ = goal;
    reference_stamp_ns_ = rclcpp::Time(goal->path.header.stamp).nanoseconds();
    reference_received_time_ = steadySeconds();
    reference_deadline_ns_ = referenceDeadline(*goal);
    tracker_.setReference(goal, new_task);
    goal_marker_x_ = goal->path.points.back().x;
    goal_marker_y_ = goal->path.points.back().y;
    goal_marker_visible_ = true;
    tick(true);
  }

  void finish(uint8_t code, const std::string& message, bool retire = true, bool send_stop = true) {
    if (!active_goal_) return;
    auto result = std::make_shared<Action::Result>();
    result->task_id = reference_->task_id; result->path_revision = reference_->path_revision;
    result->code = code; result->message = message;
    RCLCPP_INFO(get_logger(), "Navigation result: task_id=%s revision=%llu code=%u reason=%s",
                result->task_id.c_str(), static_cast<unsigned long long>(result->path_revision),
                static_cast<unsigned>(code), message.c_str());
    if (retire) retired_tasks_.insert(reference_->task_id);
    if (code == Action::Result::SUCCEEDED) active_goal_->succeed(result);
    else if (active_goal_->is_canceling()) active_goal_->canceled(result);
    else active_goal_->abort(result);
    active_goal_.reset(); reference_.reset();
    cancel_requested_ = false;
    if (send_stop) {
      ++reference_generation_;
      tracker_.reset();
      goal_marker_visible_ = false;
      publishNavigationMarkers();
      publish(legged_controllers::FollowGoalTracker::Output{}, false);
    }
  }

  void checkClock() {
    const auto current = now().nanoseconds();
    if (last_ros_time_ns_ > 0 && current < last_ros_time_ns_) {
      odom_received_ = depth_ready_ = false;
      odom_stamp_ = depth_stamp_ = 0;
      if (active_goal_) finish(Action::Result::STOPPED, "ROS clock moved backwards");
    }
    last_ros_time_ns_ = current;
  }

  void odomCallback(const nav_msgs::msg::Odometry::ConstSharedPtr message) {
    checkClock();
    if (message->header.stamp.sec < 0 || message->header.stamp.nanosec >= 1000000000U) { odom_received_ = false; return; }
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
                           "odometry must contain a finite base_link pose in device0/odom");
      return;
    }

    const double x = quaternion[0] / norm;
    const double y = quaternion[1] / norm;
    const double z = quaternion[2] / norm;
    const double w = quaternion[3] / norm;
    const double yaw = std::atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z));
    const legged_controllers::FollowGoalTracker::Pose2D current_pose{
        position.x, position.y, yaw};
    if (!std::isfinite(current_pose.x) || !std::isfinite(current_pose.y) || !std::isfinite(current_pose.yaw) ||
        std::abs(current_pose.x) > std::numeric_limits<float>::max()/16.0 ||
        std::abs(current_pose.y) > std::numeric_limits<float>::max()/16.0)
    {
      odom_received_ = false;
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "odometry pose exceeds planner numeric range");
      return;
    }
    const double receive_seconds = steadySeconds();
    pose_ = current_pose;
    odom_time_seconds_ = receive_seconds;
    const auto stamp = rclcpp::Time(message->header.stamp).nanoseconds();
    if (check_sensor_timestamps_ &&
        (stamp <= 0 || (now().nanoseconds()-stamp)*1e-9 < 0. || (now().nanoseconds()-stamp)*1e-9 > .5)) {
      odom_received_ = false; return;
    }
    odom_stamp_ = stamp;
    odom_received_ = true;
  }

  void tick(bool force = false) {
    checkClock();
    if (!active_goal_) {
      const double current = steadySeconds();
      if (current-last_publish_time_ >= .2) {
        last_publish_time_ = current;
        publishNavigationMarkers();
      }
      return;
    }
    if (cancel_requested_) {
      if (active_goal_->is_canceling()) finish(Action::Result::CANCELED, "canceled by navigation client");
      return;
    }
    if (referenceExpired()) {
      finish(Action::Result::REFERENCE_EXPIRED, reference_receive_timeout_ ?
          "reference receive timeout; no accepted path received within reference_timeout" :
          "reference expired; retransmission does not renew its stamp");
      return;
    }
    const double current = steadySeconds();
    if (!force && current-last_publish_time_ < .2) return;
    last_publish_time_ = current;
    const bool fresh = sensorsFresh();
    auto output = fresh ? tracker_.update(pose_) : legged_controllers::FollowGoalTracker::Output{};
    const auto& end = reference_->path.points.back();
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
        "[navigation_tracking] task_id=%s revision=%llu fresh=%d mode=%s use_heading=%d "
        "pose=(%.3f,%.3f,%.3f) goal=(%.3f,%.3f) remaining=%.3f reached=%d "
        "goal_body=(%.3f,%.3f) reference_vx=%.3f heading_error=%.3f "
        "goal_yaw=%.3f endpoint_distance=%.3f requested_turn_speed=%.3f",
        reference_->task_id.c_str(), static_cast<unsigned long long>(reference_->path_revision),
        static_cast<int>(fresh), output.mode, static_cast<int>(reference_->use_heading),
        pose_.x, pose_.y, pose_.yaw, end.x, end.y, output.remaining_distance,
        static_cast<int>(output.reached), output.data[0], output.data[1], output.data[2], output.heading_error,
        reference_->goal_yaw, std::hypot(end.x-pose_.x, end.y-pose_.y), reference_->turn_speed);
    if (fresh && output.reached) {
      auto feedback = std::make_shared<Action::Feedback>();
      feedback->task_id = reference_->task_id; feedback->path_revision = reference_->path_revision;
      feedback->phase = Action::Feedback::HOLD;
      feedback->remaining_distance = output.remaining_distance; feedback->heading_error = output.heading_error;
      active_goal_->publish_feedback(feedback);
      finish(Action::Result::SUCCEEDED, "final position and requested absolute heading reached");
      return;
    }
    std::string reason;
    if (!fresh) reason = "odometry or depth is unavailable/stale";
    else if (current-planner_status_time_ > .5) reason = "planner status is unavailable/stale";
    else reason = planner_stop_reason_;
    logStopReason(reason);
    auto feedback = std::make_shared<Action::Feedback>();
    feedback->task_id = reference_->task_id; feedback->path_revision = reference_->path_revision;
    feedback->remaining_distance = output.remaining_distance; feedback->heading_error = output.heading_error;
    feedback->stop_reason = reason;
    feedback->phase = !reason.empty() ? Action::Feedback::HOLD :
        std::string(output.mode) == "Nav" ? Action::Feedback::NAV :
        std::string(output.mode) == "Face" ? Action::Feedback::FACE : Action::Feedback::HOLD;
    active_goal_->publish_feedback(feedback);
    publishNavigationMarkers();
    if (!fresh) output.data.fill(0.f);
    publish(output, fresh);
  }

  bool sensorsFresh() const {
    const auto age = [this](int64_t ns) { return (now().nanoseconds()-ns)*1e-9; };
    return depth_ready_ && steadySeconds()-depth_time_ <= .5 && odom_received_ &&
        steadySeconds()-odom_time_seconds_ <= .5 &&
        (!check_sensor_timestamps_ || (depth_stamp_ > 0 && age(depth_stamp_) >= 0. &&
         age(depth_stamp_) <= .5 && odom_stamp_ > 0 && age(odom_stamp_) >= 0. && age(odom_stamp_) <= .5));
  }

  void logStopReason(const std::string& reason) {
    if (stop_reason_logged_ && reason == last_stop_reason_) return;
    stop_reason_logged_ = true;
    last_stop_reason_ = reason;
    const auto ros_time = now().nanoseconds();
    const auto source_age = [ros_time](int64_t stamp) { return stamp > 0 ? (ros_time-stamp)*1e-6 : -1.; };
    const double received_time = steadySeconds();
    RCLCPP_INFO(get_logger(), "[navigation_stop] reason=%s timestamp_checks=%d "
                "depth_source_age_ms=%.1f odom_source_age_ms=%.1f "
                "depth_receive_age_ms=%.1f odom_receive_age_ms=%.1f depth_valid=%d odom_valid=%d "
                "reference_receive_timeout=%d reference_receive_age_ms=%.1f task_id=%s",
                reason.empty() ? "ready" : reason.c_str(), static_cast<int>(check_sensor_timestamps_),
                source_age(depth_stamp_), source_age(odom_stamp_),
                depth_time_ > 0. ? (received_time-depth_time_)*1000. : -1.,
                odom_time_seconds_ > 0. ? (received_time-odom_time_seconds_)*1000. : -1.,
                static_cast<int>(depth_ready_), static_cast<int>(odom_received_),
                static_cast<int>(reference_receive_timeout_),
                reference_ ? (received_time-reference_received_time_)*1000. : -1.,
                reference_ ? reference_->task_id.c_str() : "(none)");
  }

  void depthCallback(const sensor_msgs::msg::Image& image) {
    checkClock();
    if (image.header.stamp.sec < 0 || image.header.stamp.nanosec >= 1000000000U) { depth_ready_ = false; return; }
    const auto ns = rclcpp::Time(image.header.stamp).nanoseconds();
    const double age = (now().nanoseconds()-ns)*1e-9;
    if (image.encoding != "32FC1" || image.width != 64 || image.height != 40 || image.step < 64*4 ||
        image.data.size() < static_cast<std::size_t>(image.step)*40 ||
        image.header.frame_id != "device0/lidar" ||
        (check_sensor_timestamps_ && (ns <= 0 || age < 0. || age > .5 || ns <= depth_stamp_))) {
      depth_ready_ = false; depth_stamp_ = 0; return;
    }
    const uint16_t probe = 1;
    const bool big = *reinterpret_cast<const uint8_t*>(&probe) == 0;
    for (std::size_t row = 0; row < 40; ++row)
      for (std::size_t col = 0; col < 64; ++col) {
        uint8_t bytes[4];
        std::memcpy(bytes, image.data.data()+row*image.step+4*col, 4);
        if (bool(image.is_bigendian) != big) std::reverse(bytes, bytes+4);
        float value; std::memcpy(&value, bytes, 4);
        if (!std::isfinite(value) || value < 0.f) { depth_ready_ = false; return; }
        depth_[row*64+col] = value;
      }
    depth_stamp_ = ns; depth_time_ = steadySeconds(); depth_ready_ = true;
    // Refresh the planner's cache immediately; its own timer controls inference cadence.
    tick(true);
  }

  void publish(const legged_controllers::FollowGoalTracker::Output& output, bool fresh) {
    std_msgs::msg::Float32MultiArray input;
    input.data.assign(output.data.begin(), output.data.end());
    input.data.insert(input.data.end(), depth_.begin(), depth_.end());
    const bool active = active_goal_ && !cancel_requested_;
    if (!active) logStopReason("navigation reference is inactive");
    input.data.push_back(active && fresh ? 1.f : 0.f);
    std_msgs::msg::MultiArrayDimension identity;
    identity.label = "goal:"+session_+":"+std::to_string(task_epoch_);
    identity.size = input.data.size(); identity.stride = input.data.size();
    input.layout.dim.push_back(identity);
    std_msgs::msg::MultiArrayDimension provenance;
    provenance.label = "sensor_stamp_ns:"+std::to_string(std::min(depth_stamp_, odom_stamp_));
    input.layout.dim.push_back(provenance);
    std_msgs::msg::MultiArrayDimension reference;
    reference.label = "reference:"+std::to_string(reference_generation_);
    input.layout.dim.push_back(reference);
    reference.label = "reference_deadline_ns:"+std::to_string(active ? reference_deadline_ns_ : 0);
    input.layout.dim.push_back(reference);
    reference.label = active ? "reference_active:1" : "reference_active:0";
    input.layout.dim.push_back(reference);
    input_pub_->publish(input);
    std_msgs::msg::String status;
    status.data = output.mode;
    status_pub_->publish(status);
  }

  legged_controllers::FollowGoalTracker tracker_;
  legged_controllers::FollowGoalTracker::Pose2D pose_;
  bool odom_received_{false};
  bool check_sensor_timestamps_{true}, stop_reason_logged_{false};
  bool reference_receive_timeout_{false};
  double reference_received_time_{0.};
  std::string last_stop_reason_;
  double odom_time_seconds_{0.0};
  bool goal_marker_visible_{false};
  double goal_marker_x_{0.0};
  double goal_marker_y_{0.0};
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr input_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp_action::Server<Action>::SharedPtr action_server_;
  std::shared_ptr<GoalHandle> active_goal_;
  std::shared_ptr<const Action::Goal> reference_;
  std::unordered_set<std::string> retired_tasks_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr planner_stop_sub_;
  std::string planner_stop_reason_, session_;
  double planner_status_time_{0.0}, last_publish_time_{0.0};
  int64_t reference_stamp_ns_{0}, reference_deadline_ns_{0}, last_ros_time_ns_{0};
  uint64_t reference_generation_{0};
  bool cancel_requested_{false};
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::uint64_t task_epoch_{0};
  std::array<float, 2560> depth_{};
  bool depth_ready_{false};
  int64_t depth_stamp_{0}, odom_stamp_{0};
  double depth_time_{0.};
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub_;
};

}  // namespace

int main(int argc, char** argv) {
  l0::printAppVersion("goal_to_planner");
  std::signal(SIGINT, requestStop);
  std::signal(SIGTERM, requestStop);
  // Keep DDS alive while terminating the Action and invalidating the planner reference.
  rclcpp::init(argc, argv, rclcpp::InitOptions(), rclcpp::SignalHandlerOptions::None);
  auto node = std::make_shared<GoalToPlanner>();
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  auto stop_timer = node->create_wall_timer(std::chrono::milliseconds(20), [&executor]() {
    if (stop_requested) executor.cancel();
  });
  executor.spin();
  stop_timer->cancel();
  if (rclcpp::ok()) node->stop();
  rclcpp::shutdown();
  return 0;
}
