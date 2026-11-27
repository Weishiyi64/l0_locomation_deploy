#include <algorithm>
#include <array>
#include <cmath>
#include <functional>
#include <memory>
#include <utility>

#include "LeggedController.h"

namespace legged {

namespace {

constexpr const char* kFollowFaceInputTopic = "/algorithm/em/follow_face";
constexpr const char* kPlannerCommandFrame = "device0/base_link";

}  // namespace

void LeggedController::ModeSubscribe(rclcpp::Node::SharedPtr /* controller_nh */) {
  if (!wbc_callback_group_)
  {
    wbc_callback_group_ = nh_->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  }

  subJoy_ = nh_->create_subscription<sensor_msgs::msg::Joy>(
      "/joy", rclcpp::QoS(10), std::bind(&LeggedController::JoyCallback, this, std::placeholders::_1));
  subEmgstop_ = nh_->create_subscription<std_msgs::msg::Bool>(
      "emergency_stop", rclcpp::QoS(1),
      std::bind(&LeggedController::EmergencyStopCallback, this, std::placeholders::_1));
  const auto latest_qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile();
  subCmdRl_ = nh_->create_subscription<robot_interfaces::msg::PolicyCommand>(
      "control/policy/command/rl", latest_qos,
      std::bind(&LeggedController::lowCmdRlCallback, this, std::placeholders::_1));
  subCmdImitation_ = nh_->create_subscription<robot_interfaces::msg::PolicyCommand>(
      "control/policy/command/imitation", latest_qos,
      std::bind(&LeggedController::lowCmdImitationCallback, this, std::placeholders::_1));
  subCmdWbc_ = nh_->create_subscription<robot_interfaces::msg::PolicyCommand>(
      "control/policy/command/wbc", rclcpp::QoS(1).best_effort().durability_volatile(),
      std::bind(&LeggedController::lowCmdWbcCallback, this, std::placeholders::_1));
  rclcpp::SubscriptionOptions wbc_options;
  wbc_options.callback_group = wbc_callback_group_;
  subWbcMotionDone_ = nh_->create_subscription<std_msgs::msg::String>(
      "control/policy/wbc/done", rclcpp::QoS(10),
      std::bind(&LeggedController::wbcMotionDoneCallback, this, std::placeholders::_1), wbc_options);
  if (wbc_enabled_) {
    joystick_soft_stop_client_ = nh_->create_client<robot_interfaces::srv::ControlWbcMotion>(
        "control/wbc/stop", rmw_qos_profile_services_default, wbc_callback_group_);
    soft_stop_wbc_service_ = nh_->create_service<robot_interfaces::srv::ControlWbcMotion>(
        "control/wbc/stop",
        std::bind(&LeggedController::softStopWbc,this,std::placeholders::_1,std::placeholders::_2),
        rmw_qos_profile_services_default,wbc_callback_group_);
  }
  initializeMotionAdmission();
  if (radarControlEnabled_)
  {
    plannerCommandSub_ = nh_->create_subscription<geometry_msgs::msg::TwistStamped>(
        "control/cmd_vel", rclcpp::QoS(10),
        std::bind(&LeggedController::plannerCommandCallback, this, std::placeholders::_1));
  }
  if (faceFollowEnabled_)
  {
    subFollowFace_ = nh_->create_subscription<std_msgs::msg::String>(
        kFollowFaceInputTopic, rclcpp::QoS(10),
        std::bind(&LeggedController::followFaceCallback, this, std::placeholders::_1));
  }

  subImitationMotionDone_ = nh_->create_subscription<std_msgs::msg::Bool>(
      "/imitation_motion_done", rclcpp::QoS(10),
      std::bind(&LeggedController::imitationMotionDoneCallback, this, std::placeholders::_1));
}

void LeggedController::ModePublish(rclcpp::Node::SharedPtr controller_nh) {
  pubState_ = nh_->create_publisher<robot_interfaces::msg::PolicyObservation>(
      "control/policy/observation", rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile());

  joyFeedbackPub_ = nh_->create_publisher<sensor_msgs::msg::JoyFeedbackArray>("/joy/set_feedback", rclcpp::QoS(1));
  policyMux_.createRosInterfaces();
  controller_status_pub_ = nh_->create_publisher<std_msgs::msg::String>(
      "/system/motion_controller/status", rclcpp::QoS(1).reliable().durability_volatile());
  controller_status_timer_ = nh_->create_wall_timer(std::chrono::seconds(1),
      [this] { publishControllerStatus(); });
  wbcActionFeedbackPub_ = nh_->create_publisher<std_msgs::msg::String>(
      "/system/state_machine/wbc_action_fdb", rclcpp::QoS(10).reliable().durability_volatile());

  VisualizationPublisher::Params vp;
  vizPub_ = std::make_unique<VisualizationPublisher>(controller_nh, vp, enableRviz_);
  RCLCPP_INFO(nh_->get_logger(), "Publishing /system/joint/jointstates independently of RViz; /joint_rad_text %s",
              enableRviz_ ? "enabled" : "disabled");
}

void LeggedController::lowCmdRlCallback(const robot_interfaces::msg::PolicyCommand::ConstSharedPtr msg) {
  policyMux_.setRlCommand(*msg);
}

void LeggedController::lowCmdImitationCallback(
    const robot_interfaces::msg::PolicyCommand::ConstSharedPtr msg) {
  policyMux_.setImitationCommand(*msg);
}

void LeggedController::lowCmdWbcCallback(const robot_interfaces::msg::PolicyCommand::ConstSharedPtr msg) {
  policyMux_.setWbcCommand(*msg);
}

void LeggedController::plannerCommandCallback(const geometry_msgs::msg::TwistStamped::ConstSharedPtr msg) {
  const int64_t stamp_ns = rclcpp::Time(msg->header.stamp).nanoseconds();
  const rclcpp::Time current = nh_->now();
  const double age = (current - rclcpp::Time(msg->header.stamp)).seconds();
  const std::array<double, 6> values{{msg->twist.linear.x, msg->twist.linear.y, msg->twist.linear.z,
                                      msg->twist.angular.x, msg->twist.angular.y, msg->twist.angular.z}};
  const bool finite = std::all_of(values.begin(), values.end(), [](double value) { return std::isfinite(value); });
  const bool planar = msg->twist.linear.z == 0.0 && msg->twist.angular.x == 0.0 && msg->twist.angular.y == 0.0;

  std::lock_guard<std::mutex> lock(plannerCommandMutex_);
  plannerCommandSeen_ = true;
  if (plannerCommandClockNs_ > 0 && current.nanoseconds() < plannerCommandClockNs_) plannerCommandStampNs_ = 0;
  plannerCommandClockNs_ = current.nanoseconds();
  if (!finite || !planar || msg->header.frame_id != kPlannerCommandFrame || stamp_ns <= 0 || age < 0.0 || age > 0.5 ||
      (plannerCommandStampNs_ > 0 && stamp_ns < plannerCommandStampNs_))
  {
    plannerCommandValid_ = false;
    return;
  }

  plannerCommand_.linear.x = msg->twist.linear.x;
  plannerCommand_.linear.y = msg->twist.linear.y;
  plannerCommand_.angular.z = msg->twist.angular.z;
  plannerCommandStampNs_ = stamp_ns;
  plannerCommandReceiveTime_ = std::chrono::steady_clock::now();
  plannerCommandValid_ = true;
}

std::array<double, 3> LeggedController::readCurrentHeadRpyRad() {
  std::array<double, 3> head{{0.0, 0.0, 0.0}};
  if (!robot_hw_) return head;
  const auto states = robot_hw_->getHyridJointMotorState();
  for (std::size_t i = 0; i < jointNames_.size() && i < states.size(); ++i)
  {
    const auto& name = jointNames_[i];
    if (name.find("neck_yaw") != std::string::npos || name.find("head_yaw") != std::string::npos)
      head[0] = states[i].q;
    else if (name.find("neck_pitch") != std::string::npos || name.find("head_pitch") != std::string::npos)
      head[1] = states[i].q;
    else if (name.find("neck_roll") != std::string::npos || name.find("head_roll") != std::string::npos)
      head[2] = states[i].q;
  }
  return head;
}

void LeggedController::followFaceCallback(const std_msgs::msg::String::ConstSharedPtr msg) {
  FaceFollowController::Target target;
  std::string error;
  if (!faceFollowCtrl_.parsePayload(msg->data, target, error))
  {
    RCLCPP_WARN_THROTTLE(nh_->get_logger(), *nh_->get_clock(), 2000, "Invalid %s JSON: %s", kFollowFaceInputTopic,
                         error.c_str());
    return;
  }
  const double wall_sec = nh_->now().seconds();
  faceFollowCtrl_.processTarget(
      target, wall_sec, headCtrl_, [this] { return readCurrentHeadRpyRad(); }, faceFollowAllowed());
}

void LeggedController::imitationMotionDoneCallback(const std_msgs::msg::Bool::ConstSharedPtr msg) {
  if (!msg->data) return;

  if (policyMux_.source() != PolicyCmdSource::kImitation) return;

  if (policyMux_.singleNodePublishesImitationCommand())
  {
    return;
  }

  returnToRlWithoutGainRamp(nullptr);
}

void LeggedController::JoyCallback(const sensor_msgs::msg::Joy::ConstSharedPtr msg) {
  const JoystickCommand command = joystickInput_.decode(*msg);
  lastJoystickCommand_ = command;
  processJoystickActions(command);
  if (!command.wbc_soft_stop_pressed || command.emergency_stop_pressed ||
      policyMux_.source() != PolicyCmdSource::kWbc || !policyMux_.wbcActiveCommanded() ||
      wbc_return_pending_.load() || !joystick_soft_stop_client_ ||
      !joystick_soft_stop_client_->service_is_ready())
    return;

  using Service = robot_interfaces::srv::ControlWbcMotion;
  auto request = std::make_shared<Service::Request>();
  {
    std::lock_guard<std::mutex> lock(motion_mutex_);
    if (!motion_reserved_.load() || motion_cancelled_.load() || motion_request_id_.empty()) return;
    request->request_id = motion_request_id_;
  }
  request->command = Service::Request::STOP;
  const auto logger = nh_->get_logger();
  joystick_soft_stop_client_->async_send_request(request,
      [logger](rclcpp::Client<Service>::SharedFuture future) {
        const auto response = future.get();
        if (response->success)
          RCLCPP_INFO(logger, "[Joystick] LT+D-pad left queued soft STOP for %s", response->motion_id.c_str());
        else
          RCLCPP_DEBUG(logger, "[Joystick] Soft STOP ignored: %s", response->message.c_str());
      });
}

void LeggedController::EmergencyStopCallback(const std_msgs::msg::Bool::ConstSharedPtr msg) {
  if (msg->data)
  {
    triggerEmergencyStop();
  }
}

}  // namespace legged
