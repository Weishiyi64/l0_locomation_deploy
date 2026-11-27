#include "MotionRuntime.h"

#include <robot_interfaces/msg/policy_observation.hpp>
#include <std_msgs/msg/string.hpp>
#include <algorithm>
#include <atomic>
#include <cmath>
#include <functional>
#include <future>
#include <map>
#include <mutex>
#include <set>
#include <thread>

namespace legged {
namespace motion {
namespace {
using String = std_msgs::msg::String;
using Observation = robot_interfaces::msg::PolicyObservation;
using Handle = rclcpp_action::ServerGoalHandle<MotionAction>;
using Client = rclcpp::Client<Transaction>;
using Request = Transaction::Request;

class MotionBackend final : public rclcpp::Node {
 public:
  MotionBackend() : Node("motion_backend") {
    group_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    poll_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    assets_ = loadAssets(declare_parameter<std::string>("deployment", ""));
    mapping_ = declare_parameter<std::string>("mapping", "");
    const double seconds = declare_parameter<double>("generation_timeout_sec", 3.0);
    require(std::isfinite(seconds) && seconds > 0 && seconds <= 3.0, "Generation timeout must be in (0,3] seconds");
    generation_ns_ = static_cast<std::int64_t>(seconds * 1000000000.0);
    allowed_ = Json::parse(declare_parameter<std::string>("fixed_actions_json", "[]"));
    require(allowed_.is_array(), "fixed_actions_json must be an array");
    for (const auto& id : allowed_)
      require(id.is_string() && assets_.actions.count(id.get<std::string>()), "Fixed action is absent from installed catalog");
    transaction_ = create_client<Transaction>(kTransaction, rmw_qos_profile_services_default, group_);
    generation_ = create_client<Transaction>(kGeneration, rmw_qos_profile_services_default, group_);
    fixed_ = create_client<Transaction>(kFixedFrontend, rmw_qos_profile_services_default, group_);
    json_pub_ = create_publisher<String>("/text_motion/json", 1);
    cancel_pub_ = create_publisher<String>("/text_motion/cancel", 10);
    status_pub_ = create_publisher<String>("/control/motion/status", rclcpp::QoS(1).reliable().transient_local());
    rclcpp::SubscriptionOptions options;
    options.callback_group = group_;
    observation_sub_ = create_subscription<Observation>("/control/policy/observation", rclcpp::QoS(1).best_effort(),
        [this](Observation::ConstSharedPtr value) { std::lock_guard<std::mutex> lock(mutex_); observation_ = value; }, options);
    json_sub_ = create_subscription<String>("/text_motion/wbc_status", 20,
        [this](String::ConstSharedPtr value) {
          try { auto event = objectJson(value->data); std::lock_guard<std::mutex> lock(mutex_);
            if (event.value("request_id", "") == current_id_) json_event_ = std::move(event);
          } catch (const std::exception&) {}
        }, options);
    terminal_sub_ = create_subscription<String>("/system/state_machine/wbc_action_fdb", 20,
        [this](String::ConstSharedPtr value) {
          try {
            const auto event = objectJson(value->data);
            std::lock_guard<std::mutex> lock(mutex_);
            if (event.value("action_id", "") != active_motion_ || !event.count("timestamp_ms") || event.at("timestamp_ms") != active_stamp_) return;
            const auto status = event.value("status", "");
            if (status == "completed" || status == "cancelled" || status == "failed" || status == "rejected") {
              terminal_ = status;
              terminal_detail_ = event.value("detail", "");
            }
          } catch (const std::exception&) {}
        }, options);
    cancel_sub_ = create_subscription<String>("/text_motion/cancel", 10,
        [this](String::ConstSharedPtr value) { std::lock_guard<std::mutex> lock(mutex_);
          if (busy_ && value->data == current_id_) external_cancel_ = true;
        }, options);
    server_ = rclcpp_action::create_server<MotionAction>(get_node_base_interface(), get_node_clock_interface(),
        get_node_logging_interface(), get_node_waitables_interface(), kAction,
        [this](const rclcpp_action::GoalUUID&, std::shared_ptr<const Goal> goal) { return admit(*goal); },
        [](const std::shared_ptr<Handle>) { return rclcpp_action::CancelResponse::ACCEPT; },
        [this](std::shared_ptr<Handle> handle) { startWorker([this, handle] { execute(handle); }); },
        rcl_action_server_get_default_options(), group_);
    poll_timer_ = create_wall_timer(std::chrono::milliseconds(50), std::bind(&MotionBackend::poll, this), poll_group_);
    RCLCPP_INFO(get_logger(), "C++ motion gateway ready; one AMP reservation, no request queue, JSON deadline <=3s");
  }
  ~MotionBackend() override {
    stopping_.store(true);
    std::lock_guard<std::mutex> lock(worker_mutex_);
    if (worker_.joinable()) worker_.join();
  }

 private:
  bool running() const { return rclcpp::ok() && !stopping_.load(); }
  void startWorker(std::function<void()> work) {
    std::lock_guard<std::mutex> lock(worker_mutex_);
    if (worker_.joinable()) worker_.join();
    worker_ = std::thread([this, work] {
      try { work(); }
      catch (const std::exception& error) { RCLCPP_ERROR(get_logger(), "Motion cleanup remains unconfirmed: %s", error.what()); }
    });
  }
  void publish(rclcpp::Publisher<String>::SharedPtr publisher, const Json& value) {
    String message; message.data = encode(value); publisher->publish(message);
  }
  void publishCancel(const std::string& id) { String message; message.data = id; cancel_pub_->publish(message); }
  Json snapshot() {
    Observation::ConstSharedPtr message;
    { std::lock_guard<std::mutex> lock(mutex_); message = observation_; }
    require(message != nullptr, "No controller observation");
    const auto stamp = static_cast<std::int64_t>(message->header.stamp.sec) * 1000000000LL + message->header.stamp.nanosec;
    const double age = (get_clock()->now().nanoseconds() - stamp) * 1e-9;
    require(message->inference_enabled && age >= 0 && age <= 0.2 &&
            message->joints.name.size() == message->joints.position.size(), "No fresh enabled controller observation");
    std::map<std::string, double> joints;
    for (std::size_t i = 0; i < message->joints.name.size(); ++i)
      require(joints.emplace(message->joints.name[i], message->joints.position[i]).second, "Duplicate observation joint");
    std::vector<double> positions;
    for (const auto& name : assets_.names) positions.push_back(joints.at(name));
    const auto& p = message->base_pose.position;
    const auto& q = message->imu.orientation;
    if (limits_.empty()) limits_ = loadLimits(mapping_, assets_.names);  // Only the single worker takes snapshots.
    return checkSnapshot({{"joint_names", assets_.names}, {"joint_positions", positions},
        {"base_position", {p.x, p.y, p.z}}, {"base_quaternion_wxyz", {q.w, q.x, q.y, q.z}},
        {"captured_stamp_ns", stamp}}, assets_.names, limits_);
  }
  void poll() {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!running()) return;
    const auto now = steadyNs();
    if (poll_pending_ && now - poll_started_ > 500000000LL) {
      transaction_->remove_pending_request(poll_request_id_);
      poll_pending_ = false;
      ++poll_epoch_;
    }
    if (poll_pending_ || !transaction_->service_is_ready()) return;
    auto request = std::make_shared<Request>(); request->command = Request::STATUS;
    const auto epoch = ++poll_epoch_;
    poll_pending_ = true;
    poll_started_ = now;
    const auto pending = transaction_->async_send_request(request, [this, epoch](Client::SharedFuture future) { statusResponse(epoch, future); });
    poll_request_id_ = pending.request_id;
  }
  void statusResponse(std::uint64_t epoch, Client::SharedFuture future) {
    try {
      const auto response = future.get();
      const auto state = objectJson(response->payload_json.empty() ? "{}" : response->payload_json);
      Goal orphan;
      bool recover = false;
      Json status;
      {
        std::lock_guard<std::mutex> lock(mutex_);
        if (epoch != poll_epoch_ || !running()) return;
        poll_pending_ = false;
        controller_ = response;
        controller_seen_ = steadyNs();
        const auto id = state.value("request_id", "");
        if (!busy_ && !id.empty() && !contains(recent_, id)) {
          busy_ = true;
          current_id_ = id;
          remember(recent_, id);
          orphan.source = state.at("source").get<std::string>();
          orphan.request_id = id;
          recover = true;
        }
        status = {{"ready", response->ready && !busy_}, {"busy", busy_}, {"request_id", current_id_},
            {"ready_since_ms", response->ready_since_ms}, {"detail", response->message}};
      }
      publish(status_pub_, status);
      if (recover) startWorker([this, orphan] { retireGenerator(orphan, nullptr); cleanup(orphan, nullptr, ""); });
    } catch (const std::exception& error) {
      RCLCPP_WARN(get_logger(), "Controller status unavailable: %s", error.what());
      std::lock_guard<std::mutex> lock(mutex_);
      if (epoch == poll_epoch_) poll_pending_ = false;
    }
  }
  rclcpp_action::GoalResponse admit(const Goal& goal) {
    try {
      require(running() && (goal.source == "wbc" || goal.source == "txt2motion") && validId(goal.request_id), "Invalid source/request ID");
      const auto now = nowMs();
      require(goal.timestamp_ms >= now - 5000 && goal.timestamp_ms <= now + 1000, "Expired/future request");
      std::int64_t deadline = 0;
      if (goal.source == "wbc") {
        require(goal.execute && assets_.actions.count(goal.action_id) && !assets_.actions.at(goal.action_id).value("json_only", false), "Unknown fixed action");
        require(allowed_.empty() || std::find(allowed_.begin(), allowed_.end(), Json(goal.action_id)) != allowed_.end(), "Fixed action outside enabled selection");
        require(fixed_->service_is_ready(), "Fixed WBC frontend is not running");
      } else {
        require(!assets_.tracker.is_null() && goal.request_id.compare(0, 2, "T_") == 0 && goal.execute && goal.output_path.empty(), "Invalid online text request");
        textRequest(goal);
        deadline = requestDeadline(objectJson(goal.options_json), steadyNs() + generation_ns_);
        require(generation_->service_is_ready(), "Text frontend is not running");
      }
      std::lock_guard<std::mutex> lock(mutex_);
      if (busy_ || contains(recent_, goal.request_id) || !controller_ || steadyNs() - controller_seen_ > 200000000LL ||
          !controller_->ready || goal.timestamp_ms < controller_->ready_since_ms) return rclcpp_action::GoalResponse::REJECT;
      busy_ = true;
      current_id_ = goal.request_id;
      deadline_ = deadline;
      remember(recent_, goal.request_id);
      external_cancel_ = false;
      terminal_.clear(); terminal_detail_.clear(); active_motion_.clear(); active_stamp_ = 0; json_event_ = nullptr;
      return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    } catch (const std::exception& error) {
      RCLCPP_WARN(get_logger(), "Motion rejected: %s", error.what());
      return rclcpp_action::GoalResponse::REJECT;
    }
  }
  bool cancelled(const std::shared_ptr<Handle>& handle) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (handle->is_canceling() || external_cancel_) return true;
    if (!controller_) return false;
    const auto state = objectJson(controller_->payload_json.empty() ? "{}" : controller_->payload_json);
    return state.value("request_id", "") == handle->get_goal()->request_id && state.value("cancelled", false);
  }
  void checkCancelled(const std::shared_ptr<Handle>& handle) {
    require(running(), "Motion gateway stopping");
    if (cancelled(handle)) throw Cancelled("Motion cancelled");
  }
  void stage(const std::shared_ptr<Handle>& handle, const std::string& name, const std::string& detail = "") {
    if (handle && handle->is_active() && running()) {
      if (handle->get_goal()->source == "txt2motion" && name != "cleanup_pending")
        RCLCPP_DEBUG(get_logger(), "Text motion %s stage=%s detail=%s",
                    handle->get_goal()->request_id.c_str(), name.c_str(), detail.c_str());
      auto value = std::make_shared<MotionAction::Feedback>(); value->stage = name; value->detail = detail;
      handle->publish_feedback(value);
    }
  }
  Transaction::Response::SharedPtr call(const Client::SharedPtr& client, const Request& request, std::int64_t deadline = 0) {
    require(client->service_is_ready(), "Required motion service is unavailable");
    auto pending = client->async_send_request(std::make_shared<Request>(request));
    const auto until = deadline ? std::min(deadline, steadyNs() + kGenerationNs) : steadyNs() + kGenerationNs;
    try {
      while (pending.wait_for(std::chrono::milliseconds(5)) != std::future_status::ready) {
        require(running(), "Motion gateway stopping");
        require(steadyNs() < until, "Motion transaction timed out; cleanup must be confirmed");
      }
      require(running(), "Motion gateway stopping");
      if (deadline) checkDeadline(deadline);
      return pending.get();
    } catch (...) { client->remove_pending_request(pending.request_id); throw; }
  }
  Transaction::Response::SharedPtr tx(const Goal& goal, std::uint8_t command, std::int64_t deadline = 0) {
    Request request;
    request.command = command; request.request_id = goal.request_id; request.source = goal.source;
    request.timestamp_ms = goal.timestamp_ms; request.action_id = goal.source == "txt2motion" ? goal.request_id : goal.action_id;
    return call(transaction_, request, deadline);
  }
  void retireGenerator(const Goal& goal, const std::shared_ptr<Handle>& handle) {
    if (goal.source != "txt2motion") return;
    while (running()) {
      try {
        Request cancel; cancel.command = Request::CANCEL; cancel.request_id = goal.request_id;
        const auto response = call(generation_, cancel);
        if (response->success && response->ready) return;
      } catch (const std::exception&) {}
      stage(handle, "cleanup_pending", "Waiting for actual text inference retirement");
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
  }
  bool cleanup(const Goal& goal, const std::shared_ptr<Handle>& handle, const std::string& directory) {
    if (running()) publishCancel(goal.request_id);
    while (running()) {
      try {
        tx(goal, Request::CANCEL);
        const auto response = tx(goal, Request::RELEASE);
        if (response->success && response->ready) {
          removeTemporaryDirectory(directory);
          std::lock_guard<std::mutex> lock(mutex_);
          busy_ = false; current_id_.clear(); controller_ = response; controller_seen_ = steadyNs();
          return true;
        }
      } catch (const std::exception& error) { RCLCPP_WARN(get_logger(), "Motion cleanup pending: %s", error.what()); }
      stage(handle, "cleanup_pending", "Waiting for worker retirement, fresh AMP and file removal");
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    return false;
  }
  void execute(const std::shared_ptr<Handle>& handle) {
    const auto goal = *handle->get_goal();
    std::string status = "failed", detail, directory;
    bool generation_complete = false;
    Client::SharedFuture generating;
    std::int64_t generation_request_id = -1, deadline = 0;
    { std::lock_guard<std::mutex> lock(mutex_); deadline = deadline_; }
    try {
      checkCancelled(handle);
      if (goal.source == "txt2motion") checkDeadline(deadline);
      const auto reserved = tx(goal, Request::RESERVE, deadline);
      require(reserved->success, reserved->message);
      stage(handle, "accepted", "Reserved while AMP controls the robot");
      if (goal.source == "txt2motion") {
        stage(handle, "preparing", "Waiting for head hold and natural arm-swing completion");
        Json initial;
        while (running()) {
          checkCancelled(handle); checkDeadline(deadline);
          std::int64_t capture_stamp = 0;
          {
            std::lock_guard<std::mutex> lock(mutex_);
            if (controller_ && steadyNs() - controller_seen_ <= 1000000000LL) {
              const auto state = objectJson(controller_->payload_json.empty() ? "{}" : controller_->payload_json);
              if (state.value("request_id", "") == goal.request_id && state.value("source", "") == "txt2motion") {
                require(!state.value("cancelled", false), "Controller cancelled text preparation");
                capture_stamp = state.value("text_capture_stamp_ns", std::int64_t{0});
              }
            }
          }
          if (capture_stamp > 0) {
            auto candidate = snapshot();
            if (candidate.at("captured_stamp_ns").get<std::int64_t>() > capture_stamp) {
              initial = std::move(candidate);
              break;
            }
          }
          std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
        require(running() && !initial.is_null(), "Motion gateway stopping before text start capture");
        checkCancelled(handle); checkDeadline(deadline);
        stage(handle, "generating");
        Request request; request.command = Request::GENERATE; request.request_id = goal.request_id;
        request.payload_json = encode({{"request", textRequest(goal)}, {"snapshot", initial}, {"deadline_monotonic_ns", deadline}});
        auto pending = generation_->async_send_request(std::make_shared<Request>(request));
        generation_request_id = pending.request_id;
        generating = pending.future.share();
        while (generating.wait_for(std::chrono::milliseconds(5)) != std::future_status::ready) {
          require(generation_->service_is_ready(), "Text frontend exited during generation");
          snapshot();
          checkCancelled(handle); checkDeadline(deadline);
        }
        checkCancelled(handle); checkDeadline(deadline);
        const auto response = generating.get();
        require(response->success, response->message);
        const auto current = snapshot();
        for (std::size_t j = 0; j < 21; ++j)
          require(std::abs(finiteNumber(current.at("joint_positions")[j]) - finiteNumber(initial.at("joint_positions")[j])) <= 0.15,
                  "Robot moved during generation; result discarded");
        require(response->payload_json.size() <= kJsonLimit, "Generated JSON exceeds transport limit");
        auto trajectory = Json::parse(response->payload_json);
        if (trajectory.is_null()) { checkDeadline(deadline); status = "no_motion"; generation_complete = true; }
        else {
          require(trajectory.at("request_id") == goal.request_id, "Generated trajectory belongs to another request");
          auto document = makeDocument(std::move(trajectory), initial, assets_);
          const auto encoded = encode(document);
          checkCancelled(handle); checkDeadline(deadline);
          directory = temporaryDirectory();
          saveDocument(directory, encoded, deadline);
          checkCancelled(handle); checkDeadline(deadline);
          generation_complete = true;
          stage(handle, "generated");
          String message; message.data = encoded; json_pub_->publish(message);
          // Admission/preparation is a separate existing budget, not another generation budget.
          const auto ready_until = steadyNs() + 5000000000LL;
          while (true) {
            checkCancelled(handle);
            Json event;
            { std::lock_guard<std::mutex> lock(mutex_); event = json_event_; }
            if (!event.is_null()) { require(event.at("state") == "ready", "JSON admission rejected: " + event.value("detail", "")); break; }
            require(steadyNs() < ready_until, "JSON admission timed out");
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
          }
        }
      }
      if (status != "no_motion") {
        const auto frontend = goal.source == "txt2motion" ? generation_ : fixed_;
        require(frontend->service_is_ready(), "Motion frontend exited before preparation");
        checkCancelled(handle);
        {
          std::lock_guard<std::mutex> lock(mutex_);
          active_motion_ = goal.source == "txt2motion" ? goal.request_id : goal.action_id;
          active_stamp_ = goal.timestamp_ms; terminal_.clear(); terminal_detail_.clear();
        }
        stage(handle, "preparing");
        const auto started = tx(goal, Request::EXECUTE);
        require(started->success, started->message);
        stage(handle, "started");
        while (running()) {
          checkCancelled(handle);
          require(frontend->service_is_ready(), "Motion frontend exited during tracking");
          {
            std::lock_guard<std::mutex> lock(mutex_);
            require(steadyNs() - controller_seen_ <= 1000000000LL, "Controller heartbeat lost during tracking");
            if (!terminal_.empty()) { status = terminal_; detail = terminal_detail_; break; }
          }
          std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        require(running(), "Motion gateway stopping");
      }
    } catch (const Cancelled& error) { status = "cancelled"; detail = error.what(); }
    catch (const std::exception& error) {
      status = "failed"; detail = error.what();
      if (goal.source == "txt2motion" && !generation_complete && steadyNs() >= deadline) {
        detail = "Text generation exceeded its 3 second deadline";
        stage(handle, "generation_timeout", detail);
      }
      RCLCPP_WARN(get_logger(), "Motion %s failed: %s", goal.request_id.c_str(), detail.c_str());
    }
    stage(handle, "returning", "Returning control to AMP and confirming cleanup");
    // Even a completed error response may own an unconfirmed device operation.
    retireGenerator(goal, handle);
    if (generation_request_id >= 0) generation_->remove_pending_request(generation_request_id);
    if (!cleanup(goal, handle, directory)) { status = "failed"; detail = "Gateway stopped before cleanup was confirmed"; }
    if (!running() || !handle->is_active()) return;
    auto result = std::make_shared<MotionAction::Result>();
    result->success = status == "completed" || status == "no_motion";
    result->status = status; result->detail = detail;
    if (handle->is_canceling()) { result->success = false; result->status = "cancelled"; handle->canceled(result); }
    else if (result->success) handle->succeed(result);
    else handle->abort(result);
  }

  Assets assets_;
  Limits limits_;
  Json allowed_, json_event_;
  std::string mapping_, current_id_, active_motion_, terminal_, terminal_detail_;
  std::int64_t generation_ns_{kGenerationNs}, deadline_{0}, controller_seen_{0}, active_stamp_{0};
  std::int64_t poll_started_{0}, poll_request_id_{0};
  std::uint64_t poll_epoch_{0};
  bool busy_{false}, external_cancel_{false}, poll_pending_{false};
  std::atomic<bool> stopping_{false};
  std::mutex mutex_, worker_mutex_;
  std::thread worker_;
  std::deque<std::string> recent_;
  Observation::ConstSharedPtr observation_;
  Transaction::Response::SharedPtr controller_;
  Client::SharedPtr transaction_, generation_, fixed_;
  rclcpp::CallbackGroup::SharedPtr group_, poll_group_;
  rclcpp::TimerBase::SharedPtr poll_timer_;
  rclcpp_action::Server<MotionAction>::SharedPtr server_;
  rclcpp::Publisher<String>::SharedPtr json_pub_, cancel_pub_, status_pub_;
  rclcpp::Subscription<Observation>::SharedPtr observation_sub_;
  rclcpp::Subscription<String>::SharedPtr json_sub_, terminal_sub_, cancel_sub_;
};
}  // namespace
std::shared_ptr<rclcpp::Node> makeMotionBackend() { return std::make_shared<MotionBackend>(); }
}  // namespace motion
}  // namespace legged
