#include "MotionRuntime.h"
#include "TextMotionPipeline.h"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <std_msgs/msg/string.hpp>
#include <functional>
#include <limits>
#include <mutex>

namespace legged {
namespace motion {
Json textRequest(const Goal& goal) {
  const auto options = objectJson(goal.options_json.empty() ? "{}" : goal.options_json);
  const auto seed = requestSeed(options);
  validateText(goal.text);
  return {{"seed", seed}, {"request_id", goal.request_id}, {"text", goal.text}, {"mode", "text"}};
}
namespace {
using String = std_msgs::msg::String;
using ClientHandle = rclcpp_action::ClientGoalHandle<MotionAction>;

class MotionFrontend final : public rclcpp::Node {
 public:
  explicit MotionFrontend(const std::string& role)
      : Node(role == "wbc" ? "wbc_frontend" : "txt2motion_frontend"), role_(role) {
    group_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    assets_ = loadAssets(declare_parameter<std::string>("deployment", ""));
    const auto mapping = declare_parameter<std::string>("mapping", "");
    const auto root = declare_parameter<std::string>("model_root",
        ament_index_cpp::get_package_share_directory("legged_deploy") + "/policy/l0_v5/txt2motion");
    client_ = rclcpp_action::create_client<MotionAction>(get_node_base_interface(), get_node_graph_interface(),
        get_node_logging_interface(), get_node_waitables_interface(), kAction, group_);
    feedback_pub_ = create_publisher<String>("/system/state_machine/wbc_action_fdb", 10);
    rclcpp::SubscriptionOptions options;
    options.callback_group = group_;
    commands_ = create_subscription<String>(role == "wbc" ? "/algorithm/brain/wbc_action_cmd" : "/algorithm/brain/txt2motion_cmd",
        rclcpp::QoS(1), std::bind(&MotionFrontend::forward, this, std::placeholders::_1), options);
    if (role == "txt2motion") {
      require(!assets_.tracker.is_null(), "txt2motion requires text tracking parameters");
      limits_ = loadLimits(mapping, assets_.names);
      pipeline_.reset(new TextMotionPipeline(root, assets_.names, limits_));
    }
    service_ = create_service<Transaction>(role == "wbc" ? kFixedFrontend : kGeneration,
        std::bind(&MotionFrontend::generate, this, std::placeholders::_1, std::placeholders::_2),
        rmw_qos_profile_services_default, group_);
    RCLCPP_INFO(get_logger(), "%s C++ frontend ready; requests use shared AMP admission", role.c_str());
  }

 private:
  void feedback(const Goal& goal, const std::string& status, const std::string& detail, const std::string& output = "") {
    String message;
    message.data = encode({{"timestamp_ms", goal.timestamp_ms}, {"action_id", goal.action_id.empty() ? goal.request_id : goal.action_id},
        {"status", status}, {"detail", detail}, {"output_path", output}});
    feedback_pub_->publish(message);
  }
  void forward(const String::ConstSharedPtr message) {
    const auto received = steadyNs();
    try {
      auto data = objectJson(message->data);
      Goal goal;
      goal.source = role_;
      goal.timestamp_ms = integer(data.at("timestamp_ms"), 1, std::numeric_limits<std::int64_t>::max());
      goal.execute = true;
      if (role_ == "txt2motion") {
        require(data.size() == 3 && data.count("timestamp_ms") && data.count("text") && data.count("seed"),
                "Text commands require exactly timestamp_ms, text and seed");
        goal.text = data.at("text").get<std::string>();
        goal.request_id = newId("T_");
        goal.options_json = encode({{"seed", data.at("seed")}, {"deadline_monotonic_ns", received + kGenerationNs}});
        textRequest(goal);
        checkDeadline(received + kGenerationNs);
      } else {
        goal.request_id = data.value("request_id", newId("W_"));
        goal.action_id = data.at("action_id").get<std::string>();
      }
      require(client_->action_server_is_ready(), "Shared motion backend is unavailable");
      rclcpp_action::Client<MotionAction>::SendGoalOptions options;
      options.goal_response_callback = [this, goal](ClientHandle::SharedPtr handle) {
        if (!handle) {
          String rejected;
          rejected.data = encode({{"timestamp_ms", goal.timestamp_ms},
              {"action_id", goal.action_id.empty() ? goal.request_id : goal.action_id},
              {"status", "rejected"}, {"detail", "Shared motion gateway busy/unavailable"}});
          feedback_pub_->publish(rejected);
        }
      };
      options.result_callback = [this, goal](const ClientHandle::WrappedResult& wrapped) {
        if (!wrapped.result) { feedback(goal, "failed", "Motion backend returned no result"); return; }
        const auto& result = *wrapped.result;
        if (role_ == "txt2motion" || result.status != "completed") feedback(goal, result.status, result.detail, result.output_path);
      };
      client_->async_send_goal(goal, options);
    } catch (const std::exception& error) {
      RCLCPP_WARN(get_logger(), "Frontend request rejected: %s", error.what());
    }
  }
  void checkpoint(const std::string& id, std::int64_t deadline) {
    require(rclcpp::ok(), "Text frontend shutting down");
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (contains(cancelled_, id)) throw Cancelled("Text generation cancelled");
    }
    checkDeadline(deadline);
  }
  void generate(const Transaction::Request::SharedPtr request, Transaction::Response::SharedPtr response) {
    using Request = Transaction::Request;
    if (role_ == "wbc") {
      response->success = request->command == Request::STATUS;
      response->ready = response->success;
      return;
    }
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (request->command == Request::STATUS || request->command == Request::CANCEL) {
        if (request->command == Request::CANCEL) remember(cancelled_, request->request_id);
        response->success = true;
        response->ready = !generating_ && pipeline_->retired();
        response->payload_json = encode({{"request_id", generator_id_}});
        return;
      }
      if (request->command != Request::GENERATE || generating_ || !pipeline_->retired() || contains(cancelled_, request->request_id)) {
        response->message = "Text generator busy or retirement unconfirmed; request rejected without queueing";
        return;
      }
      generating_ = true;
      generator_id_ = request->request_id;
    }
    try {
      const auto data = objectJson(request->payload_json);
      require(validId(request->request_id) && request->request_id.compare(0, 2, "T_") == 0 &&
              data.at("request").at("request_id") == request->request_id, "Generator request identity mismatch");
      const auto deadline = requestDeadline({{"deadline_monotonic_ns", data.at("deadline_monotonic_ns")}}, steadyNs() + kGenerationNs);
      const auto& generation = data.at("request");
      require(generation.is_object() && generation.size() == 4 && generation.at("mode") == "text", "Only online text generation is supported");
      validateText(generation.at("text").get<std::string>());
      integer(generation.at("seed"), 1, 4294967295LL);
      const auto snapshot = checkSnapshot(data.at("snapshot"), assets_.names, limits_);
      auto result = pipeline_->generate(generation, snapshot, deadline,
          [this, request, deadline] { checkpoint(request->request_id, deadline); });
      checkpoint(request->request_id, deadline);
      response->payload_json = encode(result);
      checkpoint(request->request_id, deadline);
      response->success = true;
    } catch (const std::exception& error) {
      response->message = error.what();
    }
    std::lock_guard<std::mutex> lock(mutex_);
    // Cancellation may race with JSON serialization; a tombstoned ID never succeeds.
    if (contains(cancelled_, request->request_id)) { response->success = false; response->payload_json.clear(); response->message = "Text generation cancelled"; }
    generating_ = false;
    generator_id_.clear();
  }

  std::string role_, generator_id_;
  Assets assets_;
  Limits limits_;
  std::mutex mutex_;
  bool generating_{false};
  std::deque<std::string> cancelled_;
  std::unique_ptr<TextMotionPipeline> pipeline_;
  rclcpp::CallbackGroup::SharedPtr group_;
  rclcpp_action::Client<MotionAction>::SharedPtr client_;
  rclcpp::Publisher<String>::SharedPtr feedback_pub_;
  rclcpp::Subscription<String>::SharedPtr commands_;
  rclcpp::Service<Transaction>::SharedPtr service_;
};
}  // namespace
std::shared_ptr<rclcpp::Node> makeMotionFrontend(const std::string& role) {
  return std::make_shared<MotionFrontend>(role);
}
}  // namespace motion
}  // namespace legged
