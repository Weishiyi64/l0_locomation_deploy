#pragma once
#include <chrono>
#include <deque>
#include <functional>
#include <future>
#include <mutex>
#include <unordered_map>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include "map_store.hpp"

namespace cloud {
class MapBridge {
  using Frame = luvbot::v1::Envelope;
  using Json = nlohmann::json;
  using Clock = std::chrono::steady_clock;
  using Reply = std::function<void(const Frame&, int, const std::string&)>;
  using Publish = std::function<bool(const Frame&, int*)>;
  struct Task {
    std::string start_id, cloud_id;
    int slot{0};
    struct stat previous_file{};
  };
  struct Pending {
    Frame frame;
    Clock::time_point deadline;
    Task task;
    bool timed_out{false}, artifact_ready{false};
  };
  struct Work {
    Frame frame;
    Task task;
    bool artifact{false};
    std::string bytes, error;
  };
 public:
  MapBridge(rclcpp::Node& node, Reply reply, Publish publish, std::function<Frame()> envelope)
      : node_(node), reply_(std::move(reply)), publish_(std::move(publish)), envelope_(std::move(envelope)) {
    const auto qos = rclcpp::QoS(10).reliable().durability_volatile();
    commands_ = node.create_publisher<std_msgs::msg::String>("/algorithm/brain/map_command", qos);
    uploads_ = node.create_publisher<std_msgs::msg::String>("/algorithm/brain/map_upload_result", qos);
    replies_ = node.create_subscription<std_msgs::msg::String>("/algorithm/brain/map_reply", qos,
      [this](std_msgs::msg::String::ConstSharedPtr message) {
        try {
          if (message->data.size() > 65536) throw std::invalid_argument("reply too large");
          const auto value = Json::parse(message->data);
          const auto it = pending_.find(value.at("msg_id").get<std::string>());
          if (it == pending_.end()) return;
          if (value.at("cmd") != it->second.frame.command().cmd() || !value.at("code").is_number_integer() ||
              value.at("code") < 0 || value.at("code") > 65535 || !value.at("message").is_string())
            throw std::invalid_argument("invalid map reply fields");
          const int code = value.at("code").get<int>();
          const bool start = it->second.frame.command().cmd() == "map.start";
          if (code == 0 && start) {
            // A late acceptance cannot be undone: retain the actual map context, never start another task here.
            active_ = it->second.task;
            attempted_ = submitted_ = upload_finished_ = upload_failed_ = false;
            fresh_rules_ = fresh_names_ = true;
            artifact_frame_.Clear();
            {
              std::lock_guard<std::mutex> lock(ack_mutex_);
              upload_mid_ = 0;
              upload_ack_ = false;
            }
            if (it->second.artifact_ready) acceptArtifact(active_.start_id, active_.slot);
          }
          reply_(it->second.frame, code, value.at("message").get<std::string>());
          pending_.erase(it);
        } catch (const std::exception&) {
          RCLCPP_WARN(node_.get_logger(), "[MAP] ignored malformed brain reply");
        }
      });
    artifacts_ = node.create_subscription<std_msgs::msg::String>("/algorithm/brain/map_artifact_ready", qos,
      [this](std_msgs::msg::String::ConstSharedPtr message) {
        try {
          if (message->data.size() > 1024) throw std::invalid_argument("artifact notification too large");
          const auto value = Json::parse(message->data);
          const auto& index = value.at("index");
          if (!index.is_number_integer() || index < 1 || index > 2)
            throw std::invalid_argument("invalid artifact index");
          const int map = index.get<int>();
          const auto start_id = value.at("start_msg_id").get<std::string>();
          const auto it = pending_.find(start_id);
          if (it != pending_.end() && it->second.task.start_id == start_id && it->second.task.slot == map) {
            // The artifact notification and start reply can arrive in either order.
            it->second.artifact_ready = true;
            return;
          }
          acceptArtifact(start_id, map);
        } catch (const std::exception&) {
          RCLCPP_WARN(node_.get_logger(), "[MAP] ignored malformed artifact notification");
        }
      });
  }

  static bool handles(const std::string& command) {
    return command == "map.start" || command == "map.zone.set" || command == "batch_set_room_name";
  }

  void command(const Frame& frame) {
    const auto previous = pending_.find(frame.msg_id());
    if (previous != pending_.end()) {
      if (previous->second.frame.command().SerializeAsString() != frame.command().SerializeAsString())
        reply_(frame, 1003, "msg_id reused with different content");
      return;
    }
    Params params;
    try {
      params = parameters(frame.command());
    } catch (const std::invalid_argument&) {
      reply_(frame, 1003, "duplicate map command parameter key");
      return;
    }
    const auto map_id = params.find("mapId");
    if (map_id == params.end() || map_id->second.empty()) {
      reply_(frame, 1003, "missing or empty params.mapId");
      return;
    }
    const bool start = frame.command().cmd() == "map.start";
    Task task;
    if (start) {
      try {
        task.slot = maps::slot(required(params, "mapIndex"));
        maps::path(map_id->second);
      } catch (const std::invalid_argument& error) {
        reply_(frame, 1003, error.what());
        return;
      }
      task.start_id = frame.msg_id();
      task.cloud_id = map_id->second;
    } else {
      if (map_id->second != active_.cloud_id || !submitted_ || upload_failed_) {
        reply_(frame, 1003, "not awaiting server metadata for this map");
        return;
      }
      task = active_;
    }
    try {
      if (pending_.size() >= 16 || jobs_.size() >= 8) {
        reply_(frame, 1006, "map command queue full"); return;
      }
      if (commands_->get_subscription_count() == 0) {
        reply_(frame, 1006, "brain map_command subscriber unavailable"); return;
      }
      Pending pending;
      pending.frame = frame;
      pending.deadline = Clock::now() + std::chrono::seconds(60);
      if (start) task.previous_file = maps::snapshot(maps::path(task.cloud_id));
      pending.task = task;
      pending_.emplace(frame.msg_id(), std::move(pending));
      if (start) forward(frame);
      else {
        Work work;
        work.frame = frame;
        work.task = task;
        jobs_.push_back(std::move(work));
      }
    } catch (const std::exception&) {
      pending_.erase(frame.msg_id());
      reply_(frame, 1006, "could not dispatch map command");
    }
  }

  // Called on the MQTT network thread. ROS publication stays on tick's executor thread.
  void acknowledge(int mid) {
    std::lock_guard<std::mutex> lock(ack_mutex_);
    if (mid == upload_mid_) upload_ack_ = true;
  }

  void tick() {
    const auto now = Clock::now();
    for (auto& entry : pending_) {
      if (!entry.second.timed_out && now >= entry.second.deadline) {
        reply_(entry.second.frame, 1006, "brain map reply timeout; outcome unconfirmed");
        entry.second.timed_out = true; // Keep identity until a late result; timeout does not cancel business.
      }
    }
    if (worker_.valid() && worker_.wait_for(std::chrono::seconds(0)) == std::future_status::ready) {
      const auto work = worker_.get();
      if (work.artifact) {
        if (work.task.start_id != active_.start_id) {
          RCLCPP_WARN(node_.get_logger(), "[MAP] ignored artifact from superseded task");
        } else if (!work.error.empty()) {
          uploadResult(false, work.error);
        } else {
          artifact_frame_ = envelope_();
          auto* event = artifact_frame_.mutable_event_report();
          event->set_event("map.build.done");
          event->set_level(1);
          event->set_detail(Json({{"mapId", work.task.cloud_id}, {"mapData", work.bytes}}).dump());
          upload_deadline_ = now + std::chrono::seconds(10);
          upload_finished_ = upload_failed_ = false;
          RCLCPP_INFO(node_.get_logger(), "[MAP] validated %s; event ready bytes=%zu",
                      maps::path(work.task.cloud_id).c_str(), work.bytes.size());
        }
      } else {
        const auto it = pending_.find(work.frame.msg_id());
        if (it != pending_.end() && !it->second.timed_out) {
          if (work.task.start_id != active_.start_id) {
            reply_(work.frame, 1003, "map task superseded");
            pending_.erase(it);
          } else if (work.error.empty()) {
            if (work.frame.command().cmd() == "map.zone.set") fresh_rules_ = false;
            else fresh_names_ = false;
            forward(work.frame);
          } else {
            reply_(work.frame, 1003, work.error);
            pending_.erase(it);
          }
        }
      }
    }
    if (!worker_.valid() && !jobs_.empty()) {
      auto work = std::move(jobs_.front());
      jobs_.pop_front();
      if (work.task.start_id != active_.start_id) {
        if (!work.artifact && pending_.erase(work.frame.msg_id()))
          reply_(work.frame, 1003, "map task superseded");
        return;
      }
      const bool fresh = work.frame.command().cmd() == "map.zone.set" ? fresh_rules_ : fresh_names_;
      try {
        worker_ = std::async(std::launch::async, [work, fresh]() mutable {
          try {
            if (work.artifact) {
              const auto file = maps::path(work.task.cloud_id);
              try {
                work.bytes = maps::read(file, &work.task.previous_file);
              } catch (const std::runtime_error& error) {
                work.error = file + ": " + error.what();
                return work;
              }
              maps::validate(work.bytes, work.task.slot, work.task.cloud_id);
            } else maps::saveMetadata(work.frame.command().cmd(), parameters(work.frame.command()), work.task.slot, fresh);
          } catch (const std::exception&) {
            // Never include parser exceptions: they can contain the complete base64 image.
            work.error = work.artifact ? "map file validation/read failed" : "map metadata validation/save failed";
          }
          return work;
        });
      } catch (const std::exception&) {
        if (work.artifact) uploadResult(false, "cannot start map file worker");
        else {
          reply_(work.frame, 1006, "cannot start map file worker");
          pending_.erase(work.frame.msg_id());
        }
      }
    }
    if (!artifact_frame_.has_event_report()) return;
    bool acknowledged;
    {
      std::lock_guard<std::mutex> lock(ack_mutex_);
      acknowledged = upload_ack_;
    }
    if (acknowledged && !upload_finished_) {
      uploadResult(true, "broker PUBACK received; server business processing not confirmed");
      artifact_frame_.Clear();
    } else if (!upload_finished_ && now >= upload_deadline_) {
      uploadResult(false, "map publish/PUBACK timeout; outcome unconfirmed, no automatic retry");
    } else if (!submitted_ && !upload_finished_) {
      // Serialize publication and MID registration against a fast network PUBACK.
      std::lock_guard<std::mutex> lock(ack_mutex_);
      submitted_ = publish_(artifact_frame_, &upload_mid_);
    }
  }

 private:
  void acceptArtifact(const std::string& start_id, int map) {
    if (start_id.empty() || start_id != active_.start_id || map != active_.slot || attempted_) return;
    attempted_ = true;
    Work work;
    work.task = active_;
    work.artifact = true;
    jobs_.push_back(std::move(work));
  }
  void forward(const Frame& frame) {
    const auto params = parameters(frame.command());
    std_msgs::msg::String message;
    message.data = Json({{"msg_id", frame.msg_id()}, {"cmd", frame.command().cmd()},
                         {"params", params}}).dump();
    commands_->publish(message);
    RCLCPP_INFO(node_.get_logger(), "[MAP] forwarded cmd=%s mapId=%.128s", frame.command().cmd().c_str(),
                required(params, "mapId").c_str());
  }
  void uploadResult(bool success, const std::string& detail) {
    upload_finished_ = true;
    upload_failed_ = !success;
    std_msgs::msg::String message;
    message.data = Json({{"index", active_.slot}, {"start_msg_id", active_.start_id},
                         {"success", success}, {"message", detail}}).dump();
    uploads_->publish(message);
    RCLCPP_INFO(node_.get_logger(), "[MAP] upload result index=%d success=%d: %s", active_.slot, success, detail.c_str());
  }
  rclcpp::Node& node_;
  Reply reply_;
  Publish publish_;
  std::function<Frame()> envelope_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr commands_, uploads_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr replies_, artifacts_;
  std::unordered_map<std::string, Pending> pending_;
  std::deque<Work> jobs_;
  std::future<Work> worker_;
  Task active_;
  bool attempted_{false}, submitted_{false}, upload_finished_{false}, upload_failed_{false};
  bool fresh_rules_{true}, fresh_names_{true};
  Frame artifact_frame_;
  Clock::time_point upload_deadline_{};
  std::mutex ack_mutex_;
  int upload_mid_{0};
  bool upload_ack_{false};
};
}  // namespace cloud
