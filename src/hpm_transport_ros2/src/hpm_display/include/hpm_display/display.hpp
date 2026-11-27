// Copyright (c) 2026 HPMicro
// SPDX-License-Identifier: BSD-3-Clause
#pragma once
#include "hpm_display/protocol.hpp"
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <luvbot_hardware_interfaces/msg/display_device_state.hpp>
#include <luvbot_hardware_interfaces/msg/display_request.hpp>
#include <luvbot_hardware_interfaces/msg/display_result.hpp>
#include <atomic>
#include <condition_variable>
#include <deque>
#include <map>
#include <mutex>
#include <thread>

namespace hpmDisplay {
class HpmDisplay {
public:
  static std::unique_ptr<HpmDisplay> try_create(rclcpp::Node &node) noexcept;
  ~HpmDisplay();
private:
  explicit HpmDisplay(rclcpp::Node &node);
  struct Command { uint16_t key=0; uint8_t type=DISPLAY_SET_STATE; Bytes value; uint32_t revision=0; };
  struct Cached { Command command; bool dirty=true; };
  struct Pending { Frame frame; Command command; unsigned attempts=0; int64_t deadline=0; bool hello=false; };
  void subscribe_topics();
  template<class Message,class Convert>
  void input(const std::string &topic,uint16_t key,Convert convert,bool event=false,
             const rclcpp::QoS &qos=rclcpp::QoS(10).reliable()) {
    rclcpp::SubscriptionOptions options;
    options.callback_group = callbacks_;
    subscriptions_.push_back(node_.create_subscription<Message>(topic,qos,
      [this,key,convert,event](typename Message::ConstSharedPtr m) {
        if (!running_) return;
        try { submit(key,convert(*m),event); }
        catch(const std::invalid_argument &e) {
          RCLCPP_ERROR(logger_,"%s: %s",display_function(key),e.what());
        }
        catch(const std::exception &e) { fail(e.what()); }
        catch(...) { fail("unknown display input exception"); }
      },options));
  }
  void submit(uint16_t key,Bytes value,bool event);
  void result(uint16_t action,const luvbot_hardware_interfaces::msg::DisplayResult &message);
  void worker() noexcept;
  void run_link();
  bool stop(const char *reason = nullptr);
  void fail(const char *reason) noexcept;
  void close_link();
  void connection(const std::string &state);
  bool wait_ms(int milliseconds);
  int open_socket();
  void write_all(int fd,const Bytes &data);
  void route(int fd,const Frame &frame,Pending *pending,bool &complete);
  rclcpp::Node &node_;
  rclcpp::Logger logger_;
  rclcpp::CallbackGroup::SharedPtr callbacks_;
  std::string board_ip_;
  int64_t port_,reconnect_ms_,ack_timeout_ms_,keepalive_ms_,hello_timeout_ms_,max_retries_;
  uint32_t boot_,revision_=0,sequence_=0;
  std::atomic<bool> running_{true};
  std::mutex mutex_;
  std::condition_variable changed_;
  bool maintenance_=false,connected_=false;
  char failure_reason_[256]{};
  int socket_=-1;
  std::map<uint16_t,Cached> states_;
  std::deque<Command> events_;
  std::vector<rclcpp::SubscriptionBase::SharedPtr> subscriptions_;
  std::map<uint16_t,rclcpp::Publisher<luvbot_hardware_interfaces::msg::DisplayRequest>::SharedPtr> requests_;
  rclcpp::Publisher<luvbot_hardware_interfaces::msg::DisplayDeviceState>::SharedPtr panel_state_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr connection_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr maintenance_service_;
  std::string connection_state_;
  struct Seen { uint32_t session,sequence; uint8_t type; Bytes payload,reply; };
  std::deque<Seen> seen_;
  struct Request { uint32_t boot,event; uint16_t action,value; };
  std::deque<Request> known_requests_;
  std::thread thread_;
};
}  // namespace hpmDisplay
