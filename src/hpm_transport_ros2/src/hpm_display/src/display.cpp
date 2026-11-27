// Copyright (c) 2026 HPMicro
// SPDX-License-Identifier: BSD-3-Clause
#include "hpm_display/display.hpp"
#include <luvbot_hardware_interfaces/msg/display_battery_state.hpp>
#include <luvbot_hardware_interfaces/msg/display_wifi_state.hpp>
#include <luvbot_hardware_interfaces/msg/robot_provision_state.hpp>
#include <luvbot_hardware_interfaces/msg/display_fault_state.hpp>
#include <luvbot_hardware_interfaces/msg/robot_upgrade_state.hpp>
#include <luvbot_hardware_interfaces/msg/display_identity_state.hpp>
#include <std_msgs/msg/empty.hpp>
#include <sensor_msgs/msg/illuminance.hpp>
#include <luvbot_hardware_interfaces/msg/display_voice_state.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <std_msgs/msg/bool.hpp>
#include <arpa/inet.h>
#include <fcntl.h>
#include <netinet/tcp.h>
#include <poll.h>
#include <pthread.h>
#include <sys/socket.h>
#include <unistd.h>
#include <algorithm>
#include <cinttypes>
#include <chrono>
#include <cstdio>
#include <optional>
#include <random>
#include <stdexcept>
#include <system_error>
#include <cstring>
#include <cerrno>

namespace hpmDisplay {
namespace msg=luvbot_hardware_interfaces::msg;
namespace {
class LinkError : public std::runtime_error {
public:
  using std::runtime_error::runtime_error;
};
}
static int64_t milliseconds()
{ return std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now().time_since_epoch()).count(); }
static uint16_t action_key(uint16_t action)
{
  switch(action) {
  case DISPLAY_BT_REQUEST:return DISPLAY_BLUETOOTH;
  case DISPLAY_MODE_REQUEST:return DISPLAY_MODE;
  case DISPLAY_VOLUME_REQUEST:return DISPLAY_VOLUME;
  case DISPLAY_MUTE_REQUEST:return DISPLAY_MUTE;
  case DISPLAY_FAULT_ACK:return DISPLAY_FAULT;
  case DISPLAY_PROVISION_REQUEST:return DISPLAY_PROVISION;
  default:return 0;
  }
}
std::unique_ptr<HpmDisplay> HpmDisplay::try_create(rclcpp::Node &node) noexcept
{
  try {
    rcl_interfaces::msg::ParameterDescriptor startup;
    startup.read_only = true;
    if (!node.declare_parameter<bool>("launch_display", false, startup)) {
      return nullptr;
    }
    return std::unique_ptr<HpmDisplay>(new HpmDisplay(node));
  } catch (const std::exception &e) {
    RCLCPP_ERROR(node.get_logger(), "Display disabled at startup: %s", e.what());
  } catch (...) {
    RCLCPP_ERROR(node.get_logger(), "Display disabled at startup: unknown exception");
  }
  return nullptr;
}

HpmDisplay::HpmDisplay(rclcpp::Node &node)
: node_(node), logger_(node.get_logger().get_child("display"))
{
  rcl_interfaces::msg::ParameterDescriptor startup;
  startup.read_only = true;
  const auto config_error = node_.declare_parameter<std::string>("display_config_error", "", startup);
  if (!config_error.empty()) throw std::invalid_argument(config_error);
  board_ip_ = node_.get_parameter("board_ip").as_string();
  reconnect_ms_ = node_.get_parameter("reconnect_ms").as_int();
  port_ = node_.declare_parameter<int64_t>("display_port", 5002, startup);
  ack_timeout_ms_ = node_.declare_parameter<int64_t>("ack_timeout_ms", 2000, startup);
  max_retries_ = node_.declare_parameter<int64_t>("max_retries", 3, startup);
  keepalive_ms_ = node_.declare_parameter<int64_t>("keepalive_ms", 1000, startup);
  hello_timeout_ms_ = node_.declare_parameter<int64_t>("hello_timeout_ms", 500, startup);
  if (port_ == node_.get_parameter("port").as_int()) {
    throw std::invalid_argument("display_port must differ from port");
  }
  in_addr address{};
  if(inet_pton(AF_INET,board_ip_.c_str(),&address)!=1 || port_<1 || port_>65535 || reconnect_ms_<1 ||
     ack_timeout_ms_<10 || max_retries_<0 || max_retries_>10 || keepalive_ms_<100 || keepalive_ms_>=60000 ||
     hello_timeout_ms_<20 || hello_timeout_ms_>=60000)
    throw std::invalid_argument("invalid display address, port, reconnect interval, timeout or retry count");
  boot_=std::random_device{}()|1U;
  callbacks_ = node_.create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  rclcpp::SubscriptionOptions options;
  options.callback_group = callbacks_;
  connection_=node_.create_publisher<std_msgs::msg::String>("/display/connection/state",rclcpp::QoS(1).reliable().transient_local());
  for(auto action:{DISPLAY_BT_REQUEST,DISPLAY_MODE_REQUEST,DISPLAY_VOLUME_REQUEST,DISPLAY_MUTE_REQUEST,DISPLAY_FAULT_ACK,DISPLAY_PROVISION_REQUEST}) {
    auto name=std::string(display_function(action_key(action)));
    requests_[action]=node_.create_publisher<msg::DisplayRequest>("/display/"+name+(action==DISPLAY_FAULT_ACK?"/acknowledge":"/request"),rclcpp::QoS(16).reliable());
    subscriptions_.push_back(node_.create_subscription<msg::DisplayResult>("/display/"+name+"/result",rclcpp::QoS(16).reliable(),
      [this,action](msg::DisplayResult::ConstSharedPtr m) {
        try { result(action,*m); }
        catch (const std::exception &e) { fail(e.what()); }
        catch (...) { fail("unknown display result exception"); }
      },options));
  }
  panel_state_=node_.create_publisher<msg::DisplayDeviceState>("/display/panel/state",
    rclcpp::QoS(5).reliable().transient_local());
  subscribe_topics();
  maintenance_service_=node_.create_service<std_srvs::srv::SetBool>("/display/upgrade/maintenance",
    [this](std_srvs::srv::SetBool::Request::SharedPtr request,std_srvs::srv::SetBool::Response::SharedPtr response) {
      try {
        std::unique_lock<std::mutex> lock(mutex_);
        maintenance_ = request->data;
        if (maintenance_) {
          connected_ = false;
          events_.clear();
          if (socket_ >= 0) shutdown(socket_, SHUT_RDWR);
          changed_.notify_all();
          response->success = changed_.wait_for(lock, std::chrono::seconds(2), [this]{return socket_ < 0;});
          response->message = response->success ? "display connection released; OTA may connect" :
            "display closing; maintenance remains enabled, retry until released";
        } else {
          changed_.notify_all();
          response->success = running_;
          response->message = running_ ? "display reconnect enabled" :
            (failure_reason_[0] ? failure_reason_ : "display stopped");
        }
      } catch (const std::exception &e) {
        response->success = false;
        fail(e.what());
      } catch (...) {
        response->success = false;
        fail("unknown display maintenance exception");
      }
    }, rmw_qos_profile_services_default, callbacks_);
  thread_=std::thread(&HpmDisplay::worker,this);
}
HpmDisplay::~HpmDisplay()
{
  stop();
  if(thread_.joinable()) thread_.join();
}

bool HpmDisplay::stop(const char *reason)
{
  std::lock_guard<std::mutex> lock(mutex_);
  const bool was_running = running_.exchange(false);
  if (was_running && reason) {
    std::snprintf(failure_reason_, sizeof(failure_reason_), "%s", reason);
  }
  connected_ = false;
  if (socket_ >= 0) shutdown(socket_, SHUT_RDWR);
  changed_.notify_all();
  return was_running;
}

void HpmDisplay::fail(const char *reason) noexcept
{
  if (stop(reason) && rclcpp::ok(node_.get_node_base_interface()->get_context())) {
    RCLCPP_ERROR(logger_, "Display stopped: %s", reason);
  }
}

void HpmDisplay::close_link()
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (socket_ >= 0) {
    close(socket_);
    socket_ = -1;
  }
  connected_ = false;
  events_.clear();
  known_requests_.clear();
  changed_.notify_all();
}
void HpmDisplay::subscribe_topics()
{
  input<msg::DisplayBatteryState>("/robot/battery/state",DISPLAY_BATTERY,[](const auto &m){ Bytes b={m.percent,m.charge}; put16(b,m.flags); return b; });
  input<sensor_msgs::msg::Illuminance>("/robot/ambient_light/illuminance",DISPLAY_AMBIENT_LIGHT,[](const auto &m){
    Bytes b(8);
    display_put_lux(b.data(),m.illuminance);
    return b;
  },false,rclcpp::SensorDataQoS().keep_last(1));
  input<msg::DisplayWifiState>("/robot/wifi/state",DISPLAY_WIFI,[](const auto &m){ return Bytes{m.state,m.signal}; });
  for(const auto &entry:std::vector<std::pair<uint16_t,std::string>>{{DISPLAY_BLUETOOTH,"bluetooth/state"},{DISPLAY_VOLUME,"volume/state"},{DISPLAY_MODE,"mode/state"},{DISPLAY_DOCK,"dock/state"},{DISPLAY_OPERATION,"operation/state"}})
    input<std_msgs::msg::UInt8>("/robot/"+entry.second,entry.first,[](const auto &m){return Bytes{m.data};});
  for(const auto &entry:std::vector<std::pair<uint16_t,std::string>>{{DISPLAY_HOMING,"homing/state"},{DISPLAY_NIGHT,"night/state"},{DISPLAY_MUTE,"mute/state"},{DISPLAY_VOLUME_LOCK,"volume_lock/state"}})
    input<std_msgs::msg::Bool>("/robot/"+entry.second,entry.first,[](const auto &m){return Bytes{static_cast<uint8_t>(m.data)};});
  input<msg::RobotProvisionState>("/robot/provision/state",DISPLAY_PROVISION,[](const auto &m){ Bytes b; put32(b,m.id); b.push_back(m.phase); b.push_back(m.result); put16(b,m.phase_left_s); put16(b,m.window_left_s); return b; });
  input<msg::DisplayFaultState>("/robot/fault/state",DISPLAY_FAULT,[](const auto &m){
    if(m.faults.size()>32) throw std::invalid_argument("too many faults");
    Bytes b={static_cast<uint8_t>(m.faults.size())};
    for(const auto &f:m.faults) { put16(b,f.id); b.push_back(f.severity); b.push_back(f.acknowledged); put32(b,f.order); } return b;
  });
  input<msg::RobotUpgradeState>("/robot/upgrade/state",DISPLAY_UPGRADE,[](const auto &m){ Bytes b; put32(b,m.id); b.insert(b.end(),{m.target,m.phase,m.progress,m.result}); return b; });
  input<msg::DisplayIdentityState>("/robot/identity/state",DISPLAY_IDENTITY,[](const auto &m){
    if(m.name.size()>63) throw std::invalid_argument("name exceeds 63 UTF-8 bytes");
    Bytes b={static_cast<uint8_t>(m.bound),static_cast<uint8_t>(m.name.size())}; b.insert(b.end(),m.name.begin(),m.name.end()); return b;
  });
  input<msg::DisplayVoiceState>("/robot/voice/state",DISPLAY_VOICE,[](const auto &m){
    if(m.bins.size()>64) throw std::invalid_argument("too many voice bins");
    Bytes b={m.stage,static_cast<uint8_t>(m.bins.size())}; b.insert(b.end(),m.bins.begin(),m.bins.end()); return b;
  });
  input<std_msgs::msg::UInt8>("/robot/interaction/event", DISPLAY_INTERACTION,
    [](const auto &m){ return Bytes{m.data}; }, true);
  input<std_msgs::msg::Empty>("/robot/task/succeeded", DISPLAY_TASK_SUCCESS,
    [](const auto &){ return Bytes{1}; }, true);
}
void HpmDisplay::submit(uint16_t key,Bytes value,bool event)
{
  if(display_validate(key,value.data(),value.size())!=DISPLAY_OK) throw std::invalid_argument("invalid display field value");
  std::lock_guard<std::mutex> lock(mutex_);
  if (!running_) return;
  if(!event) {
    auto it=states_.find(key);
    if(it!=states_.end() && it->second.command.value==value) return;
    if (it==states_.end() && states_.size()==32) {
      RCLCPP_ERROR(logger_, "display state cache full");
      return;
    }
  } else {
    if (!connected_ || maintenance_ || events_.size()==16) {
      RCLCPP_ERROR(logger_, "%s: display offline/maintenance or event queue full",
                   display_function(key));
      return;
    }
  }
  if(++revision_==0) ++revision_;
  Command c; c.key=key; c.value=std::move(value); c.revision=revision_;
  if(event) events_.push_back(std::move(c)); else states_[key]={std::move(c),true};
  changed_.notify_all();
}
void HpmDisplay::result(uint16_t action,const msg::DisplayResult &m)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!running_) return;
  if(!m.event_id || m.result>DISPLAY_TIMEOUT || !connected_ || maintenance_ || events_.size()==16) {
    RCLCPP_ERROR(logger_,"%s result rejected: invalid/offline/full",display_function(action_key(action))); return;
  }
  unsigned matches=0;
  for(const auto &r:known_requests_) if(r.action==action && r.event==m.event_id && (!m.session || r.boot==m.session)) ++matches;
  if(matches!=1) { RCLCPP_ERROR(logger_,"result must identify one received request (use session for source boot)"); return; }
  Command c; c.key=action_key(action); c.type=DISPLAY_RESULT;
  put32(c.value,m.event_id); put16(c.value,m.result); put16(c.value,m.reason); put32(c.value,m.revision);
  events_.push_back(std::move(c)); changed_.notify_all();
}
void HpmDisplay::connection(const std::string &state)
{
  if(connection_state_==state) return;
  connection_state_=state;
  auto context=node_.get_node_base_interface()->get_context();
  if(!rclcpp::ok(context)) return;
  std_msgs::msg::String m; m.data=state;
  try { connection_->publish(m); }
  catch(const rclcpp::exceptions::RCLError &) { if(rclcpp::ok(context)) throw; }
  if(state=="connecting" || state=="gateway_ready" || state=="offline") {
    RCLCPP_DEBUG(logger_,"display connection: %s",state.c_str());
  } else {
    RCLCPP_INFO(logger_,"display connection: %s",state.c_str());
  }
}
bool HpmDisplay::wait_ms(int duration)
{
  std::unique_lock<std::mutex> lock(mutex_);
  changed_.wait_for(lock,std::chrono::milliseconds(duration),[this]{return !running_ || maintenance_;});
  return running_ && !maintenance_;
}
int HpmDisplay::open_socket()
{
  int fd=::socket(AF_INET,SOCK_STREAM|SOCK_NONBLOCK,0);
  if(fd<0) throw LinkError("socket creation failed");
  { std::lock_guard<std::mutex> lock(mutex_);
    if(!running_ || maintenance_) { close(fd); throw LinkError("maintenance/stopping"); }
    socket_=fd;
  }
  sockaddr_in address{}; address.sin_family=AF_INET; address.sin_port=htons(port_); inet_pton(AF_INET,board_ip_.c_str(),&address.sin_addr);
  int one=1; setsockopt(fd,IPPROTO_TCP,TCP_NODELAY,&one,sizeof(one));
  if(::connect(fd,reinterpret_cast<sockaddr*>(&address),sizeof(address))<0 && errno!=EINPROGRESS) throw LinkError("gateway connect failed");
  auto deadline=milliseconds()+3000;
  while(running_ && milliseconds()<deadline) {
    { std::lock_guard<std::mutex> lock(mutex_); if(maintenance_) throw LinkError("maintenance"); }
    pollfd p{fd,POLLOUT,0};
    if(poll(&p,1,50)>0) {
      int error=0; socklen_t n=sizeof(error); getsockopt(fd,SOL_SOCKET,SO_ERROR,&error,&n);
      if(error) throw LinkError("gateway connect refused");
      return fd;
    }
  }
  throw LinkError("gateway connect timeout");
}
void HpmDisplay::write_all(int fd,const Bytes &data)
{
  size_t offset=0; auto deadline=milliseconds()+2000;
  while(offset<data.size() && running_) {
    { std::lock_guard<std::mutex> lock(mutex_); if(maintenance_) throw LinkError("maintenance"); }
    auto n=send(fd,data.data()+offset,data.size()-offset,MSG_NOSIGNAL);
    if(n>0) offset+=n;
    else if(n<0 && (errno==EAGAIN || errno==EWOULDBLOCK || errno==EINTR)) {
      if(milliseconds()>=deadline) throw LinkError("gateway send timeout");
      pollfd p{fd,POLLOUT,0}; poll(&p,1,20);
    } else throw LinkError("gateway send failed");
  }
}
void HpmDisplay::route(int fd,const Frame &f,Pending *pending,bool &complete)
{
  const auto &b=f.payload;
  if(pending && f.session==pending->frame.session && f.sequence==pending->frame.sequence) {
    if(pending->hello && (f.type==0x70 || f.type==0x71) && b.size()==100) {
      if(f.type!=0x70 || display_u16(b.data())) throw LinkError("P4 HELLO rejected");
      complete=true; return;
    }
    if(!pending->hello && f.type==DISPLAY_RESULT && b.size()==12 && !display_u32(b.data())) {
      const auto result_code=display_u16(b.data()+4);
      if(result_code>DISPLAY_RECEIVED || display_u32(b.data()+8)!=pending->command.revision) return;
      if(result_code!=DISPLAY_RECEIVED && result_code!=DISPLAY_OK) {
        RCLCPP_WARN(logger_,"P4 ACK rejected: function=%s session=%" PRIu32 " sequence=%" PRIu32 " result=%u reason=%u",
                    display_function(pending->command.key),f.session,f.sequence,
                    static_cast<unsigned>(result_code),static_cast<unsigned>(display_u16(b.data()+6)));
      }
      complete=true; return;
    }
    if(!pending->hello && (f.type==0x70 || f.type==0x71)) throw LinkError("P4 firmware lacks display business protocol");
  }
  if(f.type!=DISPLAY_INPUT_EVENT && f.type!=DISPLAY_REPORT_STATE) return;
  { std::lock_guard<std::mutex> lock(mutex_); if(!connected_) return; }
  uint16_t code=DISPLAY_RECEIVED;
  Bytes reply(12,0);
  if(f.type==DISPLAY_REPORT_STATE && b.size()>=10) display_put32(reply.data()+8,display_u32(b.data()+4));
  for(const auto &old:seen_) if(old.session==f.session && old.sequence==f.sequence) {
    if(old.type==f.type && old.payload==b) { write_all(fd,encode({DISPLAY_RESULT,f.session,f.sequence,old.reply})); return; }
    code=DISPLAY_INVALID; break;
  }
  if(!f.session || !f.sequence) code=DISPLAY_INVALID;
  if(code==DISPLAY_RECEIVED && f.type==DISPLAY_INPUT_EVENT) {
    if(b.size()!=8 || !display_u32(b.data()) || !display_event_valid(display_u16(b.data()+4),display_u16(b.data()+6))) code=DISPLAY_INVALID;
    else {
      msg::DisplayRequest m; m.boot_id=f.session; m.sequence=f.sequence; m.event_id=display_u32(b.data()); m.value=display_u16(b.data()+6);
      auto action=display_u16(b.data()+4); bool duplicate=false;
      { std::lock_guard<std::mutex> lock(mutex_);
        for(const auto &r:known_requests_) if(r.boot==m.boot_id && r.event==m.event_id) {
          duplicate=true; if(r.action!=action || r.value!=m.value) code=DISPLAY_INVALID;
        }
        if(!duplicate) { known_requests_.push_back({m.boot_id,m.event_id,action,m.value}); if(known_requests_.size()>64) known_requests_.pop_front(); }
      }
      if(!duplicate) requests_.at(action)->publish(m);
    }
  } else if(code==DISPLAY_RECEIVED) {
    if(b.size()<10 || !display_u32(b.data()) || !display_u32(b.data()+4)) code=DISPLAY_INVALID;
    else {
      auto key=display_u16(b.data()+8);
      code=display_validate(key,b.data()+10,b.size()-10);
      if(key<DISPLAY_PANEL || key>DISPLAY_TOUCH) code=DISPLAY_UNSUPPORTED;
      if(code==DISPLAY_OK) {
        const auto *v=b.data()+10; msg::DisplayDeviceState m; m.boot_id=display_u32(b.data()); m.revision=display_u32(b.data()+4);
        if(m.boot_id!=f.session) code=DISPLAY_INVALID;
        else {
          m.target=static_cast<uint8_t>(key-DISPLAY_PANEL);
          m.status=v[0]; m.error=display_u16(v+1);
          if(key==DISPLAY_BACKLIGHT) m.brightness=v[3];
          panel_state_->publish(m); code=DISPLAY_RECEIVED;
        }
      }
    }
  }
  display_put16(reply.data()+4,code);
  if(f.session && f.sequence) {
    seen_.push_back({f.session,f.sequence,f.type,b,reply}); if(seen_.size()>32) seen_.pop_front();
  }
  write_all(fd,encode({DISPLAY_RESULT,f.session,f.sequence,reply}));
}
void HpmDisplay::worker() noexcept
{
  try {
    const sched_param normal{};
    const int error = pthread_setschedparam(pthread_self(), SCHED_OTHER, &normal);
    if (error != 0) {
      throw std::system_error(error, std::generic_category(), "display thread scheduling");
    }
    run_link();
  } catch (const std::exception &e) {
    fail(e.what());
  } catch (...) {
    fail("unknown display worker exception");
  }
  close_link();
  try {
    connection("stopped");
  } catch (const std::exception &e) {
    RCLCPP_DEBUG(logger_, "Cannot publish final display state: %s", e.what());
  } catch (...) {
    RCLCPP_DEBUG(logger_, "Cannot publish final display state");
  }
}

void HpmDisplay::run_link()
{
  const int initial_backoff=std::min<int64_t>(reconnect_ms_,10000);
  int backoff=initial_backoff;
  uint32_t session=boot_;
  bool outage_reported=false;
  while(running_) {
    { std::unique_lock<std::mutex> lock(mutex_);
      if(maintenance_) { lock.unlock(); connection("maintenance"); lock.lock(); changed_.wait(lock,[this]{return !running_ || !maintenance_;}); }
      if(!running_) break;
    }
    std::optional<Pending> pending;
    try {
      connection("connecting"); int fd=open_socket();
      write_all(fd,Bytes{'E','S','P','O','T','A','1','\n'});
      std::string greeting; auto greet_deadline=milliseconds()+5000;
      while(running_ && greeting.size()<128 && (greeting.empty() || greeting.back()!='\n')) {
        { std::lock_guard<std::mutex> lock(mutex_); if(maintenance_) throw LinkError("maintenance"); }
        if(milliseconds()>=greet_deadline) throw LinkError("gateway handshake timeout");
        uint8_t c; auto n=recv(fd,&c,1,0);
        if(n==1) greeting.push_back(c);
        else if(n==0) throw LinkError("gateway closed during handshake");
        else if(errno!=EAGAIN && errno!=EWOULDBLOCK && errno!=EINTR) throw LinkError("gateway handshake failed");
        else { pollfd p{fd,POLLIN,0}; poll(&p,1,20); }
      }
      if(greeting!="ESP_READY\n") throw LinkError("gateway not ready: "+greeting);
      // A reachable gateway should keep probing for P4 at the initial retry interval.
      backoff=initial_backoff;
      connection("gateway_ready");
      Parser parser;
      // New connection session makes P4 discard uncertain queued actions.
      // Keep upstream dedup history across TCP reconnects for the same P4 boot.
      if(++session==0) ++session;
      std::optional<Pending> probe;
      bool online=false; auto last_reply=milliseconds();
      while(running_) {
        { std::lock_guard<std::mutex> lock(mutex_); if(maintenance_) throw LinkError("maintenance"); }
        pollfd p{fd,POLLIN,0}; int selected=poll(&p,1,20);
        if(selected<0 && errno!=EINTR) throw LinkError("gateway poll failed");
        if(selected>0) {
          if(p.revents&(POLLERR|POLLNVAL)) throw LinkError("gateway socket error");
          if(p.revents&(POLLIN|POLLHUP)) {
            uint8_t block[1024]; auto n=recv(fd,block,sizeof(block),0);
            if(n==0) throw LinkError("gateway closed");
            if(n<0 && errno!=EAGAIN && errno!=EWOULDBLOCK && errno!=EINTR) throw LinkError("gateway receive failed");
            if(n>0) parser.append(block,n);
          }
        }
        Frame f;
        while(parser.next(f)) {
          auto &awaited=probe && f.session==probe->frame.session && f.sequence==probe->frame.sequence ? probe : pending;
          bool complete=false; route(fd,f,awaited?&*awaited:nullptr,complete);
          if(complete) {
            if(!online) {
              { std::lock_guard<std::mutex> lock(mutex_); connected_=true;
                for(auto &entry:states_) entry.second.dirty=true;
              }
              online=true; outage_reported=false; connection("online");
            }
            awaited.reset();
            // A matched business reply also proves the P4 link is alive.
            probe.reset(); last_reply=milliseconds();
          }
        }
        auto now=milliseconds();
        // Probe independently of business ACK retries on the same TCP/UART session.
        if(!probe && (!online || now-last_reply>=keepalive_ms_)) {
          if(!++sequence_) throw std::overflow_error("display sequence exhausted");
          probe.emplace(); probe->hello=true; probe->frame={1,session,sequence_,{}};
          probe->deadline=now;
        }
        if(probe && now>=probe->deadline) {
          if(probe->attempts>=3) {
            throw LinkError("P4 HELLO response timeout");
          }
          ++probe->attempts; probe->deadline=now+hello_timeout_ms_; write_all(fd,encode(probe->frame));
        }
        if(pending && now>=pending->deadline) {
          if(pending->attempts>static_cast<unsigned>(max_retries_)) {
            throw LinkError("ACK retry limit reached");
          }
          ++pending->attempts; pending->deadline=now+ack_timeout_ms_; write_all(fd,encode(pending->frame));
        }
        if(!pending && online) {
          std::optional<Command> next;
          { std::lock_guard<std::mutex> lock(mutex_);
            if(!events_.empty()) { next=events_.front(); events_.pop_front(); }
            else for(auto &entry:states_) if(entry.second.dirty) { next=entry.second.command; entry.second.dirty=false; break; }
          }
          if(!next) continue;
          Pending request;
          request.command=*next;
          Bytes payload;
          if(next->type==DISPLAY_SET_STATE) {
            put32(payload,boot_); put32(payload,next->revision); put16(payload,next->key); payload.insert(payload.end(),next->value.begin(),next->value.end());
          } else payload=next->value;
          request.frame={next->type,session,++sequence_,std::move(payload)};
          if(!sequence_) throw std::overflow_error("display sequence exhausted");
          request.attempts=1; request.deadline=now+ack_timeout_ms_; pending=request; write_all(fd,encode(request.frame));
        }
      }
    } catch(const LinkError &e) {
      bool maintaining;
      { std::lock_guard<std::mutex> lock(mutex_); maintaining=maintenance_; }
      if(running_ && !maintaining) {
        if(pending) {
          RCLCPP_WARN(logger_,"P4 ACK missing: function=%s session=%" PRIu32 " sequence=%" PRIu32 " attempts=%u (%s)",
                      display_function(pending->command.key),pending->frame.session,pending->frame.sequence,
                      pending->attempts,e.what());
        }
        if(outage_reported) {
          RCLCPP_DEBUG(logger_,"display link retry failed: %s",e.what());
        } else {
          RCLCPP_WARN(logger_,"display link offline: %s",e.what());
          outage_reported=true;
        }
      }
    }
    close_link();
    if(!running_) break;
    connection("offline"); wait_ms(backoff); backoff=std::min(backoff*2,10000);
  }
}
}  // namespace hpmDisplay
