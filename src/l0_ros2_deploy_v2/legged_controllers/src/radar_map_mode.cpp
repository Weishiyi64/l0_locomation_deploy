#include <chrono>
#include <memory>
#include <stdexcept>
#include <string>
#include <sys/stat.h>
#include <unistd.h>
#include <rclcpp/rclcpp.hpp>
#include <luvbot_control_interfaces/srv/map_control.hpp>
#include <odin_ros_driver_rev1/srv/device_mode.hpp>
#include <odin_ros_driver_rev1/srv/load_map.hpp>
#include "AppVersion.h"

class RadarMapMode final : public rclcpp::Node {
  using Control = luvbot_control_interfaces::srv::MapControl;
  using Device = odin_ros_driver_rev1::srv::DeviceMode;
  using LoadMap = odin_ros_driver_rev1::srv::LoadMap;
  using Res = Control::Response;
  using Clock = std::chrono::steady_clock;
 public:
  RadarMapMode() : Node("radar_map_mode") {
    const auto prefix = declare_parameter<std::string>("device_prefix", "/manifold/device0/");
    if (prefix.empty() || prefix.back() != '/') throw std::runtime_error("device_prefix requires trailing slash");
    load_path_ = declare_parameter<std::string>("load_path", "/userdata/forlinx/auto_exploration/map/");
    if (load_path_.empty() || load_path_.front() != '/')
      throw std::runtime_error("load_path must be an absolute directory");
    if (load_path_.back() != '/') load_path_ += '/';
    client_ = create_client<Device>(prefix + "device_mode");
    load_client_ = create_client<LoadMap>(prefix + "load_map");
    service_ = create_service<Control>("/control/radar/map_mode",
      [this](std::shared_ptr<rmw_request_id_t> header, std::shared_ptr<Control::Request> request) {
        using Req = Control::Request;
        if ((request->index != Req::MAP_INDEX_1 && request->index != Req::MAP_INDEX_2) ||
            (request->action != Req::ACTION_MAPPING && request->action != Req::ACTION_RELOCATION)) {
          respond(header, Res::ERROR_INVALID_REQUEST, "Supported: index=1/2, action=MAPPING/RELOCATION");
          return;
        }
        if (busy_) {
          respond(header, Res::ERROR_SERVICE_DOWN, "Previous Odin request still outstanding; outcome may be unconfirmed");
          return;
        }
        if (!client_->service_is_ready()) {
          respond(header, Res::ERROR_SERVICE_DOWN, "Odin device_mode service unavailable");
          return;
        }
        std::string map_path;
        if (request->action == Req::ACTION_RELOCATION) {
          const auto& id = request->map_id;
          if (id.empty() || id.size() > 128 ||
              id.find_first_not_of("ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789_-") != std::string::npos) {
            respond(header, Res::ERROR_INVALID_REQUEST, "Relocation requires a valid map_id");
            return;
          }
          map_path = load_path_ + id + ".bin";
          struct stat file{};
          if (::lstat(map_path.c_str(), &file) != 0 || !S_ISREG(file.st_mode) ||
              file.st_size <= 0 || ::access(map_path.c_str(), R_OK) != 0) {
            respond(header, Res::ERROR_STEP_FAILED, "Odin map missing, empty or unreadable: " + map_path);
            return;
          }
          if (!load_client_->service_is_ready()) {
            respond(header, Res::ERROR_SERVICE_DOWN, "Odin load_map service unavailable");
            return;
          }
        }
        auto device = std::make_shared<Device::Request>();
        device->mode = request->action == Req::ACTION_MAPPING ? 3 : 4;
        header_ = header;
        busy_ = true;
        started_ = Clock::now();
        stage_ = "device_mode";
        RCLCPP_INFO(get_logger(), "[map_mode] action=%u index=%u Odin mode=%u",
                    request->action, request->index, device->mode);
        try {
          client_->async_send_request(device, [this, map_path](rclcpp::Client<Device>::SharedFuture future) {
            std::string message;
            if (!stepSucceeded(future, message)) return;
            if (map_path.empty()) finish(Res::ERROR_NONE, message);
            else loadMap(map_path);
          });
        } catch (const std::exception& error) {
          finish(Res::ERROR_SERVICE_DOWN, "Odin device_mode request failed: " + std::string(error.what()));
        }
      });
    timer_ = create_wall_timer(std::chrono::milliseconds(50), [this]() { expire(); });
  }
 private:
  void expire() {
    if (header_ && Clock::now() - started_ >= std::chrono::seconds(10)) {
      respond(header_, Res::ERROR_SERVICE_DOWN, "Odin " + stage_ + " timed out; outcome unconfirmed");
      header_.reset();  // Keep busy until Odin retires the request.
    }
  }

  void finish(uint16_t error, const std::string& message) {
    if (header_) respond(header_, error, message);
    header_.reset();
    busy_ = false;
  }

  template <typename Future>
  bool stepSucceeded(const Future& future, std::string& message) {
    expire();
    if (!header_) {
      RCLCPP_WARN(get_logger(), "Late Odin %s response; workflow not resumed", stage_.c_str());
      busy_ = false;
      return false;
    }
    try {
      const auto result = future.get();
      message = result->message + " (Odin error=" + std::to_string(result->error_code) + ")";
      if (result->success && result->error_code == 0) return true;
      finish(Res::ERROR_STEP_FAILED, "Odin " + stage_ + " failed: " + message);
    } catch (const std::exception& error) {
      finish(Res::ERROR_SERVICE_DOWN, "Odin " + stage_ + " response failed: " + error.what());
    }
    return false;
  }

  void loadMap(const std::string& path) {
    stage_ = "load_map";
    if (!load_client_->service_is_ready()) {
      finish(Res::ERROR_SERVICE_DOWN, "Odin reached Relocation but load_map service is unavailable");
      return;
    }
    RCLCPP_INFO(get_logger(), "[map_mode] Odin entered Relocation, loading %s", path.c_str());
    try {
      auto request = std::make_shared<LoadMap::Request>();
      request->path = path;
      load_client_->async_send_request(request, [this, path](rclcpp::Client<LoadMap>::SharedFuture future) {
        std::string message;
        if (stepSucceeded(future, message)) {
          RCLCPP_INFO(get_logger(), "[map_mode] Odin map loaded: %s", path.c_str());
          finish(Res::ERROR_NONE, "Odin entered Relocation and loaded " + path);
        }
      });
    } catch (const std::exception& error) {
      finish(Res::ERROR_SERVICE_DOWN, "Odin load_map request failed: " + std::string(error.what()));
    }
  }

  void respond(const std::shared_ptr<rmw_request_id_t>& header, uint16_t error, const std::string& message) {
    Control::Response response;
    response.accepted = error == Control::Response::ERROR_NONE;
    response.error_code = error;
    response.message = message;
    try { service_->send_response(*header, response); }
    catch (const std::exception& failure) { RCLCPP_ERROR(get_logger(), "Mode reply failed: %s", failure.what()); }
  }
  bool busy_{false};
  std::string load_path_;
  std::string stage_;
  Clock::time_point started_{};
  std::shared_ptr<rmw_request_id_t> header_;
  rclcpp::Client<Device>::SharedPtr client_;
  rclcpp::Client<LoadMap>::SharedPtr load_client_;
  rclcpp::Service<Control>::SharedPtr service_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
  l0::printAppVersion("radar_map_mode");
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RadarMapMode>());
  rclcpp::shutdown();
  return 0;
}
