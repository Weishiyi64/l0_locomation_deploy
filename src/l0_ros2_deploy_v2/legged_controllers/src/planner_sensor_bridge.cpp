#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstring>
#include <cstdint>
#include <fstream>
#include <iterator>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_ros/buffer.h>
#include <tf2/time.h>
#include <std_msgs/msg/string.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <urdf/model.h>
#include <yaml-cpp/yaml.h>
#include <odin_ros_driver_rev1/srv/get_calibration.hpp>
#include "PlannerDepthOpenCl.h"
#include "AppVersion.h"

namespace {
constexpr auto kDepthPeriod = std::chrono::milliseconds(200);
}

class PlannerSensorBridge final : public rclcpp::Node {
 public:
  PlannerSensorBridge() : Node("planner_sensor_bridge"), buffer_(get_clock()), listener_(buffer_) {
    check_sensor_timestamps_ = declare_parameter<bool>("nav_check_sensor_timestamps", true);
    if (check_sensor_timestamps_)
      RCLCPP_INFO(get_logger(), "Navigation sensor timestamp checks enabled");
    else
      RCLCPP_WARN(get_logger(), "Navigation sensor timestamp checks DISABLED: latest received data; "
                               "source age/order/alignment ignored, 0.5s receive watchdog retained");
    prefix_ = declare_parameter<std::string>("device_prefix", "/manifold/device0/");
    if (prefix_.empty() || prefix_.back() != '/') throw std::runtime_error("device_prefix requires trailing slash");
    const auto path = declare_parameter<std::string>("robot_description_path", "");
    urdf::Model robot;
    if (!robot.initFile(path)) throw std::runtime_error("Cannot load robot description: "+path);
    auto link = robot.getLink("lidar_link");
    Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
    while (link && link->name != "base_link") {
      const auto joint = link->parent_joint;
      if (!joint || joint->type != urdf::Joint::FIXED) throw std::runtime_error("lidar_link must have a fixed base_link mount");
      const auto& p = joint->parent_to_joint_origin_transform;
      Eigen::Isometry3d parent = Eigen::Isometry3d::Identity();
      parent.translation() = Eigen::Vector3d(p.position.x, p.position.y, p.position.z);
      parent.linear() = Eigen::Quaterniond(p.rotation.w, p.rotation.x, p.rotation.y, p.rotation.z).normalized().toRotationMatrix();
      transform = parent*transform;
      link = robot.getLink(joint->parent_link_name);
    }
    if (!link || !transform.matrix().allFinite()) throw std::runtime_error("Missing valid base_link -> lidar_link chain");
    geometry_.base_from_lidar = transform;
    gpu_ = std::make_unique<legged_controllers::PlannerDepthOpenCl>();
    RCLCPP_INFO(get_logger(), "Depth fusion GPU: %s", gpu_->deviceName().c_str());
    const auto qos = rclcpp::QoS(1).reliable().durability_volatile();
    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("/planner/base_odometry", qos);
    tf_pub_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    static_tf_pub_ = std::make_unique<tf2_ros::StaticTransformBroadcaster>(*this);
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(prefix_+"odometry_lf", rclcpp::SensorDataQoS(),
        [this](nav_msgs::msg::Odometry::ConstSharedPtr msg) { odometry(*msg); });
    fused_pub_ = create_publisher<sensor_msgs::msg::Image>("/planner/depth_fused", qos);
    cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(prefix_+"cloud/raw", qos,
      [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) {
        const auto callback_time = std::chrono::steady_clock::now();
        cloud_callback_interval_ms_ = last_cloud_callback_.time_since_epoch().count() == 0 ? -1. :
            std::chrono::duration<double, std::milli>(callback_time-last_cloud_callback_).count();
        last_cloud_callback_ = callback_time;
        ++cloud_received_count_;
        if (msg->header.frame_id != "device0/lidar") {
          RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
              "Raw cloud frame mismatch: expected device0/lidar, got '%s'", msg->header.frame_id.c_str());
          return;
        }
        if (!fresh(msg->header.stamp)) {
          const double stamp = msg->header.stamp.sec+msg->header.stamp.nanosec*1e-9;
          const double current = now().seconds();
          RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
              "Raw cloud timestamp rejected: source=%.6f ROS_now=%.6f age=%.6fs; "
              "require 0 <= age <= 0.5s in the same clock domain", stamp, current, current-stamp);
          return;
        }
        pending_ = msg;
        pending_received_ = callback_time;
      });
    calib_ = create_client<odin_ros_driver_rev1::srv::GetCalibration>(prefix_+"get_calibration");
    online_sub_ = create_subscription<std_msgs::msg::String>("/manifold/driver/device_online",
        rclcpp::QoS(10).transient_local(), [this](std_msgs::msg::String::ConstSharedPtr msg) {
          try {
            const auto device = YAML::Load(msg->data);
            if (device["prefix"].as<std::string>() != prefix_) return;
            const auto sn = device["sn"].as<std::string>();
            if (sn == device_sn_) return;
            device_sn_ = sn; ++calibration_generation_;
            calibrated_ = odom_valid_ = false; odom_stamp_ = cloud_stamp_ = 0; pending_.reset();
            if (request_pending_) calib_->remove_pending_request(request_id_);
            request_pending_ = false;
          } catch (const std::exception& error) { RCLCPP_WARN(get_logger(), "Invalid device notification: %s", error.what()); }
        });
    calibration_timer_ = create_wall_timer(std::chrono::seconds(1), [this] { calibration(); });
    timer_ = create_wall_timer(kDepthPeriod, [this] { process(); });
  }

 private:
  bool fresh(const builtin_interfaces::msg::Time& stamp) const {
    if (stamp.sec < 0 || stamp.nanosec >= 1000000000U) return false;
    if (!check_sensor_timestamps_) return true;
    const int64_t ns = static_cast<int64_t>(stamp.sec)*1000000000LL+stamp.nanosec;
    const double age = (now().nanoseconds()-ns)*1e-9;
    return ns > 0 && age >= 0. && age <= .5;
  }

  void calibration() {
    if (calibrated_ || !calib_->service_is_ready()) return;
    const auto current = std::chrono::steady_clock::now();
    if (request_pending_) {
      if (current-request_started_ < std::chrono::seconds(3)) return;
      calib_->remove_pending_request(request_id_);
      request_pending_ = false;
    }
    request_pending_ = true; request_started_ = current;
    const auto generation = calibration_generation_;
    auto result = calib_->async_send_request(std::make_shared<odin_ros_driver_rev1::srv::GetCalibration::Request>(),
        [this, generation](rclcpp::Client<odin_ros_driver_rev1::srv::GetCalibration>::SharedFuture future) {
          if (generation != calibration_generation_) return;
          request_pending_ = false;
          try {
            const auto response = future.get();
            if (!response->success) throw std::runtime_error("Device calibration is not ready");
            const auto root = YAML::Load(response->yaml_content);
            YAML::Node camera = root["cam_0_640_544"];
            if (!camera) camera = root["cam_0_1280_1088"];
            if (!camera) throw std::runtime_error("Missing camera0 640x544 or 1280x1088 calibration");
            const double width = camera["image_width"].as<double>(), height = camera["image_height"].as<double>();
            if (!(width > 0.) || !(height > 0.)) throw std::runtime_error("Invalid calibration dimensions");
            geometry_.intrinsics.setIdentity();
            geometry_.intrinsics(0, 0) = camera["A11"].as<double>()*640./width;
            geometry_.intrinsics(0, 1) = camera["A12"].as<double>(0.)*640./width;
            geometry_.intrinsics(1, 1) = camera["A22"].as<double>()*544./height;
            geometry_.intrinsics(0, 2) = camera["u0"].as<double>()*640./width;
            geometry_.intrinsics(1, 2) = camera["v0"].as<double>()*544./height;
            const auto values = root["Tcl_0"].as<std::vector<float>>();
            if (values.size() != 16) throw std::runtime_error("Invalid Tcl_0 length");
            for (int i = 0; i < 16; ++i) geometry_.camera_from_lidar(i/4, i%4) = values[i];
            const auto& t = geometry_.camera_from_lidar;
            if (!t.allFinite() || !geometry_.intrinsics.allFinite() || geometry_.intrinsics(0, 0) <= 0. ||
                geometry_.intrinsics(1, 1) <= 0. ||
                (t.row(3)-Eigen::RowVector4f(0, 0, 0, 1)).norm() > 1e-5 ||
                std::abs(t.block<3, 3>(0, 0).determinant()-1.f) > .01f ||
                (t.block<3, 3>(0, 0).transpose()*t.block<3, 3>(0, 0)-Eigen::Matrix3f::Identity()).norm() > .01f)
              throw std::runtime_error("Invalid device camera calibration");
            calibrated_ = true;
            RCLCPP_INFO(get_logger(), "Device calibration ready; robot mount loaded from URDF");
          } catch (const std::exception& error) { RCLCPP_WARN(get_logger(), "Calibration unavailable: %s", error.what()); }
        });
    request_id_ = result.request_id;
  }

  void odometry(const nav_msgs::msg::Odometry& msg) {
    const auto received = std::chrono::steady_clock::now();
    try {
      if (msg.header.frame_id != "device0/odom" || msg.child_frame_id != "device0/imu")
        throw std::runtime_error("odometry_lf frame mismatch: expected device0/odom -> device0/imu, got "+
                                 msg.header.frame_id+" -> "+msg.child_frame_id);
      if (!fresh(msg.header.stamp)) {
        const double stamp = msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9;
        const double current = now().seconds();
        throw std::runtime_error("odometry_lf timestamp rejected: source="+std::to_string(stamp)+
            " ROS_now="+std::to_string(current)+" age="+std::to_string(current-stamp)+
            "s; require 0 <= age <= 0.5s in the same clock domain");
      }
      const auto stamp_ns = rclcpp::Time(msg.header.stamp).nanoseconds();
      if (check_sensor_timestamps_ && stamp_ns <= odom_stamp_)
        throw std::runtime_error("Odometry clock did not advance");
      const auto mount = buffer_.lookupTransform("device0/lidar", "device0/imu", tf2::TimePointZero);
      const auto& tr = mount.transform;
      Eigen::Quaterniond qm(tr.rotation.w, tr.rotation.x, tr.rotation.y, tr.rotation.z);
      if (!qm.coeffs().allFinite() || qm.norm() < 1e-8) throw std::runtime_error("Invalid device internal rotation");
      Eigen::Isometry3d lidar_from_imu = Eigen::Isometry3d::Identity();
      lidar_from_imu.linear() = qm.normalized().toRotationMatrix();
      lidar_from_imu.translation() = Eigen::Vector3d(tr.translation.x, tr.translation.y, tr.translation.z);
      if (!lidar_from_imu.matrix().allFinite()) throw std::runtime_error("Invalid device internal translation");
      const auto& p = msg.pose.pose;
      Eigen::Quaterniond q(p.orientation.w, p.orientation.x, p.orientation.y, p.orientation.z);
      if (!q.coeffs().allFinite() || q.norm() < 1e-8) throw std::runtime_error("Invalid odometry quaternion");
      Eigen::Isometry3d world_from_imu = Eigen::Isometry3d::Identity();
      world_from_imu.linear() = q.normalized().toRotationMatrix();
      world_from_imu.translation() = Eigen::Vector3d(p.position.x, p.position.y, p.position.z);
      const Eigen::Isometry3d base_from_imu = geometry_.base_from_lidar*lidar_from_imu;
      const Eigen::Isometry3d imu_from_base = base_from_imu.inverse();
      const Eigen::Isometry3d world_from_base = world_from_imu*imu_from_base;
      const auto& v = msg.twist.twist.linear; const auto& w = msg.twist.twist.angular;
      const Eigen::Vector3d angular = base_from_imu.linear()*Eigen::Vector3d(w.x, w.y, w.z);
      const Eigen::Vector3d velocity = base_from_imu.linear()*Eigen::Vector3d(v.x, v.y, v.z)-angular.cross(base_from_imu.translation());
      if (!world_from_base.matrix().allFinite() || !velocity.allFinite() || !angular.allFinite())
        throw std::runtime_error("Non-finite base odometry");
      if (!mount_tf_published_) {
        geometry_msgs::msg::TransformStamped mount_tf;
        mount_tf.header.stamp = msg.header.stamp;
        mount_tf.header.frame_id = msg.child_frame_id;
        mount_tf.child_frame_id = "device0/base_link";
        mount_tf.transform.translation.x = imu_from_base.translation().x();
        mount_tf.transform.translation.y = imu_from_base.translation().y();
        mount_tf.transform.translation.z = imu_from_base.translation().z();
        const Eigen::Quaterniond rotation = Eigen::Quaterniond(imu_from_base.linear()).normalized();
        mount_tf.transform.rotation.x = rotation.x(); mount_tf.transform.rotation.y = rotation.y();
        mount_tf.transform.rotation.z = rotation.z(); mount_tf.transform.rotation.w = rotation.w();
        static_tf_pub_->sendTransform(mount_tf);
        mount_tf_published_ = true;
        RCLCPP_INFO(get_logger(), "Published URDF mount TF: device0/imu -> device0/base_link");
      }
      nav_msgs::msg::Odometry output = msg;
      output.child_frame_id = "base_link";
      const Eigen::Quaterniond qb(world_from_base.linear());
      output.pose.pose.position.x = world_from_base.translation().x();
      output.pose.pose.position.y = world_from_base.translation().y();
      output.pose.pose.position.z = world_from_base.translation().z();
      output.pose.pose.orientation.x = qb.x(); output.pose.pose.orientation.y = qb.y();
      output.pose.pose.orientation.z = qb.z(); output.pose.pose.orientation.w = qb.w();
      output.twist.twist.linear.x = velocity.x(); output.twist.twist.linear.y = velocity.y(); output.twist.twist.linear.z = velocity.z();
      output.twist.twist.angular.x = angular.x(); output.twist.twist.angular.y = angular.y(); output.twist.twist.angular.z = angular.z();
      // The SDK does not populate covariance; do not carry an untransformed covariance.
      output.pose.covariance.fill(0.); output.twist.covariance.fill(0.);
      odom_pub_->publish(output);
      geometry_msgs::msg::TransformStamped tf;
      tf.header = output.header; tf.child_frame_id = "base_link";
      tf.transform.translation.x = output.pose.pose.position.x; tf.transform.translation.y = output.pose.pose.position.y;
      tf.transform.translation.z = output.pose.pose.position.z; tf.transform.rotation = output.pose.pose.orientation;
      tf_pub_->sendTransform(tf);
      world_from_base_ = world_from_base; odom_stamp_ = stamp_ns; odom_valid_ = true;
      odom_received_ = received;
    } catch (const std::exception& error) {
      odom_valid_ = false;
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "%s", error.what());
    }
  }

  sensor_msgs::msg::Image makeImage(const std::array<float, 2560>& values, const std_msgs::msg::Header& header) {
    sensor_msgs::msg::Image image;
    image.header = header; image.width = 64; image.height = 40;
    image.encoding = "32FC1"; image.step = 64*sizeof(float);
    const uint16_t probe = 1;
    image.is_bigendian = *reinterpret_cast<const uint8_t*>(&probe) == 0;
    image.data.resize(sizeof(float)*values.size());
    std::memcpy(image.data.data(), values.data(), image.data.size());
    return image;
  }

  void process() {
    const auto started = std::chrono::steady_clock::now();
    const auto deadline = started+kDepthPeriod;
    const double timer_interval_ms = last_process_started_.time_since_epoch().count() == 0 ? -1. :
        std::chrono::duration<double, std::milli>(started-last_process_started_).count();
    last_process_started_ = started;
    auto cloud = std::move(pending_);
    const auto received = pending_received_;
    const double queue_ms = cloud ? std::chrono::duration<double, std::milli>(started-received).count() : -1.;
    double unpack_ms = -1., depth_compute_ms = -1., fused_pub_ms = -1.;
    double source_delta_ms = -1.;
    const auto log_timing = [&](const char* status) {
      const auto current = std::chrono::steady_clock::now();
      const double receive_age_ms = received.time_since_epoch().count() == 0 ? -1. :
          std::chrono::duration<double, std::milli>(current-received).count();
      const double odom_age_ms = odom_received_.time_since_epoch().count() == 0 ? -1. :
          std::chrono::duration<double, std::milli>(current-odom_received_).count();
      RCLCPP_DEBUG_THROTTLE(get_logger(), *get_clock(), 2000,
          "[depth_timing] status=%s backend=opencl rx=%llu fused=%llu callback_dt_ms=%.2f timer_dt_ms=%.2f "
          "queue_ms=%.2f unpack_ms=%.2f depth_compute_ms=%.3f fused_pub_ms=%.2f "
          "total_ms=%.2f receive_age_ms=%.2f odom_age_ms=%.2f source_delta_ms=%.2f",
          status, static_cast<unsigned long long>(cloud_received_count_),
          static_cast<unsigned long long>(fused_published_count_),
          cloud_callback_interval_ms_, timer_interval_ms, queue_ms, unpack_ms, depth_compute_ms, fused_pub_ms,
          std::chrono::duration<double, std::milli>(current-started).count(), receive_age_ms, odom_age_ms, source_delta_ms);
    };
    const auto deadline_missed = [&](const char* stage) {
      const auto current = std::chrono::steady_clock::now();
      if (current <= deadline) return false;
      RCLCPP_ERROR(get_logger(),
          "[depth_timeout] stage=%s elapsed_ms=%.3f budget_ms=200: depth generation missed "
          "the planner's 5 Hz cycle; discarding this frame's remaining outputs",
          stage, std::chrono::duration<double, std::milli>(current-started).count());
      log_timing("deadline_missed");
      return true;
    };
    if (gpu_failed_) { log_timing("gpu_failed"); return; }
    if (!cloud) { log_timing("no_cloud"); return; }
    if (!calibrated_) { log_timing("wait_calibration"); return; }
    if (!fresh(cloud->header.stamp) || queue_ms > 500.) { log_timing("input_stale"); return; }
    const auto& header = cloud->header;
    try {
      const std::size_t step = cloud->point_step, count = static_cast<std::size_t>(cloud->width)*cloud->height;
      if (count != 49152)
        throw std::runtime_error("Raw cloud point count="+std::to_string(count)+"; expected 49152 (check UDP packet loss)");
      if (cloud->is_bigendian || step == 0 ||
          cloud->row_step < static_cast<std::size_t>(cloud->width)*step ||
          cloud->data.size() < static_cast<std::size_t>(cloud->row_step)*cloud->height)
        throw std::runtime_error("Invalid PointCloud2 layout");
      std::size_t offsets[3]{};
      const char* names[] = {"x", "y", "z"};
      for (int axis = 0; axis < 3; ++axis) {
        bool found = false;
        for (const auto& field : cloud->fields) if (field.name == names[axis]) {
          if (found || field.datatype != sensor_msgs::msg::PointField::FLOAT32 || field.count != 1 ||
              static_cast<std::size_t>(field.offset)+4 > step) throw std::runtime_error("Invalid xyz field");
          found = true; offsets[axis] = field.offset;
        }
        if (!found) throw std::runtime_error("Missing xyz field");
      }
      const auto stamp = rclcpp::Time(cloud->header.stamp).nanoseconds();
      if (check_sensor_timestamps_ && stamp <= cloud_stamp_)
        throw std::runtime_error("Cloud timestamp did not advance");
      cloud_stamp_ = stamp;
      std::vector<float> xyz; xyz.reserve(count*3);
      for (std::size_t row = 0; row < cloud->height; ++row)
        for (std::size_t col = 0; col < cloud->width; ++col) {
          float point[3];
          const auto* source = cloud->data.data()+row*cloud->row_step+col*step;
          for (int axis = 0; axis < 3; ++axis) std::memcpy(&point[axis], source+offsets[axis], 4);
          if (!std::isfinite(point[0]) || !std::isfinite(point[1]) || !std::isfinite(point[2]) ||
              (point[0] == 0.f && point[1] == 0.f && point[2] == 0.f)) continue;
          xyz.insert(xyz.end(), point, point+3);
        }
      if (xyz.empty()) throw std::runtime_error("Cloud contains no valid points");
      const auto unpacked = std::chrono::steady_clock::now();
      unpack_ms = std::chrono::duration<double, std::milli>(unpacked-started).count();
      if (deadline_missed("unpack")) return;
      const double odom_age_ms = std::chrono::duration<double, std::milli>(
          std::chrono::steady_clock::now()-odom_received_).count();
      source_delta_ms = std::abs(stamp-odom_stamp_)*1e-6;
      if (!odom_valid_ || (check_sensor_timestamps_ && source_delta_ms > 150.) || odom_age_ms > 500.) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
            "Depth fusion waiting: odometry_valid=%d source_delta=%.1fms (limit 150), "
            "odometry_receive_age=%.1fms (limit 500), elapsed_before_fusion=%.1fms timestamp_checks=%d",
            static_cast<int>(odom_valid_), source_delta_ms, odom_age_ms,
            std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now()-started).count(),
            static_cast<int>(check_sensor_timestamps_));
        log_timing("wait_odometry"); return;
      }
      const auto compute_started = std::chrono::steady_clock::now();
      legged_controllers::PlannerDepthOpenCl::Result depth;
      try {
        depth = gpu_->process(geometry_, xyz, world_from_base_);
      } catch (const std::exception& error) {
        gpu_failed_ = true;
        RCLCPP_ERROR(get_logger(), "GPU depth processing failed; depth output stopped until restart: %s", error.what());
        throw;
      }
      const auto completed = std::chrono::steady_clock::now();
      depth_compute_ms = std::chrono::duration<double, std::milli>(completed-compute_started).count();
      if (deadline_missed("depth_compute")) return;
      auto fused_image = makeImage(depth.fused, header);
      if (deadline_missed("prepare_output")) return;
      const auto ready = std::chrono::steady_clock::now();
      if (fresh(cloud->header.stamp) && ready-received <= std::chrono::milliseconds(500)) {
        const auto fused_started = std::chrono::steady_clock::now();
        if (deadline_missed("fused_publish")) return;
        fused_pub_->publish(fused_image);
        fused_pub_ms = std::chrono::duration<double, std::milli>(
            std::chrono::steady_clock::now()-fused_started).count();
        ++fused_published_count_;
        log_timing("published");
      } else {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
            "Depth fusion discarded: source age exceeded 0.5s during processing");
        log_timing("expired_after_fusion");
      }
    } catch (const std::exception& error) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Depth rejected: %s", error.what());
      log_timing("rejected");
    }
  }

  std::string prefix_, device_sn_;
  bool check_sensor_timestamps_{true};
  std::uint64_t calibration_generation_{0};
  std::uint64_t cloud_received_count_{0}, fused_published_count_{0};
  double cloud_callback_interval_ms_{-1.};
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr online_sub_;
  legged_controllers::PlannerDepthGeometry geometry_;
  std::unique_ptr<legged_controllers::PlannerDepthOpenCl> gpu_;
  bool gpu_failed_{false};
  Eigen::Isometry3d world_from_base_{Eigen::Isometry3d::Identity()};
  tf2_ros::Buffer buffer_;
  tf2_ros::TransformListener listener_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_pub_;
  std::unique_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_pub_;
  bool mount_tf_published_{false};
  bool calibrated_{false}, odom_valid_{false}, request_pending_{false};
  int64_t request_id_{0}, odom_stamp_{0}, cloud_stamp_{0};
  std::chrono::steady_clock::time_point odom_received_{}, request_started_{}, pending_received_{};
  std::chrono::steady_clock::time_point last_cloud_callback_{}, last_process_started_{};
  sensor_msgs::msg::PointCloud2::ConstSharedPtr pending_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr fused_pub_;
  rclcpp::Client<odin_ros_driver_rev1::srv::GetCalibration>::SharedPtr calib_;
  rclcpp::TimerBase::SharedPtr timer_, calibration_timer_;
};

int main(int argc, char** argv) {
  l0::printAppVersion("planner_sensor_bridge");
  rclcpp::init(argc, argv);
  try { rclcpp::spin(std::make_shared<PlannerSensorBridge>()); }
  catch (const std::exception& error) {
    RCLCPP_FATAL(rclcpp::get_logger("planner_sensor_bridge"), "%s", error.what());
    rclcpp::shutdown(); return 1;
  }
  rclcpp::shutdown(); return 0;
}
