#pragma once

#include "PlannerDepthGeometry.h"
#include <memory>
#include <string>
#include <vector>

namespace legged_controllers {

// C++14 / Eigen3 / GPU OpenCL 1.2 interface, independent of ROS.
// One thread owns and reuses this object; construction compiles GPU kernels.
// Initialization/execution errors throw std::runtime_error. No automatic CPU fallback.
class PlannerDepthOpenCl {
 public:
  PlannerDepthOpenCl();
  ~PlannerDepthOpenCl();
  PlannerDepthOpenCl(const PlannerDepthOpenCl&) = delete;
  PlannerDepthOpenCl& operator=(const PlannerDepthOpenCl&) = delete;

  const std::string& deviceName() const;
  struct Result { std::array<float, 2560> fused; };
  // xyz: ordered, packed [x,y,z,...] float32 LiDAR coordinates in metres;
  // at most 49152 points, size divisible by 3. Preserve point order.
  // geometry: intrinsics at 640x544; camera_from_lidar and base_from_lidar
  // are calibrated transforms. world_from_base is the current odometry pose.
  // Synchronous result: row-major [40][64], metres, valid [0.2,5], invalid 0.
  // Keep the CPU double-precision boundary repairs in the implementation.
  Result process(const PlannerDepthGeometry& geometry, const std::vector<float>& xyz,
                 const Eigen::Isometry3d& world_from_base);

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};
}  // namespace legged_controllers
