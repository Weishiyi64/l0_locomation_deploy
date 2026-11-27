#pragma once

#include <Eigen/Dense>
#include <algorithm>
#include <array>
#include <cmath>

namespace legged_controllers {

// Matches the supplied deployment's projection and occupancy-depth conventions.
class PlannerDepthGeometry {
 public:
  static constexpr int kRows = 53, kCols = 64;
  using Grid = std::array<bool, kRows*kCols>;

  // Double-precision expressions repair ambiguous GPU boundaries.
  struct FusionState {
    Eigen::Isometry3d world_from_base, world_from_lidar, world_from_camera;
    Eigen::Matrix3d inverse;
    double c, s;

    int cell(const Eigen::Vector3d& world) const {
      if (!world.allFinite() || world.z() < .05 || world.z() > 1.20) return -1;
      const auto delta = (world-world_from_base.translation()).eval();
      const double x = c*delta.x()+s*delta.y(), y = -s*delta.x()+c*delta.y();
      if (x < -.30 || x >= 5. || y < -3.2 || y >= 3.2) return -1;
      const int row = static_cast<int>(std::floor((x+.30)/.10));
      const int col = static_cast<int>(std::floor((y+3.2)/.10));
      return row >= 0 && row < kRows && col >= 0 && col < kCols ? row*kCols+col : -1;
    }

    Eigen::Vector3d ray(int pixel) const {
      return world_from_camera.linear()*inverse*Eigen::Vector3d(pixel%64+.5, pixel/64+.5, 1.);
    }

    float castPixel(int pixel, float depth, const Grid& occupied) const {
      const Eigen::Vector3d direction = ray(pixel);
      for (int step = 0; step <= 96; ++step) {
        const double z = .2+.05*step;
        const int index = cell(world_from_camera.translation()+direction*z);
        if (index >= 0 && occupied[index])
          return depth > 0.f ? std::min(depth, static_cast<float>(z)) : static_cast<float>(z);
      }
      return depth;
    }
  };

  Eigen::Isometry3d base_from_lidar{Eigen::Isometry3d::Identity()};
  Eigen::Matrix4f camera_from_lidar{Eigen::Matrix4f::Identity()};
  Eigen::Matrix3d intrinsics{Eigen::Matrix3d::Identity()};

  FusionState prepareFusion(const Eigen::Isometry3d& world_from_base) const {
    FusionState state;
    state.world_from_base = world_from_base;
    const double yaw = std::atan2(world_from_base.linear()(1, 0), world_from_base.linear()(0, 0));
    state.c = std::cos(yaw); state.s = std::sin(yaw);
    state.world_from_lidar = world_from_base*base_from_lidar;
    const Eigen::Matrix3d raw_rotation = camera_from_lidar.block<3, 3>(0, 0).cast<double>();
    const Eigen::JacobiSVD<Eigen::Matrix3d> svd(raw_rotation, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::Matrix3d sign = Eigen::Matrix3d::Identity();
    sign(2, 2) = (svd.matrixU()*svd.matrixV().transpose()).determinant();
    const Eigen::Matrix3d camera_rotation = svd.matrixU()*sign*svd.matrixV().transpose();
    Eigen::Isometry3d lidar_from_camera = Eigen::Isometry3d::Identity();
    lidar_from_camera.linear() = camera_rotation.transpose();
    lidar_from_camera.translation() = -lidar_from_camera.linear()*camera_from_lidar.block<3, 1>(0, 3).cast<double>();
    state.world_from_camera = state.world_from_lidar*lidar_from_camera;
    Eigen::Matrix3d small = intrinsics;
    small.row(0) *= 64./640.; small.row(1) *= 40./544.;
    state.inverse = small.inverse();
    return state;
  }
};
}  // namespace legged_controllers
