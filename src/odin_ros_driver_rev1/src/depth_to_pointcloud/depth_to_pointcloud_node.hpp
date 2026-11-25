// @file depth_to_pointcloud_node.hpp
// @brief Library-free depth-map -> dense XYZRGB cloud back-projection core.
//
// Implements the simplified pipeline described in
//   src/depth_to_pointcloud/dense-colored-cloud-simplified-1b2eb7.md
//
// Inputs (per frame):
//   - depth      : float32[H*W], meters, 0 = invalid
//   - rgb_undist : uint8 [H*W*3], BGR, ALREADY undistorted, pixel-aligned
//                  with depth (same pinhole K, same resolution).
// Output:
//   - dense XYZRGB points in the lidar frame.
//
// This file deliberately avoids PCL/Eigen/OpenCV/ROS so it can be lifted
// into the device-side SDK as a pure C/C++ module without modification.

#pragma once

#include <cmath>
#include <cstddef>
#include <cstdint>

namespace odin {
namespace depth_to_cloud_core {

// Single XYZRGB point in plain memory layout (no PCL dependency).
struct XyzRgbPoint {
  float   x;
  float   y;
  float   z;
  uint8_t r;
  uint8_t g;
  uint8_t b;
};

// Tunable parameters describing the back-projection.
struct CloudParams {
  // Pinhole intrinsics (same K used to produce depth and rgb_undist).
  float A11;  // fx
  float A12;  // skew (kept; doc §3.3 retains this term)
  float A22;  // fy
  float u0;   // cx
  float v0;   // cy

  // Lidar -> camera extrinsic (4x4 row-major, bottom row [0 0 0 1]).
  float Tcl[16];

  // Pixel sampling stride (e.g. 5). Larger -> fewer points, faster.
  int   step;

  // Depth filter (meters in camera frame).
  float d_min;
  float d_max;
};

// Invert a 4x4 rigid-body transform Tcl = [R | t; 0 0 0 1]
//   Tlc = [R^T | -R^T * t; 0 0 0 1]
// Caller guarantees Tcl is rigid; we use the shortcut from doc §3.1.
static inline void InvertRigid(const float Tcl[16], float Tlc_out[16]) {
  // Transpose the 3x3 rotation block.
  Tlc_out[0]  = Tcl[0];  Tlc_out[1]  = Tcl[4];  Tlc_out[2]  = Tcl[8];
  Tlc_out[4]  = Tcl[1];  Tlc_out[5]  = Tcl[5];  Tlc_out[6]  = Tcl[9];
  Tlc_out[8]  = Tcl[2];  Tlc_out[9]  = Tcl[6];  Tlc_out[10] = Tcl[10];

  // t_lc = -R^T * t_cl.
  const float tx = Tcl[3];
  const float ty = Tcl[7];
  const float tz = Tcl[11];
  Tlc_out[3]  = -(Tlc_out[0]  * tx + Tlc_out[1]  * ty + Tlc_out[2]  * tz);
  Tlc_out[7]  = -(Tlc_out[4]  * tx + Tlc_out[5]  * ty + Tlc_out[6]  * tz);
  Tlc_out[11] = -(Tlc_out[8]  * tx + Tlc_out[9]  * ty + Tlc_out[10] * tz);

  Tlc_out[12] = 0.0f;
  Tlc_out[13] = 0.0f;
  Tlc_out[14] = 0.0f;
  Tlc_out[15] = 1.0f;
}

// Maximum number of points the caller must allocate for a given image size
// and sampling step. Matches the loop bounds used in DepthToDenseCloud.
static inline int MaxOutputPoints(int width, int height, int step) {
  if (step <= 0) step = 1;
  const int rows = (height + step - 1) / step;
  const int cols = (width  + step - 1) / step;
  return rows * cols;
}

// Hot path: back-project a depth map into a dense XYZRGB cloud in the lidar
// frame, sampling RGB color directly from the (already-undistorted) image.
//
// Implements doc §3.2 / §3.3 verbatim:
//   y_cam = (v - v0) * d / A22
//   x_cam = ((u - u0) * d - A12 * y_cam) / A11
//   z_cam = d
//   p_lidar = Tlc * [x_cam, y_cam, z_cam, 1]^T
//   color   = rgb_undist[v, u]   (BGR memory order)
//
// Returns: number of points written into out.
static inline int DepthToDenseCloud(const float*   depth,
                                    const uint8_t* rgb_undist,
                                    int            width,
                                    int            height,
                                    const CloudParams& p,
                                    const float    Tlc[16],
                                    XyzRgbPoint*   out) {
  if (depth == nullptr || rgb_undist == nullptr || out == nullptr) return 0;
  if (width <= 0 || height <= 0) return 0;
  const int step = p.step > 0 ? p.step : 1;

  const float inv_A22 = 1.0f / p.A22;
  const float inv_A11 = 1.0f / p.A11;

  int n = 0;
  for (int v = 0; v < height; v += step) {
    const float dv = static_cast<float>(v) - p.v0;
    const int row_off  = v * width;
    const int row_off3 = row_off * 3;
    for (int u = 0; u < width; u += step) {
      const float d = depth[row_off + u];
      if (!std::isfinite(d) || d <= p.d_min || d >= p.d_max) continue;

      // Back-projection (with skew A12).
      const float y_cam = dv * d * inv_A22;
      const float x_cam = ((static_cast<float>(u) - p.u0) * d - p.A12 * y_cam) * inv_A11;
      const float z_cam = d;

      // Camera -> lidar.
      const float X = Tlc[0] * x_cam + Tlc[1] * y_cam + Tlc[2]  * z_cam + Tlc[3];
      const float Y = Tlc[4] * x_cam + Tlc[5] * y_cam + Tlc[6]  * z_cam + Tlc[7];
      const float Z = Tlc[8] * x_cam + Tlc[9] * y_cam + Tlc[10] * z_cam + Tlc[11];

      // Sample color (BGR memory order -> r/g/b struct fields).
      const uint8_t* px = rgb_undist + (row_off3 + u * 3);
      out[n].x = X;
      out[n].y = Y;
      out[n].z = Z;
      out[n].b = px[0];
      out[n].g = px[1];
      out[n].r = px[2];
      ++n;
    }
  }
  return n;
}

}  // namespace depth_to_cloud_core
}  // namespace odin
