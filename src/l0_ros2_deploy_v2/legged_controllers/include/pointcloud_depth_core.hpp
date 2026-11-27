// Projection matrix helpers used by GPU upload and CPU boundary repairs.
#pragma once

namespace odin {
namespace depth_core {

struct DepthParams {
  float A11, A12, A22, u0, v0;
  float Tcl[16];  // Row-major LiDAR-to-camera transform.
  float scale;
};

// Apply a 3x4 Kcl (stored in a 16-element row-major buffer; the last row is
// unused) to a homogeneous point (x, y, z, 1).
//   xp = M00*x + M01*y + M02*z + M03
//   yp = M10*x + M11*y + M12*z + M13
//   zp = M20*x + M21*y + M22*z + M23
static inline void mat4_apply_kcl(const float M[16], float x, float y, float z,
                                  float* xo, float* yo, float* zo) {
  *xo = M[0] * x + M[1] * y + M[2]  * z + M[3];
  *yo = M[4] * x + M[5] * y + M[6]  * z + M[7];
  *zo = M[8] * x + M[9] * y + M[10] * z + M[11];
}

// Compute Kcl = (K / scale) * Tcl, where K is the pinhole intrinsics matrix
//   [ A11   0   u0 ]
//   [  0   A22  v0 ]
//   [  0    0    1 ]
// A12 (skew) is intentionally dropped to mirror the existing reprojector.
//
// Result layout (row-major, only the top three rows carry data):
//   Kcl[0..3]   = fx*Tcl[0..3] + cx*Tcl[8..11]
//   Kcl[4..7]   = fy*Tcl[4..7] + cy*Tcl[8..11]
//   Kcl[8..11]  = Tcl[8..11]
//   Kcl[12..15] = (0, 0, 0, 1) placeholder
static inline void compute_kcl_with_scale(const DepthParams& p, float scale,
                                          float Kcl_out[16]) {
  const float fx = p.A11 / scale;
  const float fy = p.A22 / scale;
  const float cx = p.u0  / scale;
  const float cy = p.v0  / scale;

  for (int j = 0; j < 4; ++j) {
    const float t0 = p.Tcl[0 * 4 + j];
    const float t1 = p.Tcl[1 * 4 + j];
    const float t2 = p.Tcl[2 * 4 + j];
    Kcl_out[0 * 4 + j] = fx * t0 + cx * t2;
    Kcl_out[1 * 4 + j] = fy * t1 + cy * t2;
    Kcl_out[2 * 4 + j] = t2;
  }
  Kcl_out[12] = 0.0f;
  Kcl_out[13] = 0.0f;
  Kcl_out[14] = 0.0f;
  Kcl_out[15] = 1.0f;
}

// Convenience overload that reads scale from params.
static inline void compute_kcl(const DepthParams& p, float Kcl_out[16]) {
  compute_kcl_with_scale(p, p.scale, Kcl_out);
}

}  // namespace depth_core
}  // namespace odin
