# 深度图 + 去畸变RGB → 稠密彩色点云（简化版）/ Depth + Undistorted RGB → Dense Colored PointCloud (Simplified)

> 假设上游已直接提供 **去畸变后的 RGB 图**（例如订阅 Odin1 的 `/odin1/image_undistort`，由 `sendrgbundistort=1` 启用），本文档把"深度图 → 稠密 XYZRGB 点云（雷达系）"的最小算法描述清楚，省去鱼眼畸变 LUT 和 `cv::remap`。复现者只需要 4 步矩阵 / 算术，**不依赖任何畸变模型**。
>
> Assuming the upstream already publishes an **undistorted** RGB image (e.g. Odin1's `/odin1/image_undistort`), this document describes the minimal pipeline from depth map to a dense XYZRGB point cloud in lidar frame, with no fisheye LUT and no `cv::remap` involved.

---

## 1. 输入 / Inputs

| 名称 / Name | 类型 / Type | 说明 / Description |
|---|---|---|
| `depth` | float32 `[H][W]`，米 | 深度图，与去畸变 RGB **同坐标系、同分辨率**，`0` = 无效 |
| `rgb_undist` | uint8 `[H][W][3]`，BGR | **已去畸变** 的 RGB 图，针孔模型 |
| `A11, A12, A22, u0, v0` | float64 | 针孔相机内参（与生成 `depth` 和 `rgb_undist` 时一致） |
| `Tcl` | float64 `[4][4]`，行主序 | 雷达 → 相机 外参 |
| `step` | int | 像素采样步长（默认 5） |
| `d_min, d_max` | float | 深度过滤（默认 0.1, 100.0 米） |

> ⚠️ **关键前提 / Key precondition**：`depth` 与 `rgb_undist` 必须**像素级对齐**——即两者建立在**同一份针孔内参 `K` 的同一个去畸变像平面**上。若上游对两者用了不同 calib，本文档不适用。
>
> ⚠️ The depth map and the undistorted RGB image must be **pixel-aligned**, i.e. produced from the **same pinhole `K`** on the **same undistorted image plane**. If they don't share calibration, this short pipeline is invalid.

---

## 2. 输出 / Output

- 数据类型 / Type：`PointCloud<XYZRGB>`，每点含 `float x, y, z` + `uint8 r, g, b`
- 坐标系 / Frame：**雷达坐标系**（与原始 `cloud_raw` 同 frame）
- 典型点数：`(H/step) × (W/step) ≈ 83k` (1600×1296, step=5)

---

## 3. 算法 / Algorithm

### 3.1 一次性预计算 / One-time

```
Tlc = inverse(Tcl)         # 4x4, camera -> lidar
```
对刚体变换可简化：`R_lc = R_clᵀ`，`t_lc = -R_clᵀ · t_cl`。

### 3.2 每帧主循环 / Per-frame

```python
def depth_to_dense_cloud(depth, rgb_undist,
                         A11, A12, A22, u0, v0,
                         Tlc,
                         step=5, d_min=0.1, d_max=100.0):
    H, W = depth.shape
    out = []
    for v in range(0, H, step):
        for u in range(0, W, step):
            d = depth[v, u]
            if not (d_min < d < d_max):
                continue
            # ----- 反投影到相机系 (back-project, 带 skew A12) -----
            y_cam = (v - v0) * d / A22
            x_cam = ((u - u0) * d - A12 * y_cam) / A11
            z_cam = d
            # ----- 相机系 → 雷达系 -----
            X = Tlc[0,0]*x_cam + Tlc[0,1]*y_cam + Tlc[0,2]*z_cam + Tlc[0,3]
            Y = Tlc[1,0]*x_cam + Tlc[1,1]*y_cam + Tlc[1,2]*z_cam + Tlc[1,3]
            Z = Tlc[2,0]*x_cam + Tlc[2,1]*y_cam + Tlc[2,2]*z_cam + Tlc[2,3]
            # ----- 取色 (BGR order) -----
            b, g, r = rgb_undist[v, u]
            out.append((X, Y, Z, r, g, b))
    return out
```

仅此一段循环，**无任何畸变处理、无 remap、无 LUT**。

### 3.3 反投影公式（带 skew）/ Back-projection (with skew)

针孔 `K = [[A11,A12,u0],[0,A22,v0],[0,0,1]]`，`[u,v,1]ᵀ = (1/d)·K·[x_cam,y_cam,z_cam]ᵀ`：
```
y_cam = (v - v0) * d / A22
x_cam = ((u - u0) * d - A12 * y_cam) / A11
z_cam = d
```
若标定 `A12 ≈ 0`（典型情况），可进一步简化：
```
x_cam = (u - u0) * d / A11
y_cam = (v - v0) * d / A22
z_cam = d
```

---

## 4. 库无关 C 参考实现 / Library-free C Reference

```c
#include <stdint.h>

typedef struct { float x, y, z; uint8_t r, g, b; } odin_xyzrgb_t;

/* Inputs:
 *   depth        : [H*W] float32, meters, 0 = invalid
 *   rgb_undist   : [H*W*3] uint8 BGR, ALREADY undistorted, pixel-aligned with depth
 *   W, H         : image dimensions (e.g. 1600, 1296)
 *   A11..v0      : pinhole intrinsics (same K used to produce depth & rgb_undist)
 *   Tlc          : 4x4 row-major, camera -> lidar
 *   step         : pixel sampling stride (e.g. 5)
 *   d_min,d_max  : depth filter (e.g. 0.1, 100.0)
 *   out          : caller-allocated buffer, capacity >= ceil(H/step)*ceil(W/step)
 * Returns: number of points written.
 */
int depth_to_dense_cloud(const float*   depth,
                         const uint8_t* rgb_undist,
                         int W, int H,
                         float A11, float A12, float A22,
                         float u0,  float v0,
                         const float Tlc[16],
                         int   step,
                         float d_min, float d_max,
                         odin_xyzrgb_t* out)
{
    int n = 0;
    for (int v = 0; v < H; v += step) {
        for (int u = 0; u < W; u += step) {
            float d = depth[v*W + u];
            if (!(d > d_min && d < d_max)) continue;

            float y_cam = ((float)v - v0) * d / A22;
            float x_cam = (((float)u - u0) * d - A12 * y_cam) / A11;
            float z_cam = d;

            float X = Tlc[0]*x_cam + Tlc[1]*y_cam + Tlc[2] *z_cam + Tlc[3];
            float Y = Tlc[4]*x_cam + Tlc[5]*y_cam + Tlc[6] *z_cam + Tlc[7];
            float Z = Tlc[8]*x_cam + Tlc[9]*y_cam + Tlc[10]*z_cam + Tlc[11];

            const uint8_t* px = rgb_undist + (v*W + u)*3;
            out[n].x = X; out[n].y = Y; out[n].z = Z;
            out[n].b = px[0]; out[n].g = px[1]; out[n].r = px[2];
            ++n;
        }
    }
    return n;
}
```

调用方仅需准备：
1. `depth`（来自上游"点云→深度图"）；
2. `rgb_undist`（订阅去畸变 topic 或上游已处理）；
3. `Tlc = Tcl⁻¹`；
4. 内参 `A11/A12/A22/u0/v0`。

---

## 5. 与完整版的差异 / Diff vs Full Version

| 项 / Item | 完整版 / Full | 简化版 / Simplified |
|---|---|---|
| 反向畸变 LUT (`inv_map_x/y`) | 启动时算 1600×1296×7 阶多项式 | **省略** |
| `cv::remap` | 每帧一次 ~2 ms | **省略** |
| 输入 RGB topic | `/odin1/image` (带畸变) | `/odin1/image_undistort` (已去畸变) |
| 上游开关 | `sendrgb=1` | `sendrgbundistort=1` |
| 内参依赖 | 需要 `A11..v0` + `k2..k7` | **只需 `A11..v0`** |
| 算法核心 | 反投影 + 取色 | 反投影 + 取色（完全相同） |

> 注意：去畸变本身没有消失，只是搬到了上游（`sendrgbundistort=1` 由 host SDK 启动时构建一次 LUT，每帧 remap）。如果你的目标系统里 RGB 已经是别的渠道吐出来的去畸变图，那就是真省了。
>
> Note: the undistortion didn't disappear—it moved upstream. If your downstream system happens to consume an already-undistorted image from elsewhere, the saving is real.

---

## 6. 测试与可视化 / Testing & Visualization

### 6.1 自检 / Sanity check
取一帧 `cloud_raw` 中任意点 `P_lidar`：
1. 上游"点云→深度图"会把它投到深度图像素 `(u, v)` 处，深度值为 `z_cam`；
2. 用本文档反投影该 `(u, v, z_cam)` → `Tlc·K⁻¹·[u,v,1]·z_cam`；
3. 应得到原始 `P_lidar`，误差 `< 10⁻³ m`（取整 + float32 量化误差）。

### 6.2 可视化（Open3D）/ Visualization
```python
import open3d as o3d, numpy as np
xyz = np.array([(p.x,p.y,p.z) for p in cloud], dtype=np.float64)
rgb = np.array([(p.r,p.g,p.b) for p in cloud], dtype=np.float64) / 255.0
pcd = o3d.geometry.PointCloud()
pcd.points = o3d.utility.Vector3dVector(xyz)
pcd.colors = o3d.utility.Vector3dVector(rgb)
o3d.visualization.draw_geometries([pcd])
```

或在 RViz 订阅发布出的 `PointCloud2` topic（`fixed_frame` = lidar frame）。

---

## 7. Caveats

1. **必须确保 `depth` 与 `rgb_undist` 使用同一份 `K`**：内参不一致会出现颜色与几何错位。
2. **稠密 ≠ 高精度**：上游深度图本身是稀疏 dTOF + 邻域膨胀 + 上采样的结果，本步反投影出的"稠密"点中绝大多数是上采样产生的复制点，**几何精度等同 `cloud_raw + 邻域复制`**。对几何精度敏感的下游请直接使用 `cloud_raw`。
3. **`Tcl` 必须是刚体变换**：才能用 `[Rᵀ, -Rᵀt; 0, 1]` 求逆；当前 SDK 标定满足。
4. **OpenCV / 多数库颜色顺序为 BGR**：写入 PCL `PointXYZRGB` 时按 `point.r=r; point.g=g; point.b=b`。
5. **`A12`（skew）几乎为 0**：典型标定下可用简化公式；保留 `A12` 项不会增加成本（多 1 mul + 1 add）。

---

## 8. 一句话总结 / TL;DR

> 输入 = **已去畸变 RGB** + **深度图**（同 K、同分辨率、像素对齐）+ 内参 `A11..v0` + 外参 `Tcl`。每个像素：`p_cam = K⁻¹·[u,v,1]·d`，`p_lidar = Tcl⁻¹·p_cam`，颜色直接 `rgb_undist[v,u]`。无 LUT，无 remap。
>
> Inputs = undistorted RGB + depth (same K, same resolution, pixel-aligned) + intrinsics + extrinsics. Per pixel: back-project with `K⁻¹`, transform with `Tcl⁻¹`, sample color directly. No LUT, no remap.
