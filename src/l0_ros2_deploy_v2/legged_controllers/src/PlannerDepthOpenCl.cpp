#include "PlannerDepthOpenCl.h"

#include <cstdint>
#include <cstring>
#include <limits>
#include <stdexcept>

#define CL_TARGET_OPENCL_VERSION 120
#include <CL/cl.h>
#include "PlannerDepthKernel.h"
#include "pointcloud_depth_core.hpp"

namespace legged_controllers {
namespace {
void checked(cl_int error, const char* operation) {
  if (error != CL_SUCCESS)
    throw std::runtime_error(std::string(operation)+": OpenCL error "+std::to_string(error));
}
template <class T, cl_int (*Release)(T)> struct Handle {
  T value{};
  ~Handle() { if (value) Release(value); }
  Handle() = default;
  Handle(const Handle&) = delete;
  Handle& operator=(const Handle&) = delete;
};
using Buffer = Handle<cl_mem, clReleaseMemObject>;
using Kernel = Handle<cl_kernel, clReleaseKernel>;
constexpr std::size_t kMaxPoints = 49152, kCells = 53*64, kPixels = 2560;
constexpr std::size_t kFirst = 7007, kGrid = 14014, kClosed = 17406,
                      kHit = 20798, kFlags = 23358, kCounts = 25918, kFrameWords = 25920;
}

struct PlannerDepthOpenCl::Impl {
  // Declaration order ensures buffers/kernels die before their context.
  Handle<cl_context, clReleaseContext> context;
  Handle<cl_command_queue, clReleaseCommandQueue> queue;
  Handle<cl_program, clReleaseProgram> program;
  Kernel clear, build, morphology, raycast;
  Buffer xyz, matrix, frame, point_repairs, samples;
  std::string name;
  bool pending{false};
  struct Drain {
    Impl& owner;
    ~Drain() { if (owner.pending) { clFinish(owner.queue.value); owner.pending = false; } }
  };

  Impl() {
    cl_uint platforms_count = 0;
    checked(clGetPlatformIDs(0, nullptr, &platforms_count), "enumerate OpenCL platforms");
    std::vector<cl_platform_id> platforms(platforms_count);
    if (platforms.empty()) throw std::runtime_error("no OpenCL platforms");
    checked(clGetPlatformIDs(platforms_count, platforms.data(), nullptr), "get OpenCL platforms");
    cl_device_id device = nullptr;
    for (auto platform : platforms) {
      // Never silently run an OpenCL CPU device and call it GPU acceleration.
      const cl_int error = clGetDeviceIDs(platform, CL_DEVICE_TYPE_GPU, 1, &device, nullptr);
      if (error == CL_SUCCESS) break;
      if (error != CL_DEVICE_NOT_FOUND) checked(error, "enumerate GPU");
    }
    if (!device) throw std::runtime_error("no OpenCL GPU device");
    std::size_t size = 0;
    checked(clGetDeviceInfo(device, CL_DEVICE_NAME, 0, nullptr, &size), "GPU name length");
    std::vector<char> text(size);
    checked(clGetDeviceInfo(device, CL_DEVICE_NAME, size, text.data(), nullptr), "GPU name");
    name = text.data();
    cl_int error;
    context.value = clCreateContext(nullptr, 1, &device, nullptr, nullptr, &error);
    checked(error, "create context");
    queue.value = clCreateCommandQueue(context.value, device, 0, &error);
    checked(error, "create in-order command queue");
    const char* source = kPlannerDepthKernel;
    program.value = clCreateProgramWithSource(context.value, 1, &source, nullptr, &error);
    checked(error, "create program");
    // No fast-relaxed-math, mad-enable, native math or FP16.
    error = clBuildProgram(program.value, 1, &device, "-cl-std=CL1.2", nullptr, nullptr);
    if (error != CL_SUCCESS) {
      checked(clGetProgramBuildInfo(program.value, device, CL_PROGRAM_BUILD_LOG, 0, nullptr, &size), "build log length");
      std::vector<char> log(size+1, '\0');
      checked(clGetProgramBuildInfo(program.value, device, CL_PROGRAM_BUILD_LOG, size, log.data(), nullptr), "build log");
      throw std::runtime_error("OpenCL kernel build failed: "+std::string(log.data()));
    }
    auto kernel = [&](Kernel& target, const char* kernel_name) {
      target.value = clCreateKernel(program.value, kernel_name, &error);
      checked(error, kernel_name);
    };
    kernel(clear, "clear_frame"); kernel(build, "build_cloud"); kernel(morphology, "close_grid"); kernel(raycast, "raycast");
    auto buffer = [&](Buffer& target, std::size_t bytes) {
      target.value = clCreateBuffer(context.value, CL_MEM_READ_WRITE, bytes, nullptr, &error);
      checked(error, "allocate persistent buffer");
    };
    buffer(xyz, kMaxPoints*3*sizeof(float)); buffer(matrix, 40*sizeof(float));
    buffer(frame, kFrameWords*sizeof(cl_uint)); buffer(point_repairs, 2*kMaxPoints*sizeof(cl_uint));
    buffer(samples, 97*sizeof(float));
    std::array<float, 97> distances;
    for (int step = 0; step <= 96; ++step) distances[step] = static_cast<float>(.2+.05*step);
    write(samples, distances.data(), sizeof(distances));
  }

  void write(Buffer& buffer, const void* data, std::size_t size, bool blocking = true, std::size_t offset = 0) {
    if (size) {
      checked(clEnqueueWriteBuffer(queue.value, buffer.value, blocking ? CL_TRUE : CL_FALSE, offset, size, data, 0, nullptr, nullptr), "upload");
      pending = !blocking;
    }
  }
  void read(Buffer& buffer, void* data, std::size_t size, bool blocking = true, std::size_t offset = 0) {
    if (size) {
      checked(clEnqueueReadBuffer(queue.value, buffer.value, blocking ? CL_TRUE : CL_FALSE, offset, size, data, 0, nullptr, nullptr), "download");
      pending = !blocking;
    }
  }
  template <class T> void arg(Kernel& kernel, cl_uint index, const T& value) {
    checked(clSetKernelArg(kernel.value, index, sizeof(T), &value), "set kernel argument");
  }
  void run(Kernel& kernel, std::size_t size, std::size_t local = 0) {
    if (size) {
      // Filtered clouds have arbitrary point counts. Letting the runtime choose
      // a divisor can yield one-thread workgroups for prime-sized frames.
      if (!local) local = 64;
      size = ((size+local-1)/local)*local;
      checked(clEnqueueNDRangeKernel(queue.value, kernel.value, 1, nullptr, &size, &local, 0, nullptr,
                                    nullptr), "enqueue kernel");
      pending = true;
    }
  }

  PlannerDepthOpenCl::Result process(const PlannerDepthGeometry& geometry,
      const std::vector<float>& points, const Eigen::Isometry3d& pose) {
    if (points.size()%3 || points.size()/3 > kMaxPoints)
      throw std::runtime_error("GPU depth requires at most 49152 packed XYZ points");
    const auto state = geometry.prepareFusion(pose);
    if (!state.world_from_lidar.matrix().allFinite() || !state.world_from_camera.matrix().allFinite() ||
        !state.inverse.allFinite()) throw std::runtime_error("non-finite depth geometry");
    odin::depth_core::DepthParams parameters{};
    parameters.A11 = geometry.intrinsics(0,0); parameters.A12 = geometry.intrinsics(0,1);
    parameters.A22 = geometry.intrinsics(1,1); parameters.u0 = geometry.intrinsics(0,2);
    parameters.v0 = geometry.intrinsics(1,2); parameters.scale = 7.f;
    for (int row = 0; row < 4; ++row)
      for (int col = 0; col < 4; ++col) parameters.Tcl[row*4+col] = geometry.camera_from_lidar(row,col);
    float kcl[16], coefficients[40]{};
    odin::depth_core::compute_kcl(parameters, kcl);
    for (int j = 0; j < 4; ++j) kcl[j] += parameters.A12*parameters.Tcl[4+j]/parameters.scale;
    std::copy(kcl, kcl+12, coefficients);
    Eigen::Matrix<double, 3, 4> local = state.world_from_lidar.matrix().topRows<3>();
    local.row(0) = state.c*state.world_from_lidar.matrix().row(0)+state.s*state.world_from_lidar.matrix().row(1);
    local.row(1) = -state.s*state.world_from_lidar.matrix().row(0)+state.c*state.world_from_lidar.matrix().row(1);
    const auto delta = (state.world_from_lidar.translation()-pose.translation()).eval();
    local(0,3) = state.c*delta.x()+state.s*delta.y();
    local(1,3) = -state.s*delta.x()+state.c*delta.y();
    for (int row = 0; row < 3; ++row)
      for (int col = 0; col < 4; ++col) coefficients[12+4*row+col] = static_cast<float>(local(row,col));
    const Eigen::Matrix3d world_rays = state.world_from_camera.linear()*state.inverse;
    Eigen::Matrix3d local_rays = world_rays;
    local_rays.row(0) = state.c*world_rays.row(0)+state.s*world_rays.row(1);
    local_rays.row(1) = -state.s*world_rays.row(0)+state.c*world_rays.row(1);
    for (int row = 0; row < 3; ++row)
      for (int col = 0; col < 3; ++col) coefficients[24+4*row+col] = static_cast<float>(local_rays(row,col));
    const auto start = (state.world_from_camera.translation()-pose.translation()).eval();
    coefficients[36] = static_cast<float>(state.c*start.x()+state.s*start.y());
    coefficients[37] = static_cast<float>(-state.s*start.x()+state.c*start.y());
    coefficients[38] = static_cast<float>(state.world_from_camera.translation().z());
    const double magnitude = 1.+pose.translation().cwiseAbs().sum()+
        state.world_from_lidar.translation().cwiseAbs().sum()+state.world_from_camera.translation().cwiseAbs().sum();
    const float double_guard = static_cast<float>(256.*std::numeric_limits<double>::epsilon()*
        (magnitude+320.*world_rays.cwiseAbs().sum()));
    const cl_uint count = static_cast<cl_uint>(points.size()/3);
    std::array<cl_uint, kFrameWords> data;
    auto* last = data.data(); auto* first = data.data()+kFirst;
    auto* cells = data.data()+kGrid;
    std::vector<cl_uint> projection_ids, occupancy_ids;
    // All asynchronous host storage is declared before Drain, including on exceptions.
    Drain drain{*this};
    arg(clear, 0, frame.value); run(clear, 91*77);
    write(xyz, points.data(), points.size()*sizeof(float), false);
    write(matrix, coefficients, sizeof(coefficients), false);
    arg(build, 0, xyz.value); arg(build, 1, count); arg(build, 2, matrix.value); arg(build, 3, double_guard);
    arg(build, 4, frame.value); arg(build, 5, point_repairs.value); run(build, count);
    read(frame, cells, (kFrameWords-kGrid)*sizeof(cl_uint), true, kGrid*sizeof(cl_uint));
    const cl_uint projection_count = data[kCounts], occupancy_count = data[kCounts+1];
    if (projection_count > count || occupancy_count > count) throw std::runtime_error("invalid GPU repair count");
    occupancy_ids.resize(occupancy_count);
    read(point_repairs, occupancy_ids.data(), occupancy_ids.size()*sizeof(cl_uint), true, kMaxPoints*sizeof(cl_uint));
    for (auto id : occupancy_ids) {
      if (id >= count) throw std::runtime_error("invalid occupancy repair index");
      const Eigen::Vector3d point(points[3*id], points[3*id+1], points[3*id+2]);
      const int index = state.cell(state.world_from_lidar*point);
      if (index >= 0) cells[index] = 1;
    }
    if (!occupancy_ids.empty()) write(frame, cells, kCells*sizeof(cl_uint), false, kGrid*sizeof(cl_uint));
    arg(morphology, 0, frame.value); run(morphology, 4*7*128, 128);
    arg(raycast, 0, frame.value); arg(raycast, 1, matrix.value);
    arg(raycast, 2, samples.value); arg(raycast, 3, double_guard); run(raycast, kPixels);
    projection_ids.resize(projection_count);
    read(point_repairs, projection_ids.data(), projection_ids.size()*sizeof(cl_uint), false);
    read(frame, data.data(), sizeof(data));
    for (auto id : projection_ids) {
      if (id >= count) throw std::runtime_error("invalid projection repair index");
      float xp, yp, zp;
      odin::depth_core::mat4_apply_kcl(kcl, points[3*id], points[3*id+1], points[3*id+2], &xp, &yp, &zp);
      if (!std::isfinite(xp) || !std::isfinite(yp) || !std::isfinite(zp) || zp <= 0.f) continue;
      const float u = xp/zp+.5f, v = yp/zp+.5f;
      if (!std::isfinite(u) || !std::isfinite(v) || u < 0.f || u >= 91.f || v < 0.f || v >= 77.f) continue;
      const int x = static_cast<int>(u), y = static_cast<int>(v);
      last[y*91+x] = std::max(last[y*91+x], id+1);
      first[y*91+x] = std::min(first[y*91+x], id+1);
    }
    PlannerDepthOpenCl::Result result{};
    const auto unset = std::numeric_limits<cl_uint>::max();
    PlannerDepthGeometry::Grid closed_grid{};
    for (std::size_t i = 0; i < kCells; ++i) closed_grid[i] = data[kClosed+i] != 0;
    for (int pixel = 0; pixel < static_cast<int>(kPixels); ++pixel) {
      const int low = ((pixel/64)*77/40)*91+(pixel%64)*91/64;
      cl_uint selected = last[low];
      if (!selected) {
        selected = unset;
        const int x = low%91, y = low/91;
        for (int dv = -1; dv <= 1; ++dv)
          for (int du = -1; du <= 1; ++du) {
            const int nx = x+du, ny = y+dv;
            if (nx >= 0 && nx < 91 && ny >= 0 && ny < 77)
              selected = std::min(selected, first[ny*91+nx]);
          }
      }
      float raw = 0.f;
      if (selected != unset) {
        if (!selected || selected > count) throw std::runtime_error("invalid projected point index");
        const auto id = selected-1;
        float xp, yp, zp;
        odin::depth_core::mat4_apply_kcl(kcl, points[3*id], points[3*id+1], points[3*id+2], &xp, &yp, &zp);
        raw = std::isfinite(zp) && zp > 0.f ? std::max(.2f, std::min(5.f,zp)) : 0.f;
      }
      float hit;
      std::memcpy(&hit, &data[kHit+pixel], sizeof(hit));
      if (!std::isfinite(hit) || (hit != 0.f && (hit < .2f || hit > 5.f)))
        throw std::runtime_error("invalid GPU ray depth");
      result.fused[pixel] = hit > 0.f ? (raw > 0.f ? std::min(raw,hit) : hit) : raw;
      if (data[kFlags+pixel]) {
        result.fused[pixel] = state.castPixel(pixel, raw, closed_grid);
      }
      const float fused = result.fused[pixel];
      if (!std::isfinite(raw) || !std::isfinite(fused) ||
          (raw != 0.f && (raw < .2f || raw > 5.f)) ||
          (fused != 0.f && (fused < .2f || fused > 5.f)))
        throw std::runtime_error("invalid GPU depth output");
    }
    return result;
  }
};

PlannerDepthOpenCl::PlannerDepthOpenCl() : impl_(std::make_unique<Impl>()) {}
PlannerDepthOpenCl::~PlannerDepthOpenCl() = default;
const std::string& PlannerDepthOpenCl::deviceName() const { return impl_->name; }
PlannerDepthOpenCl::Result PlannerDepthOpenCl::process(const PlannerDepthGeometry& geometry,
    const std::vector<float>& xyz, const Eigen::Isometry3d& world_from_base) {
  return impl_->process(geometry, xyz, world_from_base);
}
}  // namespace legged_controllers
