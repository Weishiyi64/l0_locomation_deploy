// OpenCL C 1.2; no FP64 extension, relaxed math or floating point atomics needed.
#pragma OPENCL FP_CONTRACT OFF

#define ROWS 53
#define COLS 64
#define FIRST 7007
#define GRID 14014
#define CLOSED 17406
#define HIT 20798
#define FLAGS 23358
#define COUNTS 25918

// Return -2 when FP32 cannot safely decide the reference FP64 cell. Such points
// and rays are recomputed on the CPU, including misses before a possible hit.
int guarded_cell(float3 p, float3 error) {
  if (!all(isfinite(p)) || !all(isfinite(error))) return -2;
  if (p.z < .05f-error.z || p.z > 1.20f+error.z) return -1;
  if (fabs(p.z-.05f) <= error.z || fabs(p.z-1.20f) <= error.z) return -2;
  if (p.x < -.30f-error.x || p.x > 5.f+error.x ||
      p.y < -3.2f-error.y || p.y > 3.2f+error.y) return -1;
  if (fabs(p.x+.30f) <= error.x || fabs(p.x-5.f) <= error.x ||
      fabs(p.y+3.2f) <= error.y || fabs(p.y-3.2f) <= error.y) return -2;
  const float2 grid = (p.xy+(float2)(.30f, 3.2f))*10.f;
  const float2 tolerance = 10.f*error.xy+64.f*FLT_EPSILON*(fabs(grid)+1.f);
  if (any(fabs(grid-rint(grid)) <= tolerance)) return -2;
  const int row = convert_int_rtn(grid.x), col = convert_int_rtn(grid.y);
  return row >= 0 && row < ROWS && col >= 0 && col < COLS ? row*COLS+col : -1;
}

// Integer point-order reductions reproduce the sequential projection exactly:
// latest central writer wins; a pixel with no central writer uses the earliest
// neighbouring writer. The winning point's depth is evaluated on the CPU.
void project_points(__global const float* xyz, uint count,
                             __constant const float* matrix,
                             __global uint* centers, __global uint* neighbors,
                             __global uint* repairs, volatile __global uint* repair_count) {
  const uint i = get_global_id(0);
  if (i >= count) return;
  const uint3 bits = vload3(i, (__global const uint*)xyz)&0x7fffffffu;
  if (any(bits >= 0x7f800000u) || all(bits == 0u)) return;
  if (any((bits > 0u)&(bits < 0x00800000u))) {
    repairs[atomic_inc(repair_count)] = i; return;
  }
  const float3 point = vload3(i, xyz);
  float values[3], errors[3];
  for (int axis = 0; axis < 3; ++axis) {
    const float4 m = vload4(axis, matrix);
    values[axis] = ((m.x*point.x+m.y*point.y)+m.z*point.z)+m.w;
    errors[axis] = 64.f*FLT_EPSILON*(dot(fabs(m.xyz), fabs(point))+fabs(m.w)+1.f);
  }
  const float3 p = vload3(0, values), e = vload3(0, errors);
  if (p.z < -e.z) return;
  if (!all(isfinite(p)) || !all(isfinite(e)) || p.z <= e.z) {
    repairs[atomic_inc(repair_count)] = i; return;
  }
  const float reciprocal = 1.f/p.z;
  const float2 ratio = p.xy*reciprocal;
  const float2 uv = ratio+.5f;
  const float2 tolerance = (e.xy+fabs(ratio)*e.z)*(1.f/(p.z-e.z))
                          +64.f*FLT_EPSILON*(fabs(uv)+1.f);
  if (!all(isfinite(uv)) || !all(isfinite(tolerance))) {
    repairs[atomic_inc(repair_count)] = i; return;
  }
  if (any(uv < -tolerance) || any(uv >= (float2)(91.f,77.f)+tolerance)) return;
  if (any(fabs(uv-rint(uv)) <= tolerance)) {
    repairs[atomic_inc(repair_count)] = i; return;
  }
  const int u = convert_int_rtz(uv.x), v = convert_int_rtz(uv.y);
  if (u < 0 || u >= 91 || v < 0 || v >= 77) return;
  atomic_max((volatile __global uint*)(centers+v*91+u), i+1u);
  // Dilation is reduced after projection: the earliest point in each adjacent
  // central pixel is sufficient. Two atomics per point replace nine writes.
  atomic_min((volatile __global uint*)(neighbors+v*91+u), i+1u);

}

void occupy(__global const float* xyz, uint count,
                     __constant const float* matrix, float double_guard,
                     __global uint* grid, __global uint* repairs,
                     volatile __global uint* repair_count) {
  const uint i = get_global_id(0);
  if (i >= count) return;
  const uint3 bits = vload3(i, (__global const uint*)xyz)&0x7fffffffu;
  if (any(bits >= 0x7f800000u) || all(bits == 0u)) return;
  // Mali may flush subnormal arithmetic to zero. Preserve the CPU input
  // semantics by repairing these rare points without FP32 arithmetic.
  if (any((bits > 0u)&(bits < 0x00800000u))) {
    repairs[atomic_inc(repair_count)] = i;
    return;
  }
  const float3 point = vload3(i, xyz);
  float values[3], errors[3];
  for (int axis = 0; axis < 3; ++axis) {
    const float4 m = vload4(axis, matrix);
    values[axis] = ((m.x*point.x+m.y*point.y)+m.z*point.z)+m.w;
    errors[axis] = 64.f*FLT_EPSILON*(dot(fabs(m.xyz), fabs(point))+fabs(m.w)+1.f)
                  +double_guard*(1.f+fabs(point.x)+fabs(point.y)+fabs(point.z));
  }
  const int index = guarded_cell(vload3(0, values), vload3(0, errors));
  if (index == -2) repairs[atomic_inc(repair_count)] = i;
  else if (index >= 0) atomic_or((volatile __global uint*)(grid+index), 1u);
}

__kernel void clear_frame(__global uint* frame) {
  __global uint* centers = frame;
  __global uint* neighbors = frame+FIRST;
  __global uint* grid = frame+GRID;
  __global uint* counts = frame+COUNTS;
  const int i = get_global_id(0);
  if (i < 91*77) { centers[i] = 0; neighbors[i] = 0xffffffffu; }
  if (i < ROWS*COLS) grid[i] = 0;
  if (i < 2) counts[i] = 0;
}

__kernel void build_cloud(__global const float* xyz, uint count,
                          __constant const float* matrix, float double_guard,
                          __global uint* frame, __global uint* repairs) {
  __global uint* grid = frame+GRID;
  __global uint* centers = frame;
  __global uint* neighbors = frame+FIRST;
  volatile __global uint* counts = frame+COUNTS;
  project_points(xyz, count, matrix, centers, neighbors, repairs, counts);
  occupy(xyz, count, matrix+12, double_guard, grid, repairs+49152, counts+1);
}

// Each 128-thread tile closes 8x16 cells. The intermediate dilation halo is
// shared locally; the original outside-grid=false erosion rule is preserved.
__kernel void close_grid(__global uint* frame) {
  __global const uint* source = frame+GRID;
  __global uint* target = frame+CLOSED;
  __local uint expanded[12*20];
  const int lane = get_local_id(0), group = get_group_id(0);
  const int row0 = (group/4)*8, col0 = (group%4)*16;
  for (int i = lane; i < 12*20; i += 128) {
    const int row = row0+i/20-2, col = col0+i%20-2;
    int value = 0;
    if (row >= 0 && row < ROWS && col >= 0 && col < COLS)
      for (int dr = -2; dr <= 2; ++dr)
        for (int dc = -2; dc <= 2; ++dc) {
          if (dr*dr+dc*dc > 4) continue;
          const int r = row+dr, c = col+dc;
          value |= r >= 0 && r < ROWS && c >= 0 && c < COLS && source[r*COLS+c];
        }
    expanded[i] = value;
  }
  barrier(CLK_LOCAL_MEM_FENCE);
  const int row = row0+lane/16, col = col0+lane%16;
  if (row >= ROWS || col >= COLS) return;
  int value = 1;
  for (int dr = -2; dr <= 2; ++dr)
    for (int dc = -2; dc <= 2; ++dc) {
      if (dr*dr+dc*dc <= 4) value &= expanded[(lane/16+2+dr)*20+lane%16+2+dc] != 0;
    }
  target[row*COLS+col] = value;
}

__kernel void raycast(__global uint* frame, __constant const float* parameters,
                      __constant const float* samples, float double_guard) {
  __global const uint* occupied = frame+CLOSED;
  __global float* output = (__global float*)(frame+HIT);
  __global uint* repairs = frame+FLAGS;
  __constant const float4* directions = (__constant const float4*)(parameters+24);
  __constant const float* origin_data = parameters+36;
  const int pixel = get_global_id(0);
  if (pixel >= 2560) return;
  const float3 origin = vload3(0, origin_data);
  const float3 image = (float3)(pixel%64+.5f, pixel/64+.5f, 1.f);
  const float3 direction = (float3)(dot(directions[0].xyz, image),
                                    dot(directions[1].xyz, image), dot(directions[2].xyz, image));
  const float3 direction_error = 64.f*FLT_EPSILON*((float3)(
      dot(fabs(directions[0].xyz), image), dot(fabs(directions[1].xyz), image),
      dot(fabs(directions[2].xyz), image))+1.f);
  output[pixel] = 0.f;
  repairs[pixel] = 0;
  for (int step = 0; step <= 96; ++step) {
    const float z = samples[step];
    const float3 position = origin+direction*z;
    const float3 error = 64.f*FLT_EPSILON*(fabs(origin)+fabs(direction)*z+1.f)
                         +double_guard+direction_error*z;
    const int index = guarded_cell(position, error);
    if (index == -2) { repairs[pixel] = 1; return; }
    if (index >= 0 && occupied[index]) {
      output[pixel] = z;
      return;
    }
  }
}
