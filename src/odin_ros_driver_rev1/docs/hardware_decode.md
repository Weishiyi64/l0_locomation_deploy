# 硬解码验证 / Hardware Decode Verification

> [← 返回 README](../README.md)

**中文**：驱动启动时输出的 `[JPEG Decoder] Using ...` 日志仅表征编译期代码分支选择，不足以证明硬件 JPEG 解码器**实际处于硬件加速工作状态**——当硬件 IP 初始化失败时，HybridDecoder 会透明回退至软件解码路径，而日志输出不会显式标注此次回退。以下方法提供**内核与系统级**的客观证据，用户态进程无法伪造此类信息，适用于客户现场验证与交付验收。

**English**: The `[JPEG Decoder] Using ...` log at driver startup only indicates the compile-time code path selection; it does NOT constitute proof that the hardware JPEG decoder is **actually operating in hardware-accelerated mode** — upon hardware IP initialization failure, HybridDecoder transparently falls back to the software decoding path without explicitly annotating the fallback in the log. The methods below provide **kernel- and system-level** objective evidence that user-space processes cannot forge, suitable for on-site customer verification and acceptance testing.

## Rockchip 平台 / Rockchip platform

### 1) `/proc/interrupts` —— 最可靠的内核级证据 / Most reliable kernel-level evidence

```bash
# 确认 jpegd 中断号 / Find the jpegd IRQ line
grep -E "jpegd|vdpu|rkvdec" /proc/interrupts

# 实时观察中断计数 / Watch counter live
watch -n 1 'grep "jpegd" /proc/interrupts'
```

中文：`post_process_node` 启动后，`jpegd` 行的硬件中断计数列**应呈持续递增趋势**（每解码一帧约 +1）。若计数保持不变，表明硬件解码 IP 核未被触发，系统大概率已回退至软件解码路径。

English: After `post_process_node` starts, the `jpegd` hardware interrupt counter **should exhibit continuous increment** (≈ +1 per decoded frame). A stationary counter indicates the hardware decoder IP core is not being invoked — the system has likely fallen back to the software decoding path.

### 2) `/proc/mpp_service` 内核级性能打点 / MPP kernel-level performance tracing

```bash
# 启用 / Enable
echo 1 | sudo tee /proc/mpp_service/timing_en
echo 1 | sudo tee /proc/mpp_service/jpegd/timing_check

# … 跑一段 driver / run the driver for a while …

# 查看 jpegd 硬件 IP 核执行耗时 / Inspect hardware IP core execution time
sudo dmesg | grep -E 'rk_vcodec|mpp_service' | tail -30

# 关闭 / Disable
echo 0 | sudo tee /proc/mpp_service/jpegd/timing_check
echo 0 | sudo tee /proc/mpp_service/timing_en
```

输出中 `irq - run end = XXX us` 字段表征 jpegd 硬件 IP 核的实际执行耗时，由内核中断处理函数（IRQ handler）直接采样记录 / The `irq - run end = XXX us` fields reflect the actual jpegd hardware IP core execution time, sampled directly by the kernel interrupt handler (IRQ handler).

### 3) `lsof` 检查进程对硬件设备节点的句柄占用 / Inspect process file-handle occupancy of hardware device nodes

```bash
sudo lsof /dev/mpp_service | grep post_process
```

中文：若 `post_process_node` 出现在查询结果中，表明该进程已成功打开 RK 硬件解码设备节点并持有文件句柄；软件解码路径不会对此设备节点进行任何操作。

English: If `post_process_node` appears in the query results, it indicates the process has successfully opened the RK hardware decoder device node and holds a file handle to it; the software decoding path never accesses this device node.

### 4) CPU 占用对比 / CPU usage comparison

```bash
htop -p $(pgrep post_process_node)
```

| 模式 / Mode | 1080p MJPEG @30fps 单核占用 / Single-core load |
| --- | --- |
| 软解 / Software | ~80–100% |
| 硬解 / Hardware | ~10–20%（仅余 NV12→BGR 色彩空间转换 + memcpy 拷贝开销） |

## Jetson 平台 / Jetson platform

| 等价工具 / Equivalent tool | 命令 / Command |
| --- | --- |
| `jtop` 看 NVJPG 负载 / `jtop` (NVJPG load) | `sudo -H pip3 install jetson-stats && sudo jtop` |
| `tegrastats`（无需 jtop 依赖）/ `tegrastats` (no jtop dependency) | `sudo tegrastats --interval 500 \| grep -E "NVJPG"` |
| 中断计数 / IRQ counter | `watch -n 1 'grep -i nvjpg /proc/interrupts'` |
| 设备节点占用 / Device-node ownership | `sudo lsof /dev/nvhost-nvjpg \| grep post_process` |
| 硬件时钟频率非零 / Non-zero HW clock frequency | `sudo cat /sys/kernel/debug/clk/nvjpg/clk_rate` |

### Jetson H.264 NVDEC 运行期开关 / NVDEC runtime switches

| 环境变量 / Env var | 默认 / Default | 说明 / Description |
| --- | --- | --- |
| `ODIN_NVDEC_STATS` | `0`（关闭 / off） | 置 `1` 输出每 5 秒一行的解码流水线统计（`fed` / `decoded` / `delivered` / `starved` / `dropped` / `queue_max` / `input_gap_max` / `decode_gap_max` / `convert_max`）与帧间隔告警，用于定位卡顿与延迟 / Set to `1` for a 5 s pipeline stats line plus frame-gap warnings |
| `ODIN_NVDEC_MAX_LATENCY_FRAMES` | `2` | 解码帧 FIFO 允许的常驻积压帧数；调大更平滑但延迟增加（每帧约 33 ms），调小延迟更低但突发时可能丢帧 / Steady-state backlog allowed in the decoded-frame FIFO |
| `ODIN_NVDEC_KEEP_DPB` | `0` | 置 `1` 保留解码器 DPB。默认禁用 DPB：ODIN 码流无 B 帧，DPB 只会让解码器攒帧后成批输出（实测约 400 ms 一批），造成明显延迟 / Set to `1` to keep the decoder DPB enabled |

```bash
# 排查卡顿/延迟时打开统计
ODIN_NVDEC_STATS=1 ros2 launch odin_ros_driver_rev1 odin1Lite.launch.py
```

正常状态下统计行应满足：`fed ≈ decoded ≈ delivered ≈ 帧率`，`starved`/`dropped` 为 0，`queue_max ≤ 3`，`decode_gap_max` 与 `input_gap_max` 接近 / A healthy line has matching rates, zero starved/dropped, `queue_max ≤ 3`, and `decode_gap_max` tracking `input_gap_max`.

## Intel 平台（x86_64）/ Intel platform (x86_64)

### 1) 典型症状：驱动栈不适配导致 VAAPI 初始化失败 / Typical symptom: VAAPI init failure from an outdated driver stack

**中文**：Intel 核显平台上，当用户态媒体驱动栈（`libigdgmm12` + `intel-media-va-driver` + `libva`）版本过旧、无法识别当前 GPU 型号时，VAAPI 将在驱动层直接初始化失败。`FfmpegH264Decoder` 会**静默**依次探测全部硬件后端（探测过程中不打印逐项失败日志，FFmpeg 内部日志也被捕获而非直接输出）；仅当**所有后端均失败**、回退软件解码时，才输出一条汇总告警（列出每个后端的失败原因 + FFmpeg 捕获到的诊断行）；若任一后端可用，则只输出一行成功日志、无任何错误输出。全部失败时的日志特征如下：

**English**: On Intel iGPU platforms, when the user-space media driver stack (`libigdgmm12` + `intel-media-va-driver` + `libva`) is too old to recognise the GPU model, VAAPI fails to initialise at the driver layer. `FfmpegH264Decoder` probes every HW backend **silently** (no per-backend failure lines while probing — FFmpeg's internal logs are captured instead of printed); only when **all backends fail** and software decode takes over does it emit a single consolidated warning listing each backend's failure reason plus FFmpeg's captured diagnostics. When any backend is available exactly one success line is printed and nothing else. The all-failed log signature is:

```
[WARN] [H264|camera0] [init] no hardware H.264 backend available — 4 candidate(s) tried, all failed:
[WARN] [H264|camera0] [init]   CUVID: av_hwdevice_ctx_create failed (device=<default>)
[WARN] [H264|camera0] [init]   h264: av_hwdevice_ctx_create failed (device=<default>)
[WARN] [H264|camera0] [init]   generic-VAAPI: av_hwdevice_ctx_create failed (device=<default>)
[WARN] [H264|camera0] [init]   generic-VDPAU: av_hwdevice_ctx_create failed (device=<default>)
[WARN] [H264|camera0] [init]   ffmpeg: Cannot load libcuda.so.1
[WARN] [H264|camera0] [init]   ffmpeg: libva: /usr/lib/x86_64-linux-gnu/dri/iHD_drv_video.so init failed
[WARN] [H264|camera0] [init]   ffmpeg: libva: /usr/lib/x86_64-linux-gnu/dri/i965_drv_video.so init failed
[WARN] [H264|camera0] [init]   ffmpeg: Failed to initialise VAAPI connection: -1 (unknown libva error).
[WARN] [H264|camera0]  FFmpeg H.264 SOFTWARE decoding in use (no HW backend found)
```

**中文**：其中 `ffmpeg: libva: ... init failed` / `Failed to initialise VAAPI connection` 行（源自探测期间捕获的 FFmpeg 原始输出）是驱动不适配的**直接证据**；`ffmpeg: Cannot load libcuda.so.1` 等 CUDA/VDPAU 失败行在无 NVIDIA 显卡的机器上属正常探测噪音，无需处理。

**English**: The `ffmpeg: libva: ... init failed` / `Failed to initialise VAAPI connection` lines (replayed from FFmpeg output captured during probing) are direct evidence of driver incompatibility. The `ffmpeg: Cannot load libcuda.so.1` CUDA/VDPAU lines are benign probe noise on machines without an NVIDIA GPU.

### 2) 实测案例（Meteor Lake 核显 + Ubuntu 22.04）/ Case study

**中文**：

- **GPU**：Intel Meteor Lake 核显（PCI ID `8086:7D55`）；
- **内核侧正常**：Ubuntu 22.04 + 内核 6.8，i915 已正常驱动（开机加载 `mtl_gsc_1.bin` GSC 固件）——故障**完全在用户态媒体驱动栈**，与内核无关；
- **过旧组件**：Ubuntu 官方源 `libigdgmm12` 22.1.2 + `intel-media-va-driver` 22.3.1（均为 2022 年版本）；
- **根因**：gmmlib 22.1.2 的二进制中不含 `0x7D55` 设备 ID，iHD 无法识别该 GPU → 初始化失败；i965 驱动最高仅支持 Gen9.5（Broadwell~Coffee Lake）→ 同样失败；
- **修复版本要求**：Meteor Lake（Xe-LPG）需要 gmmlib ≥ 22.3 且 media-driver ≥ 23.1（Intel 官方仓库 jammy 源提供 22.5.2 / 24.3.4）。

**English**:

- **GPU**: Intel Meteor Lake iGPU (PCI ID `8086:7D55`);
- **Kernel side is fine**: Ubuntu 22.04 + kernel 6.8, i915 drives the GPU correctly (loads `mtl_gsc_1.bin` at boot) — the failure is entirely in the **user-space media stack**;
- **Outdated packages**: Ubuntu archive `libigdgmm12` 22.1.2 + `intel-media-va-driver` 22.3.1 (both from 2022);
- **Root cause**: gmmlib 22.1.2 contains no `0x7D55` device ID → iHD cannot recognise the GPU; i965 tops out at Gen9.5 → fails too;
- **Required versions**: Meteor Lake (Xe-LPG) needs gmmlib ≥ 22.3 and media-driver ≥ 23.1 (Intel repo ships 22.5.2 / 24.3.4 for jammy).

### 3) 诊断步骤 / Diagnosis

```bash
# 确认 GPU 型号（记下 PCI ID，如 8086:7D55 / note the PCI ID）
lspci -nn | grep -Ei 'vga|display'

# 直接检验 VAAPI 初始化（最直接；失败时打印 iHD_drv_video.so init failed）
sudo apt install -y vainfo
vainfo

# 确认 gmmlib 是否认识当前 GPU（以 0x7D55 为例，返回 True 即支持）
python3 -c "d=open('/usr/lib/x86_64-linux-gnu/libigdgmm.so.12','rb').read(); print(bytes([0x55,0x7d]) in d)"
```

**中文**：`vainfo` 仅需 render 节点权限（`/dev/dri/renderD128`，用户需在 `render` 组或经 ACL 授权）。若输出 `vaInitialize` 失败或 `iHD_drv_video.so init failed`，即可判定驱动栈不适配，按下节升级。

**English**: `vainfo` only needs the render node (`/dev/dri/renderD128`; the user must be in the `render` group or granted via ACL). A failed `vaInitialize` or `iHD_drv_video.so init failed` confirms an incompatible driver stack — proceed to the next section.

### 4) 更新驱动（Ubuntu 22.04 实测有效）/ Driver upgrade

**中文**：Ubuntu 22.04 官方源内 `intel-media-va-driver` 停留在 22.3.1（不识别新平台），需引入 Intel 官方 GPU 仓库后整套升级：

**English**: The Ubuntu 22.04 archive ships media driver 22.3.1, which does not recognise newer platforms. Add Intel's official GPU repository and upgrade the whole stack:

```bash
# 1. 添加 Intel 官方 GPU 仓库（jammy）/ Add Intel GPU repository (jammy)
wget -qO - https://repositories.intel.com/gpu/intel-graphics.key | \
  sudo gpg --dearmor --batch --yes -o /usr/share/keyrings/intel-graphics-archive-keyring.gpg
echo "deb [signed-by=/usr/share/keyrings/intel-graphics-archive-keyring.gpg arch=amd64] https://repositories.intel.com/gpu/ubuntu jammy client" | \
  sudo tee /etc/apt/sources.list.d/intel-gpu-jammy.list
sudo apt update

# 2. 整套升级：驱动 + GMM + libva 全家桶必须一起升级 / Upgrade the whole stack together
sudo apt install intel-media-va-driver-non-free libigdgmm12 \
  libva2 libva-drm2 libva-x11-2 libva-wayland2
```


**中文**：升级效果（实测）：`libigdgmm12` 22.1.2 → 22.5.2，iHD 22.3.1 → 24.3.4，libva 2.14 → 2.22。

**English**: Verified upgrade result: `libigdgmm12` 22.1.2 → 22.5.2, iHD 22.3.1 → 24.3.4, libva 2.14 → 2.22.

### 5) 升级后验证 / Post-upgrade verification

```bash
# VAAPI 初始化成功 + H.264 解码 Profile 存在 / VAAPI initialises and H.264 decode profiles are present
vainfo | grep -E 'Driver version|VAProfileH264'

# 期望输出 / Expected:
# vainfo: Driver version: Intel iHD driver for Intel(R) Gen Graphics - 24.3.4
# VAProfileH264High : VAEntrypointVLD
```

**中文**：重启驱动后，日志应从软件解码警告变为硬件解码初始化成功：

**English**: After restarting the driver the log should change from the software warning to:

```
[H264|camera0] [init] FFmpeg hardware decoder: VAAPI (h264_vaapi)
```

**中文**：运行期可进一步交叉确认（可选）：

**English**: Runtime cross-check (optional):

```bash
sudo apt install intel-gpu-tools
sudo intel_gpu_top                                 # 观察 Video 引擎负载 / watch video-engine load
sudo lsof /dev/dri/renderD128 | grep post_process  # 确认进程持有 GPU 句柄 / confirm GPU node held
```

### 6) x86 平台日志噪音说明 / Benign log noise on x86

**中文**：硬件后端探测过程静默进行，以下行**不会**在探测期间单独出现；仅当全部后端失败时，才会作为汇总报告中 `ffmpeg:` 前缀行出现。

**English**: HW backend probing is silent — the lines below do NOT appear on their own while probing; they show up only inside the consolidated all-failed report with the `ffmpeg:` prefix.

| 日志 / Log | 原因 / Cause | 需处理 / Action |
| --- | --- | --- |
| `ffmpeg: Cannot load libcuda.so.1`、CUVID 失败 | 无 NVIDIA GPU / no NVIDIA GPU | 否 / No |
| `ffmpeg: libvdpau_nvidia.so ... No such file` | 无 NVIDIA VDPAU 后端 / no NVIDIA VDPAU backend | 否 / No |
| `JPEG decoder: OpenCV Software Decoder` | x86 上 JPEG 解码器为编译期软解（硬解分支仅 Jetson/RK 编译启用）/ JPEG decode is software-only on x86 by design | 否 / No |

## 多维度交叉验证方案 / Multi-dimensional Cross-verification

**中文**：建议在客户现场部署中开启 3 个终端同步观测，进行多维度交叉验证 —— **硬件中断计数持续递增（Intel 平台对应 GPU Video 引擎负载非零）+ 设备节点句柄被占用 + CPU 占用率维持低位**，三项指标同时成立方可确认硬件解码通路实际生效。

**English**: For on-site customer deployment, it is recommended to open 3 concurrent terminals for simultaneous multi-dimensional cross-verification — **continuous hardware interrupt counter increment (non-zero GPU video-engine load on Intel platforms) + device node handle occupancy + sustained low CPU utilization**. All three indicators must hold simultaneously to confirm the hardware decoding path is effectively engaged.

```bash
# Terminal 1: launch driver
ros2 launch odin_ros_driver_rev1 odin1Lite.launch.py

# Terminal 2 (RK): watch IRQ counter
watch -n 1 'grep -E "jpegd|vdpu" /proc/interrupts'
# Terminal 2 (Jetson):
sudo tegrastats --interval 500 | grep -E "NVJPG"
# Terminal 2 (Intel): watch GPU video-engine load (needs intel-gpu-tools)
sudo intel_gpu_top

# Terminal 3 (RK): confirm device-node held
sudo lsof /dev/mpp_service | grep post_process
# Terminal 3 (Jetson):
sudo lsof /dev/nvhost-nvjpg | grep post_process
# Terminal 3 (Intel):
sudo lsof /dev/dri/renderD128 | grep post_process
```
