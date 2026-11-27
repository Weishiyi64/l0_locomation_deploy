# l0_ros2_deploy_v2

## 工作空间隔离

v1 和 v2 使用独立源码及构建目录，两个版本都基于 ROS 2：

| 版本 | 工作空间 | 部署仓库 / 分支 |
| --- | --- | --- |
| v1 | `~/zy_workspace/l0_deploy_v1_ws` | `src/l0_ros2_deploy_v1` / `feature/ground_promotion` |
| v2 | `~/zy_workspace/l0_deploy_v2_ws` | `src/l0_ros2_deploy_v2` / `feature/hpm_deploy` |

仓库分别为 `git@github.com:luvbotics/l0_ros2_deploy_v1.git` 和
`git@github.com:luvbotics/l0_ros2_deploy_v2.git`。每个工作空间独立生成
`build/`、`install/`、`log/`，新终端只加载所需版本的环境；不要同时 source 两个部署工作空间。
旧目录可保留用于回退，不复制其中的构建缓存、安装结果或日志。

v2 的 `src/` 还需匹配版本的 `luvbot_ros_interfaces`；启用雷达时需要
`odin_ros_driver_rev1` 包，工作空间中只能放置该包的一份源码。
雷达使用 ROS Driver 3.7.0 和 SDK 3.6.0。先在板端构建 SDK 及设备插件，再构建部署包。
各模块接口和升级顺序见 [Odin v3.7 迁移清单](docs/odin_v37_migration.md)。
v2 已移除眼睛表情消息触发动作、眼睛动画字段和串口包。
实机启动链不包含键盘输入节点；手柄继续用于行走、待机增益调节、标定、复位和急停控制。
v1 的眼睛、串口和旧硬件驱动依赖在 v1 工作空间内独立保留。

## 同步源码到机器人

在本仓库目录执行，每次只需更换机器人 IP：

```bash
bash scripts/sync_to_robot.sh 192.168.120.31
# 只预览更新和删除，不实际传输：
bash scripts/sync_to_robot.sh 192.168.120.31 --dry-run
```

脚本从自身位置找到工作空间 `src/`，一起同步 `l0_ros2_deploy_v2`、
`luvbot_ros_interfaces`、`odin_ros_driver_rev1` 和 `hpm_transport_ros2`，固定使用
`forlinx` 用户和远端 `/userdata/forlinx/zy_workspace/l0_deploy_v2_ws/src/`。
四个源码目录均须存在。
排除各层 `.git`（目录或子模块文件）、`.DS_Store`、编辑器配置、Python 缓存及
`build/`、`install/`、`log/`。描述资源只同步 `legged_description/l0_v5_description/`，
策略资源只同步 `legged_deploy/policy/l0_v5/`，这两个父目录下其他型号的子目录
不上传，并会清理远端已有副本；本地全部型号资源保留。共享代码、配置和 SDK
源码仍正常同步，`l0_v5` 下的全部策略、WBC 和文本动作资源均保留。
保留 `--delete`，只镜像这四个仓库。前三个仓库在远端遗留的 `.git` 目录及
子模块 `.git` 文件仍会清理，HPM 已有的根目录 `.git` 会保留。
HPM 的编译数据库、OTA 暂存目录、校零日志与备份及电机日志均排除并保留远端副本。
其他排除项同样受删除保护；不要添加 `--delete-excluded`。
同步脚本保留板端 `odin_ros_driver_rev1/module/sdk_api/lib/aarch64/` 下生成的 SDK 和设备插件动态库（`*.so*`）。
厂商发布的 OpenSSL 静态库和配套头文件随源码更新。同步后在板端重建 SDK 与设备插件，再构建 ROS 包。

如果旧版同步脚本已清理 SDK 库，先同步修正后的脚本，然后在机器人工作空间恢复：

```bash
cd /userdata/forlinx/zy_workspace/l0_deploy_v2_ws
cmake -S src/odin_ros_driver_rev1/module/sdk_api -B build/odin_sdk_v37 \
  -DCMAKE_BUILD_TYPE=Release -DODIN_BUILD_SAMPLES=OFF
cmake --build build/odin_sdk_v37 -j2
source /opt/ros/humble/setup.bash
colcon build --base-paths src --packages-up-to odin_ros_driver_rev1 --cmake-force-configure \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF -DAMENT_CMAKE_SYMLINK_INSTALL=OFF
```

SDK 构建产出 `lib/aarch64/libodin_sdk.so`、版本链接和 `libodin-device-Odin2Special.so`。
强制重新配置 ROS 包后，安装规则会把所需库与驱动一起安装。

## 编译优化

控制器、策略推理、硬件适配、规划辅助节点、电机映射、云桥、配网程序和
MuJoCo 仿真器在未指定构建类型时默认使用 `RelWithDebInfo`。机器人 GCC
工具链下对应 `-O2 -g -DNDEBUG`，启用优化并保留调试符号；显式指定的
`Debug`、`Release` 等构建类型仍然生效。`motion_runtime` 保留
`-fno-fast-math -ffp-contract=off`，MuJoCo 的 AddressSanitizer 仅在
`Debug` 构建启用。Odin 驱动及 SDK 已有默认 `Release/-O3` 配置。

Mac 上用 `scripts/sync_to_robot.sh` 同步源码，再在 RK3588 机器人的 Linux 终端运行
`quick_build.sh`。脚本加载 ROS 2 Humble，先调用 Odin 的 `build_ros2.sh`，
再构建 HPM、L0 V5 实机部署及其余依赖。第二阶段排除已构建的 Odin 和独立的
`set_zero` 校零工具，`--parallel-workers 1` 限制 colcon 同时只构建一个包。

```bash
cd /userdata/forlinx/zy_workspace/l0_deploy_v2_ws
bash src/l0_ros2_deploy_v2/scripts/quick_build.sh && source install/setup.bash
```

`Release` 保留优化并省去默认 `RelWithDebInfo` 的调试符号，`BUILD_TESTING=OFF`
跳过受该开关控制的测试目标。脚本显式关闭 ament 的链接安装，按各包的普通安装规则
写入 `install/`。当前测试仍保留 `src/`，推理后端沿用项目配置，默认 `AUTO` 会在
RK3588 上选择 RKNN。

以前用过 `--symlink-install` 的工作空间，普通安装可能保留旧目录中已有的软链接。
如需重新生成 `install/`，先把旧目录改名备份，再运行快编脚本。
`build/` 可以继续用于增量构建。

重复执行脚本会复用 `build/` 的增量缓存。首次切换构建类型会重编相关源码，历史构建文件
仍占用磁盘。可在板端用 `du -sh build install log` 查看占用。

## x86 Linux 上使用 ONNX 和 MuJoCo

推理后端只有 `OPENVINO` 和 `RKNN`：x86 的 `AUTO` 选择 OpenVINO CPU，
RK3588 的 `AUTO` 选择 RKNN。已移除 Orin/TensorRT 推理代码和 CUDA 链接。
AMP、imitation、WBC Base/参考模型、局部规划器、文本分类和动作插值均支持
OpenVINO 加载 ONNX；RK3588 仍使用对应 RKNN。旧策略目录的 `.trt` 历史模型
不再被加载或作为启动检查的有效模型。

桌面机使用 x86_64 Linux/ROS2，安装 OpenVINO C++ Runtime（含 CPU 插件和 ONNX
前端），并让 CMake 可以找到 `OpenVINOConfig.cmake`，例如设置 `OpenVINO_DIR`。
还需 ROS2 依赖、GLFW/OpenGL 和 OpenCL 开发库；MuJoCo 使用仓库的 x86 动态库。
Odin 驱动/SDK 依赖按前述部署说明准备。神经网络推理使用 CPU，无需 CUDA。
导航的点云深度处理仍使用原有 OpenCL GPU 路径，启用导航时另需可用的 OpenCL GPU。

从工作空间根目录构建仿真及依赖：

```bash
cd ~/zy_workspace/l0_deploy_v2_ws
colcon build --symlink-install --cmake-force-configure \
  --packages-up-to legged_mujoco mujoco legged_controllers \
  --cmake-args -DLEGGED_DEPLOY_BACKEND=OPENVINO -DCMAKE_BUILD_TYPE=RelWithDebInfo
source install/setup.bash
# 当前 WBC ONNX 尚未补齐时，先验证行走仿真：
ros2 launch legged_controllers sim2mujoco.launch.py wbc_enabled:=false txt2motion_enabled:=false
```

当前已从 `wbc_lora_20261028_105159_584983` 补入匹配的共享 Base ONNX 和
63 个固定动作参考 ONNX；17 个同名动作版本不符、6 个新增动作未在来源中找到，
未复制旧参考来替代。可用动作及缺失清单见
[ONNX 补入记录](legged_deploy/policy/l0_v5/wbc_lora_fp16/ONNX_IMPORT.md)。
重新构建/安装后，可先只开放一个已匹配的固定动作：

```bash
ros2 launch legged_controllers sim2mujoco.launch.py \
  wbc_enabled:=true txt2motion_enabled:=false wbc_policies:="['L0_Hugme']"
```

固定参考模型在动作准备时延迟加载；选择尚未补齐的动作时会明确报出缺失路径，
不会转换 RKNN 或自动切换后端。Base 已补齐文本动作跟踪所需的共享模型，
完整固定动作/文本动作链仍需在目标 PC 验证。仿真默认开启 WBC 和文本；
先验证固定动作时使用上面的显式参数，先验证行走时关闭这两个开关。

模型名称和张量接口必须匹配 `base_manifest.json` 的 ONNX 声明，LoRA 权重保持配套。
文本分类和插值 ONNX 已在 `policy/l0_v5/txt2motion/`，但文本动作执行仍依赖 WBC Base。
局部规划器在 x86 加载时从原始 ONNX 提取 `actions`/`next_hidden_state` 子图，
固定 batch=1；速度映射、边界和看门狗仍由现有 C++ 代码负责。
ONNX 输出继续经过形状、类型和有限值检查，过期结果由原有截止时间逻辑丢弃。
CPU 文本推理同步完成后确认退休，再检查取消和三秒截止时间；超时不提前复用缓冲区。
桌面 CPU 的推理延迟和完整 ROS2/MuJoCo 运行需在目标机验证，本地只做静态检查。

## Odin 数据输出与 CPU 排查

建图模式服务 `/control/radar/map_mode`、假大脑和云端地图回传的正常流程见
[建图主流程联调](docs/mapping_workflow.md)。定位请求按 map_id 切换 Odin 模式并加载回环地图。

驱动默认配置统一由相邻雷达仓库的 `odin_ros_driver_rev1/launch/driver.launch.py` 维护。
`one_start.launch.py` 通过 `local_planner.launch.py` 按 `start_radar_driver` 开关启动一套驱动，
并启用建图专用的图像后处理。
同事只拉雷达仓库即可独立使用，不依赖本部署仓库或规划器。

雷达包的 ROS2 launch 默认以 `normal` 模式启动并输出传感器数据。
部署程序不自动切换模式；启动后由雷达同事通过 `device_mode` 控制指令切入
`slam`（mode=3），启用主机回环建图。Normal 下回环算法保持 Idle。
全部 7 个 `loop_closure/*` 输出随驱动启用，无需部署侧新增参数。
`optimization_status_marker` 显示 `IDLE / OPTIMIZATION`，`optimized_path` 显示优化轨迹；
`corrected_cloud` 仅在有点云的关键帧发布，`map_cloud` 在加载地图后由算法回调发布。
状态、优化结果和加载地图使用 Transient Local 保留最新一条，便于后打开 RViz 查看。
独立可视化用 `ros2 launch odin_ros_driver_rev1 driver.launch.py start_rviz:=true`，
其自带 RViz 的 `Loop` 组已启用六个可视化话题；`optimization_busy` 是 Bool，用终端查看。
机器人需安装可加载的回环算法库 `libmanifold_lc_sdk.so.<ABI版本>` 及依赖；
完整话题类型、数据触发条件、重定位加载地图和验证命令见雷达包 `docs/topics.md`。

驱动默认保留 `cloud/raw`、`cloud/slam`、camera0 compressed 彩色图、
`odometry_lf`、`odometry_hf`、Odin IMU 和设备 TF（高频里程计沿用当前整机入口的开启设置）。
整机的 `camera0/undistort` 随 Odin 确认的建图模式启停，支持 SLAM 和 ReMap。
`camera0/raw` 由 `send_image_raw` 独立控制，默认关闭。
关闭 camera1、彩色点云渲染、双目校正、OpenCV 去畸变、重投影和厂商点云/深度转换。
`planner_sensor_bridge` 仍需要 `cloud/raw` 和 `odometry_lf`，不能关闭。
`enable_imu` 只控制 IMU 数据通道；TF 来自独立的 `enable_tf` 通道，关闭 IMU 输出不关闭
设备内部定位使用的 IMU。TF 中出现 `device0/imu` 坐标系不代表需要发送 IMU 话题。
这些设置不影响 HPM 供 policy 使用的 `/imu/data`。

Odin 驱动自身默认 `publish_static_tf:=false`，关闭旧的手写安装偏移；
独立启动与整机启动使用同一套驱动默认配置，运控入口不再传入 TF 覆盖参数。
`planner_sensor_bridge` 是安装变换
`device0/imu → device0/base_link` 的唯一发布者，通过 `/tf_static` 发布。
它复用当前机器人 URDF 的 `base_link → lidar_link` 固定关节链和设备提供的
`lidar ← imu` 外参，计算完整平移和旋转：

```text
base_from_imu = base_from_lidar_URDF × lidar_from_imu_设备
imu_from_base = inverse(base_from_imu)
```

首次有效里程计到达且设备外参有效后，规划桥发布一次 `imu_from_base`；
外参尚未就绪时限频提示并等待，不发布零变换或旧偏移。后加入的 RViz 等订阅者
仍可收到静态 TF。安装关系在本次运行期间固定；修改 URDF、设备标定或安装方式、
更换设备后，必须重启整套程序。
规划桥继续发布 `/planner/base_odometry` 和 `device0/odom → base_link` 动态 TF，
与静态安装 TF 复用同一个 `base_from_imu` 计算结果；`base_link` 和
`device0/base_link` 名称保留，表示同一个物理基座位置和方向。

设备 TF 通道保持 `enable_tf:=true`：设备发来的涉及 `device0/map` 的变换转发到
`/tf_static`，其他设备变换转发到 `/tf`。关闭驱动安装变换不影响设备 TF 转发。
雷达可独立启动，无需运控传入配置参数。单独运行 Odin 时不提供机器人安装变换；
启动运控规划桥后，才提供上述 URDF 安装 TF 和基座里程计。不要显式启用驱动旧安装 TF，
避免与规划桥重复发布。

机器人端重启后，用以下服务查询确认 `imu_enabled` 为 true、`image1_enabled` 为 false，
`tf_enabled`、`odom_lf_enabled`、`odom_hf_enabled` 及两路点云为 true：

```bash
ros2 service call /manifold/device0/get_channel_config \
  odin_ros_driver_rev1/srv/GetChannelConfig '{}'
```

用
`ros2 topic hz /manifold/device0/odometry_lf` 和
`ros2 run tf2_ros tf2_echo device0/odom device0/imu` 检查规划输入与 TF 持续更新。
用 `ros2 run tf2_ros tf2_echo device0/imu device0/base_link` 检查基于 URDF 的安装变换，
用 `ros2 topic echo /tf_static --qos-durability transient_local` 查看实际发布的静态 TF；
在规划桥发布后再启动订阅者，也应能收到安装变换。
在同一时间戳比较 `device0/odom → device0/base_link` 与 `/planner/base_odometry`
中的 `base_link` 位姿，应在数值精度范围内一致。确认安装 TF 仅由
`planner_sensor_bridge` 发布，TF 树没有环或重复父节点。
关闭的通道仍可能出现在话题列表中，应以数据是否持续发送为准。

`post_process:=false` 跳过 `post_process_node` 的启动，
省去驱动后处理的 JPEG 解码、色彩转换及 raw 图像拷贝/发布；主驱动仍发布
1280×1088、30 FPS 的 `camera0/compressed`。机器人端用
`ros2 topic hz /manifold/device0/camera0/compressed` 确认压缩图持续发送，
用 `ros2 topic info /manifold/device0/camera0/raw` 确认没有 raw 图像发布者。
如果下游节点自行解码 compressed，仍会有该节点的解码开销。
SDK 内部回环算法仍接收 camera0 图像，关闭 ROS 后处理不等于关闭 SDK 内部图像处理。
本机仅静态检查，实际 CPU 降幅和关闭通道后的 TF 连续性需在机器人验证。

## L0 v4 HPM 功能迁移

当前实机默认机型为 `l0_v5`，启用全部 21 个关节；WBC、雷达和 MQTT 云端通信
默认关闭。`l0_v4` 使用 RS05 双臂；`l0_v5` 使用 MR PA35-10-8 双臂，
两者配置独立。实机入口支持 `l0_v3/l0_v4/l0_v5`。v5 路由已按机器人上
`src/hpm_transport_ros2/src/hpm_motor/config/motors.yaml` 补齐；详见
[v4/v5 配置说明](system_command_arbiter/README.md#l0-v4--v5-hpm-mappings)。
正式电机表和协议数值确认前不要上电运动。

云端通信总开关为 `cloud_enabled:=true/false`，默认 `false`，控制 MQTT 上下行、
1 Hz 心跳和 1 Hz HPM 真实电量上报；BLE 配网独立。用法与机器人验收见
[云端通信说明](luvbot_cloud_bridge/README.md)。

## WBC 与 txt2motion（L0 V5 / RK3588）

固定动作和文本动作使用独立前端，共用 Base RKNN、控制器和动作准入。
固定动作是各动作 LoRA bin＋reference RKNN；文本动作是 UpperBody_Enhanced LoRA bin＋现场生成 JSON，完全不读取 reference RKNN。
AMP 沿用主 launch 的 `subpolicy`。固定动作始终读取 `legged_deploy/policy/l0_v5/wbc_lora_fp16/` 清单中的全部动作；
WBC 和文本跟踪均使用 `wbc_lora_fp16/base_lora.rknn` 这一份 Base；文本参数在同包 `text/`，组合清单为包内 `deployment.json`。
文本基座参考保持直立，高度及左右腿目标角从同包 `text/standing.yaml` 读取。

实机入口通过 `wbc_enabled:=true/false`、`txt2motion_enabled:=true/false` 独立控制两个功能，
自动启动一份公共后端，主 launch 直接启动前端，独立功能 launch 已移除。
固定/文本跟踪资源统一位于 `wbc_lora_fp16`；只开固定读取 base_manifest.json，开启文本读取包内 deployment.json。
实机入口不再暴露公共后端开关、动作筛选和选包参数。
文本唯一对外入口为 `/algorithm/brain/txt2motion_cmd`（`std_msgs/msg/String`），
JSON 必填 `timestamp_ms`、`text`、`seed`，seed 由上游传入且为 `[1, 2^32)` 的整数。
接收后自动生成并执行，回到 AMP 且清理确认后删除临时 JSON；所有请求共用名额，忙时立即拒绝。
不再提供文本 CLI、离线生成或仅生成文件入口。
在线网关、固定/文本前端和文本生成均使用 C++14；文本模型通过 RKNN C API 常驻推理。
从有效文本接收到完整 JSON 保存共用三秒截止时间，提前完成立即继续，超时结果不执行；
清理仍须确认推理退出和 AMP 回交。C++ 素材随包交付，在线不依赖 NumPy 或 RKNNLite2。
固定/文本动作预约后，控制线程捕获头部实测位置并保持，暂停跟随输入、已有头部插值及自动回中；
固定动作预对齐或文本 WBC 接管时交还头部控制，清理和 AMP 回交确认后才恢复跟随。
摆臂不强制停止：继续根据 AMP 速度自然结束输出。文本网关等待控制线程确认头部保持、摆臂已停止，
再使用确认时间戳之后的新鲜观测采集起姿；等待计入原三秒截止时间。
`MotionTransaction.STATUS.payload_json` 的 `text_capture_stamp_ns` 为此确认时间戳，0 表示未就绪，
并与当前 `request_id` 绑定；取消、故障、策略切换及释放预约会撤销确认。
动作准入不再检查速度大小，仍检查有限值、姿态、策略有效性和预约归属；导航和动作互斥由大脑协调，
不新增导航拒绝、暂停、速度锁或主动停车，也不修改既有导航反馈。
固定 WBC 启动前，worker 通过 `ControlWbcMotion.REFERENCE` 返回首帧参考，
控制线程在 AMP 持续控制双腿的同时，将双臂和头部目标平滑到该参考（沿用 `wbc_transition_sec`，默认 0.35 秒）。
预对齐期间暂停头部跟随和摆臂；完成后基于最新反馈及实际已下发的上半身目标重新准备 WBC 输出。
固定 WBC 的进入和返回均直接切换双腿策略指令，只对双臂和头部的 `q/dq/kp/kd/tau_ff` 插值。
预对齐失败或取消时，在 AMP 下平滑撤回上半身目标；故障、过期和急停保护保持有效。
固定和文本 WBC 接管后，参考轨迹从首帧直接按控制周期推进，无额外起始保持；
动作总时长仅由参考帧数和控制周期决定，不生成腿部参考插帧。
文本生成端 Inbetweener 已负责头手起始衔接及返回起姿，执行端直接使用生成帧，不再额外包首尾姿态插帧和保持帧。
执行端检查文本上身首尾与采集姿态的误差不超过 0.005 rad；腿部参考继续使用 standing.yaml，不另加腿部插帧分支。
文本动作的 AMP/WBC 交接同样直接切换双腿，只平滑双臂和头部的位置、速度、增益及前馈力矩，默认 0.35 秒；
不采用固定动作的上半身预对齐，不增加 JSON 轨迹首尾帧，原 0.15 rad 姿态失配与首尾 0.005 rad 检查保留。
进入 WBC 和返回 AMP 时，控制线程以切换前上一周期最终合成的 `q/dq/kp/kd/tau_ff` 为过渡起点，
包含头部跟随、摆臂及尚未结束的指令插值；辅助动作在控制线程实际交接时暂停，避免提前一帧撤掉覆盖。
关节指令先整帧检查数量、有限值和非负增益，通过后才替换输出和缓存；插值直接读取这份最后合法的合成指令。
预对齐没有合法起点时拒绝本次动作并保留 AMP；返回时缺少插值起点则使用合法的新鲜 AMP 输出直接接管。
运行中的 WBC 若仅超过单次推理预算（50 Hz 下 20 ms），丢弃迟到结果并重试，不立即结束 worker；
迟到输出不发布、不写入动作历史，也不延长原 `valid_until`。启动前预推理超时仍拒绝本次动作。
WBC 最后接受的指令仍有效时允许 worker 重试，不再提前 50 ms 终止动作。
指令真正过期或不可用时，控制线程保持最后合法帧并请求 AMP；worker 推理失败也走同一回交路径。
回交不等待服务或 NPU：只接受匹配本次 return ID 且仍有效的 AMP 输出，再直接切换双腿，平滑上半身。
旧 WBC 历史种子过期、不可用或不匹配时，AMP 改用通过新鲜度和完整性校验的当前实测关节状态初始化；
拒绝过期/非法种子的检查保留。尚未切入 WBC 就失败或取消时保留 AMP，等待其恢复推理。
AMP 或 WBC 指令过期、暂不可用，或合成指令未通过整帧检查时，控制线程持续重发
最后合法的 `q/dq/kp/kd/tau_ff`，保留原位置增益，不推进覆盖或插值输出，直到新鲜合法输出接管。
这是保持上一条控制目标，不是新增站姿策略或腿部过渡轨迹；它不能主动维持平衡。
保持没有策略级时间上限，不因推理延迟或回交等待过久锁定急停；原策略消息有效期不会被续期。
只有没有完整合法缓存，或缓存与硬件关节数量不匹配、无法保持时，才因无可用指令进入急停阻尼；
手动急停、电机故障、IMU 故障和姿态保护始终优先，立即打断保持。
推理输入/输出的新鲜度、单次推理预算和原策略消息有效期不变；`wbc_command_timeout_sec` 仅用于准备和清理服务。
实际切到 AMP 后才发布动作终态；故障恢复反馈为 `status=failed`，`detail` 包含 `AMP control restored`，
网关保留该说明，worker 退出和预约释放仍须确认。动作失败不等于控制器进入 EmergencyStop。
`/system/motion_controller/status` 在保持期间报告 `policy.valid=false`、`policy.holding_last_command=true`，
未触发急停时 `emergency.active=false`；不再提供恢复计时或期限字段。
机器人端还需验证满负载下的实际恢复耗时；长期保持旧目标可能失稳，IMU 姿态检测也不是平衡保证。
状态机在策略断帧期间保持 Walking；只有真实急停或无合法缓存时进入 EmergencyStop。
EmergencyStop 经手动 reset 回到 Standby；Calibration 仍只允许从 EmergencyStop 进入，退出后返回 EmergencyStop。
状态读写及有条件转换使用原子操作，避免控制线程和 ROS 回调之间的旧判断覆盖已发生的状态转换。
`robot_interfaces` 统一维护部署内部接口：`PolicyObservation.msg`、`PolicyCommand.msg`、
`SwitchPolicy.srv`、`ControlWbcMotion.srv`、`MotionTransaction.srv` 和 `RequestMotion.action`。
策略观测、策略输出、准备/取消服务及动作 Action 继续用于进程间通信；
`ControlWbcMotion.REFERENCE` 返回首帧参考。
`luvbot_ros_interfaces` 仅维护对外接口，内部接口已从其中移除。
部署迁移时须重建接口包及依赖包，并同步重启所有相关进程；上述类型改用 `robot_interfaces`，
字段、topic/service/Action 名称和大脑 JSON 指令格式保持不变。
机器人端从 `~/zy_workspace/l0_deploy_v2_ws` 执行 `colcon build --packages-up-to legged_real legged_deploy`。
吊架保护下验证固定动作进入、正常结束和中途取消：预对齐日志期间双腿持续使用 AMP，
进出策略的日志应标明 `arms/head only; legs use selected policy directly`；同时记录两路策略命令和
`/actuator/motor/command`、`/actuator/motor/state`、`/imu/data`，按电机映射对比腿部目标，确认没有混合指令。
另验证预对齐阶段取消、worker 准备超时及 RB 急停不会继续启动 WBC。
文本需验证头部保持不回中、自然摆臂回归后才采集起姿、三秒截止时间不延长、双腿进出无插值，
以及生成前取消、生成超时、姿态失配、无动作结果均能完成清理并恢复跟随。
同时验证运行中的短暂推理超时、连续延迟/worker 故障、正常结束及中途取消：检查
`Discarding late WBC result`、`[Policy Recovery]`、`[WBC Return]` 日志、原策略指令有效期和恢复耗时。
恢复成功时 `/system/motion_controller/status` 应为 AMP、非急停，动作反馈明确失败但已恢复 AMP；
验证缺口期间电机目标和增益与最后合法帧相同，旧策略消息的有效期不变；恢复后无需 reset。
另验证断帧持续超过原 300 ms 后仍保持最后合法指令、状态维持 Walking，以及保持期间 RB、硬件和姿态保护能优先打断。
实际急停后不得重放急停前的缓存；reset、重新进入 Walking 和新策略输出接管仍需验证。相关故障场景只在吊架保护下验证。
双向动作历史优先使用源策略最后有效关节目标，按接收策略的默认角和 action_scale 换算初始化；
WBC→AMP 恢复时种子失效则按新鲜实测关节角初始化；
首次进入 AMP、急停复位后重新进入及普通 observation 重置时，使用当前实测关节角
按 `(q_measured - q_default) / action_scale` 初始化动作历史，并用当前 observation 填满全部历史帧；
零 action_scale 的关节填 0，保留动作限幅和关节顺序转换。WBC 返回 AMP 仍优先使用 WBC 最后有效目标。
状态历史继续使用传感器数据。消息接口需与 luvbot_ros_interfaces 同步构建，
转换规则和板端验证见 [动作历史交接说明](docs/amp-wbc-action-history.md)。
生成参考轨迹首尾连续不保证两种策略输出的电机目标连续，因此文本头手也不能跳过这层指令平滑。
独立的启动保持、生成端返回前停留及 AMP/WBC 指令切换保护继续按原配置执行。
C++ 迁移只做本地静态检查，机器人构建及三秒生成要求仍待实机验收。
文本生成模型、词表和 C++ 素材统一位于 `policy/l0_v5/txt2motion/`，
由 legged_deploy 安装；与 `wbc_lora_fp16/` 同级，后者继续唯一持有共享 Base 和固定/文本跟踪 LoRA。
txt2motion/ 内 ONNX 与 RKNN 并排存放，materials/ 存放素材数据；不再额外使用 compiled/ 或 cpp/ 层级。
文本与其他 RKNN 模块共用 `legged_deploy/third_party/rknn_runtime/lib/librknnrt.so`，
由 legged_deploy 安装到其安装前缀的 `lib/`；文本目录不再携带运行库副本。
原策略目录 motion_runtime 已合并进 wbc_lora_fp16；15 个旧独立 WBC 目录和重复副本已清理。
所有功能的启动和在线加载均不计算或比较模型/素材 SHA256，也不要求清单中存在这些字段。
文本转换已取消文件哈希校验；WBC 离线转换仍检查源包完整性与配对。
在线保留实际张量接口、数值和动作安全检查；无执行用途的 `.rknn.json` 已删除且不再生成。

[文本模型与转换说明](legged_deploy/policy/l0_v5/txt2motion/README.md)；
[ONNX 转换工具](scripts/README.md)。本地静态检查与机器人运行验收分别记录。

## 共享关节状态

部署控制器运行时，`/system/joint/jointstates` 使用标准 `sensor_msgs/msg/JointState` 发布关节反馈，
不依赖 RViz；`rviz:=false` 仅关闭可视化，`/joint_rad_text` 不创建或发布。
关节状态沿用 `viz_frequency` 发布周期，默认跟随策略频率。
`name`、`position`、`velocity`、`effort` 按索引对应，分别表示关节名、角度（rad）、
速度（rad/s）和力矩（N·m）；订阅方按关节名查找数据。QoS 为 Reliable、Volatile、深度 10。
该消息不含逐关节反馈有效标记，反馈异常时可能保留旧值，不能仅凭消息持续到达判断电机在线。

机器人按正常配置启动并保持 `rviz:=false` 后，可在机器人或已配置相同 ROS domain 的同事电脑上验证：

```bash
ros2 topic info /system/joint/jointstates --verbose
ros2 topic echo /system/joint/jointstates sensor_msgs/msg/JointState --once
ros2 topic hz /system/joint/jointstates
```

需要 `JointState` 的节点应订阅 `/system/joint/jointstates`，不要订阅原始电机接口 `/actuator/motor/state`。

## sim2mujoco（ROS 2 Humble）

对外导航、语音、人脸及电机接口位于独立仓库 `luvbot_ros_interfaces`，将匹配版本放在本工作空间
的 `src/luvbot_ros_interfaces`。实机 HPM、MuJoCo 和部署端统一使用该仓库的
`luvbot_hardware_interfaces`。部署内部策略和动作接口位于本仓库的 `robot_interfaces`。

在 Ubuntu 22.04 / ROS 2 Humble 的新工作空间构建，避免旧版同名包的安装结果污染：

```bash
source /opt/ros/humble/setup.bash
cd ~/zy_workspace/l0_deploy_v2_ws
colcon build --symlink-install \
  --packages-up-to legged_mujoco mujoco legged_real legged_controllers system_command_arbiter robot_interfaces
source install/setup.bash

# x86/OpenVINO：先验证 l0_v5 模型和 AMP，关闭 RKNN 专用前端
ros2 launch legged_controllers sim2mujoco.launch.py \
  robot_type:=l0_v5 subpolicy:=l0_v5_radar_velocity_tracking \
  wbc_enabled:=false txt2motion_enabled:=false
```

启动后电机话题统一使用 `luvbot_hardware_interfaces`，由 `topic_hardware_adapter` 直接连接
MuJoCo 虚拟电机，不需要真实 HPM，也不启动旧协议的 arbiter 节点：

| 话题 | 类型 | 发布 → 订阅 | QoS |
| --- | --- | --- | --- |
| `/actuator/motor/enable` | `luvbot_hardware_interfaces/msg/MotorEnableCommand` | adapter → MuJoCo | Reliable / Volatile / depth 10 |
| `/actuator/motor/command` | `luvbot_hardware_interfaces/msg/MotorCommand` | adapter → MuJoCo | Reliable / Volatile / depth 10 |
| `/actuator/motor/state` | `luvbot_hardware_interfaces/msg/MotorStateArray` | MuJoCo → adapter | SensorDataQoS |

仿真启动顺序为：先加载 MuJoCo，最多等待 60 秒收到第一帧电机反馈，再启动 adapter；
随后复用 `check_hardware_ready.py false true`，在原有 20 秒期限内确认电机使能后的
反馈就绪与 IMU 在线，最后启动 policy/手柄等节点。任何前置检查失败都会清理已启动进程。
MuJoCo 的窗口和网格加载耗时不再占用 adapter 的电机使能期限，也不放宽实机保护。
退出时按相反顺序清理，adapter 发送逐个 disable 时 MuJoCo 仍在运行。

仿真与 sim2real 复用相同的 `LeggedReal`、`LeggedControlloop` 和 `LeggedController`，
共用策略观测、行走命令合成、关节映射及反馈确认。仿真保留 `isSim=true`：待机固定
`kp=80`、`kd=1`，跳过姿态保护，禁用校准入口；这些是现有仿真行为。
生命周期 timer 保留仿真的 1 ms 配置，逐个使能间隔仍至少 100 ms。
启动参数也有原有差异：仿真急停阻尼默认为 5.0、头部跟随默认关闭；实机分别为 0.2 和开启。
对比控制效果时必须选择相同的 `robot_type`、`subpolicy`、WBC/文本开关及其余控制参数；
入口默认值可能不同，以实际启动参数为准，不能直接将两个默认启动结果视为同一配置。
仿真提供物理模型中的电机、IMU 和里程计，真实 HPM/CAN 动态、传感器噪声和时延不能由
该模型完全复现，也不作为实机校准效果的验收。

MuJoCo 继续发布 `/imu/data` 和 `/odom`；Controller/Policy 使用
`control/policy/observation` 和 `control/policy/command/rl`。
旧的 `/actuator/motor/set_enabled` 服务及 `LOWCMD`、`LOWSTATE`、`rlState`、
`rlCmd_*` 不属于该启动链路。`system_command_arbiter` 的映射库和 YAML 仍被复用，
因此构建时仍需要该包及其 `luvbot_hardware_interfaces` 依赖。

当前 MuJoCo bridge 要求模型以一个 free base 开始、每个 actuator 唯一对应一个
mapping 中的单自由度 joint；按关节名称绑定 actuator、qpos 和 dof 地址，允许 MJCF
与部署 mapping 的排列顺序不同。l0_v5 的 MJCF 按头、手臂、腿排列，而 mapping 按
腿、手臂、头排列，无需修改模型或 mapping。mapping 最多 32 个关节，以匹配
`feedback_mask`；缺失、重复、额外关节或不支持的模型结构会被拒绝。
启动会打印实际场景路径及 `MuJoCo model loaded`；加载或绑定失败会报错并关闭窗口，
不再停留在空白窗口。拖入模型及 UI 重载也重新验证绑定，并清空旧命令、失能；
重载前已使能的电机反馈置为无效，须按原有故障恢复流程重新使能。

`motor_types[]` 和 `route.motor_type` 均为 mapping 中从 0 开始的索引，包含配置禁用
关节留下的空位，不是电机型号或 CAN ID；`bus_index`、`motor_id` 不用于仿真寻址。
默认 l0_v3 有 21 个关节、15 个配置启用的电机，MIT 命令帧须恰好覆盖这 15 个索引。
使能消息的两个数组必须非空、等长，索引必须有效、配置启用且不重复；命令帧必须使用
`CONTROL_MODE_MIT`，不得有重复、缺失、禁用或越界索引及非有限数据。
整条消息校验通过后才更新，非法帧不会刷新 watchdog。

虚拟电机初始失能，接受 adapter 按 100 ms 间隔逐个发送的 enable/disable；不发送 ACK。
使能期间允许完整的零增益 MIT 帧，未使能和配置禁用的关节保持零力矩。
失能、重新使能都会清空该电机目标，下一物理控制步输出零力矩，直到收到新的有效命令；
不会重放使能前的缓存。配置启用关节持续返回实际物理状态，即使该电机当前失能。
反馈的位置、速度、力矩沿用零偏、方向和减速比换算，`operating_mode` 为
`OPERATING_MODE_UNKNOWN`；正常反馈 `error_code=0`，`feedback_mask` 只对有效反馈
索引置位，与运行时使能掩码不同。最终使能完成后的新反馈仍须通过 adapter 的既有
就绪、关节位置和圈数确认，仿真端不直接设置 `/hardware_status/motors_ready`。

`command_watchdog_ms` 沿用默认 100 ms（最小有效值 1 ms）。命令的 `stamp` 比接收时间
旧超过该窗口，或在未来超过 100 ms，均被拒绝；接收后按单调时钟监控断流，首次使能
也会启动等待首条命令的 watchdog。有效通信周期内拒绝重复、倒序序列号，支持
`uint32` 回绕；超时后允许新周期从新的序列号起步。超时会清空命令、失能，并锁存
当时已使能电机的通信故障；这些电机反馈 `valid=false`，对应 mask 位清零。
恢复命令流或发送 disable 不会清除故障，必须显式 enable 对应电机，再发送新命令。
控制器若已锁存急停或位置反馈故障，仍须按原有恢复流程处理，不能靠 enable 绕过。

### Linux/ROS2 验收

使用 l0_v5 时，入口显式指定模型和策略即可。x86/OpenVINO 的当前构建未注册
`rl_wbc`（WBC 控制器只在 RKNN 后端编译），因此先关闭 WBC 和文本前端，验证 AMP
及模型显示；这与模型文件是否存在是两个独立问题：

```bash
ros2 launch legged_controllers sim2mujoco.launch.py \
  robot_type:=l0_v5 subpolicy:=l0_v5_radar_velocity_tracking \
  wbc_enabled:=false txt2motion_enabled:=false
```

工作空间构建会通过 `mujoco` 的运行依赖安装 l0_v3/l0_v5 description 及其 meshes/mjcf。
v5 成功加载应报告 21 个 actuator、21 个启用电机，并在窗口中显示机器人；确认头、腿
关节分别响应对应命令，反馈使用 mapping 顺序，不能因 MJCF 顺序不同而错位。

在具有正常图形环境的 Linux 仿真机完成上面的工作空间构建和启动后，检查：

```bash
ros2 topic info /actuator/motor/enable --verbose
ros2 topic info /actuator/motor/command --verbose
ros2 topic info /actuator/motor/state --verbose
ros2 topic echo /actuator/motor/enable
ros2 topic echo /actuator/motor/state --once
ros2 topic echo /hardware_status/motors_ready --qos-durability transient_local --once
```

每个电机话题应只有表中的类型；enable/command 的订阅者为 MuJoCo 节点
（当前 launch 节点名 `/mujoco`），没有旧 arbiter 节点或 `set_enabled` 服务。
adapter 的黄色 HPM 提示来自共享日志文字，表示电机话题缺少订阅者，不要求连接真实 HPM。
启动应依次打印 `MuJoCo motor feedback is online`、每轮一条电机 enable 汇总、
`Hardware readiness check passed`，然后才启动 policy；不能出现使能订阅者等待超时。
正常 l0_v3 应完成 15 个电机的逐个使能，`motors_ready=true`，反馈包含索引
0–11、18–20，mask 为 `0x001C0FFF`。
观察待机、站立及运动中的关节顺序和方向，确认配置禁用关节始终没有驱动力。

故障注入使用单独启动的虚拟电机端，关闭 adapter，避免它与注入发布者同时发送命令：

```bash
ros2 launch mujoco mujoco_sim.launch.py robot_type:=l0_v3
ros2 topic pub --once /actuator/motor/enable luvbot_hardware_interfaces/msg/MotorEnableCommand \
  '{motor_types: [0], enabled: [true]}'
```

| 场景 | 操作与预期 |
| --- | --- |
| 未收到首帧命令 | 上述单电机使能后超过 100 ms，应报告 watchdog 超时，索引 0 的反馈无效且 mask 位清零 |
| 单电机失能/恢复 | 以完整、当前时间戳的 15 项 MIT 帧持续发送；disable 索引 0 后立即零输出，enable 后在下一新帧前保持零输出 |
| 非法使能 | 注入空数组、不等长、重复、越界或配置禁用索引，整条拒绝，其他电机状态不变 |
| 非法命令 | 分别注入错误模式、重复/缺失/越界/禁用索引、NaN、过期或未来超过 100 ms 的 stamp，整帧拒绝且不刷新超时计时 |
| 序列号 | 在同一通信周期内验证重复、倒序被拒绝；`4294967295 → 0` 可接受；断流超时后新序列可建立，但不会自行使能 |
| 命令断流 | 停止有效命令超过 100 ms，已使能电机零输出并反馈无效；恢复命令不清故障，逐个 enable 后才能恢复 |
| 反馈断流 | 使用完整启动链路时停止 MuJoCo，adapter 应沿用既有反馈 watchdog 报错并进入急停 |
| 启动超过 20 秒 | 模型/窗口加载超过 20 秒但小于 60 秒时，adapter 应在第一帧反馈后才启动，仍能完成使能；超过 60 秒无反馈则清理退出 |
| 正常退出 | 完整链路中单独向 adapter 发送 SIGTERM，保留 MuJoCo 和话题监视器运行，确认能收到逐个 disable 且停止驱动；随后退出其余 launch |

注入命令时使用当前 ROS 时间，可靠发布完整的配置启用索引，发送频率应明显高于 10 Hz；
反馈值应来自物理状态，不会与目标位置瞬间相等。数值超时验收以实际物理控制步为准。
本次协议对齐未增加虚拟校零服务；不使用校零操作作为上述验收的恢复手段。
如果曾出现 `Timed out waiting for /actuator/motor/enable subscriber` 和
`lifecycle_fault=true`，模型后来加载成功也不会自动重新使能；手柄的 controller reset
只重置状态机，不清除该硬件生命周期故障。应退出整个仿真链路，使用修正后的顺序重新启动，
不要屏蔽故障或只复位 FSM 后继续测试。

## HPM 实机直连

实机 HPM 与本工作区必须使用同一版本的 `luvbot_ros_interfaces`。
`MotorCommand`、`MotorEnableCommand` 和 `MotorSetZero` 按 HPM `motors.yaml` 的
`index` 路由，反馈用相同索引及 `feedback_mask` 标识；电机系列、CAN 总线与 ID 由 HPM
配置决定。按实机装配核对 HPM 和部署的关节索引、电机型号及配置，再进行联调。

运行 `scripts/sync_to_robot.sh`，将 `hpm_transport_ros2` 与部署源码一起同步到机器人
v2 工作区的 `src/`。
Colcon 只构建一份 `luvbot_hardware_interfaces`。部署仓库的 `robot_interfaces`
用于内部策略和动作，HPM 自带的同名旧包通过 `COLCON_IGNORE` 排除。

在现有 Odin SDK 和设备插件就绪后，执行[编译优化](#编译优化)里的 L0 V5 实机构建命令。

HPM 和 deploy 在两个终端启动，两个终端加载同一份安装环境。
`start_hpm.sh` 可用于启动第一个进程，HPM 默认使用 UDP。

```bash
# 终端 1：先启动 HPM
source /opt/ros/humble/setup.bash
source /userdata/forlinx/zy_workspace/l0_deploy_v2_ws/install/setup.bash
ros2 launch hpm_system hpm_system.launch.py

# 终端 2：再启动 deploy
source /opt/ros/humble/setup.bash
source /userdata/forlinx/zy_workspace/l0_deploy_v2_ws/install/setup.bash
ros2 launch legged_controllers sim2real.launch.py
```

架空并固定机器人、保持急停可用。先检查 HPM 的命令与使能订阅端、反馈发布端及
校零服务类型。控制器启动后核对发布和订阅端匹配、21 路反馈及 `motors_ready`，
并在现场验证退出时的逐个失能流程。

实机接口为 `/actuator/motor/command`、`/actuator/motor/state`、
`/actuator/motor/enable`，类型均来自 `luvbot_hardware_interfaces`。command 和 enable 为
Reliable depth 10，state 使用 SensorDataQoS。HPM 的 `/imu/data` 供 policy 使用；雷达发布的
`/odom` 仍只供 planner 使用。deploy 等到 enable topic 出现订阅者后，按 mapping 索引顺序
逐个发布 enable，完整发送三轮，每条消息只包含一个配置中启用的电机；
当前 l0_v5 的顺序为 `0–20 → 0–20 → 0–20`，每个电机共发送三次 enable。
disable 仍按同样的索引顺序只发送一轮。
相邻消息至少间隔 100 ms（受定时器调度影响可能略长），切换 enable/disable 请求时也保持该间隔。
发送期间继续零增益 MIT，第三轮 enable 全部发完后重新等待全部启用索引的有效反馈，才允许发布运动命令。
`/hardware_status/motors_ready` 在 enable 全部发布后，收到每个启用电机的正常反馈且没有反馈故障时
才置为 true；它表示反馈链路已建立，不表示收到了 enable ACK。关节位置或圈数尚未确认时，
可继续进入校准流程，运动命令仍由原有位置确认保护阻止。
启动检查统一由 `check_hardware_ready.py` 执行，同时等待电机就绪、启用时的 IMU 数据和可选电池数据，
使用单调时钟限时 20 秒；失败或脚本缺失会结束启动并清理进程。硬件桥自身的控制循环已在检查前运行，
会持续发送零增益 MIT 获取反馈，无需等待后续策略节点启动。启动检查退出后，持续保护由硬件桥的
电机反馈和 IMU watchdog 承担。
行走时的姿态保护按 `/imu/data` 接收回调中的有效帧计数：roll 或 pitch 超过 ±30°，
默认连续 50 帧才触发急停；200 Hz 下约为 0.25 秒，按收到的新 IMU 帧累计。
阈值帧数由 `safety_orientation_consecutive_violation_count` 设置。未达到阈值前收到正常帧即清零；
达到阈值后锁存到控制循环处理，避免被紧随其后的正常帧覆盖。待机、急停和校准时清零，
无新包时不重复累计；IMU 断流仍由原有 watchdog 处理。
可在机器人上用 `ros2 topic echo /actuator/motor/enable` 核查：每条 `motor_types` 和 `enabled`
均只有一个元素，enable 索引按顺序重复三轮（l0_v5 共 63 条），日志显示 `round 1/3`、`2/3`、`3/3`；
相邻 `stamp` 间隔至少 0.1 秒，跨轮也保持间隔；进程退出失能时仍逐个发送一轮。
进入和退出校准模式只切换零增益 MIT 保护，不发送 enable/disable，保持原有电机使能状态。

电机缺包 watchdog 仅统计没有收到该电机新反馈的控制周期。新机器未校零导致关节位置或圈数
无法确认时，只报告位置未确认并限制运动命令，不累计通信缺包；非有限数据、无效状态和
持续电机错误仍按原有反馈故障处理。
`feedback_mask` 的缺反馈提示只检查 mapping 中 `enabled: true` 的电机，忽略禁用电机；
每个电机独立累计连续缺反馈帧数，收到该电机反馈即清零，连续缺失至少 20 帧才列入告警，
持续缺失时保留 5 秒日志节流。
提示中的电机号仍为 mapping 索引加 1，与 enable 日志中从 0 开始的 HPM index 区分。

电机反馈或 IMU 故障触发急停后，已使能且非校准状态下仍持续发布阻尼 MIT：
`kp=0`、`dq=0`、`tau=0`，`kd` 沿用 `emergency.damping_kd` 并经过现有电机映射限幅。
反馈故障和运动命令保护继续生效。失能、校准、使能切换或等待反馈确认期间继续发布
`kp=0`、`kd=0`、`dq=0`、`tau=0` 的 MIT；这类零增益帧不代表电机已执行失能。
这些帧的 `q` 取最近一次通过校验的关节位置，再经零偏、方向、减速比、圈数和限幅映射
成电机位置；没有新有效反馈时保持旧关节位置，从未获得有效反馈时使用初始关节位置 0。
调零核查期间也继续零增益 MIT，以持续获得位置反馈。核查只比较本次请求提交后的新反馈位置，
不使用 HPM 的应答状态码判定校零结果。离开校准或急停状态不会单独停止 MIT。

在 deploy 终端按 `Ctrl+C`（或向硬件桥发送 `SIGTERM`）时，硬件桥先停止处理 ROS 回调，
向 `/actuator/motor/enable` 按上述 100 ms 间隔逐个发布全部配置中启用电机的 disable，
21 个电机从第一条到最后一条约需 2 秒；全部发完后最多等待 500 ms 的 DDS 投递确认。
控制线程在等待期间继续发送零增益 MIT，随后直接终止进程，
以进程终止结束 MIT 发送，不提前析构控制循环或关闭 ROS。日志应依次出现 `Stopping hardware bridge`、
`Published HPM disable command` 和 `Shutdown disable acknowledged by DDS`。
请保持 HPM 运行到 deploy 退出；无订阅者、投递超时或发送异常会打印错误。
DDS 确认不代表电机已执行失能，仍需在机器人上核验电机状态；`SIGKILL` 无法执行该清理。

### 手柄零位校准

HPM 必须提供 `/actuator/motor/set_zero`，类型为 `luvbot_hardware_interfaces/srv/MotorSetZero`。
可先在机器人上执行只读检查：

```bash
ros2 service type /actuator/motor/set_zero
ros2 interface show luvbot_hardware_interfaces/srv/MotorSetZero
```

固定并支撑机器人，摆到机械标定姿态，停止其他控制端的运动及使能命令。急停状态下按
`LS+RS` 进入校准；此时不发送 disable，电机保持原有使能状态，持续接收零增益 MIT。
校零不要求先失能；等待已有 enable/disable 发送流程结束后即可提交校零请求。
`LS+RS` 每次按下只切换一次；再次切换前，必须两键都松开并稳定至少 200 ms。
持续按住、只松开其中一键或短暂的释放抖动都不会重复触发校准切换。

- `A+X`：左腿（含膝盖）；`B+Y`：右腿（含膝盖）。
- `LB+LT`：左臂；`RB+RT`：右臂。
- `START`：颈部；`BACK`：腰部（只有 mapping 中存在并启用了腰部关节时有效）。

只对所选组中 `enabled: true` 的关节发送请求，`motor_types` 使用 mapping 中的关节索引，
不是 CAN ID 或电机协议枚举。`request accepted` 只表示该组已加入校零队列；deploy 按组内
关节顺序逐个请求，每个 service request 只包含一个索引，相邻提交至少间隔 100 ms。
HPM 一次只处理一个校零请求，因此还会等待上一个服务返回，实际间隔可能更长；此返回只用于
安排下一次发送，不用于判定零位。deploy 逐关节核查 `/actuator/motor/state` 中该电机自身
请求提交后的新反馈，尚未发送的电机不会提前确认，原始电机 `q` 满足 `abs(q) <= 0.005 rad`
时打印 `POSITION_CONFIRMED`，对应映射后关节位置为该关节的 `zero_offset`（不含圈数补偿）。
核查只接受 `valid=true` 且有限的原始 `q`，不以关节限位或圈数确认状态代替原始位置核查。
HPM service 的 `CONFIRMED`、`TIMEOUT` 等结果不用于 deploy 的位置确认；HPM 内部发送策略未改。
deploy 不修改 mapping 的 `zero_offset`。

请求通过定时器异步逐个提交，等待期间继续零增益 MIT、反馈和手柄处理。未获得目标位置时继续等待并打印
当前收到的 `q`，没有调零核查超时，也不自动重发；目标未全部确认时仍禁止退出校准和使能。
全部请求返回且位置确认后才结束该组校零；服务不可用或请求提交异常仍报告错误，停止后续排队发送并保持校准互锁。
LS 电机提交请求时额外提示 `LS电机需要重启来保存参数`；
位置匹配不证明参数已经掉电保存，需按电机要求重启并核验。
所有位置确认且服务返回后，按 `LS+RS` 退出并回到急停状态；退出不发送 enable/disable。
已使能电机恢复现有急停阻尼 MIT，已失能电机仍保持零增益 MIT。调零会清除所选关节的
旧位置圈数补偿，并在后续有效反馈上重新计算；恢复运动前核验位置反馈与机械姿态一致。
整个校准过程禁止发送运动命令的提示是正常保护。

机器人侧验证时，检查同组各条 `Requested zero for ...` 日志时间戳至少相隔 0.1 秒，
每颗电机在其请求之后才出现 `POSITION_CONFIRMED`；整组结束前退出校准仍应被拒绝。
单独进入、退出校准模式时，不应出现对应的 `Published HPM enable/disable` 日志；进程退出仍会逐个失能。

联调时检查类型、QoS 和实机 graph：

```bash
ros2 topic type /actuator/motor/command
ros2 topic type /actuator/motor/state
ros2 topic type /actuator/motor/enable
ros2 topic type /imu/data
ros2 topic info --verbose /actuator/motor/command
ros2 topic info --verbose /actuator/motor/state
ros2 topic info --verbose /actuator/motor/enable
ros2 node list | grep system_command_arbiter_node
ros2 service list | grep /actuator/motor/set_enabled
```

前三个类型应分别为 `luvbot_hardware_interfaces/msg/MotorCommand`、
`luvbot_hardware_interfaces/msg/MotorStateArray`、`luvbot_hardware_interfaces/msg/MotorEnableCommand`，IMU 应为
`sensor_msgs/msg/Imu`；最后两个命令应无输出。架空机器人并保持急停可用，先确认 HPM 的
`motor_type` 索引 0..20 均反馈正确、`/hardware_status/motors_ready` 为 true 且没有未知索引，
再允许进入控制状态。雷达 odom 与 planner 链路保持原样。

机器人升级后，以下旧配置可以人工检查并删除；本仓库脚本不会自动删除机器已有文件：

```text
/etc/modules-load.d/luvbot-gs-usb.conf
/etc/sudoers.d/can_setup
/etc/sudoers.d/candlelight_setup
/etc/udev/rules.d/candlelight.rules
```

## Todo List
* legged_real移植
* LeggedHardwareInterface中硬件接口底层实现
* 电机标定功能实现
* 接口延迟、系统稳定性测试


## ros1 ros2共存、切换 bash配置
```bash
 # ===== ROS environment switchers (ROS1 Noetic + ROS2 Foxy) =====
 export ROS1_SETUP=/opt/ros/noetic/setup.bash
 export ROS2_SETUP=/opt/ros/foxy/setup.bash

 # Unset ROS variables to avoid cross-contamination
 _ros_clean() {
   unset ROS_VERSION ROS_DISTRO ROS_PYTHON_VERSION
   unset ROS_MASTER_URI ROS_ROOT ROS_PACKAGE_PATH ROSLISP_PACKAGE_DIRECTORIES
   unset AMENT_PREFIX_PATH COLCON_PREFIX_PATH
   unset CMAKE_PREFIX_PATH PYTHONPATH LD_LIBRARY_PATH
 }

 # Activate ROS1 (Noetic)
 set_ros1() {
   _ros_clean
   source "$ROS1_SETUP"
   echo "[OK] ROS1 activated: $ROS_DISTRO"
 }

 # Activate ROS2 (Foxy)
 set_ros2() {
   _ros_clean
   source "$ROS2_SETUP"
   echo "[OK] ROS2 activated: $ROS_DISTRO"
 }

 # Show current ROS env
 rosenv() {
   echo "ROS_DISTRO=$ROS_DISTRO"
   echo "ROS_VERSION=$ROS_VERSION"
   which roscore 2>/dev/null | sed 's/^/roscore: /' || true
   which ros2     2>/dev/null | sed 's/^/ros2: /' || true
 }
 # =============================================================
```



## ROS2北通手柄映射

`rl_imitation`（bydmmc）已移除原 LT/RT + 方向键动作触发及对应的 `/shot_id` 发布。
模型、推理 worker、策略切换、动作执行和结束后回 AMP 的逻辑保留，等待后续接入新的触发接口。
`sim2real.launch.py`、`sim2mujoco.launch.py` 的 `imitation_policies` 默认为 `[]`；
指定 YAML 列表（如 `imitation_policies:="['mma_kick_stable']"`）仅选择并预加载模型，不触发动作。
原手柄动作开关和按键映射配置参数已移除；直接调用 sequential 脚本时需按脚本 `Usage` 使用新的参数顺序。

```shell
axes
[0] - 左摇杆 左右
[1] - 左摇杆 上下
[2] - 右摇杆 左右
[3] - 右摇杆 上下
[4] - RT
[5] - LT
[6] - 十字键 左右
[7] - 十字键 上下
buttons
[0] - A
[1] - B
[2] -
[3] - X
[4] - Y
[5] -
[6] - LB
[7] - RB
[8] - LT
[9] - RT
[10] - back
[11] - start
[12] - home
[13] - 左摇杆 按下
[14] - 右摇杆 按下
```
雷达驱动与深度规划使用 Odin v3.7 的 ROS 接口。导航入口为 `/control/navigation/follow_path`，
类型为 `luvbot_control_interfaces/action/FollowNavigationPath`。导航发送已裁去走过点的剩余路径，
`path` 为 `luvbot_control_interfaces/msg/PathReference`（统一header和 `geometry_msgs/Point[] points`，无逐点姿态），
运控按机器人在路径上的投影位置，向前按0.4米间距采样8点，不足则重复终点；同一修订内游标向前推进，修订后重新投影。
同任务路径更新保留上一条有效速度，输入回调只更新最新缓存，planner 每 200 ms 取缓存推理，
成功后更新历史和 GRU；停车保护独立运行。
默认底层策略为 `l0_v5_radar_velocity_tracking`，默认planner为 `local_planner/model_1600_dense.onnx`；
RK3588实际加载各自配套的 `.rknn`。模型与策略YAML按运控最终版本原样导入，旧11022与2400模型保留供显式回退。
底层仍使用72维观测的5帧历史、21维动作及50Hz推理；planner仍使用2631维观测和128维GRU状态。
planner线速度输出范围为 `[-0.4, 0.8] m/s`，导航参考速度默认 `0.35 m/s`，非零请求限制在 `[0.3, 0.8] m/s`；角速度仍为 `±1.57 rad/s`。
减速和到点判断按机器人到最终目标的直线距离：1.5米内按指数曲线向0.3 m/s减速，距离不超过0.25米进入位置锁定，超过0.5米解除锁定。
路径仍按上游折线采样；`remaining_distance` 反馈保留原路径距离估计，不参与减速和到点判断。因此即使路径尚长，只要接近终点也会按上述规则减速和到点。
最终朝向为 `device0/odom` 下的绝对yaw；位置锁定后，有朝向要求时转至误差不超过10度，无朝向要求则直接到点。
Face 原地转向的最终角速度在叠加模型残差后限制为左转 `[0.5, 1.57]`、右转 `[-1.57, -0.5] rad/s`，
即使请求转速低于0.5也采用此下限；到位、零参考及取消/超时等停车保护仍输出零，不应用该下限。
到点仍按现有Action逻辑反馈Hold并返回成功、清除目标；不执行下载版持续Hold后的45度重新转向。取消、替换、超时与AMP时序保持原有逻辑。
实机入口保持 `nav_check_sensor_timestamps:=false`、`nav_reference_receive_timeout:=true`，使用现有接收时效及参考续期逻辑；
不引入下载版授时开关、推理线程或清单读取。IMU、激活心跳、输入/结果超时、参考过期、命令有效期和停车保护继续生效。
电机映射及使能保持当前版本。新策略上肢动作缩放为0、默认角度为0，手臂及颈部kp为50、颈部kd为1；
由于当前上肢电机仍启用，无头部/摆臂覆盖时会执行这组零位保持参数，不能按下载版上肢禁用的状态验收。

在机器人Linux/ROS2工作空间根目录构建（并放置匹配的 `luvbot_ros_interfaces` 源码）：

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-up-to legged_controllers legged_real \
  --cmake-args -DLEGGED_DEPLOY_BACKEND=RKNN
source install/setup.bash

# 沿用原HPM启动流程后，启用导航；其余功能开关按当前部署需求传入
ros2 launch legged_controllers sim2real.launch.py radar_control_enabled:=true
```

板端确认底层加载 `l0_v5_radar_velocity_tracking/policy.rknn`、planner加载 `model_1600_dense.rknn`，
并确认传感器源时间检查关闭。先确认已启用上肢的零位保持和步行，再验证折线路径采样、近终点减速、
带/不带最终朝向的Action成功，以及取消和参考/传感器接收超时停车；线速度应始终在 `[-0.4, 0.8] m/s`。
供方1600清单记录200步连续验证、最大绝对误差约0.003108，但其速度映射检查裁到 `[-0.4, 0.4] m/s`，
不能替代当前在线0.4～0.8 m/s区间的板端验收。
旧 `/navigation_goal` Float32数组入口已移除，接口定义需与相邻 `luvbot_ros_interfaces` 一起同步构建。
导航接口字段、路径更新/续期/取消逻辑及机器人联调步骤见 [导航Action联调说明](docs/navigation_action_debug.md)。
重新转换模型需在Linux生成并验证配套RKNN；当前在线加载器直接检查实际张量接口和输出，不读取转换清单。

`planner_sensor_bridge` 每 200 ms 处理最新点云。深度结果从本轮处理开始计时，
解包、OpenCL 深度计算和图像消息构造均计入 200 ms 预算，接收后的
排队时间另记为 `queue_ms`。解包后、深度计算后和输出发布前检查期限；超过
200 ms 时以 ERROR 打印 `[depth_timeout]`，说明本轮未跟上 planner 的 5 Hz 节拍，
并丢弃尚未发布的输出。导航启用时始终生成融合深度并发布到 `/planner/depth_fused`，
不再提供深度输入开关或原始深度看图功能。融合所需的原始深度中间值保留，
不再额外输出原始深度图或 mono8 预览。超时本身不会将 GPU 标记为故障，下一轮仍可处理新帧。
OpenCL 调用是同步的，超时计算返回后才丢弃结果，无法在 200 ms 时强制中断 GPU；
200 ms 验收也不保证 DDS 发布调用及订阅端收到消息的耗时。

正常深度帧不打印 INFO；详细耗时诊断保留在 DEBUG 级别，超时及故障仍正常报错。
深度值继续通过深度图话题获取。

深度生成和 planner 推理各按自己的 200 ms 节拍运行，不要求同时启动。
深度生成完成后，`goal_to_planner` 验证新图并立即更新 `/planner_input`，路径参考更新
也立即发送。桥接层保留周期检查，用于刷新目标及发送失效/停车状态，不控制推理频率。
planner 输入回调只覆盖最新缓存；独立的互斥推理定时器每 200 ms 取一次缓存快照，
消息到达、路径更新或上一轮提前完成都不会额外触发推理。推理期间仍可接收新输入，
当前快照不变，新输入用于下一轮；缓存只保留最近已收到且通过验证的输入。

定时器从每次实际推理开始重新计 200 ms，避免调度延迟后补跑。推理超过 200 ms
时 ERROR 打印 `[planner_timeout]` 并丢弃结果，从本次结束后再等 200 ms，避免连续补跑。
取消、策略关闭、目标更换、参考修订和旧快照超时均会阻止已失效的在途结果提交，
原有参考期限、IMU/输入/结果 watchdog 和速度限幅继续生效。普通新深度更新不取消
正在运行的有效快照。没有新深度时仍按 5 Hz 使用最近的有效缓存，超过原有
500 ms 保鲜期限则停止使用；因此“最新”不保证每轮都是不同的深度帧。

正常 planner 推理不打印 INFO 时间日志。深度和 planner 的两个 200 ms
预算互相独立，不表示点云到速度结果的整条链路也限制在 200 ms 内。操作系统调度
抖动、数据失效或计算超时可使有效推理频率低于 5 Hz，不会为追赶频率额外推理。
