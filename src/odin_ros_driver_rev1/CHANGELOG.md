本项目遵循 [Keep a Changelog](https://keepachangelog.com/zh-CN/1.1.0/) 规范，
版本号遵循 [Semantic Versioning](https://semver.org/lang/zh-CN/)。
3.7.0_20261016
1.新增 loop-closure 可视化话题，发布载入地图、关键帧矫正结果、优化结果及里程计轨迹。
2.新增 ReMap 模式（device_mode=5，OdinDeviceMode 新增 kReMap），支持基于已载入地图扩展建图；主机端 loop_closure 切换到 ReMap，固件数据源处理与 SLAM 一致；SaveMap/LoadMap 说明更新为支持 ReMap；新增 corrected loop-closure 点云发布。
3.恢复 map_cloud 回调注册；RViz 以三层色阶区分载入、矫正和优化点云，并显示连续里程计轨迹。
4.修复 map_cloud 坐标系和 RViz 轨迹残留时间，恢复 SDK 算法控制与回调互斥锁分离。
5.ROS service 返回消息和日志改用 OdinResultStr 枚举名，替代裸错误码。
6.新增 ExployMap 闭环地图导出接口：loopclosure 插件 ABI、SDK ExployMap API 及 ROS1/ROS2 exploy_map service 完成透传；具体导出格式由插件实现。
7.增加openssl和crypto的静态库

## [3.6.0] - 2026-09-30

### Added
- 增加FFmpeg H.264解码器运行时硬件加速自动检测与动态解码器发现（NVDEC/CUVID/VAAPI/VDPAU），
  硬件解码失败时自动回退到软件解码
- 增加VAAPI/VDPAU硬件解码支持（Intel/AMD GPU），并支持通过环境变量指定设备节点：
  ODIN_CUDA_DEVICE / ODIN_VA_DEVICE / ODIN_VDPAU_DEVICE
- 增加libyuv可选集成，加速YUV→BGR转换（H.264/JPEG解码及NV12合成路径，较libswscale/OpenCV
  约快7倍），运行时可用ODIN_DISABLE_LIBYUV=1禁用
- 增加docs/CameraStreamDecoding.md文档，说明H.264解码流程、硬件/软件回退逻辑及各平台前置条件
- 设备专属启动文件 odin1Lite.launch / odin1Lite.launch.py（单相机）和
  odin2Special.launch / odin2Special.launch.py（双相机）
- RViz 配置按设备拆分：odin1lite_driver_view_ros1/ros2.rviz（单相机）和
  odin2special_driver_view_ros1/ros2.rviz（双相机）
- 驱动节点连接设备后自动检测相机数量（GetSensorCapability 查询 kImage1），
  单相机设备自动禁用 image1 通道、回调、队列和发布者
- launch 文件新增 camera_count 参数，根据相机数自动选择对应 RViz 配置
- SDK 层增加 GetCameraCount() 虚函数，Odin1Lite 返回 1，Odin2Special 返回 2

### Changed
- **BREAKING**: 话题/服务命名移除设备型号段：逐设备前缀由 /{topic_prefix}/{model}/device{N}/ 改为
  /{topic_prefix}/device{N}/（如 /manifold/ODIN2/device0/ → /manifold/device0/），
  不同型号设备共用稳定路径，话题/服务不再随设备型号变化；TF frame 前缀 device{N}/ 不受影响。
- device_online 消息 JSON 增加设备型号字段：{"sn":"...","model":"...","prefix":"...","ip":"..."}。
- **BREAKING**: device_offline 消息由纯 SN 字符串改为 JSON：{"sn":"...","model":"..."}。
  下游节点（post_process / pointcloud_to_depth / opencv_undistort / depth_to_pointcloud）
  已兼容新旧两种格式，但自行订阅该话题的外部脚本需按新格式适配。
- 同步更新 rviz 配置、rosbag2_qos.yaml、launch 文件注释以及 README / docs 中的
  示例话题名前缀（/manifold/ODIN2/device0/ → /manifold/device0/）。
- 设备发现增加型号支持检查：上报型号既无 builtin 实现、搜索路径上也没有对应
  插件的设备不再进入设备列表（此前会在连接时静默回退到不匹配的实现），并输出
  WARN 日志提示；空型号字段归一为 Odin1Lite。
- 遗留固件上报的裸型号 "Odin"/"ODIN"（大小写不敏感）统一映射为 Odin1Lite，
  保持可识别、可连接。
- 优化解码器初始化及首帧时的日志输出
- SDK 层对单相机设备拒绝 kImage1 开流/关流/获取配置/注册回调2，返回 kNotSupported
- SDK 设备实现插件化：Odin2Special 拆分为 dlopen 插件库
  libodin-device-Odin2Special.so（lib/<arch>/），部署时需随 libodin_sdk.so 一并交付；
  Odin1Lite 为内置默认回退设备

### Fixed
- odometry_lf / odometry_hf 话题填充设备上报的 6×6 位姿协方差（pose.covariance，
  行主序 (x,y,z,roll,pitch,yaw)，二代固件仅前 18 项位置行有效），此前恒为 0；
  twist 协方差设备未上报，保持 0。

### Removed
- 原通用启动文件 driver.launch / driver.launch.py（重命名为 odin2Special.launch /
  odin2Special.launch.py）：引用旧文件名的启动命令/脚本需改用设备专属 launch 文件
- 原 driver_view_ros1.rviz / driver_view_ros2.rviz

3.5.0_20260921
1.增加设备专属启动文件 odin1Lite.launch / odin1Lite.launch.py（单相机）和 odin2Special.launch / odin2Special.launch.py（双相机）
2.RViz 配置按设备拆分：odin1lite_driver_view_ros1/ros2.rviz（单相机，话题前缀 /manifold/Odin1Lite/）和 odin2special_driver_view_ros1/ros2.rviz（双相机，话题前缀 /manifold/Odin2Special/），删除原 driver_view_ros1.rviz / driver_view_ros2.rviz
3.驱动节点连接设备后自动检测相机数量（GetSensorCapability 查询 kImage1），单相机设备自动禁用 image1 通道、回调、队列和发布者
4.launch 文件新增 camera_count 参数，根据相机数自动选择对应 RViz 配置
5.SDK 层增加 GetCameraCount() 虚函数，Odin1Lite 返回 1，Odin2Special 返回 2
6.SDK 层对单相机设备拒绝 kImage1 开流/关流/获取配置/注册回调2，返回 kNotSupported
7.默认设备从 Odin1Lite 改为 Odin2Special，未匹配到 builtin 设备时回退到 Odin2Special

3.4.0_20260827
1.增加save_map / load_map闭环地图保存与加载service
2.增加device_mode设备工作模式切换service

3.3.0_20260827
1.增加h264在jetson平台硬件解码器支持
2.增加h264在rockchip平台硬件解码器支持

3.2.0_20260822
1.修复增加ubuntu20.04在rk平台硬编码不支持的问题
2.增加cv_bridge 与 OpenCV 版本不匹配导致崩溃问题描述<troubleshooting.md>
3.增加cloud_raw的FOV滤波和参数设置enable_row_filter和min_row过滤行数
4.修复点云发布插件point-cloud-transport
5.删除部分参数在launch.py和对应动态配置
6.增加set_channel_config service 和 get_channel_config的ros服务
7.增加 set_post_process_config service and get_post_process_config的ros服务
8.移除 control_command.yaml文件，参数移动到对应ros的launch文件配置
9.增加opencv_undistort去畸变节点
10.删除odom_lidar2cam 和 odom_lidar2imu的发布
11.增加部分参数和节点的动态开关和配置


3.1.0_20260813

【不兼容变更 / BREAKING CHANGES】升级本版本需修改现有 launch/配置文件
B1.launch 参数重命名：operating_mode → device_mode（对齐 SDK 的 OdinDeviceMode 重命名）。
   现有自定义 launch 文件中的 operating_mode 配置将不再生效，需手动改名。
B2.image_format 可选值调整：移除 yuyv/nv21/rgb24，新增 h264；现支持 mjpeg|h264|nv12。
B3.sensor_mode 默认值从 1(High Peak) 改为 -1(不下发，保持设备当前模式)。
   若依赖启动时强制切到 High Peak，需在 launch 中显式指定 sensor_mode:=1。
B4.各传感器 frame_id 默认值调整（从统一的 odom 改为语义化命名）：
   raw_frame_id: odom → lidar；gray_image_frame_id: odom → lidar；
   image_frame_id: odom → camera0；image2_frame_id: odom → camera1；imu_frame_id: odom → imu。
   依赖旧 frame_id 的下游节点/rviz 配置/TF 树需同步更新。
B5.依赖的 SDK 产物从静态库变为动态库，部署时需一并交付 libodin_sdk.so*。

【新增功能：ROS Service / NEW SERVICES】
1.增加设置/获取同步模式的service（set_sync_mode / get_sync_mode）
2.增加设置/获取 adaptive_param 模式参数的service（set_adaptive_param / get_adaptive_param）
3.增加设置初始化算法状态resetInitialPose的service
4.增加恢复设备默认持久化配置的service（restore_default_config）
5.增加PTP同步状态查询的service（get_ptp_sync_status，含同步状态/主从偏移/链路延迟/频率调整量）
6.增加诊断监控开关的service（set_diag_monitor），可运行时开关 1Hz 诊断输出，
  并返回各传感器帧率与平均延迟（raw/slam点云、image0/1、imu、odom_lf/hf、tf）

【新增功能：launch 参数 / NEW LAUNCH PARAMETERS】
7.增加设备端滤波器launch参数：filter_confidence_threshold、filter_adaptive_cdf_percentile、
  filter_adaptive_outdoor_thr、filter_range_min_depth（-1 表示不下发，使用设备默认值）。
  注意：自适应参数走 SetAdaptiveParam 专用接口下发，confidence/range 走 SetFilterParam
8.增加tf_publish_rate launch参数，可配置TF定时重发频率（0=禁用定时重发）

【新增功能：数据与诊断 / NEW FEATURES】
9.增加H264解码BGR8输出到rviz的功能
10.增加lidar的lidar filter的设置
11.增加统计sdk端发送帧率和延迟的service
12.增加diag_subscriber节点，统计ros端发布消息帧率和ros端传感器数据延迟
13.增加不同sensor下对应点云confidence滤波参数下限限制
   （HighPeak≥7、LowPeak≥2、Adaptive下关闭客户端滤波由设备端处理、HDR无下限），
   rqt 滑块下限随 sensor_mode 动态变化
14.增加部分tf转换关系

【行为变更 / BEHAVIOR CHANGES】
15.修改图像数据frame_id（详见 B4）
16.BuildTopicPrefix增加更多设备信息输出（SN/型号/IP/MAC），
   设备sensor mode查询日志降级为debug避免刷屏

【问题修复 / BUG FIXES】
17.修复launch文件参数配置不生效的问题（自适应滤波参数原先误用SetFilterParam下发，
   现改为专用SetAdaptiveParam接口）
18.修复ubuntu24.04下rviz下 TF 数据转换失败的问题

1.0.0_20260720
1.修复ubuntu24.04编译不通过问题
2.增加深度转稠密点云功能
3.增加瑞芯微RK3588图片Mjpeg硬解码功能
4.删除ess深度节点
5.增加cloud_raw转深度(pointcloud_to_depth)、深度转稠密点云(depth_to_pointcloud)、去畸变(undistort)、立体矫正(rect)的功能开关在配置文件(control_command.yaml)
6.修复cloud_raw的置信度(confidence)滤波参数launch文件默认配置不生效的问题
7.增加Readme.md对DHCP配置的说明

0.12.0_20260601
1.增加点云转深度节点
2.增加launch文件对topic_prefix的设置

0.11.0_20260518
1.增加重投影功能，control_command.yaml配置投影mode，“slam”投影模式暂不支持
2.修复rviz卡顿问题
3.增加lidar2camera和lidar2imu的odom数据发布

V0.10.0_20260513
1.增加map->odom->base_link转换TF树
2.增加publish发送qos深度
3.增加ros bag录制防丢包的qos策略

V0.9.0_20260426
1.修复ros1分辨率设置不生效问题
2.修复ros1编译不通过问题
3.增加数据流配置在配置文件(control_command.yaml)
4.兼容24.04 jazzy编译

v0.8.0_20260421
1.增加launch文件配置分辨率
2.修复图片解吗阻塞回调的问题
3.增加流控制功能
4.增加能力级获取

V0.7.0_20260415
1.增加标定文件多分辨率读取功能
2.增加lidar的adpative模式和HDR模式

V0.6.0_20260326
1.修复cloud slam时间戳回调导致的闪烁问题

V0.5.0_20260311
1.增加长短帧模式切换的ros1/ros2 service
2.增加MJPEG图片的jetson硬解码(需要机器支持)
3.优化去畸变算法
4.增加rqt对于sdk的日志打印等级

V0.4.0_20260307
1.修复去畸变图像输出时，ros1/ros2的header无时间戳
2.修改注册数据回调操作放到链接相机
3.增加高频里程计话题的发布
4.修改设备发现方式从主动发现修改为热插拔
5.增加原始点云数据点时间戳offset_time功能(区别于帧时间戳)
6.修改ros1/ros2的参数配置param命名空间

V0.3.1_20260206
1.修复o1lite/o2s标定文件不同导致启动崩溃

V0.3.0_20260205
1.增加双目去畸变立体校正图像输出(topic:xxx/image_rect)
2.增加读取模组标定文件功能
3.修改效果：
    原效果：超过一定置信度过滤raw点云的点。
    现效果：超过一定置信度过滤raw点云的点，并且将这些点设置为(0,0,0)。
4.增加双目视差图点云(NVINDA GPU only) 
### 需要ISAAC ROS ESS硬件要求

V0.2.0_20250130
1.增加双目图像去畸变图像输出
2.增加强弱光模式切换配置
3.增加置信度过滤点云配置

v0.1.0
初始版本发布