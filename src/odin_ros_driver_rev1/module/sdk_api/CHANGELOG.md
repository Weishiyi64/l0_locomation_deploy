# Changelog

本项目遵循 [Keep a Changelog](https://keepachangelog.com/zh-CN/1.1.0/) 规范，
版本号遵循 [Semantic Versioning](https://semver.org/lang/zh-CN/)。

V3.6.0_20261016
1.暴露 ReMap 设备模式：device_mode 支持 mode=5，主机端 loop_closure 切换到 ReMap，固件数据源处理与 SLAM 一致。
2.更新 save_map / load_map 说明，支持 ReMap 加载旧地图并保存增量地图。
3.修复流数据帧率配置失败的问题
4.增加数据丢帧日志警告输出

V3.5.0_20260930

【不兼容变更 / BREAKING CHANGES】升级本版本需修改现有集成代码
B1.OdinCameraPacket移除is_keyframe字段：关键帧判断移交上层，H264由调用方
   按Annex-B起始码扫描NAL类型(IDR/SPS)自行判断，MJPEG帧均可独立解码。
B2.OdinOdomData协方差由cov[18]扩展为cov[36](6x6，x/y/z/roll/pitch/yaw行优先)，
   结构体大小由125字节变为197字节；二代固件(cov[18])仅填充前18项位置协方差，
   姿态协方差行由SDK补零，对上层始终呈现完整6x6布局。
B3.设备实现插件化：Odin2Special从libodin_sdk.so拆分为dlopen插件
   libodin-device-Odin2Special.so(输出至lib/<arch>/目录)，部署时需随
   libodin_sdk.so一并交付对应插件库；Odin1Lite为内置默认回退设备。

【新增功能 / NEW FEATURES】
1.增加Windows 64位平台支持：win64库产物、MSVC编译适配、Windows下设备插件加载
2.增加设备插件加载框架(DeviceLoader)：内置设备注册+dlopen插件搜索、
   IsKnownDevice设备支持查询、插件版本可选查询、插件ABI版本不匹配降级为告警
3.增加odom三代(V3)协议支持：V2/V3结构体兼容、按帧头版本字节路由解析、
   UDP广播V2/V3兼容
4.DiscoveredDevice新增protocol_version字段：从DeviceQuery应答识别设备协议
   代际，用于CreateOdinProtocol选择命令方言
5.OdinResult新增kNotSupported返回值(设备不支持该功能)
6.增加设备相机数量校验(GetCameraCount)，按设备型号校验接入相机数
7.StartStream流程新增设备能力级查询(getcapabilities)
8.设备发现阶段归一化旧固件型号字符串(ODIN)
9.增加Windows端向foxglove发布sensor消息功能
10.3rdparty新增eigen3、openssl第三方库

【行为变更 / BEHAVIOR CHANGES】
11.命令层从设备层拆分为独立模块；协议层内部重构(OdinDataFrameView帧视图、
   ChunkReassemblyState分片重组模板、OdinPointDecoder点云解码、相机/点云
   组帧逻辑统一)，公开API不变

【问题修复 / BUG FIXES】
12.修复同一网卡配置多IP时设备上线时间与模组发送IP不匹配导致热插拔异常的问题
13.修复心跳线程与外部API竞争的问题
14.修复算法库加载失败的问题
15.修复Windows下网络相机广播无法被发现的问题
16.修复Windows下setvbuf崩溃、shutdown无法中断accept、calib_thread未join的问题
17.修复foxglove下载失败的问题


V3.4.0_20260827
1.增加建图/重定位模式：OdinDeviceMode新增kSlam(建图)和KRelocation(重定位)，
  配合主机端回环算法；新增SaveMap/LoadMap扩展接口；重定位锚点通过TF回调发布
  （frame_id=map，child_frame_id=odom）


V3.3.0_20260822
1.删除odom类型里面kLidar2Camera,kLidar2Imu,kOdom2Map
2.删除etImageEncoderParam / GetImageEncoderParam 扩展接口
3.修改GetChannelConfig的注释
4.修复网络相机断链的时候崩溃的问题
5.删除SetImageEncoderParam/SetImageEncoderParam 扩展接口
6.删除sendUserMessage扩展接口


v3.2.0_20260813

【不兼容变更 / BREAKING CHANGES】升级本版本需修改现有集成代码
B1.头文件分层重构：odin_lidar_api.h 拆分为核心 API，新增 odin_lidar_api_ext.h(扩展/调优 API)；
   odin_lidar_def.h 拆分为核心类型定义，新增 odin_lidar_def_ext.h(扩展类型定义)。
   仅用基础功能的用户继续包含 odin_lidar_api.h；使用滤波/编码器/PTP/自适应/网络配置等
   高级接口的用户需改为包含 odin_lidar_api_ext.h。
B2.图像回调接口重命名合并：RegisterImageCallback/RegisterImageCallback2 →
   RegisterCameraCallback/RegisterCameraCallback2；回调类型 OdinImageCallback/
   OdinImageCallback2 合并为 OdinCameraCallback；数据包 OdinImagePacket → OdinCameraPacket。
   同一回调统一承载 MJPEG/H264/NV12 等所有格式，由 packet.format 字段区分编码类型。
B3.设备工作模式重命名：枚举 OdinOperatingMode → OdinDeviceMode；
   接口 SetOperatingMode → SetDeviceMode，并新增配套的 GetDeviceMode。
B4.SetSensorMode 参数强类型化：入参从 uint8_t 改为新增的 OdinSensorMode 枚举
   (kHDR=0/kHighPeak=1/kLowPeak=2/kAdaptive=3)，并新增配套的 GetSensorMode。
B5.IMU/Odom 数据结构体类型化改造：统一为含 frame_count(帧序号) + timestamp(帧时间戳ns)
   + std::vector<样本> 的 Packet 结构(OdinImuPacket/OdinOdomPacket)，协议层字段不再对外暴露。
B6.移除 DiscoveredDevice 结构体中的 firmware_version 字段(改用 GetFirmwareVersion 接口获取)。
B7.命令码(cmd_id)重排以对齐协议 V2，需配套固件版本，旧固件不兼容。
B8.GetNetworkAttribute/SetNetworkAttribute 由核心 API 迁移至扩展 API(odin_lidar_api_ext.h)。
B9.图像格式支持范围调整：移除 yuyv/nv21/rgb24，新增 h264。
B10.sdk库产物从静态库变成动态库，部署时需一并交付 libodin_sdk.so*。

【新增功能 / NEW FEATURES】
1.增加SendUserMessage用户定制化数据下行下发接口(需固件和算法支持、非标准功能)
2.增加sendAlgoData算法数据下行接口(需固件和算法支持、非标准功能)
3.增加SetFilterParam和GetFilterParam滤波接口
4.增加odomHf通道，分离高低频odom传输
5.增加跨网段热插拔发现功能
6.增加相机H264编码能力级的输出
7.增加sdk包生成deb包的功能
8.增加获取时间同步模式接口GetDefaultSyncMode
9.增加mac地址变量在DiscoveredDevice结构体
10.增加网络丢包时候，提示增大网卡缓存的日志
11.增加获取流数据订阅通道状态GetChannelConfig接口
12.增加设置sensorMode=adaptive模式下，adaptive的获取/设置接口SetAdaptiveParam和GetAdaptiveParam
13.增加初始化模组算法状态ResetInitialPose接口
14.增加初始化模组持久化配置(网卡ptp状态、ip、同步模式等)状态接口RestoreDefaultConfig
15.增加TF坐标变换数据输出：新增OdinTfPacket结构体、OdinTfCallback回调类型、
   RegisterTfCallback注册接口，字段对齐ROS2 TransformStamped(frame_id/child_frame_id)
16.增加设备遥测数据输出(0x42通道，约1Hz)：新增OdinTelemetryPacket/OdinTelemetryData结构体、
   OdinTelemetryCallback回调类型、StartTelemetry启动接口、RegisterTelemetryCallback注册接口。
   SDK内部完成JSON解析，对外输出结构化字段(运行时长、CPU/内存占用、系统负载)
17.增加PTP同步状态查询接口GetPtpSyncStatus(同步状态/主从偏移/链路延迟/频率调整量)
18.增加图像编码器参数获取/设置接口SetImageEncoderParam和GetImageEncoderParam
   (JPEG质量、H264码率、H264 QP上下限)
19.增加设置时间同步模式接口SetDefaultSyncMode
20.热插拔回调类型(OnAttachCallback/OnDetachCallback/HotplugCallbacks)移入公开头文件
   odin_lidar_def.h，用户不再需要包含SDK内部头文件

【行为变更 / BEHAVIOR CHANGES】
21.热插拔轮询间隔默认值从1000ms调整为500ms(SetHotplugPollingInterval)

【问题修复 / BUG FIXES】
22.修复网络相机热插拔不连接状态下掉线不检测的问题
23.修复ota升级成功函数无退出的问题
24.修复网络相机丢包导致崩溃的问题
25.修复odom和slam的坐标转换同步机制，由关键点匹配改为时间戳近邻匹配
26.修复数据帧尾部丢包导致点云点数不完整的问题(PadZerosForTailLoss)
27.修复设备发现功能偶发找不到设备的问题

V2.0.3_20260518
1.修改odom结构体名字
2.增加获取和设置模组网络属性接口
3.增加模组重启接口

V2.0.2_20260513
1.增加设备map->odom->base_link转换TF树
2.优化IMU数据传输

V2.0.1_20260426
1.增加心跳检测功能(需要设备兼容)
2.修改网络相机设备机制，使用心跳机制做掉线判断

V2.0.0_20260421
1.增加sensor能力级获取/设置功能和接口
2.修改命令传输从udp改为tcp
3.增加图像数据传输通道配置(udp/tcp)
4.优化传输协议内容
5.增加startStream/closeStream流控制开关接口

V1.0.2_20260326
1.修复slam点云时间戳回调导致闪烁的问题

V1.0.1_20260311
1.增加图片数据校验，过滤不完整图片
2.增加编译优化O3，优化编译速度

V1.0.0_20260307
1.修复ota功能失败的问题
2.增加网络设备热插拔功能
3.增加长短帧模式多实例
4.增加高频里程计输出
5.增加OdomSourceType里程计类型结构体
6.增加slam和odom同步接口多设备模式
7.增加设备数据端口自适配(自动查找可用端口)
8.增加发现设备接口多网卡寻找

V0.3.0_20260205
1.增加标定文件读取功能
2.新增odometry里程计和slam数据输出同步功能接口

V0.2.0_20250130
1.增加odin2s双目彩色图像输出
2.新增OTA升级功能

V0.1.0_20251205
1.初始发版本