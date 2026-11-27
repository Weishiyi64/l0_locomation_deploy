# hpm_display ROS2 对外接口说明 V2.0

`hpm_display` 由 `hpm_system` 的 `launch_display` 参数控制。启用后，以下话题与
`/display/upgrade/maintenance` 服务注册在 `/hpm_system`，显示通道使用独立的 `display_port`（默认 5002）。

## hpm_display 订阅的话题

| 话题 | 需求文档及编号 | 对应需求内容 |
|---|---|---|
| `/robot/battery/state` | 背屏 PAGE1-01、MSG-05、MSG-06<br>指示灯 S-04～S-08、STA-001、STA-002 | 电池电量、充电状态与告警供背屏、电池灯效和 App 共用。首页充电百分比取 `percent`（255 表示未知）。充电中或电量充足时，App 电池图标为绿色。未充电时，低于 20% 持续 10 秒变橙，低于 10% 立即变红。过热暂停（`charge=3` 且 `flags` 的 bit2 置位）保留原图标颜色，恢复充电后更新。 |
| `/robot/operation/state` | 指示灯 STA-001 | 机器人业务服务发布首页设备状态（`std_msgs/msg/UInt8`）。0 离线，可引导重新配网。1 在线正常使用。2 低电且无法自行回充，原地等待。3 正在返回充电桩。4 已接桩充电，此时用电池话题补充百分比。App 可直接订阅该状态，P4 收到后打印。 |
| `/robot/bluetooth/state` | 背屏 PAGE1-02～PAGE1-04 | 显示蓝牙关闭、已开启但未连接、连接中、已连接四种状态，配网完成或用户关闭蓝牙后更新图标。 |
| `/robot/wifi/state` | 背屏 PAGE1-06～PAGE1-09 | 显示 Wi-Fi 离线、连接中或已连接，连接后显示信号格数，掉线后引导用户在 App 重新配网。 |
| `/robot/volume/state` | 背屏 PAGE2-02、PAGE2-03、PAGE3-03、PAGE3-07 | 按实际音量更新滑块和喇叭声波，反映退出静音、关闭安静模式或重启后恢复的音量。 |
| `/robot/mute/state` | 背屏 PAGE2-01、PAGE2-03 | 静音时显示静音图标、滑块归零，解除后按恢复的实际音量显示。 |
| `/robot/volume_lock/state` | 背屏 PAGE2-04、PAGE3-03 | 安静模式下将音量控件置灰，用户尝试调节时提示音量不可调。 |
| `/robot/mode/state` | 背屏 PAGE3-01～PAGE3-07（含 PAGE3-02 隐私模式） | 显示当前生效的隐私、安静、待机、户外或换装模式。切换后只显示新模式，重启后清除原模式。PAGE3-02 以此话题确认隐私模式已生效。 |
| `/robot/dock/state` | 背屏 PAGE3-04、PAGE3-06<br>指示灯 S-04 | 待机或换装时显示“在巢内”。在巢状态与电池充电状态共同用于判断本体灯和桩灯的充电提示。 |
| `/robot/provision/state` | 背屏 PAGE1-03、PAGE1-05、PAGE1-07～PAGE1-09、SYS-01<br>指示灯 S-09 | 显示蓝牙配对和 Wi-Fi 配网的进度、成功、失败或超时。蓝牙配对期间暂停自动熄屏，配对或配网期间本体灯蓝色快闪。 |
| `/robot/fault/state` | 背屏 MSG-01、MSG-03、MSG-04、MSG-07～MSG-10<br>指示灯 S-08 | 接收机器人故障集合，按发生顺序展示消息。背屏按故障解除或用户知悉更新提示及光圈，本体故障灯持续到故障解除。本体灯和桩灯均为开环控制，灯具没有状态、亮度或故障回传。XBN-005 灯具异常告警需要独立的故障来源。 |
| `/robot/upgrade/state` | 背屏 MSG-02、MSG-08<br>指示灯 S-10 | 根据升级目标和阶段决定表现。系统升级时提示并锁定操作、暂缓其他消息，本体灯紫色呼吸。小屏自身升级时黑屏、本体灯熄灭。 |
| `/robot/identity/state` | 背屏 PAGE1-03 第 5 条 | 提供蓝牙配对时展示的机器人名称，支持默认名称和 App 重命名后的名称。 |
| `/robot/interaction/event` | 指示灯 S-02 | 语音唤醒、触摸或 App 遥控发生时，本体灯蓝色呼吸 3 秒，确认已收到交互。 |
| `/robot/task/succeeded` | 指示灯 S-03 | 用户指令执行成功后，本体灯绿色常亮 2 秒。 |
| `/robot/voice/state` | 指示灯 S-12 | 语音对话期间本体灯黄色呼吸，对话结束后延续 3 秒。该需求使用语音阶段，频谱没有单独的背屏需求编号。 |
| `/robot/homing/state` | 指示灯 S-11、STA-001 | 自动寻桩时本体灯绿色慢闪、桩灯绿色呼吸，归位或寻桩失败后结束提示。寻桩状态也可供 App 首页显示“回充中”。 |
| `/robot/night/state` | 指示灯 §4.3、S-02、S-06、S-12 | 夜间将交互确认和语音灯效亮度调至 20%，低电提醒调至 30%。常态继续熄灭，严重低电和故障保持原亮度。 |
| `/robot/ambient_light/illuminance` | 背屏 SYS-02（联调） | 使用 `sensor_msgs/msg/Illuminance` 的 `illuminance` 数值直接设置 P4 背光 PWM 占空比。0 关闭背光，30 对应约 30%，100 和更高值对应 100%。P4 在屏幕初始化完成后应用最新值。 |
| `/display/volume/result` | 背屏 PAGE2-02、PAGE2-04 | 返回拖动滑块后的调音结果，供背屏反馈调节成功或音量受限。 |
| `/display/mute/result` | 背屏 PAGE2-01、PAGE2-03、PAGE2-04 | 返回点击喇叭后静音或解除静音的处理结果。 |
| `/display/mode/result` | 背屏 PAGE3-01～PAGE3-07（含 PAGE3-02 隐私模式） | 按 `event_id` 返回本次模式切换结果。PAGE3-02 要求在开启请求成功、模式状态确认生效后，由 P4 将整屏背景变暗，居中显示带斜杠的相机图标及“隐私模式已开启”，2 秒后返回 PAGE3 原状态。 |
| `/display/bluetooth/result` | 背屏 PAGE1-03～PAGE1-05 | 返回蓝牙开启或关闭的处理结果，供背屏提示开启失败等异常。 |
| `/display/fault/result` | 背屏 MSG-04 | 返回业务方对“我知道了”的处理结果，供背屏结束该条送修提示及光圈闪烁。 |
| `/display/provision/result` | 背屏 PAGE1-09（关联） | 返回配网请求是否受理。需求规定用户在背屏看到离线引导后，从 App 发起重新配网。 |

## hpm_display 发布的话题

P4 下行通信 ACK 由 `hpm_display` 内部处理。重试后仍未收到 ACK，或等待期间链路中断时，节点在终端打印 `P4 ACK missing`，记录功能名、会话号、帧序号和发送次数。

| 话题 | 需求文档及编号 | 对应需求内容 |
|---|---|---|
| `/display/volume/request` | 背屏 PAGE2-02、PAGE2-04 | 用户拖动音量滑块，向机器人请求设置目标音量。安静模式下调节受限。 |
| `/display/mute/request` | 背屏 PAGE2-01、PAGE2-03、PAGE2-04 | 用户点击喇叭图标，请求静音或解除静音。 |
| `/display/mode/request` | 背屏 PAGE3-01～PAGE3-07（含 PAGE3-02 隐私模式） | 用户滑动选择模式并确认后，向机器人请求切换模式。PAGE3-02 要求 P4 在轮播停驻隐私模式至少 0.5 秒时高亮，松手后弹窗询问是否开启。用户确认后才发布开启请求。 |
| `/display/bluetooth/request` | 背屏 PAGE1-03、PAGE1-04 | 用户点击蓝牙图标请求开启蓝牙，或在确认弹窗中同意关闭蓝牙并断开手机连接。 |
| `/display/fault/acknowledge` | 背屏 MSG-04 | 用户点击“我知道了”，向机器人上报已知悉的故障编号。 |
| `/display/provision/request` | 背屏 PAGE1-09（关联） | 屏端配网请求接口，对应离线重配网引导场景。需求规定实际重配网由 App 发起。 |
| `/display/panel/state` | 背屏 §2.1、SYS-01、SYS-02、PAGE2-02、PAGE3-01（诊断） | 每条 `luvbot_hardware_interfaces/msg/DisplayDeviceState` 消息上报一个部件的 `status` 和 `error`。`target=0` 为面板，`target=1` 为背光，`target=2` 为触摸。仅背光消息包含有效的 `brightness`，用于核对亮屏、变淡、熄屏和调光。触摸消息用于排查点击、滑动和拖动问题。 |
| `/display/connection/state` | 工程诊断，两份需求文档均无独立编号 | 上报背屏通信链路的连接、在线、离线或维护状态。显示模块因内部异常停用时上报 `stopped`，具体原因见 `hpm_system.display` 日志。 |
