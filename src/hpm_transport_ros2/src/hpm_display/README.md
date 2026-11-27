# hpm_display 通信骨架

`hpm_system` 内的 C++ 显示模块，连接 HPM TCP5002，经 J17 UART5 与 ESP32-P4 通信。
HPM 只转发字节；P4 保留 UART1 RX=GPIO4、TX=GPIO5、921600 8N1、UHCI/DMA。
P4 根据机器人状态选择背屏和指示灯表现。现有回调仍仅打印 USB 日志。完整接口见 [V2.0 对外接口说明](docs/hpm_display_ROS2对外接口说明_V2.0.md)。

## 构建与启动

```bash
cd /home/a/HPM/my_project/99_09_motor_adc_imu/deploy_ws
source /opt/ros/humble/setup.bash
colcon build --packages-up-to hpm_system --cmake-args -DAMENT_CMAKE_SYMLINK_INSTALL=OFF
source install/setup.bash
ros2 launch hpm_system hpm_system.launch.py launch_display:=true board_ip:=192.168.1.10 display_port:=5002
```

系统 launch 的 `launch_display` 默认 true。用 `launch_display:=false` 关闭显示。
主链路和显示连接共用 `board_ip`，分别使用 `port`（默认 5001）和 `display_port`（默认 5002）。
启用时读取 `display.yaml` 的超时、重试和保活参数。显示话题与维护服务由 `/hpm_system` 提供。
用 `ros2 topic info -v /display/connection/state` 查看端点。显示运行时日志使用 `hpm_system.display`。
掉线后显示自动重连。显示配置或初始化失败时查看 `Display disabled at startup`。
内部异常时查看 `Display stopped`，修复原因后随系统重新启动显示模块。
环境照度订阅使用 SensorDataQoS（Best Effort，深度 1），其余业务输入使用 Reliable。设备上报和连接状态使用 Reliable + TransientLocal。
真实电池/语音等服务尚未接入，当前使用下列语义测试话题。

## 下行功能测试

P4 的通信 ACK 由节点内部处理。`result=7 RECEIVED` 表示 P4 已校验并分发。
重试结束仍无 ACK 或等待期间链路中断时，节点打印 `P4 ACK missing`，包含功能名、会话号、帧序号、发送次数和原因。
P4 明确拒绝时，节点打印 `P4 ACK rejected` 及结果码。输入值校验失败时，节点直接记录错误。

```bash
ros2 topic echo /display/connection/state
ros2 topic pub --once /robot/battery/state luvbot_hardware_interfaces/msg/DisplayBatteryState '{percent: 75, charge: 1, flags: 0}'
ros2 topic pub --once /robot/bluetooth/state std_msgs/msg/UInt8 '{data: 3}'
ros2 topic pub --once /robot/wifi/state luvbot_hardware_interfaces/msg/DisplayWifiState '{state: 2, signal: 3}'
ros2 topic pub --once /robot/volume/state std_msgs/msg/UInt8 '{data: 30}'
ros2 topic pub --once /robot/mute/state std_msgs/msg/Bool '{data: true}'
ros2 topic pub --once /robot/volume_lock/state std_msgs/msg/Bool '{data: true}'
ros2 topic pub --once /robot/mode/state std_msgs/msg/UInt8 '{data: 2}'
ros2 topic pub --once /robot/dock/state std_msgs/msg/UInt8 '{data: 1}'
ros2 topic pub --once /robot/provision/state luvbot_hardware_interfaces/msg/RobotProvisionState '{id: 7, phase: 4, result: 0, phase_left_s: 60, window_left_s: 240}'
ros2 topic pub --once /robot/fault/state luvbot_hardware_interfaces/msg/DisplayFaultState '{faults: [{id: 4, severity: 3, acknowledged: false, order: 1}]}'
ros2 topic pub --once /robot/upgrade/state luvbot_hardware_interfaces/msg/RobotUpgradeState '{id: 9, target: 3, phase: 2, progress: 45, result: 0}'
ros2 topic pub --once /robot/identity/state luvbot_hardware_interfaces/msg/DisplayIdentityState '{bound: true, name: "小伴 RUMI"}'
ros2 topic pub --once /robot/task/succeeded std_msgs/msg/Empty '{}'
ros2 topic pub --once /robot/interaction/event std_msgs/msg/UInt8 '{data: 2}'
ros2 topic pub --once /robot/voice/state luvbot_hardware_interfaces/msg/DisplayVoiceState '{stage: 3, bins: [32, 80, 160, 64]}'
ros2 topic pub --once /robot/homing/state std_msgs/msg/Bool '{data: true}'
ros2 topic pub --once /robot/night/state std_msgs/msg/Bool '{data: true}'
ros2 topic pub --once /robot/ambient_light/illuminance sensor_msgs/msg/Illuminance '{illuminance: 12.5}'
ros2 topic pub --once /robot/operation/state std_msgs/msg/UInt8 '{data: 3}'
ros2 topic pub --once /robot/battery/state luvbot_hardware_interfaces/msg/DisplayBatteryState '{percent: 48, charge: 3, flags: 4}'
```

预期 USB 控制台日志：`display-cb: volume_set percent=30 business=stub`。
持续状态按值去重并在重连时补发。交互与任务成功每次触发，重连后不重放。
频谱只保留最新待发值。连接断开/维护时拒绝动作事件，状态仍更新缓存。

## P4 上行接口

在 P4 `display.h` 声明，任务上下文调用，参数在函数内复制。
`ESP_OK` 表示入队/缓存成功；未初始化/未建链返回 `ESP_ERR_INVALID_STATE`，
事件队列满返回 `ESP_ERR_NO_MEM`。状态允许断线期间缓存，重连后补发。

| 函数 | ROS2 话题 | 类型 |
|---|---|---|
| display_send_volume_request(percent) | /display/volume/request | DisplayRequest |
| display_send_mute_request(enabled) | /display/mute/request | DisplayRequest |
| display_send_mode_request(mode) | /display/mode/request | DisplayRequest |
| display_send_bluetooth_request(enabled) | /display/bluetooth/request | DisplayRequest |
| display_send_fault_acknowledge(id) | /display/fault/acknowledge | DisplayRequest |
| display_send_provision_request() | /display/provision/request | DisplayRequest |
| display_update_panel_state(status,error) | /display/panel/state | DisplayDeviceState |
| display_update_backlight_state(status,error,brightness) | /display/panel/state | DisplayDeviceState |
| display_update_touch_state(status,error) | /display/panel/state | DisplayDeviceState |

订阅 `/display/panel/state` 后按 `target` 分流。0 为面板，1 为背光，2 为触摸。每条消息对应一个部件。
状态码为 0 未知、1 就绪、2 故障、3 禁用。只有背光消息的 `brightness` 有效。
本体灯和桩灯均为开环控制。灯具没有状态、亮度或故障回传。下行通信回执由 P4 生成，只确认收到机器人状态或事件。
请求含 boot_id、sequence、event_id、value；独立重复操作产生新 event_id。
业务服务可向同功能 `/display/volume/result` 等发布 DisplayResult，填写
`event_id/result/reason/revision`；可用 `session=boot_id` 指定请求来源。
节点仅接受本次连接已接收的请求对应结果，P4 的结果回调同样只打印。

```bash
ros2 topic echo /display/volume/request
ros2 topic echo /display/panel/state
# 将 event_id/boot_id 换成上行请求实际值；0=业务成功，通信层不会自行生成成功。
ros2 topic pub --once /display/volume/result luvbot_hardware_interfaces/msg/DisplayResult '{session: 153, event_id: 1, result: 0, revision: 1}'
```

P4 `CONFIG_DISPLAY_TEST_UPLINK=y` 是测试专用选项：首次 HELLO 后调用全部上行
接口一次。默认 n，正式固件不生成虚构状态，也不连接实际硬件反馈。

## OTA 维护交接

```bash
ros2 service call /display/upgrade/maintenance std_srvs/srv/SetBool '{data: true}'
# 等 success: true，连接已释放，然后使用原 OTA 命令
python3 src/hpm_transport_ros2/scripts/ota/ota_send.py esp32p4 --query
python3 src/hpm_transport_ros2/scripts/ota/ota_send.py esp32p4
ros2 service call /display/upgrade/maintenance std_srvs/srv/SetBool '{data: false}'
```

HPM 自升级也先释放显示连接。维护态不自动抢连；脚本崩溃后由操作人员恢复。
维护/断线期间只保留最新状态，丢弃尚未执行的动作，不盲目重放操作。
普通保活不重复打印。保活间隔、业务 ACK 超时和重试次数由 `display.yaml` 配置。

## 帧与字段

P4UP 20 字节头 + payload + CRC32/ISO-HDLC（小端），payload≤1100；
Ethernet TCP 载荷与 UART 有效帧逐字节一致，DMA 对齐零在帧外。

| 类型 | payload |
|---|---|
| 0x10 SET_STATE | boot_id:u32, revision:u32, key:u16, value[] |
| 0x11 INPUT_EVENT | event_id:u32, action:u16, value:u16；头session=P4 boot_id |
| 0x12 RESULT | event_id:u32, result:u16, reason:u16, revision:u32；通信回执event_id=0，回显头session/sequence |
| 0x13 REPORT_STATE | P4 boot_id:u32, revision:u32, key:u16, value[]；头session=P4 boot_id |

业务结果从3588下发时 event_id 非零；P4 为它返回 event_id=0、revision=0 的通信回执。
现有 OTA HELLO/QUERY/BEGIN/DATA/END/COMMIT/ABORT、100字节ACK和BOOT_REPORT保持原格式。
3588 每次 TCP 建连使用新的帧 session；SET_STATE 的 boot_id 标识节点启动代次，
两者职责不同。P4 在新的 HELLO 会话丢弃结果不明的动作，只补发最新设备状态。

| key | 功能 | value布局 | P4回调 |
|---|---|---|---|
| 0x0001 | battery | percent:u8,charge:u8,flags:u16 | display_on_battery_state、display_on_charge_progress（STA-002） |
| 0x0036 | ambient_light | illuminance:f64，小端，单位 lx | display_on_ambient_light |
| 0x0037 | operation | state:u8，0 离线、1 在线、2 低电等待、3 回充途中、4 正在充电 | display_on_operation_state（STA-001） |
| 0x0004 | dock | state:u8 | display_on_dock_state |
| 0x0030 | provision | id:u32,phase:u8,result:u8,phase_left:u16,window_left:u16 | display_on_provision_state |
| 0x0006 | fault | count:u8 + records{id:u16,severity:u8,ack:u8,order:u32} | display_on_fault_state |
| 0x0031 | upgrade | id:u32,target:u8,phase:u8,progress:u8,result:u8 | display_on_upgrade_state |
| 0x0008 | identity | bound:u8,UTF8长度:u8,name[] | display_on_identity_state |
| 0x0034 | interaction | 来源:u8，1 语音唤醒，2 触摸，3 App | P4 本地仲裁 |
| 0x0035 | task success | 成功事件:u8，固定 1 | P4 本地仲裁 |
| 0x0011 | voice | stage:u8,count:u8,bins[] | display_on_voice_state |
| 0x0032 | homing | bool:u8 | P4 本地仲裁 |
| 0x0033 | night | bool:u8 | P4 本地仲裁 |
| 0x0020 | volume | percent:u8 | display_on_volume_set |
| 0x0021 | mute | bool:u8 | display_on_mute_set |
| 0x0022 | mode | mode:u8 | display_on_mode_set |
| 0x0023 | volume_lock | bool:u8 | display_on_volume_lock_set |
| 0x0024 | bluetooth | state:u8 | display_on_bluetooth_state |
| 0x0025 | wifi | state:u8,signal:u8 | display_on_wifi_state |
| 0x8000 | panel上报 | status:u8,error:u16 | 上行函数 |
| 0x8001 | backlight上报 | status:u8,error:u16,brightness:u8 | 上行函数 |
| 0x8002 | touch上报 | status:u8,error:u16 | 上行函数 |

本版以独立字段替代 D0.2 的 MODE_AUDIO/CONNECTIVITY 复合输入；不发送旧复合
key 或 SNAPSHOT TLV，重连逐项补发最新状态。D0.2 是历史设计草案，不是旧固件ABI。
协议校验只判断长度/枚举/编码，不做低电阈值、灯优先级、模式联动等业务判断。
原灯状态上报字段 `0x8003` 已停用并保留。ROS2 收到旧帧时返回协议结果 `UNSUPPORTED`。

## 离线测试

```bash
colcon test --packages-select hpm_display hpm_system --return-code-on-test-failure \
  --ctest-args -R display --output-on-failure
colcon test-result --verbose
```

运行测试覆盖19类机器人下行、6类输入事件、3类设备状态、CRC/分段、丢ACK重试、
去重、超时重连、维护交接，以及同一个board_ip下原端口与display_port相互隔离。
离线测试不证明物理UART或屏灯动作，实机验证需安装匹配的P4应用后进行。

V1.1 历史验证（2026-11-17）：9个ROS2包及P4固件编译通过；协议单元测试、ROS2模拟
链路、系统launch端口隔离、P4原生业务测试及既有OTA回归通过。没有烧录P4，
没有部署远端，没有执行物理屏灯或电机动作。
