# hpm_display ROS2 对外接口说明

版本 V1.0　｜　日期 2026 年 11 月 17 日　｜　接口基准 当前 hpm_display 实现

## 1 适用范围和实现状态

本文供 RK3588 上的机器人业务、音频、网络、升级及 App 接入开发者使用，定义背屏通信节点 hpm_display 当前订阅和发布的 ROS2 话题、消息字段、发送时机、反馈方式和联调命令。

调用者向本文列出的输入话题发布业务数据，hpm_display 将数据传给 ESP32-P4；调用者订阅本文列出的输出话题，接收屏端用户请求、设备状态和通信回执。调用者不需要自行拼接 TCP 或 UART 协议帧。

链路为：3588 业务节点 → ROS2 → hpm_display → HPM TCP 5002 → UART → ESP32-P4，反馈沿相反方向返回。HPM 只转发字节，不计算业务状态、不决定灯效、不生成业务结果。

当前已实现话题转换、双向传输、字段校验、重连、去重、回执和 P4 回调分发。P4 下行业务回调目前只通过 USB 日志打印名称和参数；实际屏幕、音量、灯光、配网等业务动作仍待接入。设备上报函数已实现编码和发送，但由实际业务代码提供数据并主动调用，不会自动采集或伪造设备状态。

本接口共包含 21 个订阅话题、26 个发布话题、1 个维护服务。其中订阅为 15 个功能输入加 6 个执行结果输入；发布为 6 个用户请求、4 个设备状态、15 个通信回执及 1 个连接状态。以下所有“订阅”“发布”均以 hpm_display 节点为参照。

## 2 接入启动和基本约定

### 2.1 启动方式

远端部署目录为 ~/zyc_ws。先加载 ROS2 和工作区环境；开发者在其他工作区调用时，应编译或安装同版本 robot_interfaces 消息包。

```bash
cd ~/zyc_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
```

独立联调只启动显示节点：

```bash
ros2 launch hpm_display hpm_display.launch.py \
  board_ip:=192.168.1.10 port:=5002 reconnect_ms:=1000
```

系统集成启动：

```bash
ros2 launch hpm_system hpm_system.launch.py \
  launch_display:=true board_ip:=192.168.1.10 \
  port:=5001 display_port:=5002
```

两种方式选一种，同一 TCP5002 通道只运行一个 hpm_display 实例。系统启动共用 board_ip；port 属于原 hpm_system 通道，display_port 属于显示通道。launch_display 默认 false，关闭时不启动该节点、不创建该节点的订阅、发布及维护服务、不连接 TCP5002。显示开关独立于电机、IMU、UDP 开关；系统启动中其他功能是否运行由其各自配置决定。

### 2.2 调用约定

- 所有话题使用以 /display 开头的绝对名称。自定义消息的完整类型前缀统一为 robot_interfaces/msg/，下文表格中省略此前缀；标准消息保留完整类型。
- 机器人业务服务负责产生权威状态。hpm_display 负责传输，P4 负责显示和提交用户请求。音量锁定、模式限制、低电量阈值、故障优先级、灯效仲裁等判断由业务方完成。
- 每个字段的数值均按消息类型传递；百分比为整数，时间字段的单位由后缀 _s 或 _ms 明确区分。未知值只在明确允许的字段中使用 255。
- 普通状态输入只在内容变化时发送，断线期间保留各项最新值，重连后同步。节点重启会丢失内存缓存，业务方应重新发布完整当前状态。
- 表情是操作事件，重复发布同一内容也会发送；离线或维护期间不缓存表情操作。语音频谱在待发缓存中只保留最新值，不保证每一帧都播放。
- 同一功能建议由一个权威业务节点发布。hpm_display 不处理多个发布者之间的优先级或冲突。

## 3 hpm_display 订阅的功能输入

业务方负责发布下表话题。P4 当前收到后调用对应的日志回调；本表的“用途”说明最终业务目的，不表示当前已执行相应硬件动作。

| 功能和预期用途 | 输入话题 | 消息类型 |
|---|---|---|
| 电量和充电显示 | /display/battery/state | DisplayBatteryState |
| 蓝牙状态显示 | /display/bluetooth/state | std_msgs/msg/UInt8 |
| Wi-Fi 状态和信号显示 | /display/wifi/state | DisplayWifiState |
| 设置显示端音量状态 | /display/volume/set | std_msgs/msg/UInt8 |
| 设置静音状态 | /display/mute/set | std_msgs/msg/Bool |
| 设置音量调整锁定 | /display/volume_lock/set | std_msgs/msg/Bool |
| 设置模式状态 | /display/mode/set | std_msgs/msg/UInt8 |
| 显示在巢状态 | /display/dock/state | std_msgs/msg/UInt8 |
| 显示配网阶段和结果 | /display/provision/state | DisplayProvisionState |
| 显示当前故障集合 | /display/fault/state | DisplayFaultState |
| 显示升级阶段和进度 | /display/upgrade/state | DisplayUpgradeState |
| 显示绑定和设备名称 | /display/identity/state | DisplayIdentityState |
| 提交表情播放动作 | /display/expression/set | DisplayExpression |
| 显示语音阶段和频谱 | /display/voice/state | DisplayVoiceState |
| 下发已决策的指示灯输出 | /display/indicator/set | DisplayIndicator |

### 3.1 简单消息和发送时机

| 话题功能 | 字段和取值 | 建议业务发送时机 |
|---|---|---|
| bluetooth | uint8 data，允许 0～4；当前共享校验代码未定义各值的产品语义，联调前应统一枚举表 | 实际蓝牙状态变化时 |
| volume | uint8 data，0～100，单位百分比 | 音量执行完成或权威音量变化时 |
| mute | bool data，true 静音，false 取消静音 | 实际静音状态变化时 |
| volume_lock | bool data，true 锁定，false 解锁 | 音量调整权限变化时 |
| mode | uint8 data，允许 0～5；当前共享校验代码未定义各值的产品语义，联调前应统一枚举表 | 模式切换完成或权威模式变化时 |
| dock | uint8 data，0 不在巢，1 在巢，255 未知 | 在巢状态变化时 |

锁定状态只是一项下行数据；当前通信层不会据此拒绝后续 volume 输入。若业务不允许调节，业务服务应作出拒绝决定并返回相应执行结果。

### 3.2 电池和 Wi-Fi

DisplayBatteryState：

| 字段 | 类型 | 语义和约束 |
|---|---|---|
| percent | uint8 | 0～100；255 表示电量未知 |
| charge | uint8 | 0 未充电，1 充电中，2 已充满，3 暂停，4 充电故障 |
| flags | uint16 | 位标志，可按位或组合；bit0 低电量，bit1 严重低电量，bit2 过热，bit3 电池故障；其他位必须为 0 |

在上述任一字段变化时发布。低电量阈值由业务方判断，通信节点不从 percent 自动生成 flags。

DisplayWifiState：

| 字段 | 类型 | 语义和约束 |
|---|---|---|
| state | uint8 | 0 离线，1 连接中，2 已连接，3 故障 |
| signal | uint8 | 信号等级 0～4；255 未知；不是 dBm |

### 3.3 配网状态

DisplayProvisionState，阶段、结果、倒计时变化时发布；倒计时由业务方更新，通信节点不在本地推算配网是否超时。

| 字段 | 类型 | 语义和约束 |
|---|---|---|
| id | uint32 | 本次配网流程标识，由业务方分配 |
| phase | uint8 | 0 空闲，1 广播，2 配对中，3 已配对，4 配网中，5 成功，6 失败，7 超时，8 取消 |
| result | uint8 | 0 待定，1 成功，2 失败，3 超时，4 取消 |
| phase_left_s | uint16 | 当前阶段剩余秒数 |
| window_left_s | uint16 | 整个流程窗口剩余秒数 |
| keep_awake | bool | 业务方给出的保持唤醒要求 |

### 3.4 故障集合

DisplayFaultState 只有 faults 字段，类型为 DisplayFault[<=32]。每次发布当前完整集合，空数组表示清空集合；不是增删单条故障的操作接口。

DisplayFault 每项定义：

| 字段 | 类型 | 语义和约束 |
|---|---|---|
| id | uint16 | 非零故障编号，同一集合内不得重复 |
| severity | uint8 | 1 提示，2 警告，3 严重 |
| acknowledged | bool | 是否已被业务方确认知悉；不等于故障已解除 |
| order | uint32 | 业务方提供的排序值；通信层不排序、不决定优先级 |

故障新增、解除、确认状态或排序变化时，重新发布完整集合。故障编号及其文案由业务方维护目录。

### 3.5 升级状态

DisplayUpgradeState 用来展示升级状态。发布它不会启动 OTA、烧录或重启；实际 OTA 仍由既有升级工具执行。

| 字段 | 类型 | 语义和约束 |
|---|---|---|
| id | uint32 | 升级流程标识 |
| target | uint8 | 1 系统，2 HPM，3 P4，4 资源，5 组合包；当前 HPM 值不进一步区分 6E80 与 5151 |
| phase | uint8 | 0 空闲，1 准备，2 传输，3 校验，4 重启，5 自检，6 完成，7 失败 |
| progress | uint8 | 0～100，255 未知 |
| ui_locked | bool | 业务给出的界面锁定要求 |
| result | uint8 | 0 待定，1 成功，2 失败，3 取消 |

流程标识、阶段、整数进度或结果变化时发布。进入维护模式后业务连接关闭，期间发布的状态会保留最新值，恢复连接后再同步，不能依赖该通道实时显示自身 P4 OTA 的进度。

### 3.6 身份和名称

DisplayIdentityState：bool bound 表示绑定状态，string<=63 name 表示设备名称。线协议额外要求名称不超过 63 个 UTF-8 字节，且不能包含 NUL 字符；中文通常占多个字节，不能只按字符数判断。允许空名称。

绑定状态或设备名称变化时发布；不在本话题传递账号口令、配网密码或其他认证信息。

### 3.7 表情和语音

DisplayExpression：

| 字段 | 类型 | 语义和约束 |
|---|---|---|
| id | uint16 | 表情资源编号；资源目录由业务方和 P4 UI 方共同维护 |
| action | uint8 | 0 保持，1 进入，2 循环，3 退出 |
| priority | uint8 | 0 普通，1 紧急；通信层只转发，不仲裁 |

每次需要触发表情动作时发布。该输入允许同值重复，离线或维护期间拒绝入队，连接恢复后不重放历史动作。

DisplayVoiceState：uint8 stage 为 0 空闲、1 聆听、2 思考、3 说话；uint8[<=64] bins 为频谱幅度数组，每项 0～255，可为空。阶段变化或有新频谱数据时发布。频谱分箱与幅度的物理含义尚未在接口中定义，音频与 UI 双方应约定；通信层只保留最新待发值。

### 3.8 指示灯输出

DisplayIndicator 表达业务方已决定的目标灯效。当前不包含低电量、故障、充电、语音等规则的自动仲裁，也不证明机身灯或巢灯硬件已接通。

| 字段 | 类型 | 语义和约束 |
|---|---|---|
| target | uint8 | 1 机身，2 巢灯；按目标分别缓存最新命令 |
| state_id | uint8 | 灯需求状态编号 1～12；见后表 |
| color | uint8 | 0 无颜色，1 红，2 橙，3 黄，4 绿，5 蓝，6 紫 |
| mode | uint8 | 0 关闭，1 常亮，2 呼吸，3 闪烁 |
| brightness | uint8 | 0～100；mode=0 时必须为 0 |
| period_ms | uint16 | 呼吸或闪烁周期；mode=2 或 3 时必须大于 0 |
| window_ms | uint16 | 周期性显示窗口时长；与 repeat_ms 同为 0 表示不启用窗口 |
| repeat_ms | uint16 | 窗口重复间隔；启用时必须满足 0 < window_ms <= repeat_ms |
| activation_id | uint32 | 本次灯效激活标识，由业务方分配；需要重新触发同一灯效时更新此值 |
| remaining_ms | uint32 | 0 已到期，4294967295 持续，其余为剩余毫秒数 |
| phase_elapsed_ms | uint32 | 业务提供的已运行相位时长，单位毫秒 |

| state_id | 需求场景 | state_id | 需求场景 |
|---|---|---|---|
| 1 | 关闭 | 7 | 严重故障 |
| 2 | 交互 | 8 | 一般故障 |
| 3 | 成功提示 | 9 | 配对配网 |
| 4 | 充电中 | 10 | OTA 升级 |
| 5 | 已充满 | 11 | 回巢 |
| 6 | 低电量 | 12 | 语音 |

state_id 仅标识场景；颜色、模式、亮度、时序必须由业务方一并填写，传输层不会按场景编号自动补齐。灯效切换时发布，新触发应更新 activation_id。发送前及重连补发时，节点扣减自接收命令以来的 remaining_ms 并累计 phase_elapsed_ms；持续值不扣减。所有灯效字段在 P4 当前仍只进入日志回调，具体渲染需后续实现。

## 4 hpm_display 发布的屏端用户请求

业务方订阅下表话题，验证权限并执行请求；执行完后向第 5 节的结果话题回传结果，并向第 3 节相应功能话题发布实际最终状态。收到请求不代表操作已执行。

所有请求的类型均为 robot_interfaces/msg/DisplayRequest。

| 输出话题 | value 的含义 | P4 触发接口 |
|---|---|---|
| /display/volume/request | 音量百分比 0～100 | display_send_volume_request |
| /display/mute/request | 0 取消静音，1 静音 | display_send_mute_request |
| /display/mode/request | 模式编号 0～5 | display_send_mode_request |
| /display/bluetooth/request | 蓝牙请求布尔值 0 或 1；具体启停动作须业务与 UI 统一 | display_send_bluetooth_request |
| /display/fault/acknowledge | 非零故障编号，表示用户请求确认知悉 | display_send_fault_acknowledge |
| /display/provision/request | 固定 0，表示发起配网请求 | display_send_provision_request |

DisplayRequest 字段：

| 字段 | 类型 | 调用方处理方式 |
|---|---|---|
| boot_id | uint32 | 来源 P4 的启动标识，和 event_id 一起识别请求 |
| sequence | uint32 | 传输帧序号，仅作诊断和定位 |
| event_id | uint32 | 非零请求编号，执行结果必须回填原值 |
| value | uint16 | 按上表解释 |

这些话题由屏端主动调用对应接口后才产生数据；启动节点不会自动产生用户请求。传输层会抑制重复帧和最近的重复事件，业务方仍应按 boot_id 与 event_id 避免重复执行有副作用的操作。

## 5 hpm_display 订阅的请求执行结果

以下 6 个话题的方向都是“业务方发布 → hpm_display 订阅 → P4”。它们不是 hpm_display 向业务方发布的结果。

| 执行结果输入话题 | 对应的屏端请求输出话题 |
|---|---|
| /display/volume/result | /display/volume/request |
| /display/mute/result | /display/mute/request |
| /display/mode/result | /display/mode/request |
| /display/bluetooth/result | /display/bluetooth/request |
| /display/fault/result | /display/fault/acknowledge |
| /display/provision/result | /display/provision/request |

消息类型均为 robot_interfaces/msg/DisplayResult，字段定义如下。该类型同时用于第 7 节通信回执，使用时必须区分方向与字段含义。

| 字段 | 类型 | 业务方发布 result 时的填写方式 |
|---|---|---|
| session | uint32 | 推荐填原请求 boot_id；0 仅在唯一匹配时允许，建议始终明确填写 |
| sequence | uint32 | 业务结果输入不使用，填 0；不要用它代替 event_id |
| event_id | uint32 | 填原请求 event_id，必须非零 |
| result | uint16 | 执行结果 0～6；业务结果输入不接受 7 |
| reason | uint16 | 原因码，由业务与 P4 约定；0 表示未细分 |
| revision | uint32 | 业务方提供的结果或状态版本值；无版本机制可填 0 |

result 取值：0 SUCCESS 成功，1 REJECTED 拒绝，2 BUSY 忙，3 INVALID 参数无效，4 UNSUPPORTED 不支持，5 FAILED 失败，6 TIMEOUT 超时。7 RECEIVED 只用于通信回执，不能用作执行成功的替代值。

结果只能关联本次连接内确实收到的请求，节点保存最近 64 条请求关联信息；无匹配、存在歧义、已断线、维护中或队列满时，只记录错误并拒绝发送。连接重建后不会自动重放旧请求的结果，业务方不能依靠该队列保证跨断线事务完成。

建议的音量交互顺序：P4 发布 volume 请求 → 业务方执行实际音量设置 → 发布 volume/result → 发布 volume/set 最终实际音量。失败时同样应返回失败结果，并在需要时同步实际音量。当前 P4 结果回调也只记录日志，UI 提示需后续接入。

## 6 hpm_display 发布的设备状态

消息类型均为 robot_interfaces/msg/DisplayDeviceState，数据来自 P4 上报函数的调用方，不由传输层采集。业务方订阅，用于设备诊断或进一步业务判断。

| 输出话题 | 内容 | P4 上报函数 |
|---|---|---|
| /display/panel/state | 面板状态和错误码 | display_update_panel_state |
| /display/backlight/state | 背光状态、错误码和亮度 | display_update_backlight_state |
| /display/touch/state | 触摸状态和错误码 | display_update_touch_state |
| /display/indicator/state | 目标灯状态、错误码和亮度 | display_update_indicator_state |

| 字段 | 类型 | 语义和约束 |
|---|---|---|
| boot_id | uint32 | 数据来源 P4 的启动标识 |
| revision | uint32 | 该次设备状态的版本标识，由 P4 传输侧生成 |
| target | uint8 | 指示灯为 1 机身或 2 巢灯；其他设备话题为 0 |
| status | uint8 | 0 未知，1 就绪，2 故障，3 禁用 |
| error | uint16 | 设备错误码，由实际驱动与业务约定；0 表示无具体错误码 |
| brightness | uint8 | 背光和指示灯为 0～100；面板、触摸为 0，不具有亮度含义 |

业务代码应在初始化完成、状态变化或错误变化时调用上报接口。就绪或亮度值是固件报告的状态，不能直接证明屏幕有光或 LED 有电流。ROS2 的历史缓存也不代表设备仍在线，调用者应同时检查连接状态及 boot_id。

## 7 hpm_display 发布的通信回执和连接状态

### 7.1 每个功能的通信回执

以下 15 个话题的类型均为 robot_interfaces/msg/DisplayResult，方向为“hpm_display 发布 → 业务方订阅”。它们报告一次已发送命令的通信结果。

| 功能 | 通信回执输出话题 | 功能 | 通信回执输出话题 |
|---|---|---|---|
| battery | /display/battery/feedback | bluetooth | /display/bluetooth/feedback |
| wifi | /display/wifi/feedback | volume | /display/volume/feedback |
| mute | /display/mute/feedback | volume_lock | /display/volume_lock/feedback |
| mode | /display/mode/feedback | dock | /display/dock/feedback |
| provision | /display/provision/feedback | fault | /display/fault/feedback |
| upgrade | /display/upgrade/feedback | identity | /display/identity/feedback |
| expression | /display/expression/feedback | voice | /display/voice/feedback |
| indicator | /display/indicator/feedback |  |  |

回执中的 session、sequence 标识下行传输，revision 是节点为该发送命令记录的版本。普通下行状态的 event_id=0；当回执对应第 5 节的业务结果传输时，event_id 为原用户请求编号。sequence 和 revision 由节点管理，调用者不要预先构造或假定它们。

P4 当前空业务回调正常返回时，result=7 RECEIVED、reason=0，只证明数据已校验并分发，不能据此声称屏幕、音量或灯效实际执行成功。result=6 可能由节点等待 P4 应答超时产生，不等于业务已确认失败；副作用是否发生应结合实际状态判断。

同值状态被去重、待发旧值被新值覆盖、本地字段校验失败、离线动作被拒绝等情况，不保证产生逐条 feedback；输入校验失败和队列拒绝当前只记录节点日志。该话题不是每条 ROS 输入都必定对应一条回执的事务 API。当前 DisplayResult 也没有 indicator 的 target 字段，不可只凭一条灯回执判断某个物理灯的完成状态。

### 7.2 连接状态

/display/connection/state，类型 std_msgs/msg/String，data 为以下字符串之一：

| data | 含义 |
|---|---|
| connecting | 正在连接 HPM TCP 网关 |
| gateway_ready | HPM 已返回 ESP_READY，尚待 P4 HELLO 应答 |
| online | P4 HELLO 已应答，业务链路可发送 |
| offline | 连接失败、断开或 P4 超时，等待重连 |
| maintenance | 已进入维护，释放连接并暂停重连 |
| stopped | 节点工作线程停止；节点退出后不能依靠历史消息持续可用 |

只有 online 表示 P4 的应用协议已回应；ping 成功、TCP 连接成功或 gateway_ready 都不足以证明 P4 业务链路可用。状态只在变化时发布，空闲 HELLO 保活不会形成每秒日志。

## 8 QoS 发送保障和维护服务

### 8.1 QoS 与调用者要求

所有话题均为 KEEP_LAST，具体设置如下。

| 话题类别 | Reliability | Durability | 深度 |
|---|---|---|---|
| 15 个功能输入订阅 | RELIABLE | VOLATILE | 10 |
| 6 个执行结果输入订阅 | RELIABLE | VOLATILE | 16 |
| 6 个用户请求输出 | RELIABLE | VOLATILE | 16 |
| 15 个通信回执输出 | RELIABLE | VOLATILE | 16 |
| 4 个设备状态输出 | RELIABLE | TRANSIENT_LOCAL | 5 |
| 连接状态输出 | RELIABLE | TRANSIENT_LOCAL | 1 |

向节点发布输入时使用 RELIABLE；BEST_EFFORT 发布者与节点的可靠订阅不兼容。输入订阅不请求历史数据，因此业务发布者即使设置 TRANSIENT_LOCAL，也应在节点重新上线后重新发送当前状态，不能只发布一次后长期假定对端已持有。

调用者若希望订阅时获得连接或设备状态缓存，应使用 TRANSIENT_LOCAL。缓存只在发布者存活且保存历史时有效；多个灯目标共用一个设备状态话题，深度 5 不是按 target 独立的永久快照。用户请求和回执为 VOLATILE，迟到订阅者不能补收旧请求。

### 8.2 重试和缓存边界

状态缓存最多 32 项，操作事件队列最多 16 项；灯输出按目标分别占缓存项。单条已发送消息的应答等待默认 2 秒，最多重试 3 次，即首次加重试最多 4 次。超时后关闭连接并重连；重连间隔从 reconnect_ms 起退避，最高 10 秒。空闲默认每 10 秒 HELLO 保活。

ROS2 发布函数不会等待网络往返。通信层对重复帧应答但不重复调用业务回调；它不提供跨节点重启的持久化事务、离线事件存储或严格实时保证。业务操作结果不明时应查询或同步实际状态，不能盲目重放历史操作。

### 8.3 OTA 维护服务

服务 /display/upgrade/maintenance，类型 std_srvs/srv/SetBool。

| 请求或响应字段 | 含义 |
|---|---|
| request.data=true | 关闭显示连接，清除待发事件，禁止重连，供 OTA 工具独占 TCP5002 |
| request.data=false | 允许重连；成功连接后补发最新状态 |
| response.success | 进入维护为 true 时连接已释放，为 false 时维护开关仍可能已开启但释放尚未完成；退出维护为 true 仅表示允许重连 |
| response.message | 具体状态说明 |

进入维护返回 success=true 后再启动现有 OTA 工具。若为 false，应检查 message，并再次确认连接已释放，不能直接假定通道可用。退出维护 success=true 只表示允许重连，仍需等待 connection/state=online。维护期间保留状态缓存，拒绝新的表情操作和请求执行结果。

```bash
ros2 service call /display/upgrade/maintenance \
  std_srvs/srv/SetBool '{data: true}'
# 维护释放成功后，运行现有 OTA 工具。
# OTA 工具退出后恢复业务连接。
ros2 service call /display/upgrade/maintenance \
  std_srvs/srv/SetBool '{data: false}'
```

## 9 最小联调命令

在保持显示节点运行的前提下，另开终端并加载环境。下列命令覆盖典型字段形式；由于状态去重，重复测试时应改变数值。

### 9.1 检查节点和链路

```bash
source /opt/ros/humble/setup.bash
source ~/zyc_ws/install/setup.bash
ros2 node info /hpm_display
ros2 topic echo /display/connection/state \
  --qos-durability transient_local
```

应先观察到 online。连接不上时检查 IP、TCP5002 是否被其他显示节点或 OTA 工具占用，以及 P4 是否运行支持该协议的固件。

### 9.2 下发功能数据

```bash
# 音量 30 改为 60，便于观察状态变化。
ros2 topic pub --once /display/volume/set \
  std_msgs/msg/UInt8 '{data: 60}'
ros2 topic pub --once /display/mute/set \
  std_msgs/msg/Bool '{data: true}'
ros2 topic pub --once /display/battery/state \
  robot_interfaces/msg/DisplayBatteryState \
  '{percent: 75, charge: 1, flags: 0}'
ros2 topic pub --once /display/wifi/state \
  robot_interfaces/msg/DisplayWifiState '{state: 2, signal: 3}'
ros2 topic pub --once /display/fault/state \
  robot_interfaces/msg/DisplayFaultState \
  '{faults: [{id: 4, severity: 3, acknowledged: false, order: 1}]}'
ros2 topic pub --once /display/fault/state \
  robot_interfaces/msg/DisplayFaultState '{faults: []}'
ros2 topic pub --once /display/expression/set \
  robot_interfaces/msg/DisplayExpression '{id: 1, action: 1, priority: 0}'
ros2 topic pub --once /display/indicator/set \
  robot_interfaces/msg/DisplayIndicator \
  '{target: 1, state_id: 2, color: 5, mode: 2, brightness: 50,
    period_ms: 2000, window_ms: 0, repeat_ms: 0, activation_id: 1,
    remaining_ms: 3000, phase_elapsed_ms: 0}'
```

另开终端订阅回执，再执行音量发布：

```bash
ros2 topic echo /display/volume/feedback
```

当前正常收到的回执为 result: 7，P4 USB 应有 display-cb 对应功能日志。只有 ROS2 的 publishing 字样说明发布者发出了消息，不能代替 P4 回执。

### 9.3 接收请求并回传结果

```bash
# 等待真实屏端业务调用 volume_request 上报接口。
ros2 topic echo /display/volume/request
```

记录真实消息中的 boot_id、event_id、value。业务代码实际执行完成后，向 /display/volume/result 发布 DisplayResult：session=原 boot_id，event_id=原 event_id，sequence=0，result=实际执行结果，reason=具体原因码，revision=业务版本或 0。随后发布 /display/volume/set 同步真实音量。不要对并未执行的操作回传 SUCCESS。

查看设备状态历史：

```bash
ros2 topic echo /display/panel/state \
  --qos-durability transient_local
ros2 interface show robot_interfaces/msg/DisplayResult
```

没有设备状态输出时，应先确认 P4 业务代码是否实际调用上报函数。不能通过向节点的输出话题手工发布消息来证明真实上行链路正常。

## 10 对接分工和交付核对

业务方需要完成：产生并重发权威状态；订阅屏端请求并校验权限；执行实际动作；回传执行结果和最终状态；计算灯效优先级及时间参数；统一资源、模式、蓝牙、错误码等目录。

P4 业务方需要完成：在已分发的各功能回调中接入 UI、灯效及硬件动作；把真实触摸或交互事件接到上行请求函数；把真实驱动状态接到设备上报函数；处理业务结果反馈。当前日志回调和编码函数已经提供接入点。

联调验收应分别确认：ROS2 订阅匹配、connection/state=online、功能数据得到通信回执、P4 对应回调被调用、真实设备行为完成、设备状态或请求结果回到业务方。前四项通过不能代替后两项的实际业务验收。

当前待统一的业务目录包括模式 0～5、蓝牙状态 0～4、蓝牙布尔请求对应的具体操作、故障编号和文案、表情资源编号、reason 与 error 错误码、语音频谱分箱定义。DisplayUpgradeState 的 HPM 目标值目前不区分两种 HPM 芯片，若界面需要分别展示，应先扩展契约而不是私自复用其他字段。

本文按 2026 年 11 月 17 日本地实现核对，核对依据为 hpm_display 的话题注册、编码、校验、回执、连接和维护实现，以及已构建 robot_interfaces 的 ros2 interface show 输出。后续修改消息定义、枚举语义或话题方向时，应更新文档版本并同步双方消息包。
