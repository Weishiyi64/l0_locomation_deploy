# hpm_display ROS2 对外接口说明

版本 V1.1　｜　日期 2026 年 11 月 18 日　｜　新增背屏和指示灯需求编号对应

## 1 适用范围和实现状态

本文供 RK3588 上的机器人业务、音频、网络、升级及 App 接入开发者使用，定义背屏通信节点 hpm_display 当前订阅和发布的 ROS2 话题、消息字段、发送时机、反馈方式和联调命令。

调用者向本文列出的输入话题发布业务数据，hpm_display 将数据传给 ESP32-P4；调用者订阅本文列出的输出话题，接收屏端用户请求、设备状态和通信回执。调用者不需要自行拼接 TCP 或 UART 协议帧。

链路为：3588 业务节点 → ROS2 → hpm_display → HPM TCP 5002 → UART → ESP32-P4，反馈沿相反方向返回。HPM 只转发字节，不计算业务状态、不决定灯效、不生成业务结果。

当前已实现话题转换、双向传输、字段校验、重连、去重、回执和 P4 回调分发。P4 下行业务回调目前只通过 USB 日志打印名称和参数；实际屏幕、音量、灯光、配网等业务动作仍待接入。设备上报函数已实现编码和发送，但由实际业务代码提供数据并主动调用，不会自动采集或伪造设备状态。

本接口共包含 21 个订阅话题、26 个发布话题、1 个维护服务。其中订阅为 15 个功能输入加 6 个执行结果输入；发布为 6 个用户请求、4 个设备状态、15 个通信回执及 1 个连接状态。以下所有“订阅”“发布”均以 hpm_display 节点为参照。

表中需求编号来自《l0 - V2.1.0 - 背后椭圆屏》（屏，§4.1～4.5）和《l0-2.4.1-指示灯 【新】-1》（灯，§2.1、§3、§7.2）。编号标明接口向哪个需求提供数据、请求或诊断。产品行为的完成情况需按对应 PRD 验收。

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
| 电池图标、电量数字和充电状态。屏 PAGE1-01、MSG-05、MSG-06。充电及低电灯效的关联数据见灯 S-04～S-07。 | /display/battery/state | DisplayBatteryState |
| 蓝牙图标四态及配网成功后的关闭态。屏 PAGE1-02～PAGE1-04。 | /display/bluetooth/state | std_msgs/msg/UInt8 |
| Wi-Fi 离线、连接中、已连接和信号格。屏 PAGE1-06～PAGE1-09。 | /display/wifi/state | DisplayWifiState |
| 同步音量滑块和喇叭声波。安静模式切换后同步实际音量。屏 PAGE2-02、PAGE2-03、PAGE3-03、PAGE3-07。 | /display/volume/set | std_msgs/msg/UInt8 |
| 同步静音图标及滑块归零或恢复。屏 PAGE2-01、PAGE2-03。 | /display/mute/set | std_msgs/msg/Bool |
| 让页面在安静模式下置灰并禁用音量操作。屏 PAGE2-04、PAGE3-03。 | /display/volume_lock/set | std_msgs/msg/Bool |
| 同步五种模式的选中状态和互斥后的有效模式。屏 PAGE3-01～PAGE3-07。 | /display/mode/set | std_msgs/msg/UInt8 |
| 在待机、换装页面显示「在巢内」。屏 PAGE3-04、PAGE3-06。可提供充电场景的在巢信息，关联灯 S-04。 | /display/dock/state | std_msgs/msg/UInt8 |
| 展示配对和配网阶段、失败或超时，并给出配对期间暂停熄屏所需状态。屏 PAGE1-03、PAGE1-05、PAGE1-07～PAGE1-09、SYS-01。配对灯效关联灯 S-09。 | /display/provision/state | DisplayProvisionState |
| 同步故障集合和知悉状态，供页面显示、排序和清除提示。屏 MSG-01、MSG-03～MSG-09。故障灯效的数据关联灯 S-08。 | /display/fault/state | DisplayFaultState |
| 显示升级锁定提示，结束后恢复界面及消息展示。屏 MSG-02、MSG-08。升级提示灯关联灯 S-10。 | /display/upgrade/state | DisplayUpgradeState |
| 同步设备名称，屏 PAGE1-03 第 5 条。绑定标志是接口补充字段，屏 PRD 未分配独立编号。 | /display/identity/state | DisplayIdentityState |
| 预留表情播放接口。屏 §2.5 将情绪表达动画划归正面脸部，没有对应的背屏需求编号。 | /display/expression/set | DisplayExpression |
| 传递语音阶段和频谱。屏 §2.5 将语音交互动画划归正面。语音阶段关联灯 S-12，频谱没有对应编号。 | /display/voice/state | DisplayVoiceState |
| 下发本体灯和充电桩灯的目标灯效。灯 S-01～S-12 逐项见 §3.8。背屏自身 LED 光圈按屏 MSG-01～MSG-06、MSG-10 另行接入。 | /display/indicator/set | DisplayIndicator |

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

下表逐项对应灯 PRD 的状态编号。每个目标使用相同的 /display/indicator/set 话题，并按 target 分别发送。表中灯效来自灯 PRD 的基本场景。

| 灯需求编号 | state_id | 功能和预期用途 | 本体灯 | 充电桩灯 |
|---|---|---|---|---|
| S-01 | 1 | 常态熄灭，正常运行、待机或休眠 | 熄灭 | 熄灭 |
| S-02 | 2 | 确认语音唤醒、触摸或 App 遥控交互 | 蓝色呼吸 3 秒 | 熄灭 |
| S-03 | 3 | 任务确认成功后触发 | 绿色常亮 2 秒，亮度 50% | 熄灭 |
| S-04 | 4 | 已开始充电，持续到充满或离桩 | 绿色呼吸 | 绿色常亮 |
| S-05 | 5 | 充电完成提示 | 绿色常亮 5 秒 | 绿色常亮 5 秒 |
| S-06 | 6 | 有效电量 ≤20% 持续 10 秒后进入低电提醒 | 橙色 1 Hz 慢闪，每 30 秒提示 2 秒，亮度 60% | 熄灭 |
| S-07 | 7 | 严重低电，持续到充电或自动关机 | 红色快闪 | 熄灭 |
| S-08 | 8 | 异常或故障提示，持续到解除或人工复位 | 红色常亮 | 熄灭 |
| S-09 | 9 | 配网或配对提示，持续到成功或超时 | 蓝色快闪 | 熄灭 |
| S-10 | 10 | 系统下载或安装更新提示 | 紫色呼吸 | 熄灭 |
| S-11 | 11 | 寻桩归位，持续到归位成功或失败 | 绿色慢闪 | 绿色呼吸 |
| S-12 | 12 | 语音对话提示，结束后延续 3 秒 | 黄色呼吸 | 基本场景熄灭 |

项目对接统一采用 S-03 任务确认成功后起亮，依据灯 PRD §3.3 和 AC-03。S-06 按灯 PRD §6 将恰好 20% 纳入触发范围，按 §4.2 做 10 秒防抖。该边界用于实体指示灯 S-06。背屏 PAGE1-01 的电池图标按背屏 PRD 规则处理。

3588 业务发布方判断任务结果和电量，加上持续时间、灯效优先级及夜间亮度后再发布。未知电量 255 不参与 S-06 判定，严重低电 S-07 按灯 PRD §4.1 优先。S-12 与充电并发时，桩灯按灯 PRD §6 保持绿色常亮。

下列参数供业务发布方发送已选中的灯效，对应 /display/indicator/set。每次新的激活使用新的 activation_id，phase_elapsed_ms 初次发送为 0。

| 灯需求编号 | 本体灯字段填写 |
|---|---|
| S-03 | target=1，state_id=3，color=4，mode=1，brightness=50，remaining_ms=2000。周期和窗口字段均为 0。 |
| S-06 | target=1，state_id=6，color=2，mode=3，brightness=60，period_ms=1000，window_ms=2000，repeat_ms=30000，remaining_ms=4294967295。夜间亮度按灯 PRD §4.3 由发布方设置，场景结束时下发新的目标灯效。 |

背屏 MSG-04 的「我知道了」用于消除屏上提示，灯 S-08 的结束条件仍按灯 PRD 判定。背屏 MSG-02 规定小屏自身升级时的黑屏和灭灯，S-10 描述系统升级的紫色呼吸提示，需按升级目标和阶段分别对接。

state_id 仅标识场景；颜色、模式、亮度、时序必须由业务方一并填写，传输层不会按场景编号自动补齐。灯效切换时发布，新触发应更新 activation_id。发送前及重连补发时，节点扣减自接收命令以来的 remaining_ms 并累计 phase_elapsed_ms；持续值不扣减。所有灯效字段在 P4 当前仍只进入日志回调，具体渲染需后续实现。

## 4 hpm_display 发布的屏端用户请求

业务方订阅下表话题，验证权限并执行请求；执行完后向第 5 节的结果话题回传结果，并向第 3 节相应功能话题发布实际最终状态。收到请求不代表操作已执行。

所有请求的类型均为 robot_interfaces/msg/DisplayRequest。

| 输出话题 | 功能和预期用途及 value 含义 | P4 触发接口 |
|---|---|---|
| /display/volume/request | 拖动滑块时请求调整音量，值为 0～100%。屏 PAGE2-02、PAGE2-04。 | display_send_volume_request |
| /display/mute/request | 点击喇叭后请求切换静音，0 取消静音，1 静音。屏 PAGE2-01、PAGE2-03、PAGE2-04。 | display_send_mute_request |
| /display/mode/request | 用户确认后请求切换模式，模式编号 0～5。屏 PAGE3-01～PAGE3-07、SYS-01。 | display_send_mode_request |
| /display/bluetooth/request | 请求开启蓝牙，或经确认请求关闭。取值 0 或 1，启停对应值由双方统一。屏 PAGE1-03、PAGE1-04。 | display_send_bluetooth_request |
| /display/fault/acknowledge | 用户点击「我知道了」，值为非零故障编号。屏 MSG-04。 | display_send_fault_acknowledge |
| /display/provision/request | 发起配网请求，值固定为 0。关联屏 PAGE1-09 的重配网引导，该条实际重配网步骤由 App 发起。 | display_send_provision_request |

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

| 执行结果输入话题 | 对应的屏端请求输出话题 | 功能和预期用途 |
|---|---|---|
| /display/volume/result | /display/volume/request | 返回音量处理结果，供页面更新或提示限制。屏 PAGE2-02、PAGE2-04。 |
| /display/mute/result | /display/mute/request | 返回静音处理结果，供页面同步显示。屏 PAGE2-01、PAGE2-03、PAGE2-04。 |
| /display/mode/result | /display/mode/request | 返回模式切换结果，供页面显示切换提示。屏 PAGE3-01～PAGE3-07。 |
| /display/bluetooth/result | /display/bluetooth/request | 返回蓝牙启停结果，支持开启失败提示。屏 PAGE1-03～PAGE1-05。 |
| /display/fault/result | /display/fault/acknowledge | 返回知悉处理结果，供背屏结束相应提示。屏 MSG-04。 |
| /display/provision/result | /display/provision/request | 返回请求受理结果，配网最终状态由 provision/state 同步。关联屏 PAGE1-09。 |

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

| 输出话题 | 功能和预期用途 | P4 上报函数 |
|---|---|---|
| /display/panel/state | 上报面板状态及错误。工程诊断，关联屏 §2.1 硬件规格。 | display_update_panel_state |
| /display/backlight/state | 上报背光和亮度，辅助核对屏 SYS-01 及待定的 SYS-02。 | display_update_backlight_state |
| /display/touch/state | 上报触摸驱动状态及错误。工程诊断，辅助核对屏 SYS-01、PAGE2-02、PAGE3-01。 | display_update_touch_state |
| /display/indicator/state | 上报本体灯或桩灯的状态、错误和亮度。辅助灯 S-01～S-12 联调，故障数据可供 XBN-005 使用。 | display_update_indicator_state |

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

按话题中的功能名查 §3 对应输入行的需求编号。各条 feedback 用于对应需求的链路联调，完成产品验收还应检查实际界面或灯效。连接状态和 OTA 维护服务属于工程接口，两份 PRD 均无独立需求编号。

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
