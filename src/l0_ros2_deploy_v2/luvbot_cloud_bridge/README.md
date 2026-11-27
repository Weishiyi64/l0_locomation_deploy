# 云端通信与机器人状态

建图主流程、假大脑、两张地图 ID、服务和 MQTT 产物约定见
[建图主流程联调](../docs/mapping_workflow.md)。假大脑单独启动，感知和建图模块由各自实现方提供。
手动模拟收到开始指令可调用 `/luvbot/cloud/map_start`（MapControl，action=1、index=1/2、可选字符串 map_id）；
该入口仍走同一套真实业务与上传链路，详见联调文档的“手动模拟收到 map.start”。

`cloud_bridge_node` 使用 MQTT 3.1.1 / 双向 TLS / Protobuf，负责上线、离线、
1 Hz 心跳、1 Hz 属性上报、每 10 秒健康上报，以及下行指令与业务回执。当前属性帧按服务器联调要求
使用固定的基础属性；本包不启动 HPM 或旧电池驱动。

## 启动入口与总开关

单独启动通信节点：

```bash
ros2 launch luvbot_cloud_bridge cloud_bridge.launch.py
```

跟随机器人主程序启动，在原有实机命令中添加：

```bash
ros2 launch legged_controllers sim2real.launch.py cloud_enabled:=true
```

`cloud_enabled` 默认 `false`，控制该实机入口启动的全部 MQTT 上下行通信，
包括心跳和电量；不会停止通过其他入口独立启动的通信进程。修改身份或服务器时，
主入口使用 `cloud_device_id`、`cloud_host`、`cloud_cert_dir`，独立入口使用
`device_id`、`host`、`cert_dir`，独立入口另支持 `ca_file`。

BLE 配网可独立使用：

```bash
ros2 launch luvbot_provisioning provisioning.launch.py
```

需要常驻 BLE 配网并在网络就绪后自动启动云端通信时使用：

```bash
ros2 launch luvbot_cloud_bridge wifi_provisioning.launch.py
```

集成入口已有网络时直接启动 cloud bridge；无网时通过 `network_ready_gate` 等待
本地网络恢复或 BLE 成功，等待期间不进行 MQTT/DNS 连接重试。每次 launch 只启动
一次通信节点。配网流程见 [配网包说明](../luvbot_provisioning/README.md)。
同一设备 ID 只能选择一个通信入口运行，避免 MQTT 客户端互相挤下线。

## 属性帧与服务器报文

| 方向 / 用途 | 话题 | 内容 |
| --- | --- | --- |
| MQTT 属性上报 | `luvbot/<device_id>/up/property` | `Envelope.property_report`，周期帧为基础属性及表情；性格变化时另发一帧 |
| MQTT 状态 / 心跳 / 遗嘱 | `luvbot/<device_id>/up/status` | `Envelope.status_report`；异常断线为空载荷遗嘱 |
| MQTT 指令订阅 | `luvbot/<device_id>/down/command` | `Envelope.command` |
| MQTT 指令回执 | `luvbot/<device_id>/up/reply` | `Envelope.command_reply` |
| MQTT 健康上报 | `luvbot/<device_id>/up/event` | `Envelope.event_report`，事件名 robot.health |

按服务器联调要求，每一帧 `up/property` 均固定填写以下六个字段，包括未收到表情时：

```json
{"fwVersion":"1.0.3","battery":86,"rssi":-58,"uptime":3600,"temperature":365,"netType":"wifi"}
```

`temperature` 单位为 0.1℃，365 对应 36.5℃。这些是临时联调值，battery 不代表
实际电量，uptime 不随时间增长。本节点当前不订阅 `/battery/state`；联调结束后需
按真实采集来源恢复这些属性。表情来自下述 ROS 输入，健康事件中的系统资源仍为实际采集。

固定基础属性只属于 `up/property`，不加入 `up/reply`、`up/event`、`up/status`。
现有 Protobuf 的四种上行消息体处于同一个 oneof，不能在同一帧中混装。

MQTT 成功连接或重连后，在下一次 100 ms 调度时立即发送上线状态和属性，
之后状态心跳和属性约每秒一条，健康每 10 秒一条；连接或重连时健康先立即发送一次。
均为 QoS 1、不 retain。报文为 Protobuf
二进制，不是 JSON 文本或 hex 字符串；无需修改现有 proto。`msg_id` 为
`cloud-<进程UUID>-<序号>`，`ts` 为 Unix 毫秒时间戳，`seq` 在进程内自增。

最多允许 20 条 QoS 1 消息等待 ACK；断网期间不生成新的遥测队列，积压时跳过发送尝试。
libmosquitto 以 1～30 秒退避重连，可能重发少量未确认旧消息；服务器可通过 `msg_id` 去重。
ACK 只表示 broker 收到，不表示业务已解析。正常退出尽力发送
`online=false, reason=normal_shutdown`；异常掉线使用空载荷遗嘱。
`/luvbot/cloud_connected`（`std_msgs/msg/Bool`，reliable / transient-local）表示 MQTT 连接结果。

## 身份与证书

默认服务器为 `mqtt.luvbotics.com:8883`，测试身份为 `RUMI-TEST-007`。
`device_id` 同时用于 MQTT Client ID、Envelope.device_id、话题中段和证书文件名：

```text
luvbot_cloud_bridge/certs/RUMI-TEST-007.crt
luvbot_cloud_bridge/certs/RUMI-TEST-007.key
```

证书被 Git 忽略，同步代码时需另行同步凭据。构建会以仅安装用户可读写的权限将它们
安装到本包 `share/luvbot_cloud_bridge/certs/`，节点通过 ament 索引定位。
使用机器人运行用户构建和启动，不要 sudo 构建。默认服务器 CA 为
`/etc/ssl/certs/ca-certificates.crt`，保留 CA 与服务器域名校验。

BLE 的 `device_sn` 是独立参数，默认同为 `RUMI-TEST-007`，不自动跟随 MQTT 的
`device_id`。手机主动读取 FFF1 得到 `{"status":"RUMI-TEST-007"}`；其中 status
是 SN，不是配网状态。FFF1 写入和 FFF2 纯 SN 读取保持原样。

## 下行执行与回执

`down/command` 仍接收 Protobuf 二进制 Envelope，整帧上限 64 KiB，校验当前
SN、非空 `msg_id` 和 Command 类型。网络线程只解析并放入最多 32 条的队列，
ROS 执行线程每 100 ms 最多处理 8 条；队列满时拒绝执行并记录日志，服务器可按
未收到业务回执处理。不要把 MQTT PUBACK 当作业务执行成功。

回执发布到 `luvbot/<SN>/up/reply`，使用原协议 `Envelope.command_reply`；其中
`msg_id` 关联实际下行 Envelope 的 `msg_id`，`cmd` 对应下行指令。
回执队列最多 128 条，优先于新遥测发送；进程退出后不保留。

### 人脸数据：member.face.train（清单 28）

必填字符串参数共 5 个：`memberId`、`faceFeature`、`faceDim="512"`、
`faceEmotion`、`faceEmotionDim="5"`。`memberId` 使用服务器算法生成的唯一 ID，
始终按原字符串保存、比较，不生成本地 ID，也不转为数字，避免大整数精度丢失。

两个特征字符串固定使用 `f32le.base64`（小端 float32 经 Base64 编码），编码由通信约定确定，
无需另传编码字段。拒绝重复参数键，严格检查 Base64 字符和 padding；
人脸特征解码后为 2048 字节、512 个有限浮点数且非全零；情绪特征为 20 字节、5 个有限浮点数。
仅处理上述 5 个字段，其他下发字段全部忽略，不再解析或校验额外的模型、编码等字段。

校验通过后，仅将上述 5 个字段作为字符串保存到固定文件
`/userdata/forlinx/config/face_db/users.json`，保留 Base64 特征，不保存人脸图片。
文件格式如下（示例特征已缩写，不能直接用于加载）：

```json
{
  "users": [
    {
      "memberId": "2101198065222594562",
      "faceFeature": "<完整 Base64 特征>",
      "faceDim": "512",
      "faceEmotion": "<完整 Base64 情绪特征>",
      "faceEmotionDim": "5"
    }
  ]
}
```

`faceModel`、`fileId`、编码、情绪标签等其他下发字段不再要求，也不写入文件。
每次更新先读取本地底库：按 `memberId` 新增或替换整条成员记录，保留其他成员；
所有成员统一保存为上述 5 字段格式。旧记录含额外字段但 5 个必填字段完整时，
下次成功更新会去掉额外字段；旧记录缺少必填字段时拒绝更新，保留原文件，不补造特征。
重启后下次更新仍从磁盘合并，不清空底库。
已有文件不是上述格式、成员数据无效或 ID 重复时拒绝更新，保留原文件，不静默重建。
缺失目录自动创建，临时文件写入、同步成功后原子替换；写入失败不覆盖旧文件。
桥接是该文件的唯一写入方，业务端读取后自行暂存，不能再改写此文件。
文件权限为 `0600`，业务端须以同一运行用户读取；桥接运行用户须有目录写权限。

保存后异步调用 `/algorithm/face_perception/update_users`，请求只有 `path`。
双方须统一使用 `luvbot_control_interfaces/srv/UpdateUsers`，字段为
`string path --- bool success / string message`；旧 `robot_interfaces` 类型不能直接互调。
业务端须支持上述 JSON 格式和 `f32le.base64` 编码。

只有业务返回 `success=true, message="0"`，桥接才发送成功回执，不带 `data`：

```json
{"msgId":"<原下行指令ID>","cmd":"member.face.train","code":0,"message":"0"}
```

这仅表示**业务已接收文件并安排帧间加载**，不代表最终加载或识别成功，最终加载结果见业务日志。
业务返回 `success=false` 且 `message` 为 `"1001"`（空路径）、`"1002"`（文件读取失败）、
`"1003"`（业务暂存失败）时，回执 `code` 为对应数字，`message` 保留数字字符串。
响应字段相互矛盾或返回其他码时按本地错误处理。
桥接校验失败、保存失败、服务不可用、调用异常、忙碌、超时均回 `code=1006` 和阶段说明；
不打印特征或可能包含特征的解析异常。服务不可用时文件已保存，但不报告业务接收成功，
不会自动重试，服务器可重发该指令。

同一时刻只允许一个人脸服务调用，响应前不改写底库。10 秒超时后报告结果未确认，
保持忙碌并保留文件，直到迟到响应返回或桥接重启；迟到响应会补发最终回执。
最近 128 个已完成改名/人脸/声纹结果共用进程内缓存，相同 `msg_id` 和 Command 重发只回缓存，
不同内容复用 ID 则拒绝。服务不可用、忙碌、校验或保存失败不缓存，可修正后重试；
业务失败响应也会缓存，要重新执行须使用新 `msg_id`。进程重启后缓存丢失，成员合并仍按 ID 幂等。
不验证本地模型同源、不发送 `profile_train_done`。

#### 机器人端验证

同步部署仓库及匹配的 `luvbot_ros_interfaces`，从工作区构建（本地 macOS 不运行）：

```bash
cd ~/zy_workspace/l0_deploy_v2_ws
colcon build --packages-up-to luvbot_cloud_bridge
source install/setup.bash
ros2 interface show luvbot_control_interfaces/srv/UpdateUsers
ros2 service type /algorithm/face_perception/update_users
```

下发一条真实有效的 `member.face.train`，检查 `[FACE] saved`、`service_pending`、
`service_response` 日志，确认文件字段与下发一致，回执关联原指令 ID。
用第二个成员及同一成员的新特征分别更新，确认新增保留旧成员、更新不产生重复 ID；
重启桥接后再更新，确认底库成员保留。另检查无效特征、目录无写权限、损坏的旧文件、
业务不可用、业务 `1001/1002/1003` 失败响应和 10 秒超时均不会回成功；
等待响应期间的新指令不得覆盖文件，同一指令重发不得重复调用服务，迟到响应应补发最终结果。
直接调用以下服务仅验证业务读取链路，需要先有有效文件：

```bash
ros2 service call /algorithm/face_perception/update_users \
  luvbot_control_interfaces/srv/UpdateUsers \
  "{path: '/userdata/forlinx/config/face_db/users.json'}"
```

### 声纹数据：member.voice.train（清单序号 30）

按本次约定使用清单演示帧中的 5 个字符串字段：`memberId`、`taskId`、
`needTrain`、`voiceFeature`、`voiceDim`。`memberId` 按原字符串保存；
`voiceDim` 固定 `"192"`，`voiceFeature` 固定为 `f32le.base64`，解码后须为
768 字节、192 个有限浮点数且非全零。拒绝重复键、非法 Base64、错误维度、NaN/Inf 和全零向量。
`needTrain` 只接受 `"0"` / `"1"`；`"0"` 为纯声纹录入，`taskId` 可为空或省略（保存为空串），
`"1"` 必须携带非空 `taskId`，由业务端处理音色任务。
其他字段不解析、不保存，不下载 WAV，不在桥接中运行模型或合成音色。

下行仍使用原 MQTT topic `luvbot/{SN}/down/command` 和 `luvbot.v1.Envelope` Protobuf，
无需新增 Protobuf 类型。下列 JSON 仅用于展示字段（特征已缩写）：

```json
{
  "msgId": "cmd-voice-001",
  "deviceId": "RUMI-TEST-007",
  "command": {
    "cmd": "member.voice.train",
    "params": [
      {"key": "memberId", "value": "2101198065222594562"},
      {"key": "taskId", "value": ""},
      {"key": "needTrain", "value": "0"},
      {"key": "voiceFeature", "value": "<完整 Base64 声纹特征>"},
      {"key": "voiceDim", "value": "192"}
    ]
  }
}
```

新增固定保存地址 `/userdata/forlinx/config/voice_db/users.json`，格式为：

```json
{
  "users": [
    {
      "memberId": "2101198065222594562",
      "taskId": "",
      "needTrain": "0",
      "voiceFeature": "<完整 Base64 声纹特征>",
      "voiceDim": "192"
    }
  ]
}
```

同人脸一样按 `memberId` 合并、保留其他成员；旧文件校验、目录创建、`0600` 权限、
临时文件同步和原子替换由两模块共用，读写失败保留旧库。额外字段在下次成功保存时移除。
人脸库和声纹库完全独立，每类各允许一个未完成的服务请求，互不阻塞。

保存后新增调用 `/algorithm/voice_perception/update_users`，类型复用
`luvbot_control_interfaces/srv/UpdateUsers`：

```text
string path
---
bool success
string message
```

声纹业务端须实现上述服务，读取声纹文件中的 5 字段；请求为
`path=/userdata/forlinx/config/voice_db/users.json`。受理响应约定沿用人脸：
`success=true, message="0"` 才返回成功回执；`success=false` 的 `1001/1002/1003`
分别表示空路径、文件读取失败、本端暂存失败，按原数字返回。
桥接成功回执沿用现有人脸格式，发送至 `luvbot/{SN}/up/reply`，不带 `data`：

```json
{"msgId":"<原下行指令ID>","cmd":"member.voice.train","code":0,"message":"0"}
```

桥接校验、保存、服务不可用、异常、忙碌和 10 秒超时回 `1006`，不记录特征原文；
超时后保持该类忙碌，直到迟到响应返回或重启，不覆盖可能仍在被业务端读取的文件。
服务受理不代表最终加载或音色合成成功；按本次范围不新增 `profile_train_done`。

机器人端先构建依赖并检查服务类型：

```bash
cd ~/zy_workspace/l0_deploy_v2_ws
colcon build --packages-up-to luvbot_cloud_bridge --cmake-clean-cache
source install/setup.bash
ros2 service type /algorithm/voice_perception/update_users
ros2 service call /algorithm/voice_perception/update_users \
  luvbot_control_interfaces/srv/UpdateUsers \
  "{path: '/userdata/forlinx/config/voice_db/users.json'}"
```

下发真实声纹数据，检查 `[VOICE] saved`、`service_pending`、`service_response` 和原指令关联回执。
验证新增/替换成员、重启保留旧成员、同指令重复、不同内容复用 ID、跨人脸/声纹复用进行中 ID、
坏向量、非法 `needTrain`、`needTrain=1` 缺任务、损坏旧库、服务不可用及超时迟到响应。
同一时间分别发送人脸和声纹，确认写入各自路径且调用各自服务。

### 名称与唤醒词：device.rename（清单 24）

唯一且非空的 `name` 作为服务请求 `keyword`；`alias` 不参与唤醒词设置。
设备显示名称由服务器资料管理，设备端负责应用同名唤醒词。

语音端和桥接节点须使用通用接口的新定义：

```srv
string keyword
---
uint8 result
string evaluation
```

```bash
ros2 interface show luvbot_control_interfaces/srv/SetCustomWakeWord
ros2 service type /wake/set_custom_keyword
ros2 service call /wake/set_custom_keyword luvbot_control_interfaces/srv/SetCustomWakeWord "{keyword: '明明'}"
```

语音端必须同步修改服务类型和实现，将 success 改为 uint8 result，去掉原来的 message/resource_id/resource_path，
`evaluation` 直接返回结果说明字符串，失败时为失败原因，不嵌套 JSON。
旧 `wheeltec_mic_msg` 类型不能直接互调。
本仓库不包含语音端源码，本次不修改或启动该服务端。

桥接异步调用服务，收到响应后直接发送 `code=result`、`message=evaluation`，不带 `data`。
`result=0` 表示成功，1、2、3、4 等非零值均表示失败，具体含义由语音端定义，不映射成表格示例码。
例如服务返回 `result=2, evaluation="失败原因"`，回执为：

```json
{"msgId":"rename-001","cmd":"device.rename","code":2,"message":"失败原因"}
```

没有收到服务响应时，桥接本地错误（不可用、参数错误、调用异常、忙碌、超时）保持
`code=1001` 和对应错误说明，不伪造语音端 result。默认等待 10 秒，可设置 `wake_timeout_sec`。

同一时刻只保留一个改名调用。超时回报 `wake service timeout; outcome unconfirmed`，
**客户端超时不会取消语音端执行**；为避免两次改词交叉，继续返回忙碌，直到迟到响应
返回或桥接重启。迟到响应按真实 `result/evaluation` 补发最终回执并更新完成缓存，
服务器按同一指令 ID 的最新终态处理；桥接不会自动重试服务调用。
最近 128 个已完成改名结果保存在进程内；相同 `msg_id` 与相同 Command 重发只返回
缓存结果，不再调用服务；相同 ID 的不同内容返回失败。忙碌拒绝不进入完成缓存。

### 改名联调日志

改名全流程以 `[RENAME]` 为前缀，正常阶段使用 INFO，拒绝、超时等使用 WARN/ERROR；
默认 INFO 日志级别即可看到，不需要 DEBUG。各阶段都带原始下行 msg_id，便于串起一次请求：

```text
[RENAME] ready                 服务名、通用服务类型和超时配置
[RENAME] received              下行 Topic、msg_id、设备 ID、trace_id、参数数量、载荷字节数
[RENAME] queued                已进入命令队列及队列长度
[RENAME] processing            开始处理
[RENAME] validated             实际 name/keyword 及字节数
[RENAME] service_call          即将调用服务及超时配置
[RENAME] service_pending       服务请求已提交，带 ROS request ID
[RENAME] service_response      原始 result、evaluation 及等待毫秒数
[RENAME] reply_queued          回执 code/message、是否缓存及回执队列长度
[RENAME] finished              调用已结束，可处理下一次改名
[RENAME] reply_submitted       回执已交给 MQTT，带 Topic、MQTT MID 和载荷字节数
[RENAME] reply_puback          broker 已确认收到该回执
```

异常/重复路径另打印 rejected、validation_or_call_failed、service_unavailable、busy、
timeout、late_response、service_response_failed、stale_response_ignored、cached_reply、
duplicate_in_progress、cache_evicted、reply_queue_full、reply_serialization_failed、
reply_publish_failed 等阶段。重复 ID 内容冲突会在 reply_queued 中打印具体错误原因。
断网或 MQTT 在途消息已满时打印 reply_waiting 并保留回执；相同回执/等待原因只在首次
出现或原因变化时打印，避免每 100 ms 刷屏。恢复发送后会打印 reply_submitted。

字段文本按 JSON 字符串转义，中文可读，单个文本最多打印 256 字节，超长内容标记
`[truncated]`。人脸特征日志策略不受影响。PUBACK 只确认 broker 收到，不代表服务器
业务已处理。本地两个模块状态话题仍为 1 Hz，供健康有效性判断使用。

### 其他下行

`volume.set`、`family.sync` 保持原有接收和日志行为，
本次不增加执行与回执。未知指令不执行。整帧无效或设备 ID 不匹配时不回业务回执。

## 情绪代码：清单 70

订阅 `/algorithm/brain/eyes_expression`（`std_msgs/msg/String`，reliable/volatile）：

```json
{"timestamp_ms":1787567123123,"emotion_id":"E01","intensity":"3"}
```

要求 `timestamp_ms` 为正整数、`emotion_id` 严格为 `E01`～`E08`（区分大小写），JSON
最多 4096 字节。每秒随当前属性帧发送 `metrics.key="mood"`，代码写入文本字段
`strValue`（对应 Protobuf 的 `str_value`），例如 `{"key":"mood","strValue":"E01"}`。
不发送 intensity、不转换情绪名称或数值。输入无效不覆盖最近有效代码；没有有效
输入时省略 mood，进程重启后等待新输入。代码作为持续状态保留到下次有效更新。

服务器协议的完整 JSON 视图示例：

```json
{"msgId":"prop-001","deviceId":"RUMI-TEST-002","ts":1792737404401,"seq":1,"propertyReport":{"fwVersion":"1.0.3","battery":86,"rssi":-58,"uptime":3600,"temperature":365,"netType":"wifi","metrics":[{"key":"mood","strValue":"E01"}]}}
```

MQTT 仍发送 Protobuf 二进制，以上是解码后的字段表示。六个基础属性使用前述固定
联调值，表情代码来自实际 ROS 输入；SN、消息 ID、时间戳和序号由实际配置与运行时生成。

## 性格五维：清单 80

订阅 `/algorithm/brain/personality`（`std_msgs/msg/String`，reliable/volatile，depth=10），
接收“小伴性格五维数据格式”中的 JSON 对象。该话题名是本桥接约定，大脑需向此话题发布。
桥接读取正整数 `published_at_ms`、有效日期 `date`（`YYYY-MM-DD`）和 `values`；
五个键 O/C/E/A/N 必须齐全，每个值必须为 0～100 的有限数值，整条 JSON 最多 4096 字节。
无效输入不覆盖有效状态。同一天只接受更大的 `published_at_ms`，早于当前日期的输入丢弃。

转换为服务器要求的五个英文键，数值除以 100，通过独立的 `up/property` 帧发送：

```json
{"key":"personality","strValue":"{\"openness\":0.6,\"conscientiousness\":0.449,\"extraversion\":0.7639,\"agreeableness\":0.6165,\"neuroticism\":0.4085}"}
```

该帧包含前述六个基础属性，metrics 仅有 personality。`levels`、`stage`、`day_number`
及人格名称/提示不上传；`published_at_ms` 和 `date` 只用于过滤旧输入，信封 `ts` 仍是发送时间。
首次有效输入上报一次，此后仅五维数值变化时上报，不加入每秒周期帧，不因重连重复生成。
断网或 MQTT 在途已满时，内存中保留最新有效状态，恢复后重试；等待期间的中间状态合并，
不补历史。提交 MQTT 后由现有 QoS 1 机制负责重传，broker ACK 不代表服务器业务入库。
进程重启会清空状态，大脑需重新发布当前数据。

机器人侧联调（先构建并启动通信节点，再发布）：

```bash
ros2 topic pub --once /algorithm/brain/personality std_msgs/msg/String \
'data: >-
  {"published_at_ms":1795058811832,"date":"2026-11-19","day_number":39,"stage":"stable","values":{"O":60.0,"C":44.9,"E":76.39,"A":61.65,"N":40.85},"levels":{"O":3,"C":3,"E":4,"A":4,"N":3},"personality":{"key":"sunny","name":"开心果","grown":true,"hint":"小伴最近变得更愿意主动找你玩了。"}}'
```

确认日志出现 `[PERSONALITY] submitted`，服务器解码得到上面的五维值。重复发布应无新增
personality 帧；增大时间戳并改变 O 后应新增一帧。再发送旧时间戳、缺失 N、越界值或非法 JSON，
均不应覆盖有效状态。断网时发布更新，重连后应补发最新值一次；持续观察周期帧应不含 personality。

## 健康信息：robot.health

每 10 秒通过已有 `luvbot/<SN>/up/event` 发送 Protobuf `EventReport`：
`event="robot.health"`，`detail` 为 JSON 字符串。软件保护生效时 level=3，其他为 1；
判断健康情况应读取 detail 中具体字段，不能仅靠 level 推断正常。
不会新增 `/system/health` 或 MQTT 话题，不修改 `PolicyObservation`。

复用 `/actuator/motor/state`、`/imu/data`、`/control/policy/observation`、
`/hardware_status/motors_ready`、`/system/safety/hardware_fault` 和
`/system/safety/emergency_stop`。仅补充以下两个模块状态话题，类型均为
`std_msgs/msg/String` JSON、1 Hz、reliable/volatile，无需修改 HPM 传输协议：

| 话题 | 发布模块 | 内容 |
| --- | --- | --- |
| `/system/motion_controller/status` | legged_controllers | 四种模式、控制器保护状态与原因、策略来源/是否需要/有效性/最后有效更新间隔 |
| `/hardware_status/fault_detail` | legged_real | 硬件保护与原因、启用电机索引/关节名称、内部锁存故障与位置确认状态、IMU状态、已有反馈超时配置 |

模块的 `schema_version=1`，`sample_age_ms` 来自控制线程实际快照；同时附带
`sample_monotonic_ns`，桥接使用同机 Linux 单调时钟补入 ROS 传输延迟。控制线程使用
try_lock 更新固定大小快照，不等待遥测线程；JSON 序列化由 ROS 定时器完成。
快照年龄加接收后经过时间超过 3 秒时，相应模块数据变为 `null`。定时器仍发布旧
快照时也不能把停滞的控制循环报告为正常。旧 Bool 信号没有周期心跳，只有对应硬件
模块快照有效时才附带其最近值；未收到仍为 null。

健康 detail 示例（数值仅演示）：

```json
{
  "schema_version": 1,
  "timestamp_ms": 1794441600000,
  "mode": "motion",
  "emergency": {
    "active": false,
    "controller": {"active": false, "reasons": []},
    "hardware": {"active": false, "reasons": []},
    "hardware_fault_signal": false,
    "hardware_stop_signal": false
  },
  "motors": {"expected": 26, "normal": 26, "abnormal": [], "ready": true},
  "imu": {"enabled": true, "timeout_ms": 5000, "valid": true, "raw_valid": true, "last_update_age_ms": 5},
  "policy": {"source": "amp", "required": true, "valid": true, "valid_for_ms": 100, "last_update_age_ms": 12},
  "system": {"cpu_temperature_c": 62, "cpu_usage_pct": 24, "memory_usage_pct": 31, "disk_available_bytes": 20000000000, "overheated": false},
  "policy_observation_live": true,
  "inference_enabled": true
}
```

- `mode` 为 standby/motion/calibration/emergency；Idle 按 standby 展示。
- 急停 active 表示软件已进入保护输出，不代表收到电机停稳确认。任一模块确认生效即
  为 true；只有两个模块都确认未生效才为 false，否则为 null。原因由实际触发位置
  记录，包括手柄、外部/硬件信号、姿态、策略过期、WBC worker 或 AMP 返回失败。
- 应工作电机集合来自实际硬件启用配置；正常数要求原始反馈新鲜、valid、无错误、
  无内部锁存故障且位置已确认。原始反馈丢失时不以 0 伪造故障码；异常条目包含
  index、joint、error_code、latched_error_code 和 reason。索引是 HPM 的零起始索引。
- IMU valid 为硬件接受的有效样本状态；raw_valid 为最近原始输入的检查结果，两者可
  在异常样本被硬件忽略时不同。更新间隔为单调时钟毫秒，禁用 IMU 时 valid 为 null。
- 策略 source 为 amp/imitation/wbc；非运动模式 required=false、valid=null，
  不将待机不推理判断为策略失效。最后更新间隔从对应来源最后被接受的命令计算。
  valid_for_ms 为命令剩余有效时间，按原命令 valid_until 衰减；到期不沿用旧的 true。
  controller_sample_age_ms/hardware_sample_age_ms 给出各模块快照年龄，便于区分采样时间。
- CPU 使用率为整机累计时间差，首个样本为 null；内存按 MemAvailable 计算；磁盘为
  根分区普通用户可用字节。温度取 CPU/core/cluster thermal zone 中最高值，无 CPU 温度时回退 SoC；不可读为 null。
- CPU 达到 85℃标记过热，降到 80℃恢复；温度读取失败时本帧 overheated=null。
  只用于上报，不触发新的硬件动作。无效模块报文也按未知处理，其余分组继续上报。

独立启动和配网入口可配置 `wake_timeout_sec`、`health_overheat_c`、
`health_recover_c`。实机入口对应 `cloud_wake_timeout_sec`、
`cloud_health_overheat_c`、`cloud_health_recover_c`，默认分别为 10/85/80。
模块状态发布随控制器启动，独立于 cloud_enabled；该开关仍只控制实机入口的 MQTT。
断网不生成新的健康遥测队列，重连后发送最新状态；libmosquitto 可能重发少量未 ACK
帧，服务器须按 Envelope.msg_id 去重，不能以到达时间覆盖更新的状态。

## 机器人端构建与验收

先同步部署仓库及匹配的 `luvbot_ros_interfaces`，语音端采用同版通用 srv，
在机器人 Linux/ROS2 环境、l0_deploy_v2_ws 根目录执行：

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-up-to luvbot_cloud_bridge legged_real
source install/setup.bash
```

构建依赖包括 libmosquitto-dev、libprotobuf-dev、protobuf-compiler、nlohmann_json
及已声明的 ROS 包。macOS 仅做静态检查，本次不创建测试程序或启动实机。

1. **人脸**：发送当前 SN 的合法向量，核对回执关联 ID、cmd、code=0、message=ok，确认无 data；检查重复键、
   缺字段、错误维度/编码/padding、NaN/Inf、全零向量、可选情绪字段不完整均失败。
   校验后按成员 ID 合并落盘并通知业务；重复已完成指令复用缓存，日志无特征原文。
   声纹按上面的清单 30 验证，最终加载事件不在本次范围。
2. **改名**：先确认上面的 service type，再下发 name=明明；核对服务实际生效及
   result/evaluation 回执；0 应成功，非零值（如1、2、3、4、255）应失败，
   `code` 与原始 result 完全一致，`message` 与 evaluation 完全一致，无 `data`。
   检查 unavailable、超时、迟到响应、
   并发忙碌、同 ID 重发及不同内容冲突。超时后不要以再次下发代替语音端结果确认。
3. **情绪**：发布上面的 String JSON，确认 up/property 解码后带
   `propertyReport.metrics=[{"key":"mood","strValue":"E01"}]`，
   无 intensity；验证 E01～E08 均可接收，E00、E09、E1、e01、空代码和其他文本被拒绝；
   非法 JSON、缺时间戳同样不覆盖有效代码。重启后等待新输入。
4. **健康**：只读观察两个模块状态及上行 JSON，核对四种模式、正常数和启用配置。
   在受控环境验证已有的急停/故障路径，不在运动中人为断开传感器。核对控制器与硬件
   各自原因、复位和恢复、策略失效/待机、数据停止超过3秒以及无法采集时的 null。
5. **回归**：确认心跳、固定属性与 BLE 配网正常。无/有表情时均检查六个属性固定值，
   改变或停止电池话题不应改变 battery=86；重连后仍附带固定属性并恢复最新表情和健康上报。
   检查服务调用期间 MQTT 心跳持续，原有控制频率、安全阈值和保护行为保持正常。
