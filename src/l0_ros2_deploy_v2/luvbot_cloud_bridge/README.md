# 电压 MQTT 联调（Ubuntu / ROS 2 / C++）

本轮是上行发布测试：每 100 ms 生成一次 `48 + 2*sin(2*pi*t/20)` V，并通过同一条
MQTT TLS 连接发送 Protobuf。范围 46–50 V，20 秒一周期。代码在
`src/voltage_mqtt_test_node.cpp` 的 `simulatedVoltage()`，模拟范围和频率写死。
不读取电池、不发布机器人电池话题、不执行动作。自动订阅 `luvbot/<device_id>/down/command`，
仅接收、解析并记录 `volume.set`，不实际调节音量，也不发送业务成功或失败回执。
本测试不能与之前使用同一 Client ID 的 smoke test、mosquitto_sub 或 MQTTX 同时运行。
本轮不要同时下发动作指令。持久会话可能继承之前的订阅：不属于当前设备下行 Topic
的消息会被忽略。MQTT 库仍会确认收包；这不代表指令已执行，后续执行测试请发送新的指令编号。

## 1. 只在机器人上安装编译库、编译

需要 Ubuntu 已有 ROS 2 开发环境，另需普通 C/C++ 库（不是 ROS MQTT 插件）：

```bash
sudo apt update
sudo apt install libmosquitto-dev libprotobuf-dev protobuf-compiler pkg-config
```

将本仓库的修改和 `luvbot_cloud_bridge/certs/` 中的证书文件一起同步到机器人原有 ROS 工作空间。
证书被 Git 忽略，单独 git pull 不会带上它们；必须先复制证书再构建。
终端先加载你实际使用的 ROS 2 环境，
再进入工作空间根目录（包含 src 的那一层）：

```bash
cd /你的/机器人/ROS工作空间
source install/setup.bash
colcon build --packages-select luvbot_cloud_bridge --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

如果工作空间尚未有 install，可跳过第一次 `source install/setup.bash`，但必须先加载系统 ROS 环境。
`protoc` 在 Ubuntu 构建时生成 C++ 代码，原始 proto 与同事提供的文件一致。

## 2. 首次测试只启动通信节点

设备证书放在本包源码目录中：

```text
luvbot_cloud_bridge/certs/LUV2026090001.crt
luvbot_cloud_bridge/certs/LUV2026090001.key
```

构建时将它们安装到 ROS 包的 `share/luvbot_cloud_bridge/certs/`，节点通过 ament 索引自动定位，
不再依赖程序文件夹以外的设备证书目录。默认启动不需要传 `cert_dir`。
在机器人上确认安装结果（仅列出路径，不输出证书或私钥内容）：

```bash
ls -l "$(ros2 pkg prefix luvbot_cloud_bridge)/share/luvbot_cloud_bridge/certs/"
```

源码及常规安装后的凭据权限为 600，使用机器人运行用户构建并启动。
服务器 CA 验证仍使用 Ubuntu 系统信任库 `/etc/ssl/certs/ca-certificates.crt`。

停止同身份的其他 MQTT 客户端，然后运行：

```bash
ros2 launch luvbot_cloud_bridge voltage_test.launch.py
```

默认服务器 `mqtt.luvbotics.com:8883`，MQTT 3.1.1，设备 ID 为 `LUV2026090001`，
QoS 1、不 retain、保留会话，校验 CA 和服务器域名。设备证书保存在包内，但不提交 Git。
如需显式指定其他证书目录（可选）：

```bash
ros2 launch luvbot_cloud_bridge voltage_test.launch.py cert_dir:=/你的证书目录
```

启动显示 `SIMULATION`；收到成功 CONNACK 后显示 `MQTT connected`。
每 5 秒汇总一次电压、连接状态、queued_total、broker_ack_total、pending 和 skipped。
计数包括电压事件与 30 秒一次的上线心跳。
`queued_total` 只表示交给客户端库；`broker_ack_total` 表示 EMQX 已确认，不能证明服务端业务已解析。
断线自动重连；最多保留 20 条未确认消息，断网／积压时跳过新样本，重连可能重发少量未确认的旧消息。
正常网络下目标 10 Hz，非硬实时保证；QoS 1 可能重复，接收端可用 msg_id 去重。
Ctrl+C 停止，离线帧尽力发送；异常断线采用 up/status 空载荷遗嘱。

## 2.1 只接收音量指令

服务器已确认下行 Topic 为 `luvbot/{SN}/down/command`，节点使用当前 `device_id` 作为 SN
自动生成订阅地址。默认订阅 `luvbot/LUV2026090001/down/command`，不需要额外 Topic 参数。

在机器人上完成第一节的构建更新后启动单节点：

```bash
ros2 launch luvbot_cloud_bridge voltage_test.launch.py
```

订阅使用 QoS 1，每次重连后重新订阅。只有看到
`Command subscription confirmed: <Topic> QoS=... (receive only)` 才表示 broker 已接受订阅；
订阅被拒绝时会输出错误，此时检查设备证书对应的 Topic ACL。

让服务器发送 Protobuf 二进制 `Envelope`，不要发送 hex 文本或 JSON 文本。
必须使用实际设备 ID：当前默认 `LUV2026090001`；同事示例中的 `RUMI-TEST-002`
需要在服务端重新生成报文时替换为实际 ID，否则节点会拒绝。若使用测试设备身份，
需同时传入对应 `device_id`，并提供匹配的设备证书。

对于 `command.cmd = "volume.set"`、`params = [{key: "volume", value: "24"}]`，
收到并校验通过后输出：

```text
RECEIVE ONLY volume.set volume=24; no volume change or business reply. frame={...}
```

`frame` 包含解码后的 `msg_id`、`device_id`、`ts`、参数和 `trace_id`（有值时显示），
可与服务器日志逐项核对。仅接受当前 Topic、匹配设备 ID、非空消息 ID 且包含 Command 的帧；
载荷限制为 1–4096 字节，`volume` 必须是唯一且非空的十进制整数字符串。
本阶段不转换或限制音量范围，实际范围和执行语义留待音量功能接入时确定。
未知指令、错误设备 ID、无效 Protobuf、缺失或重复音量参数均有日志，不执行任何动作。
QoS 1 可能重复投递，本阶段逐条记录，不做执行去重。业务端等待回执可能超时，这是本轮仅验证接收的预期。

机器人端验收：发送实际设备的 `volume=24`，核对日志中的参数和消息 ID；
断网重连后确认重新收到订阅确认及新指令；发送错误设备 ID、缺失音量参数或 hex 文本，
确认被拒绝。此阶段不会向 `up/reply` 发布业务回执。

## 3. 发给服务器同事的核对信息

电压没有现成的专用字段；proto 的 `battery` 是电量百分比，不能塞入电压。
因此本轮使用一个**新增的测试事件名**，保持 Envelope/EventReport 原有格式：

```text
Topic: luvbot/LUV2026090001/up/event
Payload: Protobuf 二进制，不是文本 JSON，也不是 hex 字符串
Envelope.device_id: LUV2026090001
Envelope.msg_id: voltage-test-<本次进程UUID>-<序号>
Envelope.ts: 当前 Unix 毫秒时间戳
Envelope.seq: 本进程自增序号
Envelope.event_report.event: test.battery_voltage
Envelope.event_report.level: 1
Envelope.event_report.detail: {"voltage":48.000,"unit":"V","simulated":true}
```

上面的 JSON 是 `detail` 字符串的内容。服务器使用现有 `Envelope.parseFrom(payload)` 解码，
读取 `getEventReport().getDetail()`，再将该字符串解析成 JSON。
每秒约 10 条，电压随时间在 46–50 V 内变化。
另有 `luvbot/LUV2026090001/up/status` 上线报文和 30 秒一次心跳。

`test.battery_voltage` 是本轮新增的联调约定，**不属于现有 Excel 的业务事件清单**。
如果业务层按事件白名单过滤，需在 MQTT 入站／Protobuf 解码处查看或允许该测试事件。
不能以 App 没显示电压作为“服务器没有收到”的判断，也不保证自动入库。
同事如另有电压字段／事件名约定，应双方一致后再替换，不能把本事件当成已确认的生产协议。

## 4. 跟随现有机器人 launch 启动

同步修改后的 `legged_controllers/launch/sim2real.launch.py` 并更新安装：

```bash
colcon build --packages-select legged_controllers
source install/setup.bash
```

先退出单独运行的测试节点，然后在你平时启动命令末尾加：

```bash
ros2 launch legged_controllers sim2real.launch.py cloud_voltage_test_enabled:=true
```

保留你原有的其他启动参数。此命令会启动机器人原有完整程序；首次通信测试使用第二节的单节点命令即可。
当前 HPM 主 launch 的通信开关默认 `false`；可以传 `cloud_voltage_test_enabled:=false` 关闭。无需启用有故障的电池驱动。
主 launch 可用 `cloud_cert_dir`、`cloud_device_id`、`cloud_host` 覆盖路径和身份。
下行 Topic 自动跟随 `cloud_device_id`，无需额外配置。

## 5. 后续换回真实电压

将 `tick()` 中的 `simulatedVoltage(seconds)` 替换为已校验的新鲜真实电压，删除正弦函数。
若数据来自原有电池 ROS 消息，读取 `/battery/status` 的 `sum_voltage`（V），由订阅回调缓存；
不要在 100 ms 定时回调中阻塞等待硬件。真实电压接入后必须同时将 detail 的 `simulated` 改为 `false`，
移除模拟日志，并与服务器约定正式电压上报方式。

本轮未在 Mac 编译或连接服务器。静态校验后，需要在 RK3588 上确认编译、TLS、频率和服务端解析结果。
库接口参考：https://mosquitto.org/api/files/mosquitto-h.html
