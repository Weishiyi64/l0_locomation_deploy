# v2 集成补充

本包引入自同事 2026-10-29 提供的 C/C++ 版本，ROS 接口和 BLE UUID 保持兼容。
完整集成、权限配置和机器人联调见 [BLE Wi-Fi 配网](../docs/ble_wifi_provisioning.md)。

- `wait_for_trigger:=true`：常驻等待 `/luvbot/provision_trigger`（std_msgs/Empty），收到后启动配网；
  Wi-Fi 连接失败后留在当前配网会话，允许手机重试；成功退出或进程异常结束后继续等下一次 trigger，活动期间忽略重复 trigger。
- 默认 `wait_for_trigger:=false`：保持下面原有的一次性启动行为。
- 新增 `/luvbot/provision_state`；成功仍发布 `/luvbot/provision_result = ok`。
- 集成入口：`ros2 launch luvbot_cloud_bridge wifi_provisioning.launch.py`。
  通过 `network_ready_gate` 先检查已有网络；无网则等待本地网络恢复或 BLE 成功，再由 launch 启动 cloud bridge。
  cloud bridge 上线后以 1 Hz 分别发送状态心跳和 HPM 电量百分比；
  仅 BLE 配网可独立运行本包的 `provisioning.launch.py`，详见 [云端通信说明](../luvbot_cloud_bridge/README.md)。
- 普通用户通过非交互 `sudo -n` 启动守护进程；构建后在机器人运行
  `ros2 run luvbot_provisioning configure_provisioning_sudo.sh` 完成指定守护进程路径的授权。
  不再核对二进制 SHA256；路径不变时重编无需重新授权，安装路径变化时需更新授权。
  旧规则需要在机器人重新运行该脚本覆盖；root 运行时直接执行守护进程。
- 补齐 GATT read Flags，连接期间拒绝重复 Wi-Fi 写入；退出时清理连接子进程。
- C 日志逐行及时输出；`starting` 表示尝试启动，`advertising` 表示 BlueZ 已确认广播，
  `start_failed` 表示启动失败。扫描时应先看到 `[ADV] "LUVRUMI" advertising ON`。
- 保留紧凑 JSON 协议限制，新增长度/NUL 校验；两种模式均不支持 JSON 转义或分片。

---

# luvbot_provisioning

ROS 2 BLE Wi-Fi 配网包。`luvbot_bridge` 负责启动 C 守护进程 `luvbot_provd`，后者通过 BlueZ 提供 BLE GATT 配网服务。另有一次性改名模式：只保存新的蓝牙广播名称，然后退出，不启动配网。

## 依赖

- ROS 2 和 `colcon`
- `rclcpp`、`std_msgs`、`ament_index_cpp`、`launch`、`launch_ros`
- GLib/GIO 开发文件，且 `pkg-config --modversion gio-2.0` 可用
- Linux、BlueZ 和可用的蓝牙适配器
- 可通过 `sudo` 启动 `luvbot_provd` 的权限。节点无法交互输入 sudo 密码，请按设备策略配置权限

## 编译

在 ROS 2 工作区根目录执行，本包位于 `src/luvbot_provisioning`：

```bash
source /opt/ros/$ROS_DISTRO/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-select luvbot_provisioning
source install/setup.bash
```

## 持久化蓝牙名称

名称保存在用户配置文件：

```text
~/.config/luvbot_provisioning/luvbot_provisioning.conf
```

格式示例：

```ini
broadcast_name=LUVRUMI-001
```

该名称会持久保存，之后每次普通配网启动都会继续使用。用户配置文件、键或值缺失或为空时，默认名称为 `LUVRUMI`。包内安装的初始配置也使用 `LUVRUMI`。

## 启动配网

```bash
source /opt/ros/$ROS_DISTRO/setup.bash
source install/setup.bash
ros2 launch luvbot_provisioning provisioning.launch.py
```

此模式启动节点 `/luvbot_provision_once` 和 BLE 守护进程 `luvbot_provd`。节点读取已保存的名称，并用它设置蓝牙适配器 Alias 和 BLE 广播名称。未连接 Wi-Fi 时持续保留 BLE 广播，不设密码输入或空闲退出时间；配网失败后允许手机重新提交信息，不限制提交次数，也不会自动循环连接。每次提交仍最多尝试连接 3 次，每次 `nmcli` 等待 20 秒。配网成功时 C 守护进程立即通知 ROS 节点发布结果，同时继续保持蓝牙广播 2 秒供手机读取 SN；2 秒后守护进程关闭广播并退出，ROS 节点随之退出。手动停止或进程异常仍可结束配网。

## ROS 接口

| 方向 | 话题 | 类型 | QoS | 行为 |
| --- | --- | --- | --- | --- |
| 发布 | `/luvbot/provision_result` | `std_msgs/msg/String` | KeepLast(1), transient-local | 配网成功时立即发布 `data: "ok"`。配网失败不发布结果，保持 BLE 等待重试 |
| 订阅（仅改名模式） | `/luvbot/set_bluetooth_name` | `std_msgs/msg/String` | KeepLast(10), volatile | 消息内容是新名称纯文本。保存成功后配置节点退出 |

改名消息就是名称本身，不是 JSON，也不带 `broadcast_name=` 前缀。空名称会忽略，首尾空白会去掉。

## 手机读取 SN（FFF1）

蓝牙连接并发现服务后，手机主动 Read FFF1，即可获取 SN；无需先发送 Wi-Fi 信息或等待联网成功。
当前 SN 独立配置，默认值为 `RUMI-TEST-007`。FFF1 读取返回含 `status` 一个字段的 UTF-8 JSON，
其中 `status` 的值就是 SN：

```json
{"status":"RUMI-TEST-007"}
```

**FFF1 的 `status` 字段承载 SN，不表示 Wi-Fi 配网状态。** 没有单独的 `sn` 字段。
App 按 JSON 解码后读取 `status` 作为设备 SN；所有配网阶段均返回同一 SN，不能用它判断联网成功或失败。
原有内部 Wi-Fi 状态、ROS 成功反馈和 cloud bridge 上线／心跳流程保持原样，不包含在 FFF1 返回值中。
向 FFF1 写入 Wi-Fi 信息的协议仍为 `{"ssid":"WiFi名","pwd":"密码"}`。

| 项目 | 约定 |
| --- | --- |
| BLE 服务 UUID | `0000fff0-0000-1000-8000-00805f9b34fb`（FFF0） |
| 配网／SN 特征 UUID | `0000fff1-0000-1000-8000-00805f9b34fb`（FFF1） |
| FFF1 属性 | Read、Write、Write Without Response；不提供 Notify |
| FFF1 Read 返回值 | UTF-8 JSON，仅包含 `status` 字符串字段，其值为 SN，无换行和结尾 NUL |
| 默认完整返回值 | `{"status":"RUMI-TEST-007"}` |
| SN 有效字符 | 1～64 个 ASCII 字母、数字、下划线或短横线 |
| 兼容 SN 特征 | FFF2（`0000fff2-0000-1000-8000-00805f9b34fb`）仍只读，返回原始 UTF-8 SN 文本，例如 13 字节 `RUMI-TEST-007` |

手机端步骤：

1. 扫描广播名称（默认 `LUVRUMI`），建立 BLE 连接。
2. 发现 FFF0 服务及 FFF1 特征，主动发起 Read；连接本身不会自动推送 SN。
3. 将返回字节按 UTF-8 JSON 解码，读取 `status`，将它保存为设备 SN。
4. 向 FFF1 写入原格式的 Wi-Fi 信息；后续 Read 仍只返回 `status: SN`，不返回配网结果。
5. 如果沿用独立 SN 接口，Read FFF2 仍返回纯 SN 文本；两个特征的读取格式不同。

SN 使用启动参数 `device_sn`，不读取 MQTT 证书，不与 cloud bridge 的 `device_id` 自动绑定。
当前两者默认值恰好相同，修改 `device_id` 或蓝牙广播名称不会自动修改 `device_sn`。
参数在节点启动时生效；一次配网进程中的 SN 保持不变。FFF1 和 FFF2 Read 均支持 offset，越界返回 InvalidOffset。
FFF1 JSON 可能超过默认 MTU 下的一次读取长度，App 应使用 CoreBluetooth 等系统提供的完整特征读取 API，
自定义 ATT 客户端需按长值读取协议取得完整 JSON 再解析。

完整配网/上云入口：

```bash
ros2 launch luvbot_cloud_bridge wifi_provisioning.launch.py device_sn:=RUMI-TEST-007
```

只启动一次 BLE 配网，也可独立配置 SN：

```bash
ros2 launch luvbot_provisioning provisioning.launch.py device_sn:=RUMI-TEST-007
```

若直接运行节点：

```bash
ros2 run luvbot_provisioning luvbot_bridge --ros-args -p device_sn:=RUMI-TEST-007
```

若直接运行 C 守护进程，必须显式提供 `--sn`，例如：

```bash
sudo ./luvbot_provd --name LUVRUMI --sn RUMI-TEST-007
```

### 更新与机器人验证

修改了守护进程后，在机器人工作空间重建；首次授权、替换旧摘要规则或安装路径变化时更新 sudo 授权：

```bash
cd ~/zy_workspace/l0_deploy_v2_ws
colcon build --packages-up-to luvbot_provisioning luvbot_cloud_bridge
source install/setup.bash
ros2 run luvbot_provisioning configure_provisioning_sudo.sh
ros2 launch luvbot_cloud_bridge wifi_provisioning.launch.py device_sn:=RUMI-TEST-007
```

在另一已 source 的终端发布 `/luvbot/provision_trigger` 后，用 App 或 BLE 调试工具验证：

- 未发送 Wi-Fi 信息时，FFF1 返回 `{"status":"RUMI-TEST-007"}`。
- Wi-Fi 连接中、成功及失败阶段，FFF1 均返回 `{"status":"RUMI-TEST-007"}`；没有 `sn` 字段。
- Wi-Fi 成功仍通过 ROS 结果话题反馈；FFF1 的 SN 响应不代表配网成功。
- 启动配网后超过 120 秒不做 BLE 读写，仍可扫描、连接并读取 SN；连接手机后停留超过 120 秒再提交密码仍可配网。
- 连续提交错误密码并等每轮连接尝试结束，至少 3 轮后再等待超过 15 秒，广播仍保留；无需重新 trigger 即可提交正确密码并成功连接。
- FFF2 仍可读出纯文本 `RUMI-TEST-007`，并拒绝写入；FFF1 写入 Wi-Fi 的格式不变。
- 配网过程中及成功后的 2 秒 BLE 保持期内仍能读 SN；C 进程退出后要重新 trigger 才能再连接。
- 以另一个 `device_sn` 启动时，FFF1 的 `status` 和 FFF2 的文本均返回新 SN，MQTT `device_id` 保持原设置。
- 手机缓存过旧 GATT 服务时，重新发现服务；调试工具仍看不到 FFF2 时断开、清除该设备的 GATT 缓存再连接。

## 不配网，只修改名称

同事或服务器要更改蓝牙广播名称时，使用一次性配置节点。它不启动 `luvbot_provd`；成功保存一个有效名称后就退出。之后启动配网时会使用保存的名称。

先启动一次性配置节点：

```bash
ros2 launch luvbot_provisioning configure_bluetooth_name.launch.py
```

在另一个已 source 环境的终端确认订阅已就绪：

```bash
ros2 topic info /luvbot/set_bluetooth_name
```

然后发送新名称：

```bash
ros2 topic pub --once /luvbot/set_bluetooth_name std_msgs/msg/String "{data: 'LUVRUMI-001'}"
```

配置节点会记录保存成功并退出。检查持久化结果：

```bash
cat ~/.config/luvbot_provisioning/luvbot_provisioning.conf
```

改名话题使用 volatile QoS，所以要等配置节点启动并能看到订阅后再发布；节点启动前发出的消息不会保留。此命令模拟 ROS 消息送达本包，服务器接入时也应把收到的新名称发布到该话题。

改名模式只负责保存配置，不启动也不重启 BLE 守护进程。名称会在下一次启动配网时生效。

## 测试配网结果

在另一个已 source 环境的终端先开始监听，然后用手机或真实 BLE 客户端完成配网：

```bash
ros2 topic echo /luvbot/provision_result
```

配网成功后应收到：

```yaml
data: ok
```

结果发布使用 transient-local durability。发布结果后节点最多再运行约 2 秒，因此监听器应在配网成功前或这 2 秒内连接：

```bash
ros2 topic echo --qos-durability transient_local /luvbot/provision_result
```

若只想测试 ROS 消息接收链路，可以手动发布一条样例：

```bash
ros2 topic pub --once /luvbot/provision_result std_msgs/msg/String "{data: 'ok'}"
```

这只测试消息接收，不代表真实 BLE 配网成功。

## 查看接口

```bash
ros2 node list
ros2 topic list
ros2 topic type /luvbot/set_bluetooth_name
ros2 topic info /luvbot/provision_result
ros2 interface show std_msgs/msg/String
```

## 恢复默认名称

通过一次性配置节点发送 `LUVRUMI`，或将用户配置中的值改为 `LUVRUMI`。名称会持续保存，正常使用中不需要删除配置文件。

## 行为说明

- BLE 守护进程在未连接 Wi-Fi 时持续等待，不因空闲或连续失败退出；成功时先立即通知 ROS 发布结果，再继续广播 2 秒后退出。普通 ROS 节点随后退出。未完成配网时可以手动停止进程。
- ROS 配网结果话题仍只发送字符串 `ok`；BLE FFF1 读取返回 `{"status":"设备SN"}`。两者均不包含 SSID、密码或蓝牙名称。
- 改名节点收到并保存名称后退出。若未收到消息，它会等待；ROS executor 空闲等待消息，不会持续轮询占用 CPU。
