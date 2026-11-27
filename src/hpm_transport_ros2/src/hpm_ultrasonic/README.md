# FD07-3 自动串口超声波

`hpm_ultrasonic` 是由 `/hpm_system` 持有的 C++ 发布库，结构与 `hpm_imu` 一致。
使用现有 UDP transport，不创建额外节点、串口设备或网络连接。

## 接线与固件

LUV 原理图 `MCU_6E80_L0_V10.pdf` 第 3、8 页：J20-1 为 5V，J20-2 为
UART6_TXD/PC24，J20-3 为 UART6_RXD/PC25，J20-4 为 GND。
FD07-3 红线接 J20-1、白线 T/TX 接 J20-3、黑线接 J20-4，绿线 E/RX 和 J20-2 不接。
按连接器实际针号接线，不按线束颜色或观察方向推断针序。

J20 是 GH1.25，说明书传感器端是 PH2.0，需要匹配线束。传感器必须是**自动串口 TTL**版本；
文档未给出软件切换到该模式的指令。5V 是供电电压，接入前应测量 TX 高电平并核对 PC25
允许输入电平，必要时增加电平转换；不能将 485 或 RS232 输出直接接入。

Core1 在 LUV 构建中启用 UART6：9600、8N1、无流控、RX 中断、32 项字节/时间接收环。
以 `ULTRASONIC_RX_GAP_MS=5` 毫秒空闲间隔划分突发，在主循环约 1 ms 周期转发。
该宏可用编译定义覆盖；需按真实字节间隔确认阈值。UART ISR 被长时间阻塞时，FIFO 中
字节的原始到达时刻无法重建，须通过混合负载实测确认中断延迟。
硬件错误、接收环溢出、超过 8 字节的突发整段丢弃，等待空闲边界恢复。
不足三字节或校验错误的突发仍原样转发。无 DMA、无 RTOS、无板端测距解析。
`ultrasonic_uart_get_stats()` 可读接收、硬件错误、环溢出、超长、反馈丢弃和已转发计数；
错误变化时最多每秒输出一次累计日志。

## 网络格式

外层沿用 30 字节 HPM v2 帧和现有 HPU1/UDP 会话。类别 `0x50..0x5F`，本期 `msg_id=0x50`。
该消息将非电机 16 字节 payload 按以下字段解释：

```text
offset 12: reserved=0
offset 13: port=6
offset 14: ide=0xFF
offset 15..18: can_id=0xFFFFFFFF
offset 19: dlc=原始突发长度，1..8
offset 20..27: 原始字节，尾部补零
```

例：`03 E8 EB` 原样存入 `data[0..2]`，`dlc=3`。外层 CRC16 与传感器累加校验独立。
无活动主机、OTA 暂停、发送失败和队列满沿用当前反馈丢弃策略，不补发旧测距。
外层时间戳是当前公共封装生成时间，不能当作超声波采样时间。

## 启动与话题

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch hpm_system hpm_system.launch.py enable_udp:=true launch_ultrasonic:=true
ros2 topic echo /sensor/ultrasonic/range --qos-reliability best_effort
```

参数：`launch_ultrasonic=false`；`ultrasonic_frame_id=ultrasonic_link`；
`ultrasonic_field_of_view=0.6981317`（40°，单位 rad）；`ultrasonic_timeout_ms=500`。
80°探头设置 `ultrasonic_field_of_view:=1.3962634`，角度需按实际型号确认。
这些参数在启动时生效，开关仅控制 ROS 库，关闭时不创建 publisher 或超时定时器。
其他 `launch_imu/launch_motor/launch_tactile/launch_battery` 参数保持原有行为。

发布 `/sensor/ultrasonic/range`，类型 `sensor_msgs/msg/Range`，QoS 为 `SensorDataQoS`。
仅接受来源元数据正确、长度 3、`(high+low)&0xFF == checksum`、距离 30..1800 mm 的帧。
`range` 单位为米，`min_range=0.03`，`max_range=1.8`，类型 `ULTRASOUND`。
`header.stamp` 是 ROS 接收到有效数据的时间，不是精确采样时间；不自动发布 TF。
无帧头，因此不跨突发拼接残帧，也不在异常突发中滑动搜索校验。
无回波特殊码在说明书中未定义，超量程或坏校验不发布，不猜测 `0/FFFF` 的含义。
连续无有效数据 500 ms 后告警一次，有效数据恢复后记录恢复；不重复发布历史距离。

## 验证

```bash
python3 tests/check_ultrasonic_uart.py
cd deploy_ws
colcon build --packages-up-to hpm_system --symlink-install
source install/setup.bash
colcon test --packages-select hpm_ultrasonic hpm_system --event-handlers console_direct+
colcon test-result --verbose
```

主机 C 测试运行实际 UART 接收代码，使用 MMIO/时钟 mock 检查分帧和异常恢复；
ROS 测试使用本机 UDP 模拟板端，检查校验、开关、Range、超时和重连。它们不证明实物工作。
固件分别使用 `./compile_app.sh --normal` 与 `./compile_app.sh` 构建。

实板验收：测量电平和 9600/8N1 波形；逐字节对照 UART/ENET 抓取；测试至少三个已知距离，
按说明书误差 `±(0.5+0.006*距离) cm` 评估；断开/重连传感器检查恢复；与电机、IMU、触觉
同时运行至少 10 分钟，核对错误与丢弃计数。烧录、波形、真实距离和混合负载验收需另行记录。

### 本次验证记录（2026-10-29）

- LUV 普通固件和 OTA 固件构建通过，ROS `--packages-up-to hpm_system` 共 8 包构建通过。
- 实际 C 接收代码的主机测试通过，包括延迟 poll、残帧原样转发、超长、硬件错误、环溢出、
  毫秒回绕、反馈队列失败和外层 C 协议 CRC。
- 超声波 3 个解析测试通过；本机 UDP 到 Range 测试通过，包括开关、10 Hz 输入、两层校验、
  量程边界、超时/恢复、会话 RESET 重连及不发布历史测距。
- 安装后的实际 `hpm_system.launch.py` 启用模块与传入类型化参数检查通过。
- 原有触觉和电池测试通过。扩展回归中，IMU `SupportsOutOfOrderFrames` 和电机
  `test_motor_set_zero_runtime` 失败；分别用 HEAD 的解析器/测试和 HEAD 的系统节点独立编译，
  均复现相同失败。未修改这些既有模块的逻辑，不能将全套回归标记为通过。
- 日志留在根目录 `build/ultrasonic_validation/`；本次没有烧录、更新签名 OTA 包或实板验收。
  编译产物为相应 normal/OTA 构建目录中的 `core0/output/demo.elf`；现存签名包不代表此次新固件。
