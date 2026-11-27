# 电池快照

HPM 仅缓存原始 CAN 数据；物理量在 `hpm_battery` 解码。该包是由
`hpm_system` 创建的库，不另占用网络连接，不发送电池或电机控制命令。

## 启动

```bash
colcon build --packages-up-to hpm_system --symlink-install
source install/setup.bash
ros2 launch hpm_system hpm_system.launch.py \
  board_ip:=192.168.1.10 enable_udp:=true launch_battery:=true \
  launch_motor:=false launch_imu:=false launch_tactile:=false enable_tactile_uart:=false
```

`launch_battery` 默认 `false`。启用后，每收到一个完整有效快照，发布一条
`luvbot_hardware_interfaces/msg/BatteryState` 到 `/battery/state`，正常为 10 Hz。
QoS 为 SensorDataQoS，深度 5；不在 ROS 端定时重复缓存数据。

## 线协议 v1

- `HPU1` 类型 11；单个 UDP 数据报恰为 709 字节。
- 公共头 20 字节，电池元信息 20 字节，29 个 23 字节槽位，尾部 CRC16/Modbus 2 字节。
- 元信息：`version:u8=1, count:u8=29, port:u8=6, reserved:u8=0`，随后依次为
  `sequence:u32, board_ms:u32, received_mask:u32, updated_mask:u32`。
- 槽位：`msg_id:u8=0x40, update_count:u16, age_ms:u32, reserved:u8=0,
  port:u8=6, ide:u8=0, can_id:u32, dlc:u8=8, data:u8[8]`。
- 外层多字节字段为大端；CAN 数据字节不变。CRC 覆盖公共头和负载，排除 CRC 本身。
- 槽位 0～13：`210,211,212(index0),214,215,216,217,218,219,21A,21B,21C,21D,21E`。
  槽位 14～28：`212(index1～15)`。CAN ID 以上均为十六进制。
- `received_mask` 表示启动后曾收到；`updated_mask` 表示自上次组包后收到新帧，
  即使字节未变化。未收到的槽位全零。`age_ms` 自 Core0 消费原始反馈时计算，
  使用 64 位单调时钟并饱和到 UINT32_MAX，不是精确 CAN 线上接收时间。
- Core0 持有本地缓存和互斥锁，以 100 ms 独立任务复制后发送；不改跨核共享内存。
  正常已聚合帧不再逐帧上传；异常/短帧和不在映射中的 ID 保留原始转发。
- 无主机会话或 OTA 期间不发送电池包；不积压旧快照。新 UDP 类型要求主机、固件配套更新。

## 解析依据与字段

解析以供应商 `YB-BMS_CAN_V2.0_2026101301.dbc` 为准，覆盖 0x210～0x21E 中
实际定义的报文中除 0x212 外的 13 种电池报文。0x213 不存在；充电器 0x500～0x502 不在电池快照范围。

消息开头依次为 `header`、`soc`、`soh`、`sop`、`charge_state`、`full_charge`、
`voltage`、`current`。其余字段保留 DBC 信号对应的小写下划线名称；MOS2/3 的
ROS 字段名沿用原接口，消息注释标明新版 DBC 的原始信号名。

- `voltage`：V；`current = uint16_le(Byte6..7) * 0.01 - 327.68` A。
  电流保留 DBC 原始符号，不根据端口状态取绝对值，也不再取反。0x8000 为零；
  实测小负载放电的 0x7FEB 对应 -0.21 A。幅值精度和实际充电读数仍待仪表校验。
- `sop`：kW，原始值 × 0.1。例如 0x0057=87，输出 8.7 kW，不能按旧接口解释成 87 W。
- `soc`、显示 SOC 和 `realsoc` 均为百分数；`soc` 的 DBC 范围为 0～120。
- `charge_state`：2位原始状态，0 空载、1 放电、2 充电、3 关闭。
  DBC 的 VAL_ 还列出“4 故障”，但2位无法表示4；代码保持2位提取，待厂家澄清。
  `full_charge`：false 未满充、true 满充。`bms_status_is_awake` 保持原始位：
  false=awake、true=sleep，不按名称反转。
- MOS2：Byte3=ChgMos2State，Byte4=DchMos2State；
  MOS3：Byte5=ChgMos3State，Byte6=DchMos3State。
- 0x218 新增 `bms_cell_volt_num`；0x21B 新增 `sum_volt`（V）。
- 0x21C：`asw_soc_get_clbrt_flag`、`get_bms_fault_level`、`get_sys_state`、
  `get_step`、`get_fault_list`、`get_fault_num`、`realsoc`（原始值 × 0.1 %）。
  DBC 没有提供状态/故障字段的枚举，不据名称推导具体故障含义或扩展故障数组。
- 0x21D 额外提供 `ex_volt2`、`chg_volt`、`inter_volt`（原始值 × 0.1 V），
  `get_sys_key_io` 和 `get_chg_wake_up`；三个原有唤醒位继续保留。
- 0x21E 只有5路温度，Byte0～4依次为电芯1、2、MOS1、2、预放电阻，
  原始值减50，DBC 范围 -50～205 ℃。移除旧表中的电芯温度3、4。
- 单体电压 BCU_Vol_1～11 的 DBC 范围是 0～65.535 V；FF FF 对应65.535 V。
  0x216 的温度极值范围仍为 -50～200 ℃，与 0x21E 不同。

DBC 没有定义额外无效码，不再沿用旧表统一把 FF/FFFF 当成无效值的规则。
浮点值超出各信号 DBC 标注范围时发布 NaN；范围内原样换算，不擅自按物理经验裁剪。
未收到的浮点字段为 NaN、整数为255、循环次数为 UINT32_MAX；布尔字段默认false。
已收到但过期的数据保留最后一次解码值；本话题不输出链路诊断，不能仅凭数值存在
判断来源 CAN 仍在更新。ROS 只额外保留 `header` 接收时间，不恢复 raw 数组、年龄、掩码等字段。

ROS 不再解码或重组 0x212，也不发布 `bms_serial_number_index`、
`bms_serial_number_total`、`bms_serial_number_data`。板端及传输层仍保留原始序列号槽位。
移除已不再使用的 `battery_serial_timeout_ms`、`battery_timeout_multiplier` 参数；
其他物理量仍保留最后收到的值，不按年龄清空。

HPM 固件、固定槽位和709字节UDP格式保持不变；本次仅需更新3588的ROS包。
消息字段和 SOP 单位发生变化，依赖该消息的程序须重新编译、source并重启。

```bash
ros2 interface show luvbot_hardware_interfaces/msg/BatteryState
ros2 topic echo /battery/state --field current
```

## 验证

```bash
colcon test --packages-select hpm_battery
colcon test --packages-select hpm_system --ctest-args -R test_battery_runtime --output-on-failure
# test_battery 的 --decode 模式供独立参考检查调用；路径按实际构建目录调整。
python3 src/hpm_transport_ros2/src/hpm_battery/test/test_dbc_reference.py \
  build/hpm_battery/test_battery /path/to/YB-BMS_CAN_V2.0_2026101301.dbc
```

测试覆盖最新值覆盖、低频保留、年龄饱和、各字段换算、DBC范围边界、NaN、电流偏置编码边界与实测原始值、
0x212 原始槽位保留但不进入话题、固定线格式，以及本机 UDP 会话、CRC、布局、
重复/乱序拒绝、发布开关和断流后不伪造样本。本机模拟不是实物通信证明。

部署顺序：先构建并部署 3588 的 ROS 包，再更新 HPM。随后仅启用电池模块，
连续采样 60 秒，期望约 600 包、话题约 10 Hz，并核对真实原始帧与解析数据。
本次若远端不可达，部署和此项实物验收必须明确标记为未完成。

### 2026-10-22 本地验证记录

- LUV 双核构建、7 个 ROS 包构建通过；电池缓存/解码和本机 UDP→话题集成测试通过。
  首次模拟采样为 40 个唯一快照、约 9.86 Hz，不是实物上报频率。
- 电机包测试、触觉测试及现有 USB 日志门控测试通过。
- 现有 IMU `SupportsOutOfOrderFrames` 和系统 `test_motor_set_zero_runtime` 失败；
  已分别在从未修改 HEAD 导出的独立构建中复现相同失败，本次未修改其实现或预期。
- 首次本地验证时 SSH 不可达，随后已恢复连接并完成远端源码同步及 ROS 编译。
  上述离线测试不替代实物验收；固件更新与实物采样结果须单独记录。

### 2026-10-23 DBC V2.0 校正验证

- 本地及3588的7包构建、电池单元测试、UDP→ROS集成测试通过。
- 独立读取供应商DBC：14种报文、80个电池字段、99组参考向量、7821项标量检查及序列号重组对照通过。
- 3588临时只启用电池模块，20秒收到201条、约10 Hz；话题电流 -0.33～-0.18 A，
  最后一条 -0.21 A、SOP 8.1 kW、电压39.90 V。电芯温度30/30 ℃、MOS及预放电阻温度33/33/33 ℃。
- 实物话题采样和编码/解码对照通过不等于电流表标定。临时采样节点已退出；启动正常launch使用新解析。

### 2026-10-23 移除 ROS 序列号输出

- 删除 0x212 重组逻辑、三个序列号消息字段及无用途的超时参数。
- 板端和传输层线协议不变；更新后须重新构建消息及依赖包，并重新 source、重启节点。
