# 电机 CAN ID 标定

`motor_id_calibration.py` 使用 ROS service，通过 `hpm_system → TCP → HPM → CAN` 配置电机 ID。
支持 RS、MR、LS；不执行运动、使能/失能或调零，不修改 YAML，不配置 RS zero_sta 或 LS CAN_Timeout。

## 启动与操作

在外层工作区 `deploy_ws` 中构建并加载环境：

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-up-to hpm_system
source install/setup.bash
```

标定前停止运动指令并让电机停止运动。待标定 CAN 总线只允许一颗电机上电。
开启专用标定模式（将 board_ip 改为实际 HPM 地址）：

```bash
ros2 launch hpm_system hpm_system.launch.py \
  enable_tcp:=true board_ip:=192.168.1.10 \
  motor_id_calibration_mode:=true \
  motor_config_file:="$PWD/src/hpm_motor/config/motors.yaml"
```

标定模式会持续拒绝控制、使能/失能和调零，但不会主动发送失能命令。完成标定后，停止节点，
用 `motor_id_calibration_mode:=false` 重新启动。参数只允许在启动时设置。

另开终端，在同一工作区加载环境后运行：

```bash
source install/setup.bash
# 交互选择 CAN 总线下的关节（输入配置 index）
python3 src/hpm_transport_ros2/scripts/motor/motor_id_calibration.py
# 单颗 MR，默认当前 ID=1；RS 默认当前 ID=0x7F，LS 默认=1
python3 src/hpm_transport_ros2/scripts/motor/motor_id_calibration.py --motor-index 1
# 指定已改过的当前 ID
python3 src/hpm_transport_ros2/scripts/motor/motor_id_calibration.py --motor-index 1 --current-id 0x03
# 只读查询旧/新 ID，普通运行模式也可使用
python3 src/hpm_transport_ros2/scripts/motor/motor_id_calibration.py --motor-index 1 --query --current-id 0x02
# 不知道当前 ID：自动查询所选关节对应的总线和协议
python3 src/hpm_transport_ros2/scripts/motor/motor_id_calibration.py --motor-index 18 --query
# 全机自检：配置电机均上电，连续查询三次；LS 按总线广播
python3 src/hpm_transport_ros2/scripts/motor/motor_id_calibration.py --check-all
# 不初始化 ROS、不发送命令、不写日志
python3 src/hpm_transport_ros2/scripts/motor/motor_id_calibration.py --dry-run --motor-index 1
```

脚本默认使用本工作区源码配置，可用 `--config` 指定其他路径。它在任何写入前先查询并比较节点
返回的名称、协议、MR 型号、总线和目标 ID；不一致就停止。非 MR 型号字段不参与此标定。
ID 范围：RS/MR 为 1～255，LS 为 1～2047。index 为 0～255，bus_index 为 0～7。

`--query` 未指定 `--current-id` 时，LS 广播返回当前 ID；RS/MR 先探测配置 ID、出厂 ID，
再遍历其余 1～255，完整扫描默认约需 4 分钟。`--service-timeout` 是单次服务调用期限，不是整次扫描期限。
可 Ctrl+C 中断。所有探测均为 QUERY，不写入 ID；填写 `--current-id` 则只查询该 ID。
查询只能识别该总线上的同协议 ID，不能判断物理关节身份；要确认某一颗电机的当前 ID，应只给它上电。
检测到多个 ID 时会列出全部 ID，提示隔离，返回非零退出码；超时只表示没有收到确认应答。

每次配置需要输入 `yes` 确认隔离电机。服务返回后仍必须人工断电重启，再输入 `yes`，
通过三次新 ID 查询才显示最终 PASS。断电确认仅由操作员提供，程序无法测量电源。
服务 CONFIGURE 在当前 ID 与目标 ID 相同时只查询，不重新写入或保存，但仍要求断电复检。

## 服务语义与异常

```bash
ros2 service call /actuator/motor/calibrate_id \
  luvbot_hardware_interfaces/srv/MotorCalibrateId \
  '{operation: 0, motor_index: 1, current_id: 1}'
```

`operation=0` 为查询，`operation=1` 为配置；目标 ID、协议和总线取自节点配置。
CONFIGURE 必须在标定模式执行。直接调用者同样必须确保总线只上电一颗电机。
LS QUERY 返回该总线所有检测到的 LS ID，CONFIGURE 检测到多个 ID 时拒绝写入。
两个相同 ID 电机不一定能被检测出来，不可用查询代替物理隔离。

`CONFIRMED=0` 只表示此次操作收到匹配协议应答。`power_cycle_required=true` 表示还须验证掉电保存。
其他结果：REJECTED=1、SEND_FAILED=2、DEVICE_FAILED=3、TIMEOUT=4、UNSUPPORTED=5。
`stage` 指出停止阶段：validate、query_current、write_id、query_new、save_id 或 await_power_cycle。
MR 的顺序是确认写入、查询新 ID、通过新 ID 保存；RS/LS 使用协议 ID 设置命令，无额外寄存器保存。
LS 与参考标定工具一致：收到目标 ID 的写入应答后返回 `await_power_cycle`，不在断电前要求广播查询已变为新 ID。
此时旧 ID 仍可能有效；只有人工断电重启后的三次目标 ID 查询全部通过，脚本才显示 PASS。

节点每阶段默认等候 1000 ms，可设置 `motor_id_calibration_timeout_ms`；脚本单次调用默认等候
10 秒，可通过 `--service-timeout` 加长以覆盖节点全部阶段。全局只允许一个 ID 操作在途。
查询会收集整个应答窗口，尤其避免遗漏 LS 广播中的其他电机。

发送失败、写入或保存超时均可能意味着电机已收到命令，**不要自动重试或盲目沿用旧 ID**。
先仅查询旧、新 ID，再检查实际状态。MR 保存结果未知时，新 ID 可用不代表 Flash 已保存；必须断电复检。
节点在各阶段输出日志，脚本在 `log/motor_id_calibration/` 记录请求、返回阶段及操作员断电确认和最终结果。

普通 MIT 帧、发送回环、错误总线/帧类型/长度/寄存器/ID不能作为确认依据。RS 还校验 MCU UID 连续性。
这些协议没有事务编号：相同请求的迟到应答无法完全区分，单次应答不证明持久化成功。
人工断电后连续三次只读查询是最终验收步骤，不包含机械零点或运动反馈验收。

## 离线验证与真机验收

```bash
python3 src/hpm_transport_ros2/scripts/motor/test_motor_id_calibration.py
source install/setup.bash
python3 src/hpm_transport_ros2/src/hpm_system/test/test_motor_id_calibration_runtime.py \
  install/hpm_system/lib/hpm_system/hpm_system_node
```

运行时测试使用 localhost TCP 模拟板卡和独立 ROS domain，覆盖三种协议、两款 MR、应答过滤、
发送失败、各阶段超时、并发拒绝、标定模式保护、真实 service CLI 和交互脚本。
模拟器不能证明真机改 ID 或掉电保存成功。实际部署后分别使用一颗 RS、MR、LS 验证应答和断电后的三次查询。
