# 真机部署测试：十条文本动作 + 五条 WBC

**2026-10-29 资产更新：** 五条 `A-*` 动作现已改用 `checkpoint_2000.onnx`，原有 `checkpoint_10000.rknn` 的链接和五条动作的参考 RKNN 文件已从动作目录移除。本篇下面的旧 RKNN 转换、文件检查和真机启动命令不能按原样执行，旧测试结果仅是历史记录；在 RK3588 上部署这五条动作前，必须从新 ONNX 重新转换并验证对应 RKNN。当前先按 [MuJoCo 测试文档](text-wbc-mujoco-test.md) 测试。

更新：2026-10-28。顺序：先完成 [MuJoCo 十五条测试](text-wbc-mujoco-test.md)，再执行本篇。机器人为 L0 v5，AMP 为 `l0_v5_amp_07090`，五条 WBC 位于 `legged_deploy/policy/l0_v5/wbc`；本篇不启动 80 条 LoRA worker。

**真机通过手柄 B、B+Y 进入 AMP。真机没有 MuJoCo 动力学启动步骤，不执行 `/sim/mujoco/start_dynamics`。**

## 0. 笔记本目录与环境准备

**地址更新：机器人为 `forlinx@192.168.5.25`。机器人工作区等待用户提供新路径；下文 `/userdata/forlinx/text_workspace` 暂为此前路径，涉及创建、上传、编译和启动的命令待路径确认后再执行。**

本篇的“笔记本”是 x86_64 Linux，“板端”是机器人 RK3588。文本在笔记本生成，完整 JSON 经 ROS 2 发给机器人执行；WBC 的 RKNN 在机器人上运行。机器人已有 HPM 环境，本篇不提供 HPM 安装或启动步骤。

| 项目 | 本次路径/地址 |
| --- | --- |
| 笔记本工作区 | `/home/luvbot/Downloads/l0_deploy_v2_ws` |
| 笔记本代码 | `/home/luvbot/Downloads/l0_deploy_v2_ws/src/l0_ros2_deploy_v2` |
| 机器人 SSH | `forlinx@192.168.5.25` |
| 机器人新工作区 | `/userdata/forlinx/text_workspace` |
| 上传后的代码 | `/userdata/forlinx/text_workspace/src/l0_ros2_deploy_v2` |
| 板端编译结果 | `/userdata/forlinx/text_workspace/install` |

将整个 `src` 放进笔记本工作区，避免变成 `src/src`。笔记本源码与板端源码保留相同相对结构；板端工作区直接是 `text_workspace`，不再嵌套 `l0_deploy_v2_ws`。

笔记本需准备 ROS 2 Humble 和文本推理依赖。复制源码不会安装这些环境，也不能复用旧电脑的 `build/install`。**唯一尚未确认的本机路径是 Python 环境**：下面按 `/home/luvbot/miniconda3/envs/unitree_rl_mjlab/bin/python` 写出；如果笔记本没有该环境，必须先准备环境并把 `TEXT_MOTION_PYTHON` 指向实际 Python。笔记本工作区已确定，SSH 地址已更新，机器人工作区等待确认。

下面只设置当前终端变量并检查环境，不会安装依赖、连接电机或发送动作。`export` 指定路径，`test` 检查文件，`file` 查看库架构，Python `import` 检查依赖是否可用。Python 路径只是待核实的示例，不要求必须使用名为 `unitree_rl_mjlab` 的 Conda 环境。

每个新的笔记本终端先执行：

```bash
export local_ws=/home/luvbot/Downloads/l0_deploy_v2_ws
export local_repo="$local_ws/src/l0_ros2_deploy_v2"
export TEXT_MOTION_PYTHON=/home/luvbot/miniconda3/envs/unitree_rl_mjlab/bin/python
export robot_target=forlinx@192.168.5.25
export robot_ws=/userdata/forlinx/text_workspace
cd /home/luvbot/Downloads/l0_deploy_v2_ws/src/l0_ros2_deploy_v2
test -x "$TEXT_MOTION_PYTHON"
test -s texttomotion/run.onnx
file texttomotion/libluvbot_onnx_text.so.1
"$TEXT_MOTION_PYTHON" -c 'import numpy, onnx, onnxruntime, mujoco; print("文本推理依赖可导入")'
```

若 Python 不存在或导入失败，先准备相应环境，不能仅修改路径便视为安装完成。`run_offline.sh --live` 的姿态采集、发布使用 `/usr/bin/python3`，因此还需要系统 ROS Python 和项目消息接口；文本推理才使用 `TEXT_MOTION_PYTHON`。参考窗口还需要可用图形桌面。

首次迁移建议保留整个工作区 `src`（包括匹配的 `luvbot_ros_interfaces` 和描述资源），保留模型与相对符号链接。笔记本仅生成/发送文本时，可先编译消息接口；若还要运行主 MuJoCo，则按仿真文档构建 OPENVINO 后端。机器人按本篇第 5 节构建 RKNN 后端。笔记本接口构建和检查（首次迁移或接口源码更新后执行，不必每次测试都编译；每个新终端仍需执行两条 `source`）：

```bash
source /opt/ros/humble/setup.bash
cd /home/luvbot/Downloads/l0_deploy_v2_ws
colcon build --symlink-install --packages-up-to luvbot_control_interfaces
source /home/luvbot/Downloads/l0_deploy_v2_ws/install/setup.bash
/usr/bin/python3 -c 'import rclpy; import luvbot_control_interfaces.msg; print("ROS 接口可导入")'
```

## 1. 哪些文件需要转 RKNN

| 部分 | 转换与执行方式 |
| --- | --- |
| AMP | 使用现有 `l0_v5_amp_07090/policy.rknn`，本次不替换 AMP 策略 |
| 五条直接 WBC | 共用 `checkpoint_10000.rknn`；每条还需要独立 `reference_motion.rknn`，共 6 个 RKNN |
| 文本生成 | x86 主机用 `run_fp16.onnx` + `libluvbot_onnx_text.so.1` 生成完整 JSON；RK3588 控制器播放 JSON，无需把每条 JSON 转 RKNN |

现有文本模型包含自定义分词、字符串和动态分支，Toolkit2 2.3.2 实测 build 失败于 `KeyError: 'mode'`，现有分词库是 x86-64。**目前不是纯板端文本生成方案**；若要求脱离 x86 主机，需要另外提供可在 aarch64 使用的算子/模型，或重新导出 RKNN 支持的模型。不要把 ONNX 改名成 RKNN。

FP16 指模型权重、LoRA 参数和参考轨迹表；外部浮点 ABI 保持 FP32，索引、字符串、物理限位和控制计算保持必要类型。转换器设置 `float_dtype=float16`、`do_quantization=False`。

## 2. 在 x86 主机准备文本模型

已有文件正常时不必重复导出。原始模型更新后执行：

```bash
cd /home/luvbot/Downloads/l0_deploy_v2_ws/src/l0_ros2_deploy_v2
export TEXT_MOTION_PYTHON=/home/luvbot/miniconda3/envs/unitree_rl_mjlab/bin/python
bash texttomotion/run_offline.sh --model texttomotion/run.onnx \
  --export-fp16 texttomotion/run_fp16.onnx
file texttomotion/libluvbot_onnx_text.so.1
```

原始 `run.onnx` 不覆盖。后续 `--live` 默认使用 `run_fp16.onnx`，每条重新采集当前姿态作为 start/end pose。文本模型、分词库及 Python 依赖留在 x86 主机。

## 3. WBC 怎么转 RKNN（已有文件可以跳过重新转换）

**就是使用你指出的两个现成脚本，没有另写一套转换程序：**

- `convert_onnx_to_rknn_rk3588_wbc_policy.py`：把共享控制策略 `checkpoint_10000.onnx` 转成 `checkpoint_10000.rknn`，读取观测并输出控制动作。
- `convert_onnx_to_rknn_rk3588_wbc_reference.py`：把每条动作的 `reference_motion.onnx` 转成同目录的 `reference_motion.rknn`，提供该动作的参考数据。

**没有连接机器人，也能生成 RKNN。** Toolkit2 可以在开发电脑上离线编译面向 RK3588 的模型；只有到板端加载模型、测 NPU 时延、实际执行动作，才需要机器人。这与在电脑上编译一个随后拷贝到板端运行的程序类似。

当前目录里的 RKNN 是此前调试阶段已经转换并保存的产物，并非你刚连接或启动了机器人才生成。已核对其旁边 `.rknn.json`：`precision=float16`、`toolkit_version=2.3.2`、`verification_backend=toolkit_simulator`、`validation_status=python_verified`。这表示做过 Toolkit 模拟器检查，**不表示已经在你的机器人上验证通过**。

五条动作实际上是 **1 个共享策略 + 5 个独立参考 = 6 个 RKNN 文件**。每个动作目录的 `policy.rknn` 是指向上一级 `checkpoint_10000.rknn` 的链接，不是五个不同策略。它们属于本篇的五条直接 WBC，不是 80 条 LoRA。

| 你现在要做什么 | 是否需要重新转换 |
| --- | --- |
| 解压后使用当前五条 WBC，模型未变、文件完整 | 不需要；执行本节末尾文件检查后进入第 4 节 |
| 只是换笔记本目录或 SSH 地址 | 不需要，RKNN 不绑定这些路径 |
| 更新 checkpoint 或某条 reference ONNX | 重转对应策略或参考，并同步更新后的资产 |
| 更新目标芯片、转换参数或需要匹配不同 Toolkit/Runtime | 重新核对兼容性，按目标环境重转并验收 |

以下六条命令是需要重新转换时的操作说明；不会启动电机。请在独立资产副本或控制程序停止时转换，避免替换正在使用的文件。

### 3.1 是否需要准备新的 Python 环境

**当前文件完整且模型未更新：跳过 3.1、3.2，不安装 Toolkit，不重复转换。**

你之前在机器人执行 `python3 转换脚本 ONNX文件`，说明该机器的 `python3` 已有转换依赖，不需要再建 venv。`RKNN_PYTHON` 只是文档里用于选择 Python 的变量，不是额外的软件，更不是运行机器人的必要配置。

如果需要在机器人重转，先完成第 4 节上传，再 SSH 登录机器人检查已有环境：

```bash
ssh forlinx@192.168.5.25
cd /userdata/forlinx/text_workspace/src/l0_ros2_deploy_v2
python3 -c 'from rknn.api import RKNN; import onnx,numpy; from importlib.metadata import version; print("Toolkit2:",version("rknn-toolkit2"),"ONNX:",onnx.__version__,"NumPy:",numpy.__version__)'
```

本批文件的转换环境是 Toolkit2 2.3.2、ONNX 1.16.2、NumPy 1.26.4；上面的检查只读版本，不安装或更改机器人环境。导入失败或版本不同，应先核对现场环境，不直接升级原有机器人依赖。已有兼容环境时，下面直接用原来的 `python3`。

只有选择在一台**没有 Toolkit 的笔记本上重新转换**时，才需要独立环境；这与文本推理环境不同。可选命令（在笔记本执行，正常使用已有 RKNN 时跳过）：

```bash
python3.10 -m venv /home/luvbot/.venvs/l0-rknn232
/home/luvbot/.venvs/l0-rknn232/bin/python -m pip install \
  'rknn-toolkit2==2.3.2' 'onnx==1.16.2' 'numpy==1.26.4' 'protobuf==4.25.4'
source /home/luvbot/.venvs/l0-rknn232/bin/activate
cd /home/luvbot/Downloads/l0_deploy_v2_ws/src/l0_ros2_deploy_v2
```

激活后 `python3` 就是该环境的 Python，下面六条命令也可在笔记本执行。不要复制开发时的 `/tmp/l0_rknn_toolkit`；不使用它也不影响已经转换好的 RKNN 运行。

### 3.2 用原有脚本转换共享策略与五条参考（可选）

在 3.1 中选定的终端、仓库目录执行。`--build-only` 表示只生成文件，不连接机器人 NPU 做 Python 推理；这是与原来直接运行命令相比显式增加的选项。输出写在输入 ONNX 同目录，分别为 `checkpoint_10000.rknn` 和 `reference_motion.rknn`。

```bash
python3 -B convert_onnx_to_rknn_rk3588_wbc_policy.py \
  legged_deploy/policy/l0_v5/wbc/checkpoint_10000.onnx --build-only
```

```bash
python3 -B convert_onnx_to_rknn_rk3588_wbc_reference.py \
  legged_deploy/policy/l0_v5/wbc/A-C04_mod/reference_motion.onnx --build-only
```

```bash
python3 -B convert_onnx_to_rknn_rk3588_wbc_reference.py \
  legged_deploy/policy/l0_v5/wbc/A-C07_mod/reference_motion.onnx --build-only
```

```bash
python3 -B convert_onnx_to_rknn_rk3588_wbc_reference.py \
  legged_deploy/policy/l0_v5/wbc/A-S04_mod/reference_motion.onnx --build-only
```

```bash
python3 -B convert_onnx_to_rknn_rk3588_wbc_reference.py \
  legged_deploy/policy/l0_v5/wbc/A-C08_mod/reference_motion.onnx --build-only
```

```bash
python3 -B convert_onnx_to_rknn_rk3588_wbc_reference.py \
  legged_deploy/policy/l0_v5/wbc/A-T08-02_mod/reference_motion.onnx --build-only
```

`--build-only` 只编译，不启动 Python NPU Runtime；不要与 `--target`/`--device-id` 同用。每个 `.rknn` 旁有转换记录 `.rknn.json`，`converted_pending_runtime` 是 build-only 的正常状态，不是实测运行通过。当前开发版另做过模拟器数值对比，但不等于板端实时验收。

确认共享链接和输出（循环只用于文件检查，不会自动发送动作）：

```bash
wbc=legged_deploy/policy/l0_v5/wbc
test -s "$wbc/checkpoint_10000.rknn"
for action in A-C04_mod A-C07_mod A-S04_mod A-C08_mod A-T08-02_mod; do
  test -s "$wbc/$action/policy.rknn" || exit 1
  test -s "$wbc/$action/reference_motion.rknn" || exit 1
  test -s "$wbc/$action/rl_config.yaml" || exit 1
  readlink -f "$wbc/$action/policy.rknn"
done
```

五个 `policy.rknn` 都应解析到同一个 `wbc/checkpoint_10000.rknn`。不要只复制动作子文件夹，必须复制整个 wbc 目录及相对链接。

## 4. 新建机器人工作区并上传整个 src（在笔记本执行）

### 4.1 检查笔记本目录

```bash
ls /home/luvbot/Downloads/l0_deploy_v2_ws/src
test -d /home/luvbot/Downloads/l0_deploy_v2_ws/src/l0_ros2_deploy_v2
test -d /home/luvbot/Downloads/l0_deploy_v2_ws/src/luvbot_ros_interfaces
ssh forlinx@192.168.5.25 'whoami; ls -ld /userdata/forlinx'
```

应有 `l0_ros2_deploy_v2`、`luvbot_ros_interfaces` 及工作区其他依赖源码。SSH 按提示输入机器人密码。笔记本、机器人都需有 `rsync` 命令。

### 4.2 在机器人上创建新工作区

```bash
ssh forlinx@192.168.5.25 'mkdir -p /userdata/forlinx/text_workspace/src /userdata/forlinx/text_workspace/deploy_config'
```

只创建目录，不启动任何控制程序，不修改原有工作区。首次不需要 `colcon init`；上传源码后执行 `colcon build` 会生成 `build/install/log`。

### 4.3 上传源码和模型

```bash
rsync -av --progress --backup --backup-dir="/userdata/forlinx/text_workspace/upload_backups/$(date +%Y%m%d_%H%M%S)" \
  --exclude='.git' --exclude='__pycache__' \
  --exclude='build/' --exclude='install/' --exclude='log/' \
  /home/luvbot/Downloads/l0_deploy_v2_ws/src/ \
  forlinx@192.168.5.25:/userdata/forlinx/text_workspace/src/
```

源目录末尾的 `/src/` 表示复制其**内容**，最终是 `text_workspace/src/l0_ros2_deploy_v2`，不会多出 `src/src`。上传整个 src 保留依赖、描述模型、RKNN 和相对链接；不上传笔记本编译产物。没有 `--delete`，不删除远端额外文件；重复上传覆盖的旧文件会保存在 `upload_backups`。更新时先停止这个工作区的控制程序。

新工作区首次上传包括仓库配置，不再排除 `system_command_arbiter/config/`，否则新目录缺少配置。**上传的仓库配置仍需与这台机器已确认的关节映射/校准核对；不能因目录创建成功就认为校准已匹配。** 若机器人有独立的现场校准版本，应在新工作区沿用该版本，再构建启动。此步骤不改 HPM 配置。

### 4.4 登录板端，检查上传结果

```bash
ssh forlinx@192.168.5.25
cd /userdata/forlinx/text_workspace
ls src
test -f src/l0_ros2_deploy_v2/legged_controllers/launch/sim2real.launch.py
test -s src/l0_ros2_deploy_v2/legged_deploy/policy/l0_v5/wbc/checkpoint_10000.rknn
readlink -f src/l0_ros2_deploy_v2/legged_deploy/policy/l0_v5/l0_v5_amp_07090/policy.rknn
```

AMP 当前目录链接到 `../l0_v4/l0_v3_amp_07090`，整个 src 上传会保留链接目标，不要只上传 `l0_v5` 子目录。若链接原本指向旧电脑的绝对路径，需先修复为有效目标，不能使用断开的链接启动。以下第 5～7 节均在机器人 SSH 终端执行；第 8～9 节回到笔记本。

## 5. 板端编译与文件检查（板端终端）

```bash
robot_ws=/userdata/forlinx/text_workspace
robot_repo="$robot_ws/src/l0_ros2_deploy_v2"
source /opt/ros/humble/setup.bash
cd "$robot_ws"
colcon build --symlink-install \
  --packages-up-to legged_deploy legged_real legged_controllers \
  --cmake-args -DLEGGED_DEPLOY_BACKEND=RKNN \
  -DRKNN_ROOT="$robot_repo/legged_deploy/third_party/rknn_runtime"
source install/setup.bash
cat install/legged_deploy/share/legged_deploy/inference_backend.txt
ldd install/legged_deploy/lib/legged_deploy/rl_controller
readlink -f "$robot_repo/legged_deploy/policy/l0_v5/l0_v5_amp_07090/policy.rknn"
```

必须显示 RKNN，`ldd` 不得有 `not found`。仓库 RKNN_ROOT 包含头文件和 aarch64 `librknnrt.so`；板端 NPU 驱动/运行时兼容性按已有设备配置确认，不能通过跳过检查解决。AMP 从编译时源码目录读取模型，WBC 从安装目录读资产，因此保留源码路径，更新后重新构建/source。

## 6. 配置与 HPM 一致的 ROS 环境（板端，只做一次）

沿用机器人现场已有的 ROS_DOMAIN_ID、RMW 和 DDS 设置，不改 HPM 环境。将实际值填入下面配置，供部署终端和笔记本使用。下面保存专用于本测试的配置，不覆盖既有 production.env：

```bash
(
set -e
robot_ws=/userdata/forlinx/text_workspace
mkdir -p "$robot_ws/deploy_config"
config="$robot_ws/deploy_config/text-wbc-test.env"
if test -e "$config"; then cat "$config"; exit 0; fi
read -r -p 'HPM 实际 ROS_DOMAIN_ID（0..232）: ' domain
[[ "$domain" =~ ^[0-9]{1,3}$ ]] && (( 10#$domain <= 232 ))
read -r -p 'HPM 实际 ROS_LOCALHOST_ONLY（0 或 1）: ' localhost
[[ "$localhost" == 0 || "$localhost" == 1 ]]
read -r -p 'HPM 的 RMW_IMPLEMENTATION（系统默认则留空）: ' rmw
[[ -z "$rmw" || "$rmw" =~ ^rmw_[a-zA-Z0-9_]+$ ]]
(
set -C
{
  printf 'export ROS_DOMAIN_ID=%q\n' "$((10#$domain))"
  printf 'export ROS_LOCALHOST_ONLY=%q\n' "$localhost"
  if test -n "$rmw"; then printf 'export RMW_IMPLEMENTATION=%q\n' "$rmw"; else printf 'unset RMW_IMPLEMENTATION\n'; fi
} > "$config"
)
cat "$config"
)
```

x86 主机需要跨网络读取真实关节姿态并发布文本，要求两侧 `ROS_LOCALHOST_ONLY=0`。若现有机器人配置仅限本机通信，笔记本无法连接，须先确认现场跨机通信配置；只改笔记本无效。DDS 自定义 profile 也要一致，并保证两台机器时间同步，否则姿态新鲜度/动作时间戳校验会拒绝。

## 7. 启动与进入 AMP（板端）

前提：机器人已有环境按现场方式就绪。以下只启动本部署程序，不包含 HPM 启动操作。

### 终端 R2：部署程序（保持运行）

```bash
source /opt/ros/humble/setup.bash
source /userdata/forlinx/text_workspace/install/setup.bash
source /userdata/forlinx/text_workspace/deploy_config/text-wbc-test.env
ros2 launch legged_controllers sim2real.launch.py \
  robot_type:=l0_v5 subpolicy:=l0_v5_amp_07090 hpm_mode:=real \
  wbc_enabled:=true wbc_mode:=static \
  'wbc_policies:=[A-C04_mod,A-C07_mod,A-S04_mod,A-C08_mod,A-T08-02_mod]' \
  joystick_sequence_enabled:=false \
  joystick_back_emergency_sequence_enabled:=false rviz:=false
```

等全部电机就绪、AMP 持续有效命令、`WBC worker ready` 后，使用手柄 **B → B+Y** 进入 AMP Walking，保持零速度站稳。确认 `FSM: Standby -> Walking`。不要反复按 B 盲目增加增益；后续每条动作前无需重新进入 AMP。真机不调用任何 MuJoCo 服务。

### 终端 R3：状态检查和 WBC 回执

```bash
source /opt/ros/humble/setup.bash
source /userdata/forlinx/text_workspace/install/setup.bash
source /userdata/forlinx/text_workspace/deploy_config/text-wbc-test.env
ros2 topic echo /hardware_status/motors_ready std_msgs/msg/Bool --once
ros2 service type /control/policy/wbc/prepare
ros2 topic echo /system/state_machine/wbc_action_fdb std_msgs/msg/String
```

就绪为 true，prepare 类型应为 `luvbot_control_interfaces/srv/ControlWbcMotion`。保持最后一条回执监听运行。持续的策略命令过期、硬件故障或反馈中断都需要先排查。

## 8. 文本生成主机准备（x86 终端 P1）

先将已确认的环境配置复制到主机，检查内容后 source；自定义 DDS 文件路径若只在板端存在，改为主机对应路径。主机 source 自己的工作区，不能 source 板端 install。

```bash
scp forlinx@192.168.5.25:/userdata/forlinx/text_workspace/deploy_config/text-wbc-test.env /tmp/l0-real-test.env
cat /tmp/l0-real-test.env
source /opt/ros/humble/setup.bash
source /home/luvbot/Downloads/l0_deploy_v2_ws/install/setup.bash
source /tmp/l0-real-test.env
cd /home/luvbot/Downloads/l0_deploy_v2_ws/src/l0_ros2_deploy_v2
export TEXT_MOTION_PYTHON=/home/luvbot/miniconda3/envs/unitree_rl_mjlab/bin/python
ros2 topic info /control/policy/observation
ros2 topic info /text_motion/json
ros2 topic info /text_motion/status
```

应看到 observation 的发布者、text_motion/json 的订阅者和 status 的发布者。先结束之前的 MuJoCo 测试，不要让仿真与真机共用同一 ROS 域。网络不可见时先修复发现问题，不能绕过姿态采集使用不匹配的旧 JSON。

可选终端 P2：常驻参考窗口，仍需相同 ROS 环境：

```bash
source /opt/ros/humble/setup.bash
source /home/luvbot/Downloads/l0_deploy_v2_ws/install/setup.bash
source /tmp/l0-real-test.env
cd /home/luvbot/Downloads/l0_deploy_v2_ws/src/l0_ros2_deploy_v2
export TEXT_MOTION_PYTHON=/home/luvbot/miniconda3/envs/unitree_rl_mjlab/bin/python
bash texttomotion/run_offline.sh --preview
```

这是运动学参考窗口，不连接真实电机；实际动作直接看机器人。它会自动切换十条文本，不需重开。

## 9. 十条文本动作（x86 终端 P1，逐条运行）

每条 `--live` 自动读取 `/control/policy/observation` 中已校准、按关节名匹配的新鲜姿态，作为 start/end pose；生成、裁剪并验证后一次发送完整 JSON。板端完整接收、后台验证后才开始，控制线程不阻塞等待推理。等 `received → ready → started → completed`，确认站稳，再手工执行下一条。

1. 头以大幅度慢速点头两次。

```bash
bash texttomotion/run_offline.sh --live --text '头以大幅度慢速点头两次。' --output /tmp/l0_real_text_01.json
```

2. 头以小幅度快速左右倾斜三次。

```bash
bash texttomotion/run_offline.sh --live --text '头以小幅度快速左右倾斜三次。' --output /tmp/l0_real_text_02.json
```

3. 头以中幅度慢速向左转。

```bash
bash texttomotion/run_offline.sh --live --text '头以中幅度慢速向左转。' --output /tmp/l0_real_text_03.json
```

4. 左手以大幅度抬起。

```bash
bash texttomotion/run_offline.sh --live --text '左手以大幅度抬起。' --output /tmp/l0_real_text_04.json
```

5. 右手以小幅度慢速前后摆动两次。

```bash
bash texttomotion/run_offline.sh --live --text '右手以小幅度慢速前后摆动两次。' --output /tmp/l0_real_text_05.json
```

6. 左手摸肚子两次。

```bash
bash texttomotion/run_offline.sh --live --text '左手摸肚子两次。' --output /tmp/l0_real_text_06.json
```

7. 头点头；右手放后面。

```bash
bash texttomotion/run_offline.sh --live --text '头点头；右手放后面。' --output /tmp/l0_real_text_07.json
```

8. 头以小幅度慢速点头；左手以大幅度抬起。

```bash
bash texttomotion/run_offline.sh --live --text '头以小幅度慢速点头；左手以大幅度抬起。' --output /tmp/l0_real_text_08.json
```

9. 左手以小幅度前后摆动两次；右手放后面。

```bash
bash texttomotion/run_offline.sh --live --text '左手以小幅度前后摆动两次；右手放后面。' --output /tmp/l0_real_text_09.json
```

10. 头以大幅度慢速点头两次；左手抬起；右手摸肚子两次。

```bash
bash texttomotion/run_offline.sh --live --text '头以大幅度慢速点头两次；左手抬起；右手摸肚子两次。' --output /tmp/l0_real_text_10.json
```


生成器会报告限位裁剪；JSON 中保留逐关节原始/发送范围。`rejected`、`stopped`、超时或未站稳时不继续发送。仿真文本手臂 Kp≥16/Kd≥0.4 的修复仅用于仿真，真机仍使用原实机增益，需以实际反馈验收跟随质量。

## 10. 五条 WBC（板端终端 R4，文本结束后）

```bash
source /opt/ros/humble/setup.bash
source /userdata/forlinx/text_workspace/install/setup.bash
source /userdata/forlinx/text_workspace/deploy_config/text-wbc-test.env
```

R3 持续查看回执。每条使用新的当前时间戳，等待对应 action_id 的 `started → completed`、AMP 恢复且机器人站稳，再执行下一条。不要同时发文本动作。

1. `A-C04_mod`

```bash
ros2 topic pub --once /algorithm/brain/wbc_action_cmd std_msgs/msg/String \
  "{data: '{\"timestamp_ms\": $(date +%s%3N), \"action_id\": \"A-C04_mod\"}'}"
```

2. `A-C07_mod`

```bash
ros2 topic pub --once /algorithm/brain/wbc_action_cmd std_msgs/msg/String \
  "{data: '{\"timestamp_ms\": $(date +%s%3N), \"action_id\": \"A-C07_mod\"}'}"
```

3. `A-S04_mod`

```bash
ros2 topic pub --once /algorithm/brain/wbc_action_cmd std_msgs/msg/String \
  "{data: '{\"timestamp_ms\": $(date +%s%3N), \"action_id\": \"A-S04_mod\"}'}"
```

4. `A-C08_mod`

```bash
ros2 topic pub --once /algorithm/brain/wbc_action_cmd std_msgs/msg/String \
  "{data: '{\"timestamp_ms\": $(date +%s%3N), \"action_id\": \"A-C08_mod\"}'}"
```

5. `A-T08-02_mod`

```bash
ros2 topic pub --once /algorithm/brain/wbc_action_cmd std_msgs/msg/String \
  "{data: '{\"timestamp_ms\": $(date +%s%3N), \"action_id\": \"A-T08-02_mod\"}'}"
```


WBC 参考下半身填充的是此前采集的 AMP 站姿；运行时腿部仍由 WBC 做平衡控制，不是锁死电机。启动拒绝、推理超时或动作失败时先看 R2/R3 日志，不取消安全校验。

## 11. 验收、排障与停机

每条记录：动作描述/ID、是否完成、方向次数是否符合要求、是否裁剪、是否稳定回到 AMP。源码构建、RKNN 编译成功不代表真机动作已通过。

| 问题 | 检查 |
| --- | --- |
| 文本模型在 RK3588 报错 | 当前分词库为 x86-64；按本篇在 x86 主机生成，不在板端加载该库 |
| --live 采集超时 | 是否进入 AMP、同一 ROS 域/网络、时间同步、observation 是否新鲜；不能使用 /sim 话题 |
| WBC prepare 不存在 | worker 是否启动失败、资产是否位于安装目录、RKNN 依赖是否齐全 |
| 动作 rejected | 是否处于 Walking、另一动作是否占用控制权、时间戳是否过期、动作是否在预加载列表 |
| 手臂幅度小 | 比较参考/下发/反馈与实机增益，不能直接照搬仿真增益或放宽机械限位 |
| 停机 disable 未确认 | 保持 HPM 在线并检查真实电机状态，不能把进程退出当作 disable 成功 |

停机顺序：

1. 停止发送新动作，等待当前动作结束并返回 AMP；按现场手柄停机流程退出 Walking，需要急停时使用原手柄急停功能。
2. 在 R2 按一次 Ctrl+C，等待部署进程组清理；此时已有硬件通信服务保持在线，以接收硬件桥 disable。
3. 确认 disable 送达、实际电机已禁用，再关闭回执监听与主机参考窗口。现场已有服务按原有管理方式处理，本篇不操作 HPM。

本文提供操作流程，没有在本轮文档编写中启动真机。已知开发验证范围为本机 MuJoCo、RKNN Toolkit 模拟器和资产完整性；板端时延、真实 HPM 通信、实际电机跟随与停机仍需要按本篇验收。LoRA 80 动作的独立流程见 [LoRA 正式部署文档](wbc-lora-production-deployment.md)。
