# WBC deploy

独立的 MimicLite L0 V2 部署打包与 MuJoCo sim2sim 工具。从 active-adaptation 的 `projects/mimic-lite/scripts/deploy` 提取，不需要安装训练工程。

## 目录

- `policy_onnx_cfg/`：原始 WBC policy ONNX 和同名 YAML，附带 o47xcbco checkpoint 10000 的导出结果。
- `policy_chckpoint_cfg/`：预留 checkpoint 存放目录（沿用指定拼写），当前为空。checkpoint → ONNX 仍在训练工程导出，再将 ONNX 和同名 YAML 放入 `policy_onnx_cfg/`。
- `motion_onnx_cfg/`：从原工程复制的 motion ONNX；不包含 TensorRT 缓存。
- `assets/l0_v2/`：sim2sim 和 motion 导出所需 XML、mesh。
- `scripts/deploy/`：部署打包、motion 导出、原始/部署版 sim2sim。
- `deploy_bundles/`：生成的部署包，每包只有 `policy.onnx`、`motion.onnx`、`rl_config.yaml`。
- `sim2sim_save/`：录像输出目录（运行时生成）。

## 环境

使用 Python 3.12，在本仓库根目录执行：

```bash
python3.12 -m venv .venv
.venv/bin/python -m pip install -r requirements.txt
```

requirements 固定为迁移验证所使用的环境版本。脚本优先使用 `WBC_DEPLOY_PYTHON` 指定的解释器，其次兼容 `MIMIC_LITE_PYTHON`，默认使用本仓库 `.venv/bin/python`，不存在时使用 PATH 中的 `python3`。也可以指向已装好依赖的现有环境。

## 打包部署

以下命令都在本仓库根目录执行：

```bash
bash scripts/deploy/prepare_monolithic_deploy.sh \
  policy_onnx_cfg/policy-o47xcbco.onnx \
  motion_onnx_cfg/L0_yoga_24fps_direct_ocp.onnx \
  deploy_bundles/L0_yoga_24fps_direct_ocp_o47xcbco_10000
```

输入 policy 必须有同名 YAML。打包会校验 ONNX 和输入输出接口；临时 adapter 自动清理。请为不同 policy/motion 组合使用独立输出目录。

最终 `policy.onnx` 接受 `obs[1,560]`，输出 `actions[1,21]`，包含 WBC 网络、观测适配和该动作的参考数据。最终 `motion.onnx` 是控制器使用的 root 位姿/时间步接口，其中 `contact_mask` 字段承载时间步。**更换动作必须重新打包整套，不能只替换最终 motion.onnx。**

默认 policy 50 Hz、底层控制器 500 Hz；底层频率可通过 `MIMIC_LITE_LOWLEVEL_FREQUENCY` 指定。sim2sim 的物理步长单独配置。

在目标 Jetson 上转换（进入对应部署包目录）：

```bash
trtexec --onnx=policy.onnx --saveEngine=policy.trt --fp16
trtexec --onnx=motion.onnx --saveEngine=motion.trt --fp16
```

## sim2sim：原始 policy

使用原始 policy ONNX、同名 YAML 和完整 motion ONNX：

```bash
bash scripts/deploy/sim2sim.sh native \
  --policy policy_onnx_cfg/policy-o47xcbco.onnx \
  --config policy_onnx_cfg/policy-o47xcbco.yaml \
  --motion motion_onnx_cfg/L0_yoga_24fps_direct_ocp.onnx \
  --duration 12.5 --no-randomization
```

默认打开 MuJoCo viewer；无界面快速运行加 `--headless --no-realtime`。录制参数见 `bash scripts/deploy/sim2sim.sh native --help`。

## 打头顶 ONNX + 原始双输入 WBC：本机 MuJoCo

原始 `/home/luvbotic/下载/jianyan/json/打头顶.onnx` 只有 `time_seconds -> joint_positions[9]`。以下入口会在临时目录把它补为完整参考：9 个关节保留原始值，未提供的 12 个腿部关节参考保持 0，root 保持站姿；仿真结束后自动删除临时 ONNX。不修改 `checkpoint_10000.onnx` 或源 motion，不录制视频。

```bash
cd /home/luvbotic/wbc_deploy
/home/luvbotic/miniconda3/envs/unitree_rl_mjlab/bin/python \
  scripts/deploy/run_upper_body_wbc_mujoco.py \
  --policy /home/luvbotic/下载/jianyan/policy/checkpoint_10000.onnx \
  --config /home/luvbotic/下载/jianyan/policy/checkpoint_10000.yaml \
  --motion /home/luvbotic/下载/jianyan/json/打头顶.onnx \
  --debug-summary
```

默认打开 MuJoCo 窗口，按原轨迹运行 6.74 秒；无界面验证加 `--headless --no-realtime`。2026-09-24 无界面跑满 337 帧，最大关节参考误差约 0.120 rad，最大机身倾斜约 1.07°。源文件的肩/肘关节几乎静止，主要是颈部运动；颈部 pitch 有 68 帧超出 L0 V2 模型的关节上限。这个命令只用于本机仿真；真机 ROS2 控制器目前不支持原始 checkpoint 的双输入接口。

`complete_upper_body_motion_onnx.py` 是临时补全所用的通用工具。若将来必须确认源动作包含明显手臂运动，可单独运行该工具并加 `--require-arm-motion` 检查。

## sim2sim：部署版 policy

先打包，再运行：

```bash
bash scripts/deploy/sim2sim.sh deploy \
  --policy deploy_bundles/L0_yoga_24fps_direct_ocp_o47xcbco_10000/policy.onnx \
  --config deploy_bundles/L0_yoga_24fps_direct_ocp_o47xcbco_10000/rl_config.yaml \
  --reference-motion motion_onnx_cfg/L0_yoga_24fps_direct_ocp.onnx \
  --duration 12.5 --no-headless --no-randomization
```

`--reference-motion` 必须是与打包一致的**原始完整 motion ONNX**，用于初始化与跟踪误差计算，不能使用部署包里的精简版 motion.onnx。此模式验证部署 policy 的 MuJoCo 执行，不加载真实 ROS2 控制器或 TensorRT 引擎。

无界面快速验证使用 `--headless --no-realtime`；录像加 `--record`。服务器无显示器录像可设置 `MUJOCO_GL=egl`。支持 `--debug-csv records/run.csv`、`--physics-dt`、动作延迟和随机化等参数，详见 `bash scripts/deploy/sim2sim.sh deploy --help`。日志、CSV 和视频放在部署包以外。

## 从动作文件导出 motion

已有 motion ONNX 可直接使用；新增 NPZ 或可信本地 GMR PKL 时：

```bash
.venv/bin/python scripts/deploy/export_motion_to_onnx.py \
  --motion-file /path/to/motion.npz
```

默认输出 `motion_onnx_cfg/<动作文件名>.onnx`，其余选项见 `--help`。

## 范围

本仓库只包含 L0 V2 当前接口对应的部署与 sim2sim 流程、输入资产；没有训练、数据预处理工程、历史实验、远程启动脚本或 checkpoint 导出器。没有复制历史部署包，附带 yoga 部署包由本仓库脚本重新生成。生成包、checkpoint、录像、日志和 TensorRT 引擎默认不进入 Git。

## 迁移验证（2026-09-24）

使用 Python 3.12.13 和 requirements.txt 中的依赖版本，复用本机已有解释器进行验证，未创建新虚拟环境。仓库脚本未写死该解释器路径。

- 上述 yoga 打包命令成功，两个产物均通过 ONNX checker 与 ONNX Runtime 接口校验。
- 上述 native 与 deploy sim2sim 命令将时长改成 `--duration 1`，使用 `--headless --no-realtime --no-randomization`，均正常退出；部署版输出 `SIM2SIM_PASS`，完成 50 个控制步。
- 修复原 native runner 对 `command_short` 输入名的兼容性，同时保留 `command` 支持。
- 验证日志保存在本地 `records/`。此次为迁移冒烟验证，未验证完整动作、GUI、录像或目标 Jetson 执行。
