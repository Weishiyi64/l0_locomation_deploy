# L0 V5 完整部署工作区

分支：`luvbotics/l0_locomotion_deploy:l0_v5`。更新：2026-11-09。

本分支包含 L0 V5 的 **真机与 MuJoCo 两套 WBC + LoRA 链路、文本动作生成、AMP10281 和运行模型资产**。仓库根目录就是 ROS 工作区；不要只下载或上传 `src/`。

完整代码功能、目录结构和最新资产说明见 [部署源码 README](src/l0_ros2_deploy_v2/README.md)。

## 功能与资产

| 链路 | 默认资产 | 后端 |
| --- | --- | --- |
| 真机普通动作 | `wbc_lora_fp16`，**86 个动作**、共享 Base、每动作 adapter/reference | RKNN / RK3588 |
| 真机文本动作 | 最新文本模型 + `text_wbc_bundle/UpperBody_Enhanced` | 文本 RKNNLite + WBC RKNN |
| MuJoCo 普通动作 | `wbc_lora_sim`，14 个已集成固定测试动作 | OpenVINO CPU / ONNX |
| MuJoCo 停止试验 | `wbc_base_sonic800_73983_20261124_sim`，一个 Base 播放 14 个交付动作和 120 秒随机上肢动作 | OpenVINO CPU / ONNX |
| MuJoCo 文本动作 | 文本 ONNX + `text_wbc_sim/UpperBody_Enhanced` | ONNX Runtime CPU + OpenVINO |

普通动作和文本动作按各自文档分开启动。MuJoCo 不使用 RKNN，真机 86 动作不等同于 86 动作全部已纳入仿真。

停止切换试验分支的 MuJoCo 默认 AMP 为 `l0_v5_amp_1124`。新策略已完成零速站姿采样，补间使用 [实测站姿和同拍目标角](experiments/stop_switch/l0_v5_amp_1124/stand_reference.json)。观察中途收势可播放 `stand_random_upper_120s`，再发送 `soft-stop`。[仿真操作手册](src/l0_ros2_deploy_v2/docs/sim2mujoco-deployment.md#运行中途软停止) · [动态停止试验](experiments/stop_switch/README.md)

## 手柄与终端控制

2026-11-10 新增 AMP/WBC 交接历史初始化修复，默认入场插值 1.0 秒、回 AMP 插值 0.35 秒。
`.23` 工作区已完成板端编译；新交接逻辑的 MuJoCo 与真机动作验收仍待进行。其他工作区
需同步构建更新后的控制接口；详见 [AMP/WBC 交接说明](src/l0_ros2_deploy_v2/docs/amp-wbc-handoff.md)。

普通 WBC 的真机与 MuJoCo 入口默认启用手柄动作请求节点，原终端控制继续可用。LT/RT 配合十字键直接请求以下八个动作：

| 方向 | LT | RT |
| --- | --- | --- |
| 上 | `L0_fear_7_A` | `L0_satisfied_10` |
| 下 | `L0_excited_1` | `L0_SayNo_17` |
| 左 | `L0_excited_new_1` | `L0_angry_9` |
| 右 | `L0_excited_new_5` | `L0_angry_new_1` |

先按原流程进入 Walking/AMP，摇杆回中、站稳后松开所有按钮与十字键，再按组合键。八个动作正常完成后返回 AMP；长按不连发，动作执行期间不排队，完成或断连恢复后需松开再按。原急停、复位和控制器检查继续生效。

加载对应环境后，在各自设备上选择一个入口：

```bash
# 真机：先按现场流程准备 HPM、电机及站立状态
ros2 launch legged_controllers wbc_lora.launch.py hpm_mode:=real joystick_wbc_enabled:=true

# MuJoCo：加载仿真工作区环境后执行
ros2 launch legged_controllers sim2mujoco.launch.py joystick_wbc_enabled:=true
```

使用 `joystick_wbc_enabled:=false` 可关闭新增手柄入口，继续使用终端。文本生成入口默认关闭这八个固定动作绑定。映射配置见 [joystick_wbc_actions.yaml](src/l0_ros2_deploy_v2/legged_controllers/config/joystick_wbc_actions.yaml)，真机环境加载、终端请求和反馈查看见 [手柄与终端操作流程](src/l0_ros2_deploy_v2/docs/joystick-wbc-control.md)。

2026-11-09 已完成 MuJoCo 等价 Joy 消息链路验证，以及 `192.168.120.41` 上 `/userdata/forlinx/wbc_text_workspace1104` 的手柄代码更新、RKNN 后端 9 包编译和安装检查；用户已确认实体手柄及真机动作完成现场验证。

## 获取完整工作区

```bash
git clone --branch l0_v5 --single-branch   git@github.com:luvbotics/l0_locomotion_deploy.git l0_deploy_v5_ws
cd l0_deploy_v5_ws
sha256sum -c TEXT_MODELS_SHA256SUMS
```

本分支的模型作为实际二进制文件随 Git 提交，不需要从训练机另外下载运行资产。需要仓库访问权限；系统 ROS、OpenVINO、MuJoCo、HPM 和编译依赖按目标环境另行准备。

| 必须携带的目录/文件 | 内容 |
| --- | --- |
| `src/` | 部署源码、匹配 ROS/HPM 接口、雷达源码、全部策略及机器人网格 |
| `model_sources/text_to_motion/` | 真机文本 RKNN、素材、词表、RKNNLite2 与私有 Runtime |
| `deploy_config/` | 86 个动作的生产选择与录包配置 |
| `PACKAGE_INFO.json`、`TEXT_MODELS_SHA256SUMS`、`文本模型放置说明.txt` | 文本交付来源与校验资料 |

完整 policy 位于 `src/l0_ros2_deploy_v2/legged_deploy/policy/`，包括 V5 真机、仿真与 AMP 资产。`handoff/` 是分支原有历史交付档案，保留用于追溯，不作为当前启动入口。构建缓存、日志、设备证书/私钥、重复压缩包不在本次运行交付范围内。

## 部署与测试

- [【final1104】真机 WBC 与文本生成启动测试](src/l0_ros2_deploy_v2/docs/【final1104】real-wbc-text-startup-testing.md)：已使用的新工作区为 `/userdata/forlinx/wbc_text_workspace1104`，包含上传、构建、86 动作和文本生成测试步骤。
- [MuJoCo 组成与部署](src/l0_ros2_deploy_v2/docs/sim2mujoco-deployment.md)：CPU 后端、模型、原 14 个 LoRA 动作、15 个单 Base 停止试验动作及文本链路。
- [WBC 手柄与终端并用](src/l0_ros2_deploy_v2/docs/joystick-wbc-control.md)：LT/RT + 十字键 8 个动作，保留终端入口；2026-11-09 已验证 MuJoCo Joy 消息链路，完成板端编译；实体手柄及真机动作已由用户完成现场验证。
- [MuJoCo 测试记录](src/l0_ros2_deploy_v2/docs/sim2mujoco-testing.md)：原有测试结果及模型版本适用范围。

2026-11-04 已完成真机 9 包编译和 3 条文本 NPU 离线生成；用户反馈最终真机操作文档可正常使用。`L0_SayNo_17`、`L0_sad_7`、`L0_Approve_8` 的 reference 首尾静止帧已各保留 50 帧（1 秒），新帧数为 272、431、414，板端转换、逐帧输出验证、重新构建与本地同步均通过。其 `start_hold_sec=1.0`、Base 和 LoRA 保持不变。

模型制作历史手册中的旧路径和旧默认值，以当前源码 README、launch 和对应测试文档为准。
