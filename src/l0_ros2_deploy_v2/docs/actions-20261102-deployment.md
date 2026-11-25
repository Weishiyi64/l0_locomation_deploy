# 2026-11-02 LoRA 动作集成与板端验收

`wbc_lora_fp16` 保留 86 个普通 WBC 动作，其中 4 个动作的 LoRA 与 reference 已用 `/home/luvbotic/checkpoints/wholebody1102/bushu_4motion_1102` 更新。`text_wbc_bundle` 已从原交付包精简为仅有 `UpperBody_Enhanced` 的文本 JSON 运行包，保留共享 Base RKNN、该动作的 LoRA adapter、配置和 reference。两套包独立加载，不混用文件。`INSTALLATION.json` 的哈希清单分别覆盖 433 和 8 个资产文件。

### 四动作替换（2026-11-02）

四个新 `lorackpt_2000.pt` 的 `base_sha256` 均与现有共享 WBC Base 一致，内部记录 `num_updates=2000`、`iteration=1999`。本地导出新 `weights.bin`；真机上仅在独立临时目录编译四个新 reference ONNX 的 RK3588 FP16 RKNN，再下载 RKNN 与转换记录。真机 `/userdata/forlinx/wbc_workspace/l0_deploy_v2_ws` 的 source/install 两份运行包已替换四个 reference RKNN、转换记录、必要帧数配置和哈希元数据；真机 LoRA、共享 Base、其余动作及 `zy_workspace` 均未修改。本地运行包还更新了四个 LoRA adapter；本地共享 Base RKNN、动作总数和其余 82 个动作文件保持不变。

| 动作 ID | 本地旧 → 新帧数 | 真机旧 → 新帧数 | 新 LoRA checkpoint SHA256 |
| --- | ---: | ---: | --- |
| `L0_strange_new_1_A` | 278 → 378 | 778 → 378 | `e5fd1d7050329a1cbcf796ca000d5da953c3ef8219be843964f843f3872bfb52` |
| `L0_strange_1` | 302 → 402 | 802 → 402 | `5a33ba52fe050ed8c775eca31bb2a1e01b1343475ff284174e5f8baeb6e827fe` |
| `L0_excited_8` | 302 → 402 | 802 → 402 | `83cf4a7cf962950ed32e908b70a34bd1117b3dc75a180539d44797cc7bcc4f41` |
| `L0_angry_new_2` | 284 → 384 | 784 → 384 | `fa2384b6a050a15a0d64118e44c634f9c4440727d7809ec675f1dd76fd74af54` |

四个 `reference_motion.rknn.json` 保存新 ONNX 和 RKNN 哈希，adapter manifest 保存新参数哈希；更新后的 `base_manifest.json`、共享 RKNN 转换记录中的清单哈希及 `INSTALLATION.json` 已一起校准。转换采用真机 `/usr/bin/python3`、RKNN Toolkit2 2.3.2、`--build-only`；C++ worker 加载及实机动作仍须在部署新包后验收。

## 运行链路

- 普通动作：`wbc_lora.launch.py` → `sim2real.launch.py` → `one_start.launch.py` → `RLWbc`，读取 `wbc_lora_fp16/base_manifest.json`。默认只预加载两个动作；通过 `wbc_policies` 指定额外动作，动作 ID 必须出现在该 manifest 中。
- 文本动作：`text_wbc.launch.py` 同样进入正式 WBC，但读取 `text_wbc_bundle`，仅预加载 `UpperBody_Enhanced`，以该动作的 LoRA 参数执行动态 JSON reference。`text_wbc_generate` 使用工作空间中的 `model_sources/text_to_motion/compiled/`、`deps/rknnlite` 与 `runtime/librknnrt.so`；这些来自 2026-11-02 文本模型补充包。`text_wbc_1029_code_update_20261102` 的补丁已应用。`l0_v5_amp_10281` 已放入源码策略目录。文本模型原始 ONNX 和编译流程见 [文本动作说明](text-motion-deployment.md)。
- 两种入口都依赖机器人端 HPM、ROS2 Humble、RK3588 RKNN Runtime 和匹配的 `luvbot_ros_interfaces`。启动前按机器人现有流程检查电机、急停和校准。

## 板端构建与检查

在目标工作空间中放入这份源码及匹配的接口包，先确认资产已随源码传到板端。建议使用新的安装工作区；若复用已有 `install/`，需在相关进程停止后清理旧 `text_wbc_bundle` 安装目录，避免旧动作文件残留。以下命令只构建与检查文件，不启动电机：

```bash
cd /userdata/forlinx/你的部署工作空间
source /opt/ros/humble/setup.bash
test -f src/l0_ros2_deploy_v2/legged_deploy/policy/l0_v5/wbc_lora_fp16/INSTALLATION.json
test -f src/l0_ros2_deploy_v2/legged_deploy/policy/l0_v5/text_wbc_bundle/INSTALLATION.json
test -f src/l0_ros2_deploy_v2/legged_deploy/policy/l0_v5/l0_v5_amp_10281/policy.rknn
test -f src/l0_ros2_deploy_v2/legged_deploy/policy/l0_v5/l0_v5_amp_10281/policy.onnx
colcon build --symlink-install --packages-up-to legged_real legged_controllers \
  --cmake-args -DLEGGED_DEPLOY_BACKEND=RKNN
source install/setup.bash
test -f install/legged_deploy/share/legged_deploy/policy/l0_v5/wbc_lora_fp16/base_lora.rknn
test -f install/legged_deploy/share/legged_deploy/policy/l0_v5/text_wbc_bundle/base_lora.rknn
test -f install/legged_deploy/share/legged_deploy/policy/l0_v5/l0_v5_amp_10281/rl_config.yaml
test -f install/legged_deploy/share/legged_deploy/policy/l0_v5/l0_v5_amp_10281/joy_config.yaml
```

运行前可验证安装目录的清单哈希：

```bash
python3 - <<'PY'
import hashlib, json
from pathlib import Path
root = Path('install/legged_deploy/share/legged_deploy/policy/l0_v5')
for name in ('wbc_lora_fp16', 'text_wbc_bundle'):
    bundle = root / name
    files = json.loads((bundle / 'INSTALLATION.json').read_text())['files']
    bad = [rel for rel, expected in files.items()
           if not (bundle / rel).is_file()
           or hashlib.sha256((bundle / rel).read_bytes()).hexdigest() != expected]
    if bad:
        raise SystemExit(f'{name}: invalid assets: {bad[:5]}')
    print(f'{name}: {len(files)} asset hashes OK')
text_bundle = root / 'text_wbc_bundle'
actions = json.loads((text_bundle / 'base_manifest.json').read_text())['actions']
assert [action['action_id'] for action in actions] == ['UpperBody_Enhanced']
listed = set(json.loads((text_bundle / 'INSTALLATION.json').read_text())['files'])
actual = {str(path.relative_to(text_bundle)) for path in text_bundle.rglob('*') if path.is_file()}
assert actual == listed | {'INSTALLATION.json'}, 'stale or missing text bundle assets'
print('text_wbc_bundle: only UpperBody_Enhanced remains')
PY
```

## 实机启动与验收

在 HPM 就绪、机器人支撑固定且急停可用后，普通动作以 `ros2 launch legged_controllers wbc_lora.launch.py hpm_mode:=real` 启动。需要其他动作时传 `wbc_policies:='[L0_Hugme, L0_sad_1]'`；不要将两个包中同名动作的文件交叉复制。进入 Walking 并保持运动指令接近零后，通过 `/algorithm/brain/wbc_action_cmd` 发送在预加载名单中的动作 ID，按 [LoRA 正式部署说明](wbc-lora-production-deployment.md)检查 prepare、active、结束回 AMP、取消和急停路径。

文本链路先核对工作空间根目录的 `TEXT_MODELS_SHA256SUMS` 与板端安装的 `l0_v5_amp_10281`，再按 [文本动作说明](text-motion-deployment.md)设置 `TEXT_WBC_MODEL`、`TEXT_WBC_BUNDLE` 与 `TEXT_WBC_MAPPING`。启动 `text_wbc.launch.py` 后先用离线 snapshot 只生成 JSON，再在实时观测就绪时用 `--execute` 验证 JSON → WBC → AMP 的完整回路。当前源码集成和清单哈希检查不能替代板端 RKNN 加载及实机动作验收。
