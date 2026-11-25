# L0 文本动作生成部署
```
## 板端编译为 RKNN

使用已配置的 Python 3.10.12、RKNN-Toolkit2 2.3.2 环境，在交付目录执行：

```bash
cd text_to_motion_onnx
RKNN_PYTHON=python3.10 bash compile.sh
```

结果保存到同目录的 `compiled/`。

## 推理

板端需要 NumPy、RKNN-Toolkit-Lite2，以及兼容的 NPU 驱动和运行库。进入编译结果目录后执行：

```bash
cd compiled
python3.10 -m deploy_rknn.infer \
  --model manifest.json \
  --text '右手腹部摆动两次。' \
  --output motion.json
```

指定首尾姿态：

```bash
python3.10 -m deploy_rknn.infer \
  --model manifest.json \
  --text '头点头；左手抬起。' \
  --start-pose start.json \
  --end-pose end.json \
  --output motion.json
```

### 输入参数及格式

| 参数 | 格式与说明 |
|---|---|
| `--model` | 可选，完整部署包的 `manifest.json` 路径，例如 `manifest.json` 或 `/path/to/compiled/manifest.json`，不是单个 `.onnx` 或 `.rknn` 文件。使用编译包内的推理代码时，默认读取该包根目录的 `manifest.json`。 |
| `--text` | 必填，非空动作文本，用引号包住，例如 `'右手腹部摆动两次。'`。 |
| `--start-pose` | 可选，9 个关节角，单位为弧度；省略时使用内置 A-pose。 |
| `--end-pose` | 可选，格式同上，省略时使用内置 A-pose。作为末尾回正目标，仅作用于有动作部位；无动作部位保持初始姿态。目标角度必须在关节限位内。 |
| `--output` | 可选，输出 JSON 文件路径，例如 `motion.json`；省略时打印到标准输出。自动创建父目录，已有同名文件会被覆盖。 |

姿态顺序均为：**右手 3 维、左手 3 维、头部 3 维**。具体关节名见输出的 `joint_names`。
两种姿态参数都支持以下三种输入形式（零值仅为格式示意，不代表内置 A-pose）：

```text
JSON 数组：  --start-pose '[0,0,0,0,0,0,0,0,0]'
逗号分隔：  --start-pose '0,0,0,0,0,0,0,0,0'
数组文件：  --start-pose start.json
```

`start.json` / `end.json` 的内容应直接是九维数组，例如 `[0,0,0,0,0,0,0,0,0]`，不是带字段的对象。

### 输出格式

输出为 JSON 对象。主要字段：

| 字段 | 格式与说明 |
|---|---|
| `motion` | `N × 9` 数值数组，每行是一帧关节角，单位为弧度，顺序同输入姿态。 |
| `frame_count` | 单元素数组 `[N]`，表示最终输出帧数。 |
| `fps` | 固定为 `50`，每帧间隔 0.02 秒。 |
| `joint_names` | 9 个关节名称，与 `motion` 的列一一对应。 |
| `success` | 单元素布尔数组 `[true]`。推理失败时脚本报错退出。 |

文件还包含输入文本、动作 token、素材 ID、回正分界和推理耗时等辅助信息。
轨迹已包含默认回正和整体变速处理，按输出的 50 FPS 播放即可。
