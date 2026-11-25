# L0 上半身动作部署包

请复制整个目录，不要只复制一个 ONNX。包内有两个数值模型、词表、动作素材、便携运行时和 RK3588 编译脚本。分词与动作组合在主机运行时执行，不在数值 ONNX 图中。

## 输入、输出与使用

在本目录执行（需安装 NumPy、ONNX Runtime）：

```bash
python -m deploy_rknn.infer --backend onnx-reference --model manifest.json --text '头以大幅度慢速点头两次。' --output motion.json
python -m deploy_rknn.infer --backend onnx-reference --model manifest.json --mode random --seed 42 --output random.json
```

- `--model`：完整部署包的 manifest.json 路径。
- `--text`：中文动作描述，默认 `--mode text`。
- `--start-pose`、`--end-pose`：九维弧度 JSON 数组、逗号分隔数值或数组文件，均默认内置 A-pose。顺序为右手三维、左手三维、头部三维；结束姿态仅用于活动部位回正。
- `--output`：输出 JSON 路径；轨迹字段 `motion` 为 float32 语义的 `[帧数,9]` 数组，单位弧度，`fps=50`；同时包含 token、素材 ID、帧数与回正分界等元数据。
- `--mode random`：随机选择 1–3 个部位，为每个选中的部位随机选择一个可用动作；其他部位保持不动。不需要输入文本。`--seed` 可复现动作选择、素材选择和肩部扰动。
- `--return True/False`：默认 True。开启后，所有动作结束先保持 0.5 秒，再回正；全无动作时不增加停顿。
- `--return-frames`：默认 -1（模型预测），也支持 30/60/90。
- `--speed`：默认 1.2；动作与回正分别变速，中间的 0.5 秒停顿不受影响。
- `--mirror-head true/false`：默认 false；`--mirror-hands true/false`：默认 true。也可仅写 `--mirror-head` 启用头部镜像。

## 测试文本怎么生成、组合

项目使用 `runs/text_generate.sh`，根据已有动作素材的标注生成文本，不是让大语言模型自由编写。

1. **选已有动作**：从标注中读取“部位、动作、幅度、速度、次数”，相同的属性组合只保留一份，不随意拼出素材里不存在的属性组合。
2. **选 1–3 个部位组合**：部位只有头、左手、右手，每个部位最多选一个动作。可以只动头，也可以头和一只手一起动，或三个部位一起动；没有选中的部位不写入文本。
3. **拼成一句话**：固定按“头 → 左手 → 右手”排列，不同部位用“；”连接，最后加“。”。这些动作同时执行，不表示先做头部动作、再做手部动作。
4. **生成不同详细程度的描述**：同一组动作生成“仅动作”“动作+幅度”“动作+幅度+速度”“动作+幅度+速度+次数”四种文本。简化模式省略“中幅度”“中速”和“一次”。

例如，已有标注为“头：点头、大幅度、慢速、两次”，可以生成：

- 头点头。
- 头以大幅度点头。
- 头以大幅度慢速点头。
- 头以大幅度慢速点头两次。

再选一个左手动作，可以组合为：**头以大幅度慢速点头两次；左手抬起。**

再选一个右手动作，可以组合为：**头以大幅度慢速点头两次；左手抬起；右手摸肚子两次。**

示例中的手部动作也必须有对应的素材标注。自行编写测试文本时，可按上述格式组合本包已有的动作。`--mode random` 则直接随机选择动作，不会先生成这些测试文本。

## 板端编译 RKNN

在本目录使用 Python 3.10、RKNN-Toolkit2 **2.3.2**：

```bash
RKNN_PYTHON=python3.10 bash compile.sh
```

输出到 `compiled/`，不做 INT8 量化。进入 compiled 目录后，在匹配 Lite2/NPU 驱动的 RK3588 上运行：

```bash
python -m deploy_rknn.infer --model manifest.json --text '头点头' --output motion.json
```

ONNX 数值验证不代表 RKNN 编译或真机验证通过。
