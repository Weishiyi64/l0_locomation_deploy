# 文本生成动作

模型和算子库集中放在本目录，操作统一使用 `run_offline.sh`。文本 JSON 经 `checkpoint_2000` WBC 打包并在 MuJoCo 执行的步骤见 [MuJoCo WBC 测试流程](../docs/text-wbc-mujoco-test.md)。

WBC 打包流程先运行 `--wbc-input`，自动采集当前姿态并生成 JSON，不直接发布到文本叠加控制器：

```bash
bash texttomotion/run_offline.sh --wbc-input --text '左手以大幅度抬起。' --output /tmp/l0_text_wbc/TEXT04.json
```

随后使用 `package_text_wbc.py` 生成 WBC 参考 ONNX、配置和共享策略链接。以下 `--live` 是原有的直接叠加播放模式，两种模式的测试步骤分开使用。

已启动 MuJoCo、进入 AMP 并手动开始动力学后，在相同 ROS 域中执行：

```bash
bash texttomotion/run_offline.sh --live --text '左手以大幅度抬起。' --output /tmp/l0_text.json
```

每次重新采集当前姿态，作为新模型 `--start-pose` 与 `--end-pose`；自动发送完整轨迹并等待完成。不要继续使用旧 `--init_pose`。

参考窗口只开一次：

```bash
bash texttomotion/run_offline.sh --preview
```

它监听实际发送的 JSON 和播放状态，自动切换动作。纯离线生成可去掉 `--live` 并显式提供 `--start-pose 文件 --end-pose 文件`；不传姿态则使用模型内置 A-pose。默认使用脚本旁FP16 参数副本 `run_fp16.onnx`（原始 `run.onnx` 保留） 与 `libluvbot_onnx_text.so.1`，其他生成参数保留模型默认值，不做特定文本的轨迹替换。机械超限角度仍会裁剪并记录数量。

文本播放在 AMP Walking 上叠加上半身，腿部仍由 AMP 控制；WBC 是独立模式，二者不会同时占用上半身。Python 文件是生成、采集、发布和预览的内部模块，不需要逐个调用。

FP16 参数核对、十条动作裁剪和实测跟随结果、板端控制 + x86 文本生成的真机流程，见完整部署文档第 9～10 节。`--live` 从 `/control/policy/observation` 读取已校准关节角，仿真/真机共用；现有分词库是 x86-64，不在 RK3588 上直接加载。
