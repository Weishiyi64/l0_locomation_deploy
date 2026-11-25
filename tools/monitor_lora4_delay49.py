#!/usr/bin/env python3
"""Read-only progress of the four fresh LoRA runs on d0501-lrc."""
import argparse
import json
import subprocess
import time
from datetime import datetime
from zoneinfo import ZoneInfo

REMOTE = '/home/wsy/runs/lora4-rollout1800-scratch2000-delay4to9-1030'
COLLECT = '''
from pathlib import Path
import json,time
root=Path(ROOT)
rows=[]
for entry in json.loads((root/'records/preparation.json').read_text())['motions']:
 p=root/'motions'/entry['key'];updates=0;state='排队等待';age=None
 for phase in ['smoke','train']:
  output=p/phase
  if not output.exists():continue
  state='小规模验证中' if phase=='smoke' else '训练初始化中'
  metrics=output/'metrics.jsonl'
  if metrics.exists():
   age=time.time()-metrics.stat().st_mtime
   for line in reversed(metrics.read_text().splitlines()):
    try:record=json.loads(line)
    except json.JSONDecodeError:continue
    if phase=='train':updates=int(record['iteration'])+1;state='训练中' if age<120 else '日志暂未更新'
    break
  outcome=output/'queue_outcome.json'
  if outcome.exists() and json.loads(outcome.read_text()).get('completed'):
   state='验证通过，等待正式训练' if phase=='smoke' else '已完成'
   if phase=='train':updates=2000
 rows.append(dict(name=entry['name'],updates=updates,status=state,age_s=age))
status=root/'records/queue_status.json'
print(json.dumps({'gpu':(root/'records/selected_gpu.txt').read_text().strip(),'motions':rows,'queue':json.loads(status.read_text()) if status.exists() else {}},ensure_ascii=False))
'''


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--watch', type=float, default=0)
    args = parser.parse_args()
    while True:
        result = subprocess.run(
            ['ssh', '-o', 'BatchMode=yes', '-o', 'ConnectTimeout=8', 'd0501-lrc', 'python3 -'],
            input='ROOT=' + repr(REMOTE) + '\n' + COLLECT,
            text=True, capture_output=True, check=True, timeout=30)
        data = json.loads(result.stdout)
        if args.watch:
            print('\033[2J\033[H', end='')
        now = datetime.now(ZoneInfo('Asia/Shanghai')).strftime('%Y-%m-%d %H:%M:%S')
        print(f'四动作全新 LoRA · 延迟 4–9 物理步 · GPU{data["gpu"]} · {now}')
        for row in data['motions']:
            print(f'{row["name"]:24} {row["updates"]:4}/2000  {row["status"]}')
        if data['queue'].get('status') == 'failed':
            print('队列失败：', data['queue'].get('name'), data['queue'].get('phase'))
        print('Ctrl+C 只退出查看，不会停止服务器训练。')
        if not args.watch:
            return
        time.sleep(max(1, args.watch))


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        pass
