#!/usr/bin/env python3
"""Interactive front end to the existing WBC action topic, never a controller."""

import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import re
import shlex
import signal
import subprocess
import sys
import termios
import time

import yaml

COMMAND_TOPIC = '/algorithm/brain/wbc_action_cmd'
FEEDBACK_TOPIC = '/system/state_machine/wbc_action_fdb'
WORKER = '/rl_controller_wbc'


def load_actions(bundle, selected):
    actions = json.loads(selected)
    if (not isinstance(actions, list) or not actions or
            any(not isinstance(a, str) or not re.fullmatch(r'[A-Za-z0-9_-]{1,32}', a) for a in actions) or
            len(actions) != len(set(actions))):
        raise ValueError('动作名单必须非空、无重复且使用合法 ID')
    graph = json.loads((bundle / 'base_manifest.json').read_text())
    entries = {a['action_id']: a for a in graph['actions']}
    durations = {}
    for action in actions:
        entry = entries[action]
        config = (bundle / entry['config']).resolve(strict=True)
        if not config.is_relative_to(bundle.resolve()):
            raise ValueError('动作配置路径越界')
        data = config.read_bytes()
        if hashlib.sha256(data).hexdigest() != entry['config_sha256']:
            raise ValueError('动作配置哈希不符：' + action)
        cfg = yaml.safe_load(data)['WbcCfg']
        duration = float(cfg['reference_num_frames']) * float(cfg['control_dt']) + float(cfg.get('start_hold_sec', 0))
        if cfg['motion_id'] != action or not math.isfinite(duration) or duration <= 0:
            raise ValueError('动作 ID 或时长无效：' + action)
        durations[action] = duration
    return actions, durations


def resolve_action(value, actions):
    # Exact ID wins if a deployment happens to use a numeric ID.
    if value in actions:
        return value
    if re.fullmatch(r'[0-9]{1,6}', value):
        index = int(value) - 1
        if 0 <= index < len(actions):
            return actions[index]
    return None


def show_menu(actions):
    print('\nWBC 动作控制台')
    for number, action in enumerate(actions, 1):
        print(f'  {number:2d}. {action}')
    print('输入编号或完整动作 ID 后回车；list 重新显示，q 退出。')
    print('每次只请求一个动作；退出控制台不会停止机器人或取消正在执行的动作。')


def stop_publisher(process):
    if process is not None and process.poll() is None:
        try:
            os.killpg(process.pid, signal.SIGINT)
        except ProcessLookupError:
            process.wait()
            return
        try:
            process.wait(timeout=2)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGKILL)
            process.wait()


def run_console(args, actions, durations):
    # Keep preview/list validation independent of ROS and DDS.
    import rclpy
    from rcl_interfaces.srv import GetParameters
    from rclpy.executors import ExternalShutdownException
    from rclpy.parameter import parameter_value_to_python
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
    from std_msgs.msg import Bool, String

    graph = json.loads((args.bundle / 'base_manifest.json').read_text())
    expected_mode = {'wbc_base_graph_v1': 'base', 'wbc_lora_graph_v1': 'lora'}[graph['format']]
    if not sys.stdin.isatty():
        raise ValueError('动作控制台需要交互终端，不接受管道或批量输入')
    rclpy.init(args=[])
    node = rclpy.create_node(f'wbc_action_console_{os.getpid()}')
    pending = None
    terminal = None
    active = None
    publisher = None
    last_timestamp = 0
    feedback_path = args.run_dir / 'action_feedback.jsonl'
    request_path = args.run_dir / 'action_requests.jsonl'

    def record(path, value):
        with path.open('a') as stream:
            stream.write(json.dumps(value, ensure_ascii=False) + '\n')

    def feedback(message):
        nonlocal terminal
        try:
            value = json.loads(message.data)
            if not isinstance(value, dict):
                return
        except (ValueError, TypeError):
            return
        record(feedback_path, {'received_at_ms': time.time_ns() // 1_000_000, 'feedback': value})
        if pending and (value.get('timestamp_ms'), value.get('action_id')) == pending:
            status = value.get('status')
            print(f'反馈：{pending[1]} → {status}', flush=True)
            if status in ('completed', 'rejected', 'failed'):
                terminal = status

    def set_active(message):
        nonlocal active
        active = message.data

    node.create_subscription(String, FEEDBACK_TOPIC, feedback, QoSProfile(depth=100))
    node.create_subscription(Bool, '/control/policy/wbc/active', set_active,
                             QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                                        durability=DurabilityPolicy.TRANSIENT_LOCAL))
    client = node.create_client(GetParameters, WORKER + '/get_parameters')

    def ready():
        if not client.wait_for_service(timeout_sec=2):
            raise ValueError('WBC worker 未就绪，未发送动作')
        names = [namespace.rstrip('/') + '/' + name for name, namespace in node.get_node_names_and_namespaces()]
        if names.count(WORKER) != 1:
            raise ValueError('必须只有一个 /rl_controller_wbc')
        request = GetParameters.Request()
        request.names = ['wbc_mode', 'wbc_lora_manifest', 'available_wbc_policies']
        future = client.call_async(request)
        rclpy.spin_until_future_complete(node, future, timeout_sec=3)
        if not future.done():
            future.cancel()
            raise ValueError('读取 worker 参数超时，未发送动作')
        response = future.result()
        if response is None or len(response.values) != 3:
            raise ValueError('无法读取 worker 参数')
        mode, manifest, loaded = [parameter_value_to_python(v) for v in response.values]
        if (mode != expected_mode or not isinstance(manifest, str) or
                Path(manifest).resolve() != (args.bundle / 'base_manifest.json').resolve() or
                not isinstance(loaded, list) or loaded != actions):
            raise ValueError('运行中的 worker 与控制台加载的配置不一致，请核对启动参数并重启部署/控制台')
        services = dict(node.get_service_names_and_types())
        if 'luvbot_control_interfaces/srv/ControlWbcMotion' not in services.get('/control/policy/wbc/prepare', []):
            raise ValueError('prepare 服务未就绪，未发送动作')
        deadline = time.monotonic() + 2
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
            if (active is not None and node.count_publishers(FEEDBACK_TOPIC) > 0 and
                    node.count_subscribers(COMMAND_TOPIC) > 0):
                break
        if active is not False:
            raise ValueError('WBC 正在执行，或尚未收到 active 状态；未发送动作')
        if node.count_publishers(FEEDBACK_TOPIC) == 0 or node.count_subscribers(COMMAND_TOPIC) == 0:
            raise ValueError('动作请求/反馈端点未就绪，未发送动作')

    try:
        show_menu(actions)
        print('执行前按原手柄流程进入 Walking 并保持零速度；控制台不会切换状态。')
        while rclpy.ok():
            try:
                value = input('\n动作> ').strip()
            except EOFError:
                break
            if value.lower() in ('q', 'quit', 'exit'):
                break
            if value.lower() in ('list', 'help', '?'):
                show_menu(actions)
                continue
            if not value:
                continue
            action = resolve_action(value, actions)
            if action is None:
                print('未识别输入。请输入列表编号或完整动作 ID，不支持模糊匹配。')
                continue
            try:
                ready()
            except ValueError as error:
                print(error)
                termios.tcflush(sys.stdin.fileno(), termios.TCIFLUSH)
                continue
            timestamp = max(time.time_ns() // 1_000_000, last_timestamp + 1)
            last_timestamp = timestamp
            pending, terminal = (timestamp, action), None
            payload = json.dumps({'data': json.dumps({'timestamp_ms': timestamp, 'action_id': action})})
            command = ['ros2', 'topic', 'pub', '--once', '--wait-matching-subscriptions', '1',
                       '--keep-alive', '0.2', '--qos-reliability', 'reliable', '--qos-durability', 'volatile',
                       COMMAND_TOPIC, 'std_msgs/msg/String', payload]
            print(shlex.join(command), flush=True)
            record(request_path, {'timestamp_ms': timestamp, 'action_id': action, 'command': command, 'status': 'attempted'})
            deadline = time.monotonic() + 3
            with (args.run_dir / 'action_publish.log').open('a') as log:
                publisher = subprocess.Popen(command, stdin=subprocess.DEVNULL, stdout=log, stderr=log,
                                             start_new_session=True)
                while publisher.poll() is None and time.monotonic() < deadline:
                    rclpy.spin_once(node, timeout_sec=0.05)
                if publisher.poll() is None:
                    stop_publisher(publisher)
                    raise RuntimeError('发布超时，结果不确定；控制台退出且不重发，请检查反馈与机器人状态')
                if publisher.returncode != 0:
                    raise RuntimeError('发布指令失败；控制台退出且不重发，请检查 action_publish.log')
            publisher = None
            deadline = time.monotonic() + durations[action] + 30
            while terminal is None and time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=0.1)
            # Do not turn input typed during execution into a queued action.
            termios.tcflush(sys.stdin.fileno(), termios.TCIFLUSH)
            if terminal is None:
                raise RuntimeError('未收到终态反馈；控制台退出且不重发，请检查部署日志')
            if terminal == 'failed':
                raise RuntimeError('动作反馈 failed；控制台退出，请按原流程检查故障')
            print('本次请求结束；执行期间提前输入的内容已丢弃。')
            pending = None
    except ExternalShutdownException:
        print('\nROS 上下文已关闭；未发送新的动作请求。')
    finally:
        stop_publisher(publisher)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--bundle', type=Path, required=True)
    parser.add_argument('--actions', required=True)
    parser.add_argument('--run-dir', type=Path, required=True)
    parser.add_argument('--dry-run', action='store_true')
    args = parser.parse_args()
    try:
        actions, durations = load_actions(args.bundle, args.actions)
        if args.dry_run:
            show_menu(actions)
            print('仅预览：未创建 ROS 节点，未发布动作。')
            return 0
        run_console(args, actions, durations)
        return 0
    except KeyboardInterrupt:
        print('\n控制台已退出；机器人和正在执行的动作仍由原控制链管理。')
        return 130
    except (OSError, ValueError, KeyError, RuntimeError) as error:
        print(f'错误：{error}', file=sys.stderr)
        return 1


if __name__ == '__main__':
    sys.exit(main())
