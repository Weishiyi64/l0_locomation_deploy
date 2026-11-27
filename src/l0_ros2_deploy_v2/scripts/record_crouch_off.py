#!/usr/bin/python3
"""Record one real poweroff crouch-off, including exact actor input and feedback."""
import argparse
from collections import Counter, deque
from datetime import datetime
from functools import partial
import json
import os
from pathlib import Path
import time

import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.serialization import deserialize_message
import rosbag2_py
from rosidl_runtime_py.utilities import get_message

ACTION = 'L0_crouch_off'
EVENT = '/system/state_machine/wbc_action_fdb'
LOG = '/rosout'
ACTOR = '/control/policy/wbc/actor_sample'
OBS = '/control/policy/observation'
MOTOR = '/actuator/motor/state'
IMU = '/imu/data'
AMP = '/control/policy/amp/actor_input'
COMMAND = '/control/policy/command/wbc'
REQUIRED = {EVENT, LOG, ACTOR, OBS, MOTOR, IMU, COMMAND, AMP}
OPTIONAL = { '/actuator/motor/command', '/control/policy/command/rl', '/odom',
            '/control/policy/wbc/active', '/control/policy/wbc/done',
            '/algorithm/brain/wbc_action_cmd', '/system/safety/emergency_stop',
            '/system/safety/hardware_fault', '/actuator/motor/enable'}
NS = 1_000_000_000


def stamp_ns(stamp):
    return stamp.sec * NS + stamp.nanosec


def load_run(run):
    cfg = json.loads((run / 'launch_config.json').read_text())
    actions = json.loads((run / 'base_manifest.json').read_text())['actions']
    selected = json.loads(cfg['wbc_policies'])
    if cfg['robot_type'] != 'l0_v5' or ACTION not in selected or ACTION not in {a['action_id'] for a in actions}:
        raise ValueError('当前 L0 V5 部署必须包含并启用 L0_crouch_off')
    for line in (run / 'version.txt').read_text().splitlines():
        key, _, value = line.partition('=')
        if key in {'ROS_DOMAIN_ID', 'ROS_LOCALHOST_ONLY', 'RMW_IMPLEMENTATION'}:
            if value == 'unset':
                os.environ.pop(key, None)
            else:
                os.environ[key] = value


class Capture:
    def __init__(self, node, path):
        self.node = node
        self.path = path
        self.writer = None
        self.types = {}
        self.subs = {}
        self.count = Counter()
        self.last = {}
        self.command_source = self.command_sequence = None
        self.live = set()
        self.ready = False
        self.start = self.end = self.stop_mono = None
        self.pending = deque(maxlen=128)
        self.previous_obs = deque(maxlen=128)
        self.handoff = self.sequence = None
        self.gaps = self.frames = 0
        self.error = None
        self.loggers = set()

    def discover(self):
        for topic, names in self.node.get_topic_names_and_types():
            if topic in self.subs or topic not in REQUIRED | OPTIONAL or len(names) != 1:
                continue
            publishers = self.node.get_publishers_info_by_topic(topic)
            if not publishers:
                continue
            cls = get_message(names[0])
            if topic == EVENT:
                self.loggers = {p.node_name for p in publishers}
            self.types[topic] = (names[0], cls)
            qos = QoSProfile(depth=200, reliability=ReliabilityPolicy.BEST_EFFORT)
            if topic in (EVENT, LOG) and all(
                    p.qos_profile.reliability == ReliabilityPolicy.RELIABLE for p in publishers):
                qos.reliability = ReliabilityPolicy.RELIABLE
            self.subs[topic] = self.node.create_subscription(
                cls, topic, partial(self.receive, topic), qos, raw=True)
            if self.writer is not None:
                self.register(topic)
        if not self.ready and {OBS, IMU, MOTOR} <= self.live and REQUIRED <= self.subs.keys():
            self.ready = True
            print('就绪，可执行 L0_crouch_off。', flush=True)

    def register(self, topic):
        self.writer.create_topic(rosbag2_py.TopicMetadata(
            name=topic, type=self.types[topic][0], serialization_format='cdr'))

    def begin(self, now):
        self.writer = rosbag2_py.SequentialWriter()
        self.writer.open(rosbag2_py.StorageOptions(uri=str(self.path), storage_id='sqlite3'),
                         rosbag2_py.ConverterOptions('', ''))
        for topic in self.types:
            self.register(topic)
        self.start = now
        print(f'动作已开始，正在录制 {self.path}', flush=True)

    def write(self, topic, raw, now):
        self.writer.write(topic, raw, now)
        self.count[topic] += 1
        self.last[topic] = now

    def actor(self, raw, msg, now):
        if self.handoff is None:
            self.handoff = msg.handoff_id
        if self.handoff != msg.handoff_id:
            self.error = '录制期间 WBC 切换到了另一次动作'
            return
        if not msg.preparation:
            self.frames += 1
        if self.sequence is not None and msg.sequence != self.sequence + 1:
            self.gaps += 1
        self.sequence = msg.sequence
        if (len(msg.policy), len(msg.command), len(msg.reference_joint_position)) != (399, 240, 21):
            self.error = 'WBC actor 或参考轨迹维数异常'
        self.write(ACTOR, raw, now)

    def receive(self, topic, raw):
        now = self.node.get_clock().now().nanoseconds
        self.live.add(topic)
        if not self.ready or (self.end is not None and now > self.end):
            return
        if topic == EVENT:
            data = json.loads(deserialize_message(raw, self.types[topic][1]).data)
            if data.get('status') == 'started':
                if self.start is not None:
                    self.error = '录制窗口内出现了第二次 started 事件'
                elif data.get('action_id') == ACTION:
                    self.begin(now)
        if topic == ACTOR:
            msg = deserialize_message(raw, self.types[topic][1])
            if msg.motion_id != ACTION:
                return
            if self.start is None:
                self.pending.append((now, raw, msg))
                return
            if self.handoff is None:
                # Preparation runs before "started". Include only this handoff and
                # its feedback, backfilled at the bag boundary with original headers.
                previous = [item for item in self.pending if item[2].handoff_id == msg.handoff_id]
                stamps = {stamp_ns(item[2].header.stamp) for item in previous}
                stamps.add(stamp_ns(msg.header.stamp))
                for stamp, sample in self.previous_obs:
                    if stamp in stamps:
                        self.write(OBS, sample, self.start)
                for _, sample, frame in previous:
                    self.actor(sample, frame, self.start)
                self.pending.clear()
                self.previous_obs.clear()
            self.actor(raw, msg, now)
            return
        if self.start is None:
            if topic == OBS:
                header = deserialize_message(raw, self.types[topic][1]).header
                self.previous_obs.append((stamp_ns(header.stamp), raw))
            return
        if topic == COMMAND:
            command = deserialize_message(raw, self.types[topic][1])
            if len(command.joint_names) != 21 or len(command.target_positions) != 21:
                self.error = 'WBC command 关节数异常'
            if self.command_source is not None and command.source_id != self.command_source:
                self.error = '录制窗口内切换到了另一次 WBC 请求'
            if self.command_sequence is not None and command.sequence_id != self.command_sequence + 1:
                self.error = 'WBC command 序号不连续'
            self.command_source = command.source_id
            self.command_sequence = command.sequence_id
        if topic == LOG and self.end is None:
            msg = deserialize_message(raw, self.types[topic][1])
            if (msg.msg.startswith('FSM: -> EmergencyStop (prev=') and
                    msg.name.rsplit('.', 1)[-1] in self.loggers):
                transition = stamp_ns(msg.stamp)
                if transition >= self.start:
                    self.end = transition + 6 * NS
                    self.stop_mono = time.monotonic() + max(0, self.end - now) / NS
                    print('检测到 EmergencyStop，录制至状态切换后 6 秒。', flush=True)
        self.write(topic, raw, now)

    def verify(self):
        if not self.frames or not self.count[COMMAND] or self.gaps:
            raise RuntimeError(f'WBC 推理帧 {self.frames}，序号间断 {self.gaps}，命令 {self.count[COMMAND]}')
        for topic in (IMU, MOTOR):
            if self.last.get(topic, 0) < self.end - NS // 4:
                raise RuntimeError(f'{topic} 缺少 EmergencyStop 后末段反馈')
        if not self.count[OBS]:
            raise RuntimeError('缺少控制器原始 observation')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('run', type=Path)
    args = parser.parse_args()
    load_run(args.run)
    bags = args.run / 'bags'
    bags.mkdir(exist_ok=True)
    rclpy.init()
    node = rclpy.create_node('record_poweroff_crouch_off')
    capture = Capture(node, bags / datetime.now().strftime('crouch_off_%Y%m%d_%H%M%S_%f'))
    tick = 0
    waiting_since = time.monotonic()
    try:
        while rclpy.ok():
            now = time.monotonic()
            if now >= tick:
                capture.discover()
                tick = now + 1
            if capture.error:
                raise RuntimeError(capture.error)
            if capture.stop_mono is not None and now >= capture.stop_mono:
                capture.verify()
                print(f'完成，WBC command {capture.count[COMMAND]} 帧，原始推理采样 {capture.frames} 帧，rosbag 位于 {capture.path}', flush=True)
                return
            if capture.start and node.get_clock().now().nanoseconds - capture.start > 120 * NS:
                raise RuntimeError('动作开始后 120 秒内未完成 EmergencyStop 录制窗口，已封存部分数据')
            if not capture.ready and now - waiting_since > 45:
                missing = sorted((REQUIRED - capture.subs.keys()) | ({OBS, IMU, MOTOR} - capture.live))
                raise RuntimeError(f'等待发布端超时，缺少 {missing}')
            rclpy.spin_once(node, timeout_sec=0.01)
        raise RuntimeError('ROS 已退出，录制未完成')
    finally:
        capture.writer = None  # Humble finalizes metadata in the writer destructor.
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        raise SystemExit('录制已中断')
    except (ValueError, OSError, RuntimeError) as error:
        raise SystemExit(str(error))
