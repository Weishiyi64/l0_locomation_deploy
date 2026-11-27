#!/usr/bin/env python3
"""Single-task mapping integration brain; no retries, restart recovery or localization."""
import json
import re
import time
import uuid
from collections import OrderedDict
from pathlib import Path
from runpy import run_path

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from std_msgs.msg import String
from luvbot_control_interfaces.msg import ExploreStatus
from luvbot_control_interfaces.srv import MapControl


REQUIRED_MAP_STAGES = frozenset({'Mapping', 'Segmentation'})


class FakeBrainMapping(Node):
    def __init__(self):
        super().__init__('fake_brain_mapping')
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.VOLATILE)
        self.reply_pub = self.create_publisher(String, '/algorithm/brain/map_reply', qos)
        self.artifact_pub = self.create_publisher(String, '/algorithm/brain/map_artifact_ready', qos)
        self.perception_pub = self.create_publisher(String, '/algorithm/em/mode_command', qos)
        self.create_subscription(String, '/algorithm/brain/map_command', self.command, qos)
        self.create_subscription(String, '/algorithm/brain/map_upload_result', self.upload_result, qos)
        self.create_subscription(String, '/algorithm/em/mode_state', self.perception_state, qos)
        self.create_subscription(ExploreStatus, '/algorithm/mapping/status', self.status, qos)
        self.radar = self.create_client(MapControl, '/control/radar/map_mode')
        self.mapping = self.create_client(MapControl, '/algorithm/mapping/control')
        self.phase = 'IDLE'
        self.slot = 0
        self.map_id = ''
        self.start_command = None
        self.start_replied = False
        self.results = OrderedDict()
        self.call = None
        self.perception_target = None
        self.perception_ok = False
        self.radar_ok = False
        self.finished_stages = set()
        self.next_log = 0.0
        self.create_timer(0.05, self.tick)
        self.get_logger().info('Fake brain ready; waiting for server map.start via cloud bridge')

    @staticmethod
    def publish(publisher, value):
        publisher.publish(String(data=json.dumps(value, ensure_ascii=False, separators=(',', ':'))))

    def transition(self, phase):
        self.phase = phase
        self.next_log = 0.0
        self.get_logger().info(f'[mapping] mapId={self.map_id} phase={phase}')

    def reply(self, command, code, message):
        key = command['msg_id']
        result = {'msg_id': key, 'cmd': command['cmd'], 'code': code, 'message': message}
        self.results[key] = (command, result)
        if len(self.results) > 128:
            self.results.popitem(last=False)
        self.publish(self.reply_pub, result)

    def fail(self, message):
        self.get_logger().error(f'[mapping] mapId={self.map_id} phase={self.phase}: {message}; no automatic recovery')
        if self.start_command and not self.start_replied:
            self.reply(self.start_command, 1001, message)
            self.start_replied = True
        self.call = None
        self.perception_target = None
        self.transition('FAILED')  # Restart manually only after checking actual robot/module state.

    def command(self, message):
        try:
            if len(message.data) > 65536:
                raise ValueError('command too large')
            value = json.loads(message.data)
            if not isinstance(value, dict) or not isinstance(value.get('msg_id'), str) or not value['msg_id']:
                raise ValueError('missing msg_id')
            if not isinstance(value.get('params'), dict) or not isinstance(value.get('cmd'), str):
                raise ValueError('invalid cmd/params')
            previous = self.results.get(value['msg_id'])
            if previous:
                if previous[0] == value:
                    self.publish(self.reply_pub, previous[1])
                else:
                    self.publish(self.reply_pub, {'msg_id': value['msg_id'], 'cmd': value['cmd'],
                                                 'code': 1001, 'message': 'msg_id reused with different content'})
                return
            if self.start_command and value['msg_id'] == self.start_command['msg_id']:
                if value != self.start_command:
                    self.publish(self.reply_pub, {'msg_id': value['msg_id'], 'cmd': value['cmd'],
                                                 'code': 1001, 'message': 'msg_id reused with different content'})
                return
            cloud_id = value['params'].get('mapId')
            if not isinstance(cloud_id, str) or not re.fullmatch(r'[A-Za-z0-9_-]{1,128}', cloud_id):
                self.reply(value, 1003, 'params.mapId must contain 1 to 128 ASCII letters, digits, hyphens or underscores')
                return
            if value['cmd'] == 'map.start':
                index = value['params'].get('mapIndex')
                if index not in ('1', '2'):
                    self.reply(value, 1003, 'params.mapIndex must be "1" or "2"')
                    return
                if self.phase not in ('IDLE', 'COMPLETE'):
                    self.reply(value, 1005, 'mapping workflow busy or failed; actual state must be checked')
                    return
                self.map_id = cloud_id
                self.slot = int(index)
                self.start_command = value
                self.start_replied = False
                self.radar_ok = False
                self.finished_stages.clear()
                self.transition('PREPARING')
                self.set_perception('mapping')
                self.start_call(self.radar, MapControl.Request.ACTION_MAPPING, 'radar_mapping')
            elif value['cmd'] in ('map.zone.set', 'batch_set_room_name'):
                if cloud_id != self.map_id or self.phase not in (
                        'UPLOADING', 'SWITCHING_MAPPING_MODE', 'RELOCATING', 'RESTORING_PERCEPTION', 'COMPLETE'):
                    self.reply(value, 1003, 'metadata does not match the current map or phase')
                    return
                self.reply(value, 0, 'received and saved; navigation application not implemented in this demo')
            else:
                self.reply(value, 1003, 'unsupported mapping command')
        except (ValueError, TypeError, KeyError) as error:
            self.get_logger().warning(f'Ignored invalid map command: {error}')

    def set_perception(self, target):
        self.perception_ok = False
        self.perception_target = target
        self.perception_request = uuid.uuid4().hex
        self.perception_deadline = time.monotonic() + 30.0
        self.perception_sent = False

    def perception_state(self, message):
        if not self.perception_target or len(message.data) > 65536:
            return
        try:
            state = json.loads(message.data)
            if state.get('last_request_id') != self.perception_request:
                return
            if state.get('phase') == 'failed':
                self.fail('perception switch failed: ' + str(state.get('reason', 'unknown')))
            elif state.get('phase') == 'active' and state.get('mode') == self.perception_target:
                self.get_logger().info(f'[mapping] Perception confirmed mode={self.perception_target} '
                                       f'request_id={self.perception_request}')
                self.perception_ok = True
                self.perception_target = None
        except (ValueError, AttributeError):
            self.get_logger().warning('Ignored malformed perception mode_state')

    def start_call(self, client, action, purpose):
        self.call = {'client': client, 'action': action, 'purpose': purpose,
                     'deadline': time.monotonic() + 10.0, 'future': None}

    def tick_call(self, now):
        call = self.call
        if not call:
            return
        if now >= call['deadline']:
            self.fail(call['purpose'] + ' timeout; outcome unconfirmed if request was sent')
            return
        if call['future'] is None:
            if not call['client'].service_is_ready():
                return
            request = MapControl.Request(action=call['action'], map_id=self.map_id, index=self.slot)
            try:
                call['future'] = call['client'].call_async(request)
                call['deadline'] = now + 10.0
            except Exception as error:
                self.fail(call['purpose'] + ' call failed: ' + str(error))
            return
        if not call['future'].done():
            return
        try:
            result = call['future'].result()
            if not result.accepted or result.error_code != 0:
                self.fail(f"{call['purpose']} rejected: {result.error_code} {result.message}")
                return
        except Exception as error:
            self.fail(call['purpose'] + ' response failed: ' + str(error))
            return
        self.call = None
        if call['purpose'] == 'radar_mapping':
            self.radar_ok = True
        elif call['purpose'] == 'mapping_start':
            # shortcut: use the service reply until the mapping module publishes RUNNING reliably.
            self.reply(self.start_command, 0, 'mapping started')
            self.start_replied = True
            self.transition('MAPPING')
        elif call['purpose'] == 'mapping_relocation':
            self.transition('RELOCATING')
            self.start_call(self.radar, MapControl.Request.ACTION_RELOCATION, 'radar_relocation')
        elif call['purpose'] == 'radar_relocation':
            self.transition('RESTORING_PERCEPTION')
            self.set_perception('interaction')

    def status(self, message):
        if self.phase not in ('STARTING', 'MAPPING') or message.node_name not in REQUIRED_MAP_STAGES:
            return
        self.get_logger().info(f'[mapping] {message.node_name} state={message.state} cost_sec={message.cost_time} '
                               f'error={message.error_code} {message.error_msg}')
        if message.state == ExploreStatus.FAILED or message.error_code != 0:
            self.fail(message.node_name + ' failed: ' + message.error_msg)
        elif message.state == ExploreStatus.FINISHED:
            self.finished_stages.add(message.node_name)

    def upload_result(self, message):
        if self.phase != 'UPLOADING' or len(message.data) > 4096:
            return
        try:
            result = json.loads(message.data)
            if type(result.get('index')) is not int or result['index'] != self.slot:
                return
            if result.get('start_msg_id') != self.start_command['msg_id']:
                return
            if result.get('success') is True:
                self.transition('SWITCHING_MAPPING_MODE')
                self.start_call(self.mapping, MapControl.Request.ACTION_RELOCATION, 'mapping_relocation')
            else:
                self.fail('map upload unconfirmed/failed: ' + str(result.get('message', 'unknown')))
        except (ValueError, AttributeError):
            self.get_logger().warning('Ignored malformed map upload result')

    def tick(self):
        now = time.monotonic()
        if self.phase in ('IDLE', 'COMPLETE', 'FAILED'):
            return
        if now >= self.next_log:
            self.get_logger().info(f'[mapping] mapId={self.map_id} waiting phase={self.phase}')
            self.next_log = now + 10.0
        if self.perception_target:
            if now >= self.perception_deadline:
                if self.perception_sent:
                    self.fail(f'perception switch timeout; no matching active mode={self.perception_target} '
                              f'for request_id={self.perception_request} on /algorithm/em/mode_state')
                else:
                    self.fail('perception switch timeout; no matched subscriber on /algorithm/em/mode_command')
                return
            # Wait for discovery, then send exactly once; no automatic command retry.
            if not self.perception_sent and self.perception_pub.get_subscription_count() > 0:
                self.publish(self.perception_pub, {'command': 'set_mode', 'mode': self.perception_target,
                                                  'request_id': self.perception_request, 'force': False})
                self.perception_sent = True
                self.get_logger().info(f'[mapping] Sent perception set_mode={self.perception_target} '
                                       f'request_id={self.perception_request}')
        self.tick_call(now)
        if self.phase == 'PREPARING' and self.perception_ok and self.radar_ok:
            self.transition('STARTING')
            self.start_call(self.mapping, MapControl.Request.ACTION_MAPPING, 'mapping_start')
        if self.phase == 'MAPPING' and self.finished_stages == REQUIRED_MAP_STAGES:
            if self.artifact_pub.get_subscription_count() == 0:
                self.fail('cloud bridge artifact subscriber unavailable')
                return
            self.transition('UPLOADING')
            self.publish(self.artifact_pub, {'index': self.slot, 'start_msg_id': self.start_command['msg_id']})
        if self.phase == 'RESTORING_PERCEPTION' and self.perception_ok:
            self.transition('COMPLETE')
            self.get_logger().info('Workflow complete: Odin entered Relocation and loaded the map; localization pending verification')


def main():
    version_file = Path(__file__).with_name('version.py')
    version = run_path(str(version_file))['APP_VERSION'] if version_file.is_file() else 'unbuilt'
    print(f'[VERSION] fake_brain_mapping.py version={version}', flush=True)

    rclpy.init()
    node = FakeBrainMapping()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
