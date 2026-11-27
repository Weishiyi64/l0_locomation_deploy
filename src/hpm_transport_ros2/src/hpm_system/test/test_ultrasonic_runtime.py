#!/usr/bin/env python3
# Copyright (c) 2026 HPMicro
# SPDX-License-Identifier: BSD-3-Clause
"""Local UDP board simulator -> hpm_system -> Range; no hardware access."""
import math
import struct
import subprocess
import sys
import tempfile
import time

import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Range

from udp_test_peer import UdpPeer


def wire(raw=b'\x03\xe8\xeb', msg_id=0x50, port=6):
    frame = bytearray(struct.pack('!IBBHIBBBIB8s', 0x0a4d5048, 2, msg_id, 1, 0,
                                 0, port, 0xff, 0xffffffff, len(raw), raw))
    crc = 0xffff
    for byte in frame:
        crc ^= byte
        for _ in range(8):
            crc = (crc >> 1) ^ (0xa001 if crc & 1 else 0)
    return frame + struct.pack('!H', crc)


def main():
    rclpy.init()
    node = rclpy.create_node('ultrasonic_runtime_check')
    messages = []
    sub = node.create_subscription(Range, '/sensor/ultrasonic/range', messages.append,
                                   qos_profile_sensor_data)

    def spin(seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.02)

    with UdpPeer() as peer, tempfile.TemporaryFile(mode='w+') as log:
        def launch(enabled):
            return subprocess.Popen([sys.argv[1], '--ros-args',
                '-p', 'launch_motor:=false', '-p', 'launch_imu:=false',
                '-p', 'launch_tactile:=false', '-p', 'launch_battery:=false',
                '-p', f'launch_ultrasonic:={str(enabled).lower()}',
                '-p', 'enable_udp:=true', '-p', 'board_ip:=127.0.0.1',
                '-p', f'port:={peer.port}', '-p', 'ultrasonic_frame_id:=test_ultrasonic',
                '-p', 'ultrasonic_field_of_view:=1.3962634',
                '-p', 'ultrasonic_timeout_ms:=500'], stdout=log, stderr=log)

        process = launch(False)
        try:
            peer.wait_ready()
            spin(1.0)
            assert process.poll() is None
            assert node.count_publishers('/sensor/ultrasonic/range') == 0
            peer.send_frame(wire())
            spin(0.2)
            assert not messages
            process.terminate()
            process.wait(timeout=5)
            peer.ready.clear()
            process = launch(True)
            peer.wait_ready()
            spin(1.2)
            assert process.poll() is None
            assert node.count_publishers('/sensor/ultrasonic/range') == 1
            assert node.get_node_names().count('hpm_system') == 1

            # No valid measurement has arrived, including after startup timeout.
            invalid = [wire(b'\x03\xe8\xec'), wire(b'\x03\xe8'), wire(b'\xeb'),
                       wire(b'\x03\xe8\xeb\x00'), wire(msg_id=0x51), wire(port=5),
                       wire(b'\x00\x1d\x1d'), wire(b'\x07\x09\x10')]
            bad_crc = wire()
            bad_crc[-1] ^= 1
            for frame in invalid + [bad_crc]:
                peer.send_frame(frame)
                spin(0.03)
            spin(0.2)
            assert not messages, 'Invalid sensor or outer CRC frame was published'

            for raw, expected in [(b'\x00\x1e\x1e', .03),
                                  (b'\x03\xe8\xeb', 1.),
                                  (b'\x07\x08\x0f', 1.8)]:
                before = len(messages)
                peer.send_frame(wire(raw))
                spin(0.2)
                assert len(messages) == before + 1
                msg = messages[-1]
                assert math.isclose(msg.range, expected, abs_tol=1e-6)
                assert msg.radiation_type == Range.ULTRASOUND
                assert msg.header.frame_id == 'test_ultrasonic'
                assert msg.header.stamp.sec > 0
                assert math.isclose(msg.field_of_view, 1.3962634, abs_tol=1e-6)
                assert math.isclose(msg.min_range, .03, abs_tol=1e-6)
                assert math.isclose(msg.max_range, 1.8, abs_tol=1e-6)
            before = len(messages)
            for _ in range(10):
                peer.send_frame(wire())
                spin(0.1)
            assert len(messages) == before + 10, '10 Hz input must publish one sample per frame'
            spin(0.7)
            before = len(messages)
            spin(0.6)
            assert len(messages) == before, 'Stale measurements must not be republished'
            peer.send_frame(wire())
            spin(0.2)
            assert len(messages) == before + 1

            # A board RESET requests a new session (CLOSE is sent by the host).
            with peer._lock:
                peer.ready.clear()
                peer._send_control(8, peer._session)  # UDP_WIRE_RESET
            peer.wait_ready()
            spin(0.2)
            peer.send_frame(wire())
            spin(0.2)
            assert len(messages) == before + 2
            process.terminate()
            process.wait(timeout=5)
            log.seek(0)
            output = log.read()
            assert 'FD07-3 valid data timeout' in output
            assert 'FD07-3 valid data recovered' in output
            print('PASS: module switch, raw UDP/CRC, invalid burst rejection, Range fields, '
                  'timeout/recovery, reconnect, no stale publication')
        except Exception:
            log.seek(0)
            print(log.read(), file=sys.stderr)
            raise
        finally:
            if process.poll() is None:
                process.terminate()
                process.wait(timeout=5)
            node.destroy_subscription(sub)
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()
