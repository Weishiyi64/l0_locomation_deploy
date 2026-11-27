#!/usr/bin/env python3
"""Local UDP -> hpm_system -> BatteryState integration; never addresses hardware."""
import os
import socket
import struct
import subprocess
import sys
import tempfile
import threading
import time

import rclpy
from rclpy.qos import qos_profile_sensor_data
from luvbot_hardware_interfaces.msg import BatteryState


def crc(data):
    result = 0xffff
    for byte in data:
        result ^= byte
        for _ in range(8):
            result = (result >> 1) ^ (0xa001 if result & 1 else 0)
    return result


def packet(session, sequence, snapshot_sequence, kind=11):
    payload = bytearray(689)
    struct.pack_into('!BBBBIIII', payload, 0, 1, 29, 6, 0, snapshot_sequence,
                     sequence * 100, (1 << 1) | (1 << 4), (1 << 1) | (1 << 4))
    for index, can_id, raw in [
        (1, 0x211, bytes([snapshot_sequence & 255, 0x62, 0, 0, 0, 1, 1, 0])),
        (4, 0x215, bytes.fromhex('50 64 f4 01 a0 0f f4 81')),
    ]:
        offset = 20 + index * 23
        struct.pack_into('!BHI BBBIB', payload, offset, 0x40, sequence & 65535,
                         0, 0, 6, 0, can_id, 8)
        payload[offset + 15:offset + 23] = raw
    header = struct.pack('!4sBBHQI', b'HPU1', kind, 29, len(payload), session, sequence)
    result = bytearray(header + payload)
    struct.pack_into('!H', result, len(result) - 2, crc(result[:-2]))
    return result


def main():
    assert crc(b'123456789') == 0x4b37
    fields = BatteryState.get_fields_and_field_types()
    removed = ['bms_serial_number_index', 'bms_serial_number_total', 'bms_serial_number_data', 'age_ms', 'board_snapshot_ms', 'fresh_mask', 'online', 'raw_data', 'raw_dlc', 'received_mask', 'serial_complete', 'serial_fragment_mask', 'serial_index_base', 'snapshot_sequence', 'update_count', 'updated_mask']
    assert not set(removed).intersection(fields)
    assert list(fields)[:8] == ['header', 'soc', 'soh', 'sop', 'charge_state', 'full_charge', 'voltage', 'current']
    assert all(name in ('header', 'soc', 'soh', 'sop', 'charge_state', 'full_charge', 'voltage', 'current') or name.startswith(('bms_', 'bcu_')) or
               name in ('key_wakeup', 'chg_det_wakeup', 'io_det_wakeup', 'sum_volt', 'asw_soc_get_clbrt_flag', 'get_bms_fault_level', 'get_sys_state', 'get_step', 'get_fault_list', 'get_fault_num', 'realsoc', 'ex_volt2', 'chg_volt', 'inter_volt', 'get_sys_key_io', 'get_chg_wake_up') for name in fields)
    assert 'voltage' in fields
    assert all(f'bcu_vol_{i}' in fields for i in range(1, 12))
    assert all(f'bms_temp_cell{i}' in fields for i in range(1, 3))
    binary = sys.argv[1]
    rclpy.init()
    node = rclpy.create_node('battery_runtime_check')
    messages = []
    subscription = node.create_subscription(BatteryState, '/battery/state',
                                            lambda m: messages.append((time.monotonic(), m)),
                                            qos_profile_sensor_data)
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(('127.0.0.1', 0)); sock.settimeout(.02)
    stop = threading.Event()
    errors = []

    def board():
        session = peer = None
        next_send = 0.0
        sequence = 0
        while not stop.is_set():
            try:
                data, sender = sock.recvfrom(2048)
                if len(data) >= 20:
                    _, kind, _, _, sid, _ = struct.unpack('!4sBBHQI', data[:20])
                    if kind == 2:
                        session, peer = sid, sender
                        sock.sendto(struct.pack('!4sBBHQI', b'HPU1', 3, 0, 0, sid, 0), peer)
                        # Invalid packets must not consume sequence1 or publish sentinel states.
                        bad = packet(sid + 1, 1, 0xdeadbeef); sock.sendto(bad, peer)
                        bad = packet(sid, 1, 0xdeadbeef); bad[-1] ^= 1; sock.sendto(bad, peer)
                        for offset, value in [(20, 2), (20 + 12, 0x80),
                                              (20 + 20 + 4 * 23 + 13, 0x16)]:
                            bad = packet(sid, 1, 0xdeadbeef); bad[offset] = value
                            struct.pack_into('!H', bad, len(bad)-2, crc(bad[:-2]))
                            sock.sendto(bad, peer)
                    elif kind == 4 and sid == session:
                        sock.sendto(struct.pack('!4sBBHQI', b'HPU1', 5, 0, 0, sid, 0), sender)
            except socket.timeout:
                pass
            except Exception as exc:
                errors.append(exc); return
            now = time.monotonic()
            if peer and now >= next_send:
                sequence += 1; next_send = now + .1
                good = packet(session, sequence, sequence)
                sock.sendto(good, peer)
                sock.sendto(good, peer)  # Duplicate must not publish twice.
                if sequence > 1:
                    sock.sendto(packet(session, sequence - 1, 0xdeadbeef), peer)

    def spin(seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=.05)

    def launch(enabled, log):
        return subprocess.Popen([binary, '--ros-args', '-p', 'launch_motor:=false',
            '-p', 'launch_imu:=false', '-p', 'launch_tactile:=false',
            '-p', f'launch_battery:={str(enabled).lower()}',
            '-p', f'enable_udp:={str(enabled).lower()}', '-p', 'board_ip:=127.0.0.1',
            '-p', f'port:={sock.getsockname()[1]}'], stdout=log, stderr=log)

    with tempfile.TemporaryFile(mode='w+') as log:
        process = launch(False, log)
        worker = threading.Thread(target=board, daemon=True)
        try:
            spin(1.5)
            assert process.poll() is None
            assert node.count_publishers('/battery/state') == 0
            process.terminate(); process.wait(timeout=5)
            process = launch(True, log); worker.start()
            spin(4)
            assert process.poll() is None and not errors
            assert len(messages) >= 20, len(messages)
            # Use the simulated BMS heartbeat to check packet filtering without ROS diagnostics.
            seqs = [m.bms_alive_cnt4_charger for _, m in messages]
            assert len(seqs) == len(set(seqs)) and 0xef not in seqs
            assert all(abs(m.voltage - 40) < .001 and
                       abs(m.current - 5) < .001 for _, m in messages)
            assert all(m.header.stamp.sec > 0 for _, m in messages)
            hz = (len(messages) - 1) / (messages[-1][0] - messages[0][0])
            assert 8.5 <= hz <= 11.5, hz
            stop.set(); worker.join(timeout=2)
            spin(.3); before = len(messages); spin(.7)
            assert len(messages) == before, 'ROS must not fabricate periodic snapshots after UDP stops'
            print(f'PASS: launch switch; {len(messages)} unique snapshots at {hz:.2f}Hz; '
                  'CRC/layout/session/duplicate/reorder rejection; no fabricated samples')
        except Exception:
            log.seek(0); print(log.read(), file=sys.stderr)
            raise
        finally:
            stop.set()
            if worker.is_alive(): worker.join(timeout=2)
            if process.poll() is None: process.terminate(); process.wait(timeout=5)
            sock.close()
            node.destroy_subscription(subscription); node.destroy_node(); rclpy.shutdown()


if __name__ == '__main__':
    main()
