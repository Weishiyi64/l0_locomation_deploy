#!/usr/bin/env python3
# Copyright (c) 2026 HPMicro
# SPDX-License-Identifier: BSD-3-Clause
"""Compare the compiled decoder to signal definitions read from the supplier DBC.

Run: python3 test_dbc_reference.py /path/to/test_battery /path/to/vendor.dbc
Only Python stdlib and the ROS environment's PyYAML are used. No hardware access.
"""
import math
from pathlib import Path
import random
import re
import subprocess
import sys

import yaml


def snake(name):
    name = re.sub(r'(.)([A-Z][a-z]+)', r'\1_\2', name)
    return re.sub('_+', '_', re.sub(r'([a-z0-9])([A-Z])', r'\1_\2', name)).lower()


def main():
    binary, dbc = sys.argv[1:]
    messages = {}
    current = None
    signal_pattern = re.compile(r'\s*SG_ (\w+) : (\d+)\|(\d+)@(\d)([+-]) '
                                r'\(([^,]+),([^\)]+)\) \[([^|]+)\|([^\]]+)\]')
    for line in Path(dbc).read_bytes().decode('gb18030').splitlines():
        match = re.match(r'BO_ (\d+) ', line)
        if match:
            current = int(match[1])
            if 0x210 <= current <= 0x21e:
                messages[current] = []
        match = signal_pattern.match(line)
        if match and current in messages:
            name, start, width, order, sign, scale, offset, low, high = match.groups()
            assert order == '1', 'Reference checker intentionally covers this little-endian DBC only'
            messages[current].append((name, int(start), int(width), sign,
                                      float(scale), float(offset), float(low), float(high)))
    aliases = {'BMS_PackStatus_SOC': 'soc', 'BMS_PackStatus_SOH': 'soh',
               'BMS_PackStatus_SOP': 'sop', 'BMS_PackStatus_Voltage': 'voltage',
               'BMS_PackStatus_Current': 'current', 'BMS_Status_PortState': 'charge_state',
               'BMS_Status_IsFullCharge': 'full_charge',
               'ChgMos2State': 'bms_status_charge_mos2_state',
               'DchMos2State': 'bms_status_discharge_mos2_state',
               'ChgMos3State': 'bms_status_charge_mos3_state',
               'DchMos3State': 'bms_status_discharge_mos3_state',
               'PreMosState': 'bms_status_pre_mos_state'}
    rng = random.Random(0x215)
    ids = sorted(messages)
    cases = [{i: bytes(8) for i in ids}, {i: bytes([255] * 8) for i in ids}]
    # Walk every payload bit independently, then use distinct bytes per message to catch routing errors.
    cases += [{i: (1 << bit).to_bytes(8, 'little') for i in ids} for bit in range(64)]
    cases += [{i: bytes(rng.randrange(256) for _ in range(8)) for i in ids} for _ in range(32)]
    captures = {0x211: '1a 61 00 00 00 01 01 00',
                0x215: '28 64 57 00 aa 0f eb 7f', 0x21c: '00 00 02 04 00 00 96 01',
                0x21d: '00 02 00 00 00 95 01 01', 0x21e: '4d 4e 50 50 50 00 00 00'}
    cases.append({i: bytes.fromhex(captures[i]) if i in captures else bytes(8) for i in ids})
    checks = 0
    coverage = set()
    for number, frames in enumerate(cases):
        records = [(i, 99000, d) for i, d in frames.items() if i != 0x212]
        # Raw serial slots still arrive, but no serial fields may appear in ROS.
        for part in range(4):
            records.append((0x212, 90000 + part, bytes([0x40 | part]) + frames[0x212][1:]))
        text = str(len(records)) + '\n' + ''.join(
            f'{i:x} {timestamp} ' + d.hex(' ') + '\n' for i, timestamp, d in records)
        result = subprocess.run([binary, '--decode'], input=text, text=True,
                                capture_output=True, check=True)
        actual = yaml.safe_load(result.stdout)
        expected_fields = set()
        for identifier, signals in messages.items():
            if identifier == 0x212:
                continue
            word = int.from_bytes(frames[identifier], 'little')
            for name, start, width, sign, scale, offset, low, high in signals:
                field = aliases.get(name, snake(name))
                expected_fields.add(field)
                assert field in actual, (name, field, 'missing ROS field')
                raw = (word >> start) & ((1 << width) - 1)
                if sign == '-' and raw & (1 << (width - 1)):
                    raw -= 1 << width
                expected = raw * scale + offset
                value = actual[field]
                if isinstance(value, str) and value.lower() in ('nan', '.nan'):
                    value = float('nan')
                if isinstance(value, float):
                    if expected < low - 1e-8 or expected > high + 1e-8:
                        assert math.isnan(value), (number, name, expected, value)
                    else:
                        assert math.isclose(value, expected, rel_tol=2e-6, abs_tol=2e-4), \
                            (number, name, expected, value)
                elif field == 'bms_capacity_cycle_count' and expected > high:
                    assert value == 0xffffffff
                else:
                    assert value == expected, (number, name, expected, value)
                checks += 1
        assert set(actual) == expected_fields | {'header'}, ('extra or missing fields', set(actual) ^ (expected_fields | {'header'}))
        coverage |= expected_fields
    print(f'PASS: {len(messages)} DBC messages, {len(coverage)} ROS battery fields, '
          f'{len(cases)} reference vectors, {checks} scalar checks; 0x212 ignored')


if __name__ == '__main__':
    main()
