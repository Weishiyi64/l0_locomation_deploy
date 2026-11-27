#!/usr/bin/env python3
# Copyright (c) 2026 HPMicro
# SPDX-License-Identifier: BSD-3-Clause
"""Offline workflow/configuration tests; no ROS or motor connection required."""

import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch

import motor_id_calibration as tool


def response(motor, ids=None, result=0, power_cycle=False, **changes):
    fields = dict(result=result, stage="test", message="test", motor_name=motor.name,
                  motor_type=motor.protocol, model=motor.model, bus_index=motor.bus,
                  target_id=motor.target_id, detected_ids=ids or [motor.target_id],
                  power_cycle_required=power_cycle)
    fields.update(changes)
    return SimpleNamespace(**fields)


class FakeClient:
    def __init__(self, responses):
        self.responses = iter(responses)
        self.calls = []

    def call(self, motor, operation, current_id):
        self.calls.append((motor.index, operation, current_id))
        result = next(self.responses)
        tool.check_metadata(motor, result)
        return result


class CalibrationTest(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.logger = tool.JsonlLogger(Path(self.tmp.name))
        self.motor = tool.Motor("left_hip_roll_joint", 1, "MR", "PA43-10-40", 0, 2)

    def configure(self, client, answers=("yes", "yes")):
        answers = iter(answers)
        return tool.configure(client, self.motor, 1, self.logger,
                              prompt=lambda _: next(answers), output=lambda _: None)

    def test_power_cycle_and_three_queries_required(self):
        client = FakeClient([response(self.motor, [1]),
                             response(self.motor, power_cycle=True)] +
                            [response(self.motor)] * 3)
        self.assertTrue(self.configure(client))
        self.assertEqual(client.calls, [(1, tool.QUERY, 1), (1, tool.CONFIGURE, 1)] +
                         [(1, tool.QUERY, 2)] * 3)
        self.assertIn('"passed": true', self.logger.path.read_text())

    def test_failure_cancel_and_unverified_power_cycle(self):
        for replies, answers, expected_calls in (
            ([response(self.motor, [1], result=4)], (), 1),
            ([response(self.motor, [1])], ("no",), 1),
            ([response(self.motor, [1]), response(self.motor, result=4)], ("yes",), 2),
            ([response(self.motor, [1]), response(self.motor, power_cycle=True)], ("yes", "no"), 2),
        ):
            client = FakeClient(replies)
            self.assertFalse(self.configure(client, answers))
            self.assertEqual(len(client.calls), expected_calls)

    def test_config_mismatch_stops_before_write(self):
        client = FakeClient([response(self.motor, [1], bus_index=3)])
        with self.assertRaisesRegex(RuntimeError, "配置不一致"):
            self.configure(client)
        self.assertEqual(len(client.calls), 1)

    def test_post_cycle_missing_or_unexpected_id_fails(self):
        for bad in (response(self.motor, [1]), response(self.motor, [2, 3]),
                    response(self.motor, result=4)):
            client = FakeClient([response(self.motor, [1]),
                                 response(self.motor, power_cycle=True),
                                 response(self.motor), bad, response(self.motor)])
            self.assertFalse(self.configure(client))

    def test_ls_inventory_grouped_by_bus_and_partial_failure(self):
        ls = [tool.Motor(f"ls{i}", i, "LS", "", 1, i + 10) for i in range(3)]
        rs = tool.Motor("rs", 3, "RS", "", 0, 1)
        client = FakeClient([response(ls[0], [10, 11, 12])] * 3 +
                            [response(rs, result=4)] * 3)
        self.assertFalse(tool.check_all(client, ls + [rs], output=lambda _: None))
        self.assertEqual(len(client.calls), 6)
        self.assertEqual([item[0] for item in client.calls], [0, 0, 0, 3, 3, 3])

    def test_ls_individual_counts_and_final_lists(self):
        ls = [tool.Motor(f"ls{i}", i, "LS", "", 1, i + 0x41) for i in range(3)]
        for inventories, passed in (([[65]] * 3, 1),
                                     ([[65, 66], [65], [65, 66]], 1),
                                     ([[65, 66, 67]] * 3, 3),
                                     ([[]] * 3, 0)):
            output = []
            client = FakeClient([response(ls[0], detected_ids=ids) for ids in inventories])
            self.assertEqual(tool.check_all(client, ls, output=output.append), passed == 3)
            summary = output[output.index(f"自检结果: {passed}/3 个电机 PASS"):]
            self.assertEqual(summary.count(f"  {tool.describe(ls[0])}"), 1)
            self.assertIn(f"PASS 电机 ({passed}):", summary)
            self.assertIn(f"FAIL 电机 ({3 - passed}):", summary)
            if passed:
                self.assertIn(f"PASS: {tool.describe(ls[0])}", output)
            self.assertEqual(len(client.calls), 3)

    def test_ls_timeout_and_unexpected_inventory(self):
        ls = tool.Motor("ls", 0, "LS", "", 1, 65)
        for replies, expected_pass in (([response(ls)] + [response(ls, result=4)] + [response(ls)], 0),
                                       ([response(ls, [65, 99])] * 3, 1)):
            output = []
            self.assertFalse(tool.check_all(FakeClient(replies), [ls], output=output.append))
            self.assertIn(f"自检结果: {expected_pass}/1 个电机 PASS", output)

    def test_discover_ls_broadcast_and_ambiguity(self):
        ls = tool.Motor("neck", 18, "LS", "", 1, 65)
        for ids, expected in (([1], True), ([1, 65], False), ([], False)):
            output = []
            client = FakeClient([response(ls, detected_ids=ids)])
            self.assertEqual(tool.discover_ids(client, ls, output.append), expected)
            self.assertEqual(client.calls, [(18, tool.QUERY, 65)])
            self.assertIn("检测到的当前 ID: " + (", ".join(f"0x{i:02X}" for i in ids) or "无"), output)

    def test_discover_rs_mr_full_scan_and_errors(self):
        for protocol in ("RS", "MR"):
            motor = tool.Motor("joint", 1, protocol, self.motor.model if protocol == "MR" else "", 0, 2)
            candidates = list(dict.fromkeys([2, tool.FACTORY_IDS[protocol]] + list(range(1, 256))))
            replies = [response(motor, detected_ids=[i], result=0 if i == 255 else 4)
                       for i in candidates]
            output = []
            client = FakeClient(replies)
            self.assertTrue(tool.discover_ids(client, motor, output.append))
            self.assertEqual({call[2] for call in client.calls}, set(range(1, 256)))
            self.assertTrue(all(call[1] == tool.QUERY for call in client.calls))
            self.assertIn("检测到的当前 ID: 0xFF", output)
            client = FakeClient([response(motor, result=2)])
            self.assertFalse(tool.discover_ids(client, motor, lambda _: None))
            self.assertEqual(len(client.calls), 1)

    def test_models_and_config_validation(self):
        path = Path(self.tmp.name) / "motors.yaml"
        for model in sorted(tool.MR_MODELS):
            path.write_text(f"motors:\n  mr: {{index: 0, motor_type: MR, model: {model}, bus_index: 0, motor_id: 0x02}}\n")
            self.assertEqual(tool.load_motors(path)[0].model, model)
        for model in ("", "unknown"):
            path.write_text(f"motors:\n  mr: {{index: 0, motor_type: MR, model: '{model}', bus_index: 0, motor_id: 2}}\n")
            with self.assertRaises(ValueError):
                tool.load_motors(path)
        path.write_text("motors:\n  a: {index: 0, motor_type: LS, bus_index: 0, motor_id: 1}\n  b: {index: 1, motor_type: RS, bus_index: 0, motor_id: 1}\n")
        with self.assertRaisesRegex(ValueError, "重复"):
            tool.load_motors(path)

    def test_dry_run_does_not_create_client_or_log(self):
        with patch.object(tool, "RosClient", side_effect=AssertionError("ROS accessed")), \
                patch.object(tool, "JsonlLogger", side_effect=AssertionError("Log written")), \
                patch("builtins.print"):
            self.assertEqual(tool.main(["--dry-run", "--motor-index", "1"]), 0)


if __name__ == "__main__":
    unittest.main()
