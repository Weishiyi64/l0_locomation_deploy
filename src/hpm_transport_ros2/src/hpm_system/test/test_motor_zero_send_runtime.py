#!/usr/bin/env python3
# Copyright (c) 2026 HPMicro
# SPDX-License-Identifier: BSD-3-Clause
"""Loopback simulator only: dispatch returns without any motor acknowledgement."""
import sys
import time

import rclpy
from luvbot_hardware_interfaces.srv import MotorSetZero
from test_motor_id_calibration_runtime import session, receive, result, no_send, can_id


def run(executable):
    rclpy.init()
    try:
        with session(executable, False) as (node, _, conn, __):
            send = node.create_client(MotorSetZero, "/actuator/motor/set_zero_send")
            assert send.wait_for_service(timeout_sec=4)
            for _ in range(3):
                started = time.monotonic()
                future = send.call_async(MotorSetZero.Request(motor_types=[1, 2]))
                assert can_id(receive(conn)) == 0x102
                assert can_id(receive(conn)) == 0x7FF
                response = result(node, future)  # Deliberately send no device feedback.
                assert list(response.results) == [0, 0], response
                assert all("not verified" in m for m in response.messages)
                assert time.monotonic() - started < 0.2
            no_send(conn)
            rejected = result(node, send.call_async(MotorSetZero.Request(motor_types=[1, 1])))
            assert list(rejected.results) == [1, 1]
            no_send(conn)
            original = node.create_client(MotorSetZero, "/actuator/motor/set_zero")
            assert original.wait_for_service(timeout_sec=4)
            future = original.call_async(MotorSetZero.Request(motor_types=[1]))
            receive(conn)
            response = result(node, future)
            assert response.results[0] == MotorSetZero.Response.TIMEOUT
            print("PASS: repeated send-only requests return without feedback; original service still verifies")
        with session(executable, True) as (node, _, conn, __):
            send = node.create_client(MotorSetZero, "/actuator/motor/set_zero_send")
            assert send.wait_for_service(timeout_sec=4)
            response = result(node, send.call_async(MotorSetZero.Request(motor_types=[1])))
            assert list(response.results) == [1]
            no_send(conn)
            print("PASS: send-only service respects ID commissioning mode")
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    run(sys.argv[1])
