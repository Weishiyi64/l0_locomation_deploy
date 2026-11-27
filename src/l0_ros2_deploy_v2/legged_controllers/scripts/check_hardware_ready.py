#!/usr/bin/env python3
"""Wait for motor feedback readiness and enabled sensors before starting policy nodes."""

import argparse
import time
from pathlib import Path
from runpy import run_path

import rclpy
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from luvbot_hardware_interfaces.msg import BatteryState
from sensor_msgs.msg import Imu
from std_msgs.msg import Bool


def main():
    version_file = Path(__file__).with_name('version.py')
    version = run_path(str(version_file))['APP_VERSION'] if version_file.is_file() else 'unbuilt'
    print(f'[VERSION] check_hardware_ready.py version={version}', flush=True)

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('battery_enabled', nargs='?', choices=('true', 'false'), default='true')
    parser.add_argument('imu_enabled', nargs='?', choices=('true', 'false'), default='true')
    args = parser.parse_args()

    rclpy.init(args=[])
    node = rclpy.create_node('check_hardware_ready')
    ready = {
        '/hardware_status/motors_ready': False,
        '/imu/data': args.imu_enabled == 'false',
        '/battery/state': args.battery_enabled == 'false',
    }

    def motors_received(msg):
        ready['/hardware_status/motors_ready'] = bool(msg.data)

    def imu_received(_msg):
        ready['/imu/data'] = True

    def battery_received(_msg):
        ready['/battery/state'] = True

    try:
        motors_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        node.create_subscription(Bool, '/hardware_status/motors_ready', motors_received, motors_qos)
        if args.imu_enabled == 'true':
            node.create_subscription(Imu, '/imu/data', imu_received, qos_profile_sensor_data)
        if args.battery_enabled == 'true':
            node.create_subscription(BatteryState, '/battery/state', battery_received, qos_profile_sensor_data)

        pending = ', '.join(topic for topic, received in ready.items() if not received)
        node.get_logger().info(f'Waiting up to 20s for: {pending}')
        deadline = time.monotonic() + 20.0
        while rclpy.ok():
            remaining = deadline - time.monotonic()
            if remaining <= 0.0:
                break
            rclpy.spin_once(node, timeout_sec=min(0.1, remaining))
            if all(ready.values()):
                node.get_logger().info('Hardware readiness check passed')
                return 0

        pending = ', '.join(topic for topic, received in ready.items() if not received)
        node.get_logger().error(f'Hardware readiness check failed; still waiting for: {pending}')
        return 1
    except KeyboardInterrupt:
        return 130
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
