#!/usr/bin/env python3
"""Stationary HPM-protocol endpoint for the unchanged real deployment control loop.

Only /wbc_isolated endpoints are created. No motor transport is imported or opened.
Motor readiness, watchdogs and command composition remain in LeggedReal.
"""
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, qos_profile_sensor_data
from robot_interfaces.msg import MotorCommand, MotorEnableCommand, MotorState, MotorStateArray
from robot_interfaces.srv import MotorSetZero
from sensor_msgs.msg import Imu
import yaml


class IsolatedHpm(Node):
    def __init__(self):
        super().__init__('isolated_hpm')
        self.declare_parameter('mapping_file', '')
        self.declare_parameter('robot_cfg_path', '')
        self.declare_parameter('feedback_enabled', True)
        self.declare_parameter('imu_feedback_enabled', True)
        with open(self.get_parameter('mapping_file').value, encoding='utf-8') as stream:
            mapping = yaml.safe_load(stream)
        with open(self.get_parameter('robot_cfg_path').value, encoding='utf-8') as stream:
            cfg = yaml.safe_load(stream)['RobotCfg']
        joints, positions = cfg['joint_order_mj'], cfg['default_joint_angles']
        if len(joints) != 21 or len(positions) != 21 or len(mapping['motors']) != 21:
            raise ValueError('Isolated HPM requires the same 21-joint hardware configuration')
        self.state = MotorStateArray()
        self.enabled = [False] * 21
        for index, (name, position, entry) in enumerate(zip(joints, positions, mapping['motors'])):
            model = mapping['models'][entry['model']]
            direction = float(entry['motor_direction']) * float(model['rotation_direction'])
            reduction = float(entry['reduction'])
            if name != entry['joint_name'] or not math.isfinite(position) or abs(direction) != 1 or reduction <= 0:
                raise ValueError('Joint/mapping mismatch in isolated HPM configuration')
            motor_q = (position - float(entry['zero_offset'])) * direction * reduction
            if not math.isfinite(motor_q) or not model['p_min'] <= motor_q <= model['p_max'] or \
                    not entry['joint_lower'] <= position <= entry['joint_upper']:
                raise ValueError(f'Isolated default pose is outside mapping limits: {name}')
            state = MotorState()
            state.route.motor_type = index
            state.q = motor_q
            state.valid = True
            state.operating_mode = MotorState.OPERATING_MODE_UNKNOWN
            self.state.states.append(state)
            self.state.feedback_mask |= 1 << index
        reliable = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE,
                              durability=DurabilityPolicy.VOLATILE)
        self.state_pub = self.create_publisher(MotorStateArray, '/wbc_isolated/actuator/motor/state', qos_profile_sensor_data)
        self.imu_pub = self.create_publisher(Imu, '/wbc_isolated/imu/data', qos_profile_sensor_data)
        self.enable_sub = self.create_subscription(MotorEnableCommand, '/wbc_isolated/actuator/motor/enable',
                                                   self.enable, reliable)
        self.command_sub = self.create_subscription(MotorCommand, '/wbc_isolated/actuator/motor/command',
                                                    self.command, reliable)
        self.zero_service = self.create_service(MotorSetZero, '/wbc_isolated/actuator/motor/set_zero', self.reject_zero)
        self.command_count = 0
        self.last_summary_ns = 0
        self.timer = self.create_timer(0.002, self.publish_feedback)
        self.get_logger().warning('HPM ISOLATED: stationary feedback; commands are consumed without motor transport')

    def enable(self, msg):
        if len(msg.motor_types) != len(msg.enabled) or any(i >= 21 for i in msg.motor_types):
            self.get_logger().error('Invalid isolated enable request')
            return
        for index, enabled in zip(msg.motor_types, msg.enabled):
            self.enabled[index] = enabled
        self.get_logger().info(f'Isolated enable sequence={msg.sequence_id}; enabled joints={sum(self.enabled)}')

    def command(self, msg):
        for target in msg.commands:
            if target.route.motor_type >= 21 or not all(math.isfinite(x) for x in
                                                       (target.q, target.dq, target.tau, target.kp, target.kd)):
                self.get_logger().error('Invalid isolated motor command')
                return
        self.command_count += 1
        now = self.get_clock().now().nanoseconds
        if now - self.last_summary_ns >= 5_000_000_000:
            self.last_summary_ns = now
            self.get_logger().info(f'Isolated sink received={self.command_count}, sequence={msg.sequence_id}, '
                                   f'joints={len(msg.commands)}; stationary feedback')

    @staticmethod
    def reject_zero(request, response):
        response.motor_types = request.motor_types
        response.results = [MotorSetZero.Response.REJECTED] * len(request.motor_types)
        response.messages = ['Calibration is unavailable in isolated mode'] * len(request.motor_types)
        return response

    def publish_feedback(self):
        stamp = self.get_clock().now().to_msg()
        if self.get_parameter('feedback_enabled').value:
            self.state.stamp = stamp
            self.state.sequence_id = (self.state.sequence_id + 1) & 0xFFFFFFFF
            self.state_pub.publish(self.state)
        if self.get_parameter('imu_feedback_enabled').value:
            imu = Imu()
            imu.header.stamp = stamp
            imu.header.frame_id = 'imu_link'
            imu.orientation.w = 1.0
            imu.linear_acceleration.z = 9.81
            self.imu_pub.publish(imu)


def main():
    rclpy.init()
    node = None
    try:
        node = IsolatedHpm()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
