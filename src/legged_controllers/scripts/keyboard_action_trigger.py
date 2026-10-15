#!/usr/bin/env python3
"""Publish a configured action name when a configured command line is entered."""

import os
import select
import sys

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from std_msgs.msg import String


class KeyboardActionTrigger(Node):
    def __init__(self) -> None:
        super().__init__("keyboard_action_trigger")
        self.declare_parameter("config_path", "")
        self.declare_parameter("trigger_topic", "/keyboard_action_trigger")

        config_param = self.get_parameter("config_path").get_parameter_value().string_value
        topic_param = self.get_parameter("trigger_topic").get_parameter_value().string_value
        self.command_to_action = self.load_bindings(config_param)
        self.publisher = self.create_publisher(String, topic_param or "/keyboard_action_trigger", 10)

        bindings = ", ".join(
            f"{command}->{action_name}" for command, action_name in sorted(self.command_to_action.items())
        )
        self.get_logger().info(
            f"Keyboard action trigger publishing action names on {topic_param or '/keyboard_action_trigger'}; "
            f"enter a command and press Enter; bindings: {bindings or 'none'}"
        )

    def resolve_config_path(self, raw_path: str) -> str:
        if not raw_path:
            return ""
        if os.path.isabs(raw_path):
            return raw_path
        try:
            return os.path.join(get_package_share_directory("legged_controllers"), raw_path)
        except Exception:
            return raw_path

    def load_bindings(self, raw_config_path: str) -> dict:
        config_path = self.resolve_config_path(raw_config_path)
        if not config_path:
            self.get_logger().error("keyboard_action_trigger.py requires config_path")
            return {}
        try:
            with open(config_path, "r", encoding="utf-8") as stream:
                root = yaml.safe_load(stream) or {}
        except Exception as exc:
            self.get_logger().error(f"Failed to load keyboard action config {config_path}: {exc}")
            return {}

        cfg = root.get("keyboard_actions", root)
        actions = cfg.get("actions", [])
        if not isinstance(actions, list):
            self.get_logger().error(f"keyboard action config {config_path} must contain actions: []")
            return {}

        command_to_action = {}
        for entry in actions:
            if not isinstance(entry, dict):
                self.get_logger().warning("Skipping keyboard action entry that is not a map")
                continue
            command = str(entry.get("key", "")).strip()
            action_name = str(entry.get("name") or command).strip()
            if not command or not action_name:
                self.get_logger().warning("Skipping keyboard action entry without key/name")
                continue
            command_to_action[command] = action_name
        return command_to_action

    def publish_trigger(self, action_name: str) -> None:
        msg = String()
        msg.data = action_name
        self.publisher.publish(msg)
        self.get_logger().info(f"Published keyboard action trigger: {action_name}")


def main() -> None:
    rclpy.init()
    node = KeyboardActionTrigger()

    input_stream = sys.stdin
    close_input_stream = False
    if not input_stream.isatty():
        try:
            input_stream = open("/dev/tty", "r")
            close_input_stream = True
        except OSError as exc:
            node.get_logger().error(f"keyboard_action_trigger.py requires an interactive terminal: {exc}")
            node.destroy_node()
            rclpy.shutdown()
            return

    if not input_stream.isatty():
        node.get_logger().error("keyboard_action_trigger.py requires an interactive terminal")
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        return

    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.0)
            readable, _, _ = select.select([input_stream], [], [], 0.1)
            if not readable:
                continue
            command = input_stream.readline()
            if command == "":
                break
            command = command.strip()
            if not command:
                continue
            action_name = node.command_to_action.get(command)
            if action_name:
                node.publish_trigger(action_name)
            else:
                node.get_logger().warning(f"Ignoring unknown keyboard action command: {command}")
    except KeyboardInterrupt:
        pass
    finally:
        if close_input_stream:
            input_stream.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
