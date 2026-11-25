#!/usr/bin/env python3
"""Send one identified text/manual-token request and await its terminal result."""
import argparse
import json
import math
import time
import uuid

import rclpy
from rclpy.duration import Duration
from rclpy.signals import SignalHandlerOptions
from std_msgs.msg import String


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--text")
    parser.add_argument("--tokens", nargs=3, type=int, metavar=("HEAD", "LEFT", "RIGHT"))
    parser.add_argument("--speed", type=float, default=1.2)
    parser.add_argument("--seed", type=int)
    parser.add_argument("--return-frames", type=int, choices=(-1, 30, 60, 90), default=-1)
    parser.add_argument("--no-return", action="store_true")
    parser.add_argument("--limit-policy", choices=("reject", "clip"), default="reject")
    parser.add_argument("--cancel", metavar="REQUEST_ID")
    parser.add_argument("--timeout", type=float, default=120.0)
    args = parser.parse_args()
    if not math.isfinite(args.timeout) or args.timeout <= 0:
        parser.error("timeout must be finite and positive")
    if not args.cancel and (bool(args.text) == (args.tokens is not None)):
        parser.error("provide exactly one of --text or --tokens")
    identifier = args.cancel or uuid.uuid4().hex
    request = {"command": "cancel" if args.cancel else "generate", "request_id": identifier}
    if not args.cancel:
        request.update(mode="web" if args.tokens is not None else "text", text=args.text or "",
                       token_ids=args.tokens, speed=args.speed, seed=args.seed,
                       return_home=not args.no_return, return_frames=args.return_frames, limit_policy=args.limit_policy)
    rclpy.init(args=[], signal_handler_options=SignalHandlerOptions.NO)
    node = rclpy.create_node("text_motion_request_" + uuid.uuid4().hex[:8])
    events = []
    node.create_subscription(String, "/text_motion/status", lambda msg: events.append(msg.data), 20)
    publisher = node.create_publisher(String, "/text_motion/request", 10)

    def publish(value):
        message = String()
        message.data = json.dumps(value, ensure_ascii=False, allow_nan=False)
        publisher.publish(message)

    try:
        deadline = time.monotonic() + 10
        while time.monotonic() < deadline:
            if publisher.get_subscription_count() >= 1 and node.count_publishers("/text_motion/status") == 1:
                break
            rclpy.spin_once(node, timeout_sec=0.1)
        else:
            raise RuntimeError("Exactly one generator and its status publisher must be available")
        publish(request)
        print("request_id=" + identifier, flush=True)
        deadline = time.monotonic() + args.timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
            while events:
                event = json.loads(events.pop(0))
                if event.get("request_id") != identifier:
                    continue
                print(json.dumps(event, ensure_ascii=False), flush=True)
                if event["state"] in ("completed", "no_motion", "cancelled"):
                    return
                if event["state"] in ("rejected", "failed", "stopped"):
                    raise RuntimeError(event.get("detail", event["state"]))
        publish({"command": "cancel", "request_id": identifier})
        # Allow the reliable cancellation to leave DDS before destroying this node.
        if hasattr(publisher, "wait_for_all_acked"):
            publisher.wait_for_all_acked(Duration(seconds=2))
        raise RuntimeError("Timed out; cancellation sent for " + identifier + "; inspect /text_motion/status")
    except KeyboardInterrupt:
        if rclpy.ok():
            publish({"command": "cancel", "request_id": identifier})
            if hasattr(publisher, "wait_for_all_acked"):
                publisher.wait_for_all_acked(Duration(seconds=2))
        raise
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
