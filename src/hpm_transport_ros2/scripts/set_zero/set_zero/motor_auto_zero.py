#!/usr/bin/env python3
# Copyright (c) 2026 HPMicro
# SPDX-License-Identifier: BSD-3-Clause
"""Fixture-based sequential homing. --dry-run never initializes ROS or writes logs."""

import argparse
from collections import deque
from dataclasses import asdict, dataclass
from datetime import datetime, timezone
import json
import math
import os
from pathlib import Path
from queue import Full, Queue
import signal
import statistics
import sys
from threading import Thread
import time
import uuid
import xml.etree.ElementTree as ET

import yaml


COMMAND_TOPIC = "/actuator/motor/command"
ENABLE_TOPIC = "/actuator/motor/enable"
STATE_TOPIC = "/actuator/motor/state"
ID_SERVICE = "/actuator/motor/calibrate_id"
ZERO_SERVICE = "/actuator/motor/set_zero_send"
LS_ZERO_SERVICE = "/actuator/motor/set_zero"
POSITIVE = (
    "rate_hz", "feedback_timeout_s", "feedback_start_timeout_s",
    "minimum_motion_rad", "stall_window_s",
    "stall_confirm_s", "position_epsilon_rad", "stall_velocity_rad_s",
    "stop_settle_s", "stop_timeout_s",
    "zero_reference_tolerance_rad", "post_zero_tolerance_rad", "arrival_tolerance_rad", "arrival_velocity_rad_s",
    "arrival_settle_s", "hold_error_rad", "seek_speed_rad_s", "return_speed_rad_s", "return_max_lead_rad",
    "bck_kp", "kd", "damping_kd", "contact_torque_nm",
    "brake_kd", "brake_max_torque_nm", "brake_speed_ratio", "brake_speed_margin_rad_s",
    "brake_position_error_rad", "brake_confirm_s", "brake_velocity_rad_s", "brake_settle_s", "brake_timeout_s",
    "brake_overspeed_confirm_s", "brake_emergency_speed_ratio", "motion_settle_position_rad",
    "max_torque_nm", "max_temperature_c", "max_seek_time_s", "max_seek_travel_rad",
    "max_return_time_s", "max_feedback_step_rad", "return_overspeed_torque_limit_nm",
)
NONNEGATIVE = ("bck_kd", "hold_kp", "hold_kd", "return_overspeed_kd")
FEEDFORWARD = ("seek_torque_ff_nm", "move_torque_ff_nm", "hold_torque_ff_nm")
JOINT_GAINS = ("bck_kp", "bck_kd", "kd", "hold_kp", "hold_kd", "return_overspeed_kd", "motion_test_decel_kd")
ADAPTIVE = ("adaptive_ki", "adaptive_torque_limit_nm", "adaptive_torque_rate_nm_s")
DEGREE_TOLERANCES = {"arrival_tolerance_deg": "arrival_tolerance_rad", "hold_error_deg": "hold_error_rad",
                     "post_zero_tolerance_deg": "post_zero_tolerance_rad",
                     "brake_position_error_deg": "brake_position_error_rad",
                     "motion_settle_position_deg": "motion_settle_position_rad",
                     "return_max_lead_deg": "return_max_lead_rad"}
OVERRIDABLE = tuple(key for key in POSITIVE + NONNEGATIVE if key not in JOINT_GAINS) + tuple(DEGREE_TOLERANCES)
MODULES = ("left_arm", "right_arm", "left_leg", "right_leg", "neck")


class HomingError(RuntimeError):
    def __init__(self, message, *, diagnostics=None):
        super().__init__(message)
        self.diagnostics = diagnostics


class UniqueLoader(yaml.SafeLoader):
    """Reject duplicate keys rather than silently replacing safety parameters."""


def unique_mapping(loader, node):
    loader.flatten_mapping(node)
    result = {}
    for key_node, value_node in node.value:
        key = loader.construct_object(key_node)
        if key in result:
            raise ValueError(f"Duplicate YAML key: {key}")
        result[key] = loader.construct_object(value_node)
    return result


UniqueLoader.add_constructor(yaml.resolver.BaseResolver.DEFAULT_MAPPING_TAG, unique_mapping)


def read_yaml(path):
    with Path(path).open(encoding="utf-8") as stream:
        result = yaml.load(stream, Loader=UniqueLoader)
    if not isinstance(result, dict):
        raise ValueError(f"Expected YAML mapping: {path}")
    return result


def integer(value):
    if isinstance(value, bool) or not isinstance(value, (int, str)):
        raise ValueError(f"Expected integer, got {value!r}")
    return int(value, 0) if isinstance(value, str) else value


@dataclass(frozen=True)
class Motor:
    name: str
    index: int
    protocol: str
    model: str
    bus: int
    motor_id: int


@dataclass
class Target:
    motor: Motor
    settings: dict
    lower: float
    upper: float
    limits: dict

    @property
    def zero_target(self):
        return -math.radians(self.settings["home_angle_deg"])


def normalize_tolerances(settings):
    """Normalize each precedence layer separately; legacy radian keys still work."""
    normalized = dict(settings)
    for degrees, radians in DEGREE_TOLERANCES.items():
        if degrees not in normalized:
            continue
        if radians in normalized:
            raise ValueError(f"Specify only one of {degrees} / {radians} in the same settings block")
        value = normalized.pop(degrees)
        if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or value <= 0:
            raise ValueError(f"{degrees} must be a finite positive number")
        normalized[radians] = math.radians(value)
    return normalized


def load_plan(motor_path, calibration_path, urdf_path, module=None, motor_index=None, overrides=None):
    if (module is None) == (motor_index is None):
        raise ValueError("Choose exactly one of --module or --motor-index")
    root = read_yaml(motor_path)
    config = read_yaml(calibration_path)
    motors, indices, routes = {}, set(), set()
    entries = root.get("motors")
    if not isinstance(entries, dict) or not entries:
        raise ValueError("motor config requires a nonempty motors mapping")
    for name, row in entries.items():
        if not isinstance(name, str) or not name or not isinstance(row, dict):
            raise ValueError("Invalid motor entry")
        index, bus, motor_id = (integer(row[key]) for key in ("index", "bus_index", "motor_id"))
        protocol = row["motor_type"]
        if protocol not in {"RS", "MR", "LS", "DM", "ENC"}:
            raise ValueError(f"{name}: invalid protocol")
        maximum = 2047 if protocol in {"LS", "ENC"} else 255
        if not 0 <= index <= 20 or not 0 <= bus <= 7 or not 1 <= motor_id <= maximum:
            raise ValueError(f"{name}: route outside HPM snapshot/protocol range")
        if index in indices or (bus, motor_id) in routes:
            raise ValueError(f"{name}: duplicate index or bus/ID")
        model = str(row.get("model", "")) if protocol == "MR" else ""
        if protocol == "MR" and model not in {"PA35-10-8", "PA43-10-40", "PA43-10-20.25"}:
            raise ValueError(f"{name}: unsupported MR model")
        motors[name] = Motor(name, index, protocol, model, bus, motor_id)
        indices.add(index)
        routes.add((bus, motor_id))
    profiles, groups, defaults = (config.get(key) for key in ("joints", "modules", "defaults"))
    if not all(isinstance(item, dict) for item in (profiles, groups, defaults)):
        raise ValueError("Calibration config requires joints/modules/defaults mappings")
    if any(key in defaults or key in (overrides or {}) for key in
           JOINT_GAINS + FEEDFORWARD + ADAPTIVE + ("adaptive_torque_enabled", "kp", "move_kd")):
        raise ValueError("gains, feedforward and adaptive torque must be specified per joint, not as shared defaults/overrides")
    names = groups.get(module) if module is not None else [m.name for m in motors.values() if m.index == motor_index]
    if not isinstance(names, list) or not names or len(names) != len(set(names)):
        raise ValueError("Unknown/empty/duplicate motor selection")
    urdf = ET.parse(urdf_path).getroot()
    joints = {}
    for joint in urdf.findall("joint"):
        name = joint.get("name")
        if name in joints:
            raise ValueError(f"Duplicate URDF joint: {name}")
        joints[name] = joint
    selected = []
    for name in names:
        if name not in motors or name not in profiles or name not in joints:
            raise ValueError(f"Missing motor/profile/URDF joint: {name}")
        if not isinstance(profiles[name], dict):
            raise ValueError(f"Invalid profile: {name}")
        p = {"zero_max_attempts": 3, "ls_zero_attempt_timeout_s": 2.0, "ls_zero_total_timeout_s": 10.0,
             "ls_zero_retry_until_success": True, "post_zero_static_check": True,
             "motion_test_amplitude_deg": 15.0, "motion_test_position_margin_deg": 2.0,
             "motion_test_arrival_tolerance_deg": 2.0, "motion_test_torque_ff_nm": 0.0,
             "motion_test_torque_source": "seek", "motion_test_speed_rad_s": 0.5,
             "motion_test_cycles": 3, "motion_test_timeout_s": 60.0,
             "motion_test_torque_rate_nm_s": 0.25,
             "motion_test_use_seek_feedforward": True,
             "adaptive_torque_enabled": False, "damping_kd": 0.2,
             "final_mode": "damping", "brake_kd": 0.2, "brake_max_torque_nm": 0.5,
             "brake_speed_ratio": 1.5, "brake_speed_margin_rad_s": 0.2,
             "brake_position_error_rad": math.radians(10), "brake_confirm_s": 0.1,
             "brake_velocity_rad_s": 0.02, "brake_settle_s": 0.2, "brake_timeout_s": 2.0,
             "brake_overspeed_confirm_s": 0.05, "brake_emergency_speed_ratio": 2.0,
             "motion_settle_position_rad": math.radians(0.1), "max_brake_attempts": 3,
             "return_max_lead_rad": math.radians(8),
             "return_overspeed_kd": 0.0, "return_overspeed_torque_limit_nm": 0.5,
             **dict.fromkeys(FEEDFORWARD + ADAPTIVE, 0.0), **normalize_tolerances(defaults),
             **normalize_tolerances(profiles[name]), **normalize_tolerances(overrides or {})}
        p.setdefault("post_zero_tolerance_rad", p.get("zero_reference_tolerance_rad"))
        joint = joints[name]
        limit = joint.find("limit")
        if joint.get("type") != "revolute" or limit is None:
            raise ValueError(f"{name}: expected revolute joint with limits")
        lower, upper = float(limit.attrib["lower"]), float(limit.attrib["upper"])
        if not all(math.isfinite(v) for v in (lower, upper)) or not lower <= 0 <= upper or lower >= upper:
            raise ValueError(f"{name}: invalid URDF range or zero outside range")
        motor = motors[name]
        limits = root["motor_protocols"][motor.protocol]
        if motor.protocol == "MR":
            limits = limits["models"][motor.model]
        limits = {key: float(limits[key]) for key in
                  ("p_min", "p_max", "v_min", "v_max", "kp_max", "kd_max", "t_min", "t_max")}
        limits["urdf_velocity"] = float(limit.attrib["velocity"])
        if not all(math.isfinite(v) for v in limits.values()):
            raise ValueError(f"{name}: non-finite protocol/URDF limits")
        for prefix in ("p", "v", "t"):
            if not limits[prefix + "_min"] < 0 < limits[prefix + "_max"]:
                raise ValueError(f"{name}: protocol range must include zero")
        if any(limits[key] <= 0 for key in ("kp_max", "kd_max", "urdf_velocity")):
            raise ValueError(f"{name}: invalid protocol/URDF limits")
        selected.append(Target(motor, p, lower, upper, limits))
    return list(motors.values()), selected


def motion_errors(target):
    p, errors = target.settings, []
    if "motion_test_decel_kd" in p:
        value = p["motion_test_decel_kd"]
        if (isinstance(value, bool) or not isinstance(value, (int, float)) or
                not math.isfinite(value) or not 0 < value <= target.limits["kd_max"]):
            errors.append("motion_test_decel_kd must be positive and within the protocol range")
    if type(p.get("post_zero_static_check")) is not bool:
        errors.append("post_zero_static_check must be true or false")
    if p.get("final_mode") not in ("damping", "position"):
        errors.append("final_mode must be damping or position")
    if type(p.get("max_brake_attempts")) is not int or p["max_brake_attempts"] < 1:
        errors.append("max_brake_attempts must be a positive integer")
    if type(p.get("adaptive_torque_enabled")) is not bool:
        errors.append("adaptive_torque_enabled must be true or false")
    for key in ADAPTIVE:
        value = p.get(key)
        if (isinstance(value, bool) or not isinstance(value, (int, float)) or
                not math.isfinite(value) or value < 0 or (p.get("adaptive_torque_enabled") and value == 0)):
            errors.append(f"{key} must be finite and positive when adaptive torque is enabled")
    if type(p.get("zero_max_attempts")) is not int or p["zero_max_attempts"] < 1:
        errors.append("zero_max_attempts must be a positive integer (total writes)")
    if target.motor.protocol == "LS":
        if type(p.get("ls_zero_retry_until_success")) is not bool:
            errors.append("ls_zero_retry_until_success must be true or false")
        for key in ("ls_zero_attempt_timeout_s", "ls_zero_total_timeout_s"):
            value = p.get(key)
            if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or value <= 0:
                errors.append(f"{key} must be finite and positive")
    if "acceleration_rad_s2" in p:
        errors.append("acceleration_rad_s2 is no longer supported; remove it")
    if p.get("direction_confirmed") is not True:
        errors.append("direction_confirmed must be true (placeholder direction)")
    if type(p.get("seek_direction")) is not int or p["seek_direction"] not in (-1, 1):
        errors.append("seek_direction must be -1 or +1")
    home = p.get("home_angle_deg")
    if isinstance(home, bool) or not isinstance(home, (int, float)) or not math.isfinite(home):
        errors.append("home_angle_deg must be finite")
    # Motor seek sign and the fixture's contact angle are configured independently.
    # The return target below is still checked against the protocol position range.
    for key in POSITIVE + NONNEGATIVE:
        value = p.get(key)
        if key == "max_temperature_c" and key in p and value is None:
            continue
        if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
            errors.append(f"{key} is missing or non-finite")
        elif value < 0 or (key in POSITIVE and value == 0):
            errors.append(f"{key} outside allowed range")
    for key in FEEDFORWARD:
        value = p.get(key)
        if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
            errors.append(f"{key} must be a finite signed number")
        elif not target.limits["t_min"] <= value <= target.limits["t_max"]:
            errors.append(f"{key} exceeds protocol torque range")
        elif isinstance(p.get("max_torque_nm"), (int, float)) and abs(value) > p["max_torque_nm"]:
            errors.append(f"{key} exceeds torque budget")
    if "kp" in p or "move_kd" in p:
        errors.append("rename kp to bck_kp; set bck_kd from move_kd (or kd if absent), then remove old keys")
    if errors:
        return errors
    limits = target.limits
    if p["brake_max_torque_nm"] > min(p["max_torque_nm"], -limits["t_min"], limits["t_max"]):
        errors.append("brake_max_torque_nm exceeds torque budget/protocol range")
    if p["return_overspeed_kd"] > 0 and p["return_overspeed_torque_limit_nm"] > min(
            p["max_torque_nm"], -limits["t_min"], limits["t_max"]):
        errors.append("return_overspeed_torque_limit_nm exceeds torque budget/protocol range")
    if p["return_max_lead_rad"] >= p["brake_position_error_rad"]:
        errors.append("return_max_lead_deg must be below brake_position_error_deg")
    if p["brake_speed_ratio"] <= 1:
        errors.append("brake_speed_ratio must exceed 1")
    if p["brake_emergency_speed_ratio"] <= 1:
        errors.append("brake_emergency_speed_ratio must exceed 1")
    if p["brake_overspeed_confirm_s"] >= p["feedback_timeout_s"]:
        errors.append("brake_overspeed_confirm_s must be below feedback timeout")
    if p["brake_velocity_rad_s"] >= p["return_speed_rad_s"]:
        errors.append("brake_velocity_rad_s must be below return speed")
    if p["brake_timeout_s"] <= max(p["brake_settle_s"], 4 / p["rate_hz"]) + 2 / p["rate_hz"]:
        errors.append("brake_timeout_s too short for brake settling")
    if p["adaptive_torque_enabled"] and p["adaptive_torque_limit_nm"] > min(p["max_torque_nm"], -limits["t_min"], limits["t_max"]):
        errors.append("adaptive torque limit exceeds torque budget/protocol range")
    if max(p["bck_kp"], p["hold_kp"]) > limits["kp_max"] or max(
        p["kd"], p["bck_kd"] + p["return_overspeed_kd"],
        p["hold_kd"], p["brake_kd"], p["damping_kd"]
    ) > limits["kd_max"]:
        errors.append("gains exceed protocol encoding range")
    if max(p["seek_speed_rad_s"], p["return_speed_rad_s"]) > min(
        -limits["v_min"], limits["v_max"], limits["urdf_velocity"]
    ):
        errors.append("speed exceeds protocol/URDF range")
    if p["max_torque_nm"] > min(-limits["t_min"], limits["t_max"]):
        errors.append("torque threshold exceeds protocol range")
    if not limits["p_min"] <= target.zero_target <= limits["p_max"]:
        errors.append("zero target exceeds protocol position range")
    if p["rate_hz"] > 500 or p["rate_hz"] < 10:
        errors.append("rate_hz must be 10..500")
    if p["contact_torque_nm"] >= p["max_torque_nm"]:
        errors.append("contact torque must be below maximum torque")
    seek_torque = p["seek_torque_ff_nm"] + p["kd"] * p["seek_direction"] * p["seek_speed_rad_s"]
    if abs(seek_torque) > p["max_torque_nm"]:
        errors.append("seek feedforward + velocity term exceeds torque budget")
    if p["contact_torque_nm"] >= max(0.0, abs(seek_torque) - p["kd"] * p["stall_velocity_rad_s"]):
        errors.append("contact torque is unreachable with the configured seek feedforward/kd/speed")
    if p["seek_speed_rad_s"] * p["stall_window_s"] < 5 * p["position_epsilon_rad"]:
        errors.append("normal seek motion must exceed 5 * position_epsilon per window")
    if p["minimum_motion_rad"] <= p["position_epsilon_rad"]:
        errors.append("minimum_motion must exceed position_epsilon")
    if p["stall_velocity_rad_s"] >= p["seek_speed_rad_s"]:
        errors.append("stall_velocity must be below seek speed")
    if p["feedback_timeout_s"] <= 2 / p["rate_hz"]:
        errors.append("feedback_timeout must exceed two control periods")
    if p["max_seek_time_s"] <= p["stall_window_s"] + p["stall_confirm_s"]:
        errors.append("max_seek_time too short for contact detection")
    if p["max_seek_travel_rad"] <= p["minimum_motion_rad"]:
        errors.append("max_seek_travel must exceed minimum_motion")
    duration = trajectory_duration(abs(target.zero_target), p)
    if p["max_return_time_s"] <= duration + max(p["arrival_settle_s"], 4 / p["rate_hz"]):
        errors.append("max_return_time too short for the configured return speed")
    if p["hold_error_rad"] < p["arrival_tolerance_rad"]:
        errors.append("hold_error must not be smaller than arrival_tolerance")
    return errors


def trajectory_duration(distance, p):
    return distance / p["return_speed_rad_s"]


def trajectory(start, end, elapsed, duration):
    if duration <= 0 or elapsed >= duration:
        return (end, 0.0)
    if elapsed < 0:
        return (start, 0.0)
    velocity = (end - start) / duration
    return (start + velocity * elapsed, velocity)


@dataclass(frozen=True)
class Feedback:
    received_at: float
    sequence: int
    q: float
    dq: float
    tau: float
    temperature: float
    enabled: bool | None = None


def motion_test_plan(target):
    p = target.settings
    for key in ("motion_test_amplitude_deg", "motion_test_position_margin_deg", "motion_test_arrival_tolerance_deg",
                "motion_test_speed_rad_s", "motion_test_torque_rate_nm_s", "motion_test_timeout_s"):
        value = p.get(key)
        if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or value <= 0:
            raise ValueError(f"{target.motor.name}: {key} must be finite and positive")
    cycles = p.get("motion_test_cycles")
    if type(cycles) is not int or cycles <= 0:
        raise ValueError("motion_test_cycles must be a positive integer")
    extension = p.get("motion_test_turn_extension_s", 0.0)
    smooth_reverse = p.get("motion_test_smooth_reverse", False)
    if type(smooth_reverse) is not bool:
        raise ValueError("motion_test_smooth_reverse must be true or false")
    if (isinstance(extension, bool) or not isinstance(extension, (int, float))
            or not math.isfinite(extension) or extension < 0):
        raise ValueError("motion_test_turn_extension_s must be finite and nonnegative")
    if type(p.get("motion_test_use_seek_feedforward")) is not bool:
        raise ValueError("motion_test_use_seek_feedforward must be true or false")
    test_ff = p["motion_test_torque_ff_nm"]
    if isinstance(test_ff, bool) or not isinstance(test_ff, (int, float)) or not math.isfinite(test_ff) or test_ff < 0:
        raise ValueError("motion_test_torque_ff_nm must be a finite nonnegative peak magnitude")
    if p["motion_test_torque_source"] not in {"seek", "fixed"}:
        raise ValueError("motion_test_torque_source must be seek or fixed")
    speed = p["motion_test_speed_rad_s"]
    if speed > min(-target.limits["v_min"], target.limits["v_max"], target.limits["urdf_velocity"]):
        raise ValueError("motion_test_speed_rad_s exceeds protocol/URDF velocity range")
    default_limit = speed * p["brake_speed_ratio"] + p["brake_speed_margin_rad_s"]
    emergency_speed = p.get("motion_test_emergency_speed_rad_s", default_limit * p["brake_emergency_speed_ratio"])
    if (isinstance(emergency_speed, bool) or not isinstance(emergency_speed, (int, float)) or
            not math.isfinite(emergency_speed) or emergency_speed < speed):
        raise ValueError("motion_test_emergency_speed_rad_s must be finite and at least the test speed cap")
    speed_limit = p.get("motion_test_speed_limit_rad_s", default_limit)
    if (isinstance(speed_limit, bool) or not isinstance(speed_limit, (int, float)) or
            not math.isfinite(speed_limit) or not speed <= speed_limit <= emergency_speed):
        raise ValueError("motion_test_speed_limit_rad_s must be finite, at least the test speed cap, "
                         "and no higher than the emergency threshold")
    amplitude = math.radians(p["motion_test_amplitude_deg"])
    center = target.zero_target
    # SETZERO anchors one mechanical end at motor q=0. Use the URDF stroke
    # length and the commissioned seek sign, without changing home_angle_deg.
    stroke = target.upper - target.lower
    bound_low, bound_high = ((0.0, stroke) if p["seek_direction"] < 0 else (-stroke, 0.0))
    bound_low = max(bound_low, target.limits["p_min"])
    bound_high = min(bound_high, target.limits["p_max"])
    low, high = max(center - amplitude, bound_low), min(center + amplitude, bound_high)
    if not bound_low <= center <= bound_high or high - low <= 1e-9:
        raise ValueError(f"{target.motor.name}: no valid test travel around calibrated zero")
    single_sided = low >= center - 1e-9 or high <= center + 1e-9
    midpoint, half_width = (low + high) / 2, (high - low) / 2
    omega = speed / half_width
    low, high = midpoint - half_width, midpoint + half_width
    torque_peak = abs(p["seek_torque_ff_nm"]) if p["motion_test_torque_source"] == "seek" else test_ff
    if not p["motion_test_use_seek_feedforward"]:
        torque_peak = 0.0
    if torque_peak + p["kd"] * half_width * omega > p["max_torque_nm"] + 1e-9:
        raise ValueError(f"{target.motor.name}: motion test exceeds torque budget")
    reverse_margin = p.get("motion_test_reverse_margin_deg", 2.0)
    if (isinstance(reverse_margin, bool) or not isinstance(reverse_margin, (int, float)) or
            not math.isfinite(reverse_margin) or reverse_margin < 0):
        raise ValueError("motion_test_reverse_margin_deg must be finite and nonnegative")
    reverse_margin = min(math.radians(reverse_margin), (high - low) / 4)
    start, end = (high, low) if single_sided and p["seek_direction"] > 0 else (low, high)
    return {"start_rad": start, "low_rad": low, "high_rad": high,
            "center_rad": (start + end) / 2, "cosine_rad": (start - end) / 2,
            "omega_rad_s": omega, "cycles": cycles, "peak_speed_rad_s": half_width * omega, "speed_cap_rad_s": speed,
            "nominal_period_s": 2 * math.pi / omega, "timeout_s": p["motion_test_timeout_s"],
            "turn_extension_s": extension, "smooth_reverse": smooth_reverse,
            "speed_limit_rad_s": speed_limit, "emergency_speed_rad_s": emergency_speed,
            "single_sided": single_sided, "actual_span_deg": math.degrees(high - low),
            "joint_low_rad": bound_low, "joint_high_rad": bound_high,
            "reverse_low_rad": low + reverse_margin, "reverse_high_rad": high - reverse_margin,
            "reverse_margin_deg": math.degrees(reverse_margin),
            "margin_rad": math.radians(p["motion_test_position_margin_deg"]),
            "arrival_tolerance_rad": min(p["arrival_tolerance_rad"],
                                         math.radians(p["motion_test_arrival_tolerance_deg"])),
            "torque_ff_peak_nm": torque_peak, "torque_source": p["motion_test_torque_source"],
            "torque_rate_nm_s": p["motion_test_torque_rate_nm_s"]}


class MotorRun:
    """Pure single-motor state machine; bounded zero writes and enable actions handled by Runner."""

    def __init__(self, target, now):
        self.target, self.p = target, target.settings
        self.position_target = target.zero_target
        self.final_phase = "HOLD" if self.p["final_mode"] == "position" else "DAMPING"
        self.phase, self.started, self.phase_at = "WAIT_FEEDBACK", now, now
        self.last, self.first_q, self.contact_q = None, None, None
        self.previous_feedback = None
        self.published_commands = deque(maxlen=20)
        self.window = deque()
        self.seek_diagnostics = {}
        self.next_seek_status_at = now
        self.candidate_at = None
        self.settled_at = None
        self.travel = self.peak_torque = 0.0
        self.zero_sent = self.zero_dispatched = False
        self.legacy_ls = False
        self.require_enable_status = False
        self.zero_feedback_received = False
        self.zero_confirmed = False
        self.return_feedback_ready = False
        self.brake_count = 0
        self.brake_reason = None
        self.brake_limited = False
        self.brake_compensation_nm = 0.0
        self.loss_at = None
        self.overspeed_at = None
        self.motion_span_rad = self.motion_mean_velocity_rad_s = None
        self.return_started_at = None
        self.return_replans = 0
        self.return_reference = None
        self.return_reference_at = None
        self.return_reference_limited = False
        self.return_speed_limit_rad_s = None
        self.return_emergency_speed_rad_s = None
        self.return_catchup_active = False
        self.return_speed_damping_nm = 0.0
        self.endpoint_handoff_done = False
        self.endpoint_handoff_nm = 0.0
        self.next_return_status_at = now
        self.zero_attempts = 0
        self.zero_enable_count = 0
        self.zero_reenable_pending = False
        self.adaptive_torque = 0.0
        self.adaptive_sample = None
        self.adaptive_ki_frozen = False
        self.adaptive_limited = False
        self.hold_limited = False
        self.hold_transfer_limited = False
        self.hold_requested_torque_nm = None
        self.hold_applied_torque_nm = None
        self.arm_motion_test = False
        self.hold_torque_window = deque()
        self.motion_test_downhill = 0
        self.motion_test = None
        self.motion_test_phase_offset = 0.0
        self.motion_test_reverse_side = 0
        self.motion_test_reverse_count = 0
        self.motion_test_reversal = {}
        self.motion_test_last_direction = 0
        self.motion_test_half_cycles = 0
        self.motion_test_exit = (0.0, 0.0)
        self.motion_test_decel_ready = False
        self.motion_test_decel_support_nm = 0.0
        self.motion_test_return_active = False
        self.motion_test_velocity_window = deque()
        self.motion_test_velocity_rad_s = None
        self.motion_test_torque_nm = 0.0
        self.motion_test_torque_sample = None

    @property
    def zero_unknown(self):
        return self.zero_sent and not self.zero_confirmed

    def enable_ready(self, feedback):
        return feedback.enabled is True or (self.legacy_ls and feedback.enabled is None)

    def fail(self, reason, *, diagnostics=None):
        details = dict(diagnostics or {})
        details["recent_ros_commands"] = list(self.published_commands)
        details["adaptive_torque_nm"] = self.adaptive_torque
        details["adaptive_limited"] = self.adaptive_limited
        if self.phase == "SEEK":
            details["seek_window"] = dict(self.seek_diagnostics)
        raise HomingError(f"{self.target.motor.name} [{self.phase}]: {reason}", diagnostics=details)

    def transition(self, phase, now):
        self.hold_torque_window.clear()
        if phase == "HOLD":
            self.hold_transfer_limited = False
        if phase == "RECOVER":
            self.hold_transfer_limited = False
        if phase == "HOLD" and self.phase in {"MOVE", "RECOVER"} and self.p["adaptive_torque_enabled"] and self.last:
            # Transfer the departing PD contribution into the integral when gains
            # change. This is state rebasing, not a step in commanded total torque.
            f, p = self.last, self.p
            inherited = (self.adaptive_torque + (p["bck_kp"] - p["hold_kp"]) * (self.position_target - f.q)
                         - (p["bck_kd"] - p["hold_kd"]) * f.dq)
            if abs(inherited) > p["adaptive_torque_limit_nm"]:
                # Keep the stronger existing return gains until the bounded
                # integral can take over; do not withdraw support on handoff.
                phase = "RECOVER"
                self.hold_transfer_limited = True
            else:
                self.adaptive_torque = inherited
        if phase not in {"HOLD", "RECOVER"}:
            self.adaptive_torque = 0.0
            self.adaptive_sample = None
            self.adaptive_limited = False
        if phase != "MOVE":
            self.motion_test_torque_nm = 0.0
            self.motion_test_torque_sample = None
        if phase == "MOTION_TEST":
            self.motion_test_phase_offset = 0.0
            self.motion_test_phase_at = now
            self.motion_test_turn_used_s = self.motion_test_total_extension_s = 0.0
            self.motion_test_reverse_side = 0
            self.motion_test_reverse_count = 0
            self.motion_test_reversal = {}
            self.motion_test_last_direction = 0
            self.motion_test_half_cycles = 0
        self.motion_test_velocity_window.clear()
        self.motion_test_velocity_rad_s = None
        self.phase, self.phase_at = phase, now
        self.window.clear()
        self.settled_at = self.candidate_at = None
        self.loss_at = None
        if phase != "MOTION_TEST_DECEL":
            self.overspeed_at = None
        self.motion_span_rad = self.motion_mean_velocity_rad_s = None

    def limited_test_torque(self, desired, feedback):
        """Ramp assistance up; withdraw immediately when it would oppose braking."""
        previous = self.motion_test_torque_sample
        dt = (feedback.received_at - previous[1] if previous and feedback.sequence != previous[0]
              else 1 / self.p["rate_hz"] if previous is None else 0.0)
        dt = max(0.0, min(dt, self.p["feedback_timeout_s"]))
        self.motion_test_torque_sample = (feedback.sequence, feedback.received_at)
        current = self.motion_test_torque_nm
        if current * desired <= 0:
            current = 0.0
        magnitude = min(abs(desired), abs(current) + self.p["motion_test_torque_rate_nm_s"] * dt)
        self.motion_test_torque_nm = math.copysign(magnitude, desired) if desired else 0.0
        return self.motion_test_torque_nm

    def extend_test_half_cycle(self, angle, now, feedback, fresh):
        """Hold the outgoing velocity briefly while actual position trails the sine."""
        test = self.motion_test
        dt = max(0.0, now - self.motion_test_phase_at)
        self.motion_test_phase_at = now
        omega = test["omega_rad_s"]
        phase = angle % math.pi
        # Two thirds through a half-cycle the reference is halfway out and
        # the commanded speed is still useful. Leave both turning points alone.
        checkpoint = 2 * math.pi / 3
        if not fresh or self.motion_test_reverse_side or not checkpoint <= phase < 0.8 * math.pi:
            return angle
        q_ref = test["center_rad"] + test["cosine_rad"] * math.cos(angle)
        direction = math.copysign(1, -test["cosine_rad"] * math.sin(angle))
        if direction * (q_ref - feedback.q) <= math.radians(test["reverse_margin_deg"]):
            return angle
        halves = 2 * test["cycles"] - self.motion_test_half_cycles
        nominal_remaining = (halves * math.pi - phase) / omega
        slack = test["timeout_s"] - (now - self.phase_at) - nominal_remaining - 0.05
        # Share time saved by early boundary reversals across the remaining cycles.
        budget = (slack + self.motion_test_turn_used_s) / max(1, (halves + 1) // 2)
        cap = min(test["turn_extension_s"], budget)
        delay = max(0.0, min(dt, (phase - checkpoint) / omega, cap - self.motion_test_turn_used_s))
        self.motion_test_phase_offset -= omega * delay
        self.motion_test_turn_used_s += delay
        self.motion_test_total_extension_s += delay
        return angle - omega * delay

    def motion_test_return_command(self, command, feedback):
        if not self.motion_test_return_active:
            return command
        q, dq, kp, kd, ff = command
        # Separate from the arrival window, which is cleared while outside tolerance.
        w = self.motion_test_velocity_window
        if not w or feedback.received_at > w[-1].received_at:
            if w and feedback.received_at - w[-1].received_at > self.p["feedback_timeout_s"]:
                w.clear()
            w.append(feedback)
        duration = max(self.p["arrival_settle_s"], 4 / self.p["rate_hz"])
        while len(w) > 1 and w[1].received_at <= feedback.received_at - duration:
            w.popleft()
        self.motion_test_velocity_rad_s = None
        if len(w) >= 5 and w[-1].received_at - w[0].received_at >= duration - 1e-9:
            self.motion_test_velocity_rad_s = (w[-1].q - w[0].q) / (w[-1].received_at - w[0].received_at)
        error = self.position_target - feedback.q
        desired = 0.0
        if abs(error) > self.p["arrival_tolerance_rad"]:
            direction = math.copysign(1.0, error)
            # Help only toward the goal and while the joint trails the reference.
            # At the endpoint the reference velocity is zero; allow only slow
            # breakaway assistance, then withdraw it once the joint is moving.
            assist_speed = abs(dq) if direction * dq > 0 else max(
                self.p["arrival_velocity_rad_s"], self.p["stall_velocity_rad_s"])
            assist_speed = min(self.p["return_speed_rad_s"], assist_speed)
            velocity = feedback.dq
            # At zero reference speed, encoder progress confirms motion. Raw MIT
            # dq noise must not repeatedly erase breakaway torque at a fixed q.
            if dq == 0.0 and self.motion_test_velocity_rad_s is not None:
                velocity = self.motion_test_velocity_rad_s
            lag = max(0.0, min(1.0, (assist_speed - direction * velocity) / assist_speed))
            if abs(feedback.dq) >= self.p["return_speed_rad_s"]:
                lag = 0.0  # Immediate withdrawal for real/possible high speed.
            if direction * (q - feedback.q) > 0:
                desired = direction * self.motion_test["torque_ff_peak_nm"] * lag
        base = kp * (q - feedback.q) + kd * (dq - feedback.dq) + ff
        available = max(0.0, self.p["max_torque_nm"] - abs(base))
        desired = max(-available, min(available, desired))
        return (q, dq, kp, kd, ff + self.limited_test_torque(desired, feedback))

    def position_command(self, q, dq, kp, kd, feedback):
        """Bounded integral replaces fixed feedforward in MOVE/HOLD/RECOVER."""
        p, f = self.p, feedback
        self.return_speed_damping_nm = 0.0
        # At the endpoint, use the base return gains while waiting to settle.
        # Do not amplify residual velocity noise with extra cruise damping.
        if (self.phase == "MOVE" and p["return_overspeed_kd"] > 0 and
                dq * f.dq > 0 and abs(f.dq) > abs(dq)):
            extra_kd = min(p["return_overspeed_kd"],
                           p["return_overspeed_torque_limit_nm"] / abs(f.dq - dq),
                           max(0.0, self.target.limits["kd_max"] - kd))
            kd += extra_kd
            self.return_speed_damping_nm = extra_kd * (dq - f.dq)
        if not p["adaptive_torque_enabled"]:
            return (q, dq, kp, kd, p["move_torque_ff_nm" if self.phase == "MOVE" else "hold_torque_ff_nm"])
        previous = self.adaptive_sample
        if not self.adaptive_ki_frozen and (previous is None or
                (f.sequence != previous[0] and f.received_at > previous[1])):
            self.adaptive_sample = (f.sequence, f.received_at)
            # No integration of pre-MOVE history, duplicate samples or feedback gaps.
            dt = f.received_at - previous[1] if previous else 0.0
            if 0 < dt <= p["feedback_timeout_s"]:
                change = p["adaptive_ki"] * (q - f.q) * dt
                rate_step = p["adaptive_torque_rate_nm_s"] * dt
                limited_change = max(-rate_step, min(rate_step, change))
                limit = p["adaptive_torque_limit_nm"]
                candidate = max(-limit, min(limit, self.adaptive_torque + limited_change))
                delta = candidate - self.adaptive_torque
                total = kp * (q - f.q) + kd * (dq - f.dq) + candidate
                # Conditional integration: do not add torque into saturation;
                # allow unwinding. Runner still rejects an excessive total.
                blocked = abs(total) > p["max_torque_nm"] and total * delta > 0
                # During speed damping, do not grow compensation that adds speed.
                blocked |= self.return_speed_damping_nm != 0 and delta * f.dq > 0
                self.adaptive_limited = (blocked or limited_change != change or
                                         candidate != self.adaptive_torque + limited_change)
                if not blocked:
                    self.adaptive_torque = candidate
        return (q, dq, kp, kd, self.adaptive_torque)

    def brake_command(self, feedback):
        kd = self.p["brake_kd"]
        if feedback.dq:
            kd = min(kd, self.p["brake_max_torque_nm"] / abs(feedback.dq))
        self.brake_limited = kd < self.p["brake_kd"]
        return (0.0, 0.0, 0.0, kd, 0.0)

    def limit_hold_torque(self, command, feedback):
        """Limit HOLD/RECOVER output without editing configured gains or limits."""
        q, dq, kp, kd, requested_ff = command
        limits = self.target.limits
        limit = min(self.p["max_torque_nm"], -limits["t_min"], limits["t_max"])
        ff = max(-limit, min(limit, requested_ff))
        pd = kp * (q - feedback.q) + kd * (dq - feedback.dq)
        self.hold_requested_torque_nm = pd + requested_ff
        total = pd + ff
        bounded = max(-limit, min(limit, total))
        scale = max(0.0, min(1.0, (bounded - ff) / pd)) if pd and bounded != total else 1.0
        self.hold_limited = scale < 1.0 or ff != requested_ff
        self.hold_applied_torque_nm = pd * scale + ff
        return (q, dq, kp * scale, kd * scale, ff)

    def sample_hold_torque(self, feedback):
        w, p = self.hold_torque_window, self.p
        gap_limit = min(p["feedback_timeout_s"], max(0.05, 4 / p["rate_hz"]))
        if w and not 0 < feedback.received_at - w[-1][0].received_at <= gap_limit:
            w.clear()
        if abs(feedback.dq) > max(0.1, p["brake_velocity_rad_s"]):
            w.clear()
            return
        w.append((feedback, self.hold_applied_torque_nm))
        duration = max(p["brake_settle_s"], 4 / p["rate_hz"])
        while len(w) > 1 and w[1][0].received_at <= feedback.received_at - duration:
            w.popleft()

    def stable_hold_torque(self, now):
        w, p = self.hold_torque_window, self.p
        if self.phase != "HOLD" or len(w) < 5:
            return None
        first, last = w[0][0], w[-1][0]
        span_s = last.received_at - first.received_at
        gap_limit = min(p["feedback_timeout_s"], max(0.05, 4 / p["rate_hz"]))
        if (not 0 <= now - last.received_at <= gap_limit or
                span_s < max(p["brake_settle_s"], 4 / p["rate_hz"]) - 1e-9 or
                abs(last.q - first.q) > span_s * p["brake_velocity_rad_s"] or
                max(f.q for f, _ in w) - min(f.q for f, _ in w) > p["motion_settle_position_rad"]):
            return None
        return statistics.median(torque for _, torque in w)

    def begin_brake(self, now, reason):
        if self.brake_count >= self.p["max_brake_attempts"]:
            self.fail("BRAKE_RETRY_LIMIT", diagnostics={"trigger": reason, "brake_count": self.brake_count})
        self.brake_count += 1
        self.brake_reason = reason
        self.brake_compensation_nm = self.adaptive_torque
        self.transition("BRAKE", now)

    def begin_return(self, now, feedback, resume=False):
        if not resume:
            self.endpoint_handoff_done = False
            self.endpoint_handoff_nm = 0.0
        if self.return_started_at is None:
            self.return_started_at = now
        self.move_start = feedback.q
        self.return_reference, self.return_reference_at = feedback.q, now
        self.return_reference_limited = False
        # Return may start directly after SETZERO, which changes coordinates.
        self.previous_feedback, self.last = self.last, feedback
        self.duration = trajectory_duration(abs(self.position_target - feedback.q), self.p)
        # A single near/slow frame is insufficient to declare arrival.
        self.transition("MOVE", now)
        if self.phase != "DAMPING" and self.p["adaptive_torque_enabled"]:
            self.adaptive_torque = self.brake_compensation_nm if resume else 0.0
            self.adaptive_sample = (feedback.sequence, feedback.received_at)

    def stable(self, condition, now, duration):
        if not condition:
            self.settled_at = None
        elif self.settled_at is None:
            self.settled_at = now
        return self.settled_at is not None and now - self.settled_at >= duration

    def motion_settled(self, feedback, duration, velocity_limit, near=True, position_limit=None):
        """Fresh position range and mean velocity, independent of noisy MIT dq."""
        duration = max(duration, 4 / self.p["rate_hz"])
        w = self.window
        if not near:
            w.clear()
        elif w and feedback.sequence == w[-1].sequence and feedback.received_at == w[-1].received_at:
            return False  # Duplicate reads do not advance or erase a window.
        elif not w or feedback.received_at > w[-1].received_at:
            if w and feedback.received_at - w[-1].received_at > self.p["feedback_timeout_s"]:
                w.clear()
            w.append(feedback)
        else:
            w.clear()  # Non-increasing timestamps cannot establish a window.
        while len(w) > 1 and w[1].received_at <= feedback.received_at - duration - 1e-9:
            w.popleft()
        self.motion_span_rad = self.motion_mean_velocity_rad_s = None
        if len(w) < 5 or w[-1].received_at - w[0].received_at < duration - 1e-9:
            return False
        interval = w[-1].received_at - w[0].received_at
        self.motion_span_rad = max(v.q for v in w) - min(v.q for v in w)
        self.motion_mean_velocity_rad_s = (w[-1].q - w[0].q) / interval
        span_limit = self.p["motion_settle_position_rad"] if position_limit is None else position_limit
        return (self.motion_span_rad <= span_limit and
                abs(self.motion_mean_velocity_rad_s) <= velocity_limit)

    def step(self, now, feedback):
        p = self.p
        bck_kd = p["bck_kd"]
        if feedback is None or feedback.received_at < self.started:
            if now - self.started > p["feedback_start_timeout_s"]:
                self.fail("NO_FEEDBACK")
            return (0.0, 0.0, 0.0, 0.0, 0.0)
        f = feedback
        if now - f.received_at > p["feedback_timeout_s"]:
            self.fail("FEEDBACK_TIMEOUT")
        if not all(math.isfinite(v) for v in (f.q, f.dq, f.tau, f.temperature, f.received_at)):
            self.fail("INVALID_FEEDBACK")
        if self.return_feedback_ready and not self.enable_ready(f):
            self.fail("ENABLE_FEEDBACK_LOST", diagnostics={"enabled": f.enabled, "q_rad": f.q,
                                                           "sequence": f.sequence})
        torque_exceeded = abs(f.tau) > p["max_torque_nm"]
        temperature_exceeded = p["max_temperature_c"] is not None and f.temperature > p["max_temperature_c"]
        if torque_exceeded or temperature_exceeded:
            self.fail("TORQUE_LIMIT" if torque_exceeded else "TEMPERATURE_LIMIT", diagnostics={
                "elapsed_s": now - self.started,
                "feedback": {"q_rad": f.q, "dq_rad_s": f.dq, "tau_nm": f.tau,
                             "temperature_c": f.temperature, "sequence": f.sequence,
                             "age_s": now - f.received_at},
                "max_torque_nm": p["max_torque_nm"], "max_temperature_c": p["max_temperature_c"],
            })
        self.peak_torque = max(self.peak_torque, abs(f.tau))
        fresh = self.last is None or f.sequence != self.last.sequence
        position_jump = None
        if fresh:
            if self.last and self.phase != "ZERO_PENDING":
                delta = abs(f.q - self.last.q)
                if delta > p["max_feedback_step_rad"]:
                    position_jump = {"previous_q_rad": self.last.q, "q_rad": f.q,
                                     "position_delta_rad": f.q - self.last.q,
                                     "feedback_interval_s": f.received_at - self.last.received_at,
                                     "dq_rad_s": f.dq, "tau_nm": f.tau,
                                     "sequence": f.sequence, "previous_sequence": self.last.sequence,
                                     "max_feedback_step_rad": p["max_feedback_step_rad"]}
                    # MOVE first gets a chance to replace an unsafe return
                    # command with braking. Other phases retain the hard guard.
                    if self.phase != "MOVE":
                        self.fail("POSITION_JUMP", diagnostics=position_jump)
                if self.phase == "SEEK":
                    self.travel += delta
            self.previous_feedback, self.last = self.last, f
        if self.phase == "WAIT_FEEDBACK":
            if self.require_enable_status and f.enabled is None and not self.legacy_ls:
                self.fail("ENABLE_FEEDBACK_UNAVAILABLE", diagnostics={"protocol": self.target.motor.protocol})
            self.first_q = f.q
            self.transition("SEEK", now)
        if self.phase == "SEEK":
            if now - self.phase_at > p["max_seek_time_s"] or self.travel > p["max_seek_travel_rad"]:
                self.fail("SEEK_TIME_OR_TRAVEL_LIMIT")
            progress = p["seek_direction"] * (f.q - self.first_q)
            if progress < -p["minimum_motion_rad"]:
                self.fail("WRONG_DIRECTION")
            if fresh:
                stationary = self.motion_settled(f, p["stall_window_s"], p["stall_velocity_rad_s"],
                                                  position_limit=p["position_epsilon_rad"])
                w = self.window
                span_s = w[-1].received_at - w[0].received_at if w else 0.0
                min_torque = min((abs(v.tau) for v in w), default=0.0)
                contact = stationary and min_torque >= p["contact_torque_nm"]
                self.seek_diagnostics = {
                    "samples": len(w), "window_span_s": span_s,
                    "window_full": len(w) >= 5 and span_s >= max(p["stall_window_s"], 4 / p["rate_hz"]) - 1e-9,
                    "position_span_rad": max(v.q for v in w) - min(v.q for v in w) if w else None,
                    "mean_velocity_rad_s": self.motion_mean_velocity_rad_s,
                    "reported_velocity_peak_rad_s": max((abs(v.dq) for v in w), default=0.0),
                    "min_abs_torque_nm": min_torque,
                    "position_epsilon_rad": p["position_epsilon_rad"],
                    "stall_velocity_rad_s": p["stall_velocity_rad_s"],
                    "contact_torque_nm": p["contact_torque_nm"],
                    "position_stable": stationary, "torque_contact": min_torque >= p["contact_torque_nm"],
                    "contact_condition": contact,
                }
                if not contact:
                    self.candidate_at = None
                elif self.candidate_at is None:
                    self.candidate_at = f.received_at
                elif f.received_at - self.candidate_at >= p["stall_confirm_s"]:
                    self.contact_q = statistics.median(v.q for v in w)
                    self.transition("STOP", now)
            if self.phase == "SEEK":
                # Velocity seek with optional feedforward; the position field is ignored with kp=0.
                return (f.q, p["seek_direction"] * p["seek_speed_rad_s"], 0.0, p["kd"], p["seek_torque_ff_nm"])
        if self.phase == "STOP":
            if abs(f.q - self.contact_q) > p["zero_reference_tolerance_rad"]:
                self.fail("CONTACT_REFERENCE_MOVED")
            if now - self.phase_at > p["stop_timeout_s"]:
                self.fail("STOP_TIMEOUT")
            if fresh and self.motion_settled(f, p["stop_settle_s"], p["stall_velocity_rad_s"],
                                             position_limit=p["position_epsilon_rad"]):
                self.transition("ZERO_REQUEST", now)
            return (f.q, p["seek_direction"] * p["seek_speed_rad_s"], 0.0, p["kd"], p["seek_torque_ff_nm"])
        if self.phase in {"ZERO_REQUEST", "ZERO_PREPARE"}:
            if abs(f.q - self.contact_q) > p["zero_reference_tolerance_rad"]:
                self.fail("CONTACT_REFERENCE_MOVED")
            return (f.q, p["seek_direction"] * p["seek_speed_rad_s"], 0.0, p["kd"], p["seek_torque_ff_nm"])
        if self.phase == "ZERO_PENDING":
            if self.zero_sent and self.target.motor.protocol != "LS":
                return (0.0, 0.0, 0.0, 0.0, 0.0)
            return (f.q, p["seek_direction"] * p["seek_speed_rad_s"], 0.0, p["kd"], p["seek_torque_ff_nm"])
        if self.phase in {"MOVE", "BRAKE"} and self.return_started_at is not None:
            if now - self.return_started_at > p["max_return_time_s"]:
                self.fail("RETURN_TIMEOUT")
        if self.phase == "BRAKE":
            if now - self.phase_at > p["brake_timeout_s"]:
                self.fail("BRAKE_TIMEOUT")
            emergency_speed = ((p["return_speed_rad_s"] * p["brake_speed_ratio"] +
                                p["brake_speed_margin_rad_s"]) * p["brake_emergency_speed_ratio"])
            if fresh and self.motion_settled(f, p["brake_settle_s"], p["brake_velocity_rad_s"],
                                             near=abs(f.dq) <= emergency_speed):
                self.begin_return(now, f, resume=True)
            else:
                return self.brake_command(f)
        if self.phase == "MOVE":
            if now - (self.return_started_at if self.return_started_at is not None else self.phase_at) > p["max_return_time_s"]:
                self.fail("RETURN_TIMEOUT", diagnostics={
                    "target_rad": self.position_target, "q_rad": f.q,
                    "position_error_rad": self.position_target - f.q, "dq_rad_s": f.dq,
                    "arrival_tolerance_rad": p["arrival_tolerance_rad"],
                })
            error = self.position_target - f.q
            if (fresh and abs(error) > p["arrival_tolerance_rad"] and
                    error * (self.position_target - self.move_start) < 0):
                self.move_start, self.phase_at = f.q, now
                self.duration = trajectory_duration(abs(error), p)
                self.return_replans += 1
                self.return_reference, self.return_reference_at = f.q, now
            if self.return_reference is None:
                self.return_reference, self.return_reference_at = self.move_start, self.phase_at
            previous_reference = self.return_reference
            dt = max(0.0, now - self.return_reference_at)
            goal = self.position_target
            proposed, _ = trajectory(previous_reference, goal, dt,
                                     trajectory_duration(abs(goal - previous_reference), p))
            direction = 1 if goal >= self.move_start else -1
            lead_bound = f.q + direction * p["return_max_lead_rad"]
            if direction > 0:
                q = max(previous_reference, min(proposed, lead_bound, goal))
            else:
                q = min(previous_reference, max(proposed, lead_bound, goal))
            # Pace forward progress, never pull the reference back to chase a fall.
            # Keep the configured velocity/gains/integral while waiting for catch-up.
            dq = direction * p["return_speed_rad_s"] if abs(q - goal) > 1e-12 else 0.0
            self.return_reference_limited = abs(q - proposed) > 1e-9
            self.return_reference, self.return_reference_at = q, now
            speed_limit = p["return_speed_rad_s"] * p["brake_speed_ratio"] + p["brake_speed_margin_rad_s"]
            emergency_speed = speed_limit * p["brake_emergency_speed_ratio"]
            previous = self.previous_feedback
            lag = direction * (q - f.q)
            # A lagging joint may briefly catch its reference faster than cruise.
            # Require measured forward progress and retain the original hard limit.
            catchup = (previous is not None and position_jump is None and
                       0 < f.received_at - previous.received_at <= p["feedback_timeout_s"] and
                       direction * (f.q - previous.q) > 0 and direction * f.dq > 0 and
                       0 < lag <= p["return_max_lead_rad"] + 1e-9 and
                       direction * error > p["arrival_tolerance_rad"])
            if catchup:
                speed_limit = max(speed_limit, min(emergency_speed,
                    p["return_speed_rad_s"] + lag / p["brake_confirm_s"]))
            self.return_speed_limit_rad_s = speed_limit
            self.return_emergency_speed_rad_s = emergency_speed
            self.return_catchup_active = catchup
            overspeed = abs(f.dq) > emergency_speed
            if fresh:
                if previous and f.received_at - previous.received_at > p["feedback_timeout_s"]:
                    self.overspeed_at = None
                if abs(f.dq) <= speed_limit:
                    self.overspeed_at = None
                elif self.overspeed_at is None:
                    self.overspeed_at = f.received_at
                else:
                    overspeed |= f.received_at - self.overspeed_at >= p["brake_overspeed_confirm_s"] - 1e-9
            if overspeed:
                self.begin_brake(now, "RETURN_OVERSPEED")
                return self.brake_command(f)
            if fresh:
                if abs(q - f.q) > p["brake_position_error_rad"]:
                    if self.loss_at is None:
                        self.loss_at = f.received_at
                    elif f.received_at - self.loss_at >= p["brake_confirm_s"]:
                        self.begin_brake(now, "RETURN_TRACKING_ERROR")
                        return self.brake_command(f)
                else:
                    self.loss_at = None
            low = min(0.0, self.position_target, self.move_start)
            high = max(0.0, self.position_target, self.move_start)
            if not low - p["hold_error_rad"] <= f.q <= high + p["hold_error_rad"]:
                self.begin_brake(now, "RETURN_POSITION_OUTSIDE_CORRIDOR")
                return self.brake_command(f)
            arrived = fresh and self.motion_settled(
                f, p["arrival_settle_s"], p["arrival_velocity_rad_s"],
                near=abs(f.q - self.position_target) <= p["arrival_tolerance_rad"]
            )
            if not arrived:
                if (p["adaptive_torque_enabled"] and not self.endpoint_handoff_done and
                        dq == 0.0 and abs(previous_reference - goal) > 1e-12 and
                        direction * error > p["arrival_tolerance_rad"]):
                    # Preserve the departing velocity contribution while the real
                    # joint still trails the goal. Rebase once per deliberate move,
                    # not again after each brake/resume; retain the integral budget.
                    previous_torque = self.adaptive_torque
                    limit = p["adaptive_torque_limit_nm"]
                    self.adaptive_torque = max(-limit, min(limit,
                        previous_torque + bck_kd * direction * p["return_speed_rad_s"]))
                    self.endpoint_handoff_nm = self.adaptive_torque - previous_torque
                    self.endpoint_handoff_done = True
                command = self.position_command(q, dq, p["bck_kp"], bck_kd, f)
                command = self.motion_test_return_command(command, f)
                q_cmd, dq_cmd, kp, kd, ff = command
                estimate = kp * (q_cmd - f.q) + kd * (dq_cmd - f.dq) + ff
                if abs(estimate) > p["max_torque_nm"] + 1e-9:
                    self.begin_brake(now, "COMMAND_TORQUE_BUDGET")
                    return self.brake_command(f)
            if position_jump:
                self.fail("POSITION_JUMP", diagnostics=position_jump)
            if arrived:
                self.transition(self.final_phase, now)
            else:
                return command
        if self.phase in {"MOTION_TEST", "MOTION_TEST_DECEL"}:
            test = self.motion_test
            if not max(test["joint_low_rad"] - p["position_epsilon_rad"], test["low_rad"] - test["margin_rad"]) <= f.q <= min(
                    test["joint_high_rad"] + p["position_epsilon_rad"], test["high_rad"] + test["margin_rad"]):
                self.fail("MOTION_TEST_POSITION_LIMIT", diagnostics={
                    "q_rad": f.q, "joint_low_rad": test["joint_low_rad"],
                    "joint_high_rad": test["joint_high_rad"], "last_reversal": self.motion_test_reversal})
            limit = test["speed_limit_rad_s"]
            speed_details = {"dq_rad_s": f.dq, "speed_limit_rad_s": limit,
                             "emergency_speed_rad_s": test["emergency_speed_rad_s"],
                             "confirm_s": p["brake_overspeed_confirm_s"]}
            if abs(f.dq) > test["emergency_speed_rad_s"]:
                self.fail("MOTION_TEST_OVERSPEED", diagnostics=speed_details)
            if fresh and abs(f.dq) > limit:
                if self.overspeed_at is None:
                    self.overspeed_at = f.received_at
                elif f.received_at - self.overspeed_at >= p["brake_overspeed_confirm_s"]:
                    self.fail("MOTION_TEST_OVERSPEED", diagnostics=speed_details)
            elif fresh:
                self.overspeed_at = None
            if self.phase == "MOTION_TEST_DECEL":
                elapsed = now - self.phase_at
                if elapsed >= p["brake_timeout_s"]:
                    self.fail("MOTION_TEST_DECEL_TIMEOUT", diagnostics={
                        "q_rad": f.q, "dq_rad_s": f.dq, "position_span_rad": self.motion_span_rad,
                        "mean_velocity_rad_s": self.motion_mean_velocity_rad_s})
                # Fade the test setpoints into the frozen load compensation.
                progress = min(1.0, elapsed / 0.15)
                scale = (1 - progress) ** 2 * (1 + 2 * progress)
                decel_kd = p.get("motion_test_decel_kd", p["kd"])
                kd = decel_kd + scale * (p["kd"] - decel_kd)
                dq, ff = (value * scale for value in self.motion_test_exit)
                at_low = f.q <= test["reverse_low_rad"]
                at_high = f.q >= test["reverse_high_rad"]
                if (at_low and (dq < 0 or ff < 0)) or (at_high and (dq > 0 or ff > 0)):
                    # Do not keep driving toward an edge while tapering.
                    self.motion_test_exit = (0.0, 0.0)
                    dq = ff = 0.0
                support = self.motion_test_decel_support_nm
                if not ((at_low and support < 0) or (at_high and support > 0)):
                    ff += (1 - scale) * support
                if abs(kd * (dq - f.dq) + ff) > p["max_torque_nm"] + 1e-9:
                    self.fail("MOTION_TEST_COMMAND_TORQUE_BUDGET")
                if fresh and progress >= 1.0:
                    # Allow low-level MIT speed quantization/noise; require low
                    # encoder-derived speed and position span over fresh samples.
                    self.motion_test_decel_ready = self.motion_settled(
                        f, p["brake_settle_s"], p["brake_velocity_rad_s"],
                        near=abs(f.dq) <= max(0.1, p["brake_velocity_rad_s"]))
                self.motion_test_torque_nm = ff
                return (f.q, dq, 0.0, kd, ff)
            if now - self.phase_at >= test["timeout_s"]:
                self.fail("MOTION_TEST_TIMEOUT", diagnostics={
                    "completed_cycles": self.motion_test_half_cycles // 2,
                    "requested_cycles": test["cycles"], "timeout_s": test["timeout_s"]})
            clock_angle = test["omega_rad_s"] * max(0.0, now - self.phase_at)
            angle = clock_angle + self.motion_test_phase_offset
            if test["turn_extension_s"]:
                angle = self.extend_test_half_cycle(angle, now, f, fresh)
            q = test["center_rad"] + test["cosine_rad"] * math.cos(angle)
            dq = -test["cosine_rad"] * test["omega_rad_s"] * math.sin(angle)
            # Constant feedforward magnitude; at a zero-speed turning point use
            # the upcoming half-cycle direction, not a noisy feedback velocity.
            direction = dq if abs(dq) > 1e-9 else -test["cosine_rad"] * math.cos(angle)
            side = self.motion_test_reverse_side
            boundary = test["reverse_high_rad"] if side > 0 else test["reverse_low_rad"]
            if side and fresh and side * (f.q - boundary) < -2 * p["position_epsilon_rad"]:
                self.motion_test_reverse_side = side = 0
            if not side:
                side = (1 if f.q >= test["reverse_high_rad"] and direction > 0 else
                        -1 if f.q <= test["reverse_low_rad"] and direction < 0 else 0)
                if side:
                    self.motion_test_reverse_side = side
                    self.motion_test_reverse_count += 1
                    self.motion_test_reversal = {"side": "upper" if side > 0 else "lower",
                        "q_rad": f.q, "dq_rad_s": f.dq,
                        "boundary_rad": test["reverse_high_rad"] if side > 0 else test["reverse_low_rad"]}
                    # Rebase only the waveform, never reset cycle count or timeout.
                    angle = math.pi if side * test["cosine_rad"] < 0 else 0.0
                    self.motion_test_phase_offset = angle - clock_angle
            if side:
                # Latch the inward command until fresh position feedback shows
                # retreat. Neither old clock phase nor edge noise may push out.
                if side * direction > 0:
                    angle = math.pi if side * test["cosine_rad"] < 0 else 0.0
                    self.motion_test_phase_offset = angle - clock_angle
                q = test["high_rad"] if side > 0 else test["low_rad"]
                dq = -side * test["peak_speed_rad_s"]
                if test["smooth_reverse"]:
                    # Use the rebased sine speed, with one MIT velocity count
                    # inward so the first reversed command survives encoding.
                    limits = self.target.limits
                    quantum = (limits["v_max"] - limits["v_min"]) / 4095
                    dq = -side * min(abs(dq), max(quantum, abs(dq * math.sin(angle))))
                direction = -side
            command_direction = 1 if direction > 0 else -1
            ff = math.copysign(test["torque_ff_peak_nm"], direction)
            if test["smooth_reverse"] or (self.arm_motion_test and self.motion_test_downhill in {0, command_direction}):
                # A fixed-width band also tapers assistance near a sine turning point.
                speed_deficit = abs(dq) - command_direction * f.dq
                taper_band = 0.25 * test["peak_speed_rad_s"]
                ff *= max(0.0, min(1.0, speed_deficit / taper_band))
                ff = self.limited_test_torque(ff, f)
            elif self.arm_motion_test:
                # Do not accumulate ramp time during the uphill half-cycle.
                self.motion_test_torque_sample = (f.sequence, f.received_at)
            estimate = p["kd"] * (dq - f.dq) + ff
            if abs(estimate) > p["max_torque_nm"] + 1e-9:
                self.fail("MOTION_TEST_COMMAND_TORQUE_BUDGET", diagnostics={
                    "tau_ff_nm": ff, "estimate_nm": estimate, "max_torque_nm": p["max_torque_nm"]})
            # Count the final command direction after any boundary override.
            if self.motion_test_last_direction and command_direction != self.motion_test_last_direction:
                self.motion_test_half_cycles += 1
                self.motion_test_turn_used_s = 0.0
            self.motion_test_last_direction = command_direction
            self.motion_test_torque_nm = ff
            self.motion_test_exit = (dq, ff)
            return (q, dq, 0.0, p["kd"], ff)
        if self.phase == "DAMPING":
            return (0.0, 0.0, 0.0, p["damping_kd"], 0.0)
        if self.phase == "HOLD":
            if abs(f.q - self.position_target) > p["hold_error_rad"]:
                self.transition("RECOVER", now)
        if self.phase == "RECOVER":
            if fresh and self.motion_settled(
                f, p["arrival_settle_s"], p["arrival_velocity_rad_s"],
                near=abs(f.q - self.position_target) <= p["arrival_tolerance_rad"]
            ):
                self.transition("HOLD", now)
        if self.phase in {"HOLD", "RECOVER"}:
            kp, kd = (p["bck_kp"], bck_kd) if self.phase == "RECOVER" else (p["hold_kp"], p["hold_kd"])
            command = self.position_command(self.position_target, 0.0, kp, kd, f)
            command = self.limit_hold_torque(command, f)
            if self.arm_motion_test and self.phase == "HOLD" and fresh:
                self.sample_hold_torque(f)
            return command
        self.fail("invalid state")


def validate_id_reply(motor, response, expected, allowed):
    metadata = (motor.name, motor.protocol, motor.model, motor.bus, motor.motor_id)
    actual = (response.motor_name, response.motor_type, response.model, response.bus_index, response.target_id)
    if actual != metadata:
        raise HomingError(f"{motor.name}: node/script configuration mismatch: {actual} != {metadata}")
    detected = set(response.detected_ids)
    if response.result != 0 or not expected <= detected or detected - allowed:
        raise HomingError(f"{motor.name}: ID check failed result={response.result}, "
                          f"missing={sorted(expected - detected)}, unknown={sorted(detected - allowed)}")
    return sorted(detected - expected)


class Logger:
    def __init__(self, directory):
        directory = Path(directory)
        directory.mkdir(parents=True, exist_ok=True)
        stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
        self.path = directory / f"set_zero_{stamp}_{uuid.uuid4().hex[:8]}.jsonl"
        self.stream = self.path.open("x", encoding="utf-8")
        self.queue = Queue(maxsize=256)
        self.error = None
        self.worker = Thread(target=self.flush, name="set-zero-log", daemon=True)
        self.worker.start()

    def check(self):
        if self.error is not None:
            raise HomingError(f"LOG_WRITE_FAILED: {self.error}")

    def flush(self):
        try:
            while True:
                line = self.queue.get()
                if line is None:
                    break
                self.stream.write(line + "\n")
                self.stream.flush()
                print(line, flush=True)
        except Exception as exc:
            self.error = exc
        finally:
            try:
                self.stream.close()
            except Exception as exc:
                self.error = exc

    def write(self, event, **data):
        self.check()
        record = {"time": datetime.now(timezone.utc).isoformat(), "event": event, **data}
        line = json.dumps(record, ensure_ascii=False, allow_nan=False)
        try:
            self.queue.put_nowait(line)
        except Full as exc:
            raise HomingError("LOG_QUEUE_FULL") from exc

    def close(self):
        self.check()
        self.queue.put(None, timeout=1.0)
        self.worker.join(timeout=2.0)
        if self.worker.is_alive():
            raise HomingError("LOG_FLUSH_TIMEOUT")
        self.check()


def make_ros_io(selected, motion, system_node, ros_args, *, shared_context=False):
    # Lazy imports keep config validation and offline tests independent of ROS.
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from rclpy.signals import SignalHandlerOptions
    from rcl_interfaces.srv import GetParameters
    from luvbot_hardware_interfaces.msg import MotorCommand, MotorEnableCommand, MotorStateArray, MotorTarget
    from luvbot_hardware_interfaces.srv import MotorCalibrateId, MotorSetZero

    class RosIO(Node):
        def __init__(self):
            super().__init__("motor_auto_zero")
            self.feedback, self.sequence, self.last_state_sequence = {}, 0, None
            self.routes = {t.motor.index: t.motor for t in selected}
            self.query_client = self.create_client(MotorCalibrateId, ID_SERVICE)
            if motion:
                self.command_pub = self.create_publisher(MotorCommand, COMMAND_TOPIC, 1)
                self.enable_pub = self.create_publisher(MotorEnableCommand, ENABLE_TOPIC, 1)
                self.subscription = self.create_subscription(MotorStateArray, STATE_TOPIC, self.on_state, qos_profile_sensor_data)
                self.zero_client = self.create_client(MotorSetZero, ZERO_SERVICE)
                self.ls_zero_client = (self.create_client(MotorSetZero, LS_ZERO_SERVICE)
                                       if any(t.motor.protocol == "LS" for t in selected) else None)
                self.parameter_client = self.create_client(GetParameters, system_node.rstrip("/") + "/get_parameters")

        def spin(self, timeout):
            if not rclpy.ok():
                raise HomingError("ROS context stopped")
            rclpy.spin_once(self, timeout_sec=max(0.0, timeout))

        def ready(self):
            return self.query_client.service_is_ready() and (not motion or (
                self.zero_client.service_is_ready() and self.parameter_client.service_is_ready() and
                (self.ls_zero_client is None or self.ls_zero_client.service_is_ready()) and
                self.command_pub.get_subscription_count() > 0 and self.enable_pub.get_subscription_count() > 0))

        def exclusive(self):
            if self.count_publishers(COMMAND_TOPIC) != 1 or self.count_publishers(ENABLE_TOPIC) != 1:
                raise HomingError("Other motor command/enable publishers detected")

        def normal_mode(self):
            names = ["motor_id_calibration_mode", "launch_motor"]
            if self.ls_zero_client is not None:
                names.append("ls_firmware_version")
            request = GetParameters.Request(names=names)
            return self.parameter_client.call_async(request)

        def on_state(self, message):
            sequence = int(message.sequence_id)
            if self.last_state_sequence is not None:
                if sequence == self.last_state_sequence:
                    return
                if sequence < self.last_state_sequence:
                    raise HomingError("State sequence regressed; node restart/reordered feedback")
            self.last_state_sequence = sequence
            now, seen = time.monotonic(), set()
            for state in message.states:
                route = state.route
                motor = self.routes.get(route.motor_type)
                if motor is None:
                    continue
                if (route.bus_index, route.motor_id) != (motor.bus, motor.motor_id):
                    raise HomingError(f"{motor.name}: feedback route mismatch")
                if motor.index in seen or not (message.feedback_mask & (1 << motor.index)):
                    raise HomingError(f"{motor.name}: duplicate/mask-invalid feedback")
                seen.add(motor.index)
                temperatures = [v for v in (state.mos_temperature, state.rotor_temperature) if math.isfinite(v)]
                if not state.valid or state.error_code != 0:
                    raise HomingError(f"{motor.name}: invalid state / motor fault {state.error_code}")
                enabled = {state.ENABLE_ENABLED: True, state.ENABLE_DISABLED: False}.get(state.enable_state)
                self.feedback[motor.index] = Feedback(now, sequence, state.q, state.dq, state.tau,
                                                      max(temperatures) if temperatures else math.nan, enabled)

        def query(self, motor):
            return self.query_client.call_async(MotorCalibrateId.Request(
                operation=MotorCalibrateId.Request.QUERY, motor_index=motor.index, current_id=motor.motor_id))

        def zero(self, motor):
            client = self.ls_zero_client if motor.protocol == "LS" else self.zero_client
            return client.call_async(MotorSetZero.Request(motor_types=[motor.index]))

        def header(self, message):
            self.sequence = (self.sequence + 1) & 0xFFFFFFFF
            message.stamp = self.get_clock().now().to_msg()
            message.sequence_id = self.sequence

        def enable(self, motors, enabled):
            message = MotorEnableCommand()
            self.header(message)
            message.motor_types = [m.index for m in motors]
            message.enabled = [enabled] * len(motors)
            self.enable_pub.publish(message)

        def command(self, commands):
            if not commands:
                return
            message = MotorCommand()
            self.header(message)
            message.control_mode = MotorCommand.CONTROL_MODE_MIT
            for motor, (q, dq, kp, kd, tau_ff) in commands:
                target = MotorTarget()
                target.route.motor_type = motor.index
                target.route.bus_index = motor.bus
                target.route.motor_id = motor.motor_id
                target.q, target.dq, target.kp, target.kd, target.tau = float(q), float(dq), float(kp), float(kd), float(tau_ff)
                message.commands.append(target)
            self.command_pub.publish(message)

        def close(self):
            self.destroy_node()
            if not shared_context:
                rclpy.shutdown()

    if not shared_context:
        rclpy.init(args=ros_args, signal_handler_options=SignalHandlerOptions.NO)
    try:
        return RosIO()
    except BaseException:
        if not shared_context:
            rclpy.shutdown()
        raise


def knee_assist_plan(targets, module):
    if module not in {"left_leg", "right_leg"}:
        return None
    side = module.removesuffix("_leg")
    hip_name, knee_name = f"{side}_hip_pitch_joint", f"{side}_knee_joint"
    names = [t.motor.name for t in targets]
    if hip_name not in names:
        return None
    if knee_name not in names or names.index(knee_name) >= names.index(hip_name):
        raise ValueError(f"{module}: knee must be calibrated before HipPitch")
    hip, knee = targets[names.index(hip_name)], targets[names.index(knee_name)]
    gains = {}
    for key, default in (("hip_pitch_assist_seek_kd", 0.2),
                         ("hip_pitch_assist_return_kd", 0.5)):
        value = knee.settings.get(key, default)
        if (isinstance(value, bool) or not isinstance(value, (int, float)) or
                not math.isfinite(value) or not 0 <= value <= knee.limits["kd_max"]):
            raise ValueError(f"{knee_name}: {key} must be nonnegative and within protocol kd range")
        gains[key] = value
    if gains["hip_pitch_assist_return_kd"] < gains["hip_pitch_assist_seek_kd"]:
        raise ValueError(f"{knee_name}: hip_pitch_assist_return_kd must not be below hip_pitch_assist_seek_kd")
    return {"hip_index": hip.motor.index, "knee_index": knee.motor.index, **gains}



class Runner:
    def __init__(self, io, motors, targets, logger, timeout=10.0, clock=time.monotonic, knee_assist=None):
        self.io, self.motors, self.targets, self.log = io, motors, targets, logger
        self.timeout, self.clock = timeout, clock
        self.knee_assist = knee_assist
        self.module = None
        self.module_joint_pause_s = 0.0
        self.runs, self.owned, self.completed = [], {}, []
        self.current = None
        self.all_done = False
        self.checkpoint = None
        self.test_completed = 0
        self.ls_firmware_version = None
        self.preflight_motor = None
        self.motion_test_running = False
        self.last_pump = self.clock()
        self.next_control = self.last_pump
        self.rate = max(t.settings.get("rate_hz", 100.0) or 100.0 for t in targets)

    def pump(self, force=False, spin=True):
        if self.checkpoint:
            self.checkpoint()
        self.log.check()
        now = self.clock()
        if self.runs and now - self.last_pump > min(r.p["feedback_timeout_s"] for r in self.runs):
            raise HomingError("CONTROL_LOOP_STALL")
        self.last_pump = now
        if self.runs and (force or now >= self.next_control):
            self.io.exclusive()
            commands = []
            for run in self.runs:
                f = self.io.feedback.get(run.target.motor.index)
                old_phase, old_brake_count = run.phase, run.brake_count
                old_reverse_count = run.motion_test_reverse_count
                old_test_half_cycles = run.motion_test_half_cycles
                try:
                    command = run.step(now, f)
                except HomingError:
                    self.current = run  # Report the motor that failed during concurrent return.
                    raise
                if run.phase == "SEEK" and run.seek_diagnostics and now >= run.next_seek_status_at:
                    self.log.write("SEEK_STATUS", index=run.target.motor.index, q_rad=f.q,
                                   dq_rad_s=f.dq, tau_nm=f.tau, **run.seek_diagnostics)
                    run.next_seek_status_at = now + 1.0
                if old_phase == "SEEK" and run.phase in {"STOP", "ZERO_REQUEST"}:
                    self.log.write("CONTACT_DETECTED", index=run.target.motor.index,
                                   contact_position_rad=run.contact_q, **run.seek_diagnostics)
                if f is not None:
                    q, dq, kp, kd, tau_ff = command
                    position_term = kp * (q - f.q)
                    velocity_term = kd * (dq - f.dq)
                    estimate = position_term + velocity_term + tau_ff
                    if abs(estimate) > run.p["max_torque_nm"] + 1e-9:
                        previous = run.previous_feedback
                        interval = f.received_at - previous.received_at if previous else None
                        delta = f.q - previous.q if previous else None
                        run.fail("COMMAND_TORQUE_BUDGET", diagnostics={
                            "elapsed_s": now - run.started,
                            "command": {"q_rad": q, "dq_rad_s": dq, "kp": kp, "kd": kd,
                                        "tau_ff_nm": tau_ff, "published": False},
                            "feedback": {"q_rad": f.q, "dq_rad_s": f.dq, "tau_nm": f.tau,
                                         "temperature_c": f.temperature, "sequence": f.sequence,
                                         "age_s": now - f.received_at},
                            "torque_estimate": {"position_term_nm": position_term,
                                                "velocity_term_nm": velocity_term,
                                                "total_nm": estimate, "limit_nm": run.p["max_torque_nm"]},
                            "previous_feedback": ({"q_rad": previous.q, "dq_rad_s": previous.dq,
                                                   "sequence": previous.sequence} if previous else None),
                            "feedback_interval_s": interval,
                            "position_delta_rad": delta,
                            # ROS receive timing, not the drive's sampling clock; diagnostic only.
                            "position_rate_from_ros_receive_rad_s": delta / interval if interval and interval > 0 else None,
                        })
                if run.brake_count != old_brake_count:
                    previous = run.previous_feedback
                    self.log.write("BRAKE_START", index=run.target.motor.index, reason=run.brake_reason,
                                   attempt=run.brake_count, q_rad=f.q, dq_rad_s=f.dq,
                                   previous_q_rad=previous.q if previous else None,
                                   position_delta_rad=f.q - previous.q if previous else None,
                                   feedback_interval_s=f.received_at - previous.received_at if previous else None,
                                   reference_rad=run.return_reference,
                                   reference_error_rad=run.return_reference - f.q if run.return_reference is not None else None,
                                   configured_kd=run.p["brake_kd"], actual_kd=command[3],
                                   speed_limit_rad_s=run.return_speed_limit_rad_s,
                                   emergency_speed_rad_s=run.return_emergency_speed_rad_s,
                                   catchup_active=run.return_catchup_active,
                                   torque_limit_nm=run.p["brake_max_torque_nm"])
                elif old_phase == "BRAKE" and run.phase != "BRAKE":
                    self.log.write("BRAKE_RESUME", index=run.target.motor.index, phase=run.phase,
                                   start_rad=run.move_start, target_rad=run.position_target,
                                   dq_rad_s=f.dq, attempt=run.brake_count)
                if old_phase != "RECOVER" and run.phase == "RECOVER":
                    self.log.write("HOLD_RECOVER_START", index=run.target.motor.index,
                                   reason="COMPENSATION_HANDOFF_LIMIT" if run.hold_transfer_limited else "POSITION_ERROR",
                                   target_rad=run.position_target, q_rad=f.q, dq_rad_s=f.dq,
                                   kp=command[2], kd=command[3], tau_ff_nm=command[4],
                                   requested_torque_nm=run.hold_requested_torque_nm,
                                   applied_estimate_nm=run.hold_applied_torque_nm, limited=run.hold_limited)
                elif old_phase == "RECOVER" and run.phase == "HOLD":
                    self.log.write("HOLD_RECOVERED", index=run.target.motor.index,
                                   target_rad=run.position_target, q_rad=f.q,
                                   adaptive_torque_nm=run.adaptive_torque)
                if run.motion_test_reverse_count != old_reverse_count:
                    self.log.write("MOTION_TEST_REVERSE", index=run.target.motor.index,
                                   count=run.motion_test_reverse_count, **run.motion_test_reversal,
                                   dq_target_rad_s=command[1], tau_ff_nm=command[4])
                if run.motion_test_half_cycles != old_test_half_cycles:
                    self.log.write("MOTION_TEST_DIRECTION_CHANGE", index=run.target.motor.index,
                                   direction_changes=run.motion_test_half_cycles,
                                   requested_direction_changes=2 * run.motion_test["cycles"],
                                   total_turn_extension_s=run.motion_test_total_extension_s, q_rad=f.q,
                                   direction=run.motion_test_last_direction,
                                   dq_target_rad_s=command[1], tau_ff_nm=command[4])
                if run.motion_test_half_cycles // 2 != old_test_half_cycles // 2:
                    self.log.write("MOTION_TEST_CYCLE", index=run.target.motor.index,
                                   completed_cycles=run.motion_test_half_cycles // 2,
                                   requested_cycles=run.motion_test["cycles"], q_rad=f.q)
                commands.append((run.target.motor, command))
            for run in self.runs:
                repeat_enable = (run.target.motor.protocol != "LS" and run.zero_sent and
                                 not run.zero_feedback_received and run.phase in {"ZERO_PENDING", "BRAKE"})
                if run.zero_reenable_pending or repeat_enable:
                    # LS re-enable requests are scheduled only after its SETZERO ACK.
                    # Send it before the same cycle's MIT command.
                    self.io.enable([run.target.motor], True)
                    run.zero_enable_count += 1
                    run.zero_reenable_pending = False
            self.io.command(commands)
            published_at = self.clock()
            for run, (_, command) in zip(self.runs, commands):
                q, dq, kp, kd, tau_ff = command
                f = run.last
                # ROS publish returned, not confirmation of CAN delivery or execution.
                run.published_commands.append({
                    "elapsed_s": published_at - run.started, "phase": run.phase,
                    "q_rad": q, "dq_rad_s": dq, "kp": kp, "kd": kd, "tau_ff_nm": tau_ff,
                    "adaptive_torque_nm": run.adaptive_torque, "adaptive_limited": run.adaptive_limited,
                    "motion_test_torque_nm": run.motion_test_torque_nm,
                    "return_reference_limited": run.return_reference_limited if run.phase == "MOVE" else False,
                    "motion_span_rad": run.motion_span_rad,
                    "motion_mean_velocity_rad_s": run.motion_mean_velocity_rad_s,
                    "brake_count": run.brake_count, "brake_limited": run.brake_limited if run.phase == "BRAKE" else False,
                    "hold_limited": run.hold_limited if run.phase in {"HOLD", "RECOVER"} else False,
                    "hold_requested_torque_nm": run.hold_requested_torque_nm,
                    "hold_applied_torque_nm": run.hold_applied_torque_nm,
                    "feedback": ({"sequence": f.sequence, "q_rad": f.q, "dq_rad_s": f.dq,
                                  "tau_nm": f.tau, "received_elapsed_s": f.received_at - run.started}
                                 if f else None),
                })
                if run.phase == "MOVE" and f is not None and now >= run.next_return_status_at:
                    estimate = kp * (q - f.q) + kd * (dq - f.dq) + tau_ff
                    self.log.write("RETURN_STATUS", index=run.target.motor.index,
                                   reference_rad=q, target_rad=run.position_target, q_rad=f.q,
                                   reference_error_rad=q - f.q, dq_target_rad_s=dq, dq_rad_s=f.dq,
                                   reference_limited=run.return_reference_limited,
                                   catchup_active=run.return_catchup_active,
                                   speed_damping_nm=run.return_speed_damping_nm,
                                   endpoint_handoff_nm=run.endpoint_handoff_nm,
                                   speed_limit_rad_s=run.return_speed_limit_rad_s,
                                   emergency_speed_rad_s=run.return_emergency_speed_rad_s,
                                   max_lead_rad=run.p["return_max_lead_rad"], kp=kp, kd=kd,
                                   arrival_tolerance_deg=math.degrees(run.p["arrival_tolerance_rad"]),
                                   adaptive_ki_frozen=run.adaptive_ki_frozen,
                                   motion_test_torque_nm=run.motion_test_torque_nm,
                                   motion_test_velocity_rad_s=run.motion_test_velocity_rad_s,
                                   tau_ff_nm=tau_ff, estimate_nm=estimate, feedback_tau_nm=f.tau,
                                   max_torque_nm=run.p["max_torque_nm"])
                    run.next_return_status_at = now + 1.0
            self.next_control = now + 1 / self.rate
        if spin:
            self.io.spin(max(0.0, self.next_control - self.clock()) if self.runs else 1 / self.rate)

    def wait(self, condition, timeout, reason):
        deadline = self.clock() + timeout
        while not condition():
            if self.clock() >= deadline:
                raise HomingError(reason)
            self.pump()

    def response(self, future, reason):
        self.wait(future.done, self.timeout, reason)
        result = future.result()
        if result is None:
            raise HomingError(reason)
        return result

    def preflight(self, motion):
        if self.checkpoint:
            self.checkpoint()
        self.wait(self.io.ready, self.timeout, "ROS service/subscriber unavailable")
        if motion:
            self.io.exclusive()
            response = self.response(self.io.normal_mode(), "Cannot verify hpm_system mode")
            values = response.values
            if len(values) < 2 or any(v.type != 1 for v in values[:2]) or values[0].bool_value or not values[1].bool_value:
                raise HomingError("Require motor_id_calibration_mode=false and launch_motor=true")
            if any(t.motor.protocol == "LS" for t in self.targets):
                if len(values) != 3 or values[2].type != 4 or values[2].string_value not in {"old", "new"}:
                    raise HomingError("Cannot verify hpm_system ls_firmware_version (expected old or new)")
                self.ls_firmware_version = values[2].string_value
                self.log.write("LS_FIRMWARE_VERSION", version=self.ls_firmware_version)
        checked_ls = set()
        for target in self.targets:
            motor = self.preflight_motor = target.motor
            if motor.protocol not in {"RS", "MR", "LS"}:
                raise HomingError(f"{motor.name}: ID query unsupported for {motor.protocol}")
            group, allowed = [motor], {motor.motor_id}
            if motor.protocol == "LS":
                if motor.bus in checked_ls:
                    continue
                checked_ls.add(motor.bus)
                group = [t.motor for t in self.targets if t.motor.protocol == "LS" and t.motor.bus == motor.bus]
                allowed = {m.motor_id for m in self.motors if m.protocol == "LS" and m.bus == motor.bus}
            expected = {m.motor_id for m in group}
            for attempt in range(1, 4):
                if self.checkpoint:
                    self.checkpoint()
                try:
                    response = self.response(self.io.query(motor), f"{motor.name}: ID query timeout")
                    extras = validate_id_reply(motor, response, expected, allowed)
                except Exception as exc:
                    self.log.write("ONLINE_FAIL", indices=[m.index for m in group], attempt=attempt, reason=str(exc))
                    raise
                self.log.write("ID_QUERY_PASS", index=motor.index, attempt=attempt,
                               detected_ids=list(response.detected_ids), known_bystanders=extras)
            for item in group:
                self.log.write("ONLINE_PASS", **asdict(item))
        self.log.write("PREFLIGHT_PASS", selected=len(self.targets), scope="ID queries only")

    def damp_knee(self, run, kd, stage):
        run.p = {**run.p, "damping_kd": kd}
        run.transition("DAMPING", self.clock())
        self.log.write("KNEE_ASSIST_DAMPING", index=run.target.motor.index,
                       stage=stage, kd=kd, kp=0.0, dq_rad_s=0.0, tau_ff_nm=0.0)

    def confirm_ls_zero(self, run):
        """Require the dedicated LS ACK and fresh zero coordinates before return."""
        motor, p = run.target.motor, run.p
        unlimited = p["ls_zero_retry_until_success"]
        zero_started_at = self.clock()
        total_deadline = zero_started_at + p["ls_zero_total_timeout_s"]
        attempt = 0
        while unlimited or attempt < p["zero_max_attempts"]:
            attempt += 1
            if self.clock() >= total_deadline:
                break
            attempt_started = self.clock()
            deadline = min(total_deadline, attempt_started + p["ls_zero_attempt_timeout_s"])
            run.zero_attempts = attempt
            run.zero_feedback_received = False
            run.transition("ZERO_PENDING", self.clock())
            self.pump(force=True, spin=False)
            self.log.write("ZERO_REQUEST", index=motor.index, attempt=attempt,
                           write_count="until_timeout" if unlimited else p["zero_max_attempts"],
                           confirmation="LS_ACK_AND_ZERO_POSITION", retry_until_success=unlimited,
                           total_timeout_s=p["ls_zero_total_timeout_s"],
                           remaining_timeout_s=max(0.0, total_deadline - self.clock()))
            run.zero_sent = True
            future = self.io.zero(motor)
            # A lost ROS response is not a completed device timeout: do not
            # overlap a still-running server transaction with another request.
            wait_reason = "LS_ZERO_TOTAL_TIMEOUT" if deadline == total_deadline else "LS_ZERO_SERVICE_TIMEOUT"
            self.wait(future.done, max(0.0, deadline - self.clock()), f"{motor.name}: {wait_reason}")
            response = future.result()
            if (response is None or list(response.motor_types) != [motor.index] or
                    len(response.results) != 1):
                run.fail(f"LS_ZERO_RESPONSE_INVALID: {response}")
            result = response.results[0]
            if result != 0 and result != response.TIMEOUT and not (
                    unlimited and result in (response.DEVICE_FAILED, response.REJECTED)):
                run.fail(f"LS_ZERO_FAILED: {response}")
            run.zero_dispatched = True
            reason = "ACK_TIMEOUT" if result != 0 and result == response.TIMEOUT else "WRITE_REJECTED"
            if result == 0:
                previous, consecutive = None, 0
                run.zero_reenable_pending = True
                self.pump(force=True, spin=False)
                self.log.write("ZERO_ACK", index=motor.index, attempt=attempt, reenable_requested=True)
                enable_requests = 1
                required_enables = 3 if run.legacy_ls else 1
                last_enable = self.clock()
                while self.clock() < deadline:
                    if enable_requests < required_enables and self.clock() - last_enable >= 0.05:
                        run.zero_reenable_pending = True
                        self.pump(force=True, spin=False)
                        last_enable = self.clock()
                        enable_requests += 1
                        consecutive = 0
                    f = self.io.feedback.get(motor.index)
                    if (f is not None and f.received_at > last_enable and
                            (previous is None or (f.sequence != previous.sequence and
                                                  f.received_at > previous.received_at))):
                        run.step(self.clock(), f)
                        previous = f
                        consecutive = consecutive + 1 if abs(f.q) <= p["post_zero_tolerance_rad"] else 0
                        if consecutive >= 2 and enable_requests == required_enables:
                            run.zero_feedback_received = True
                            self.log.write("ZERO_CONFIRMED", index=motor.index, attempt=attempt,
                                           q_rad=f.q, device_verified=True,
                                           confirmation="LS_ACK_AND_ZERO_POSITION",
                                           enable_requests=enable_requests)
                            return
                    self.pump()
                reason = "ZERO_POSITION_TIMEOUT"
            self.log.write("ZERO_RETRY", index=motor.index, attempt=attempt, reason=reason,
                           will_retry=(unlimited or attempt < p["zero_max_attempts"]) and self.clock() < total_deadline,
                           retry_until_success=unlimited, response_result=int(result),
                           response_messages=list(response.messages), q_rad=run.last.q if run.last else None)
            retry_at = (min(total_deadline, attempt_started + min(0.2, p["ls_zero_attempt_timeout_s"]))
                        if unlimited else self.clock())
            self.wait(lambda: self.clock() >= retry_at, max(0.0, retry_at - self.clock()) + 1 / self.rate,
                      f"{motor.name}: LS_ZERO_RETRY_WAIT")
        if self.clock() >= total_deadline:
            run.fail("LS_ZERO_TOTAL_TIMEOUT", diagnostics={
                "zero_elapsed_s": self.clock() - zero_started_at,
                "ls_zero_total_timeout_s": p["ls_zero_total_timeout_s"], "zero_attempts": run.zero_attempts})
        run.fail("LS_ZERO_NOT_CONFIRMED: attempts exhausted")

    def confirm_return_ready(self, run):
        """Observe the new coordinate epoch and drive status before applying return gains."""
        motor, p = run.target.motor, run.p
        started = self.clock()
        deadline = started + p["feedback_start_timeout_s"]
        stable_s = 0.1  # The MR can report enabled for ~38 ms before SETZERO disables it.
        last_enable = started
        previous = None
        run.window.clear()
        run.zero_feedback_received = True  # End the dispatch burst; retry only on explicit disabled feedback.
        while self.clock() < deadline:
            self.pump()
            if self.clock() >= deadline:
                break
            f = self.io.feedback.get(motor.index)
            if f is None or f.received_at <= started or (previous is not None and
                    (f.sequence == previous.sequence or f.received_at <= previous.received_at)):
                continue
            run.step(self.clock(), f)
            if f.enabled is None and not run.legacy_ls:
                run.fail("ENABLE_FEEDBACK_UNAVAILABLE", diagnostics={"protocol": motor.protocol})
            if (not p["post_zero_static_check"] and previous is not None and
                    run.enable_ready(previous) and run.enable_ready(f) and
                    f.received_at - previous.received_at <= p["feedback_timeout_s"]):
                self.log.write("RETURN_FEEDBACK_READY", index=motor.index, q_rad=f.q, dq_rad_s=f.dq,
                               enabled=f.enabled, enable_verified=f.enabled is True, zero_verified=False,
                               handoff="TWO_FRESH_FRAMES", first_q_rad=previous.q, first_dq_rad_s=previous.dq,
                               feedback_interval_s=f.received_at - previous.received_at)
                return previous, f
            if not run.enable_ready(f) or abs(f.q) > p["post_zero_tolerance_rad"]:
                run.window.clear()
                if f.enabled is False and self.clock() - last_enable >= 0.05:
                    run.zero_reenable_pending = True
                    self.pump(force=True, spin=False)
                    last_enable = self.clock()
            elif p["post_zero_static_check"]:
                if previous and f.received_at - previous.received_at > max(0.05, 2 / self.rate):
                    run.window.clear()
                if not run.motion_settled(f, stable_s, p["stall_velocity_rad_s"],
                                          position_limit=p["position_epsilon_rad"]):
                    previous = f
                    continue
                run.zero_confirmed = True
                self.log.write("RETURN_FEEDBACK_READY", index=motor.index, q_rad=f.q, dq_rad_s=f.dq,
                               enabled=f.enabled, enable_verified=f.enabled is True, stable_s=f.received_at - run.window[0].received_at,
                               position_span_rad=run.motion_span_rad,
                               post_zero_tolerance_rad=p["post_zero_tolerance_rad"],
                               zero_verified=True, first_q_rad=previous.q, first_dq_rad_s=previous.dq,
                               feedback_interval_s=f.received_at - previous.received_at)
                return previous, f
            previous = f
        run.fail("RETURN_READY_TIMEOUT", diagnostics={
            "timeout_s": p["feedback_start_timeout_s"], "required_stable_s": stable_s,
            "q_rad": previous.q if previous else None, "enabled": previous.enabled if previous else None,
            "post_zero_tolerance_rad": p["post_zero_tolerance_rad"]})

    def run_motion_test(self):
        self.test_completed = 0
        self.motion_test_running = True
        tested = []
        for run in self.runs:
            self.current = run
            motor = run.target.motor
            plan = motion_test_plan(run.target)
            original_p, original_final = run.p, run.final_phase
            support = run.adaptive_torque
            hold_support_source = None
            if run.arm_motion_test:
                total = run.stable_hold_torque(self.clock())
                hold_support_source = "stable_hold_total" if total is not None else "adaptive_fallback"
                limits = run.target.limits
                limit = min(run.p["max_torque_nm"], -limits["t_min"], limits["t_max"])
                if run.p["adaptive_torque_enabled"]:
                    limit = min(limit, run.p["adaptive_torque_limit_nm"])
                base = total if total is not None else support if run.p["adaptive_torque_enabled"] else 0.0
                run.motion_test_decel_support_nm = max(-limit, min(limit, base))
                # Holding torque opposes the load. Without a usable direction,
                # stop assisting at the target speed in either direction.
                run.motion_test_downhill = (0 if total is None or abs(total) < 0.01 else
                                            -1 if total > 0 else 1)
            # Start velocity-mode testing in place; only return after the test.
            run.p = {**run.p, "arrival_tolerance_rad": plan["arrival_tolerance_rad"],
                     "return_speed_rad_s": plan["speed_cap_rad_s"]}
            run.motion_test = plan
            run.final_phase = "HOLD"
            run.return_started_at = None
            run.transition("MOTION_TEST", self.clock())
            self.log.write("MOTION_TEST_START", index=motor.index,
                           cycle_count_mode="command_direction_changes",
                           preposition=False, actual_start_rad=self.io.feedback[motor.index].q,
                           arrival_tolerance_deg=math.degrees(run.p["arrival_tolerance_rad"]),
                           kp=0.0, kd=run.p["kd"], ki_frozen=run.adaptive_ki_frozen, **plan)
            self.wait(lambda: run.motion_test_half_cycles >= 2 * plan["cycles"],
                      plan["timeout_s"] + 1 / self.rate,
                      f"{motor.name}: MOTION_TEST_TIMEOUT")
            test_elapsed_s = self.clock() - run.phase_at
            if run.arm_motion_test:
                run.motion_test_decel_ready = False
                run.transition("MOTION_TEST_DECEL", self.clock())
                self.log.write("MOTION_TEST_DECEL_START", index=motor.index,
                               decel_kd=run.p.get("motion_test_decel_kd", run.p["kd"]),
                               timeout_s=run.p["brake_timeout_s"], hold_support_nm=run.motion_test_decel_support_nm,
                               hold_support_source=hold_support_source, downhill_direction=run.motion_test_downhill)
                self.wait(lambda: run.motion_test_decel_ready, run.p["brake_timeout_s"] + 1 / self.rate,
                          f"{motor.name}: MOTION_TEST_DECEL_TIMEOUT")
                self.log.write("MOTION_TEST_DECEL_PASS", index=motor.index, q_rad=run.last.q,
                               dq_rad_s=run.last.dq, position_span_rad=run.motion_span_rad,
                               tau_ff_nm=run.motion_test_torque_nm,
                               elapsed_s=self.clock() - run.phase_at)
                if run.p["adaptive_torque_enabled"]:
                    support = run.motion_test_torque_nm
            run.position_target = run.target.zero_target
            self.log.write("MOTION_TEST_RETURN", index=motor.index, target_rad=run.position_target,
                           elapsed_s=test_elapsed_s,
                           direction_changes=run.motion_test_half_cycles,
                           completed_cycles=run.motion_test_half_cycles // 2)
            self.return_after_test(run, support)
            run.p, run.final_phase = original_p, original_final
            if original_final == "DAMPING" or run.phase != "RECOVER":
                run.transition(original_final, self.clock())
            run.motion_test = None
            tested.append(motor.index)
            self.test_completed = len(tested)
            self.log.write("MOTION_TEST_MOTOR_PASS", index=motor.index, actual_rad=run.last.q,
                           target_rad=run.target.zero_target, final_mode=run.p["final_mode"])
        self.pump(force=True)
        self.motion_test_running = False
        self.current = None
        self.log.write("MOTION_TEST_PASS", module=self.module, tested=tested)

    def return_after_test(self, run, support):
        run.return_started_at = None
        run.brake_count = 0
        run.motion_test_return_active = True
        run.motion_test_torque_nm = 0.0
        run.motion_test_torque_sample = None
        try:
            run.begin_return(self.clock(), self.io.feedback[run.target.motor.index])
            # Preserve learned gravity support without re-enabling module Ki.
            if run.p["adaptive_torque_enabled"]:
                run.adaptive_torque = support
            self.pump(force=True)
            self.wait(lambda: run.phase == "HOLD" or (run.phase == "RECOVER" and run.hold_transfer_limited),
                      run.p["max_return_time_s"] + 1 / self.rate,
                      f"{run.target.motor.name}: motion test return timeout")
        finally:
            run.motion_test_return_active = False
            run.motion_test_torque_nm = 0.0
            run.motion_test_torque_sample = None

    def calibrate(self, write_zero=False, *, keep_running=True):
        if not write_zero and len(self.targets) != 1:
            raise HomingError("Seek-only diagnosis requires one --motor-index")
        for ordinal, target in enumerate(self.targets):
            if self.checkpoint:
                self.checkpoint()
            knee = None
            if self.knee_assist and target.motor.index == self.knee_assist["hip_index"]:
                knee = next((r for r in self.runs if r.target.motor.index == self.knee_assist["knee_index"]), None)
                if knee is None or knee.target.motor.index not in self.completed:
                    raise HomingError("KNEE_ASSIST_NOT_CALIBRATED")
                knee_settings, knee_hold_target = knee.p, knee.position_target
                self.damp_knee(knee, self.knee_assist["hip_pitch_assist_seek_kd"], "seek")
            motor = target.motor
            self.current = run = MotorRun(target, self.clock())
            run.arm_motion_test = self.module in {"left_arm", "right_arm"}
            run.legacy_ls = motor.protocol == "LS" and self.ls_firmware_version == "old"
            run.require_enable_status = write_zero
            if knee is not None:
                run.p = {**run.p, "final_mode": "position"}
                run.final_phase = "HOLD"
            self.runs.append(run)
            self.owned[motor.index] = motor  # Register before publish: cleanup even if sending fails.
            self.log.write("MOTOR_START", **asdict(motor), seek_direction=run.p["seek_direction"],
                           home_angle_deg=run.p["home_angle_deg"], target_rad=target.zero_target)
            self.io.enable([motor], True)
            self.pump(force=True)
            self.wait(lambda: run.phase == "ZERO_REQUEST", run.p["max_seek_time_s"] + run.p["stop_timeout_s"] +
                      run.p["feedback_start_timeout_s"], f"{motor.name}: homing timeout")
            if not write_zero:
                # main's finally immediately neutralizes/disables this motor.
                self.log.write("CONTACT_CANDIDATE", **asdict(motor), contact_position_rad=run.contact_q,
                               travel_rad=run.travel, peak_torque_nm=run.peak_torque,
                               zero_written=False, mechanical_stop_confirmed=False)
                return
            run.transition("ZERO_PREPARE", self.clock())
            # Maintain seek preload while checking the contact reference.
            def ready_to_zero():
                f = self.io.feedback.get(motor.index)
                if f is None or f.received_at <= run.phase_at:
                    return False
                run.step(self.clock(), f)
                if f.enabled is None and not run.legacy_ls:
                    run.fail("ENABLE_FEEDBACK_UNAVAILABLE", diagnostics={"protocol": motor.protocol})
                return run.motion_settled(f, run.p["stop_settle_s"], run.p["stall_velocity_rad_s"],
                                          position_limit=run.p["position_epsilon_rad"])
            self.wait(ready_to_zero, run.p["stop_timeout_s"], f"{motor.name}: ZERO_PREPARE_TIMEOUT")
            if motor.protocol == "LS":
                self.confirm_ls_zero(run)
            write_count = (0 if motor.protocol == "LS" else
                           run.p["zero_max_attempts"] if motor.protocol in {"MR", "RS", "DM"} else 1)
            for _ in range(write_count):
                run.zero_attempts += 1
                run.transition("ZERO_PENDING", self.clock())
                self.pump(force=True, spin=False)
                self.log.write("ZERO_REQUEST", index=motor.index, attempt=run.zero_attempts,
                               write_count=write_count, confirmation="TRANSPORT_SEND_ONLY")
                run.zero_sent = True
                future = self.io.zero(motor)
                self.pump(force=True, spin=False)
                response = self.response(future, f"{motor.name}: ZERO_SEND_TIMEOUT")
                if (list(response.motor_types) != [motor.index] or len(response.results) != 1 or
                        response.results[0] != 0):
                    run.fail(f"ZERO_SEND_FAILED: {response}")
                run.zero_dispatched = True
                for _ in range(3):
                    run.zero_reenable_pending = True
                    self.pump(force=True, spin=False)
                self.log.write("ZERO_SENT", index=motor.index, attempt=run.zero_attempts,
                               device_verified=False, immediate_enable_count=3, continuous_reenable=True,
                               command=run.published_commands[-1])
            first, observed = self.confirm_return_ready(run)
            run.return_feedback_ready = True
            if knee is not None:
                self.damp_knee(knee, self.knee_assist["hip_pitch_assist_return_kd"], "return")
            run.begin_return(self.clock(), observed)
            # Let normal MOVE guards inspect the two same-coordinate samples.
            run.previous_feedback, run.last = None, first
            self.pump(force=True)
            self.log.write("RETURN_START", index=motor.index, start_rad=run.move_start,
                           target_rad=target.zero_target, phase=run.phase,
                           final_mode=run.p["final_mode"],
                           adaptive_torque_enabled=run.p["adaptive_torque_enabled"],
                           tau_ff_nm=run.published_commands[-1]["tau_ff_nm"])
            self.wait(lambda: run.phase == run.final_phase or
                      (run.final_phase == "HOLD" and run.phase == "RECOVER" and run.hold_transfer_limited),
                      run.p["max_return_time_s"] + 1 / self.rate,
                      f"{motor.name}: return timeout")
            if not run.enable_ready(self.io.feedback[motor.index]):
                run.fail("ENABLE_FEEDBACK_LOST")
            self.completed.append(motor.index)
            self.log.write("MOTOR_PASS", **asdict(motor), contact_position_rad=run.contact_q,
                           set_zero="CONFIRMED" if motor.protocol == "LS" else "SENT_UNVERIFIED",
                           zero_reply_kind="LS_ACK_AND_ZERO_POSITION" if motor.protocol == "LS" else "TRANSPORT_SEND_ONLY",
                           zero_coordinate_verified=run.zero_confirmed, return_enable_verified=not run.legacy_ls,
                           zero_attempts=run.zero_attempts, target_rad=target.zero_target,
                           final_mode=run.p["final_mode"], phase=run.phase,
                           actual_rad=run.last.q, error_rad=run.last.q - target.zero_target,
                           adaptive_torque_nm=run.adaptive_torque,
                           peak_torque_nm=run.peak_torque, elapsed_s=self.clock() - run.started)
            if knee is not None:
                knee.p = knee_settings
                knee.final_phase = "HOLD"
                knee.position_target = knee_hold_target
                knee.return_started_at = None
                self.current = knee
                knee.begin_return(self.clock(), self.io.feedback[knee.target.motor.index])
                self.pump(force=True)
                self.log.write("KNEE_ASSIST_RESTORE", index=knee.target.motor.index,
                               target_rad=knee_hold_target, actual_rad=knee.last.q)
                self.wait(lambda: knee.phase == "HOLD" or
                          (knee.phase == "RECOVER" and knee.hold_transfer_limited),
                          knee.p["max_return_time_s"] + 1 / self.rate,
                          f"{knee.target.motor.name}: restore hold timeout")
                self.log.write("KNEE_ASSIST_DONE", index=knee.target.motor.index,
                               stage="hold", target_rad=knee.position_target,
                               actual_rad=knee.last.q, phase=knee.phase, control_restored=True)
            if self.module_joint_pause_s > 0 and ordinal + 1 < len(self.targets):
                pause_until = self.clock() + self.module_joint_pause_s
                self.log.write("JOINT_PAUSE_START", index=motor.index,
                               next_index=self.targets[ordinal + 1].motor.index,
                               duration_s=self.module_joint_pause_s)
                # Keep all enabled motors controlled and all feedback guards active.
                self.wait(lambda: self.clock() >= pause_until,
                          self.module_joint_pause_s + 1 / self.rate, "JOINT_PAUSE_TIMEOUT")
                self.log.write("JOINT_PAUSE_END", index=motor.index,
                               next_index=self.targets[ordinal + 1].motor.index)
        self.pump(force=True)
        self.all_done = True
        self.current = None
        # Freeze Ki only after the entire module (including knee restore) passes.
        # Retain learned support; PD gain handoffs keep their existing rebasing.
        if self.module is not None:
            for run in self.runs:
                run.adaptive_ki_frozen = True
                run.adaptive_limited = False
        phases = {run.phase for run in self.runs}
        self.log.write("CALIBRATION_PASS", completed=self.completed,
                       state=next(iter(phases)) if len(phases) == 1 else "MIXED",
                       final_states={run.target.motor.index: run.phase for run in self.runs},
                       adaptive_ki_frozen_indices=[run.target.motor.index for run in self.runs
                                                   if run.adaptive_ki_frozen and run.p["adaptive_torque_enabled"]])
        if not keep_running:
            return
        while True:
            self.pump()

    def shutdown(self):
        """Explicit exit cleanup, called by main's finally while ROS is still alive.

        Disable every motor this run attempted to enable, including completed ones.
        Do not defer hardware cleanup to __del__: its timing is not guaranteed.
        """
        motors = list(self.owned.values())
        self.runs.clear()  # No ordinary motion/hold publishes after shutdown starts.
        if not motors:
            return True
        errors = []
        attempts, interval = 10, 0.05
        started = time.monotonic()
        try:
            self.io.command([(m, (0.0, 0.0, 0.0, 0.0, 0.0)) for m in motors])
        except Exception as exc:
            errors.append(str(exc))
        # After the initial neutral command, publish only disable requests.
        for _ in range(attempts):
            try:
                self.io.enable(motors, False)
            except Exception as exc:
                errors.append(str(exc))
            try:
                self.io.spin(0.0)
            except Exception as exc:
                errors.append(str(exc))
            # spin_once can return immediately on feedback (or raise). Keep real
            # spacing, including time for the final publish before ROS closes.
            time.sleep(interval)
        self.log.write("SHUTDOWN", motor_indices=list(self.owned), disable_confirmed=False,
                       disable_attempts=attempts, disable_interval_s=interval,
                       shutdown_elapsed_s=time.monotonic() - started,
                       message="Stop/disable requested; existing interface has no physical disable acknowledgement",
                       errors=errors)
        return not errors


def data_path(name):
    source = Path(__file__).resolve().parents[1] / "config" / name
    if source.is_file():
        return source
    from ament_index_python.packages import get_package_share_directory
    return Path(get_package_share_directory("set_zero")) / "config" / name


def motor_config_path():
    source = Path(__file__).resolve().parents[3] / "src/hpm_motor/config/motors.yaml"
    if source.is_file():
        return source
    from ament_index_python.packages import get_package_share_directory
    return Path(get_package_share_directory("hpm_motor")) / "config/motors.yaml"


def parse_args(argv):
    parser = argparse.ArgumentParser(description=__doc__)
    selection = parser.add_mutually_exclusive_group(required=True)
    selection.add_argument("--module", choices=MODULES, help=argparse.SUPPRESS)
    selection.add_argument("--motor-index", type=integer)
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument("--dry-run", action="store_true")
    mode.add_argument("--check-only", action="store_true")
    parser.add_argument("--write-zero", action=argparse.BooleanOptionalAction, default=None,
                        help="Override YAML write_zero: --write-zero enables SETZERO/return; --no-write-zero seeks only")
    parser.add_argument("--config", type=Path)
    parser.add_argument("--calibration-config", type=Path)
    parser.add_argument("--urdf", type=Path)
    parser.add_argument("--log-dir", type=Path, default=Path("set_zero_logs"))
    parser.add_argument("--service-timeout", type=float, default=10.0)
    parser.add_argument("--system-node", default="/hpm_system")
    for key in OVERRIDABLE:
        parser.add_argument("--" + key.replace("_", "-"), type=float, default=None,
                            help="Override this numeric setting for all selected motors")
    args = parser.parse_args(argv)
    if not math.isfinite(args.service_timeout) or args.service_timeout <= 0:
        parser.error("--service-timeout must be finite and positive")
    for key in OVERRIDABLE:
        value = getattr(args, key)
        if value is not None and not math.isfinite(value):
            parser.error(f"--{key.replace('_', '-')} must be finite")
    return args


def prepare_run(args):
    overrides = {key: getattr(args, key) for key in OVERRIDABLE if getattr(args, key) is not None}
    paths = (args.config or motor_config_path(), args.calibration_config or data_path("motor_zero_calibration.yaml"),
             args.urdf or data_path("l0_v5.urdf"))
    motors, selected = load_plan(*paths, args.module, args.motor_index, overrides)
    calibration = read_yaml(paths[1])
    module_joint_pause_s = calibration.get("module_joint_pause_s", 0.5)
    if (isinstance(module_joint_pause_s, bool) or not isinstance(module_joint_pause_s, (int, float)) or
            not math.isfinite(module_joint_pause_s) or module_joint_pause_s < 0):
        raise ValueError("Top-level module_joint_pause_s must be a finite nonnegative number")
    if args.module is None:
        module_joint_pause_s = 0.0
    assist_enabled = calibration.get("hip_pitch_assist_enabled", True)
    if type(assist_enabled) is not bool:
        raise ValueError("Top-level hip_pitch_assist_enabled must be YAML true or false")
    knee_assist = None if args.check_only or not assist_enabled else knee_assist_plan(selected, args.module)
    configured_write_zero = calibration.get("write_zero", True)
    if type(configured_write_zero) is not bool:
        raise ValueError("Top-level write_zero must be YAML true or false")
    if args.write_zero is None:
        args.write_zero = configured_write_zero
    issues = {t.motor.name: motion_errors(t) for t in selected}
    if args.module is not None and not (args.check_only or args.dry_run):
        for target in selected:
            motion_test_plan(target)  # Validate every test before enabling any motor.
    return paths, motors, selected, module_joint_pause_s, knee_assist, issues


def acquire_motion_lock():
    import fcntl
    import os
    lock = open(f"/tmp/hpm_set_zero_{os.getuid()}_{os.environ.get('ROS_DOMAIN_ID', '0')}.lock", "a")
    try:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BaseException:
        lock.close()
        raise
    return lock


def main(argv=None):
    if __package__:
        from .version import APP_VERSION
    else:
        from version import APP_VERSION
    print(f"[VERSION] motor_auto_zero version={APP_VERSION}", flush=True)
    ros_args = list(sys.argv[1:] if argv is None else argv)
    cli_args = ros_args
    if "--ros-args" in cli_args:
        from rclpy.utilities import remove_ros_args
        cli_args = remove_ros_args(args=["motor_auto_zero", *cli_args])[1:]
    args = parse_args(cli_args)
    if args.module is not None:
        print("整组标零请调用 /set_zero/calibrate Action", file=sys.stderr)
        return 2
    try:
        paths, motors, selected, module_joint_pause_s, knee_assist, issues = prepare_run(args)
        for target in selected:
            print(json.dumps({"event": "PLAN", **asdict(target.motor), "settings": target.settings,
                              "write_zero": args.write_zero,
                              "motion_errors": issues[target.motor.name]}, ensure_ascii=False), flush=True)
        if args.module is not None:
            print(json.dumps({"event": "MODULE_PLAN", "module": args.module,
                              "module_joint_pause_s": module_joint_pause_s}), flush=True)
        if knee_assist:
            print(json.dumps({"event": "KNEE_ASSIST_PLAN", **knee_assist}, ensure_ascii=False), flush=True)
        if args.dry_run:
            return 0
        if not args.check_only and any(issues.values()):
            raise ValueError("Motion blocked: fill/confirm all selected profiles listed in motion_errors")
        # Check-only pacing must not depend on incomplete motion settings.
        if args.check_only:
            for target in selected:
                target.settings["rate_hz"] = 100.0
    except (ValueError, KeyError, TypeError, OSError, yaml.YAMLError, ET.ParseError) as exc:
        print(f"CONFIG_ERROR: {exc}", file=sys.stderr)
        return 2

    logger = io = runner = lock = None
    code = 1
    previous_handlers = {}
    diagnostics = []

    def record(event, **data):
        # Error reporting must never prevent stop/disable or resource cleanup.
        try:
            if logger:
                logger.write(event, **data)
        except Exception as exc:
            diagnostics.append(f"LOG_ERROR {event}: {exc}; {data}")

    try:
        if not args.check_only:
            lock = acquire_motion_lock()

        def interrupt(signum, frame):
            raise KeyboardInterrupt

        for sig in (signal.SIGINT, signal.SIGTERM):
            previous_handlers[sig] = signal.signal(sig, interrupt)
        logger = Logger(args.log_dir)
        logger.write("RUN_START", configs=[str(p.resolve()) for p in paths], selected=[asdict(t.motor) for t in selected],
                     check_only=args.check_only, write_zero=args.write_zero, module=args.module,
                     knee_assist=knee_assist, module_joint_pause_s=module_joint_pause_s,
                     resolved_settings={t.motor.name: t.settings for t in selected})
        io = make_ros_io(selected, not args.check_only, args.system_node, ros_args)
        runner = Runner(io, motors, selected, logger, args.service_timeout)
        runner.knee_assist = knee_assist
        runner.module = args.module
        runner.module_joint_pause_s = module_joint_pause_s
        runner.preflight(not args.check_only)
        if args.check_only:
            logger.write("CHECK_PASS", selected=[t.motor.index for t in selected])
            code = 0
        else:
            runner.calibrate(write_zero=args.write_zero)
            code = 0  # Seek-only completed; this is not a calibration pass.
    except KeyboardInterrupt:
        code = 0 if runner and runner.all_done and not runner.motion_test_running else 130
        record("OPERATOR_STOP", completed=runner.completed if runner else [],
               zero_unknown=bool(runner and runner.current and runner.current.zero_unknown))
    except Exception as exc:
        current = runner.current if runner else None
        fault_details = ({"diagnostics": exc.diagnostics}
                         if isinstance(exc, HomingError) and exc.diagnostics is not None else {})
        if current:
            record("MOTOR_FAIL", **asdict(current.target.motor), phase=current.phase, reason=str(exc), **fault_details)
        record("RUN_FAIL", reason=str(exc), failed_index=current.target.motor.index if current else None,
               phase=current.phase if current else "IDLE" if runner and runner.all_done else "PREFLIGHT",
               completed=runner.completed if runner else [],
               not_started=[t.motor.index for t in selected if not runner or t.motor.index not in runner.owned],
               zero_unknown=bool(current and current.zero_unknown), **fault_details)
        diagnostics.append(f"FAILED: {exc}")
        code = 1
    finally:
        # Keep ROS alive for cleanup, including after Ctrl+C. A second signal must
        # not cut short the bounded best-effort stop sequence.
        for sig in previous_handlers:
            signal.signal(sig, signal.SIG_IGN)
        if runner:
            try:
                if not runner.shutdown():
                    code = 1
            except Exception as exc:
                code = 1
                diagnostics.append(f"SHUTDOWN_ERROR: {exc}")
        if io:
            try:
                io.close()
            except Exception as exc:
                code = 1
                diagnostics.append(f"ROS_CLOSE_ERROR: {exc}")
        if logger:
            record("RUN_END", exit_code=code)
            try:
                logger.close()
            except Exception as exc:
                code = 1
                diagnostics.append(f"LOG_CLOSE_ERROR: {exc}")
            print(f"Result log: {logger.path}", flush=True)
        if lock:
            lock.close()
        for sig, handler in previous_handlers.items():
            signal.signal(sig, handler)
    # Terminal backpressure must not delay the stop/disable attempts above.
    for diagnostic in diagnostics:
        print(diagnostic, file=sys.stderr)
    return code


if __name__ == "__main__":
    raise SystemExit(main())
