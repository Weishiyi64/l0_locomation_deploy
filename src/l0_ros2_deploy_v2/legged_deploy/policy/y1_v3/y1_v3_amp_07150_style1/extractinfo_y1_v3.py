import os
import sys

import numpy as np
import onnx
import onnxruntime

JOINT_ORDER_MJ = [
    "waist_yaw_joint",
    "left_shoulder_pitch_joint",
    "left_shoulder_roll_joint",
    "left_shoulder_yaw_joint",
    "left_elbow_joint",
    "right_shoulder_pitch_joint",
    "right_shoulder_roll_joint",
    "right_shoulder_yaw_joint",
    "right_elbow_joint",
    "head_yaw_joint",
    "head_pitch_joint",
    "left_hip_pitch_joint",
    "left_hip_roll_joint",
    "left_hip_yaw_joint",
    "left_knee_joint",
    "left_ankle_pitch_joint",
    "left_ankle_roll_joint",
    "right_hip_pitch_joint",
    "right_hip_roll_joint",
    "right_hip_yaw_joint",
    "right_knee_joint",
    "right_ankle_pitch_joint",
    "right_ankle_roll_joint",
]

REQUIRED_ARRAY_KEYS = [
    "default_joint_pos",
    "joint_stiffness",
    "joint_damping",
    "action_scale",
]


def isaac_to_mj(isaac_data, joint_isaac, joint_mj=JOINT_ORDER_MJ):
    missing = [joint for joint in joint_mj if joint not in joint_isaac]
    if missing:
        raise ValueError(f"Policy metadata is missing MuJoCo joints: {missing}")
    return np.array([isaac_data[joint_isaac.index(joint)] for joint in joint_mj])


def parse_float_array(metadata, key):
    if key not in metadata:
        raise KeyError(f"ONNX metadata is missing required key: {key}")
    return np.array([float(value) for value in metadata[key].split(",")])


def validate_joint_sets(joint_order_isaac, joint_order_mj):
    missing_from_policy = [joint for joint in joint_order_mj if joint not in joint_order_isaac]
    extra_in_policy = [joint for joint in joint_order_isaac if joint not in joint_order_mj]
    if missing_from_policy or extra_in_policy:
        raise ValueError(
            "Policy and y1_v3 MuJoCo joint sets do not match. "
            f"missing_from_policy={missing_from_policy}, extra_in_policy={extra_in_policy}"
        )


def extract_yaml_metadata(metadata, joint_order_mj=JOINT_ORDER_MJ):
    if "joint_names" not in metadata:
        raise KeyError("ONNX metadata is missing required key: joint_names")

    joint_order_isaac = metadata["joint_names"].split(",")
    validate_joint_sets(joint_order_isaac, joint_order_mj)

    arrays_isaac = {key: parse_float_array(metadata, key) for key in REQUIRED_ARRAY_KEYS}
    bad_lengths = {key: len(value) for key, value in arrays_isaac.items() if len(value) != len(joint_order_isaac)}
    if bad_lengths:
        raise ValueError("Metadata arrays must match joint_names length " f"({len(joint_order_isaac)}): {bad_lengths}")

    return {
        "joint_order_isaac": joint_order_isaac,
        "joint_order_mj": list(joint_order_mj),
        "default_joint_angles": isaac_to_mj(
            arrays_isaac["default_joint_pos"],
            joint_order_isaac,
            joint_order_mj,
        ).tolist(),
        "kps": isaac_to_mj(
            arrays_isaac["joint_stiffness"],
            joint_order_isaac,
            joint_order_mj,
        ).tolist(),
        "kds": isaac_to_mj(
            arrays_isaac["joint_damping"],
            joint_order_isaac,
            joint_order_mj,
        ).tolist(),
        "action_scale": isaac_to_mj(
            arrays_isaac["action_scale"],
            joint_order_isaac,
            joint_order_mj,
        ).tolist(),
    }


def format_string_list(values):
    return ", ".join([f"'{value}'" for value in values])


def format_float_list(values):
    return ", ".join([f"{value:.4f}" for value in values])


def load_onnx_metadata(policy_path):
    model = onnx.load(policy_path)
    return model, {prop.key: prop.value for prop in model.metadata_props}


def print_metadata(model):
    print("All metadata properties:")
    if hasattr(model, "metadata_props") and model.metadata_props:
        for index, prop in enumerate(model.metadata_props, 1):
            print(f"{index:2d}. {prop.key}: {prop.value}")
    else:
        print("No metadata properties found.")


def print_yaml_metadata(yaml_metadata):
    print("\nYaml metadata for y1_v3:")
    print("joint_order_isaac:", yaml_metadata["joint_order_isaac"])
    print("joint_order_mj: [{}]".format(format_string_list(yaml_metadata["joint_order_mj"])))
    print("default_joint_angles: [{}]".format(format_float_list(yaml_metadata["default_joint_angles"])))
    print("kps: [{}]".format(format_float_list(yaml_metadata["kps"])))
    print("kds: [{}]".format(format_float_list(yaml_metadata["kds"])))
    print("action_scale: [{}]".format(format_float_list(yaml_metadata["action_scale"])))


def main():
    if len(sys.argv) != 2:
        print("Usage: python extractinfo_y1_v3.py <policy_file_path>")
        print("Example: python extractinfo_y1_v3.py /path/to/policy.onnx")
        sys.exit(1)

    policy_path = sys.argv[1]

    if not os.path.exists(policy_path):
        print(f"Policy file does not exist: {policy_path}")
        sys.exit(1)

    model, metadata = load_onnx_metadata(policy_path)
    print_metadata(model)
    print_yaml_metadata(extract_yaml_metadata(metadata))

    print("\nPolicy IO:")
    policy = onnxruntime.InferenceSession(policy_path)
    input_info = policy.get_inputs()[0]
    output_info = policy.get_outputs()[0]
    print(f"input_name: {input_info.name}")
    print(f"input_shape: {input_info.shape}")
    print(f"output_name: {output_info.name}")
    print(f"output_shape: {output_info.shape}")


if __name__ == "__main__":
    main()
