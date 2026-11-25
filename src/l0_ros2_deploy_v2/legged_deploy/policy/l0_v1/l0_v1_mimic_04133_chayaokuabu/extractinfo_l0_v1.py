import os
import sys
import numpy as np
import onnx
import onnxruntime


joint_order_mj = [
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
    "left_shoulder_pitch_joint",
    "left_shoulder_roll_joint",
    "left_elbow_roll_joint",
    "right_shoulder_pitch_joint",
    "right_shoulder_roll_joint",
    "right_elbow_roll_joint",
    "neck_yaw_joint",
    "neck_pitch_joint",
    "neck_roll_joint",
]


def isaac_to_mj(isaac_data, joint_isaac, joint_mj):
    return np.array([isaac_data[joint_isaac.index(joint)] for joint in joint_mj])


if __name__ == "__main__":
    # 检查命令行参数
    if len(sys.argv) != 2:
        print("❌ 使用方法: python extractinfo.py <policy_file_path>")
        print("   例如: python extractinfo.py /path/to/policy.onnx")
        sys.exit(1)
    
    policy_path = sys.argv[1]
    
    # 检查文件是否存在
    if not os.path.exists(policy_path):
        print(f"❌ 策略文件不存在: {policy_path}")
        sys.exit(1)

    model = onnx.load(policy_path)
    
    print("📋 所有元数据属性:")
    if hasattr(model, 'metadata_props') and model.metadata_props:
        for i, prop in enumerate(model.metadata_props, 1):
            print(f"{i:2d}. {prop.key}: {prop.value}")
    else:
        print("❌ 未找到元数据属性")

    print("\n🔍 Yaml所需的元数据属性:")
    for prop in model.metadata_props:
        if prop.key == "joint_names":
            joint_order_isaac = prop.value.split(",")
            print("joint_order_isaac:", joint_order_isaac)
            value = isaac_to_mj(joint_order_isaac, joint_order_isaac, joint_order_mj)
            value_str = ", ".join([f"'{v}'" for v in value])
            print("joint_order_mj: [{}]".format(value_str))
        if prop.key == "default_joint_pos":   
            joint_pos_array_seq = np.array([float(x) for x in prop.value.split(",")])
            joint_pos_array = isaac_to_mj(joint_pos_array_seq, joint_order_isaac, joint_order_mj)
            value = joint_pos_array
            value_str = ", ".join([f"{x:.4f}" for x in value])
            print("default_joint_angles: [{}]".format(value_str))
        if prop.key == "joint_stiffness":
            stiffness_array_seq = np.array([float(x) for x in prop.value.split(",")])
            stiffness_array = isaac_to_mj(stiffness_array_seq, joint_order_isaac, joint_order_mj)
            value = stiffness_array
            value_str = ", ".join([f"{x:.4f}" for x in value])
            print("kps: [{}]".format(value_str))
        if prop.key == "joint_damping":
            damping_array_seq = np.array([float(x) for x in prop.value.split(",")])
            damping_array = isaac_to_mj(damping_array_seq, joint_order_isaac, joint_order_mj)
            value = damping_array
            value_str = ", ".join([f"{x:.4f}" for x in value])
            print("kds: [{}]".format(value_str))
        if prop.key == "action_scale":
            action_scale = np.array([float(x) for x in prop.value.split(",")])
            action_array = isaac_to_mj(action_scale, joint_order_isaac, joint_order_mj)
            value = action_array
            value_str = ", ".join([f"{x:.4f}" for x in value])
            print("action_scale: [{}]".format(value_str))

    # 加载策略
    print("\n🔍 策略相关信息:")
    policy = onnxruntime.InferenceSession(policy_path)
    input_name = policy.get_inputs()[0].name
    output_name = policy.get_outputs()[0].name
    print(f"input_name: {input_name}")
    print(f"output_name: {output_name}")