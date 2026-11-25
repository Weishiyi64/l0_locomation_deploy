#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
检查URDF文件中左右对称性
"""
import sys
import xml.etree.ElementTree as ET


def parse_urdf(filepath):
    """解析URDF文件"""
    tree = ET.parse(filepath)
    root = tree.getroot()
    return root


def get_inertial_data(link):
    """提取link的惯性数据"""
    inertial = link.find('inertial')
    if inertial is None:
        return None

    origin = inertial.find('origin')
    mass_elem = inertial.find('mass')
    inertia_elem = inertial.find('inertia')

    data = {}
    if origin is not None:
        data['origin_xyz'] = [float(x) for x in origin.get('xyz', '0 0 0').split()]
        data['origin_rpy'] = [float(x) for x in origin.get('rpy', '0 0 0').split()]

    if mass_elem is not None:
        data['mass'] = float(mass_elem.get('value', '0'))

    if inertia_elem is not None:
        data['inertia'] = {
            'ixx': float(inertia_elem.get('ixx', '0')),
            'ixy': float(inertia_elem.get('ixy', '0')),
            'ixz': float(inertia_elem.get('ixz', '0')),
            'iyy': float(inertia_elem.get('iyy', '0')),
            'iyz': float(inertia_elem.get('iyz', '0')),
            'izz': float(inertia_elem.get('izz', '0')),
        }

    return data


def get_joint_origin(joint):
    """提取joint的origin数据"""
    origin = joint.find('origin')
    if origin is None:
        return None

    return {
        'xyz': [float(x) for x in origin.get('xyz', '0 0 0').split()],
        'rpy': [float(x) for x in origin.get('rpy', '0 0 0').split()],
    }


def check_symmetry(left_data, right_data, link_name, tolerance=1e-5):
    """检查左右对称性"""
    issues = []

    # 检查质量
    if abs(left_data['mass'] - right_data['mass']) > tolerance:
        issues.append(
            f"  质量不对称: 左={left_data['mass']:.6f}, 右={right_data['mass']:.6f}, 差值={abs(left_data['mass'] - right_data['mass']):.6e}"
        )

    # 检查origin xyz (y坐标应该取反，x和z应该相同)
    left_xyz = left_data['origin_xyz']
    right_xyz = right_data['origin_xyz']

    if abs(left_xyz[0] - right_xyz[0]) > tolerance:
        issues.append(
            f"  Origin X不对称: 左={left_xyz[0]:.6e}, 右={right_xyz[0]:.6e}, 差值={abs(left_xyz[0] - right_xyz[0]):.6e}"
        )

    if abs(left_xyz[1] + right_xyz[1]) > tolerance:  # y应该取反
        issues.append(
            f"  Origin Y不对称: 左={left_xyz[1]:.6e}, 右={right_xyz[1]:.6e}, 期望右={-left_xyz[1]:.6e}, 差值={abs(left_xyz[1] + right_xyz[1]):.6e}"
        )

    if abs(left_xyz[2] - right_xyz[2]) > tolerance:
        issues.append(
            f"  Origin Z不对称: 左={left_xyz[2]:.6e}, 右={right_xyz[2]:.6e}, 差值={abs(left_xyz[2] - right_xyz[2]):.6e}"
        )

    # 检查惯量张量
    # 对于左右对称，惯量张量的变换规则：
    # ixx, izz 应该相同
    # iyy 应该相同
    # ixy 应该取反（因为y取反）
    # ixz 应该相同
    # iyz 应该取反（因为y取反）

    left_i = left_data['inertia']
    right_i = right_data['inertia']

    if abs(left_i['ixx'] - right_i['ixx']) > tolerance:
        issues.append(
            f"  惯量 Ixx不对称: 左={left_i['ixx']:.6e}, 右={right_i['ixx']:.6e}, 差值={abs(left_i['ixx'] - right_i['ixx']):.6e}"
        )

    if abs(left_i['iyy'] - right_i['iyy']) > tolerance:
        issues.append(
            f"  惯量 Iyy不对称: 左={left_i['iyy']:.6e}, 右={right_i['iyy']:.6e}, 差值={abs(left_i['iyy'] - right_i['iyy']):.6e}"
        )

    if abs(left_i['izz'] - right_i['izz']) > tolerance:
        issues.append(
            f"  惯量 Izz不对称: 左={left_i['izz']:.6e}, 右={right_i['izz']:.6e}, 差值={abs(left_i['izz'] - right_i['izz']):.6e}"
        )

    if abs(left_i['ixy'] + right_i['ixy']) > tolerance:  # 应该取反
        issues.append(
            f"  惯量 Ixy不对称: 左={left_i['ixy']:.6e}, 右={right_i['ixy']:.6e}, 期望右={-left_i['ixy']:.6e}, 差值={abs(left_i['ixy'] + right_i['ixy']):.6e}"
        )

    if abs(left_i['ixz'] - right_i['ixz']) > tolerance:
        issues.append(
            f"  惯量 Ixz不对称: 左={left_i['ixz']:.6e}, 右={right_i['ixz']:.6e}, 差值={abs(left_i['ixz'] - right_i['ixz']):.6e}"
        )

    if abs(left_i['iyz'] + right_i['iyz']) > tolerance:  # 应该取反
        issues.append(
            f"  惯量 Iyz不对称: 左={left_i['iyz']:.6e}, 右={right_i['iyz']:.6e}, 期望右={-left_i['iyz']:.6e}, 差值={abs(left_i['iyz'] + right_i['iyz']):.6e}"
        )

    return issues


def check_joint_symmetry(left_joint, right_joint, joint_name, tolerance=1e-5):
    """检查joint的对称性"""
    issues = []

    left_origin = get_joint_origin(left_joint)
    right_origin = get_joint_origin(right_joint)

    if left_origin is None or right_origin is None:
        return issues

    left_xyz = left_origin['xyz']
    right_xyz = right_origin['xyz']

    if abs(left_xyz[0] - right_xyz[0]) > tolerance:
        issues.append(
            f"  Joint Origin X不对称: 左={left_xyz[0]:.6e}, 右={right_xyz[0]:.6e}, 差值={abs(left_xyz[0] - right_xyz[0]):.6e}"
        )

    if abs(left_xyz[1] + right_xyz[1]) > tolerance:  # y应该取反
        issues.append(
            f"  Joint Origin Y不对称: 左={left_xyz[1]:.6e}, 右={right_xyz[1]:.6e}, 期望右={-left_xyz[1]:.6e}, 差值={abs(left_xyz[1] + right_xyz[1]):.6e}"
        )

    if abs(left_xyz[2] - right_xyz[2]) > tolerance:
        issues.append(
            f"  Joint Origin Z不对称: 左={left_xyz[2]:.6e}, 右={right_xyz[2]:.6e}, 差值={abs(left_xyz[2] - right_xyz[2]):.6e}"
        )

    # 检查rpy
    left_rpy = left_origin['rpy']
    right_rpy = right_origin['rpy']

    # 对于左右对称，rpy的变换规则：
    # Roll (绕X轴): 应该取反（因为左右镜像时，绕X轴的旋转会反向）
    # Pitch (绕Y轴): 应该相同（因为左右镜像时，绕Y轴的旋转方向不变）
    # Yaw (绕Z轴): 应该取反（因为左右镜像时，绕Z轴的旋转会反向）
    if abs(left_rpy[0] + right_rpy[0]) > tolerance:  # roll应该取反
        issues.append(
            f"  Joint RPY Roll不对称: 左={left_rpy[0]:.6e}, 右={right_rpy[0]:.6e}, 期望右={-left_rpy[0]:.6e}, 差值={abs(left_rpy[0] + right_rpy[0]):.6e}"
        )

    if abs(left_rpy[1] - right_rpy[1]) > tolerance:  # pitch应该相同
        issues.append(
            f"  Joint RPY Pitch不对称: 左={left_rpy[1]:.6e}, 右={right_rpy[1]:.6e}, 差值={abs(left_rpy[1] - right_rpy[1]):.6e}"
        )

    if abs(left_rpy[2] + right_rpy[2]) > tolerance:  # yaw应该取反
        issues.append(
            f"  Joint RPY Yaw不对称: 左={left_rpy[2]:.6e}, 右={right_rpy[2]:.6e}, 期望右={-left_rpy[2]:.6e}, 差值={abs(left_rpy[2] + right_rpy[2]):.6e}"
        )

    # 检查axis是否对称
    # 注意：对于左右对称的机器人，相同类型的joint（如pitch、roll、yaw）的axis应该完全相同
    # 因为axis表示旋转轴的方向，左右对称时旋转轴方向应该相同
    left_axis = left_joint.find('axis')
    right_axis = right_joint.find('axis')

    if left_axis is not None and right_axis is not None:
        left_axis_xyz = [float(x) for x in left_axis.get('xyz', '0 0 0').split()]
        right_axis_xyz = [float(x) for x in right_axis.get('xyz', '0 0 0').split()]

        # axis应该完全相同
        if abs(left_axis_xyz[0] - right_axis_xyz[0]) > tolerance:
            issues.append(
                f"  Joint Axis X不对称: 左={left_axis_xyz[0]:.6e}, 右={right_axis_xyz[0]:.6e}, 差值={abs(left_axis_xyz[0] - right_axis_xyz[0]):.6e}"
            )

        if abs(left_axis_xyz[1] - right_axis_xyz[1]) > tolerance:
            issues.append(
                f"  Joint Axis Y不对称: 左={left_axis_xyz[1]:.6e}, 右={right_axis_xyz[1]:.6e}, 差值={abs(left_axis_xyz[1] - right_axis_xyz[1]):.6e}"
            )

        if abs(left_axis_xyz[2] - right_axis_xyz[2]) > tolerance:
            issues.append(
                f"  Joint Axis Z不对称: 左={left_axis_xyz[2]:.6e}, 右={right_axis_xyz[2]:.6e}, 差值={abs(left_axis_xyz[2] - right_axis_xyz[2]):.6e}"
            )

    return issues


def main():
    urdf_file = '/home/luvbot/y1_v1_description/urdf/y1_v1_description.urdf'
    root = parse_urdf(urdf_file)

    # 定义左右对应的link和joint
    link_pairs = [
        ('left_shoulder_pitch_link', 'right_shoulder_pitch_link'),
        ('left_shoulder_roll_link', 'right_shoulder_roll_link'),
        ('left_shoulder_yaw_link', 'right_shoulder_yaw_link'),
        ('left_elbow_link', 'right_elbow_link'),
        ('left_wrist_pitch_link', 'right_wrist_pitch_link'),
        ('left_hip_pitch_link', 'right_hip_pitch_link'),
        ('left_hip_roll_link', 'right_hip_roll_link'),
        ('left_hip_yaw_link', 'right_hip_yaw_link'),
        ('left_knee_link', 'right_knee_link'),
        ('left_ankle_pitch_link', 'right_ankle_pitch_link'),
        ('left_ankle_roll_link', 'right_ankle_roll_link'),
    ]

    joint_pairs = [
        ('left_shoulder_pitch_joint', 'right_shoulder_pitch_joint'),
        ('left_shoulder_roll_joint', 'right_shoulder_roll_joint'),
        ('left_shoulder_yaw_joint', 'right_shoulder_yaw_joint'),
        ('left_elbow_joint', 'right_elbow_joint'),
        ('left_wrist_pitch_joint', 'right_wrist_pitch_joint'),
        ('left_hip_pitch_joint', 'right_hip_pitch_joint'),
        ('left_hip_roll_joint', 'right_hip_roll_joint'),
        ('left_hip_yaw_joint', 'right_hip_yaw_joint'),
        ('left_knee_joint', 'right_knee_joint'),
        ('left_ankle_pitch_joint', 'right_ankle_pitch_joint'),
        ('left_ankle_roll_joint', 'right_ankle_roll_joint'),
    ]

    # 收集所有link和joint
    links = {}
    joints = {}

    for link in root.findall('link'):
        links[link.get('name')] = link

    for joint in root.findall('joint'):
        joints[joint.get('name')] = joint

    print("=" * 80)
    print("URDF左右对称性检查报告")
    print("=" * 80)
    print()

    total_issues = 0

    # 检查link的对称性
    print("【Link对称性检查】")
    print("-" * 80)
    for left_name, right_name in link_pairs:
        if left_name not in links or right_name not in links:
            print(f"⚠️  {left_name} 或 {right_name} 不存在")
            continue

        left_link = links[left_name]
        right_link = links[right_name]

        left_data = get_inertial_data(left_link)
        right_data = get_inertial_data(right_link)

        if left_data is None or right_data is None:
            print(f"⚠️  {left_name} 或 {right_name} 缺少惯性数据")
            continue

        issues = check_symmetry(left_data, right_data, left_name)

        if issues:
            print(f"\n❌ {left_name} vs {right_name}:")
            for issue in issues:
                print(issue)
            total_issues += len(issues)
        else:
            print(f"✓ {left_name} vs {right_name}: 对称")

    print()

    # 检查joint的对称性
    print("【Joint对称性检查】")
    print("-" * 80)
    for left_name, right_name in joint_pairs:
        if left_name not in joints or right_name not in joints:
            print(f"⚠️  {left_name} 或 {right_name} 不存在")
            continue

        left_joint = joints[left_name]
        right_joint = joints[right_name]

        issues = check_joint_symmetry(left_joint, right_joint, left_name)

        if issues:
            print(f"\n❌ {left_name} vs {right_name}:")
            for issue in issues:
                print(issue)
            total_issues += len(issues)
        else:
            print(f"✓ {left_name} vs {right_name}: 对称")

    print()
    print("=" * 80)
    if total_issues == 0:
        print("✓ 所有检查项均通过！左右完全对称。")
    else:
        print(f"❌ 发现 {total_issues} 个不对称问题")
    print("=" * 80)


if __name__ == '__main__':
    main()
