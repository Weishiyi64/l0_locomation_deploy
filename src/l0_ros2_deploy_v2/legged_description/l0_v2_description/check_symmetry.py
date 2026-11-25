import math
import xml.etree.ElementTree as ET

URDF_PATH = 'urdf/l0_v2_description.urdf'
EPS_POS = 1e-3  # 位置阈值 m
EPS_MASS = 1e-4  # 质量阈值 kg
EPS_INERT = 1e-5  # 惯量阈值 kg·m²
EPS_LIM = 1e-3  # 限位阈值 rad

PAIRS = [
    ('left_hip_pitch_link', 'right_hip_pitch_link'),
    ('left_hip_roll_link', 'right_hip_roll_link'),
    ('left_hip_yaw_link', 'right_hip_yaw_link'),
    ('left_knee_link', 'right_knee_link'),
    ('left_ankle_pitch_link', 'right_ankle_pitch_link'),
    ('left_ankle_roll_link', 'right_ankle_roll_link'),
    ('left_shoulder_pitch_link', 'right_shoulder_pitch_link'),
    ('left_shoulder_roll_link', 'right_shoulder_roll_link'),
    ('left_elbow_roll_link', 'right_elbow_roll_link'),
]

LIMIT_SAME_ONLY = {
    'left_hip_pitch_link',
    'left_knee_link',
    'left_ankle_pitch_link',
    'left_shoulder_pitch_link',
}

KNOWN_FALSE_POSITIVES = {
    ('left_hip_pitch_link', 'right_hip_pitch_link', 'axis'),
    ('left_shoulder_pitch_link', 'right_shoulder_pitch_link', 'axis'),
    ('left_hip_roll_link', 'right_hip_roll_link', 'inertia_ixz'),
    ('left_hip_yaw_link', 'right_hip_yaw_link', 'inertia_ixz'),
    ('left_knee_link', 'right_knee_link', 'inertia_ixz'),
    ('left_ankle_roll_link', 'right_ankle_roll_link', 'inertia_ixz'),
}


def rpy_to_matrix(rpy):
    r, p, y = rpy
    cr, sr = math.cos(r), math.sin(r)
    cp, sp = math.cos(p), math.sin(p)
    cy, sy = math.cos(y), math.sin(y)
    Rz = [[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]]
    Ry = [[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]]
    Rx = [[1, 0, 0], [0, cr, -sr], [0, sr, cr]]

    def mm(A, B):
        return [[sum(A[i][k] * B[k][j] for k in range(3)) for j in range(3)] for i in range(3)]

    return mm(mm(Rz, Ry), Rx)


def mv(R, v):
    return [sum(R[i][j] * v[j] for j in range(3)) for i in range(3)]


def add(a, b):
    return [a[i] + b[i] for i in range(3)]


def compose(T1, T2):
    R1, t1 = T1
    R2, t2 = T2
    R = [[sum(R1[i][k] * R2[k][j] for k in range(3)) for j in range(3)] for i in range(3)]
    return (R, add(mv(R1, t2), t1))


I3 = [[1, 0, 0], [0, 1, 0], [0, 0, 1]]

tree = ET.parse(URDF_PATH)
root = tree.getroot()
child_to_parent = {}
joint_raw = {}
for j in root.findall('joint'):
    p = j.find('parent').get('link')
    c = j.find('child').get('link')
    oel = j.find('origin')
    xyz = list(map(float, oel.get('xyz', '0 0 0').split())) if oel is not None else [0, 0, 0]
    rpy = list(map(float, oel.get('rpy', '0 0 0').split())) if oel is not None else [0, 0, 0]
    ax = j.find('axis')
    axis = list(map(float, ax.get('xyz', '1 0 0').split())) if ax is not None else [1, 0, 0]
    lim = j.find('limit')
    limits = (
        {
            'lower': float(lim.get('lower', 0)),
            'upper': float(lim.get('upper', 0)),
            'effort': float(lim.get('effort', 0)),
            'velocity': float(lim.get('velocity', 0)),
        }
        if lim is not None
        else None
    )
    child_to_parent[c] = (p, (rpy_to_matrix(rpy), xyz))
    joint_raw[c] = {'parent': p, 'xyz': xyz, 'rpy': rpy, 'axis': axis, 'limits': limits}

link_data = {}
for link in root.findall('link'):
    name = link.get('name')
    mass_el = link.find('.//mass')
    iel = link.find('.//inertia')
    oel = link.find('.//inertial/origin')
    link_data[name] = {
        'mass': float(mass_el.get('value')) if mass_el is not None else None,
        'inertia': (
            {k: float(iel.get(k, 0)) for k in ['ixx', 'ixy', 'ixz', 'iyy', 'iyz', 'izz']} if iel is not None else None
        ),
        'com': list(map(float, oel.get('xyz', '0 0 0').split())) if oel is not None else [0, 0, 0],
    }


def get_global_T(name):
    if name not in child_to_parent:
        return (I3, [0, 0, 0])
    par, lT = child_to_parent[name]
    return compose(get_global_T(par), lT)


global_pos = {}
global_com = {}
for name in link_data:
    R, t = get_global_T(name)
    global_pos[name] = t
    global_com[name] = add(mv(R, link_data[name]['com']), t)

issues = []
skipped = []


def check(label, l, r, check_type, errs):
    if not errs:
        return
    key = (l, r, check_type)
    if key in KNOWN_FALSE_POSITIVES:
        skipped.append(f"[已知误报] [{label}] {l.replace('left_','')}: {errs}")
    else:
        issues.append(f"[{label}] {l.replace('left_','')}: {errs}")


for l, r in PAIRS:
    # A. 全局质心
    lc = global_com[l]
    rc = global_com[r]
    errs = []
    if abs(lc[0] - rc[0]) > EPS_POS:
        errs.append(f"x: L={lc[0]:.4f} R={rc[0]:.4f} Δ={abs(lc[0]-rc[0]):.4f}")
    if abs(lc[1] + rc[1]) > EPS_POS:
        errs.append(f"y之和={lc[1]+rc[1]:.4f} (应≈0)")
    if abs(lc[2] - rc[2]) > EPS_POS:
        errs.append(f"z: L={lc[2]:.4f} R={rc[2]:.4f} Δ={abs(lc[2]-rc[2]):.4f}")
    check('A.全局质心', l, r, 'com', errs)

    # B. 全局关节原点
    lp = global_pos[l]
    rp = global_pos[r]
    errs = []
    if abs(lp[0] - rp[0]) > EPS_POS:
        errs.append(f"x: Δ={abs(lp[0]-rp[0]):.4f}")
    if abs(lp[1] + rp[1]) > EPS_POS:
        errs.append(f"y之和={lp[1]+rp[1]:.4f} (应≈0)")
    if abs(lp[2] - rp[2]) > EPS_POS:
        errs.append(f"z: Δ={abs(lp[2]-rp[2]):.4f}")
    check('B.关节原点', l, r, 'origin', errs)

    # C. 质量
    lm = link_data[l]['mass']
    rm = link_data[r]['mass']
    if lm and rm and abs(lm - rm) > EPS_MASS:
        check('C.质量', l, r, 'mass', [f"L={lm:.4f} R={rm:.4f} Δ={abs(lm-rm):.4f} kg"])

    # D. 转动惯量
    li = link_data[l]['inertia']
    ri = link_data[r]['inertia']
    if li and ri:
        for k in ['ixx', 'iyy', 'izz']:
            if abs(li[k] - ri[k]) > EPS_INERT:
                check(f'D.惯量_{k}', l, r, f'inertia_{k}', [f"L={li[k]:.2e} R={ri[k]:.2e} Δ={abs(li[k]-ri[k]):.2e}"])
        for k in ['ixy', 'ixz', 'iyz']:
            if abs(li[k] + ri[k]) > EPS_INERT:
                check(f'D.惯量_{k}', l, r, f'inertia_{k}', [f"sum={li[k]+ri[k]:.2e} (应≈0)"])

    # E. 关节限位
    ll = joint_raw[l]['limits']
    rl = joint_raw[r]['limits']
    if ll and rl:
        same = abs(ll['lower'] - rl['lower']) < EPS_LIM and abs(ll['upper'] - rl['upper']) < EPS_LIM
        mirror = abs(ll['lower'] + rl['upper']) < EPS_LIM and abs(ll['upper'] + rl['lower']) < EPS_LIM
        if l in LIMIT_SAME_ONLY:
            if not same:
                check(
                    'E.限位',
                    l,
                    r,
                    'limits',
                    [
                        f"pitch/knee 限位必须完全一致: L=[{ll['lower']:.3f},{ll['upper']:.3f}] R=[{rl['lower']:.3f},{rl['upper']:.3f}]"
                    ],
                )
        else:
            if not mirror:
                check(
                    'E.限位',
                    l,
                    r,
                    'limits',
                    [
                        f"roll/yaw 限位必须互为镜像: L=[{ll['lower']:.3f},{ll['upper']:.3f}] R=[{rl['lower']:.3f},{rl['upper']:.3f}]"
                    ],
                )
        if abs(ll['effort'] - rl['effort']) > EPS_LIM:
            check('E.限位_effort', l, r, 'effort', [f"L={ll['effort']} R={rl['effort']}"])
        if abs(ll['velocity'] - rl['velocity']) > EPS_LIM:
            check('E.限位_vel', l, r, 'velocity', [f"L={ll['velocity']} R={rl['velocity']}"])

    # F. 关节轴方向
    RL, _ = get_global_T(l)
    RR, _ = get_global_T(r)
    al = mv(RL, joint_raw[l]['axis'])
    ar = mv(RR, joint_raw[r]['axis'])
    dot = sum(al[i] * ar[i] for i in range(3))
    if abs(abs(dot) - 1.0) > 1e-2:
        check('F.轴方向', l, r, 'axis', [f"L={[round(v,3) for v in al]} R={[round(v,3) for v in ar]} dot={dot:.3f}"])

# 总质量
total = sum(d['mass'] for d in link_data.values() if d['mass'])
print(f"总质量: {total:.4f} kg（共 {len(link_data)} 个 link）\n")

print("=" * 70)
if issues:
    print(f"发现 {len(issues)} 个问题（请反馈给机械部门重新导出）：")
    for i, s in enumerate(issues, 1):
        print(f"  {i}. {s}")
else:
    print("✓ 全部检查通过，未发现对称性问题")

if skipped:
    print(f"\n已自动跳过 {len(skipped)} 个已知误报：")
    for s in skipped:
        print(f"  {s}")
