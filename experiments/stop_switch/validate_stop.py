"""Deterministic, headless PartyMimic -> AMP stop experiment on the deployment MJCF."""

import argparse
from collections import deque
import hashlib
import importlib.util
import json
from pathlib import Path

import mujoco as mj
import numpy as np
import yaml
from scipy.spatial.transform import Rotation, Slerp

HERE = Path(__file__).resolve().parent
DEPLOY = HERE.parents[1] / "src/l0_ros2_deploy_v2"
DT = 0.02
BLEND = 30
HOLD_LIMIT = 150
POST = 168  # includes 0.35 s transition, followed by 3 s under AMP alone


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


class Clip:
    def __init__(self, path, names):
        with np.load(path, allow_pickle=False) as a:
            source_names = a["joint_names"].tolist()
            if sorted(names) != sorted(source_names) or a["fps"].item() != 50:
                raise ValueError(f"Unexpected motion ABI in {path}")
            self.q = a["joint_pos"][:, [source_names.index(n) for n in names]].astype(float)
            self.pos = a["root_pos"].astype(float)
            self.quat = a["root_quat_w"].astype(float)
        if not all(np.isfinite(x).all() for x in [self.q, self.pos, self.quat]):
            raise ValueError("Non-finite reference")
        self.frames = [
            dict(joint_pos=q, body_pos_w=p[None], body_quat_w=r[None])
            for q, p, r in zip(self.q, self.pos, self.quat, strict=True)
        ]

    def frame(self, i):
        return self.frames[min(max(i, 0), len(self.frames) - 1)]


class StopReference:
    """Armed only on STOP. Retain original frames through the selected landing frame."""

    def __init__(self, clip, start, stand, upper, root_stand=None):
        self.clip, self.start = clip, start
        first = clip.frame(start)
        q = first["joint_pos"][upper]
        dq = (clip.frame(start + 1)["joint_pos"][upper] - clip.frame(start - 1)["joint_pos"][upper]) / (
            2 * DT
        )
        self.plan = [first]
        if root_stand is not None:
            start_rot = Rotation.from_quat(clip.quat[start], scalar_first=True)
            target_rot = Rotation.from_quat(root_stand[1])
            yaw_delta = start_rot.as_euler("ZYX")[0] - target_rot.as_euler("ZYX")[0]
            target_rot = Rotation.from_euler("Z", yaw_delta) * target_rot
            rotations = Slerp([0, 1], Rotation.concatenate([start_rot, target_rot]))
        for k in range(1, BLEND + 1):
            u = k / BLEND
            joints = first["joint_pos"].copy()
            joints[upper] = (
                (2 * u**3 - 3 * u**2 + 1) * q
                + (u**3 - 2 * u**2 + u) * BLEND * DT * dq
                + (3 * u**2 - 2 * u**3) * stand[upper]
            )
            frame = dict(first, joint_pos=joints)
            if root_stand is not None:
                weight = u * u * (3 - 2 * u)
                pos = first["body_pos_w"].copy()
                pos[0, 2] += weight * (root_stand[0][2] - pos[0, 2])
                frame.update(
                    body_pos_w=pos, body_quat_w=rotations(weight).as_quat(scalar_first=True)[None]
                )
            self.plan.append(frame)

    def frame(self, i):
        if i < self.start:
            return self.clip.frame(i)
        return self.plan[min(i - self.start, BLEND)]


class Trial:
    def __init__(self, player, policy):
        self.p = player
        stand = json.loads((HERE / "l0_v5_amp_1124/stand_reference.json").read_text())
        self.names = stand["joint_names"]
        self.stand = np.array(stand["representative_frame"]["q_stand_rad"])
        self.root_stand = (
            stand["representative_frame"]["pelvis_pos_m"],
            stand["representative_frame"]["pelvis_quat_xyzw"],
        )
        self.upper = np.array(
            [
                i
                for i, n in enumerate(self.names)
                if n.startswith("neck_") or "shoulder" in n or "elbow" in n
            ]
        )
        self.leg = np.setdiff1d(np.arange(len(self.names)), self.upper)
        self.cfg = player._load_config(policy, self.names)
        self.wbc = player._session(policy, "cpu", 1)
        self.command_name = next(i.name for i in self.wbc.get_inputs() if i.name != "policy")
        self.scene = next(DEPLOY.rglob(stand["scene"]))
        self.model = mj.MjModel.from_xml_path(str(self.scene))
        self.model.jnt_solref[self.model.jnt_limited.astype(bool), 0] = 0.004
        self.decimation = round(DT / self.model.opt.timestep)
        self.qa, self.va, self.act = player._joint_addresses(self.model, self.names)
        self.floor = self.model.geom("floor").id
        self.feet = [self.model.body(n).id for n in stand["foot_body_names"]]
        amp_dir = DEPLOY / "legged_deploy/policy/l0_v5" / stand["policy_name"]
        self.amp_path = amp_dir / "policy.onnx"
        if (
            digest(self.amp_path) != stand["policy_sha256"]
            or digest(self.scene) != stand["scene_sha256"]
        ):
            raise ValueError("Stand capture and experiment assets disagree")
        amp = yaml.safe_load((amp_dir / "rl_config.yaml").read_text())["RobotCfg"]
        if amp["joint_order_isaac"] != self.names or amp["joint_order_mj"] != self.names:
            raise ValueError("AMP and tracking joint order mismatch")
        self.amp = player._session(self.amp_path, "cpu", 1)
        self.offset, self.scale, self.akp, self.akd = [
            np.array(amp[k]) for k in ("default_joint_angles", "action_scale", "kps", "kds")
        ]

    def support(self, d):
        force = np.zeros(2)
        wrench = np.zeros(6)
        for k, c in enumerate(d.contact):
            if self.floor not in (c.geom1, c.geom2):
                continue
            body = self.model.geom_bodyid[c.geom2 if c.geom1 == self.floor else c.geom1]
            if body in self.feet:
                mj.mj_contactForce(self.model, d, k, wrench)
                force[self.feet.index(body)] += wrench[0]
        return int(force[0] > 2) + 2 * int(force[1] > 2)

    def run(self, clip, stop=None, landing=None, legacy=False, whole_body=False):
        cfg, p = self.cfg, self.p
        d = mj.MjData(self.model)
        # Begin the tracking trial at clip frame zero in both branches.
        d.qpos[:3], d.qpos[3:7], d.qpos[self.qa] = clip.pos[0], clip.quat[0], clip.q[0]
        mj.mj_forward(self.model, d)
        b = p.NativeObservationBuilder(
            joint_count=21,
            history_steps=cfg["history"],
            future_steps=cfg["future"],
            default_joint_pos=cfg["offset"],
        )
        plan = None
        history = deque(maxlen=5)
        sent = d.qpos[self.qa].copy()
        release = None
        streak = 0
        rows, commands, actions = [], [], []
        dips = []
        foot_anchor = [None, None]
        max_loaded_foot_xy = np.zeros(2)
        prior_mask = 0
        result = dict(stop=stop, landing=landing, status="clip_complete")
        end = len(clip.frames) if stop is None else landing + BLEND + HOLD_LIMIT + POST
        for k in range(end):
            quat = d.qpos[3:7]
            g = p._quat_rotate_inverse(quat, np.array([0.0, 0.0, -1.0]))
            w = d.qvel[3:6].copy()
            q, v = d.qpos[self.qa], d.qvel[self.va]
            support = self.support(d)
            if stop is not None and k >= stop and release is None:
                for side, body in enumerate(self.feet):
                    xy = d.xpos[body, :2]
                    if not support & (1 << side):
                        foot_anchor[side] = None
                    elif foot_anchor[side] is None or not prior_mask & (1 << side):
                        foot_anchor[side] = xy.copy()
                    else:
                        displacement = np.linalg.norm(xy - foot_anchor[side])
                        max_loaded_foot_xy[side] = max(max_loaded_foot_xy[side], displacement)
                prior_mask = support
            speed, omega = np.linalg.norm(d.qvel[:2]), np.linalg.norm(w)
            tilt = np.rad2deg(np.arccos(np.clip(-g[2], -1, 1)))
            rows.append([support, speed, omega, tilt, d.qpos[2]])
            if k == stop:
                joints = np.arange(21) if whole_body else self.upper
                plan = StopReference(
                    clip, landing, self.stand, joints, self.root_stand if whole_body else None
                )
                result["leg_pose_rms_rad_at_blend"] = float(
                    np.sqrt(np.mean((clip.q[landing, self.leg] - self.stand[self.leg]) ** 2))
                )
                result["peak_planned_joint_speed_rad_s"] = float(
                    np.max(abs(np.diff([f["joint_pos"] for f in plan.plan], axis=0) / DT))
                )
                result["peak_planned_joint_acceleration_rad_s2"] = float(
                    np.max(abs(np.diff([f["joint_pos"] for f in plan.plan], n=2, axis=0) / (DT * DT)))
                )
                assert plan.frame(landing) is clip.frame(landing)
                assert np.allclose(plan.frame(landing + BLEND)["joint_pos"][joints], self.stand[joints])
                result["support_on_stop"] = support
            if tilt > 40 or d.qpos[2] < 0.29:
                result.update(status="fell", ended_at=k)
                break
            if release is None:
                b.push_robot_state(root_ang_vel=w, projected_gravity=g, joint_pos=q, joint_vel=v)
                if legacy:
                    refs = [clip.frame(k + i) for i in cfg["future"]]
                else:
                    reader = plan if plan is not None else clip
                    refs = [reader.frame(k + i) for i in cfg["future"]]
                cmd = b.command_input(
                    robot_root_pos=d.qpos[:3],
                    robot_root_quat=quat,
                    reference_frames=refs,
                    root_body_index=0,
                )[0]
                action = self.wbc.run(None, {"policy": b.policy_input()[0], self.command_name: cmd})[
                    0
                ].reshape(21)
                b.push_action(action)
                commands.append(cmd)
                actions.append(action)
                target = np.clip(action * cfg["scale"] + cfg["offset"], -10, 10)
                kp, kd = cfg["kp"], cfg["kd"]

            if plan and k >= landing + BLEND:
                if release is None:
                    # Probe the actual deployment seed algorithm on this physical state.
                    # Source is the preceding target sent to the simulated joints.
                    seed = np.divide(
                        sent - self.offset, self.scale, out=np.zeros(21), where=self.scale != 0
                    )
                    obs = np.r_[w, g, np.zeros(3), q - self.offset, v, seed]
                    amp_in = np.tile(obs, 5)
                else:
                    history.append(np.r_[w, g, np.zeros(3), q - self.offset, v, amp_action])
                    amp_in = np.concatenate(history)
                amp_action = self.amp.run(None, {"obs": amp_in.astype("f4")[None]})[0][0]
                amp_target = self.offset + self.scale * amp_action
                if release is None:
                    gap = abs(amp_target - sent)
                    leg_gap, upper_gap = gap[self.leg].max(), gap[self.upper].max()
                    dips.append([speed, omega, leg_gap, upper_gap])
                    good = (
                        support == 3
                        and speed < 0.12
                        and omega < 0.6
                        and tilt < 15
                        and leg_gap < 0.45
                        and upper_gap < 1.5
                    )
                    streak = streak + 1 if good else 0
                    if streak == 5:
                        release, source = k, sent.copy()
                        history.extend([obs.copy()] * 5)
                        result.update(
                            handoff=k,
                            leg_delta_rad=float(leg_gap),
                            upper_delta_rad=float(upper_gap),
                            delta_by_joint_rad=(amp_target - sent).tolist(),
                        )
                    elif k >= landing + BLEND + HOLD_LIMIT:
                        result.update(status="gate_timeout", ended_at=k)
                        break
                if release is not None:
                    target, kp, kd = amp_target, self.akp, self.akd
            for sub in range(self.decimation):
                if release is not None:
                    u = min(1.0, ((k - release) * DT + sub * self.model.opt.timestep) / 0.35)
                    a = u * u * (3 - 2 * u)
                    sent = source + a * (target - source)
                    pk, dk = cfg["kp"] + a * (kp - cfg["kp"]), cfg["kd"] + a * (kd - cfg["kd"])
                else:
                    sent, pk, dk = target, kp, kd
                d.ctrl[self.act] = pk * (sent - d.qpos[self.qa]) - dk * d.qvel[self.va]
                mj.mj_step(self.model, d)
            if release is not None and k >= release + POST:
                last = np.array(rows[-50:])
                ok = (
                    np.all(last[:, 0] == 3)
                    and last[:, 1].max() < 0.12
                    and last[:, 2].max() < 0.6
                    and last[:, 3].max() < 15
                )
                result.update(
                    status="stood" if ok else "unstable",
                    ended_at=k,
                    final_height_m=float(d.qpos[2]),
                    last_second_both=int(sum(last[:, 0] == 3)),
                    last_second_max_xy_m_s=float(last[:, 1].max()),
                    last_second_max_tilt_deg=float(last[:, 3].max()),
                )
                break
        if stop is not None:
            result["loaded_foot_max_horizontal_excursion_m"] = max_loaded_foot_xy.tolist()
        if dips:
            result["hold_min_speed_omega_leg_gap_upper_gap"] = np.min(dips, axis=0).tolist()
        return result, np.array(rows), np.array(commands), np.array(actions)


def select_cases(rows, is_wave, clip, trial):
    phase, speed, omega, tilt, _ = rows.T
    calm = (phase == 3) & (speed < 0.12) & (omega < 0.6) & (tilt < 15)
    cases = []
    if is_wave:
        eligible = np.flatnonzero(
            calm & (np.arange(len(rows)) > 25) & (np.arange(len(rows)) < len(rows) - 32)
        )
        picks = np.random.default_rng(1124).choice(eligible, 2, replace=False)
        return [(int(i), int(i), "upper") for i in sorted(set([100, *picks]))]
    for code, label in ((1, "left"), (2, "right"), (3, "double")):
        for k in range(110, len(rows) - 8):
            if not np.all(phase[k - 2 : k + 2] == code):
                continue
            landing = next(
                (j for j in range(k, min(k + 101, len(rows) - 4)) if np.all(calm[j : j + 5])), None
            )
            if landing is not None:
                candidates = [
                    j for j in range(k, min(k + 101, len(rows) - 4)) if np.all(calm[j : j + 5])
                ]
                best = min(
                    candidates,
                    key=lambda j: np.linalg.norm(clip.q[j, trial.leg] - trial.stand[trial.leg]),
                )
                cases.extend(
                    [
                        (k, k, label + "_whole_body_direct"),
                        (k, landing, label + "_whole_body_after_landing"),
                        (k, k, label + "_upper_direct"),
                        (k, landing, label + "_upper_after_landing"),
                        (k, best, label + "_upper_matched_landing"),
                    ]
                )
                break
        else:
            raise RuntimeError(f"No observed {label} support with a subsequent quiet landing")
    return cases


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    for arg in ["player", "policy", "wave", "dance", "output"]:
        ap.add_argument("--" + arg, type=Path, required=True)
    args = ap.parse_args()
    spec = importlib.util.spec_from_file_location("native_player", args.player)
    player = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(player)
    trial = Trial(player, args.policy)
    out = dict(
        mujoco=mj.__version__,
        policy_sha256=digest(args.policy),
        amp_sha256=digest(trial.amp_path),
        scene_sha256=digest(trial.scene),
        player_sha256=digest(args.player),
        motions={},
    )
    for label, path in (("wave", args.wave), ("wave_on_amp_stance", args.wave), ("dance", args.dance)):
        clip = Clip(path, trial.names)
        if label == "wave_on_amp_stance":
            # Isolate the real upper-body motion on the measured AMP lower-body reference.
            clip.q[:, trial.leg] = trial.stand[trial.leg]
            clip.pos[:] = trial.root_stand[0]
            clip.quat[:] = np.roll(trial.root_stand[1], 1)
        old, _, old_cmd, old_act = trial.run(clip, legacy=True)
        regular, rows, cmd, act = trial.run(clip)
        assert old == regular and np.array_equal(cmd, old_cmd) and np.array_equal(act, old_act)
        record = dict(
            clip=path.name,
            sha256=digest(path),
            baseline=regular,
            compared_ticks=len(cmd),
            clip_frames=len(clip.frames),
            command_max_abs=float(abs(cmd - old_cmd).max()),
            action_max_abs=float(abs(act - old_act).max()),
            cases={},
        )
        out["motions"][label] = record
        for stop, landing, case in select_cases(rows, label != "dance", clip, trial):
            result, actual, cc, aa = trial.run(clip, stop, landing, whole_body="whole_body" in case)
            assert np.array_equal(cc[:stop], cmd[:stop]) and np.array_equal(aa[:stop], act[:stop])
            record["cases"][f"{case}_{stop}"] = result
            print(label, case, stop, landing, result["status"], result.get("handoff"), flush=True)
        args.output.write_text(json.dumps(out, indent=2, allow_nan=False) + "\n")


if __name__ == "__main__":
    main()
