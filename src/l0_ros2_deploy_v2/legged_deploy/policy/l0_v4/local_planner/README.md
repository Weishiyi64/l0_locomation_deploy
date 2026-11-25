# RK3588 planner artifact

`planner_09090.onnx` is the source model copied byte-for-byte from the verified
`compare_source` planner. Generate the deployable model and its required
manifest on an x86_64 Linux host with RKNN-Toolkit2 installed:

```bash
python3 convert_onnx_to_rknn_rk3588.py \
  legged_deploy/policy/l0_v5/local_planner/planner_09090.onnx \
  --verify --samples 20
```

Deploy these three files together:

- `planner_09090.onnx`
- `planner_09090.rknn`
- `planner_09090.rknn.json`

The follow-goal bridge uses `0.35 m/s` and `1.57 rad/s` when perception sends a
zero speed field. Nonzero linear targets are clamped to `[0.3, 0.4] m/s`, and
the issued yaw rate is bounded to `[-1.57, 1.57] rad/s`.

`/navigation_goal` supplies `[x_odom, y_odom, speed, target_yaw, turn_speed,
visible]`; `/goal_pose` accepts the equivalent one-shot `PoseStamped` goal in
`device0/odom`. The bridge uses fresh `device0/odom -> device0/base_link` odometry to
convert the absolute XY goal into the body-relative XY input expected by the
planner and converts the absolute target yaw into a relative heading error.
During Nav, the bridge samples the straight line to the goal
at 1 m, 2 m, and 3 m lookahead. Points beyond the goal repeat the final goal,
matching the model's current-plus-two-future-waypoints contract.

The runtime rejects an unverified manifest, changed hashes, wrong tensor
shapes, a non-RK3588 target, a planner frequency other than 5 Hz, or failed
Nav/Face/Hold output interlocks.
