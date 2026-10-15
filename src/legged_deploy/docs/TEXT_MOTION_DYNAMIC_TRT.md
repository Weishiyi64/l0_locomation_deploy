# Text-to-motion fixed WBC deployment

## Runtime data flow

```text
x86 PC: infer.py -> one complete JSON message -> /text_motion/json
Jetson: C++ JSON worker -> mirrored/resampled/filtered full-body reference
        -> policy[399] + command[240] -> fixed policy.trt -> /rlCmd_imitation
```

The JSON remains trajectory data. It is not converted to TensorRT. The fixed
stg25 policy is converted once on the Jetson and every later motion reuses the
same engine. The callback only queues the complete JSON; parsing and trajectory
preparation run on a background thread and do not block the controller loop.

## Rollback points

- Working legacy implementation tag: `text-motion-python-packager-v1`
- New implementation branch: `feature/text-motion-dynamic-trt`
- Runtime-only fallback without changing Git:

  ```bash
  ros2 launch legged_controllers one_start.launch.py \
    text_motion_runtime:=legacy_packager
  ```

Do not copy an x86 TensorRT engine to Jetson. `policy.onnx` is portable, but
`policy.trt` must be generated on the target Jetson/TensorRT version.

## x86 MuJoCo validation

```bash
cd /home/luvbotic/atom_ros2_deploy_baseline_texttomotion
source /opt/ros/humble/setup.bash
colcon build --packages-select legged_deploy legged_controllers --symlink-install
source install/setup.bash
```

Start the existing MuJoCo launch exactly as before. `one_start.launch.py`
defaults to `text_motion_runtime:=dynamic_trt`; x86 uses the fixed `policy.onnx`
through OpenVINO. Enter AMP/PD using the existing joystick sequence, then from
another terminal publish a motion:

```bash
cd /home/luvbotic/atom_ros2_deploy_baseline_texttomotion
source install/setup.bash
/home/luvbotic/miniconda3/envs/unitree_rl_mjlab/bin/python infer.py \
  "右手腹部摆动两次。" --overwrite
```

Useful checks:

```bash
ros2 topic echo /text_motion/status
ros2 topic echo /text_motion/ready
ros2 topic hz /rlCmd_imitation
```

Expected status contains `dynamic-reference frames` and
`fixed_policy=text_motion_wbc_stg25_dynamic`. No
`text_motion_wbc_packager` process and no per-motion ONNX/TRT build should
appear.

## Jetson deployment

Upload the source tree once, build it natively, then generate the fixed engine:

```bash
# x86 PC
rsync -av --delete-excluded \
  --exclude build --exclude install --exclude log --exclude .git \
  /home/luvbotic/atom_ros2_deploy_baseline_texttomotion/ \
  nvidia@192.168.50.103:~/atom_ros2_deploy_baseline_texttomotion/
```

```bash
# Jetson
cd ~/atom_ros2_deploy_baseline_texttomotion
source /opt/ros/humble/setup.bash
colcon build --packages-select legged_deploy legged_controllers
source install/setup.bash
ros2 run legged_deploy build_dynamic_wbc_trt.sh
```

After `policy.trt` exists, start the real-robot launch with
`text_motion_runtime:=dynamic_trt`. Run `infer.py` on the x86 PC in the same
ROS domain/network; Jetson needs neither PyTorch nor ONNX Runtime. Before an
unsupported, full-amplitude motion, test under a safety gantry with a small
motion and an operator ready to stop PD output.
