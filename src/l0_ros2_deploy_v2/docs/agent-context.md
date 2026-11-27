# Agent Context

Read this reference only for tasks involving package ownership, controller
composition, launch wiring, or related architecture. The always-applicable
rules live in `../AGENTS.md`.

## Project Map

This repository is a ROS2/ament workspace slice for legged robot deployment.

- `legged_controllers/`: shared controller library. It owns
  `LeggedController`, state-machine integration, joystick and head control,
  policy mux, safety checks, and command composition.
- `legged_env_mujoco/legged_mujoco/`: MuJoCo-side hardware layer using the
  shared controller.
- `legged_env_mujoco/mujoco/`: MuJoCo simulator launch/runtime package.
- `legged_env_real/legged_real/`: real-robot hardware layer using the shared
  controller.
- `legged_deploy/`: RL controller launch, configuration, and policy assets.
- `robot_interfaces/`: interfaces internal to this deployment repository,
  including `PolicyObservation`, `PolicyCommand`, `SwitchPolicy`,
  `ControlWbcMotion`, `MotionTransaction` and `RequestMotion`.
- `luvbot_hardware_interfaces` (sibling package): shared motor command, feedback,
  enable and calibration types used by deployment, MuJoCo and HPM.
- `luvbot_control_interfaces/` (sibling package): external navigation, voice
  and perception contracts. Deployment-internal interfaces belong to
  `robot_interfaces`, not the shared `luvbot_ros_interfaces` repository.
- `legged_description/`: robot descriptions and display launches.

The v2 workspace is `~/zy_workspace/l0_deploy_v2_ws`, with this repository at
`src/l0_ros2_deploy_v2` and matching `luvbot_ros_interfaces` sources alongside it.
The v1 repository and its dependencies belong in the separate
`~/zy_workspace/l0_deploy_v1_ws` workspace. Each workspace owns its build,
install, and log directories. Source only the selected deployment workspace.
The v2 controller has no eye-animation input or serial transport dependency;
HPM owns the real motor and IMU transport in its own workspace.

## Header Layout

Project headers in `legged_controllers`, `legged_real`, and
`system_command_arbiter` live directly in each package's `include/` directory.
Use the header filename in includes, without repeating the package name.
Controller navigation headers also live in `include/`. The OpenCL header
input template is `legged_controllers/include/PlannerDepthKernel.h.in`;
CMake generates the embedded kernel header under the build directory and
excludes the template from installation. Keep third-party and logical module
subdirectories unchanged.

## Controller Architecture

- The hardware control thread uses a steady-clock periodic grid derived from
  lowlevel_frequency (500 Hz = 2 ms). Each cycle completes read/update/write,
  then sleeps until the first strictly future grid tick; elapsed ticks are
  skipped without catch-up cycles. The unused skip_control_on_overrun switch
  has been removed. Hardware uses SCHED_FIFO priority 97; the shared AMP,
  imitation and WBC inference configuration uses priority 98. Inference cadence
  and late-result validation remain independent of the hardware loop.
- Arm motion is driven through policy commands and procedural walking arm swing.
  Do not restore the old hand/arm YAML script overlay path.
- Fixed head scripts and their YAML assets have been removed. `FaceFollow.cpp`
  owns face-follow target processing and interpolated head output while walking.
- `JointCommandComposer` priority is:
  1. Procedural walking arm swing when enabled and allowed.
  2. Head overlay when auxiliary overlays are allowed.
  3. Policy/default standby command.
  4. Position handoff and emergency overrides.
- Calibration joint-group selection is local to `legged_real/src/HardwareCommand.cpp`.
- Prefer existing helpers such as `ParameterManager`, `ControllerFileUtils`,
  `PolicyMux` and `JointCommandComposer`.
- Orientation safety uses one shared `SafetyChecker`, attached to hardware before
  callbacks start. Real IMU callbacks count accepted frames; the control loop arms
  monitoring only while walking and handles the latched emergency threshold.
  The default is 50 consecutive frames, with normal frames resetting the count
  before the threshold. Duplicate control reads do not advance it.
- Local CSV playback and action-sequence execution have been removed, including
  joystick sequence bindings and the Back-button playback-then-stop action.
  RB manual emergency stop and the emergency-stop topic remain available.
  `JointCommandTransition` still provides imitation/WBC policy handoffs.
- Imitation joystick bindings and their `/shot_id` publisher have been removed.
  `imitation_assets.py` validates the explicit `imitation_policies` launch list
  (default `[]`) for worker preloading, independently of joystick input.
  `startImitationAction()` retains safe policy activation for a future trigger;
  it currently has no callers. Inference, policy mux and AMP return remain.
- WBC uses `PolicyObservation` and `PolicyCommand`, with a dedicated inference worker.
  `LeggedControllerWbc.cpp` stages policy handoffs for the control loop; callbacks
  must not mutate the shared `JointCommandTransition` directly. Face following
  pauses during WBC and policy handoff. See the root README for deployment.
- Real hardware accepts `l0_v3`, `l0_v4` (RS05 arms), and `l0_v5` (MR
  PA35-10-8 arms). The v5 mapping stores the physical HPM bus directly.
  HPM routes LeggedReal commands by the configured zero-based joint index.
  The v5 mechanical calibration and joint limits are inherited from v4 and
  require joint-by-joint hardware verification. Preserve the current mapping
  and hardware enablement. AMP follows the user-facing launch subpolicy.
  Real and MuJoCo default to l0_v5_radar_velocity_tracking; navigation defaults
  to model_1600_dense. Their supplied model/config bytes are unchanged. Current
  enabled upper motors use the new policy's zero default angles and kp=50;
  supplier hardware disabled those motors. Keep the existing motor mapping,
  Action path/success lifecycle, AMP timing and navigation timestamp switches.
  One shared WBC Base has 399/240 observations and 31 resident parameter
  tensors in two slots. Fixed actions use matched LoRA bins and lazy-loaded
  reference RKNNs; text uses UpperBody_Enhanced LoRA and JSON only, including
  initialization. Text root is upright at local [0,0,configured height], yaw
  aligned at start; each leg uses named target angles from text/standing.yaml.
  AMP policy defaults remain intact. Inactive upper joints hold the measured start.
  Text JSON frames already include upper-body entry and return-to-start. JSON
  admission checks both endpoints against the captured upper-body pose within
  0.005 rad and uses the generated frames directly, with configured standing legs.
  It adds no entry/exit pose blends, hold frames or separate leg-transition branch.
  Fixed WBC queries the first reference through ControlWbcMotion.REFERENCE,
  then prepositions arms/head on the control thread while AMP keeps controlling
  the legs (wbc_transition_sec, default 0.35 seconds). Head following pauses on
  reservation; the control thread holds measured head positions until preposition
  takes over. Afterward a fresh START seeds WBC from
  current observations and composed upper targets; no old prepared output is reused.
  Both fixed entry and return switch leg commands directly, smoothing only the
  nine arm/head joints, including gains. Cancelled/failed preposition blends
  upper targets back under AMP before the reservation can be released.
  Text has no preposition stage; both fixed and text switch legs directly and
  blend only the nine upper joints. Text generation waits for head hold to be
  applied and natural arm-swing output to finish, without forcing a swing reset.
  MotionTransaction.STATUS payload_json adds text_capture_stamp_ns (0 until
  ready), bound to the current reservation. The gateway captures only a newer
  observation for that request; waiting shares the existing three-second budget.
  Cancellation, faults, policy takeover and release invalidate this timestamp.
  Head following resumes from measured pose after confirmed AMP return/cleanup;
  held targets never override preposition, WBC tracking or emergency damping.
  Motion admission has no speed-magnitude gate; finite values, orientation,
  command freshness, ownership and safety checks remain. Brain coordinates
  navigation/action exclusion; do not add navigation rejection or velocity locks.
  Preserve the existing 0.15 rad pose-mismatch and 0.005 rad endpoint checks.
  Fixed and text WBC references advance immediately from frame zero, without
  an extra start hold; duration is reference_num_frames * control_dt.
  Policy and motion interfaces belong to this repository's robot_interfaces package;
  REFERENCE returns joint_names and reference_positions in radians.
  Handoffs capture all five command fields from the previous composed hardware
  command on the control thread, including overlays and any unfinished blend;
  callbacks no longer stage a raw policy pose. Arm overlay suppression follows
  the applied source/transition; head following stays paused through reservation
  cleanup. JointCommandComposer validates a complete frame before writing or
  caching it. Handoffs use that cache, without re-reading unchecked hardware
  command buffers. Missing preposition/entry origins reject the action and
  retain AMP; a missing return blend origin permits fresh AMP to take over directly.
  Continuous JSON reference endpoints do not guarantee continuous neural-policy output targets;
  do not bypass text head/arm command blending. The previous text position bypass
  was removed after startup shaking was reported; robot verification is pending.
  Only the control loop starts or applies the transition; the reservation cannot
  be released before retirement. Imitation return keeps its existing behavior.
  Runtime WBC deadline misses discard output without renewing command validity
  or committing action history; the worker retries while its last command is
  still valid. Preparation deadline misses still reject admission. There is
  no early 50 ms return threshold. Once WBC output actually becomes unavailable,
  the control loop requests AMP return independently of worker callbacks while
  holding the last validated frame. Worker failures use the same nonblocking
  return path. The mux freezes
  the accepted WBC seed during return; all inference deadlines and incoming
  command validity checks remain intact. AMP takeover requires a matching
  return ID and unexpired output. The redundant AMP revision counter and
  original-WBC-deadline stop branches were removed. Expired, missing or
  mismatched return history seeds are rejected; AMP initializes from validated
  fresh measured joints instead. A cancelled pending WBC entry keeps AMP;
  there is no separate awaiting-return latch that can block this AMP restart.
  If the selected policy expires or composition fails, the controller reissues
  its last complete validated q/dq/kp/kd/tau frame until fresh output takes over.
  There is no policy hold timeout and no deadline-triggered EmergencyStop;
  inference/input freshness and the original policy valid_until remain intact.
  The cache includes overlays and upper-body interpolation; those outputs freeze
  during the gap. This is target retention, not another balance policy or leg
  blend. Only an absent cache or joint-count mismatch causes NoValidCommand
  emergency damping. Hardware/manual/IMU/orientation interlocks preempt the hold.
  Status keeps emergency.active false during recovery and reports
  policy.valid=false and holding_last_command=true; there are no recovery timer
  or limit fields. Emergency composition clears the cache. Walking stays Walking
  through policy gaps; reset and calibration keep their existing interlocks.
  StateMachine uses atomic state reads and compare/exchange for guarded events;
  unconditional emergency exchange cannot be overwritten by an event that read
  a different state before the emergency. Redundant policy-failure stop reasons,
  timer fields, duplicate state-name branches and repeated transition code were
  removed; there is no additional recovery FSM state.
  The worker observes return intent in PolicyObservation before another
  inference, independently of an execution callback waiting on its mutex.
  Terminal feedback is sent after AMP source takeover (or AMP retention before
  WBC entry); recovered failure includes detail=AMP control restored. The
  gateway preserves detail; worker retirement stays in cleanup after takeover.
  Robot validation must establish recovery latency under full load; keeping
  old position targets indefinitely and an IMU interlock cannot guarantee
  balance or recovery. Hardware transport timeouts are unchanged.
  Text gateway stage changes are logged with the request ID (except repeated
  cleanup_pending), so generation-time symptoms can be separated from execution.
  Generator return hold and AMP handoff remain.
  Base deployment optionally starts one worker and motion gateway.
  sim2real and sim2mujoco expose independent wbc_enabled and txt2motion_enabled
  switches. Either starts one shared backend; one_start creates frontend Nodes
  directly, with no standalone wbc/txt2motion launch wrappers. Both real and simulation entrypoints currently default WBC and text on.
  All tracking assets live in policy/l0_v5/wbc_lora_fp16. Fixed-only reads
  base_manifest.json; text-enabled reads deployment.json in the same bundle,
  whose model_manifest and catalogs refer only to local Base/fixed/text files.
  The old policy/motion_runtime directory and cross-bundle fixed_bundle path
  have been removed. Model/weight/config bytes remain intact; stale conversion-record paths were removed from manifests.
  sim2real exposes no backend, policy-filter or bundle selection switches.
  The text frontend keeps its classification/interpolation models resident.
  RequestMotion Action reserves before generation and rejects concurrent work
  through preparation, tracking, return and acknowledged cleanup. The C++
  MotionTransaction service owns the controller reservation, request epoch
  and AMP-ready checks; gateway heartbeat loss cancels execution without
  releasing unconfirmed cleanup. New online requests must use current epoch
  timestamps; no preemption or queue. The sole public text entrypoint is
  `/algorithm/brain/txt2motion_cmd` (std_msgs/String), with exactly timestamp_ms,
  text and an upstream-provided integer seed in [1, 2^32). It always generates
  and executes; no CLI, offline or generation-only mode remains. Internal
  Action/service interfaces retain their schema and coordinate execution;
  text Action goals require execute=true, empty output_path and seed options.
  The native frontend adds deadline_monotonic_ns to internal options; seed-only
  internal goals remain compatible with a deadline established on admission.
  The gateway saves reference.json in a private legged_txt2motion_* temporary
  directory and removes it after worker retirement and AMP return, before
  clearing busy. File removal failure keeps cleanup_pending and retries.
  Abrupt process termination can leave temporary directories behind.
  Active ownership is in wbc_lora_fp16/deployment.json; text provenance is in
  that bundle's MIGRATION.json. WBC offline conversion retains source hash/pairing
  checks; text conversion no longer computes or compares file hashes and ignores
  legacy SHA256 declarations. All startup/online loaders ignore SHA256 fields,
  including text material/model declarations and JSON Base/adapter/normalization
  bindings. JSON still requires the configured text action. Online WBC/text/depth
  loaders do not read conversion records. Unused
  .rknn.json and text conversion_record.json are removed and no longer generated.
  Keep actual tensor shape/type, parameter buffer bounds, finite-value checks,
  path containment, and all dynamic motion safety checks. Text JSON is parsed
  once in the C++ callback and passed as YAML::Node to WbcJsonReference.
  Conversion checks are defined in the corresponding scripts/convert_*.py,
  matching AMP's self-contained checker layout. The former wbc_lora_assets.py
  was folded into the WBC policy/reference converters. Full-bundle conversion
  reuses reference normalization/checks and rknn_build_config from the reference
  converter. All root AMP/planner/WBC/txt2motion conversion entrypoints live under
  scripts/. Text conversion is split into three independent commands:
  convert_onnx_to_rknn_rk3588_qwen_tokenizer.py and
  convert_onnx_to_rknn_rk3588_inbetweener.py each accept one ONNX and default to a
  same-name RKNN beside it; -o selects another RKNN path. Each checks the native
  tensor ABI and graph before Toolkit2 2.3.2 non-quantized compilation; no manifest,
  other model, NPZ or inference runtime is needed for preflight.
  convert_npz_to_bin_txt2motion.py accepts one NPZ and exports motion_assets.bin
  plus assets.json to materials/ beside it; -o selects a new output directory.
  Material export only needs NumPy. All three support --dry-run without writes or
  RKNN SDK imports, refuse to replace existing outputs, and import no local helper.
  The old combined converter and whole-bundle packaging mode have been removed;
  no script modifies the manifest or copies unrelated resources. ONNX/RKNN
  are paired at the model root. One manifest.json shares tensor/material declarations;
  each model's file names its runtime RKNN and onnx names its source backup.
  The model converters check ONNX interfaces against their native C++ contracts,
  without reading existing RKNN files. No separate source manifest is retained or
  generated. No file hash fields are required. materials/ stores data; the model directory carries no runtime copy;
  there are no compiled/ or cpp/ directories. ONNX/NPZ are retained for rebuilding.
  The old baseline compile.sh, conversion
  entrypoint, deploy_rknn Python package and export_cpp_assets.py have been removed.
  The unused source record has been removed and is no longer generated.
  Text manifests/material metadata carry no SHA256 declarations.
  Model/material binary bytes and vocabulary token content remain intact.
  motion_runtime is a C++14 executable with the same backend/wbc/txt2motion roles
  in separate processes. Only the text role dlopens the shared RKNN C API runtime;
  both text models remain resident on NPU_CORE_0 and are warmed before service
  readiness. Host composition uses native CPU code, without Python/Lite2; x86 uses the OpenVINO CPU backend described below. std::mt19937 may choose different materials from NumPy.
  Text reception through complete JSON validation/serialization/saving shares
  one three-second CLOCK_MONOTONIC deadline; no one-second/P95 target or resource
  benchmarking is required. RKNN receives the remaining run timeout, and cleanup
  confirms actual inference retirement before reuse; uncertain SDK state stays
  cleanup_pending. The three-second deadline does not bound recovery or playback.
  Models/vocabulary and materials/ binary data live in
  legged_deploy/policy/l0_v5/txt2motion/, alongside wbc_lora_fp16, and are installed
  once by legged_deploy's recursive policy/ installation. Launch and the native
  frontend default model_root use that installed directory. Text uses the same
  runtime as other RKNN modules: legged_deploy/third_party/rknn_runtime/lib/librknnrt.so
  by default, installed by legged_deploy to lib/ (or the RKNN_LIBRARY selected at build).
  The text role resolves legged_deploy's installation prefix through ament and
  dlopens that lib/librknnrt.so, without a model-local or environment override.
  Conversion output carries no runtime copy. No WBC Base/LoRA is duplicated. The old
  texttomotion/models and baseline directories have been removed. Original ONNX,
  NPZ, vocabulary and source declarations now live beside the RKNN files; duplicate
  vocabulary/NPZ copies were collapsed. RK3588 online code does not open ONNX/NPZ; x86 reads ONNX directly.
  The unused models/deps Python SDK, compiled/deploy_rknn
  host copy and obsolete models/DELIVERY.json have been removed. The original NPZ
  and model-directory ONNX/manifest/vocabulary/NPZ files remain for rebuilding.
  Material export preserves the same binary data and requires a new output directory.
  Updated material dimensions, A-pose, vocabulary and labels must remain consistent
  with the runtime manifest; online validation still checks them.
  Online material declarations,
  dimensions and finite values are checked, without runtime hashing. The strict
  JSON implementation uses nlohmann_json; launch metadata still uses Python/yaml.
  Launch metadata parsing lives in legged_controllers/launch/motion_assets.py beside
  imitation_assets.py. Their launch entrypoints are load_motion_assets() and
  load_imitation_assets(). one_start imports both from its own launch directory, which
  is installed by the existing directory rule; there is no texttomotion share path.
  See ../legged_deploy/policy/l0_v5/txt2motion/README.md for models/conversion and
  ../README.md for launch. This reorganization was statically checked on macOS only.
- BLE provisioning's sudo setup authorizes only the resolved luvbot_provd path,
  without a SHA256 command digest. Re-run configure_provisioning_sudo.sh on the
  robot to replace an existing digest rule; no sudoers changes are applied locally.

## Launch Flow

- Simulation entry point:
  `ros2 launch legged_controllers sim2mujoco.launch.py`
- Real-robot entry point:
  `ros2 launch legged_controllers sim2real.launch.py`
- Both entry points call a `sequential_launch_*.sh` script, which starts the
  environment bridge and `one_start.launch.py`.
  Both scripts source `launch_processes.bash` for the same process-group cleanup;
  install this helper beside the entrypoints in `lib/legged_controllers/` as well
  as in the shared scripts directory.
- Simulation connects `topic_hardware_adapter` directly to MuJoCo with
  `luvbot_hardware_interfaces` motor command/enable/state topics, without an arbiter node.
  Its sequential script starts MuJoCo first and waits up to 60 seconds for a
  motor-state message before starting the adapter, so model/window loading does
  not consume the unchanged 20-second hardware enable deadline. It then reuses
  `check_hardware_ready.py false true` before starting policy nodes; failures
  clean up all managed process groups. Cleanup stops the adapter before MuJoCo.
  MuJoCo uses mapping indices and enabled entries, with latched invalid feedback
  after its command watchdog expires; explicit enable is required for recovery.
  The `system_command_arbiter` mapping library/YAML and its interface dependency
  remain shared. Virtual calibration services are not implemented.
  MuJoCo binds mapping joints by name to actuator/qpos/dof addresses, rather than
  assuming MJCF order matches mapping order (v5 differs). Initial load and reload
  validate unique scalar joints and a leading free base; reload clears commands
  and disables motors, invalidating feedback for previously enabled motors.
  The simulation adapter retains `isSim=true`: fixed standby gains, no orientation
  monitoring and no calibration entry. Walking control and motor mapping are shared.
  Compare runs with matched robot, policy and feature settings; simulation and
  real robot/policy defaults differ.
- Real startup runs `check_hardware_ready.py` before policy workers. One monotonic
  20-second deadline covers motor readiness and enabled IMU/battery topics.
  The bridge control loop already sends zero-gain MIT during enable and feedback
  initialization. Enable has no ACK; `motors_ready` requires post-enable normal
  feedback for every enabled motor, with no feedback fault. Unconfirmed joint
  positions/turns remain eligible for calibration; powered commands retain the
  existing turn-confirmation guard. A missing/failed checker aborts startup.
- `one_start.launch.py` starts RViz, joystick, RL-controller support, and
  configured imitation/WBC motion workers. Head/topic parameters flow through the
  simulation or real environment launch.
- With radar enabled, `one_start.launch.py` includes `local_planner.launch.py`,
  which includes the sibling Odin driver's `driver.launch.py` once. Driver
  stream/image/TF defaults belong to the Odin repository. Deployment enables
  `post_process` with `undistort_mapping_only` so `camera0/undistort` follows
  the driver's confirmed SLAM/ReMap mode. Standalone driver users need no deployment packages.
  Navigation always consumes fused depth; there is no depth-input enable switch.
  The deployment-owned raw-depth publisher, mono8 preview node, preview launch
  arguments and build target have been removed. Raw depth remains a local
  intermediate in the fusion calculation. The nav_check_sensor_timestamps and
  nav_reference_receive_timeout switches retain their behavior and defaults.
  The driver defaults to Normal. Radar operators switch to SLAM through the
  existing device_mode service after startup; deployment does not switch modes.

When changing a launch argument, inspect and update the consumers across this
chain as needed:

- `legged_controllers/launch/sim2mujoco.launch.py`
- `legged_controllers/launch/sim2real.launch.py`
- `legged_controllers/scripts/sequential_launch_sim.sh`
- `legged_controllers/scripts/sequential_launch_real.sh`
- `legged_env_mujoco/legged_mujoco/launch/legged_mujoco_sim.launch.py`
- `legged_env_real/legged_real/launch/legged_real.launch.py`
- `legged_controllers/launch/one_start.launch.py` when it consumes the argument

## x86 ONNX backend update

TensorRT/Orin inference code and CUDA linkage have been removed. InferenceModel
selects native RknnModel on RK3588 and OnnxModel (OpenVINO CPU) on x86. AMP,
imitation, WBC Base/reference and local_planner share that API. Text uses
TextMotionInference with RKNN or OpenVINO sessions; legged_deploy exports the
selected backend to legged_controllers through inference_backend.cmake.
local_planner replaces the old local_planner_rknn executable on both platforms.
Its CPU loader extracts actions/next_hidden_state from the delivered ONNX and
reshapes observations/hidden_state to batch one, retaining the C++ velocity map.
The text CPU session retires synchronously before checking cancellation/deadlines.
The earlier RKNN-only/no-CPU-fallback descriptions above apply only to the RK3588
backend. The current sim2real/sim2mujoco entrypoints enable WBC and text by default.
The WBC bundle now includes the matching base_lora.onnx and 63 fixed reference
ONNX files from wbc_lora_20261028_105159_584983. Their manifest hashes, Base/text
fingerprints, tensor contracts and adapter weights match the active deployment.
17 same-name actions have newer incompatible reference/adapter versions and six
additional actions are absent from that source; their ONNXs remain missing.
See policy/l0_v5/wbc_lora_fp16/ONNX_IMPORT.md for the full action lists.
Existing RKNNs, LoRA weights, configuration and catalogs were preserved. Fixed
references load lazily; missing models must explicitly fail with their paths.
Do not silently substitute old references or disable actions. For a focused
x86 check use wbc_enabled:=true txt2motion_enabled:=false
wbc_policies:="['L0_Hugme']"; for AMP-only simulation disable WBC/text as documented.
