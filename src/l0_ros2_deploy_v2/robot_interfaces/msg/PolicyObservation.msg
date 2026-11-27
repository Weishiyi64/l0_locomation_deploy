# Controller-generated, synchronized policy input snapshot.
std_msgs/Header header
builtin_interfaces/Duration motion_time
bool inference_enabled
geometry_msgs/Pose base_pose
sensor_msgs/Imu imu
sensor_msgs/JointState joints
geometry_msgs/Twist velocity_command
float32 behavior_up_down
float32 behavior_left_right
# Last validated policy target, before overlays and handoff blending.
uint8 SOURCE_NONE=0
uint8 SOURCE_AMP=1
uint8 SOURCE_IMITATION=2
uint8 SOURCE_WBC=3
uint8 source_policy
PolicyCommand source_command
# Nonempty during/after a seeded WBC-to-AMP return; cleared on the next WBC start or abort.
string<=32 amp_return_id
