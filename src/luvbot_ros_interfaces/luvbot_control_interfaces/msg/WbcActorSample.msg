# Time of the exact PolicyObservation consumed in this inference.
std_msgs/Header header
string motion_id
uint64 handoff_id
uint64 sequence
bool preparation
bool command_published
int32 reference_frame
float64[3] reference_root_position
float64[4] reference_root_quat_wxyz
float32[] reference_joint_position
# Unmodified RKNN inputs in network order.
float32[] policy
float32[] command
