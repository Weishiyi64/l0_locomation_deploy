# Instantaneous joint position targets. Units: radians.
std_msgs/Header header
uint64 sequence_id
string<=32 source_id
builtin_interfaces/Time valid_until
string<=64[<=64] joint_names
float64[<=64] target_positions
# AMP commands echo the WBC execution whose return history was initialized.
string<=32 amp_return_id
