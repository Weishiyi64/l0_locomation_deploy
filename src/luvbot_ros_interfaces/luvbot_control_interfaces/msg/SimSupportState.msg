# MuJoCo physics feedback for the simulation-only stop trial, at the odometry stamp.
std_msgs/Header header
float64 left_normal_force_n
float64 right_normal_force_n
float64 horizontal_speed_m_s
bool command_valid
string<=64[<=64] joint_names
float64[<=64] target_positions
