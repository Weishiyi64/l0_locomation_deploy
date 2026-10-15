bool rl_inference_signal

float64[3] body_xpos
float64[4] body_xquat
IMU imu
MotorState[] motor_state
geometry_msgs/Twist cmd_vel
float32 behavior_ud
float32 behavior_lr

#time shifted_time
builtin_interfaces/Time shifted_time
