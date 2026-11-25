uint8 WAITING=0
uint8 POSITION=1
uint8 HEADING=2
uint8 DONE=3

std_msgs/Header header
builtin_interfaces/Time valid_until
uint32 task_id
uint8 phase
float32 local_x
float32 local_y
float32 linear_speed
float32 heading_error
float32 angular_speed
