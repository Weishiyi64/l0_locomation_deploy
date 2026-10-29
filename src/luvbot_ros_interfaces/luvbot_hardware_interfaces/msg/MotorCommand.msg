uint8 CONTROL_MODE_UNKNOWN=0
uint8 CONTROL_MODE_POSITION=1
uint8 CONTROL_MODE_VELOCITY=2
uint8 CONTROL_MODE_TORQUE=3
uint8 CONTROL_MODE_HYBRID=4

builtin_interfaces/Time stamp
uint64 sequence_id
string<=32 source_id
builtin_interfaces/Time valid_until
uint8 control_mode
ActuatorMotorCommand[<=64] commands
