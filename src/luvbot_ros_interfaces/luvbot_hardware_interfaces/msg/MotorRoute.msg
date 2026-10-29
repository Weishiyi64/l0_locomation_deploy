# Stable actuator address. bus_index=0 is valid; supported buses are platform-defined.
uint8 MOTOR_TYPE_UNKNOWN=0
uint8 MOTOR_TYPE_DM=1
uint8 MOTOR_TYPE_RS=2
uint8 MOTOR_TYPE_ENC=3
uint8 MOTOR_TYPE_MTE=4
uint8 MOTOR_TYPE_LS=5

uint8 motor_type
uint8 bus_index
uint32 motor_id
