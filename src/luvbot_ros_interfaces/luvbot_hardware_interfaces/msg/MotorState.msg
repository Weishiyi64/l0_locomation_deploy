uint8 OPERATING_MODE_UNKNOWN=255

luvbot_hardware_interfaces/MotorRoute route
bool valid
uint16 error_code
uint8 operating_mode
float64 q
float64 dq
float64 tau
float64 mos_temperature
float64 rotor_temperature

# Drive-reported output status for this feedback frame.
uint8 ENABLE_UNKNOWN=0
uint8 ENABLE_DISABLED=1
uint8 ENABLE_ENABLED=2
uint8 enable_state
