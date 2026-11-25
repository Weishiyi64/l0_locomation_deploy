# Raw motor-space feedback. Units: rad, rad/s, Nm and degrees Celsius.
MotorRoute route

uint16 ERROR_NONE=0
uint16 ERROR_COMMUNICATION=1
uint16 ERROR_OVERTEMPERATURE=2
uint16 ERROR_OVERCURRENT=3
uint16 ERROR_ENCODER=4
uint16 ERROR_DRIVER_FAULT=5
uint16 ERROR_UNKNOWN=65535

uint8 OPERATING_MODE_DISABLED=0
uint8 OPERATING_MODE_POSITION=1
uint8 OPERATING_MODE_VELOCITY=2
uint8 OPERATING_MODE_TORQUE=3
uint8 OPERATING_MODE_HYBRID=4
uint8 OPERATING_MODE_FAULT=255

bool valid
uint16 error_code
uint8 operating_mode
float32 q
float32 dq
float32 tau
float32 mos_temperature
float32 rotor_temperature
