# Actual values must be supplied by firmware business code; never fabricated by transport.
uint32 boot_id
uint32 revision
# Diagnostic source: 0 panel, 1 backlight, 2 touch.
uint8 target
# 0 unknown,1 ready,2 fault,3 disabled
uint8 status
uint16 error
# Backlight only. Driver status does not prove optical output.
uint8 brightness
