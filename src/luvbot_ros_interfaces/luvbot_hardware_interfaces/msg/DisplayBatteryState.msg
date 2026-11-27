# Explicit display input. 255 means unknown percentage; charge: 0 off, 1 charging,
# 2 full, 3 paused, 4 fault. Flags: bit0 low, bit1 critical, bit2 hot, bit3 battery fault.
uint8 percent
uint8 charge
uint16 flags
