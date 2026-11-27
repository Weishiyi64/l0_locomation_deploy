# 1 body,2 dock; command describes an output, not proof of physical connectivity.
uint8 target
# S-03 (3): after task success, steady green at 50% for 2000 ms.
# S-06 (6): after valid battery percentage stays <=20 for 10 s, slow orange flash at 60%.
# S-06 timing: period_ms=1000, window_ms=2000, repeat_ms=30000, persistent until superseded.
# The robot business publisher evaluates triggers, priority and when to clear a scene.
uint8 state_id
# 0 none,1 red,2 orange,3 yellow,4 green,5 blue,6 purple
uint8 color
# 0 off,1 steady,2 breathing,3 flashing
uint8 mode
uint8 brightness
uint16 period_ms
uint16 window_ms
uint16 repeat_ms
uint32 activation_id
# 4294967295 continuous; 0 expired
uint32 remaining_ms
uint32 phase_elapsed_ms
