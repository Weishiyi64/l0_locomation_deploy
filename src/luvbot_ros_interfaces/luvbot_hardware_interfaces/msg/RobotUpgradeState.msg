uint32 id
# 1 system,2 HPM,3 P4,4 assets,5 bundle
uint8 target
# 0 idle,1 preparing,2 transferring,3 verifying,4 rebooting,5 self-test,6 completed,7 failed
uint8 phase
# 0..100 or 255 unknown
uint8 progress
# 0 pending,1 success,2 failed,3 cancelled
uint8 result
