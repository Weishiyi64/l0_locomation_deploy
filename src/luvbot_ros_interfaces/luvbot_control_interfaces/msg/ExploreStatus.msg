uint8 IDLE=0
uint8 RUNNING=1
uint8 FINISHED=2
uint8 FAILED=3
# Producer name: Mapping or Segmentation (case-sensitive). Single active task, no restart recovery.
string node_name
uint16 cost_time # Task duration in seconds, saturated at 65535 (never wrap).
uint8 state
uint16 error_code # 0 = no error
string error_msg
