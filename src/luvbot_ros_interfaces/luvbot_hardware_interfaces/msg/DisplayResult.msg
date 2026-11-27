# Business result for a P4 request. session optionally selects the requesting boot_id.
uint32 session
# Reserved.
uint32 sequence
uint32 event_id
# 0 success,1 rejected,2 busy,3 invalid,4 unsupported,5 failed,6 timeout,7 received only
uint16 result
uint16 reason
uint32 revision
