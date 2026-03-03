int8 mode
float32 q
float32 dq
float32 ddq
float32 tau_est
# Motor temperature (int8): actual value, range -100 to 150.
int8 temperature
# Motor packet loss count (uint32): actual value, range 0 to 9999999999.
uint32 lost
# Motor communication info: array of two uint32s.
#   index 0 = error flags (0-255), index 1 = communication frequency (0-800).
uint32[2] reserve