std_msgs/Header header

# Derived from RTPRI getArmState / getUserSafeState.

# mode: worst asserted arm_state condition.
uint8 NORMAL=0
uint8 RECOVERY=1
uint8 COLLISION=2
uint8 SAFE_STOP_2=3
uint8 SAFE_STOP_1=4
uint8 SAFE_TORQUE_OFF=5
uint8 EMERGENCY_STOP=6
uint8 FAULT=7
uint8 mode

# zone: getUserSafeState (speed/separation zone, not a stop).
uint8 ZONE_SAFETY=0
uint8 ZONE_WARNING_2=1
uint8 ZONE_WARNING_1=2
uint8 ZONE_PROTECTION=3
uint8 zone

# Raw getArmState() bitfield.
uint32 arm_state
