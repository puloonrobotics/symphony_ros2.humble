std_msgs/Header header

# getArmErrorList history since clear_fault (oldest first), not live state.
# Max 10; see SafetyStatus for current condition.
uint32[] codes

# Most recent code, or 0 if empty.
uint32 latest

# Vendor codes raw. Bit 31 set => ERROR/FAULT; clear => WARNING.
uint32 ERROR_CODE_FLAG=2147483648
