std_msgs/Header header

# Last Tool IO Modbus transaction echo.
uint16 command
uint16 address
uint16 number
uint16[8] value

# getToolIoStatus bit 3 (TOOL_STATUS_Modbus).
bool modbus_ok
uint16 io_status
