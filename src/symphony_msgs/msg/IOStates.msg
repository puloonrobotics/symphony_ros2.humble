std_msgs/Header header

# Cabinet/controller IO. Bitmasks: pin 1 = bit 0. Tool flange: ToolIOStates.
uint32 safety_digital_inputs
uint32 safety_digital_outputs
float64[2] safety_analog_inputs
float64[2] safety_analog_outputs

# Safety IO expansion
uint32 safety_digital_inputs_ex1
uint32 safety_digital_outputs_ex1
float64[4] safety_analog_inputs_ex1
float64[4] safety_analog_outputs_ex1
uint32 safety_digital_inputs_ex2
uint32 safety_digital_outputs_ex2
float64[4] safety_analog_inputs_ex2
float64[4] safety_analog_outputs_ex2
