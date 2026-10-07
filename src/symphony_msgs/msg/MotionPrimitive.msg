# One robot-interpolated motion. Same layout as control_msgs/MotionPrimitive.

int8 UNKNOWN=-1
int8 LINEAR_JOINT=0
int8 LINEAR_CARTESIAN=50
int8 CIRCULAR_CARTESIAN=51

int8 type -1
float64 blend_radius
MotionArgument[] additional_arguments
geometry_msgs/PoseStamped[] poses
float64[] joint_positions
