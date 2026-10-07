cmake_minimum_required(VERSION 3.10)
project(symphony_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(action_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/IOStates.msg"
  "msg/ToolIOStates.msg"
  "msg/RobotError.msg"
  "msg/SafetyStatus.msg"
  "msg/ToolModbusStatus.msg"
  "msg/MotionArgument.msg"
  "msg/MotionPrimitive.msg"
  "msg/MotionPrimitiveSequence.msg"
  "action/ExecuteMotionPrimitiveSequence.action"
  "srv/SetSpeed.srv"
  "srv/SetRtStream.srv"
  "srv/ClearFault.srv"
  "srv/SetMotionOptions.srv"
  "srv/SetDigitalOutput.srv"
  "srv/SetAnalogOutput.srv"
  "srv/SetToolIO.srv"
  "srv/SetToolInfo.srv"
  "srv/SetGravity.srv"
  "srv/StartForceControl.srv"
  "srv/SetForceParameters.srv"
  "srv/SetToolAnalogMode.srv"
  "srv/OpenToolModbus.srv"
  "srv/ToolModbusRead.srv"
  "srv/ToolModbusWrite.srv"
  DEPENDENCIES action_msgs geometry_msgs std_msgs
)

install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

ament_export_include_directories("include/${PROJECT_NAME}")
ament_export_dependencies(rosidl_default_runtime action_msgs geometry_msgs std_msgs)
ament_package()
