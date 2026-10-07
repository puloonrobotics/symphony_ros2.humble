from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg = FindPackageShare("symphony_driver")
    return LaunchDescription([
        DeclareLaunchArgument("robot_ip", default_value="192.168.0.234"),
        DeclareLaunchArgument("symphony_type", default_value="symphony5"),
        DeclareLaunchArgument(
            "mode",
            default_value="relative",
            description="relative (delta mm) or absolute (current TCP + delta)",
        ),
        DeclareLaunchArgument("dx_mm", default_value="100.0"),
        DeclareLaunchArgument("dy_mm", default_value="-50.0"),
        DeclareLaunchArgument("dz_mm", default_value="80.0"),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([pkg, "launch", "symphony_bringup.launch.py"])
            ),
            launch_arguments={
                "robot_ip": LaunchConfiguration("robot_ip"),
                "symphony_type": LaunchConfiguration("symphony_type"),
                "use_fake_hardware": "false",
                "start_joint_trajectory": "false",
            }.items(),
        ),
        TimerAction(
            period=12.0,
            actions=[
                Node(
                    package="symphony_driver",
                    executable="movel_example.py",
                    output="screen",
                    parameters=[{
                        "mode": LaunchConfiguration("mode"),
                        "dx_mm": ParameterValue(
                            LaunchConfiguration("dx_mm"), value_type=float),
                        "dy_mm": ParameterValue(
                            LaunchConfiguration("dy_mm"), value_type=float),
                        "dz_mm": ParameterValue(
                            LaunchConfiguration("dz_mm"), value_type=float),
                    }],
                ),
            ],
        ),
    ])
