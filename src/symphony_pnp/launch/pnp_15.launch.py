"""Gazebo + MoveIt for symphony15. After launch: ros2 run symphony_pnp pnp_15"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    pnp_pkg = FindPackageShare("symphony_pnp")
    gripper = LaunchConfiguration("gripper")
    enable_pnp_hold = LaunchConfiguration("enable_pnp_hold")
    return LaunchDescription([
        DeclareLaunchArgument("gripper", default_value="robotiq_2f"),
        DeclareLaunchArgument("enable_pnp_hold", default_value="true"),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([pnp_pkg, "launch", "pnp.launch.py"])
            ),
            launch_arguments={
                "symphony_type": "symphony15",
                "gripper": gripper,
                "enable_pnp_hold": enable_pnp_hold,
            }.items(),
        ),
    ])
