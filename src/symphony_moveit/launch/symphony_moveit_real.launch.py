from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, OpaqueFunction, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def launch_setup(context, *args, **kwargs):
    driver_pkg = FindPackageShare("symphony_driver")
    moveit_config_package = FindPackageShare("symphony_moveit")

    name = LaunchConfiguration("name")
    symphony_type = LaunchConfiguration("symphony_type")
    prefix = LaunchConfiguration("prefix")
    gripper = LaunchConfiguration("gripper")
    robot_ip = LaunchConfiguration("robot_ip")
    use_fake_hardware = LaunchConfiguration("use_fake_hardware")
    start_joint_trajectory = LaunchConfiguration("start_joint_trajectory")
    launch_rviz_moveit = LaunchConfiguration("launch_rviz_moveit")

    delay_s = float(context.perform_substitution(LaunchConfiguration("moveit_start_delay")))

    symphony_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([driver_pkg, "launch", "symphony_bringup.launch.py"])
        ),
        launch_arguments={
            "name": name,
            "symphony_type": symphony_type,
            "prefix": prefix,
            "gripper": gripper,
            "robot_ip": robot_ip,
            "use_fake_hardware": use_fake_hardware,
            "start_joint_trajectory": start_joint_trajectory,
        }.items(),
    )

    symphony_moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([moveit_config_package, "launch", "moveit.launch.py"])
        ),
        launch_arguments={
            "name": name,
            "symphony_type": symphony_type,
            "prefix": prefix,
            "gripper": gripper,
            "use_sim_time": "false",
            "launch_rviz_moveit": launch_rviz_moveit,
        }.items(),
    )

    return [
        LogInfo(msg="symphony_moveit_real: starting bringup (robot driver + JTC)."),
        LogInfo(
            msg=(
                f"symphony_moveit_real: MoveIt + RViz will start in {delay_s:.1f} s "
                "(wait for controllers before closing this terminal)."
            )
        ),
        symphony_bringup_launch,
        TimerAction(
            period=delay_s,
            actions=[
                LogInfo(msg="symphony_moveit_real: starting MoveIt move_group + RViz."),
                symphony_moveit_launch,
            ],
        ),
    ]


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument("name", default_value="symphony"),
        DeclareLaunchArgument(
            "symphony_type",
            default_value="symphony5",
            description="Type of symphony robot.",
            choices=["symphony5", "symphony10", "symphony15", "symphony20", "symphony40"],
        ),
        DeclareLaunchArgument(
            "prefix",
            default_value='""',
            description="Joint name prefix for multi-robot setups.",
        ),
        DeclareLaunchArgument(
            "gripper",
            default_value="none",
            description="Optional gripper model.",
            choices=["robotiq_2f", '""', "none"],
        ),
        DeclareLaunchArgument(
            "robot_ip",
            default_value="192.168.0.234",
            description="Robot controller IP address.",
        ),
        DeclareLaunchArgument(
            "use_fake_hardware",
            default_value="false",
            description="Use mock hardware instead of the real robot.",
        ),
        DeclareLaunchArgument(
            "start_joint_trajectory",
            default_value="true",
            description="Spawn joint_trajectory_controller active (required for MoveIt Execute).",
        ),
        DeclareLaunchArgument(
            "moveit_start_delay",
            default_value="12.0",
            description="Seconds to wait after bringup before starting MoveIt.",
        ),
        DeclareLaunchArgument(
            "launch_rviz_moveit",
            default_value="true",
            description="Launch RViz with MoveIt Motion Planning panel.",
        ),
    ]

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
