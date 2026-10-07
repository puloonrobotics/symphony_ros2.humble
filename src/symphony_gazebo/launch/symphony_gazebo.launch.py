from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler, OpaqueFunction, SetEnvironmentVariable, ExecuteProcess
from launch.event_handlers import OnProcessExit
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
import os

def _gz_plugin_path():
    libs = []
    for prefix in os.environ.get('AMENT_PREFIX_PATH', '').split(os.pathsep):
        lib = os.path.join(prefix, 'lib')
        if prefix and os.path.isdir(lib) and lib not in libs:
            libs.append(lib)
    current = os.environ.get('GZ_SIM_SYSTEM_PLUGIN_PATH', '')
    for entry in current.split(os.pathsep):
        if entry and entry not in libs:
            libs.append(entry)
    for arch in ("aarch64", "x86_64"):
        ign_plugins = f"/usr/lib/{arch}-linux-gnu/ign-gazebo-6/plugins"
        if os.path.isdir(ign_plugins) and ign_plugins not in libs:
            libs.append(ign_plugins)
    return os.pathsep.join(libs)

def launch_setup(context, *args, **kwargs):
    if not os.environ.get('GZ_IP'):
        os.environ['GZ_IP'] = '127.0.0.1'
    description_pkg = FindPackageShare('symphony_description')
    gazebo_pkg = FindPackageShare('symphony_gazebo')

    name = LaunchConfiguration("name")
    symphony_type = LaunchConfiguration("symphony_type")
    gripper = LaunchConfiguration("gripper")
    launch_rviz = LaunchConfiguration("launch_rviz")
    prefix = LaunchConfiguration("prefix")
    gazebo_on = LaunchConfiguration("gazebo_on")

    os.environ['GZ_SIM_SYSTEM_PLUGIN_PATH'] = _gz_plugin_path()

    gripper_str = gripper.perform(context).strip().replace('"', '').replace("'", "")

    if (gripper_str == "robotiq_2f"):
        controllers_yaml = PathJoinSubstitution(
            [gazebo_pkg, "controller", "symphony_controller_gripper.yaml"]
        ).perform(context)
    else :
        controllers_yaml = PathJoinSubstitution(
            [gazebo_pkg, "controller", "symphony_controller.yaml"]
        ).perform(context)

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ", PathJoinSubstitution([description_pkg, "urdf", "symphony.urdf.xacro"]),
            " ", "name:=", name,
            " ", "symphony_type:=", symphony_type,
            " ", "gripper:=", gripper,
            " ", "prefix:=", prefix,
            " ", "sim_gazebo:=true",
            " ", "use_fake_hardware:=false",
            " ", "simulation_controllers:=", controllers_yaml,
        ]
    )
    robot_description = {
        "robot_description": ParameterValue(robot_description_content, value_type=str)
    }

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{"use_sim_time": True}, robot_description],
    )

    rviz_config_file = PathJoinSubstitution(
        [description_pkg, "rviz_config", "symphony.rviz"]
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        condition=IfCondition(launch_rviz),
    )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            robot_description,
            controllers_yaml],
        output="both",
    )

    joint_state_broadcaster_spawner = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'joint_state_broadcaster'],
        output='screen'
    )

    joint_trajectory_controller_spawner = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'joint_trajectory_controller'],
        output='screen'
    )

    gripper_controller_spawner = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'gripper_controller'],
        output='screen'
    )

    if gazebo_on.perform(context) == "true" :
        gazebo = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [FindPackageShare("ros_gz_sim"),
                '/launch', '/gz_sim.launch.py']),
            launch_arguments=[('gz_args', [' -r -v 4 empty.sdf'])]
            )
    else:
        gazebo = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [FindPackageShare("ros_gz_sim"),
                '/launch', '/gz_sim.launch.py']),
            launch_arguments=[('gz_args', [' -r -v 4 empty.sdf -s'])]
            )

    gazebo_spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        name="spawn_symphony",
        arguments=['-string', robot_description_content,
                   '-name', 'symphony',
                   '-allow_renaming', 'false'],
        output="screen",
    )

    delay_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=gazebo_spawn_robot,
            on_exit=[joint_state_broadcaster_spawner],
        )
    )

    if (gripper_str == "robotiq_2f"):
        delay_robot_controller_spawner = RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=joint_state_broadcaster_spawner,
                on_exit=[joint_trajectory_controller_spawner, gripper_controller_spawner],
            )
        )
    else:
        delay_robot_controller_spawner = RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=joint_state_broadcaster_spawner,
                on_exit=[joint_trajectory_controller_spawner],
            )
        )

    delay_rviz_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_trajectory_controller_spawner,
            on_exit=[rviz_node],
        ),
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock'],
        output='screen'
    )

    nodes_to_start = [
        SetEnvironmentVariable('GZ_SIM_SYSTEM_PLUGIN_PATH', _gz_plugin_path()),
        gazebo,
        gazebo_spawn_robot,
        robot_state_publisher_node,
        delay_joint_state_broadcaster_spawner,
        delay_robot_controller_spawner,
        delay_rviz_spawner,
        bridge,
    ]

    return nodes_to_start

def generate_launch_description():
    declared_arguments = []
    gazebo_verbose = SetEnvironmentVariable('GAZEBO_VERBOSE', '1')

    declared_arguments.append(DeclareLaunchArgument("name", default_value="symphony"))
    declared_arguments.append(DeclareLaunchArgument(
        "symphony_type", 
        default_value="symphony5", 
        choices=["symphony5", "symphony10", "symphony15", "symphony20", "symphony40"]
    ))
    declared_arguments.append(DeclareLaunchArgument("prefix", default_value='""'))
    declared_arguments.append(DeclareLaunchArgument("gripper", default_value='""', choices=["robotiq_2f", '""', 'none']))
    declared_arguments.append(DeclareLaunchArgument("launch_rviz", default_value="false"))
    declared_arguments.append(DeclareLaunchArgument("gazebo_on", default_value="true"))

    return LaunchDescription(declared_arguments + [gazebo_verbose, OpaqueFunction(function=launch_setup)])