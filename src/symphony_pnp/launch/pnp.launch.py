from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, RegisterEventHandler, ExecuteProcess, SetEnvironmentVariable
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch.conditions import IfCondition
import os
import xacro

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

def spawn_entity(context):
    pnp_package = FindPackageShare('symphony_pnp')
    models = {
        "box1": "box.xacro", "box2": "box2.xacro", "box3": "box3.xacro",
        "box4": "box4.xacro", "box5": "box5.xacro", "box6": "box6.xacro",
        "cylinder1": "cylinder1.xacro", "triangle": "triangle.xacro",
    }

    spawn_nodes = []
    for i, (model_name, xacro_file) in enumerate(models.items()):
        xacro_path = os.path.join(pnp_package.perform(context), 'models', xacro_file)
        robot_description = xacro.process_file(xacro_path).toxml()

        spawn_entity = Node(
            package="ros_gz_sim",
            executable="create",
            arguments=[
                "-name", model_name,
                '-string', robot_description,
                "-x", "0.2", "-y", "0.0", "-z", "0.0",
            ],
            output="screen",
        )
        spawn_nodes.append(spawn_entity)

    xacro_path = os.path.join(pnp_package.perform(context), 'models', "case.xacro")
    robot_description = xacro.process_file(xacro_path).toxml()

    spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-name", "case",
            '-string', robot_description,
            "-x", "0.0", "-y", "0.2", "-z", "0.0",
        ],
        output="screen",
    )
    spawn_nodes.append(spawn_entity)

    return spawn_nodes

def launch_setup(context, *args, **kwargs):
    gazebo_package = FindPackageShare('symphony_gazebo')
    moveit_config_package = FindPackageShare('symphony_moveit')

    name = LaunchConfiguration("name")
    symphony_type = LaunchConfiguration("symphony_type")
    gripper = LaunchConfiguration("gripper")
    prefix = LaunchConfiguration("prefix")
    enable_pnp_hold = LaunchConfiguration("enable_pnp_hold")

    description_pkg = FindPackageShare('symphony_description')
    gazebo_pkg = FindPackageShare('symphony_gazebo')
    launch_rviz = LaunchConfiguration("launch_rviz")
    gazebo_on = LaunchConfiguration("gazebo_on")

    os.environ['GZ_SIM_SYSTEM_PLUGIN_PATH'] = _gz_plugin_path()
    if not os.environ.get('GZ_IP'):
        os.environ['GZ_IP'] = '127.0.0.1'

    gripper_str = gripper.perform(context).strip().replace('"', '').replace("'", "")

    if gripper_str == "robotiq_2f":
        controllers_yaml = PathJoinSubstitution(
            [gazebo_pkg, "controller", "symphony_controller_gripper.yaml"]
        ).perform(context)
    else:
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
            " ", "enable_pnp_hold:=", enable_pnp_hold,
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

    if gripper_str == "robotiq_2f":
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

    spawn_entities = spawn_entity(context)

    symphony_gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [gazebo_package, "/launch", "/symphony_gazebo.launch.py"]
        ),
        launch_arguments={
            "name": name,
            "symphony_type": symphony_type,
            "prefix": prefix,
            "launch_rviz": "false",
            'gripper' : gripper,
        }.items(),
    )

    symphony_moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [moveit_config_package, "/launch", "/moveit.launch.py"]
        ),
        launch_arguments={
            "name": name,
            "symphony_type": symphony_type,
            "prefix": prefix,
            "use_sim_time": "true",
            "launch_rviz_moveit": "false",
            'gripper' : gripper,
        }.items(),
    )

    delay_entities_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=gazebo_spawn_robot,
            on_exit=[*spawn_entities]
        )
    )

    nodes_to_launch = [
        SetEnvironmentVariable('GZ_SIM_SYSTEM_PLUGIN_PATH', _gz_plugin_path()),
        gazebo,
        gazebo_spawn_robot,
        robot_state_publisher_node,
        delay_joint_state_broadcaster_spawner,
        delay_robot_controller_spawner,
        delay_rviz_spawner,
        bridge,
        symphony_moveit_launch,
        delay_entities_spawner,
    ]

    return nodes_to_launch

def generate_launch_description():
    declared_arguments = []
    declared_arguments.append(DeclareLaunchArgument("name", default_value="symphony"))
    declared_arguments.append(DeclareLaunchArgument(
        "symphony_type",
        default_value="symphony5",
        choices=["symphony5", "symphony10", "symphony15", "symphony20", "symphony40"]
    ))
    declared_arguments.append(DeclareLaunchArgument("prefix", default_value='""'))
    declared_arguments.append(DeclareLaunchArgument("gripper", default_value="robotiq_2f", choices=["robotiq_2f", '""', "none"]))
    declared_arguments.append(DeclareLaunchArgument("launch_rviz", default_value="false"))
    declared_arguments.append(DeclareLaunchArgument("gazebo_on", default_value="true"))
    declared_arguments.append(DeclareLaunchArgument("enable_pnp_hold", default_value="true"))

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
