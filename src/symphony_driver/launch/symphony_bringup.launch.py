from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    ExecuteProcess,
    LogInfo,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.events import Shutdown
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

# Controller_manager timeout after ros2_control_node start (RTPRI connect).
_CM_TIMEOUT = "120"
_CM = "/controller_manager"


def _spawner(name_or_args, *, inactive=False):
    """controller_manager spawner Node."""
    if isinstance(name_or_args, str):
        args = [name_or_args]
    else:
        args = list(name_or_args)
    if inactive and "--inactive" not in args:
        args.append("--inactive")
    args.extend(
        [
            "--controller-manager",
            _CM,
            "--controller-manager-timeout",
            _CM_TIMEOUT,
        ]
    )
    return Node(
        package="controller_manager",
        executable="spawner",
        arguments=args,
        output="screen",
        sigterm_timeout="3.0",
        sigkill_timeout="2.0",
    )


def launch_setup(context, *args, **kwargs):
    description_pkg = FindPackageShare("symphony_description")
    controllers_pkg = FindPackageShare("symphony_controllers")

    name = LaunchConfiguration("name")
    symphony_type = LaunchConfiguration("symphony_type")
    prefix = LaunchConfiguration("prefix")
    gripper = LaunchConfiguration("gripper")
    use_fake_hardware = LaunchConfiguration("use_fake_hardware")
    robot_ip = LaunchConfiguration("robot_ip")
    start_joint_trajectory = LaunchConfiguration("start_joint_trajectory")
    check_duplicate = (
        LaunchConfiguration("check_duplicate_bringup")
        .perform(context)
        .strip()
        .lower()
        in ("true", "1", "yes")
    )

    jtc_args = ["joint_trajectory_controller"]
    jtc_inactive = start_joint_trajectory.perform(context).strip().lower() in (
        "false",
        "0",
        "no",
    )

    controllers_yaml = PathJoinSubstitution(
        [controllers_pkg, "config", "symphony_controllers.yaml"]
    )

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([description_pkg, "urdf", "symphony.urdf.xacro"]),
            " ",
            "name:=",
            name,
            " ",
            "symphony_type:=",
            symphony_type,
            " ",
            "prefix:=",
            prefix,
            " ",
            "gripper:=",
            gripper,
            " ",
            "sim_gazebo:=false",
            " ",
            "use_fake_hardware:=",
            use_fake_hardware,
            " ",
            "robot_ip:=",
            robot_ip,
        ]
    )

    robot_description = {
        "robot_description": ParameterValue(robot_description_content, value_type=str)
    }

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description],
        sigterm_timeout="3.0",
        sigkill_timeout="2.0",
    )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        output="both",
        parameters=[robot_description, controllers_yaml],
        sigterm_timeout="5.0",
        sigkill_timeout="2.0",
    )

    # JSB first: fail => abort remaining spawners.
    joint_state_broadcaster_spawner = _spawner("joint_state_broadcaster")

    rest_spawners = [
        _spawner("status_controller"),
        _spawner("io_controller"),
        _spawner("force_controller"),
        _spawner("tool_modbus_controller"),
        _spawner("motion_options_controller"),
        _spawner("tcp_pose_broadcaster"),
        _spawner("force_torque_sensor_broadcaster"),
        _spawner(jtc_args, inactive=jtc_inactive),
        _spawner("motion_primitive_controller", inactive=True),
        _spawner("native_joint_trajectory_controller", inactive=True),
        _spawner("forward_position_controller", inactive=True),
        _spawner("forward_velocity_controller", inactive=True),
        _spawner("cartesian_pose_controller", inactive=True),
    ]

    def after_jsb_exit(event, _context):
        code = getattr(event, "returncode", 1)
        if code != 0:
            return [
                LogInfo(
                    msg=(
                        "[symphony_bringup] joint_state_broadcaster spawner failed "
                        f"(exit={code}). Hardware configure/activate likely failed; "
                        "not starting other spawners."
                    )
                ),
                EmitEvent(
                    event=Shutdown(
                        reason="joint_state_broadcaster spawner failed "
                        "(check robot_ip / RTPRI / on_configure)"
                    )
                ),
            ]
        return rest_spawners

    delay_jsb_after_control = RegisterEventHandler(
        OnProcessStart(
            target_action=control_node,
            on_start=[
                LogInfo(msg="[symphony_bringup] ros2_control_node started; spawning JSB"),
                joint_state_broadcaster_spawner,
            ],
        )
    )

    delay_rest_after_jsb = RegisterEventHandler(
        OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=after_jsb_exit,
        )
    )

    shutdown_on_control_exit = RegisterEventHandler(
        OnProcessExit(
            target_action=control_node,
            on_exit=[
                LogInfo(msg="[symphony_bringup] ros2_control_node exited; shutting down"),
                EmitEvent(event=Shutdown(reason="ros2_control_node exited")),
            ],
        )
    )

    # Handlers bind by action identity; nodes may start later.
    handlers = [
        delay_jsb_after_control,
        delay_rest_after_jsb,
        shutdown_on_control_exit,
    ]
    core_nodes = [control_node, robot_state_publisher_node]

    if check_duplicate:
        # Block duplicate bringup on same ROS_DOMAIN_ID before RSP/control start.
        preflight = ExecuteProcess(
            cmd=[
                "bash",
                "-lc",
                "if command -v ros2 >/dev/null 2>&1; then "
                "  nodes=$(ros2 node list 2>/dev/null || true); "
                "  if echo \"$nodes\" | grep -qx '/controller_manager'; then "
                "    echo '[symphony_bringup] /controller_manager already running "
                "(duplicate bringup?). Stop the other launch first.'; "
                "    exit 75; "
                "  fi; "
                "fi; "
                "exit 0",
            ],
            name="symphony_bringup_preflight",
            output="screen",
        )

        def after_preflight(event, _context):
            code = getattr(event, "returncode", 1)
            if code == 75:
                return [
                    EmitEvent(
                        event=Shutdown(
                            reason="duplicate /controller_manager (another bringup)"
                        )
                    )
                ]
            if code != 0:
                return [
                    LogInfo(
                        msg=(
                            f"[symphony_bringup] preflight exit={code}; "
                            "continuing (could not confirm duplicate)"
                        )
                    )
                ] + core_nodes
            return list(core_nodes)

        return [
            preflight,
            RegisterEventHandler(
                OnProcessExit(target_action=preflight, on_exit=after_preflight)
            ),
        ] + handlers

    return core_nodes + handlers


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument("name", default_value="symphony"),
        DeclareLaunchArgument(
            "symphony_type",
            default_value="symphony5",
            choices=["symphony5", "symphony10", "symphony15", "symphony20", "symphony40"],
        ),
        DeclareLaunchArgument("prefix", default_value=""),
        DeclareLaunchArgument(
            "gripper", default_value="none", choices=["robotiq_2f", '""', "none"]
        ),
        # Real robot default; set true for fake HW (sim uses other launches).
        DeclareLaunchArgument("use_fake_hardware", default_value="false"),
        DeclareLaunchArgument("robot_ip", default_value="192.168.0.234"),
        # false => JTC inactive at startup (exclusive motion modes).
        DeclareLaunchArgument("start_joint_trajectory", default_value="false"),
        # Abort if /controller_manager already exists.
        DeclareLaunchArgument("check_duplicate_bringup", default_value="true"),
    ]

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
