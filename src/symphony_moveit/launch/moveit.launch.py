import os
import time
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from symphony_moveit.launch_common import load_yaml


def launch_setup(context, *args, **kwargs):
    description_package = FindPackageShare('symphony_description')
    moveit_config_package = FindPackageShare('symphony_moveit')

    # Initialize Arguments
    name = LaunchConfiguration("name")
    symphony_type = LaunchConfiguration("symphony_type")
    prefix = LaunchConfiguration("prefix")
    launch_rviz_moveit = LaunchConfiguration("launch_rviz_moveit")
    use_sim_time = LaunchConfiguration("use_sim_time")
    gripper = LaunchConfiguration("gripper")

    gripper_str = context.perform_substitution(gripper).strip()

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([description_package, "urdf", "symphony.urdf.xacro"]),
            " ",
            "name:=", name, " ",
            "symphony_type:=", symphony_type, " ",
            "gripper:=", gripper, " ",
            "prefix:=", prefix, " ",
            "sim_gazebo:=", LaunchConfiguration("sim_gazebo"), " ",
            "use_fake_hardware:=", LaunchConfiguration("use_fake_hardware"),
        ]
    )
    robot_description = {
        "robot_description": ParameterValue(robot_description_content, value_type=str)
    }

    # MoveIt Configuration
    robot_description_semantic_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([moveit_config_package, "srdf", "symphony.srdf.xacro"]),
            " ",
            "name:=", name, " ",
            "symphony_type:=", symphony_type, " ",
            "prefix:=", prefix, " ",
            "gripper:=", gripper,
        ]
    )
    robot_description_semantic = {
        "robot_description_semantic": ParameterValue(
            robot_description_semantic_content, value_type=str
        )
    }

    robot_description_kinematics = PathJoinSubstitution(
        [moveit_config_package, "moveit_config", "kinematics.yaml"]
    )

    joint_limit_yaml = load_yaml("symphony_moveit", "moveit_config/joint_limits_6dof.yaml")
    if gripper_str not in ['', '""', "''", 'none']:
        joint_limit_yaml = load_yaml("symphony_moveit", "moveit_config/joint_limits_6dof_gripper.yaml")
        
    robot_description_planning = {
        "robot_description_planning": joint_limit_yaml
    }

    ompl_planning_pipeline_config = {
        "move_group": {
            "planning_plugin": "ompl_interface/OMPLPlanner",
            "request_adapters": (
                "default_planner_request_adapters/AddTimeOptimalParameterization "
                "default_planner_request_adapters/ResolveConstraintFrames "
                "default_planner_request_adapters/FixWorkspaceBounds "
                "default_planner_request_adapters/FixStartStateBounds "
                "default_planner_request_adapters/FixStartStateCollision "
                "default_planner_request_adapters/FixStartStatePathConstraints"
            ),
            "start_state_max_bounds_error": 0.1,
            "jiggle_fraction": 0.05,
        }
    }
    ompl_planning_yaml = load_yaml("symphony_moveit", "moveit_config/ompl_planning.yaml")
    ompl_planning_pipeline_config["move_group"].update(ompl_planning_yaml)

    controllers_yaml = load_yaml("symphony_moveit", "moveit_config/controllers_6dof.yaml")
    if gripper_str not in ['', '""', "''", 'none']:
        controllers_yaml = load_yaml("symphony_moveit", "moveit_config/controllers_6dof_gripper.yaml")

    moveit_controllers = {
        "moveit_simple_controller_manager": controllers_yaml,
        "moveit_controller_manager": "moveit_simple_controller_manager/MoveItSimpleControllerManager",
    }

    trajectory_execution = {
        "moveit_manage_controllers": False,
        "trajectory_execution.allowed_execution_duration_scaling": 1.2,
        "trajectory_execution.allowed_goal_duration_margin": 1.0,
        "trajectory_execution.allowed_start_tolerance": 0.5,
        "trajectory_execution.trajectory_duration_monitoring": False
    }

    planning_scene_monitor_parameters = {
        "publish_planning_scene": True,
        "publish_geometry_updates": True,
        "publish_state_updates": True,
        "publish_transforms_updates": True,
        "publish_robot_description":True, 
        "publish_robot_description_semantic":True
    }
    
    # Start the actual move_group node/action server  
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
            robot_description_planning,
            ompl_planning_pipeline_config,
            trajectory_execution,
            moveit_controllers,
            planning_scene_monitor_parameters,
            {"use_sim_time": use_sim_time},
        ],
    )

    # rviz with moveit configuration
    rviz_config_file = PathJoinSubstitution(
        [moveit_config_package, "rviz_config", "symphony_moveit.rviz"]
    )

    rviz_node = Node(
        condition=IfCondition(launch_rviz_moveit),
        package="rviz2",
        executable="rviz2",
        name="rviz2_moveit",
        output="screen",
        arguments=["-d", rviz_config_file],
        parameters=[
            robot_description,
            robot_description_semantic,
            ompl_planning_pipeline_config,
            robot_description_kinematics,
        ],
    )

    nodes_to_start = [move_group_node, rviz_node]

    return nodes_to_start


def generate_launch_description():
    declared_arguments = []

    declared_arguments.append(DeclareLaunchArgument("name", default_value="symphony"))
    declared_arguments.append(DeclareLaunchArgument(
        "symphony_type", 
        default_value="symphony5", 
        choices=["symphony5", "symphony10", "symphony15", "symphony20", "symphony40"]
    ))
    declared_arguments.append(DeclareLaunchArgument("prefix", default_value='""'))
    
    declared_arguments.append(DeclareLaunchArgument(
        "gripper", 
        default_value='""', 
        choices=["robotiq_2f", '""', "none"]
    ))
    
    declared_arguments.append(DeclareLaunchArgument("sim_gazebo", default_value="false"))
    declared_arguments.append(DeclareLaunchArgument("use_fake_hardware", default_value="true"))
    declared_arguments.append(DeclareLaunchArgument("use_sim_time", default_value="false"))
    declared_arguments.append(DeclareLaunchArgument("launch_rviz_moveit", default_value="true"))

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])