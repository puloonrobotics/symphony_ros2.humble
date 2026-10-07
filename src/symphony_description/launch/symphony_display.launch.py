#!/usr/bin/python3
#-*- coding: utf-8 -*-

from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    # Declare arguments
    declared_arguments = []

    declared_arguments.append(
        DeclareLaunchArgument(
            "name",
            default_value="symphony"
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "symphony_type",
            default_value="symphony5",
            description="Type of symphony robot.",
            choices=["symphony5", "symphony10", "symphony15", "symphony20", "symphony40"]
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "gripper",
            default_value="",
            description="Type of gripper",
            choices=["", "robotiq_2f"]
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "prefix",
            default_value='""',
            description="Prefix of the joint names, useful for \
            multi-robot setup. If changed than also joint names in the controllers configuration \
            have to be updated."
        )
    )

    # Initialize Arguments
    name = LaunchConfiguration("name")
    symphony_type = LaunchConfiguration("symphony_type")
    gripper = LaunchConfiguration("gripper")
    prefix = LaunchConfiguration("prefix")

    description_package = FindPackageShare('symphony_description')

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([description_package, "urdf", 'symphony.urdf.xacro']),
            " ",
            "name:=",
            name,
            " ",
            "symphony_type:=",
            symphony_type,
            " ",
            "gripper:=",
            gripper,
            " ",
            "prefix:=",
            prefix
        ]
    )

    robot_description = {"robot_description": ParameterValue(robot_description_content, value_type=str)}

    rviz_config_file = PathJoinSubstitution(
        [description_package, "rviz_config", "symphony.rviz"]
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description]
    )

    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui'
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file]
    )

    nodes = [
        joint_state_publisher_gui_node,
        robot_state_publisher_node,
        rviz_node,
    ]

    return LaunchDescription(declared_arguments + nodes)
