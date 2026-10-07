from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare

def launch_setup(context, *args, **kwargs):
    
    # description_package = FindPackageShare('symphony_description')
    gazebo_package = FindPackageShare('symphony_gazebo')
    moveit_config_package = FindPackageShare('symphony_moveit')

    # Initialize Arguments
    name = LaunchConfiguration("name")
    symphony_type = LaunchConfiguration("symphony_type")
    gripper = LaunchConfiguration("gripper")
    prefix = LaunchConfiguration("prefix")

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
            'gazebo_on' : "false",
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
            "launch_rviz_moveit": "true", # if name == "launch_rviz" => spawn 2 rviz
            'gripper' : gripper,
        }.items(),
    )

    nodes_to_launch = [
        symphony_gazebo_launch,
        symphony_moveit_launch,
    ]

    return nodes_to_launch


def generate_launch_description():
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
            "prefix",
            default_value='""',
            description="Prefix of the joint names, useful for multi-robot setup. \
            If changed than also joint names in the controllers configuration have to be updated."
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "gripper", 
            default_value='""', 
            description="choose robot gripper",
            choices=["robotiq_2f", '""']
        )
    )

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])