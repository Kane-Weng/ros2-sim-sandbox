from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument("gui", default_value="true"),
        DeclareLaunchArgument('use_sim_time', default_value='true')
    ]
    
    # Get URDF via xacro
    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name="xacro")]),
        " ",
        PathJoinSubstitution(
            [FindPackageShare("sandbox_description"), "description", "vehicle.urdf.xacro"]
        ),
    ])

    # Define state publisher nodes
    robot_description = {
        "robot_description": ParameterValue(robot_description_content, value_type=str),
        'use_sim_time': ParameterValue(LaunchConfiguration("use_sim_time"), value_type=bool),
    }

    joint_state_publisher_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        condition=IfCondition(LaunchConfiguration("gui")),
    )
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
        arguments=["--ros-args", "--log-level", "info"]
    )

    # Rviz for visualization
    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("sandbox_description"), "rviz", "robot_view.rviz"]
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        condition=IfCondition(LaunchConfiguration("gui")),
    )

    return LaunchDescription(declared_arguments + [
        joint_state_publisher_node,
        robot_state_publisher_node,
        rviz_node,
    ])