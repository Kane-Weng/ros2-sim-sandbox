import os

from launch_ros.actions import Node
from launch import LaunchDescription
from launch.event_handlers import OnProcessExit
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration
from launch.actions import ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler, DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Declare the use_sim_time argument
    use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )

    # Launch configuration for use_sim_time
    use_sim_time_config = LaunchConfiguration('use_sim_time')

    waypoint_file = os.path.join(get_package_share_directory('sandbox_navigation'), 'config', 'waypoints.yaml')
    world_file = os.path.join(get_package_share_directory("sandbox_gazebo"), 'worlds', 'empty.world')
    gazebo_params_file = os.path.join(get_package_share_directory("sandbox_gazebo"), 'config', 'gazebo_params.yaml')
    rviz_file = os.path.join(get_package_share_directory('sandbox_description'), 'rviz', 'navigation.rviz')

    # Start Gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
        launch_arguments={
            'world': world_file,
            'extra_gazebo_args': '--ros-args --params-file ' + gazebo_params_file
        }.items()
    )    

    # Get URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("sandbox_description"), "description", 'vehicle.urdf.xacro']
            ),
        ]
    )
    robot_description = {
        "robot_description": ParameterValue(robot_description_content, value_type=str)
    }

    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description, {'use_sim_time': use_sim_time_config}]
    )

    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'sim_vehicle'],
        output='screen',
        parameters=[{'use_sim_time': use_sim_time_config}]
    )

    load_joint_state_broadcaster = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'joint_state_broadcaster'],
        output='screen'
    )

    load_tricycle_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'ackermann_like_controller'],
        output='screen'
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=[
            '-d', rviz_file
        ],
        output='screen',
        parameters=[{'use_sim_time': use_sim_time_config}]
    )

    waypoint_visualizer = Node(
        package='sandbox_gazebo',
        executable='waypoint_visualizer.py',
        name='waypoint_visualizer',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time_config},
            waypoint_file
        ]
    )

    return LaunchDescription([
        use_sim_time,
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn_entity,
                on_exit=[load_joint_state_broadcaster],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_joint_state_broadcaster,
                on_exit=[load_tricycle_controller],
            )
        ),
        gazebo,
        rviz,
        node_robot_state_publisher,
        spawn_entity,
        waypoint_visualizer,
    ])