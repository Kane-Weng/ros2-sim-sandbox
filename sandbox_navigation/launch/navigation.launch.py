import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    config_path = os.path.join(
        get_package_share_directory('sandbox_navigation'),
        'config',
        'waypoints.yaml'
    )

    waypoint_follower_node = Node(
        package='sandbox_navigation',       
        executable='waypoint_follower',   
        name='waypoint_follower_node',
        output='screen',
        parameters=[
            config_path,
        ]
    )

    return LaunchDescription([
        waypoint_follower_node
    ])