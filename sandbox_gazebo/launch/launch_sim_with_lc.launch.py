import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():

    localization_launch_path = os.path.join(
        get_package_share_directory('sandbox_localization'),
        'launch',
        'localization.launch.py'
    )

    simulation_launch_path = os.path.join(
        get_package_share_directory('sandbox_gazebo'),
        'launch',
        'launch_sim.launch.py'
    )

    # ros2 launch sandbox_bringup localization.launch.py
    launch_localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(localization_launch_path)
    )

    # ros2 launch sandbox_gazebo launch_sim.launch.py
    launch_simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(simulation_launch_path)
    )

    ld = LaunchDescription()
    ld.add_action(launch_simulation)
    ld.add_action(launch_localization)

    return ld