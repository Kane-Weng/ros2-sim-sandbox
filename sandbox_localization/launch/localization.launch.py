import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('sandbox_localization')
    local_ekf_config = os.path.join(pkg_share, 'config', 'ekf_local.yaml')
    global_ekf_config = os.path.join(pkg_share, 'config', 'ekf_global.yaml')

    # LOCAL EKF (Continuous, no GPS)
    # odom -> base_link transform
    local_ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node_odom',
        output='screen',
        parameters=[local_ekf_config],
        remappings=[
            ('odometry/filtered', '/odometry/local')
        ]
    )

    # GLOBAL EKF (Fuses GPS)
    # map -> odom transform
    global_ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node_map',
        output='screen',
        parameters=[global_ekf_config],
        remappings=[
            ('odometry/filtered', '/odometry/global')
        ]
    )

    # NAVSAT TRANSFORM (GPS -> Odometry)
    navsat_node = Node(
        package='robot_localization',
        executable='navsat_transform_node',
        name='navsat_transform',
        output='screen',
        parameters=[global_ekf_config], 
        remappings=[
            ('imu', '/imu/data'),
            ('gps/fix', '/gps/data_1'),
            ('odometry/filtered', '/odometry/global'),
            ('gps/filtered', '/gps/filtered'),
            ('odometry/gps', '/odometry/gps')
        ]
    )

    return LaunchDescription([
        local_ekf_node,
        global_ekf_node,
        navsat_node
    ])