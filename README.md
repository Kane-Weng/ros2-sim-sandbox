# ros2-sim-sandbox

This repo is a ROS 2 simulation environment designed for testing and benchmarking localization and navigation algorithms.

## Overview
- **Simulation**: Gazebo Classic integration with Ackermann kinematics
- **Localization**: Sensor fusion (IMU, Odometry, GPS) using EKF
- **Navigation**: Custom waypoint-following based on Pure Pursuit algorithm

## Setup Requirements
- **OS**: Ubuntu 22.04 (Jammy Jellyfish)
- **ROS2 Distro**: Humble Hawksbill
- **Simulator**: Gazebo Classic 11
- **Kinematics**: Ackermann Steering (Plugin-based)

## Repo Structure
| Package | Description |
| :--- | :--- |
| `sandbox_description` | URDF/Xacro files for the simplified robot model and sensor placements. |
| `sandbox_gazebo` | World files, launch configurations, and Gazebo plugin integrations. |
| `sandbox_localization` | EKF configurations for IMU/Odom fusion (current work: GPS integration). |
| `sandbox_navigation` | Custom Pure Pursuit controller and waypoint management logic. |
| `sandbox_perception` | *(Future)* LiDAR-based object detection and point cloud processing. |

## Credits
This project was inspired by and utilizes logic/kinematics structures from the [hunter_robot](https://github.com/LCAS/hunter_robot) repository (Apache License 2.0).

## License
This project is licensed under the Apache License ([License](/LICENSE)). 

## Reference 
- [Pure Pursuit Algorithm](https://thomasfermi.github.io/Algorithms-for-Automated-Driving/Control/PurePursuit.html)
