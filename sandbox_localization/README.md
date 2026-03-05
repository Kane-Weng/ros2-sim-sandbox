# Localization package

This package implements a **Dual-EKF (Extended Kalman Filter)** localization system for an Ackermann-style robot using `robot_localization`. It fuses wheel odometry, IMU data, and GPS to provide a continuous, drift-free estimate of the robot's position and orientation.

## Architecture

- **Local EKF** (`ekf_filter_node_odom`):
    - Frame: `odom` → `base_link`
    - Input: Wheel Odometry and IMU.
    - Topic: `/odometry/local`
    - Purpose: Provides a smooth, continuous state estimate for local navigation. It will drift over time but never jumps.

- **Global EKF** (`ekf_filter_node_map`):
    - Frame: `map` → `odom`
    - Input: Wheel Odometry, IMU, and GPS.
    - Topic: `/odometry/global`
    - Purpose: Corrects long-term drift by fusing absolute global coordinates (UTM).

- **NavSat Transform**:
    - Translates raw GPS (`sensor_msgs/NavSatFix`) into a Cartesian coordinate system compatible with the EKF.
    - Topic: `/odometry/gps`

## Reference Documents

- Backbone of this package: [`robot_localization`](https://docs.ros.org/en/melodic/api/robot_localization/html/index.html)
- ROS standards: [REP-103](http://www.ros.org/reps/rep-0103.html) & [REP-105](http://www.ros.org/reps/rep-0105.html)


