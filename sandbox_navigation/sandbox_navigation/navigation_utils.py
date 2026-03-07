import math
import numpy as np

class NavAlgos:
    # Constants
    VEHICLE_WHEELBASE = 2.082           # [m]
    VEHICLE_MAX_STEERING_ANGLE = 0.2618 # [rad], equivalent to ±15 degrees
    VEHICLE_MAX_SPEED = 4.91744         # [m/s]

    @staticmethod
    def simple_goto(curr_x, curr_y, curr_yaw, target_x, target_y):
        """Simple Proportional Control"""
        dx = target_x - curr_x
        dy = target_y - curr_y
        angle_to_target = math.atan2(dy, dx)
        alpha = angle_to_target - curr_yaw
        return math.atan2(math.sin(alpha), math.cos(alpha))  # Normalize alpha

    @staticmethod
    def pure_pursuit(curr_x_rear, curr_y_rear, curr_yaw, target_x, target_y, lookahead_dist):
        """
        Pure Pursuit Logic:
        Calculates the curvature required to hit a lookahead point.
        steering_angle = atan(2 * L * sin(alpha) / lookahead_dist)
        """
        L = NavAlgos.VEHICLE_WHEELBASE
        dx = target_x - curr_x_rear
        dy = target_y - curr_y_rear
        
        # Angle to the lookahead point
        alpha = math.atan2(dy, dx) - curr_yaw
        alpha = math.atan2(math.sin(alpha), math.cos(alpha))    # Normalize alpha
        
        # Steering Formula
        steering_angle = math.atan2(2.0 * L * math.sin(alpha), lookahead_dist)
        return steering_angle