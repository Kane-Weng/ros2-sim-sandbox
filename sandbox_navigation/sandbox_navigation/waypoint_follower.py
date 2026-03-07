import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
import math
import numpy as np
from .navigation_utils import NavAlgos

# Constants
NAV_GOAL_TOLERANCE = 1.0        # [m]
NAV_PURE_PURSUIT_LD_MIN = 4.0   # [m]
NAV_PURE_PURSUIT_LD_MAX = 8.0   # [m]
NAV_BASE_SPEED = 3.5            # [m/s]
NAV_MIN_SPEED = 1.0             # [m/s]
NAV_CTE_TOLERANCE = 1.5         # [m]

class WaypointFollower(Node):
    def __init__(self):
        super().__init__('waypoint_follower')

        # TF Setup
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Publishers and Subscribers
        self.odom_sub = self.create_subscription(Odometry, '/odometry/global', self.velocity_callback, 10)
        self.cmd_pub = self.create_publisher(Twist, '/ackermann_like_controller/cmd_vel', 10)
        
        # TODO: Set waypoints
        self.waypoints = [
            # Straight away
            [0.0, 0.0], [10.0, 0.0], [20.0, 0.0], 
            # Large 10m radius turn (Center at 20, 10)
            [27.07, 2.93], [30.0, 10.0], [27.07, 17.07], 
            # Straight back
            [20.0, 20.0], [10.0, 20.0], [0.0, 20.0], 
            # Turn back to start (Center at 0, 10)
            [-7.07, 17.07], [-10.0, 10.0], [-7.07, 2.93], [0.0, 0.0]
        ]

        # Variables for control loop
        self.current_wp_idx = 0
        self.curr_x = 0.0
        self.curr_y = 0.0
        self.curr_x_rear = 0.0
        self.curr_y_rear = 0.0
        self.curr_yaw = 0.0
        self.curr_vel = 0.0
        
        self.timer = self.create_timer(0.1, self.control_loop)

    def velocity_callback(self, msg):
        # Get the current vehicle speed for the adaptive lookahead calculation
        self.curr_vel = msg.twist.twist.linear.x 

    def get_robot_pose(self):
        """
        Look up the current position of the robot in the MAP frame.
        This automatically handles the GPS + Odom fusion.
        """
        try:
            # Look up transform from 'map' to 'base_link'
            now = rclpy.time.Time()
            trans = self.tf_buffer.lookup_transform('map', 'base_link', now)
            
            # The translation between 'map' and 'base_link' is the vehicle's coordinates on the map
            self.curr_x = trans.transform.translation.x
            self.curr_y = trans.transform.translation.y
            
            # Convert Quaternion to Yaw
            q = trans.transform.rotation
            self.curr_yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))

            # Compute the location of the rear of the vehicle (for pure pursuit)       
            self.curr_x_rear = self.curr_x - (NavAlgos.VEHICLE_WHEELBASE/2.0)*math.cos(self.curr_yaw)
            self.curr_y_rear = self.curr_y - (NavAlgos.VEHICLE_WHEELBASE/2.0)*math.sin(self.curr_yaw)
            return True
        
        except TransformException as ex:
            self.get_logger().info(f'Could not transform map to base_link: {ex}')
            return False    

    def control_loop(self):
        # Update pose from TF instead of Odom topic
        if not self.get_robot_pose():
            return
        
        # Check if the next waypoint is the last one
        if self.current_wp_idx+1 >= len(self.waypoints) - 1:
            last_wp = self.waypoints[-1]
            dist_final = math.sqrt((last_wp[0]-self.curr_x)**2 + (last_wp[1]-self.curr_y)**2)
            # Stop if achieve goal tolerance
            if dist_final < NAV_GOAL_TOLERANCE:
                self._stop_robot()
                self.get_logger().info("Arrived at Final Destination.")
                self.timer.cancel()
                return
        
        # Define Adaptive Lookahead (ld)
        speed_ratio = np.clip(abs(self.curr_vel)/NAV_BASE_SPEED, 0.0, 1.0)
        ld = NAV_PURE_PURSUIT_LD_MIN*(1.0-speed_ratio) + NAV_PURE_PURSUIT_LD_MAX*speed_ratio

        # Get Interpolated Target Point and CTE
        tx, ty, cte = self._get_lookahead_point(ld)

        # Run pure pursuit navigation algorithm
        steering_angle = NavAlgos.pure_pursuit(
            self.curr_x_rear, self.curr_y_rear,     # Use rear here
            self.curr_yaw, tx, ty, ld
        )

        # Implement Speed Control (consider CTE & sharp steering)
        base_speed = NAV_BASE_SPEED
        speed_factor = np.clip(1.0 - (cte / NAV_CTE_TOLERANCE), 0.5, 1.0)
        steer_limit_factor = 1.0 - (abs(steering_angle)/NavAlgos.VEHICLE_MAX_STEERING_ANGLE)*0.4
        target_speed = base_speed*speed_factor*steer_limit_factor

        # Publish speed and steering angle to the controller
        msg = Twist()
        msg.linear.x = np.clip(target_speed, NAV_MIN_SPEED, 
                               NavAlgos.VEHICLE_MAX_SPEED)
        msg.angular.z = np.clip(steering_angle, 
                                -NavAlgos.VEHICLE_MAX_STEERING_ANGLE, 
                                NavAlgos.VEHICLE_MAX_STEERING_ANGLE)
        self.cmd_pub.publish(msg)
        
        self.get_logger().info(f'v: {target_speed:.2f} | Ld: {ld:.2f} | CTE: {cte:.2f} | next_WP: {self.current_wp_idx+1}')
        
    def _get_lookahead_point(self, lookahead_dist):
        """
        Finds the point on the path that is 'lookahead_dist' away from (curr_x, curr_y).
        Returns: (target_x, target_y, cross_track_error)
        """
        best_point = None
        cte = 0.0
        # Search {i}-{i+1} segment & {i+1}-{i+2} segment only
        search_end_idx = min(self.current_wp_idx + 2, len(self.waypoints) - 1)

        # Iterate through current & next path segments to find the best target point
        for i in range(self.current_wp_idx, search_end_idx):
            p1 = np.array(self.waypoints[i])
            p2 = np.array(self.waypoints[i+1])
            curr_pos = np.array([self.curr_x_rear, self.curr_y_rear])

            V = p2 - p1         # Vector from P1 to P2
            D = p1 - curr_pos   # Vector from Robot to P1
            # Solve quadratic: |p1 + t*V - curr_pos|^2 = Ld^2
            #                  ((V^2)t^2 + (2DV)t + (D^2 - Ld^2) = 0)
            a = np.dot(V, V)
            b = 2 * np.dot(D, V)
            c = np.dot(D, D) - lookahead_dist**2
            discriminant = b**2 - 4*a*c

            if discriminant >= 0:
                discriminant = math.sqrt(discriminant)
                # t1 = (-b - discriminant) / (2 * a)
                t2 = (-b + discriminant) / (2 * a)

                # Check the further point (t2), ignore the point closer to p1 (t1)
                if 0 <= t2 <= 1:
                    best_point = p1 + t2 * V
                    # Check if the valid target point is on the next segment, 
                    # Then update our waypoint index progress
                    if i > self.current_wp_idx:
                        self.get_logger().info(f"Advancing to Segment {i}-{i+1}")
                        self.current_wp_idx = i
                    break

        # Calculate Cross Track Error (CTE) for speed control
        p1 = np.array(self.waypoints[self.current_wp_idx])
        p2 = np.array(self.waypoints[self.current_wp_idx+1])
        cte = abs(np.cross(p2-p1, p1-np.array([self.curr_x_rear, self.curr_y_rear]))) / np.linalg.norm(p2-p1)       

        # Fallback: if no intersection (segment too short or Ld too big), 
        # Aim for the next waypoint 
        if best_point is None:
            target = self.waypoints[min(self.current_wp_idx+1, len(self.waypoints)-1)]
            return target[0], target[1], cte

        return best_point[0], best_point[1], cte
    
    def _stop_robot(self):
        msg = Twist()
        self.cmd_pub.publish(msg)

    # TODO: add visualization tools in rviz

def main():
    rclpy.init()
    node = WaypointFollower()
    rclpy.spin(node)
    rclpy.shutdown()