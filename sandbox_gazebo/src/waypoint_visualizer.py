#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point

class WaypointVisualizer(Node):
    def __init__(self):
        super().__init__('waypoint_visualizer')
        self.publisher = self.create_publisher(MarkerArray, 'navigation_waypoints', 10)
        
        # Waypoints
        # TODO: Set waypoints using yaml
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

    def publish_markers(self):
        marker_array = MarkerArray()
        
        # Sphere Markers for points
        for i, wp in enumerate(self.waypoints):
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "waypoint_nodes"
            marker.id = i
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position.x = float(wp[0])
            marker.pose.position.y = float(wp[1])
            marker.pose.position.z = 0.1
            marker.scale.x = 0.3
            marker.scale.y = 0.3
            marker.scale.z = 0.3
            marker.color.a = 1.0
            marker.color.g = 1.0 # Green dots
            marker_array.markers.append(marker)

        # Line Strip to show the path
        line_strip = Marker()
        line_strip.header.frame_id = "map"
        line_strip.header.stamp = self.get_clock().now().to_msg()
        line_strip.ns = "waypoint_path"
        line_strip.id = 100
        line_strip.type = Marker.LINE_STRIP
        line_strip.action = Marker.ADD
        line_strip.scale.x = 0.05 # Line width
        line_strip.color.a = 0.8
        line_strip.color.r = 1.0
        line_strip.color.g = 1.0 # Yellow path

        for wp in self.waypoints:
            p = Point()
            p.x = float(wp[0])
            p.y = float(wp[1])
            p.z = 0.05
            line_strip.points.append(p)
        
        marker_array.markers.append(line_strip)
        self.publisher.publish(marker_array)

def main():
    rclpy.init()
    node = WaypointVisualizer()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()