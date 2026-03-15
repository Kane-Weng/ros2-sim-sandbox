#!/usr/bin/env python3
import math
from collections import deque
from typing import List, Tuple

import numpy as np
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Point32, PolygonStamped
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2


# ----------------------------
# Utility functions
# ----------------------------

def point_in_roi(p: np.ndarray,
                 min_x: float, max_x: float,
                 min_y: float, max_y: float,
                 min_z: float, max_z: float) -> bool:
    return (
        min_x <= p[0] <= max_x and
        min_y <= p[1] <= max_y and
        min_z <= p[2] <= max_z
    )


def fit_plane_from_points(p1: np.ndarray, p2: np.ndarray, p3: np.ndarray):
    v1 = p2 - p1
    v2 = p3 - p1
    normal = np.cross(v1, v2)
    norm = np.linalg.norm(normal)
    if norm < 1e-9:
        return None
    normal = normal / norm
    d = -np.dot(normal, p1)
    return normal, d


def point_plane_distance(points: np.ndarray, normal: np.ndarray, d: float) -> np.ndarray:
    return np.abs(points @ normal + d)


def ransac_ground_segmentation(points: np.ndarray,
                               max_iterations: int,
                               distance_threshold: float,
                               max_ground_tilt_deg: float):
    """
    Returns:
      ground_points: (G,3)
      nonground_points: (N,3)
    """
    if len(points) < 3:
        return np.empty((0, 3)), points

    best_inliers = np.array([], dtype=int)
    best_model = None

    rng = np.random.default_rng()

    for _ in range(max_iterations):
        idx = rng.choice(len(points), size=3, replace=False)
        model = fit_plane_from_points(points[idx[0]], points[idx[1]], points[idx[2]])
        if model is None:
            continue

        normal, d = model

        # We want a roughly horizontal ground plane in the LiDAR frame.
        # Assume +Z is up in the LiDAR frame.
        z_axis = np.array([0.0, 0.0, 1.0])
        cos_theta = np.clip(np.abs(np.dot(normal, z_axis)), -1.0, 1.0)
        tilt_deg = math.degrees(math.acos(cos_theta))

        if tilt_deg > max_ground_tilt_deg:
            continue

        distances = point_plane_distance(points, normal, d)
        inliers = np.where(distances < distance_threshold)[0]

        if len(inliers) > len(best_inliers):
            best_inliers = inliers
            best_model = (normal, d)

    if best_model is None or len(best_inliers) == 0:
        return np.empty((0, 3)), points

    mask = np.zeros(len(points), dtype=bool)
    mask[best_inliers] = True
    ground_points = points[mask]
    nonground_points = points[~mask]

    return ground_points, nonground_points


def dbscan_2d(points_xy: np.ndarray, eps: float, min_points: int) -> List[np.ndarray]:
    """
    Simple pure-Python/Numpy DBSCAN over XY points.
    Returns a list of arrays of point indices, one array per cluster.
    """
    n = len(points_xy)
    if n == 0:
        return []

    UNVISITED = -99
    NOISE = -1

    labels = np.full(n, UNVISITED, dtype=int)
    cluster_id = 0

    def region_query(i: int) -> np.ndarray:
        d = np.linalg.norm(points_xy - points_xy[i], axis=1)
        return np.where(d <= eps)[0]

    for i in range(n):
        if labels[i] != UNVISITED:
            continue

        neighbors = region_query(i)
        if len(neighbors) < min_points:
            labels[i] = NOISE
            continue

        labels[i] = cluster_id
        seeds = deque(neighbors.tolist())

        while seeds:
            j = seeds.popleft()

            if labels[j] == NOISE:
                labels[j] = cluster_id

            if labels[j] != UNVISITED:
                continue

            labels[j] = cluster_id
            j_neighbors = region_query(j)

            if len(j_neighbors) >= min_points:
                for nn in j_neighbors:
                    if labels[nn] in (UNVISITED, NOISE):
                        seeds.append(nn)

        cluster_id += 1

    clusters = []
    for cid in range(cluster_id):
        idx = np.where(labels == cid)[0]
        if len(idx) > 0:
            clusters.append(idx)

    return clusters


def convex_hull_2d(points_xy: np.ndarray) -> np.ndarray:
    """
    Monotonic chain convex hull.
    Returns hull points ordered around the hull.
    """
    if len(points_xy) < 3:
        return points_xy

    pts = sorted(set((float(p[0]), float(p[1])) for p in points_xy))
    if len(pts) < 3:
        return np.array(pts, dtype=np.float32)

    def cross(o, a, b):
        return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])

    lower = []
    for p in pts:
        while len(lower) >= 2 and cross(lower[-2], lower[-1], p) <= 0:
            lower.pop()
        lower.append(p)

    upper = []
    for p in reversed(pts):
        while len(upper) >= 2 and cross(upper[-2], upper[-1], p) <= 0:
            upper.pop()
        upper.append(p)

    hull = lower[:-1] + upper[:-1]
    return np.array(hull, dtype=np.float32)


def make_cloud_msg(header, xyz: np.ndarray) -> PointCloud2:
    pts = [(float(p[0]), float(p[1]), float(p[2])) for p in xyz]
    return point_cloud2.create_cloud_xyz32(header, pts)


# ----------------------------
# Node
# ----------------------------

class LidarObstacleNode(Node):
    def __init__(self):
        super().__init__('LidarObstacleNode')

        # Parameters
        self.declare_parameter('input_topic', '/velodyne_points')
        self.declare_parameter('obstacles_topic', '/obstacles')
        self.declare_parameter('publish_debug_clouds', True)

        self.declare_parameter('roi_min_x', 0.0)
        self.declare_parameter('roi_max_x', 12.0)
        self.declare_parameter('roi_min_y', -4.0)
        self.declare_parameter('roi_max_y', 4.0)
        self.declare_parameter('roi_min_z', -1.5)
        self.declare_parameter('roi_max_z', 2.0)

        self.declare_parameter('ransac_max_iterations', 120)
        self.declare_parameter('ransac_distance_threshold', 0.15)
        self.declare_parameter('max_ground_tilt_deg', 20.0)

        self.declare_parameter('dbscan_eps', 0.45)
        self.declare_parameter('dbscan_min_points', 8)
        self.declare_parameter('min_cluster_size', 8)
        self.declare_parameter('max_cluster_size', 5000)

        self.declare_parameter('min_hull_points', 3)

        input_topic = self.get_parameter('input_topic').value
        obstacles_topic = self.get_parameter('obstacles_topic').value
        self.publish_debug_clouds = bool(self.get_parameter('publish_debug_clouds').value)

        self.roi_min_x = float(self.get_parameter('roi_min_x').value)
        self.roi_max_x = float(self.get_parameter('roi_max_x').value)
        self.roi_min_y = float(self.get_parameter('roi_min_y').value)
        self.roi_max_y = float(self.get_parameter('roi_max_y').value)
        self.roi_min_z = float(self.get_parameter('roi_min_z').value)
        self.roi_max_z = float(self.get_parameter('roi_max_z').value)

        self.ransac_max_iterations = int(self.get_parameter('ransac_max_iterations').value)
        self.ransac_distance_threshold = float(self.get_parameter('ransac_distance_threshold').value)
        self.max_ground_tilt_deg = float(self.get_parameter('max_ground_tilt_deg').value)

        self.dbscan_eps = float(self.get_parameter('dbscan_eps').value)
        self.dbscan_min_points = int(self.get_parameter('dbscan_min_points').value)
        self.min_cluster_size = int(self.get_parameter('min_cluster_size').value)
        self.max_cluster_size = int(self.get_parameter('max_cluster_size').value)

        self.min_hull_points = int(self.get_parameter('min_hull_points').value)

        # ROS interfaces
        self.subscription = self.create_subscription(
            PointCloud2,
            input_topic,
            self.pointcloud_callback,
            10
        )

        self.obstacles_pub = self.create_publisher(PolygonStamped, obstacles_topic, 50)

        if self.publish_debug_clouds:
            self.ground_pub = self.create_publisher(PointCloud2, '/ground_points', 10)
            self.nonground_pub = self.create_publisher(PointCloud2, '/nonground_points', 10)
            self.front_pub = self.create_publisher(PointCloud2, '/front_points', 10)

        self.get_logger().info('LiDAR obstacle publisher is ready')

    def pointcloud_callback(self, msg: PointCloud2):
        # Read XYZ points. Skip NaNs immediately.
        xyz = np.array(
            list(point_cloud2.read_points(msg, field_names=('x', 'y', 'z'), skip_nans=True)),
            dtype=np.float32
        )

        if xyz.size == 0:
            self.get_logger().warn('Received empty point cloud')
            return

        # 1) Ground segmentation on full local cloud
        ground_pts, nonground_pts = ransac_ground_segmentation(
            xyz,
            max_iterations=self.ransac_max_iterations,
            distance_threshold=self.ransac_distance_threshold,
            max_ground_tilt_deg=self.max_ground_tilt_deg
        )

        # 2) Crop to region in front of the LiDAR
        if len(nonground_pts) > 0:
            roi_mask = np.array([
                point_in_roi(
                    p,
                    self.roi_min_x, self.roi_max_x,
                    self.roi_min_y, self.roi_max_y,
                    self.roi_min_z, self.roi_max_z
                )
                for p in nonground_pts
            ], dtype=bool)
            front_pts = nonground_pts[roi_mask]
        else:
            front_pts = np.empty((0, 3), dtype=np.float32)

        # Debug clouds
        if self.publish_debug_clouds:
            self.ground_pub.publish(make_cloud_msg(msg.header, ground_pts))
            self.nonground_pub.publish(make_cloud_msg(msg.header, nonground_pts))
            self.front_pub.publish(make_cloud_msg(msg.header, front_pts))

        if len(front_pts) < self.min_cluster_size:
            return

        # 3) DBSCAN in XY plane
        clusters = dbscan_2d(front_pts[:, :2], eps=self.dbscan_eps, min_points=self.dbscan_min_points)

        obstacle_count = 0

        for cluster_indices in clusters:
            if len(cluster_indices) < self.min_cluster_size or len(cluster_indices) > self.max_cluster_size:
                continue

            cluster_pts = front_pts[cluster_indices]
            hull_xy = convex_hull_2d(cluster_pts[:, :2])

            if len(hull_xy) < self.min_hull_points:
                continue

            poly = PolygonStamped()
            poly.header = msg.header

            for p in hull_xy:
                pt = Point32()
                pt.x = float(p[0])
                pt.y = float(p[1])
                pt.z = 0.0
                poly.polygon.points.append(pt)

            self.obstacles_pub.publish(poly)
            obstacle_count += 1

        self.get_logger().info(
            f'Published {obstacle_count} obstacle polygon(s) in frame "{msg.header.frame_id}"'
        )


def main(args=None):
    rclpy.init(args=args)
    node = LidarObstacleNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()