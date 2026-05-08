import rclpy
from rclpy.node import Node

from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from geometry_msgs.msg import PointStamped

import cv2
import numpy as np
import open3d as o3d


MIN_CLUSTER_POINTS = 50
DBSCAN_EPS = 0.02      
DBSCAN_MIN_POINTS = 30


class CubeTracker(Node):

    def __init__(self):
        super().__init__('cube_tracker')

        self.sub = self.create_subscription(
            PointCloud2,
            '/camera/camera/depth/color/points',
            self.callback,
            10,
        )

        self.pub_red = self.create_publisher(PointStamped, '/cube_red_3d', 10)
        self.pub_blue = self.create_publisher(PointStamped, '/cube_blue_3d', 10)

        # Open3D visualizer
        self.vis = o3d.visualization.Visualizer()
        self.vis.create_window(window_name='Red + Blue Cube Tracker')

        self.pcd = o3d.geometry.PointCloud()
        self.vis.add_geometry(self.pcd)

        self.first = True

        self.get_logger().info("Cube Tracker started")

    def callback(self, msg):

        pc = point_cloud2.read_points(
            msg,
            field_names=("x", "y", "z", "rgb"),
            skip_nans=True,
        )

        xyz, rgb_float = self._extract_xyz_rgb(pc)
        if xyz.size == 0:
            return

        rgb = self._unpack_rgb(rgb_float)

        hsv = cv2.cvtColor(rgb[np.newaxis, :, :], cv2.COLOR_RGB2HSV)[0]
        h, s, v = hsv[:, 0], hsv[:, 1], hsv[:, 2]

        red_mask = ((h < 10) | (h > 170)) & (s > 120) & (v > 70)
        blue_mask = (h > 100) & (h < 140) & (s > 150) & (v > 50)

        # Publish cubes with clustering + filtering
        self._publish_clusters(xyz, red_mask, msg.header, self.pub_red, 'red')
        self._publish_clusters(xyz, blue_mask, msg.header, self.pub_blue, 'blue')

        # Visualization
        colors = rgb.astype(np.float32) / 255.0
        if red_mask.any():
            colors[red_mask] = (1.0, 0.0, 0.0)
        if blue_mask.any():
            colors[blue_mask] = (0.0, 0.0, 1.0)

        self.pcd.points = o3d.utility.Vector3dVector(xyz)
        self.pcd.colors = o3d.utility.Vector3dVector(colors)

        if self.first:
            self.vis.reset_view_point(True)
            self.first = False

        self.vis.update_geometry(self.pcd)
        self.vis.poll_events()
        self.vis.update_renderer()

    # -----------------------------
    # CLUSTERING + FILTERING
    # -----------------------------
    def _publish_clusters(self, xyz, mask, header, publisher, label):

        points = xyz[mask]
        if len(points) < MIN_CLUSTER_POINTS:
            return

        clusters = self._get_clusters(points)

        for cluster in clusters:

            if len(cluster) < MIN_CLUSTER_POINTS:
                continue

            if not self._is_cube_like(cluster):
                continue

            centroid = cluster.mean(axis=0)

            pt = PointStamped()
            pt.header = header
            pt.point.x = float(centroid[0])
            pt.point.y = float(centroid[1])
            pt.point.z = float(centroid[2])

            publisher.publish(pt)

            self.get_logger().info(
                f"{label} cube -> ({centroid[0]:+.3f}, {centroid[1]:+.3f}, {centroid[2]:+.3f})"
            )

    def _get_clusters(self, xyz):

        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(xyz)

        labels = np.array(
            pcd.cluster_dbscan(
                eps=DBSCAN_EPS,
                min_points=DBSCAN_MIN_POINTS,
                print_progress=False
            )
        )

        clusters = []
        for i in range(labels.max() + 1):
            clusters.append(xyz[labels == i])

        return clusters

    def _is_cube_like(self, cluster):

        min_pt = cluster.min(axis=0)
        max_pt = cluster.max(axis=0)
        size = max_pt - min_pt

        x, y, z = size

        # Reject long objects (wires)
        if max(x, y, z) > 0.06:
            return False

        # Reject flat/thin
        if min(x, y, z) < 0.015:
            return False

        # Cube size constraint (3 cm cube)
        expected = 0.03
        tol = 0.015

        if not (expected - tol < x < expected + tol):
            return False
        if not (expected - tol < y < expected + tol):
            return False
        if not (expected - tol < z < expected + tol):  
            return False

        return True

    # -----------------------------
    # UTILITIES
    # -----------------------------
    @staticmethod
    def _extract_xyz_rgb(pc):
        if isinstance(pc, np.ndarray) and pc.dtype.names:
            xyz = np.stack([pc['x'], pc['y'], pc['z']], axis=-1).astype(np.float32)
            rgb_float = np.ascontiguousarray(pc['rgb']).astype(np.float32)
            return xyz, rgb_float

        points, colors = [], []
        for p in pc:
            points.append([p[0], p[1], p[2]])
            colors.append(p[3])

        if not points:
            return np.empty((0, 3), dtype=np.float32), np.empty((0,), dtype=np.float32)

        return (
            np.asarray(points, dtype=np.float32),
            np.asarray(colors, dtype=np.float32),
        )

    @staticmethod
    def _unpack_rgb(rgb_float):
        rgb_uint = rgb_float.view(np.uint32)

        r = ((rgb_uint >> 16) & 0xFF).astype(np.uint8)
        g = ((rgb_uint >> 8) & 0xFF).astype(np.uint8)
        b = (rgb_uint & 0xFF).astype(np.uint8)

        return np.stack([r, g, b], axis=-1)


def main():
    rclpy.init()
    node = CubeTracker()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.vis.destroy_window()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()