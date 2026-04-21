import rclpy
from rclpy.node import Node

from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from geometry_msgs.msg import PointStamped

import cv2
import numpy as np
import open3d as o3d


MIN_CLUSTER_POINTS = 50


class CubeTracker(Node):

    def __init__(self):
        super().__init__('cube_tracker')

        self.sub = self.create_subscription(
            PointCloud2,
            '/camera/camera/depth/color/points',
            self.callback,
            10,
        )

        # Single publisher
        self.pub_cube = self.create_publisher(PointStamped, '/cube_3d', 10)

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

        # Publish whichever cube is detected
        self._publish_centroid(xyz, red_mask, msg.header, 'red')
        self._publish_centroid(xyz, blue_mask, msg.header, 'blue')

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

    def _publish_centroid(self, xyz, mask, header, label):
        n = int(mask.sum())

        if n < MIN_CLUSTER_POINTS:
            return

        centroid = xyz[mask].mean(axis=0)

        pt = PointStamped()
        pt.header = header
        pt.point.x = float(centroid[0])
        pt.point.y = float(centroid[1])
        pt.point.z = float(centroid[2])

        self.pub_cube.publish(pt)

        self.get_logger().info(
            f"{label} cube -> /cube_3d @ "
            f"({centroid[0]:+.3f}, {centroid[1]:+.3f}, {centroid[2]:+.3f})"
        )

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