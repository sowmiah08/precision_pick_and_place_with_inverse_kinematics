# pointcloud_view.py
# ROS2 node to visualize RealSense point cloud in Open3D

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2

import numpy as np
import open3d as o3d


class PointCloudViewer(Node):

    def __init__(self):
        super().__init__('pointcloud_viewer')

        self.sub = self.create_subscription(
            PointCloud2,
            '/camera/camera/depth/color/points',
            self.callback,
            10
        )

        self.vis = o3d.visualization.Visualizer()
        self.vis.create_window(window_name='ROS2 Point Cloud')

        self.pcd = o3d.geometry.PointCloud()
        self.vis.add_geometry(self.pcd)

        self.first = True

        self.get_logger().info("Point cloud viewer started")

    def callback(self, msg):

        points = []

        for p in point_cloud2.read_points(
            msg,
            field_names=("x", "y", "z"),
            skip_nans=True
        ):
            points.append([p[0], p[1], p[2]])

        if len(points) == 0:
            return

        xyz = np.array(points, dtype=np.float32)

        self.pcd.points = o3d.utility.Vector3dVector(xyz)

        if self.first:
            self.vis.reset_view_point(True)
            self.first = False

        self.vis.update_geometry(self.pcd)
        self.vis.poll_events()
        self.vis.update_renderer()

def main(args=None):
    rclpy.init(args=args)

    node = PointCloudViewer()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.vis.destroy_window()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()