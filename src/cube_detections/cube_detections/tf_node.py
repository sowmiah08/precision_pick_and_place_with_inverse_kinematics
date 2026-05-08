import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
import tf2_ros
import tf2_geometry_msgs
from rclpy.duration import Duration


class CubeTF(Node):

    def __init__(self):
        super().__init__('cube_tf_node')

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.sub_red = self.create_subscription(
            PointStamped,
            '/cube_red_3d',
            self.cube_callback,
            10
        )
        self.sub_blue = self.create_subscription(
            PointStamped,
            '/cube_blue_3d',
            self.cube_callback,
            10
        )

        self.pub_cube = self.create_publisher(
            PointStamped,
            '/cube_base',
            10
        )
        self.get_logger().info("Cube TF node started")

    # --------------------------------------------------
    def transform_point(self, msg):

        try:
            transform = self.tf_buffer.lookup_transform(
                'right_base_link',
                msg.header.frame_id,
                rclpy.time.Time(),
                timeout=Duration(seconds=1.0)
            )
            point_out = tf2_geometry_msgs.do_transform_point(msg, transform)
            point_out.header.frame_id = 'right_base_link'
            return point_out

        except Exception as e:
            self.get_logger().warn(f"TF failed: {str(e)}")
            return None

    # --------------------------------------------------
    def cube_callback(self, msg):
        pt = self.transform_point(msg)
        if pt:
            self.pub_cube.publish(pt)

def main():
    rclpy.init()

    node = CubeTF()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()