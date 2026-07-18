import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
import tf2_ros
import tf2_geometry_msgs
from rclpy.duration import Duration


# Cube-routing threshold, evaluated in right_base_link.
# y > LEFT_THRESHOLD → left arm; otherwise → right arm (covers middle + right).
LEFT_THRESHOLD = 0.15


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

        self.pub_cube_right = self.create_publisher(
            PointStamped,
            '/cube_right_base',
            10
        )
        self.pub_cube_left = self.create_publisher(
            PointStamped,
            '/cube_left_base',
            10
        )
        self.get_logger().info(
            f"Cube TF router started (LEFT_THRESHOLD y={LEFT_THRESHOLD} in right_base_link)"
        )

    # --------------------------------------------------
    def transform_point(self, msg, target_frame):

        try:
            transform = self.tf_buffer.lookup_transform(
                target_frame,
                msg.header.frame_id,
                rclpy.time.Time(),
                timeout=Duration(seconds=1.0)
            )
            point_out = tf2_geometry_msgs.do_transform_point(msg, transform)
            point_out.header.frame_id = target_frame
            return point_out

        except Exception as e:
            self.get_logger().warn(f"TF to {target_frame} failed: {str(e)}")
            return None

    # --------------------------------------------------
    def cube_callback(self, msg):
        pt_right = self.transform_point(msg, 'right_base_link')
        if pt_right is None:
            return

        if pt_right.point.y > LEFT_THRESHOLD:
            pt_left = self.transform_point(msg, 'left_base_link')
            if pt_left is None:
                return
            self.pub_cube_left.publish(pt_left)
        else:
            self.pub_cube_right.publish(pt_right)

def main():
    rclpy.init()

    node = CubeTF()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()