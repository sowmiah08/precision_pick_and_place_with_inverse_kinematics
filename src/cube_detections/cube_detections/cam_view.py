import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from cv_bridge import CvBridge

import cv2


class ColorViewer(Node):

    def __init__(self):
        super().__init__('color_viewer')

        self.subscription = self.create_subscription(
            Image,
            '/camera/camera/color/image_raw',
            self.callback,
            10
        )

        self.bridge = CvBridge()

        cv2.namedWindow("Color Camera", cv2.WINDOW_NORMAL)

        self.get_logger().info("Color camera viewer started")

    def callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"Conversion failed: {e}")
            return

        cv2.imshow("Color Camera", frame)
        cv2.waitKey(1)


def main():
    rclpy.init()
    node = ColorViewer()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    cv2.destroyAllWindows()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()