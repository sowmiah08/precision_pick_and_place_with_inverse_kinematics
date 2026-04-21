import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import PointStamped
from cv_bridge import CvBridge

import cv2
import numpy as np


class CubeTracker(Node):
    def __init__(self):
        super().__init__('cube_tracker')

        self.bridge = CvBridge()

        self.sub = self.create_subscription(
            Image,
            '/camera/side_camera/color/image_raw',
            self.callback,
            10
        )

        # continuous publish topic
        self.pub = self.create_publisher(PointStamped, '/cube_pixel', 10)

        # smoothing
        self.prev_x = None
        self.prev_y = None
        self.alpha = 0.7

        self.get_logger().info("Cube tracker started")

    def callback(self, msg):

        frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # ---------------- RED MASK ----------------
        lower_red1 = np.array([0, 120, 70])
        upper_red1 = np.array([10, 255, 255])

        lower_red2 = np.array([170, 120, 70])
        upper_red2 = np.array([180, 255, 255])

        red_mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
        red_mask2 = cv2.inRange(hsv, lower_red2, upper_red2)

        red_mask = red_mask1 + red_mask2

        # ---------------- BLUE MASK ----------------
        lower_blue = np.array([100, 150, 50])
        upper_blue = np.array([140, 255, 255])

        blue_mask = cv2.inRange(hsv, lower_blue, upper_blue)

        # combine both colors
        mask = red_mask + blue_mask

        # remove noise
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

        contours, _ = cv2.findContours(
            mask,
            cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE
        )

        if contours:

            # largest cube
            contour = max(contours, key=cv2.contourArea)

            area = cv2.contourArea(contour)

            if area > 500:

                x, y, w, h = cv2.boundingRect(contour)

                cx = x + w / 2.0
                cy = y + h / 2.0

                # smoothing
                if self.prev_x is not None:
                    cx = self.alpha * self.prev_x + (1 - self.alpha) * cx
                    cy = self.alpha * self.prev_y + (1 - self.alpha) * cy

                self.prev_x = cx
                self.prev_y = cy

                # draw
                cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
                cv2.circle(frame, (int(cx), int(cy)), 8, (0, 0, 255), -1)

                cv2.putText(
                    frame,
                    f"({int(cx)}, {int(cy)})",
                    (x, y - 10),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.6,
                    (255, 255, 255),
                    2
                )

                pt = PointStamped()
                pt.header.stamp = self.get_clock().now().to_msg()
                pt.header.frame_id = "camera_color_optical_frame"

                pt.point.x = float(cx)
                pt.point.y = float(cy)
                pt.point.z = 0.0

                self.pub.publish(pt)

        cv2.imshow("Cube Tracking", frame)
        cv2.waitKey(1)


# ---------------------------------------------------
def main():
    rclpy.init()
    node = CubeTracker()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    cv2.destroyAllWindows()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()