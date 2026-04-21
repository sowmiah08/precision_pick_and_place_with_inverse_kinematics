import math

import numpy as np
import rclpy
from rclpy.node import Node
from apriltag_msgs.msg import AprilTagDetectionArray
from geometry_msgs.msg import TransformStamped
import tf2_ros


def rpy_to_quat(roll, pitch, yaw):
    cr, sr = math.cos(roll / 2.0), math.sin(roll / 2.0)
    cp, sp = math.cos(pitch / 2.0), math.sin(pitch / 2.0)
    cy, sy = math.cos(yaw / 2.0), math.sin(yaw / 2.0)
    return np.array([
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    ])


def quat_to_mat(q):
    x, y, z, w = q
    xx, yy, zz = x * x, y * y, z * z
    xy, xz, yz = x * y, x * z, y * z
    wx, wy, wz = w * x, w * y, w * z
    M = np.eye(4)
    M[0, 0] = 1 - 2 * (yy + zz)
    M[0, 1] = 2 * (xy - wz)
    M[0, 2] = 2 * (xz + wy)
    M[1, 0] = 2 * (xy + wz)
    M[1, 1] = 1 - 2 * (xx + zz)
    M[1, 2] = 2 * (yz - wx)
    M[2, 0] = 2 * (xz - wy)
    M[2, 1] = 2 * (yz + wx)
    M[2, 2] = 1 - 2 * (xx + yy)
    return M


def mat_to_quat(M):
    R = M[:3, :3]
    t = np.trace(R)
    if t > 0:
        s = math.sqrt(t + 1.0) * 2
        w = 0.25 * s
        x = (R[2, 1] - R[1, 2]) / s
        y = (R[0, 2] - R[2, 0]) / s
        z = (R[1, 0] - R[0, 1]) / s
    elif (R[0, 0] > R[1, 1]) and (R[0, 0] > R[2, 2]):
        s = math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
        w = (R[2, 1] - R[1, 2]) / s
        x = 0.25 * s
        y = (R[0, 1] + R[1, 0]) / s
        z = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
        w = (R[0, 2] - R[2, 0]) / s
        x = (R[0, 1] + R[1, 0]) / s
        y = 0.25 * s
        z = (R[1, 2] + R[2, 1]) / s
    else:
        s = math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
        w = (R[1, 0] - R[0, 1]) / s
        x = (R[0, 2] + R[2, 0]) / s
        y = (R[1, 2] + R[2, 1]) / s
        z = 0.25 * s
    return np.array([x, y, z, w])


class TagToTF(Node):
    #Detect an AprilTag to calibrate the camera-to-robot transform, then publish 
    #it as a static transform so it persists even when the robot arm blocks the tag.

    def __init__(self):
        super().__init__('tag_to_tf')

        # Pose of base_link expressed in the tag frame.
        self.declare_parameter('tag_to_base_xyz', [0.0, -0.08, 0.0])
        self.declare_parameter('tag_to_base_rpy', [0.0, 0.0, 0.0])
        self.declare_parameter('calibration_samples', 30)

        xyz = list(self.get_parameter('tag_to_base_xyz').value)
        rpy = list(self.get_parameter('tag_to_base_rpy').value)
        self.n_target = int(self.get_parameter('calibration_samples').value)

        self.T_tag_base = quat_to_mat(rpy_to_quat(*rpy))
        self.T_tag_base[:3, 3] = xyz

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.static_br = tf2_ros.StaticTransformBroadcaster(self)

        self._translations = []    
        self._quats = []           
        self._ref_q = None         
        self._calibrated = False
        self._logged_first = False
        self._lookup_fail_count = 0

        self.sub = self.create_subscription(
            AprilTagDetectionArray, '/detections', self.callback, 10)

        self.get_logger().info(
            f'Calibrating camera->base_link from AprilTag '
            f'({self.n_target} samples)...')

    def _candidate_tag_frames(self, detection):
        # Different apriltag_ros versions use different conventions.(works for all tag fam)
        family = detection.family
        tid = detection.id
        return [
            f'tag{family}:{tid}',           # e.g. tag16h5:0
            f'tag{family}_{tid}',           # e.g. tag16h5_0
            f'{family}:{tid}',              # e.g. 16h5:0
            f'tag_{tid}',                   # e.g. tag_0
            f'tag{tid}',                    # e.g. tag0
            str(tid),                       # e.g. 0
        ]

    def callback(self, msg):
        if self._calibrated or not msg.detections:
            return

        detection = msg.detections[0]

        if not self._logged_first:
            self._logged_first = True
            self.get_logger().info(
                f'First detection: family="{detection.family}" id={detection.id}. '
                f'Known TF frames:\n{self.tf_buffer.all_frames_as_string()}')

        tag_frame = None
        tf = None
        for candidate in self._candidate_tag_frames(detection):
            try:
                tf = self.tf_buffer.lookup_transform(
                    'camera_color_optical_frame',
                    candidate,
                    rclpy.time.Time(),
                    timeout=rclpy.duration.Duration(seconds=0.05),
                )
                tag_frame = candidate
                break
            except tf2_ros.TransformException:
                continue

        if tf is None:
            self._lookup_fail_count += 1
            if self._lookup_fail_count % 10 == 1:
                self.get_logger().warn(
                    f'No tag TF found. Tried: '
                    f'{self._candidate_tag_frames(detection)}. '
                    f'Available frames:\n{self.tf_buffer.all_frames_as_string()}')
            return

        if tag_frame and not hasattr(self, '_logged_frame'):
            self._logged_frame = tag_frame
            self.get_logger().info(f'Using tag frame: "{tag_frame}"')

        tt = tf.transform.translation
        rr = tf.transform.rotation
        T_cam_tag = quat_to_mat(np.array([rr.x, rr.y, rr.z, rr.w]))
        T_cam_tag[:3, 3] = [tt.x, tt.y, tt.z]

        T_cam_base = T_cam_tag @ self.T_tag_base
        t_vec = T_cam_base[:3, 3]
        q_vec = mat_to_quat(T_cam_base)

        if self._ref_q is None:
            self._ref_q = q_vec
        elif np.dot(q_vec, self._ref_q) < 0:
            q_vec = -q_vec 

        self._translations.append(t_vec)
        self._quats.append(q_vec)

        n = len(self._translations)
        if n % 5 == 0:
            self.get_logger().info(f'  calibration sample {n}/{self.n_target}')

        if n >= self.n_target:
            self._publish_static()

    def _publish_static(self):
        t_mean = np.mean(np.stack(self._translations, axis=0), axis=0)
        q_sum = np.sum(np.stack(self._quats, axis=0), axis=0)
        q_mean = q_sum / np.linalg.norm(q_sum)

        out = TransformStamped()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = 'camera_color_optical_frame'
        out.child_frame_id = 'base_link'
        out.transform.translation.x = float(t_mean[0])
        out.transform.translation.y = float(t_mean[1])
        out.transform.translation.z = float(t_mean[2])
        out.transform.rotation.x = float(q_mean[0])
        out.transform.rotation.y = float(q_mean[1])
        out.transform.rotation.z = float(q_mean[2])
        out.transform.rotation.w = float(q_mean[3])

        self.static_br.sendTransform(out)
        self._calibrated = True

        self.get_logger().info(
            f'Calibration locked. Published STATIC '
            f'camera_color_optical_frame -> base_link: '
            f't=[{t_mean[0]:.3f}, {t_mean[1]:.3f}, {t_mean[2]:.3f}] '
            f'q=[{q_mean[0]:.3f}, {q_mean[1]:.3f}, {q_mean[2]:.3f}, {q_mean[3]:.3f}]. '
            f'Tag occlusion is now harmless.')


def main():
    rclpy.init()
    node = TagToTF()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
