#!/usr/bin/env python3
"""Ground-relative visualization from existing wheel TF and measured attitude."""

from collections import deque
from pathlib import Path
import struct
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import TransformStamped
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import Imu
from tf2_ros import Buffer, TransformBroadcaster, TransformException, TransformListener


def quaternion_from_rpy(roll, pitch, yaw):
    sr, sp, sy = np.sin(np.array([roll, pitch, yaw]) / 2)
    cr, cp, cy = np.cos(np.array([roll, pitch, yaw]) / 2)
    return np.array([sr*cp*cy - cr*sp*sy, cr*sp*cy + sr*cp*sy,
                     cr*cp*sy - sr*sp*cy, cr*cp*cy + sr*sp*sy])


def quaternion_multiply(a, b):
    return np.r_[a[3]*b[:3] + b[3]*a[:3] + np.cross(a[:3], b[:3]),
                 a[3]*b[3] - np.dot(a[:3], b[:3])]


def rotation_matrix(quaternion):
    q = np.asarray(quaternion, dtype=float)
    norm = np.linalg.norm(q)
    if q.shape != (4,) or not np.all(np.isfinite(q)) or not np.isfinite(norm) or norm < 1e-6:
        raise ValueError('invalid orientation quaternion')
    x, y, z, w = q / norm
    return np.array([
        [1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
        [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
        [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)],
    ])


def body_tilt(imu_quaternion, imu_frame_rpy):
    """Remove heading, not tilt. Mount RPY is the reported IMU frame in base."""
    q = np.asarray(imu_quaternion, dtype=float)
    rotation_matrix(q)
    mount = quaternion_from_rpy(*imu_frame_rpy)
    # world<-base = world<-imu * inverse(base<-imu).
    q = quaternion_multiply(q / np.linalg.norm(q), mount * [-1, -1, -1, 1])
    rotation = rotation_matrix(q)
    yaw = np.arctan2(rotation[1, 0], rotation[0, 0])
    return quaternion_multiply(quaternion_from_rpy(0, 0, -yaw), q)


def load_wheel_vertices(sdf_path, wheel_frames):
    """Read visual geometry only; linkage transforms still come from the MCU/RSP."""
    model = ET.parse(sdf_path).getroot().find('model')
    wheels = []
    for frame in wheel_frames:
        link = model.find(f"link[@name='{frame}']")
        if link is None:
            raise ValueError(f'wheel link {frame!r} is absent from the model')
        parts = []
        for visual in link.findall('visual'):
            mesh = visual.find('geometry/mesh')
            if mesh is None:
                raise ValueError(f'{frame}: only STL mesh visuals are supported')
            uri = mesh.findtext('uri', '')
            if not uri.startswith(('package://', 'model://')):
                raise ValueError(f'unsupported wheel mesh URI: {uri}')
            package, relative = uri.split('://', 1)[1].split('/', 1)
            mesh_path = Path(get_package_share_directory(package)) / relative
            data = mesh_path.read_bytes()
            if len(data) < 84:
                raise ValueError(f'invalid binary STL: {mesh_path}')
            count = struct.unpack_from('<I', data, 80)[0]
            if count == 0 or len(data) != 84 + count * 50:
                raise ValueError(f'invalid binary STL: {mesh_path}')
            vertices = np.ndarray((count, 3, 3), dtype='<f4', buffer=data,
                                  offset=96, strides=(50, 12, 4)).reshape(-1, 3).astype(float)
            scale = np.fromstring(mesh.findtext('scale', '1 1 1'), sep=' ')
            pose_element = visual.find('pose')
            pose = np.fromstring(visual.findtext('pose', '0 0 0 0 0 0'), sep=' ')
            if (pose.shape != (6,) or scale.shape != (3,)
                    or (pose_element is not None
                        and pose_element.get('relative_to', frame) != frame)):
                raise ValueError(f'unsupported visual frame or scale for {frame}')
            vertices = (vertices * scale) @ rotation_matrix(quaternion_from_rpy(*pose[3:])).T
            parts.append(vertices + pose[:3])
        if not parts:
            raise ValueError(f'{frame} has no wheel visuals')
        vertices = np.unique(np.concatenate(parts), axis=0)
        if not np.all(np.isfinite(vertices)):
            raise ValueError(f'{frame} has nonfinite wheel geometry')
        wheels.append(vertices)
    return wheels


def ground_translation(wheel_poses, wheel_vertices, orientation):
    """Place the axle midpoint over the origin; never push a wheel into z=0."""
    rotation = rotation_matrix(orientation)
    centers, lowest = [], []
    for (translation, quaternion), vertices in zip(wheel_poses, wheel_vertices):
        center = rotation @ np.asarray(translation, dtype=float)
        local_up = rotation[2] @ rotation_matrix(quaternion)
        centers.append(center)
        lowest.append(float(center[2] + np.min(vertices @ local_up)))
    if len(centers) != 2 or not np.all(np.isfinite(centers)):
        raise ValueError('two finite wheel poses are required')
    midpoint = np.mean(centers, axis=0)
    return np.array([-midpoint[0], -midpoint[1], -min(lowest)]), abs(lowest[0] - lowest[1])


class BodyPose(Node):
    def __init__(self):
        super().__init__('body_pose')
        self.base = self.declare_parameter('base_frame', 'base').value
        self.ground = self.declare_parameter('ground_frame', 'base_footprint').value
        self.wheels = self.declare_parameter('wheel_frames', ['wheel', 'wheel_2']).value
        self.use_imu = self.declare_parameter('use_imu', True).value
        self.imu_frame = self.declare_parameter('imu_frame', 'base_link').value
        self.mount_rpy = self.declare_parameter('imu_frame_rpy', [0.0, 0.0, 0.0]).value
        self.max_age = self.declare_parameter('max_age_s', 1.0).value
        self.max_skew = self.declare_parameter('max_imu_skew_s', 0.1).value
        self.contact_tolerance = self.declare_parameter('contact_tolerance_m', 0.015).value
        if (len(self.wheels) != 2 or len(set(self.wheels)) != 2
                or self.ground == self.base or len(self.mount_rpy) != 3
                or not np.all(np.isfinite(self.mount_rpy))
                or not np.isfinite(self.max_age) or self.max_age <= 0
                or not np.isfinite(self.max_skew) or self.max_skew < 0
                or not np.isfinite(self.contact_tolerance) or self.contact_tolerance < 0):
            raise ValueError('invalid body pose frames, mounting, or freshness parameters')
        sdf = Path(get_package_share_directory('cyberbot_description')) / 'sdf' / 'robot.sdf'
        self.vertices = load_wheel_vertices(sdf, self.wheels)
        self.buffer = Buffer(node=self)
        self.listener = TransformListener(self.buffer, self)
        self.broadcaster = TransformBroadcaster(self)
        self.imu_samples = deque(maxlen=256)
        self.last_stamp = None
        if self.use_imu:
            self.imu_subscription = self.create_subscription(
                Imu, '/imu/data', self.on_imu, qos_profile_sensor_data)
            self.get_logger().warning(
                f'Using IMU frame {self.imu_frame} with imu_frame_rpy={self.mount_rpy}. '
                'Requires calibrated body axes; no automatic calibration or tilt zeroing.')
        self.timer = self.create_timer(0.05, self.update)

    def on_imu(self, message):
        if message.header.frame_id != self.imu_frame:
            self.get_logger().warning('Ignoring IMU with unexpected frame_id', throttle_duration_sec=5.0)
            return
        if message.orientation_covariance[0] == -1:
            return
        q = message.orientation
        try:
            orientation = body_tilt([q.x, q.y, q.z, q.w], self.mount_rpy)
        except ValueError:
            return
        stamp = Time.from_msg(message.header.stamp).nanoseconds
        if stamp > 0:
            self.imu_samples.append((stamp, orientation))

    def update(self):
        try:
            # Look up only the articulated subtree, never our own ground frame.
            latest = [self.buffer.lookup_transform(self.base, frame, Time()) for frame in self.wheels]
            stamp = min(Time.from_msg(tf.header.stamp).nanoseconds for tf in latest)
            if self.use_imu:
                if not self.imu_samples:
                    raise ValueError('waiting for valid calibrated IMU orientation')
                stamp = min(stamp, max(sample[0] for sample in self.imu_samples))
                imu_stamp, orientation = min(self.imu_samples, key=lambda sample: abs(sample[0] - stamp))
                if abs(imu_stamp - stamp) > self.max_skew * 1e9:
                    raise ValueError('IMU and wheel TF timestamps do not match')
            else:
                orientation = np.array([0.0, 0.0, 0.0, 1.0])
            age = self.get_clock().now().nanoseconds - stamp
            if stamp <= 0 or age < 0 or age > self.max_age * 1e9:
                raise ValueError('wheel TF or IMU is stale or future-dated')
            if stamp == self.last_stamp:
                return
            transforms = [self.buffer.lookup_transform(self.base, frame, Time(nanoseconds=stamp))
                          for frame in self.wheels]
            poses = []
            for transform in transforms:
                p, q = transform.transform.translation, transform.transform.rotation
                poses.append(([p.x, p.y, p.z], [q.x, q.y, q.z, q.w]))
            translation, mismatch = ground_translation(poses, self.vertices, orientation)
            transform = TransformStamped()
            transform.header.stamp = Time(nanoseconds=stamp).to_msg()
            transform.header.frame_id = self.ground
            transform.child_frame_id = self.base
            transform.transform.translation.x = float(translation[0])
            transform.transform.translation.y = float(translation[1])
            transform.transform.translation.z = float(translation[2])
            q = transform.transform.rotation
            q.x, q.y, q.z, q.w = map(float, orientation)
            self.broadcaster.sendTransform(transform)
            self.last_stamp = stamp
            if mismatch > self.contact_tolerance:
                self.get_logger().warning(
                    f'Wheel contact mismatch {mismatch:.3f} m: flat ground, IMU alignment '
                    'and joint geometry are inconsistent; keeping both wheels above ground.',
                    throttle_duration_sec=5.0)
        except (TransformException, ValueError) as error:
            self.get_logger().warning(f'No fresh ground pose: {error}', throttle_duration_sec=5.0)


def main():
    rclpy.init()
    node = None
    try:
        node = BodyPose()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
