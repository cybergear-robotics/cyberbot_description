#!/usr/bin/env python3
"""Publish measured planar odometry as odom -> base_footprint TF only."""

import math

from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster


def validated_transform(message, now_ns, parent, child, max_age_s):
    """Copy a fresh planar pose, allowing only 1e-6 m/rad numerical residue."""
    if message.header.frame_id != parent or message.child_frame_id != child:
        raise ValueError(
            f'expected frames {parent!r} -> {child!r}, got '
            f'{message.header.frame_id!r} -> {message.child_frame_id!r}; '
            'base_link is not an alias for base_footprint')
    stamp = message.header.stamp
    stamp_ns = stamp.sec * 1_000_000_000 + stamp.nanosec
    if stamp.sec < 0 or not 0 <= stamp.nanosec < 1_000_000_000 or stamp_ns <= 0:
        raise ValueError('odometry timestamp must be positive and valid')
    age_ns = now_ns - stamp_ns
    if age_ns < 0 or age_ns > max_age_s * 1e9:
        raise ValueError('odometry is stale or future-dated')
    p, q = message.pose.pose.position, message.pose.pose.orientation
    if not all(math.isfinite(v) for v in (p.x, p.y, p.z)):
        raise ValueError('odometry position must be finite')
    if not all(math.isfinite(v) for v in (q.x, q.y, q.z, q.w)):
        raise ValueError('odometry quaternion must be finite')
    norm = math.hypot(q.x, q.y, q.z, q.w)
    if norm == 0 or not math.isfinite(norm):
        raise ValueError('odometry quaternion must have a finite nonzero norm')
    x, y, z, w = (v / norm for v in (q.x, q.y, q.z, q.w))
    if abs(p.z) > 1e-6 or 2 * math.atan2(math.hypot(x, y), math.hypot(z, w)) > 1e-6:
        raise ValueError('odometry must be planar; height, roll and pitch belong to body_pose')
    transform = TransformStamped()
    transform.header.stamp = stamp
    transform.header.frame_id = parent
    transform.child_frame_id = child
    transform.transform.translation.x = p.x
    transform.transform.translation.y = p.y
    transform.transform.translation.z = p.z
    rotation = transform.transform.rotation
    rotation.x, rotation.y, rotation.z, rotation.w = x, y, z, w
    return transform


class OdomTF(Node):
    def __init__(self):
        super().__init__('odom_tf')
        self.parent = self.declare_parameter('parent_frame', 'odom').value
        self.child = self.declare_parameter('child_frame', 'base_footprint').value
        self.max_age = self.declare_parameter('max_age_s', 1.0).value
        if (any(not frame or frame.startswith('/') or any(c.isspace() for c in frame)
                for frame in (self.parent, self.child))
                or self.parent == self.child
                or not math.isfinite(self.max_age) or self.max_age <= 0):
            self.destroy_node()
            raise ValueError('invalid odometry TF frames or max_age_s (must be finite and positive)')
        self.last_stamp = None
        self.last_now = None
        self.broadcaster = TransformBroadcaster(self)
        self.subscription = self.create_subscription(Odometry, '/odom', self.on_message, 10)

    def on_message(self, message):
        now = self.get_clock().now().nanoseconds
        if self.last_now is not None and now < self.last_now:
            # A new ROS clock epoch permits older source stamps, never restamping.
            self.last_stamp = None
        self.last_now = now
        try:
            transform = validated_transform(message, now, self.parent, self.child, self.max_age)
            stamp = transform.header.stamp
            stamp_ns = stamp.sec * 1_000_000_000 + stamp.nanosec
            if self.last_stamp is not None and stamp_ns <= self.last_stamp:
                raise ValueError('duplicate or backward odometry timestamp')
        except ValueError as error:
            self.get_logger().warning(f'Ignoring odometry TF: {error}', throttle_duration_sec=5.0)
            return
        self.broadcaster.sendTransform(transform)
        self.last_stamp = stamp_ns


def main():
    rclpy.init()
    node = None
    try:
        node = OdomTF()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
