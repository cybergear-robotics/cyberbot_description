from pathlib import Path
import runpy
from types import SimpleNamespace

from geometry_msgs.msg import TransformStamped
import numpy as np
import pytest
import rclpy
from sensor_msgs.msg import Imu


PACKAGE = Path(__file__).resolve().parents[1]
MODULE = runpy.run_path(str(PACKAGE / 'scripts' / 'body_pose.py'))
rpy = MODULE['quaternion_from_rpy']
matrix = MODULE['rotation_matrix']
multiply = MODULE['quaternion_multiply']
tilt = MODULE['body_tilt']
ground = MODULE['ground_translation']


@pytest.fixture(scope='module')
def wheels():
    return MODULE['load_wheel_vertices'](PACKAGE / 'sdf' / 'robot.sdf', ['wheel', 'wheel_2'])


@pytest.mark.parametrize('angles', [(0, 0, 0), (0.2, -0.3, 1.2), (-0.4, 0.1, -2.5)])
@pytest.mark.parametrize('mount', [(0, 0, 0), (0.1, -0.2, 1.57), (0, 0, np.pi)])
def test_tilt_and_mounting(angles, mount):
    measured = multiply(rpy(*angles), rpy(*mount))
    np.testing.assert_allclose(matrix(tilt(measured, mount)), matrix(rpy(*angles[:2], 0)), atol=1e-12)
    np.testing.assert_allclose(matrix(tilt(measured * 2, mount)), matrix(rpy(*angles[:2], 0)), atol=1e-12)


@pytest.mark.parametrize('q', [[0, 0, 0, 0], [np.nan, 0, 0, 1], [0, np.inf, 0, 1], [1, 2, 3]])
def test_invalid_orientation(q):
    with pytest.raises(ValueError):
        matrix(q)


def test_visual_geometry_not_rolling_radius(wheels):
    for vertices in wheels:
        assert abs(np.max(np.hypot(vertices[:, 0], vertices[:, 1])) - 0.091965) < 2e-6
        # Hubs and tires have their visual centers 6 mm along the local axle.
        assert abs((vertices[:, 2].max() + vertices[:, 2].min()) / 2 + 0.006) < 2e-6


@pytest.mark.parametrize('angles', [(0, 0, 0), (0.2, -0.25, 0), (-0.4, 0.3, 0)])
def test_contact_uses_full_mesh_and_attitude(wheels, angles):
    poses = [([0.03, -0.19, -0.06], rpy(np.pi/2, 0, 0.7)),
             ([-0.02, 0.18, -0.08], rpy(-np.pi/2, 0, -1.1))]
    q = rpy(*angles)
    translation, mismatch = ground(poses, wheels, q)
    lows = []
    for (center, wheel_q), vertices in zip(poses, wheels):
        transformed = (vertices @ matrix(wheel_q).T + center) @ matrix(q).T + translation
        lows.append(transformed[:, 2].min())
    assert min(lows) == pytest.approx(0, abs=1e-12)
    assert all(low >= -1e-12 for low in lows)
    assert mismatch == pytest.approx(abs(lows[0] - lows[1]))
    midpoint = matrix(q) @ np.mean([pose[0] for pose in poses], axis=0) + translation
    np.testing.assert_allclose(midpoint[:2], 0, atol=1e-12)


def test_level_symmetric_wheels_touch(wheels):
    poses = [([0, -0.19, -0.0163], rpy(np.pi/2, 0, 0)),
             ([0, 0.18, -0.0163], rpy(-np.pi/2, 0, 0))]
    translation, mismatch = ground(poses, wheels, rpy(0, 0, 0))
    assert translation[2] == pytest.approx(0.0163 + 0.091965, abs=2e-6)
    assert mismatch < 2e-6


@pytest.fixture
def node(monkeypatch):
    # Unit tests never join the robot's ROS domain or run an executor.
    monkeypatch.setenv('ROS_DOMAIN_ID', '191')
    rclpy.init(args=[])
    instance = MODULE['BodyPose']()
    sent = []
    instance.broadcaster = SimpleNamespace(sendTransform=sent.append)
    try:
        yield instance, sent
    finally:
        instance.destroy_node()
        rclpy.shutdown()


def imu(stamp, angles=(0, 0, 0)):
    message = Imu()
    message.header.frame_id = 'base_link'
    message.header.stamp = MODULE['Time'](nanoseconds=stamp).to_msg()
    q = message.orientation
    q.x, q.y, q.z, q.w = map(float, rpy(*angles))
    return message


def wheel_buffer(stamp, latest_offset=0):
    lookups = []

    def lookup(parent, child, time):
        assert parent == 'base' and child in ('wheel', 'wheel_2')
        lookups.append((child, time.nanoseconds))
        transform = TransformStamped()
        transform.header.frame_id = parent
        transform.child_frame_id = child
        own_stamp = stamp + (latest_offset if child == 'wheel' else 0)
        transform.header.stamp = MODULE['Time'](nanoseconds=time.nanoseconds or own_stamp).to_msg()
        transform.transform.translation.y = -0.19 if child == 'wheel' else 0.18
        transform.transform.translation.z = -0.02
        q = transform.transform.rotation
        q.x, q.y, q.z, q.w = map(float, rpy(np.pi/2 if child == 'wheel' else -np.pi/2, 0, 0))
        return transform

    return SimpleNamespace(lookup_transform=lookup), lookups


def test_node_common_time_no_tf_cycle_and_no_restamping(node):
    instance, sent = node
    stamp = instance.get_clock().now().nanoseconds - 100_000_000
    instance.buffer, lookups = wheel_buffer(stamp, latest_offset=20_000_000)
    instance.on_imu(imu(stamp, (0.05, -0.1, 0.8)))
    instance.update()
    assert len(sent) == 1
    transform = sent[0]
    assert transform.header.frame_id == 'base_footprint'
    assert transform.child_frame_id == 'base'
    assert MODULE['Time'].from_msg(transform.header.stamp).nanoseconds == stamp
    assert lookups[-2:] == [('wheel', stamp), ('wheel_2', stamp)]
    instance.update()
    assert len(sent) == 1


@pytest.mark.parametrize('fault', ['missing_imu', 'stale_wheel', 'stale_imu', 'future_wheel', 'skew'])
def test_node_rejects_missing_stale_and_skewed_inputs(node, fault):
    instance, sent = node
    stamp = instance.get_clock().now().nanoseconds - 100_000_000
    wheel_stamp = stamp - 2_000_000_000 if fault == 'stale_wheel' else stamp
    if fault == 'future_wheel':
        wheel_stamp += 3_000_000_000
    instance.buffer, _ = wheel_buffer(wheel_stamp)
    if fault != 'missing_imu':
        imu_stamp = stamp - 2_000_000_000 if fault == 'stale_imu' else stamp
        if fault == 'skew':
            imu_stamp += 300_000_000
        if fault == 'future_wheel':
            imu_stamp = wheel_stamp
        instance.on_imu(imu(imu_stamp))
    instance.update()
    assert not sent


@pytest.mark.parametrize('fault', ['frame', 'quaternion', 'covariance', 'zero_stamp'])
def test_node_rejects_invalid_imu(node, fault):
    instance, _ = node
    message = imu(instance.get_clock().now().nanoseconds)
    if fault == 'frame':
        message.header.frame_id = 'unknown_sensor'
    elif fault == 'quaternion':
        message.orientation.w = np.nan
    elif fault == 'covariance':
        message.orientation_covariance[0] = -1.0
    else:
        message.header.stamp.sec = message.header.stamp.nanosec = 0
    instance.on_imu(message)
    assert not instance.imu_samples


def test_explicit_level_preview_needs_no_imu(node):
    instance, sent = node
    instance.use_imu = False
    stamp = instance.get_clock().now().nanoseconds - 100_000_000
    instance.buffer, _ = wheel_buffer(stamp)
    instance.update()
    assert len(sent) == 1
    assert sent[0].transform.rotation.w == 1
