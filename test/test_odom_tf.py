"""Exercise the source node without an executor or the robot's ROS domain."""

import math
from pathlib import Path
import runpy
from types import SimpleNamespace

from nav_msgs.msg import Odometry
import pytest
import rclpy
from rclpy.qos import ReliabilityPolicy
from rclpy.time import Time


MODULE = runpy.run_path(str(Path(__file__).resolve().parents[1] / 'scripts' / 'odom_tf.py'))


@pytest.fixture
def node_factory(monkeypatch):
    """Create actual nodes in an isolated domain with a recorder instead of TF."""
    monkeypatch.setenv('ROS_DOMAIN_ID', '191')
    nodes = []

    def create(parameters=()):
        args = ['--ros-args']
        for parameter in parameters:
            args.extend(['-p', parameter])
        rclpy.init(args=args)
        instance = MODULE['OdomTF']()
        nodes.append(instance)
        sent = []
        instance.broadcaster = SimpleNamespace(sendTransform=sent.append)
        now = SimpleNamespace(nanoseconds=10_000_000_000)
        monkeypatch.setattr(instance, 'get_clock', lambda: SimpleNamespace(now=lambda: now))
        return instance, sent, now

    try:
        yield create
    finally:
        for instance in nodes:
            instance.destroy_node()
        rclpy.try_shutdown()


@pytest.fixture
def node(node_factory):
    """Use default frames with a deterministic clock."""
    return node_factory()


def odometry(stamp=9_900_000_000):
    """Make a non-unit yaw quaternion to verify normalization, not integration."""
    message = Odometry()
    message.header.frame_id = 'odom'
    message.child_frame_id = 'base_footprint'
    message.header.stamp = Time(nanoseconds=stamp).to_msg()
    message.pose.pose.position.x = 1.25
    message.pose.pose.position.y = -2.5
    message.pose.pose.orientation.z = 3 * math.sin(0.6)
    message.pose.pose.orientation.w = 3 * math.cos(0.6)
    return message


def test_copies_pose_and_stamp(node):
    """Publish exactly one measured transform with normalized yaw and reliable input."""
    instance, sent, _ = node
    message = odometry()
    instance.on_message(message)
    assert len(sent) == 1
    transform = sent[0]
    assert transform.header == message.header
    assert transform.child_frame_id == message.child_frame_id
    p = transform.transform.translation
    assert (p.x, p.y, p.z) == (1.25, -2.5, 0.0)
    q = transform.transform.rotation
    assert (q.x, q.y, q.z, q.w) == pytest.approx((0, 0, math.sin(0.6), math.cos(0.6)))
    assert message.pose.pose.orientation.z == 3 * math.sin(0.6)
    assert instance.subscription.qos_profile.reliability == ReliabilityPolicy.RELIABLE
    assert instance.subscription.qos_profile.depth == 10
    assert not list(instance.timers)


@pytest.mark.parametrize('field,value', [
    ('parent', 'map'), ('child', 'base_link'), ('child', ''),
    ('stamp', 0), ('stamp', 8_000_000_000), ('stamp', 10_000_000_001),
    ('zero_quaternion', 0), ('qx', math.nan), ('qw', math.inf),
    ('px', math.nan), ('py', math.inf), ('pz', math.nan),
    ('qx', 0.01), ('qy', 0.01), ('pz', 0.001),
])
def test_rejects_invalid_odometry(node, field, value):
    """Invalid frames, time, geometry and attitude never reach the broadcaster."""
    instance, sent, _ = node
    message = odometry()
    if field == 'parent':
        message.header.frame_id = value
    elif field == 'child':
        message.child_frame_id = value
    elif field == 'stamp':
        message.header.stamp = Time(nanoseconds=value).to_msg()
    elif field == 'zero_quaternion':
        message.pose.pose.orientation.z = message.pose.pose.orientation.w = 0.0
    else:
        target = message.pose.pose.position if field[0] == 'p' else message.pose.pose.orientation
        setattr(target, field[1], value)
    instance.on_message(message)
    assert not sent
    assert instance.last_stamp is None


def test_ordering_and_no_stale_restamp(node):
    """Duplicate/backward samples stay rejected; stale data never gets a new stamp."""
    instance, sent, now = node
    instance.on_message(odometry())
    instance.on_message(odometry())
    instance.on_message(odometry(9_800_000_000))
    now.nanoseconds += 2_000_000_000
    instance.on_message(odometry())
    assert len(sent) == 1
    assert Time.from_msg(sent[0].header.stamp).nanoseconds == 9_900_000_000
    instance.on_message(odometry(now.nanoseconds))
    assert len(sent) == 2


def test_backward_clock_resets_ordering(node):
    """A clock reset allows fresh earlier source stamps without changing them."""
    instance, sent, now = node
    instance.on_message(odometry())
    now.nanoseconds = 5_000_000_000
    instance.on_message(odometry(4_900_000_000))
    assert len(sent) == 2
    assert Time.from_msg(sent[-1].header.stamp).nanoseconds == 4_900_000_000


@pytest.mark.parametrize('parameters', [
    ['parent_frame:=""'], ['child_frame:=""'], ['parent_frame:=/odom'],
    ['child_frame:=bad frame'], ['child_frame:=odom'],
    ['max_age_s:=0.0'], ['max_age_s:=-1.0'],
    ['max_age_s:=.nan'], ['max_age_s:=.inf'],
])
def test_invalid_parameters(node_factory, parameters):
    """Reject unusable frame names and nonpositive freshness limits at startup."""
    with pytest.raises(ValueError):
        node_factory(parameters)


def test_configured_frames_and_age(node_factory):
    """Configured frame IDs and freshness limits apply to input and output."""
    instance, sent, _ = node_factory(
        ['parent_frame:=local_odom', 'child_frame:=footprint', 'max_age_s:=0.05'])
    message = odometry()
    message.header.frame_id = 'local_odom'
    message.child_frame_id = 'footprint'
    instance.on_message(message)
    assert not sent
    message.header.stamp = Time(nanoseconds=9_980_000_000).to_msg()
    instance.on_message(message)
    assert len(sent) == 1
    assert sent[0].header.frame_id == 'local_odom'
    assert sent[0].child_frame_id == 'footprint'
