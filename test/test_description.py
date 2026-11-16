from pathlib import Path
import runpy
import xml.etree.ElementTree as ET

import numpy as np
import pytest
from launch import LaunchContext
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration


PACKAGE = Path(__file__).resolve().parents[1]


def test_ros_description_meshes():
    launch = runpy.run_path(str(PACKAGE / 'launch' / 'description.launch.py'))
    description = launch['load_robot_description']()
    meshes = ET.fromstring(description).findall('.//mesh/uri')
    assert meshes
    for mesh in meshes:
        assert mesh.text.startswith('package://cyberbot_description/')
        assert (PACKAGE / mesh.text.removeprefix('package://cyberbot_description/')).is_file()


def test_sdf_joint_limits():
    model = ET.parse(PACKAGE / 'sdf' / 'robot.sdf').getroot().find('model')
    joints = model.findall('joint')
    assert len(joints) == 10
    for joint in joints:
        axis = joint.find('axis')
        assert axis.find('lower') is None
        assert axis.find('upper') is None
        assert float(axis.findtext('limit/lower')) < 0
        assert float(axis.findtext('limit/upper')) > 0
        if joint.attrib['name'].startswith('wheel_'):
            assert float(axis.findtext('limit/lower')) < -1e10
            assert float(axis.findtext('limit/upper')) > 1e10
            assert float(axis.findtext('limit/effort')) == 4
            assert float(axis.findtext('limit/velocity')) > 16


def test_balance_control_interfaces_match_source_fragment():
    model = ET.parse(PACKAGE / 'sdf' / 'robot.sdf').getroot().find('model')
    embedded = model.find('ros2_control')
    fragment = ET.fromstring('<root>' + (PACKAGE / 'sdf' / 'cyberbot_control.xml').read_text()
                             + '</root>').find('ros2_control')
    def interfaces(control):
        return {joint.attrib['name']: (
            {c.attrib['name'] for c in joint.findall('command_interface')},
            {s.attrib['name'] for s in joint.findall('state_interface')})
            for joint in control.findall('joint')}
    expected = {
        name: ({'position'}, {'position', 'velocity'}) for name in (
            'joint_left_front', 'joint_left_rear', 'joint_right_front', 'joint_right_rear')}
    expected.update({name: (set(), {'position', 'velocity'}) for name in (
        'passive_left_front', 'passive_left_rear', 'passive_right_front', 'passive_right_rear')})
    expected.update({name: ({'effort'}, {'position', 'velocity'}) for name in (
        'wheel_left', 'wheel_right')})
    assert interfaces(embedded) == interfaces(fragment) == expected
    assert float(model.findtext('link/sensor/update_rate')) == 225


@pytest.mark.parametrize('side,encoder_sign', [('left', -1), ('right', 1)])
@pytest.mark.parametrize('front,back', [(-0.2, -0.4), (-0.5, -0.3), (-0.35, -0.35)])
@pytest.mark.parametrize('front_rate,back_rate,pitch_rate,body_forward_mps', [
    (0.0, 0.0, 0.0, 0.2),
    (0.0, 0.0, 0.0, -0.2),
    (0.0, 0.0, 0.3, 0.0),
    (0.12, -0.08, 0.0, 0.0),
    (-0.07, 0.11, 0.3, 0.15),
])
def test_balance_wheel_axes_and_no_slip(
        side, encoder_sign, front, back, front_rate, back_rate, pitch_rate, body_forward_mps):
    """SDF-only oracle, independent of the adapter and core housing-sign helpers."""
    model = ET.parse(PACKAGE / 'sdf' / 'robot.sdf').getroot().find('model')
    joints = {joint.attrib['name']: joint for joint in model.findall('joint')}

    def transform(x=0, y=0, z=0, roll=0, pitch=0, yaw=0):
        cr, cp, cy = np.cos([roll, pitch, yaw])
        sr, sp, sy = np.sin([roll, pitch, yaw])
        return np.array([
            [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr, x],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr, y],
            [-sp, cp * sr, cp * cr, z],
            [0, 0, 0, 1],
        ])

    def origin(name):
        joint = joints[name]
        assert joint.find('pose').attrib['relative_to'] == joint.findtext('parent')
        assert np.fromstring(joint.findtext('axis/xyz'), sep=' ') == pytest.approx([0, 0, 1])
        child_pose = model.find(f"link[@name='{joint.findtext('child')}']/pose")
        assert child_pose.attrib['relative_to'] == name
        assert np.fromstring(child_pose.text, sep=' ') == pytest.approx(np.zeros(6))
        return transform(*map(float, joint.findtext('pose').split()))

    active = [origin(f'joint_{side}_{end}') for end in ('front', 'rear')]
    passive = [origin(f'passive_{side}_{end}') for end in ('front', 'rear')]
    wheel = origin(f'wheel_{side}')
    # Active joint zeros/directions are independently specified by the CAD poses.
    directions = (1, -1) if side == 'left' else (-1, 1)
    zeros = (0.00691612, -0.00130149) if side == 'left' else (-0.00294486, -0.00176296)
    housing_branch = 1 if side == 'left' else 0
    lower_length = np.linalg.norm(wheel[:2, 3])

    def housing(f, b, pitch):
        knees = [active[i] @ transform(yaw=directions[i] * angle + zeros[i]) @ passive[i]
                 for i, angle in enumerate((f, b))]
        # Close the two lower links by circle intersection, selecting the lower
        # assembly. No production IK or passive-joint formula is used here.
        first, second = [knee[[0, 2], 3] for knee in knees]
        delta = second - first
        distance = np.linalg.norm(delta)
        normal = np.array([-delta[1], delta[0]]) / distance
        height = np.sqrt(lower_length ** 2 - distance ** 2 / 4)
        candidates = [(first + second) / 2 + sign * height * normal for sign in (-1, 1)]
        axle_x, axle_z = min(candidates, key=lambda point: point[1])
        rest = knees[housing_branch]
        axle_y = (rest @ wheel[:, 3])[1]
        axle = np.array([axle_x, axle_y, axle_z])
        target = rest[:3, :3].T @ (axle - rest[:3, 3])
        angle = np.arctan2(target[1], target[0]) - np.arctan2(wheel[1, 3], wheel[0, 3])
        frame = transform(pitch=pitch) @ rest @ transform(yaw=angle) @ wheel
        assert frame[[0, 2], 3] == pytest.approx(
            (transform(pitch=pitch) @ np.append(axle, 1))[[0, 2]], abs=2e-6)
        housing_knee = rest[:3, 3]
        lower_angle = np.arctan2(housing_knee[2] - axle_z, housing_knee[0] - axle_x)
        return frame, lower_angle

    pitch, dt, radius = 0.08, 1e-4, 0.085
    frame, _ = housing(front, back, pitch)
    before, angle_before = housing(
        front - dt * front_rate, back - dt * back_rate, pitch - dt * pitch_rate)
    after, angle_after = housing(
        front + dt * front_rate, back + dt * back_rate, pitch + dt * pitch_rate)
    axis = frame[:3, :3] @ np.array([0, 0, 1])
    assert axis == pytest.approx([0, -encoder_sign, 0], abs=2e-5)
    skew = ((after[:3, :3] - before[:3, :3]) / (2 * dt)) @ frame[:3, :3].T
    housing_omega = np.array([skew[2, 1] - skew[1, 2], skew[0, 2] - skew[2, 0],
                              skew[1, 0] - skew[0, 1]]) / 2
    lower_rate = (angle_after - angle_before) / (2 * dt)
    assert housing_omega[1] == pytest.approx(pitch_rate - lower_rate, abs=2e-6)

    # Include axle translation from changing linkage pose and body pitch, not
    # just wheel spin with a fixed housing. Vertical base motion maintains contact.
    axle_velocity = (after[:3, 3] - before[:3, 3]) / (2 * dt)
    axle_velocity[0] -= body_forward_mps
    axle_velocity[2] = 0
    model_rate = (axle_velocity[0] / radius - housing_omega[1]) / axis[1]
    contact_velocity = axle_velocity + np.cross(
        housing_omega + axis * model_rate, [0, 0, -radius])
    assert contact_velocity[:2] == pytest.approx([0, 0], abs=2e-6)
    logical_encoder_rate = encoder_sign * model_rate
    corrected_forward_rate = logical_encoder_rate + lower_rate - pitch_rate
    assert radius * corrected_forward_rate == pytest.approx(-axle_velocity[0], abs=2e-6)
    assert (encoder_sign * 1.7) * model_rate == pytest.approx(1.7 * logical_encoder_rate)


def test_hardware_sensor_frame_transform(monkeypatch):
    launch = runpy.run_path(str(PACKAGE / 'launch' / 'description.launch.py'))
    nodes = []
    node_class = launch['Node']

    def capture_node(**kwargs):
        nodes.append(kwargs)
        return node_class(**kwargs)

    monkeypatch.setitem(launch['generate_launch_description'].__globals__, 'Node', capture_node)
    launch['generate_launch_description']()
    sensor_frame = next(node for node in nodes if node.get('name') == 'base_to_base_link')
    assert sensor_frame['package'] == 'tf2_ros'
    assert sensor_frame['executable'] == 'static_transform_publisher'
    assert sensor_frame['arguments'] == [
        '--x', '0', '--y', '0', '--z', '0',
        '--roll', '0', '--pitch', '0', '--yaw', '0',
        '--frame-id', 'base', '--child-frame-id', 'base_link',
    ]
    assert not any('camera_link' in node.get('arguments', []) for node in nodes)


@pytest.mark.parametrize('sim_time', ['true', '1', 'false', '0'])
@pytest.mark.parametrize('preview', ['true', '1', 'false', '0'])
def test_ground_defaults_follow_launch_booleans(sim_time, preview):
    launch = runpy.run_path(str(PACKAGE / 'launch' / 'description.launch.py'))
    context = LaunchContext()
    context.launch_configurations.update(use_sim_time=sim_time, preview_joint_states=preview)
    for action in launch['generate_launch_description']().entities:
        if isinstance(action, DeclareLaunchArgument):
            action.execute(context)
    ground = IfCondition(LaunchConfiguration('ground_reference')).evaluate(context)
    use_imu = IfCondition(LaunchConfiguration('body_pose_imu')).evaluate(context)
    assert ground == (sim_time not in ('true', '1'))
    assert use_imu == (preview not in ('true', '1'))
    assert not IfCondition(LaunchConfiguration('odom_reference')).evaluate(context)


@pytest.mark.parametrize('flags,valid', [
    ({}, True),
    ({'odom_reference': 'true'}, True),
    ({'odom_reference': '1'}, True),
    ({'odom_reference': 'true', 'ground_reference': 'false'}, False),
    ({'odom_reference': 'true', 'preview_joint_states': 'true'}, False),
    ({'odom_reference': 'true', 'use_sim_time': 'true'}, False),
    ({'odom_reference': '1', 'use_sim_time': '1'}, False),
])
def test_navigation_requires_real_ground_reference(flags, valid):
    launch = runpy.run_path(str(PACKAGE / 'launch' / 'description.launch.py'))
    context = LaunchContext()
    context.launch_configurations.update(flags)
    for action in launch['generate_launch_description']().entities:
        if isinstance(action, DeclareLaunchArgument):
            action.execute(context)
    if valid:
        assert launch['validate_navigation'](context) == []
    else:
        with pytest.raises(RuntimeError, match='odom_reference requires'):
            launch['validate_navigation'](context)
