#!/usr/bin/env python3

from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.conditions import IfCondition


def load_robot_description():
    package_path: Path = Path(get_package_share_directory("cyberbot_description"))
    sdf_file: Path = package_path / "sdf" / "robot.sdf"
    # RViz's resource retriever understands package://, not SDF's model://.
    return sdf_file.read_text().replace(
        "model://cyberbot_description/", "package://cyberbot_description/"
    )


def validate_navigation(context):
    flags = {name: LaunchConfiguration(name).perform(context).lower() in ('true', '1')
             for name in ('odom_reference', 'ground_reference', 'use_sim_time', 'preview_joint_states')}
    if flags['odom_reference'] and (not flags['ground_reference']
                                    or flags['use_sim_time'] or flags['preview_joint_states']):
        raise RuntimeError('odom_reference requires hardware odometry, ground_reference:=true, '
                           'use_sim_time:=false and preview_joint_states:=false')
    return []


def generate_launch_description():
    robot_description = load_robot_description()
    use_sim_time = ParameterValue(PythonExpression([
        "'", LaunchConfiguration('use_sim_time'), "'.lower() in ('true', '1')"
    ]), value_type=bool)
    return LaunchDescription([
        DeclareLaunchArgument(
            name='use_sim_time',
            default_value='false',
            description='Use simulation time'
        ),
        DeclareLaunchArgument(
            name='joint_states_topic',
            default_value='/joint_states',
            description='Model-space JointState input; the MCU publishes /joint_states'
        ),
        DeclareLaunchArgument(
            name='preview_joint_states',
            default_value='false',
            description='Publish a nominal model pose, not measured robot joints'
        ),
        DeclareLaunchArgument(
            name='ground_reference',
            default_value=PythonExpression(["'", LaunchConfiguration('use_sim_time'), "'.lower() not in ('true', '1')"]),
            description='Estimate base_footprint -> base for hardware visualization'
        ),
        DeclareLaunchArgument(
            name='odom_reference',
            default_value='false',
            description='Publish odom -> base_footprint from planar MCU odometry; requires new firmware'
        ),
        DeclareLaunchArgument(
            name='odom_topic',
            default_value='/odom',
            description='Planar odometry input, with child_frame_id base_footprint'
        ),
        DeclareLaunchArgument(
            name='body_pose_imu',
            default_value=PythonExpression(["'", LaunchConfiguration('preview_joint_states'), "'.lower() not in ('true', '1')"]),
            description='Use measured IMU tilt; false explicitly assumes a level body'
        ),
        DeclareLaunchArgument(
            name='imu_frame',
            default_value='base_link',
            description='Expected frame_id of calibrated /imu/data orientations'
        ),
        DeclareLaunchArgument(
            name='imu_frame_rpy',
            default_value='[0.0, 0.0, 0.0]',
            description='Reported IMU frame rotation in model base, radians; assumes calibrated axes'
        ),
        OpaqueFunction(function=validate_navigation),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='both',
            parameters=[
                {
                    'use_sim_time': use_sim_time,
                    'robot_description': robot_description
                }
            ],
            remappings=[('joint_states', LaunchConfiguration('joint_states_topic'))]
        ),
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='preview_joint_state_publisher',
            condition=IfCondition(LaunchConfiguration('preview_joint_states')),
            parameters=[{'use_sim_time': use_sim_time}],
            # Preview is opt-in and must not run alongside the MCU publisher.
            remappings=[('joint_states', '/joint_states')]
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_laser',
            arguments=[
                '--x', '0', '--y', '0', '--z', '0.15',
                '--roll', '0', '--pitch', '0', '--yaw', '0',
                '--frame-id', 'base', '--child-frame-id', 'laser',
            ],
            parameters=[{'use_sim_time': use_sim_time}]
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_base_link',
            # Hardware sensors use base_link; the camera driver owns its mounting TF.
            arguments=[
                '--x', '0', '--y', '0', '--z', '0',
                '--roll', '0', '--pitch', '0', '--yaw', '0',
                '--frame-id', 'base', '--child-frame-id', 'base_link',
            ],
            parameters=[{'use_sim_time': use_sim_time}]
        ),
        Node(
            package='cyberbot_description',
            executable='body_pose',
            output='screen',
            condition=IfCondition(LaunchConfiguration('ground_reference')),
            parameters=[{
                'use_sim_time': use_sim_time,
                'use_imu': ParameterValue(LaunchConfiguration('body_pose_imu'), value_type=bool),
                'imu_frame': LaunchConfiguration('imu_frame'),
                'imu_frame_rpy': LaunchConfiguration('imu_frame_rpy'),
            }]
        ),
        Node(
            package='cyberbot_description',
            executable='odom_tf',
            output='screen',
            condition=IfCondition(LaunchConfiguration('odom_reference')),
            parameters=[{'use_sim_time': use_sim_time}],
            remappings=[('/odom', LaunchConfiguration('odom_topic'))]
        ),
    ])
