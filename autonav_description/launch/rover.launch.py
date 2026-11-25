"""Publish the TASC rover description and optionally spawn it in Gazebo Harmonic."""

from pathlib import Path
from shlex import quote

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
import xacro


def launch_rover(context):
    share = Path(get_package_share_directory('autonav_description'))
    simulate = IfCondition(LaunchConfiguration('gazebo')).evaluate(context)
    description = xacro.process_file(
        str(share / 'urdf' / 'rover.urdf.xacro'),
        mappings={
            'gazebo': str(simulate).lower(),
            'mesh_root': (share / 'meshes').as_uri() if simulate else
            'package://autonav_description/meshes',
        },
    ).toxml()
    nodes = [
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': ParameterValue(description, value_type=str),
                'use_sim_time': simulate,
            }],
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', str(share / 'rviz' / 'rover.rviz')],
            parameters=[{'use_sim_time': simulate}],
            condition=IfCondition(LaunchConfiguration('rviz')),
            output='screen',
        ),
    ]
    if not simulate:
        # Disable the sliders on hardware, where the joint-state source is external.
        nodes.append(Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            condition=IfCondition(LaunchConfiguration('joint_gui')),
        ))
        return nodes

    # Physics is the sole joint-state source in simulation.
    gazebo_launch = Path(get_package_share_directory('ros_gz_sim')) / 'launch' / 'gz_sim.launch.py'
    server_only = IfCondition(LaunchConfiguration('server_only')).evaluate(context)
    gz_args = f'-r {quote(str(share / "worlds" / "rover_test.sdf"))}'
    if server_only:
        gz_args = f'-s {gz_args}'
    nodes.extend([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(gazebo_launch)),
            launch_arguments={'gz_args': gz_args}.items(),
        ),
        Node(
            package='ros_gz_sim',
            executable='create',
            arguments=[
                '-world', 'rover_test', '-name', 'tasc_rover',
                '-topic', '/robot_description', '-z', LaunchConfiguration('spawn_z'),
            ],
            output='screen',
        ),
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            parameters=[{'config_file': str(share / 'config' / 'bridge.yaml')}],
            output='screen',
        ),
    ])
    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('gazebo', default_value='false', description='Run Gazebo Harmonic'),
        DeclareLaunchArgument('rviz', default_value='true', description='Open RViz'),
        DeclareLaunchArgument('joint_gui', default_value='true', description='Open joint sliders in display mode'),
        DeclareLaunchArgument('server_only', default_value='false', description='Hide the Gazebo GUI'),
        DeclareLaunchArgument('spawn_z', default_value='0.43', description='Gazebo spawn height in metres'),
        OpaqueFunction(function=launch_rover),
    ])
