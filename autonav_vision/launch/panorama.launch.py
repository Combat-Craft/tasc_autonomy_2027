"""
ROS 2 Panorama Launch File

Launches:
    - motor_controller
    - panorama_capture

The panorama capture node loads its configuration from config/panorama.yaml.

The standalone panorama_opencv_stitcher is not launched.
It runs separately after images have been captured.

Terminal command:
    ros2 launch autonav_vision panorama.launch.py

Override Arduino serial port:
    ros2 launch autonav_vision panorama.launch.py serial_port:=/dev/ttyUSB0
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    # -------------------------------------------------
    # Launch arguments
    # -------------------------------------------------

    serial_port_arg = DeclareLaunchArgument(
        'serial_port',
        default_value='/dev/nano',
        description='Serial port for the Arduino motor controller'
    )

    baud_rate_arg = DeclareLaunchArgument(
        'baud_rate',
        default_value='115200',
        description='Arduino serial communication baud rate'
    )

    serial_port = LaunchConfiguration('serial_port')
    baud_rate = LaunchConfiguration('baud_rate')

    # -------------------------------------------------
    # Panorama configuration
    # -------------------------------------------------

    panorama_config = PathJoinSubstitution([
        FindPackageShare('autonav_vision'),
        'config',
        'panorama.yaml'
    ])

    # -------------------------------------------------
    # Motor controller
    # -------------------------------------------------

    motor_controller_node = Node(
        package='autonav_vision',
        executable='motor_controller',
        name='motor_controller',
        output='screen',
        parameters=[{
            'serial_port': serial_port,
            'baud_rate': ParameterValue(
                baud_rate,
                value_type=int
            )
        }]
    )

    # -------------------------------------------------
    # Panorama capture
    # -------------------------------------------------

    panorama_capture_node = Node(
        package='autonav_vision',
        executable='panorama_capture',
        name='panorama_capture',
        output='screen',
        parameters=[panorama_config]
    )

    # -------------------------------------------------
    # Launch description
    # -------------------------------------------------

    return LaunchDescription([
        serial_port_arg,
        baud_rate_arg,
        motor_controller_node,
        panorama_capture_node
    ])
