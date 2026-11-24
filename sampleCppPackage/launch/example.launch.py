from launch import LaunchDescription
from launch_ros.actions import Node
# the next 3 for launch files of another package
#from launch.actions import IncludeLaunchDescription   
#from launch.substitutions import PathJoinSubstitution
#from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([

        #DeclareLaunchArgument('my_parameter', default_value='my_world', description='This para is mine!')
        DeclareLaunchArgument('my_parameter', default_value='my_world'),
        Node(
            package='sampleCppPackage',
            namespace='turtlesim2',
            executable='minimal_param_node',
            name='minimal_param_node', # you can change this
            ros_arguments=['--log-level', 'warn'],
            parameters=[{
                'my_parameter': LaunchConfiguration('my_parameter')
            }]
        )
    ])