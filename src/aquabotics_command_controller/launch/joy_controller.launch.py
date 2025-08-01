from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='my_cpp_pkg',
            executable='my_executable',
            name='my_node',
            output='screen'
        ),
        # Add more nodes here if needed
    ])