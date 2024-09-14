from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='trajectory_package',
            executable='trajectory_reader',
            name='trajectory_reader',
            output='screen',
            parameters=[{
                'trajectory_file': '/path/to/trajectory.csv',
                'marker_topic': '/trajectory_markers'
            }]
        ),
    ])
