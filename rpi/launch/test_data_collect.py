from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='self_driving_pkg',
            namespace='test',
            executable='',
            name='image_test'
        ),
        Node(
            package='self_driving_pkg',
            namespace='test',
            executable='dataset_recorder',
            name='recorder'
        ),
    ])