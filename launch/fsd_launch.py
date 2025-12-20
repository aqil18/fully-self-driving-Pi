from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='self_driving_pkg',
            namespace='test',
            executable='camera_node',
            name='camera'
        ),
        Node(
            package='self_driving_pkg',
            namespace='test',
            executable='inference_node',
            name='inference'
        ),
        Node(
            package='self_driving_pkg',
            namespace='test',
            executable='display_node',
            name='display'
        ),
        Node(
            package='self_driving_pkg',
            namespace='test',
            executable='motor_node',
            name='motor'
        )
    ])