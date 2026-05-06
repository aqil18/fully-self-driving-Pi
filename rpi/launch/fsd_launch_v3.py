from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='v4l2_camera',
            namespace='fsd',
            executable='v4l2_camera_node',
            name='camera'
        ),
        # Detects lane and publishes /detection/offset
        Node(
            package='self_driving_pkg',
            namespace='fsd',
            executable='detection_node',
            name='detection'
        ),
        # Takes image + offset, runs model, publishes /motor/cmd
        Node(
            package='self_driving_pkg',
            namespace='fsd',
            executable='inference_node',
            name='inference'
        ),
        Node(
            package='self_driving_pkg',
            namespace='fsd',
            executable='motor_node',
            name='motor'
        )
    ])
