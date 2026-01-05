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
        Node(
            package='self_driving_pkg',
            namespace='fsd',
            executable='teleop_node',
            name='teleop'
        ),
        Node(
            package='web_video_server',
            namespace='fsd',
            executable='web_video_server',
            name='web_video_server'
        ),
        Node(
            package='self_driving_pkg',
            namespace='fsd',
            executable='motor_node',
            name='motor'
        )
    ])