from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='hand_gesture',
            executable='hand_gesture_demo_node',
            name='hand_gesture_demo_node',
            output='screen'
        )
    ])

