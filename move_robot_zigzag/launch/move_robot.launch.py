from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='move_robot_zigzag',
            executable='move_zigzag_exe',
            output='screen',
            emulate_tty=True),
    ])