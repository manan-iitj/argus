"""Launch the ARGUS command controller node with default flight parameters.

Usage:
    ros2 launch argus_controller argus_controller.launch.py
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='argus_controller',
            executable='argus_command_controller',
            output='screen',
            parameters=[{
                'udp_command_receiver': {
                    'udp_port': 5005,
                    # Seconds before the SAME command is accepted again
                    # (breaks up the continuous stream from a held gesture).
                    'command_cooldown': 1.0,
                },
                'argus_command_controller': {
                    'control_rate': 20.0,
                    'movement_distance': 1.0,
                    'altitude_step': 0.5,
                    'takeoff_altitude': 2.0,
                },
            }],
        ),
    ])
