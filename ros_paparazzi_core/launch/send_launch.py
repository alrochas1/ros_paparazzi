from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    return LaunchDescription([

        Node(
            package='ros_paparazzi_core',
            executable='send_waypoint',
            name='waypoint_sender',
            output='screen'
            # emulate_tty=True
        )
    ])
