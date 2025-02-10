from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    return LaunchDescription([
    

        Node(
            package='ros_paparazzi_core',
            executable='sim:core_kf',
            name='SIM_CORE_KF',
            output='screen',
            emulate_tty=True
        ),

        Node(
            package='ros_paparazzi_core',
            executable='sim:kalman',
            name='SIM_KALMAN',
            output='screen',
            emulate_tty=True
        )

    ])