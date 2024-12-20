from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    return LaunchDescription([
    

        Node(
            package='ros_paparazzi_core',
            executable='sim:core',
            name='SIM_CORE',
            output='screen',
            emulate_tty=True
        ),

        Node(
            package='ros_paparazzi_core',
            executable='sim:gps',
            name='GPS_SIM',
            output='screen',
            emulate_tty=True,
        ),

        Node(
            package='ros_paparazzi_core',
            executable='sim:imu',
            name='IMU_SIM',
            output='screen',
            emulate_tty=True
        ),


        Node(
            package='ros_paparazzi_core',
            executable='sim:kalman',
            name='KALMAN_SIM',
            output='screen',
            emulate_tty=True
        )

    ])