from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

def generate_launch_description():

    return LaunchDescription([
        
        DeclareLaunchArgument('units', default_value='LTP', description='Units for coordinates (LTP or WGS84)'),

        Node(
            package='ros_paparazzi_core',
            executable='data_provider',
            name='data_provider',
            output='screen',
            emulate_tty=True,
            parameters=[{'units': LaunchConfiguration('units')}]
        ),

        Node(
            package='ros_paparazzi_core',
            executable='telemetry',
            name='telemetry',
            output='screen',
            emulate_tty=True
        )
        
        # Node(
        #     package='ros_paparazzi_core',
        #     executable='send',
        #     name='waypoint_sender',
        #     output='screen',
        #     emulate_tty=True
        #     # parameters=[{'units': LaunchConfiguration('units')}]
        # )
    ])
