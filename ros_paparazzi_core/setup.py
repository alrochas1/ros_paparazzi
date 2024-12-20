import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'ros_paparazzi_core'

setup(
    name=package_name,
        version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include all launch files.
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*')))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ucmrospy',
    description='Manage the communication with the Paparazzi AP',
    license='Apache-2.0',
    tests_require=['pytest'],
    package_data={
        'ros_paparazzi_core.simulator': ['sim_config.yaml'],
    },
    entry_points={
        'console_scripts': [
                'computer = ros_paparazzi_core.computer_suscriber:main',    # Legacy
                'raspy = ros_paparazzi_core.raspy_publisher:main',
                'send_waypoint = ros_paparazzi_core.waypoint_sender:main',  # Legacy
                'telemetry = ros_paparazzi_core.telemetry_receiver:main',
                'data_provider = ros_paparazzi_core.data_provider:main',    # Legacy
                'bokeh_serve = ros_paparazzi_core.scripts.bokeh_serve:main',
                # Simulator
                'sim:core = ros_paparazzi_core.simulator.sim_core:main',
                'sim:imu = ros_paparazzi_core.simulator.sim_imu:main',
                'sim:gps = ros_paparazzi_core.simulator.sim_gps:main',
                'sim:kalman = ros_paparazzi_core.simulator.extended_kalman_filter:main',
        ],
        'launch.frontend': [
            'main_launch = ros_paparazzi_core.launch.main_launch',
            'send_launch = ros_paparazzi_core.launch.send_launch', 
            'sim_launch = ros_paparazzi_core.launch.sim_launch', 
        ],
    },
)
