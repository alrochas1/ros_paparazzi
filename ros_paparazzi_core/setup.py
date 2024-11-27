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
    entry_points={
        'console_scripts': [
                'computer = ros_paparazzi_core.computer_suscriber:main',    # Legacy
                'raspy = ros_paparazzi_core.raspy_publisher:main',
                'send_waypoint = ros_paparazzi_core.waypoint_sender:main',
                'telemetry = ros_paparazzi_core.telemetry_receiver:main',
                'data_provider = ros_paparazzi_core.data_provider:main',
                'bokeh_serve = ros_paparazzi_core.scripts.bokeh_serve:main',
        ],
        'launch.frontend': [
            'main_launch = ros_paparazzi_core.launch.main_launch',
            'send_launch = ros_paparazzi_core.launch.send_launch', 
        ],
    },
)
