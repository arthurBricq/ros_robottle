from glob import glob
from setuptools import setup

package_name = 'robottle'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='arthur',
    maintainer_email='arthur@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
		    'lidar = robottle.lidar_publisher:main',
		    'listener = robottle.subscriber:main',
		    'slam = robottle.slam:main',
                    'slam_viz = robottle.slam_vizualizer:main',
		    'controller_ol = robottle.controller_ol:main',
                    'uart = robottle.uart_messenger:main' 

        ],
    },
)
