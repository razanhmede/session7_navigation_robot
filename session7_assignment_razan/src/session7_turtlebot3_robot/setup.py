from setuptools import find_packages, setup
import os
from glob import glob 
package_name = 'session7_turtlebot3_robot'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/navigation_robot_launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
         'robot_driver = session7_turtlebot3_robot.robot_driver:main',
         'wall_finder_service_server = session7_turtlebot3_robot.wall_finder_service_server:main',
         'lap_time_action_server = session7_turtlebot3_robot.lap_time_action_server:main',
         'lap_time_action_client = session7_turtlebot3_robot.lap_time_action_client:main',
        ],
    },
)
