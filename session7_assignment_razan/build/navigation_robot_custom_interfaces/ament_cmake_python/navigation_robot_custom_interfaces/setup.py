from setuptools import find_packages
from setuptools import setup

setup(
    name='navigation_robot_custom_interfaces',
    version='0.0.0',
    packages=find_packages(
        include=('navigation_robot_custom_interfaces', 'navigation_robot_custom_interfaces.*')),
)
