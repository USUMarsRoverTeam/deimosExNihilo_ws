from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'rover_teleop'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Install launch files
        (os.path.join('share', package_name, 'launch'), 
            glob('launch/*.py') + glob('launch/*.launch.py')),
        # Install config files
        (os.path.join('share', package_name, 'config'), 
            glob('config/*.yaml') + glob('config/*.yml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='USU Mars Rover Club',
    maintainer_email='marsrover@usu.edu',
    description='A package to operate the rover at a distance',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'arm_teleop_joy = rover_teleop.arm_teleop_joy:main',
            'joy_to_twist = rover_teleop.joy_to_twist:main',
        ],
    },
)