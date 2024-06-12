from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'roboclaw_node'
subpackage_name = 'roboclaw_driver'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*')))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Arthur Niedzwiecki',
    maintainer_email='aniedz@cs.uni-bremen.de',
    description='ROS 2 Roboclaw Node package',
    license='BSD',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'robo_node = ' + package_name + '.roboclaw_node:main',
            'test_claw = ' + subpackage_name + '.roboclaw_driver_test:ping_wheels'
        ],
    },
)
