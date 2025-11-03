from setuptools import setup
import os
from glob import glob

package_name = 'isaac_sim_bridge'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='robotics22',
    maintainer_email='robotics22@todo.todo',
    description='ROS2 bridge for Isaac Sim to control QuadMani robot',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'fk_ik_controller = isaac_sim_bridge.fk_ik_controller:main',
        ],
    },
)

