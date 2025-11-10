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
            'tf_pose_listener = isaac_sim_bridge.tf_pose_listener:main',
            'home_position_controller = isaac_sim_bridge.home_position_controller:main',
            'check_home_position = isaac_sim_bridge.check_home_position:main',
            'joint_command_publisher = isaac_sim_bridge.joint_command_publisher:main',
            'joint_state_subscriber_demo = isaac_sim_bridge.joint_state_subscriber_demo:main',
            'joint_command_publisher_demo = isaac_sim_bridge.joint_command_publisher_demo:main',
        ],
    },
)

