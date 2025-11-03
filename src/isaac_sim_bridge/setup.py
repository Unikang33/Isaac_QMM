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
            'isaac_joint_state_publisher = isaac_sim_bridge.isaac_joint_state_publisher:main',
            'isaac_joint_command_subscriber = isaac_sim_bridge.isaac_joint_command_subscriber:main',
            'joint_state_subscriber = isaac_sim_bridge.joint_state_subscriber:main',
            'joint_state_subscriber_demo = isaac_sim_bridge.joint_state_subscriber_demo:main',
            'joint_command_publisher_demo = isaac_sim_bridge.joint_command_publisher_demo:main',
            'home_pose_publisher = isaac_sim_bridge.home_pose_publisher:main',
            'quadruped_ik_controller = isaac_sim_bridge.quadruped_ik_controller:main',
            'hip_position_comparison = isaac_sim_bridge.hip_position_comparison:main',
            'isaac_hip_position_publisher = isaac_sim_bridge.isaac_hip_position_publisher:main',
            'hip_position_calculator = isaac_sim_bridge.hip_position_calculator:main',
            'fk_ik_comparison = isaac_sim_bridge.fk_ik_comparison:main',
            'fk_ik_controller = isaac_sim_bridge.fk_ik_controller:main',
        ],
    },
)

