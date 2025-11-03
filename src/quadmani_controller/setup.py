from setuptools import setup
from glob import glob
import os

package_name = 'quadmani_controller'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, ['plugin.xml']),
        ('share/' + package_name + '/resource', glob('resource/*'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='robotics22',
    maintainer_email='robotics22@todo.todo',
    description='RQT plugin for controlling QuadMani robot',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)

