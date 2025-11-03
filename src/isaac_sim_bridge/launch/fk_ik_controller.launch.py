"""
FK-IK Controller 노드를 실행하는 Launch 파일
"""

from launch import LaunchDescription
from launch.actions import ExecuteProcess
import os


def generate_launch_description():
    script_path = os.path.join(
        os.path.expanduser('~'),
        'kiro_ws',
        'install',
        'isaac_sim_bridge',
        'bin',
        'fk_ik_controller'
    )
    
    return LaunchDescription([
        ExecuteProcess(
            cmd=['python3', script_path],
            output='screen',
            name='fk_ik_controller'
        ),
    ])

