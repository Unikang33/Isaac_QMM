"""
Hip Position 비교 노드를 실행하는 Launch 파일
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
        'hip_position_comparison'
    )
    
    return LaunchDescription([
        ExecuteProcess(
            cmd=['python3', script_path],
            output='screen',
            name='hip_position_comparison'
        ),
    ])

