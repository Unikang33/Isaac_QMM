"""
Joint State Subscriber를 실행하는 Launch 파일
"""

from launch import LaunchDescription
from launch.actions import ExecuteProcess
import os


def generate_launch_description():
    # 실행 파일 경로
    script_path = os.path.join(
        os.path.expanduser('~'),
        'kiro_ws',
        'install',
        'isaac_sim_bridge',
        'bin',
        'joint_state_subscriber'
    )
    
    return LaunchDescription([
        # Joint State Subscriber 노드
        ExecuteProcess(
            cmd=['python3', script_path],
            output='screen',
            name='joint_state_subscriber'
        ),
    ])

