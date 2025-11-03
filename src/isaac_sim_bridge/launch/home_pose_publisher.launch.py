"""
Home 자세 퍼블리시 노드를 실행하는 Launch 파일
"""

from launch import LaunchDescription
from launch.actions import ExecuteProcess
import os


def generate_launch_description():
    # 실행 파일 경로
    # colcon build 후 bin 디렉토리에 생성된 실행 파일 사용
    script_path = os.path.join(
        os.path.expanduser('~'),
        'kiro_ws',
        'install',
        'isaac_sim_bridge',
        'bin',
        'home_pose_publisher'
    )
    
    return LaunchDescription([
        # Home 자세 퍼블리시 노드
        ExecuteProcess(
            cmd=['python3', script_path],
            output='screen',
            name='home_pose_publisher'
        ),
    ])

