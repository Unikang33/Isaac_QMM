"""
Hip Position 비교 테스트 Launch 파일

두 노드를 동시에 실행:
1. hip_position_calculator: Joint state 기반으로 hip position 계산 및 발행
2. hip_position_comparison: 계산된 값과 Isaac Sim에서 받아온 값 비교
"""

from launch import LaunchDescription
from launch.actions import ExecuteProcess
import os


def generate_launch_description():
    calculator_path = os.path.join(
        os.path.expanduser('~'),
        'kiro_ws',
        'install',
        'isaac_sim_bridge',
        'bin',
        'hip_position_calculator'
    )
    
    comparison_path = os.path.join(
        os.path.expanduser('~'),
        'kiro_ws',
        'install',
        'isaac_sim_bridge',
        'bin',
        'hip_position_comparison'
    )
    
    return LaunchDescription([
        # Hip Position Calculator (계산된 값 발행)
        ExecuteProcess(
            cmd=['python3', calculator_path],
            output='screen',
            name='hip_position_calculator'
        ),
        
        # Hip Position Comparison (비교)
        ExecuteProcess(
            cmd=['python3', comparison_path],
            output='screen',
            name='hip_position_comparison'
        ),
    ])

