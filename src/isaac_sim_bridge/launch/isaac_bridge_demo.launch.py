"""
Isaac Sim 브리지 데모를 실행하는 Launch 파일
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # 조인트 상태 구독 데모
        Node(
            package='isaac_sim_bridge',
            executable='joint_state_subscriber_demo',
            name='joint_state_subscriber_demo',
            output='screen'
        ),
        
        # 조인트 명령 퍼블리시 데모 (선택적)
        # 주석 해제하여 사용:
        # Node(
        #     package='isaac_sim_bridge',
        #     executable='joint_command_publisher_demo',
        #     name='joint_command_publisher_demo',
        #     output='screen'
        # ),
    ])

