#!/usr/bin/env python3
"""
Joint Command Publisher 노드

/joint_command 토픽에 sensor_msgs/JointState 메시지를 발행하여 로봇을 제어합니다.
"""

import threading
import rclpy
from sensor_msgs.msg import JointState
import math


def main(args=None):
    rclpy.init(args=args)
    
    # 노드 생성
    node = rclpy.create_node('joint_command_publisher')
    
    # Joint command 발행 (토픽 이름 확인: /joint_command)
    pub = node.create_publisher(JointState, '/joint_command', 10)
    
    # 조인트 순서를 joint_states와 동일하게 맞춤
    # joint_states 순서: joint1, joint2, joint3, FL_hip, FR_hip, RL_hip, RR_hip, 
    #                   joint4, FL_thigh, FR_thigh, RL_thigh, RR_thigh, joint5,
    #                   FL_calf, FR_calf, RL_calf, RR_calf, joint6, gripper_left, gripper_right
    all_joints = [
        'joint1', 'joint2', 'joint3',
        'FL_hip_joint', 'FR_hip_joint', 'RL_hip_joint', 'RR_hip_joint',
        'joint4',
        'FL_thigh_joint', 'FR_thigh_joint', 'RL_thigh_joint', 'RR_thigh_joint',
        'joint5',
        'FL_calf_joint', 'FR_calf_joint', 'RL_calf_joint', 'RR_calf_joint',
        'joint6',
        'joint_gripper_left', 'joint_gripper_right'
    ]
    
    # GO1 home positions (서 있는 자세)
    go1_home_positions = {
        'FR_hip_joint': 0.0,
        'FR_thigh_joint': 0.67,
        'FR_calf_joint': -1.3,
        'FL_hip_joint': 0.0,
        'FL_thigh_joint': 0.67,
        'FL_calf_joint': -1.3,
        'RR_hip_joint': 0.0,
        'RR_thigh_joint': 0.67,
        'RR_calf_joint': -1.3,
        'RL_hip_joint': 0.0,
        'RL_thigh_joint': 0.67,
        'RL_calf_joint': -1.3,
    }
    
    # K1 home positions (모든 조인트 0.0)
    k1_home_positions = {
        'joint1': 0.0,
        'joint2': 0.0,
        'joint3': 0.0,
        'joint4': 0.0,
        'joint5': 0.0,
        'joint6': 0.0,
        'joint_gripper_left': 0.0,
        'joint_gripper_right': 0.0
    }
    
    # Joint command 메시지 생성
    joint_state_command = JointState()
    
    # 조인트 이름 설정
    joint_state_command.name = all_joints
    
    # 조인트 위치를 joint_states 순서에 맞게 설정
    joint_state_command.position = []
    for joint_name in all_joints:
        if joint_name in go1_home_positions:
            joint_state_command.position.append(go1_home_positions[joint_name])
        elif joint_name in k1_home_positions:
            joint_state_command.position.append(k1_home_positions[joint_name])
        else:
            joint_state_command.position.append(0.0)  # 기본값
    
    # 속도와 토크는 비워둠
    joint_state_command.velocity = []
    joint_state_command.effort = []
    
    # Spin in a separate thread
    thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    thread.start()
    
    # 발행 주기 설정 (10Hz)
    rate = node.create_rate(10)
    
    # 시작 메시지 출력
    node.get_logger().info('='*60)
    node.get_logger().info('✅ Joint Command Publisher 시작됨')
    node.get_logger().info('='*60)
    node.get_logger().info('  - 토픽: joint_command')
    node.get_logger().info('  - 메시지 타입: sensor_msgs/JointState')
    node.get_logger().info(f'  - 조인트 수: {len(all_joints)}개')
    node.get_logger().info('  - GO1: 서 있는 자세 (hip: 0.0, thigh: 0.67, calf: -1.3)')
    node.get_logger().info('  - K1: 모든 조인트 0.0')
    node.get_logger().info('  - 발행 주기: 10Hz')
    node.get_logger().info('='*60)
    node.get_logger().info('📤 Joint Command 발행 시작...')
    node.get_logger().info('')
    
    publish_count = 0
    
    try:
        while rclpy.ok():
            # Header 설정 (매번 업데이트)
            joint_state_command.header.stamp = node.get_clock().now().to_msg()
            joint_state_command.header.frame_id = 'base_link'
            
            # 메시지 발행
            pub.publish(joint_state_command)
            
            publish_count += 1
            
            # 첫 번째 발행 시 상세 정보 출력
            if publish_count == 1:
                node.get_logger().info('메시지 구조:')
                node.get_logger().info(f'  - header.frame_id: {joint_state_command.header.frame_id}')
                node.get_logger().info(f'  - name[]: {len(joint_state_command.name)}개 조인트')
                node.get_logger().info(f'  - position[]: {len(joint_state_command.position)}개 값')
                node.get_logger().info(f'  - velocity[]: {len(joint_state_command.velocity)}개 값 (비어있음)')
                node.get_logger().info(f'  - effort[]: {len(joint_state_command.effort)}개 값 (비어있음)')
                node.get_logger().info('')
                node.get_logger().info('조인트 순서 (joint_states와 일치):')
                node.get_logger().info(f'  총 {len(all_joints)}개 조인트')
                node.get_logger().info('GO1 조인트 (각도):')
                for leg in ['FR', 'FL', 'RR', 'RL']:
                    hip = math.degrees(go1_home_positions[f'{leg}_hip_joint'])
                    thigh = math.degrees(go1_home_positions[f'{leg}_thigh_joint'])
                    calf = math.degrees(go1_home_positions[f'{leg}_calf_joint'])
                    node.get_logger().info(f'  {leg}: hip={hip:.1f}°, thigh={thigh:.1f}°, calf={calf:.1f}°')
                node.get_logger().info('K1 조인트: 모두 0.0 rad')
                node.get_logger().info('='*60)
                node.get_logger().info('')
            elif publish_count % 50 == 0:  # 5초마다
                node.get_logger().info(f'🔄 Joint command 발행 중... ({publish_count}회)')
            
            rate.sleep()
            
    except KeyboardInterrupt:
        node.get_logger().info('\n⚠️  사용자에 의해 중단됨')
    finally:
        rclpy.shutdown()
        thread.join()


if __name__ == '__main__':
    main()

