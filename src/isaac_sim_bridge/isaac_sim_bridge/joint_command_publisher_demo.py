#!/usr/bin/env python3
"""
Joint Command Publisher 데모 노드

/joint_command 토픽에 간단한 조인트 움직임 시퀀스를 발행하는 데모입니다.
K1 Arm 조인트에 사인파 움직임을 적용합니다.
"""

import threading
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import math
import time


class JointCommandPublisherDemo(Node):
    """조인트 명령 퍼블리시 데모 노드"""
    
    def __init__(self):
        super().__init__('joint_command_publisher_demo')
        
        # Joint command 발행
        self.publisher = self.create_publisher(
            JointState,
            '/joint_command',
            10
        )
        
        # 조인트 순서
        self.all_joints = [
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
        self.go1_home_positions = {
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
        self.k1_home_positions = {
            'joint1': 0.0,
            'joint2': 0.0,
            'joint3': 0.0,
            'joint4': 0.0,
            'joint5': 0.0,
            'joint6': 0.0,
            'joint_gripper_left': 0.0,
            'joint_gripper_right': 0.0
        }
        
        # 시작 시간
        self.start_time = time.time()
        
        self.get_logger().info('='*60)
        self.get_logger().info('✅ Joint Command Publisher 데모 시작됨')
        self.get_logger().info('='*60)
        self.get_logger().info('  - 토픽: /joint_command')
        self.get_logger().info('  - 메시지 타입: sensor_msgs/JointState')
        self.get_logger().info(f'  - 조인트 수: {len(self.all_joints)}개')
        self.get_logger().info('  - GO1: 서 있는 자세 유지')
        self.get_logger().info('  - K1: 사인파 움직임 (joint1, joint2)')
        self.get_logger().info('  - 발행 주기: 10Hz')
        self.get_logger().info('='*60)
        self.get_logger().info('📤 Joint Command 발행 시작...')
        self.get_logger().info('')
    
    def create_joint_command(self):
        """조인트 명령 메시지 생성"""
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.name = self.all_joints
        msg.position = []
        msg.velocity = []
        msg.effort = []
        
        # 현재 시간 (초)
        t = time.time() - self.start_time
        
        # 조인트 위치 설정
        for joint_name in self.all_joints:
            if joint_name in self.go1_home_positions:
                # GO1은 홈 포지션 유지
                msg.position.append(self.go1_home_positions[joint_name])
            elif joint_name == 'joint1':
                # K1 joint1: 사인파 움직임 (주기 4초, 범위 ±0.5)
                msg.position.append(0.5 * math.sin(2 * math.pi * t / 4.0))
            elif joint_name == 'joint2':
                # K1 joint2: 코사인파 움직임 (주기 3초, 범위 ±0.3)
                msg.position.append(0.3 * math.cos(2 * math.pi * t / 3.0))
            elif joint_name in self.k1_home_positions:
                # 나머지 K1 조인트는 홈 포지션
                msg.position.append(self.k1_home_positions[joint_name])
            else:
                msg.position.append(0.0)
        
        return msg
    
    def run(self):
        """메인 루프"""
        rate = self.create_rate(10)  # 10Hz
        publish_count = 0
        
        try:
            while rclpy.ok():
                msg = self.create_joint_command()
                self.publisher.publish(msg)
                publish_count += 1
                
                if publish_count == 1:
                    self.get_logger().info('메시지 구조:')
                    self.get_logger().info(f'  - 조인트 수: {len(msg.name)}개')
                    self.get_logger().info('  - GO1: 홈 포지션 유지')
                    self.get_logger().info('  - K1 joint1: 사인파 (±0.5 rad)')
                    self.get_logger().info('  - K1 joint2: 코사인파 (±0.3 rad)')
                    self.get_logger().info('='*60)
                    self.get_logger().info('')
                elif publish_count % 50 == 0:  # 5초마다
                    t = time.time() - self.start_time
                    self.get_logger().info(f'🔄 Joint command 발행 중... ({publish_count}회, {t:.1f}초 경과)')
                
                rate.sleep()
        except KeyboardInterrupt:
            self.get_logger().info('\n⚠️  사용자에 의해 중단됨')


def main(args=None):
    rclpy.init(args=args)
    
    node = JointCommandPublisherDemo()
    
    # Spin in a separate thread
    thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    thread.start()
    
    try:
        node.run()
    except KeyboardInterrupt:
        node.get_logger().info('\n⚠️  사용자에 의해 중단됨')
    finally:
        rclpy.shutdown()
        thread.join()


if __name__ == '__main__':
    main()

