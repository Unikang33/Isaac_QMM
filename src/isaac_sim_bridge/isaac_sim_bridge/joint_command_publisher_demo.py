#!/usr/bin/env python3
"""
/quadmani/joint_commands 토픽으로 조인트 명령을 퍼블리시하는 데모 노드

간단한 조인트 움직임 시퀀스를 전송합니다.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import math
import time


class JointCommandPublisherDemo(Node):
    """조인트 명령을 퍼블리시하는 데모"""
    
    def __init__(self):
        super().__init__('joint_command_publisher_demo')
        
        self.publisher_ = self.create_publisher(
            JointState, 
            '/quadmani/joint_commands', 
            10)
        
        # 조인트 정의
        self.go1_joints = [
            'FR_hip_joint', 'FR_thigh_joint', 'FR_calf_joint',
            'FL_hip_joint', 'FL_thigh_joint', 'FL_calf_joint',
            'RR_hip_joint', 'RR_thigh_joint', 'RR_calf_joint',
            'RL_hip_joint', 'RL_thigh_joint', 'RL_calf_joint',
        ]
        
        self.k1_joints = [
            'joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6',
            'joint_gripper_left', 'joint_gripper_right'
        ]
        
        self.all_joints = self.go1_joints + self.k1_joints
        
        # 타이머 설정 (1Hz로 명령 전송)
        self.timer = self.create_timer(1.0, self.timer_callback)
        
        self.start_time = self.get_clock().now()
        
        self.get_logger().info(
            f'Joint Command Publisher Demo 시작됨\n'
            f'  - 조인트 개수: {len(self.all_joints)}\n'
            f'  - 퍼블리시 토픽: /quadmani/joint_commands\n'
            f'  - 주기: 1초\n'
            f'  - 데모 모드: 간단한 조인트 움직임 시퀀스')
    
    def timer_callback(self):
        """주기적으로 조인트 명령 퍼블리시"""
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.name = self.all_joints
        
        # 시간 기반 간단한 움직임 시퀀스
        elapsed = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
        
        positions = []
        
        # GO1 다리: 홈 포지션
        for joint_name in self.go1_joints:
            if 'thigh' in joint_name:
                positions.append(0.9)  # 대략 서있는 자세
            elif 'calf' in joint_name:
                positions.append(-1.8)
            else:
                positions.append(0.0)
        
        # K1 Arm: 사인파 움직임 (데모용)
        for i, joint_name in enumerate(self.k1_joints):
            if 'gripper' not in joint_name:
                # 각 조인트를 다른 주기로 움직임
                amplitude = 0.3  # 라디안
                frequency = 0.5 + i * 0.1  # 각 조인트마다 다른 주파수
                positions.append(amplitude * math.sin(elapsed * frequency))
            else:
                positions.append(0.0)  # 그리퍼는 닫힌 상태
        
        msg.position = positions
        msg.velocity = []  # 속도 제어는 선택적
        msg.effort = []   # 토크 제어는 선택적
        
        self.publisher_.publish(msg)
        
        # 주기적으로 로그 출력 (5초마다)
        if int(elapsed) % 5 == 0:
            self.get_logger().info(
                f'조인트 명령 전송됨 (경과 시간: {elapsed:.1f}초)\n'
                f'  첫 번째 K1 조인트 위치: {positions[len(self.go1_joints)]:.4f} rad')


def main(args=None):
    rclpy.init(args=args)
    
    node = JointCommandPublisherDemo()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('데모 종료')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

