#!/usr/bin/env python3
"""
Isaac Sim에서 발행하는 joint_states 토픽을 구독하여 조인트 상태를 받아오는 노드

Isaac Sim의 ros2_publish_joint_state 노드가 발행하는 joint_states 토픽을 구독합니다.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from typing import Dict, Optional
import numpy as np


class JointStateSubscriber(Node):
    """
    Isaac Sim의 joint_states 토픽을 구독하여 조인트 상태를 받아오는 노드
    """
    
    def __init__(self):
        super().__init__('joint_state_subscriber')
        
        # ROS2 Subscriber
        self.subscription = self.create_subscription(
            JointState,
            'joint_states',  # Isaac Sim의 ros2_publish_joint_state 노드가 발행하는 토픽
            self.joint_state_callback,
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
        
        # 현재 조인트 상태 저장
        self.current_joint_positions: Dict[str, float] = {}
        self.current_joint_velocities: Dict[str, float] = {}
        self.current_joint_efforts: Dict[str, float] = {}
        self.last_joint_state: Optional[JointState] = None
        
        # 메시지 수신 카운터
        self.message_count = 0
        
        self.get_logger().info('='*60)
        self.get_logger().info('✅ Joint State Subscriber 시작됨')
        self.get_logger().info('='*60)
        self.get_logger().info(f'  - 구독 토픽: joint_states')
        self.get_logger().info(f'  - 조인트 개수: {len(self.all_joints)} (GO1: {len(self.go1_joints)}, K1: {len(self.k1_joints)})')
        self.get_logger().info('='*60)
    
    def joint_state_callback(self, msg: JointState):
        """조인트 상태 콜백 - Isaac Sim에서 발행된 메시지 수신"""
        self.message_count += 1
        self.last_joint_state = msg
        
        # 조인트 상태 파싱 및 저장
        for i, joint_name in enumerate(msg.name):
            if i < len(msg.position):
                self.current_joint_positions[joint_name] = float(msg.position[i])
            else:
                self.current_joint_positions[joint_name] = 0.0
            
            if i < len(msg.velocity):
                self.current_joint_velocities[joint_name] = float(msg.velocity[i])
            else:
                self.current_joint_velocities[joint_name] = 0.0
            
            if i < len(msg.effort):
                self.current_joint_efforts[joint_name] = float(msg.effort[i])
            else:
                self.current_joint_efforts[joint_name] = 0.0
        
        # 첫 번째 메시지와 주기적으로 로그 출력
        if self.message_count == 1:
            self.get_logger().info('\n✅ 첫 번째 joint state 수신 완료!')
            self._print_joint_states()
        elif self.message_count % 300 == 0:  # 약 5초마다 (60Hz 가정)
            self.get_logger().info(f'\n📊 Joint State 수신 중... (총 {self.message_count}개 메시지)')
            self._print_joint_states_summary()
    
    def _print_joint_states(self):
        """전체 조인트 상태 출력"""
        self.get_logger().info('\n=== GO1 조인트 상태 ===')
        for joint_name in self.go1_joints:
            pos = self.current_joint_positions.get(joint_name, 0.0)
            vel = self.current_joint_velocities.get(joint_name, 0.0)
            self.get_logger().info(f'  {joint_name:20s}: pos={pos:7.4f} rad, vel={vel:7.4f} rad/s')
        
        self.get_logger().info('\n=== K1 조인트 상태 ===')
        for joint_name in self.k1_joints:
            pos = self.current_joint_positions.get(joint_name, 0.0)
            vel = self.current_joint_velocities.get(joint_name, 0.0)
            self.get_logger().info(f'  {joint_name:20s}: pos={pos:7.4f} rad, vel={vel:7.4f} rad/s')
    
    def _print_joint_states_summary(self):
        """조인트 상태 요약 출력"""
        # GO1 예시 2개만 출력
        self.get_logger().info(f'  GO1 FR_hip: {self.current_joint_positions.get("FR_hip_joint", 0.0):.4f} rad')
        self.get_logger().info(f'  GO1 FR_thigh: {self.current_joint_positions.get("FR_thigh_joint", 0.0):.4f} rad')
    
    def get_joint_position(self, joint_name: str) -> float:
        """특정 조인트의 현재 위치 반환"""
        return self.current_joint_positions.get(joint_name, 0.0)
    
    def get_joint_velocity(self, joint_name: str) -> float:
        """특정 조인트의 현재 속도 반환"""
        return self.current_joint_velocities.get(joint_name, 0.0)
    
    def get_all_joint_positions(self) -> Dict[str, float]:
        """모든 조인트의 현재 위치 반환"""
        return self.current_joint_positions.copy()
    
    def get_go1_joint_positions(self) -> np.ndarray:
        """GO1 조인트 위치를 순서대로 배열로 반환"""
        positions = []
        for joint_name in self.go1_joints:
            positions.append(self.current_joint_positions.get(joint_name, 0.0))
        return np.array(positions)
    
    def get_k1_joint_positions(self) -> np.ndarray:
        """K1 조인트 위치를 순서대로 배열로 반환"""
        positions = []
        for joint_name in self.k1_joints:
            positions.append(self.current_joint_positions.get(joint_name, 0.0))
        return np.array(positions)
    
    def get_last_joint_state(self) -> Optional[JointState]:
        """마지막 수신한 JointState 메시지 반환"""
        return self.last_joint_state


def main(args=None):
    rclpy.init(args=args)
    
    node = JointStateSubscriber()
    
    try:
        node.get_logger().info('\n🔄 Joint State 구독 시작... (Ctrl+C로 종료)\n')
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('\n\n✅ Joint State Subscriber 종료됨')
        node.get_logger().info(f'   총 {node.message_count}개 메시지 수신 완료')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

