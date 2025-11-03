#!/usr/bin/env python3
"""
Joint State 기반 Hip Position 계산 노드 (가벼운 버전)

Isaac Sim의 USD API를 직접 사용하는 대신, 이미 받아오는 joint_states를
사용하여 Forward Kinematics로 hip position을 계산합니다.

이 방법은 Isaac Sim Script Editor에서 실행할 필요가 없고,
일반 ROS2 환경에서 실행 가능합니다.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Point, PointStamped
import numpy as np
import math
from typing import Dict


class HipPositionCalculator(Node):
    """Joint state 기반으로 hip position을 계산하는 노드"""
    
    def __init__(self):
        super().__init__('hip_position_calculator')
        
        # Joint state 구독
        self.joint_state_subscriber = self.create_subscription(
            JointState,
            'joint_states',
            self.joint_state_callback,
            10
        )
        
        # 각 hip의 position을 발행하는 퍼블리셔
        self.hip_publishers = {}
        hip_names = ['FR_hip', 'FL_hip', 'RR_hip', 'RL_hip']
        
        for hip_name in hip_names:
            self.hip_publishers[hip_name] = self.create_publisher(
                PointStamped,
                f'{hip_name.lower()}_position',
                10
            )
        
        # 현재 joint state 저장
        self.current_joint_positions: Dict[str, float] = {}
        
        # Base position (기본값, 추후 업데이트 가능)
        self.base_position = np.array([0.0, 0.0, 0.27])  # 기본 높이
        self.base_orientation = np.array([0.0, 0.0, 0.0])  # [roll, pitch, yaw]
        
        # 로봇 파라미터 (GO1 기하학적 파라미터)
        self.hip_offset = 0.08505   # Hip offset from center
        
        # Hip positions in body frame (base 중심 기준)
        self.hip_positions_body = {
            'FR': np.array([0.1881, -self.hip_offset, 0.0]),
            'FL': np.array([0.1881, self.hip_offset, 0.0]),
            'RR': np.array([-0.1881, -self.hip_offset, 0.0]),
            'RL': np.array([-0.1881, self.hip_offset, 0.0])
        }
        
        # 발행 카운터
        self.publish_count = 0
        
        self.get_logger().info('='*60)
        self.get_logger().info('✅ Hip Position Calculator 시작됨 (가벼운 버전)')
        self.get_logger().info('='*60)
        self.get_logger().info('  - Joint state 기반 Forward Kinematics')
        self.get_logger().info('  - Base position + body frame offsets로 계산')
        self.get_logger().info('='*60)
    
    def rotation_matrix_from_euler(self, roll: float, pitch: float, yaw: float) -> np.ndarray:
        """오일러 각도에서 회전 행렬 계산"""
        # Roll (X축 회전)
        R_x = np.array([
            [1, 0, 0],
            [0, math.cos(roll), -math.sin(roll)],
            [0, math.sin(roll), math.cos(roll)]
        ])
        
        # Pitch (Y축 회전)
        R_y = np.array([
            [math.cos(pitch), 0, math.sin(pitch)],
            [0, 1, 0],
            [-math.sin(pitch), 0, math.cos(pitch)]
        ])
        
        # Yaw (Z축 회전)
        R_z = np.array([
            [math.cos(yaw), -math.sin(yaw), 0],
            [math.sin(yaw), math.cos(yaw), 0],
            [0, 0, 1]
        ])
        
        # 전체 회전 행렬 (ZYX 순서)
        return R_z @ R_y @ R_x
    
    def calculate_hip_positions_world(self) -> Dict[str, np.ndarray]:
        """World frame 기준으로 hip positions 계산"""
        roll, pitch, yaw = self.base_orientation
        R = self.rotation_matrix_from_euler(roll, pitch, yaw)
        
        hip_positions_world = {}
        for leg, hip_pos_body in self.hip_positions_body.items():
            # Body frame에서 world frame으로 변환
            hip_pos_world = self.base_position + R @ hip_pos_body
            hip_positions_world[leg] = hip_pos_world
        
        return hip_positions_world
    
    def joint_state_callback(self, msg: JointState):
        """Joint state 수신 및 hip position 계산/발행"""
        # Joint state 업데이트
        for i, joint_name in enumerate(msg.name):
            if i < len(msg.position):
                self.current_joint_positions[joint_name] = float(msg.position[i])
        
        # Hip positions 계산
        hip_positions_world = self.calculate_hip_positions_world()
        
        # 각 hip position 발행
        now = self.get_clock().now()
        
        for leg, hip_pos in hip_positions_world.items():
            hip_name = f'{leg}_hip'
            
            if hip_name in self.hip_publishers:
                msg = PointStamped()
                msg.header.stamp = now.to_msg()
                msg.header.frame_id = 'world'
                msg.point.x = float(hip_pos[0])
                msg.point.y = float(hip_pos[1])
                msg.point.z = float(hip_pos[2])
                
                self.hip_publishers[hip_name].publish(msg)
        
        # 주기적으로 로그 출력 (5초마다)
        self.publish_count += 1
        if self.publish_count % 150 == 0:  # 30Hz * 5초
            self.get_logger().info('📊 Hip Positions 발행 중...')
            for leg, hip_pos in hip_positions_world.items():
                self.get_logger().info(f'   {leg}_hip: [{hip_pos[0]:.4f}, {hip_pos[1]:.4f}, {hip_pos[2]:.4f}] m')
    
    def update_base_position(self, position: np.ndarray, orientation: np.ndarray = None):
        """Base position 업데이트 (예: Isaac Sim에서 받아온 실제 값으로)"""
        self.base_position = position.copy()
        if orientation is not None:
            self.base_orientation = orientation.copy()
        self.get_logger().info(f'Base position 업데이트: {position}')


def main(args=None):
    rclpy.init(args=args)
    
    node = HipPositionCalculator()
    
    try:
        node.get_logger().info('\n🔄 Hip Position Calculator 실행 중... (Ctrl+C로 종료)\n')
        node.get_logger().info('   Joint state를 기다리는 중...\n')
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('\n\n✅ Hip Position Calculator 종료됨')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

