#!/usr/bin/env python3
"""
Hip Position 비교 테스트 노드

두 가지 방법으로 4개의 hip position을 계산하고 비교:
1. World frame 기준으로 base position + body frame offsets로 계산
2. Isaac Sim으로부터 TF를 통해 실제 hip position 받아오기
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Point, PointStamped
import numpy as np
import math
from typing import Dict, Optional
from geometry_msgs.msg import TransformStamped
import tf2_ros
from tf2_ros import TransformException


class HipPositionComparison(Node):
    """Hip Position 비교 노드"""
    
    def __init__(self):
        super().__init__('hip_position_comparison')
        
        # TF Buffer와 Listener 설정
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # Joint state 구독 (FK 계산용)
        self.joint_state_subscriber = self.create_subscription(
            JointState,
            'joint_states',
            self.joint_state_callback,
            10
        )
        
        # Isaac Sim에서 발행하는 hip position 구독 (Point 또는 PointStamped 모두 지원)
        self.hip_position_subscribers = {}
        for hip_name in ['FR_hip', 'FL_hip', 'RR_hip', 'RL_hip']:
            # PointStamped 먼저 시도
            self.hip_position_subscribers[hip_name] = self.create_subscription(
                PointStamped,
                f'{hip_name.lower()}_position',
                lambda msg, name=hip_name: self.hip_position_callback_stamped(msg, name),
                10
            )
        
        # 현재 joint state 저장
        self.current_joint_positions: Dict[str, float] = {}
        
        # Isaac Sim에서 받은 hip positions 저장
        self.isaac_hip_positions: Dict[str, np.ndarray] = {}
        
        # 로봇 파라미터 (GO1 기하학적 파라미터)
        self.hip_offset = 0.08505   # Hip offset from center
        
        # Hip positions in body frame (base 중심 기준)
        self.hip_positions_body = {
            'FR': np.array([0.1881, -self.hip_offset, 0.0]),
            'FL': np.array([0.1881, self.hip_offset, 0.0]),
            'RR': np.array([-0.1881, -self.hip_offset, 0.0]),
            'RL': np.array([-0.1881, self.hip_offset, 0.0])
        }
        
        # 현재 base position과 orientation (기본값)
        self.current_base_position = np.array([0.0, 0.0, 0.27])  # 기본값
        self.current_base_orientation = np.array([0.0, 0.0, 0.0])  # [roll, pitch, yaw]
        
        # Hip joint 이름 매핑 (TF frame 이름과 일치해야 함)
        self.hip_joint_names = {
            'FR': 'FR_hip_joint',
            'FL': 'FL_hip_joint',
            'RR': 'RR_hip_joint',
            'RL': 'RL_hip_joint'
        }
        
        # 비교 타이머 (1Hz)
        self.timer = self.create_timer(1.0, self.compare_hip_positions)
        
        # 로봇 파라미터
        self.upper_leg_length = 0.213  # Thigh length
        self.lower_leg_length = 0.213  # Calf length
        
        self.get_logger().info('='*60)
        self.get_logger().info('✅ Hip Position 비교 노드 시작됨')
        self.get_logger().info('='*60)
        self.get_logger().info('  방법 1: World frame 계산 (base position + body frame offsets)')
        self.get_logger().info('  방법 2: Isaac Sim TF 또는 Joint state 기반 FK 계산')
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
    
    def calculate_hip_positions_from_base(self) -> Dict[str, np.ndarray]:
        """World frame 기준으로 base position과 body frame offsets로 hip positions 계산"""
        roll, pitch, yaw = self.current_base_orientation
        R = self.rotation_matrix_from_euler(roll, pitch, yaw)
        
        hip_positions_world = {}
        for leg, hip_pos_body in self.hip_positions_body.items():
            # Body frame에서 world frame으로 변환
            hip_pos_world = self.current_base_position + R @ hip_pos_body
            hip_positions_world[leg] = hip_pos_world
        
        return hip_positions_world
    
    def joint_state_callback(self, msg: JointState):
        """Joint state 수신"""
        for i, joint_name in enumerate(msg.name):
            if i < len(msg.position):
                self.current_joint_positions[joint_name] = float(msg.position[i])
    
    def hip_position_callback_stamped(self, msg: PointStamped, hip_name: str):
        """Isaac Sim에서 발행한 hip position 수신 (PointStamped)"""
        self.isaac_hip_positions[hip_name] = np.array([msg.point.x, msg.point.y, msg.point.z])
    
    def hip_position_callback(self, msg: Point, hip_name: str):
        """Isaac Sim에서 발행한 hip position 수신 (Point - 호환성용)"""
        self.isaac_hip_positions[hip_name] = np.array([msg.x, msg.y, msg.z])
    
    def get_hip_positions_from_isaac_sim(self) -> Optional[Dict[str, np.ndarray]]:
        """
        Isaac Sim으로부터 hip positions 받아오기
        방법 1: Isaac Sim에서 직접 발행하는 토픽 구독 (우선)
        방법 2: TF를 통해 (대안)
        """
        # 방법 1: Isaac Sim에서 직접 발행한 hip position 토픽 확인
        if len(self.isaac_hip_positions) == 4:
            # Isaac Sim 노드에서 직접 발행한 hip position 사용
            hip_positions = {}
            for leg in ['FR', 'FL', 'RR', 'RL']:
                hip_name = f'{leg}_hip'
                if hip_name in self.isaac_hip_positions:
                    hip_positions[leg] = self.isaac_hip_positions[hip_name]
                else:
                    return None
            return hip_positions
        
        # 방법 2: TF를 통해 받아오기 시도
        hip_positions = {}
        tf_available = False
        
        try:
            # 가능한 base frame 이름들 시도
            base_frames = ['base_link', 'base', 'world', 'odom']
            base_frame = None
            
            for frame in base_frames:
                try:
                    # Frame이 존재하는지 확인
                    transform = self.tf_buffer.lookup_transform(
                        frame,
                        self.hip_joint_names['FR'],
                        rclpy.time.Time()
                    )
                    base_frame = frame
                    break
                except:
                    continue
            
            if base_frame:
                for leg, joint_name in self.hip_joint_names.items():
                    try:
                        transform = self.tf_buffer.lookup_transform(
                            base_frame,
                            joint_name,
                            rclpy.time.Time()
                        )
                        t = transform.transform.translation
                        hip_positions[leg] = np.array([t.x, t.y, t.z])
                        tf_available = True
                    except:
                        break
                        
        except Exception as e:
            pass
        
        if tf_available and len(hip_positions) == 4:
            return hip_positions
        
        return None
    
    def compare_hip_positions(self):
        """두 방법으로 계산한 hip positions 비교"""
        # 1. World frame 기준 계산
        calculated_hips = self.calculate_hip_positions_from_base()
        
        # 2. Isaac Sim으로부터 받아오기
        isaac_hips = self.get_hip_positions_from_isaac_sim()
        
        if isaac_hips is None:
            self.get_logger().warn('⚠️  Isaac Sim에서 hip position을 받아올 수 없습니다.')
            self.get_logger().warn('   TF가 발행되고 있는지 확인해주세요.')
            self.get_logger().info('')
            self.get_logger().info('📊 계산된 Hip Positions (World Frame):')
            for leg in ['FR', 'FL', 'RR', 'RL']:
                pos = calculated_hips[leg]
                self.get_logger().info(f'   {leg}: [{pos[0]:.4f}, {pos[1]:.4f}, {pos[2]:.4f}] m')
            return
        
        # 3. 비교 출력
        self.get_logger().info('='*60)
        self.get_logger().info('📊 Hip Position 비교 결과')
        self.get_logger().info('='*60)
        
        total_diff = 0.0
        for leg in ['FR', 'FL', 'RR', 'RL']:
            calc_pos = calculated_hips[leg]
            isaac_pos = isaac_hips[leg]
            
            # 차이 계산
            diff = np.linalg.norm(calc_pos - isaac_pos)
            total_diff += diff
            
            self.get_logger().info(f'\n{leg} Hip:')
            self.get_logger().info(f'  계산값:  [{calc_pos[0]:.4f}, {calc_pos[1]:.4f}, {calc_pos[2]:.4f}] m')
            self.get_logger().info(f'  Isaac:   [{isaac_pos[0]:.4f}, {isaac_pos[1]:.4f}, {isaac_pos[2]:.4f}] m')
            self.get_logger().info(f'  차이:     {diff:.4f} m')
            
            if diff > 0.01:  # 1cm 이상 차이
                self.get_logger().warn(f'  ⚠️  차이가 큽니다! ({diff*100:.2f} cm)')
            else:
                self.get_logger().info(f'  ✅ 차이가 작습니다 ({diff*100:.2f} cm)')
        
        self.get_logger().info('='*60)
        self.get_logger().info(f'평균 차이: {total_diff/4:.4f} m ({total_diff/4*100:.2f} cm)')
        self.get_logger().info('='*60)
        self.get_logger().info('')


def main(args=None):
    rclpy.init(args=args)
    
    node = HipPositionComparison()
    
    try:
        node.get_logger().info('\n🔄 Hip Position 비교 시작... (Ctrl+C로 종료)\n')
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('\n\n✅ Hip Position 비교 노드 종료됨')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

