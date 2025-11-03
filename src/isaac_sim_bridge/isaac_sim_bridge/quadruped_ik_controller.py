#!/usr/bin/env python3
"""
사족로봇 IK 컨트롤러 - World 좌표계 기반 단순 IK

구현 내용:
1. 현재 world frame 기준 GO1 base 좌표 받아오기
2. 현재 world frame 기준 발 끝 좌표 받아오기  
3. Base에서 4개 hip까지의 상대 거리로 hip position 계산
4. Hip position to 발 끝 기준 analytic IK 계산
5. Joint command publish
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import numpy as np
import math
from typing import Dict, List, Tuple, Optional


class QuadrupedIKController(Node):
    """사족로봇 IK 컨트롤러 - 단순화된 버전"""
    
    def __init__(self):
        super().__init__('quadruped_ik_controller')
        
        # ROS2 Publisher - Joint commands
        self.joint_command_publisher = self.create_publisher(
            JointState,
            'joint_command',
            10
        )
        
        # ROS2 Subscriber - Joint states (현재 상태 수신)
        self.joint_state_subscriber = self.create_subscription(
            JointState,
            'joint_states',
            self.joint_state_callback,
            10
        )
        
        # 조인트 정의 (GO1)
        self.go1_joints = [
            'FR_hip_joint', 'FR_thigh_joint', 'FR_calf_joint',
            'FL_hip_joint', 'FL_thigh_joint', 'FL_calf_joint',
            'RR_hip_joint', 'RR_thigh_joint', 'RR_calf_joint',
            'RL_hip_joint', 'RL_thigh_joint', 'RL_calf_joint',
        ]
        
        # K1 조인트 (home position 유지)
        self.k1_joints = [
            'joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6',
            'joint_gripper_left', 'joint_gripper_right'
        ]
        
        self.all_joints = self.go1_joints + self.k1_joints
        
        # 로봇 파라미터 (GO1 기하학적 파라미터)
        self.hip_offset = 0.08505   # Hip offset from center
        self.upper_leg_length = 0.213  # Thigh length
        self.lower_leg_length = 0.213  # Calf length
        
        # Hip positions in body frame (base 중심 기준)
        self.hip_positions_body = {
            'FR': np.array([0.1881, -self.hip_offset, 0.0]),
            'FL': np.array([0.1881, self.hip_offset, 0.0]),
            'RR': np.array([-0.1881, -self.hip_offset, 0.0]),
            'RL': np.array([-0.1881, self.hip_offset, 0.0])
        }
        
        # 현재 상태 저장
        self.current_joint_positions: Dict[str, float] = {}
        self.current_joint_velocities: Dict[str, float] = {}
        
        # 현재 world frame 좌표 (Isaac Sim에서 받아올 예정)
        self.current_base_position = np.array([0.0, 0.0, 0.27])  # 기본값
        self.current_base_orientation = np.array([0.0, 0.0, 0.0])  # [roll, pitch, yaw]
        
        # 목표 foot positions (world frame) - 기본값으로 base 아래 설정
        self.target_foot_positions_world = {
            'FR': np.array([0.1881, -0.08505, -0.05]),  # base 아래 5cm
            'FL': np.array([0.1881, 0.08505, -0.05]),
            'RR': np.array([-0.1881, -0.08505, -0.05]),
            'RL': np.array([-0.1881, 0.08505, -0.05])
        }
        
        # K1 home positions
        self.k1_home_positions = {
            'joint1': 0.0, 'joint2': 0.0, 'joint3': 0.0, 'joint4': 0.0,
            'joint5': 0.0, 'joint6': 0.0, 'joint_gripper_left': 0.0, 'joint_gripper_right': 0.0
        }
        
        # 타이머 설정 (30Hz)
        self.timer = self.create_timer(1.0/30.0, self.control_loop)
        
        # 메시지 카운터
        self.message_count = 0
        
        self.get_logger().info('='*60)
        self.get_logger().info('✅ Quadruped IK Controller 시작됨 (단순화 버전)')
        self.get_logger().info('='*60)
        self.get_logger().info(f'  - 로봇 파라미터: thigh={self.upper_leg_length}m, calf={self.lower_leg_length}m')
        self.get_logger().info(f'  - 제어 주기: 30Hz')
        self.get_logger().info(f'  - 현재 base position: {self.current_base_position}')
        self.get_logger().info('='*60)
    
    def joint_state_callback(self, msg: JointState):
        """현재 조인트 상태 수신"""
        for i, joint_name in enumerate(msg.name):
            if i < len(msg.position):
                self.current_joint_positions[joint_name] = float(msg.position[i])
            if i < len(msg.velocity):
                self.current_joint_velocities[joint_name] = float(msg.velocity[i])
        
        # 첫 번째 메시지에서 현재 상태 출력
        if len(self.current_joint_positions) > 0 and self.message_count == 0:
            self.get_logger().info('✅ 첫 번째 joint state 수신 완료')
            self.message_count += 1
    
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
        """현재 base position에서 hip positions 계산 (world frame)"""
        roll, pitch, yaw = self.current_base_orientation
        R = self.rotation_matrix_from_euler(roll, pitch, yaw)
        
        hip_positions_world = {}
        for leg, hip_pos_body in self.hip_positions_body.items():
            # Body frame에서 world frame으로 변환
            hip_pos_world = self.current_base_position + R @ hip_pos_body
            hip_positions_world[leg] = hip_pos_world
        
        return hip_positions_world
    
    def analytic_ik_leg(self, foot_pos_hip: np.ndarray, leg: str) -> Optional[Tuple[float, float, float]]:
        """
        단일 다리에 대한 Analytic IK 계산
        
        Args:
            foot_pos_hip: Hip 좌표계에서의 foot position [x, y, z]
            leg: 다리 이름 ('FR', 'FL', 'RR', 'RL')
        
        Returns:
            (hip_angle, thigh_angle, calf_angle) 또는 None (해가 없는 경우)
        """
        x, y, z = foot_pos_hip
        
        # Hip angle 계산
        if leg in ['FR', 'RR']:  # Right legs
            hip_angle = math.atan2(-z, y)
        else:  # Left legs (FL, RL)
            hip_angle = math.atan2(-z, -y)
        
        # Hip joint 이후의 거리 계산
        if leg in ['FR', 'RR']:
            r = math.sqrt(y**2 + z**2) - self.hip_offset
        else:
            r = math.sqrt(y**2 + z**2) - self.hip_offset
        
        # Thigh, Calf angle 계산 (2D IK)
        l1 = self.upper_leg_length  # Thigh
        l2 = self.lower_leg_length  # Calf
        
        # 목표점까지의 거리
        target_distance = math.sqrt(x**2 + r**2)
        
        # 도달 가능성 검사
        if target_distance > (l1 + l2) or target_distance < abs(l1 - l2):
            self.get_logger().warn(f'{leg} leg: Target unreachable, distance={target_distance:.3f}')
            return None
        
        # Cosine law로 각도 계산
        cos_knee = (l1**2 + l2**2 - target_distance**2) / (2 * l1 * l2)
        cos_knee = np.clip(cos_knee, -1.0, 1.0)
        knee_angle = math.acos(cos_knee)
        
        # Thigh angle
        alpha = math.atan2(r, x)
        cos_alpha = (l1**2 + target_distance**2 - l2**2) / (2 * l1 * target_distance)
        cos_alpha = np.clip(cos_alpha, -1.0, 1.0)
        beta = math.acos(cos_alpha)
        thigh_angle = alpha + beta
        
        # Calf angle (knee angle를 음수로)
        calf_angle = -(math.pi - knee_angle)
        
        return (hip_angle, thigh_angle, calf_angle)
    
    def calculate_target_joint_positions(self) -> np.ndarray:
        """목표 조인트 위치 계산"""
        # 현재 base position에서 hip positions 계산
        hip_positions_world = self.calculate_hip_positions_world()
        
        # 각 다리에 대해 IK 계산
        joint_positions = []
        leg_names = ['FR', 'FL', 'RR', 'RL']
        
        for leg in leg_names:
            hip_pos_world = hip_positions_world[leg]
            foot_pos_world = self.target_foot_positions_world[leg]
            
            # Hip 좌표계에서의 foot position (hip에서 foot까지의 벡터)
            foot_pos_hip = foot_pos_world - hip_pos_world
            
            # IK 계산
            ik_result = self.analytic_ik_leg(foot_pos_hip, leg)
            
            if ik_result is not None:
                hip_angle, thigh_angle, calf_angle = ik_result
                joint_positions.extend([hip_angle, thigh_angle, calf_angle])
            else:
                # IK 실패 시 기본값 사용
                self.get_logger().warn(f'{leg} leg IK failed, using default values')
                joint_positions.extend([0.0, 0.67, -1.3])  # Stand pose
        
        return np.array(joint_positions)
    
    def control_loop(self):
        """메인 제어 루프 (30Hz)"""
        try:
            # 조인트 상태가 수신되었는지 확인
            if len(self.current_joint_positions) == 0:
                return  # 아직 joint state를 받지 못함
            
            # GO1 조인트 위치 계산
            go1_joint_positions = self.calculate_target_joint_positions()
            
            # K1 조인트 위치 (home position)
            k1_joint_positions = [self.k1_home_positions[joint] for joint in self.k1_joints]
            
            # 전체 조인트 위치
            all_joint_positions = list(go1_joint_positions) + k1_joint_positions
            
            # Joint command 메시지 생성
            msg = JointState()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'base_link'
            msg.name = self.all_joints
            msg.position = all_joint_positions
            msg.velocity = []
            msg.effort = []
            
            # 발행
            self.joint_command_publisher.publish(msg)
            
            # 주기적으로 상태 출력 (5초마다)
            self.message_count += 1
            if self.message_count % 150 == 0:  # 30Hz * 5초
                self.get_logger().info(f'📊 IK 제어 중... (총 {self.message_count}번 계산)')
                self.get_logger().info(f'   현재 base: {self.current_base_position}')
                self.get_logger().info(f'   FR 조인트: [{go1_joint_positions[0]:.3f}, {go1_joint_positions[1]:.3f}, {go1_joint_positions[2]:.3f}]')
            
        except Exception as e:
            self.get_logger().error(f'Control loop error: {e}')
    
    def set_target_foot_position(self, leg: str, position: np.ndarray):
        """목표 foot position 설정 (world frame)"""
        if leg in self.target_foot_positions_world:
            self.target_foot_positions_world[leg] = position.copy()
            self.get_logger().info(f'{leg} foot target updated: {position}')
    
    def update_base_position(self, position: np.ndarray, orientation: np.ndarray = None):
        """현재 base position 업데이트"""
        self.current_base_position = position.copy()
        if orientation is not None:
            self.current_base_orientation = orientation.copy()
        self.get_logger().info(f'Base position updated: {position}')


def main(args=None):
    rclpy.init(args=args)
    
    node = QuadrupedIKController()
    
    try:
        node.get_logger().info('\n🔄 Quadruped IK Controller 실행 중... (Ctrl+C로 종료)\n')
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('\n\n✅ Quadruped IK Controller 종료됨')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
