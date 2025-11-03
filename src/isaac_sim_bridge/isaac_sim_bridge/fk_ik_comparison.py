#!/usr/bin/env python3
"""
FK-IK 비교 테스트 노드

1. 현재 joint state에서 Forward Kinematics로 발 끝 좌표(a) 계산
2. Base에서부터 hip 좌표(b) 계산
3. b에서부터 a까지의 벡터를 기준으로 Analytic IK의 해 구하기
4. IK 해와 현재 joint position의 차이 비교
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import numpy as np
import math
from typing import Dict, Tuple, Optional


class FKIKComparison(Node):
    """FK-IK 비교 노드"""
    
    def __init__(self):
        super().__init__('fk_ik_comparison')
        
        # Joint state 구독
        self.joint_state_subscriber = self.create_subscription(
            JointState,
            'joint_states',
            self.joint_state_callback,
            10
        )
        
        # 현재 joint state 저장
        self.current_joint_positions: Dict[str, float] = {}
        
        # 로봇 파라미터 (GO1 기하학적 파라미터)
        self.hip_offset = 0.08505   # Hip offset from center
        self.upper_leg_length = 0.213  # Thigh length (m)
        self.lower_leg_length = 0.213  # Calf length (m)
        
        # Hip positions in body frame (base 중심 기준)
        self.hip_positions_body = {
            'FR': np.array([0.1881, -self.hip_offset, 0.0]),
            'FL': np.array([0.1881, self.hip_offset, 0.0]),
            'RR': np.array([-0.1881, -self.hip_offset, 0.0]),
            'RL': np.array([-0.1881, self.hip_offset, 0.0])
        }
        
        # Base position (기본값)
        self.base_position = np.array([0.0, 0.0, 0.27])  # 기본 높이
        self.base_orientation = np.array([0.0, 0.0, 0.0])  # [roll, pitch, yaw]
        
        # GO1 조인트 정의
        self.go1_joints = [
            'FR_hip_joint', 'FR_thigh_joint', 'FR_calf_joint',
            'FL_hip_joint', 'FL_thigh_joint', 'FL_calf_joint',
            'RR_hip_joint', 'RR_thigh_joint', 'RR_calf_joint',
            'RL_hip_joint', 'RL_thigh_joint', 'RL_calf_joint',
        ]
        
        # 비교 타이머 (1Hz)
        self.timer = self.create_timer(1.0, self.compare_fk_ik)
        
        self.get_logger().info('='*60)
        self.get_logger().info('✅ FK-IK 비교 노드 시작됨')
        self.get_logger().info('='*60)
        self.get_logger().info('  1. FK로 발 끝 좌표(a) 계산')
        self.get_logger().info('  2. Base에서 hip 좌표(b) 계산')
        self.get_logger().info('  3. b→a 벡터로 Analytic IK 해 구하기')
        self.get_logger().info('  4. IK 해 vs 현재 joint position 비교')
        self.get_logger().info('='*60)
    
    def rotation_matrix_from_euler(self, roll: float, pitch: float, yaw: float) -> np.ndarray:
        """오일러 각도에서 회전 행렬 계산"""
        R_x = np.array([
            [1, 0, 0],
            [0, math.cos(roll), -math.sin(roll)],
            [0, math.sin(roll), math.cos(roll)]
        ])
        R_y = np.array([
            [math.cos(pitch), 0, math.sin(pitch)],
            [0, 1, 0],
            [-math.sin(pitch), 0, math.cos(pitch)]
        ])
        R_z = np.array([
            [math.cos(yaw), -math.sin(yaw), 0],
            [math.sin(yaw), math.cos(yaw), 0],
            [0, 0, 1]
        ])
        return R_z @ R_y @ R_x
    
    def calculate_hip_positions_world(self) -> Dict[str, np.ndarray]:
        """World frame 기준으로 hip positions 계산"""
        roll, pitch, yaw = self.base_orientation
        R = self.rotation_matrix_from_euler(roll, pitch, yaw)
        
        hip_positions_world = {}
        for leg, hip_pos_body in self.hip_positions_body.items():
            hip_pos_world = self.base_position + R @ hip_pos_body
            hip_positions_world[leg] = hip_pos_world
        
        return hip_positions_world
    
    def forward_kinematics_leg(self, hip_angle: float, thigh_angle: float, calf_angle: float, leg: str) -> np.ndarray:
        """
        Forward Kinematics: 조인트 각도에서 발 끝 위치 계산 (hip 좌표계 기준)
        
        GO1의 조인트 구조:
        - Hip: Y축 회전 (좌우, hip_angle)
        - Thigh: 앞으로 뻗은 상태에서 X축 회전 (thigh_angle, 0=수평, 양수=아래)
        - Calf: 무릎 각도 (calf_angle, 음수=구부림)
        
        좌표계: Hip 기준
        - X: 앞쪽 (+)
        - Y: 좌우 (왼쪽 +, 오른쪽 -)
        - Z: 위아래 (위 +, 아래 -)
        
        Args:
            hip_angle: Hip joint angle (rad), Y축 회전
            thigh_angle: Thigh joint angle (rad), X축 회전
            calf_angle: Calf joint angle (rad), 무릎 각도 (음수)
            leg: 'FR', 'FL', 'RR', 'RL'
        
        Returns:
            Hip 좌표계에서의 발 끝 위치 [x, y, z]
        """
        l1 = self.upper_leg_length  # Thigh
        l2 = self.lower_leg_length  # Calf
        
        # Thigh 벡터 (hip에서 thigh 끝까지, hip 좌표계의 XY 평면에서 시작)
        # 초기: +X 방향 (앞쪽)
        # thigh_angle만큼 X축 주위로 회전 (0=앞쪽 수평, 양수=아래)
        thigh_local = np.array([
            l1 * math.cos(thigh_angle),  # X: 앞쪽
            0.0,                          # Y: 좌우
            -l1 * math.sin(thigh_angle)   # Z: 아래쪽 (음수)
        ])
        
        # Calf 벡터 (thigh 끝에서 발 끝까지)
        # total_angle = thigh_angle - calf_angle (calf_angle은 음수이므로 더 작아짐)
        total_angle = thigh_angle - calf_angle  # 실제 무릎 각도
        calf_local = np.array([
            l2 * math.cos(total_angle),
            0.0,
            -l2 * math.sin(total_angle)
        ])
        
        # Hip rotation (Y축 회전) 적용
        cos_h = math.cos(hip_angle)
        sin_h = math.sin(hip_angle)
        
        # Y축 회전 행렬
        R_y = np.array([
            [cos_h, 0, sin_h],
            [0, 1, 0],
            [-sin_h, 0, cos_h]
        ])
        
        # Thigh와 Calf를 hip rotation 적용
        thigh_rotated = R_y @ thigh_local
        calf_rotated = R_y @ calf_local
        
        # 최종 발 끝 위치 (hip 기준)
        foot_pos = thigh_rotated + calf_rotated
        
        return foot_pos
    
    def analytic_ik_leg(self, foot_pos_hip: np.ndarray, leg: str) -> Optional[Tuple[float, float, float]]:
        """
        Analytic IK: Hip 좌표계에서 발 끝 위치로부터 조인트 각도 계산
        
        Args:
            foot_pos_hip: Hip 좌표계에서의 발 끝 위치 [x, y, z]
            leg: 'FR', 'FL', 'RR', 'RL'
        
        Returns:
            (hip_angle, thigh_angle, calf_angle) 또는 None
        """
        # Numerical IK 사용 (더 정확함)
        return self.numerical_ik_leg(foot_pos_hip, leg)
    
    def numerical_ik_leg(self, foot_pos_hip: np.ndarray, leg: str, 
                         initial_guess: Optional[Tuple[float, float, float]] = None,
                         max_iterations: int = 100, tolerance: float = 1e-5) -> Optional[Tuple[float, float, float]]:
        """
        Numerical IK using Gradient Descent
        
        Args:
            foot_pos_hip: 목표 발 끝 위치 [x, y, z] (hip 좌표계)
            leg: 다리 이름
            initial_guess: 초기 추정값 (hip, thigh, calf)
            max_iterations: 최대 반복 횟수
            tolerance: 수렴 허용 오차
        
        Returns:
            (hip_angle, thigh_angle, calf_angle) 또는 None
        """
        # 초기 추정값 (없으면 0, 0.67, -1.3로 설정)
        if initial_guess is None:
            q = np.array([0.0, 0.67, -1.3])
        else:
            q = np.array(initial_guess)
        
        # 학습률
        alpha = 0.1
        
        for iteration in range(max_iterations):
            # FK로 현재 발 위치 계산
            current_foot = self.forward_kinematics_leg(q[0], q[1], q[2], leg)
            
            # 목표와의 차이
            error = foot_pos_hip - current_foot
            error_norm = np.linalg.norm(error)
            
            # 수렴 확인
            if error_norm < tolerance:
                return tuple(q)
            
            # Gradient 계산 (유한 차분법)
            epsilon = 1e-6
            gradients = []
            
            for i in range(3):
                q_perturbed = q.copy()
                q_perturbed[i] += epsilon
                foot_perturbed = self.forward_kinematics_leg(q_perturbed[0], q_perturbed[1], q_perturbed[2], leg)
                gradient = (foot_perturbed - current_foot) / epsilon
                gradients.append(gradient)
            
            # Jacobian 행렬 (3x3)
            J = np.array(gradients).T  # (3, 3) shape
            
            # Gradient descent 업데이트
            # delta_q = alpha * J^T * error
            delta_q = alpha * J.T @ error
            
            # 각도 제한 적용
            q_new = q + delta_q
            
            # 안전한 범위로 클리핑
            q_new[0] = np.clip(q_new[0], -math.pi/2, math.pi/2)  # hip
            q_new[1] = np.clip(q_new[1], 0, math.pi)  # thigh
            q_new[2] = np.clip(q_new[2], -math.pi, 0)  # calf (음수)
            
            q = q_new
        
        # 수렴하지 못함
        return None
    
    def joint_state_callback(self, msg: JointState):
        """Joint state 수신"""
        for i, joint_name in enumerate(msg.name):
            if i < len(msg.position):
                self.current_joint_positions[joint_name] = float(msg.position[i])
    
    def compare_fk_ik(self):
        """FK-IK 비교 수행"""
        if len(self.current_joint_positions) == 0:
            return
        
        # 1. Hip positions 계산 (b)
        hip_positions_world = self.calculate_hip_positions_world()
        
        self.get_logger().info('='*60)
        self.get_logger().info('📊 FK-IK 비교 결과')
        self.get_logger().info('='*60)
        
        leg_names = ['FR', 'FL', 'RR', 'RL']
        total_diff = 0.0
        total_count = 0
        
        for leg in leg_names:
            # 현재 joint angles
            hip_joint = f'{leg}_hip_joint'
            thigh_joint = f'{leg}_thigh_joint'
            calf_joint = f'{leg}_calf_joint'
            
            if not all(j in self.current_joint_positions for j in [hip_joint, thigh_joint, calf_joint]):
                continue
            
            current_hip = self.current_joint_positions[hip_joint]
            current_thigh = self.current_joint_positions[thigh_joint]
            current_calf = self.current_joint_positions[calf_joint]
            
            # 2. FK로 발 끝 좌표 계산 (a) - hip 좌표계 기준
            foot_pos_hip = self.forward_kinematics_leg(current_hip, current_thigh, current_calf, leg)
            
            # 3. Numerical IK로 해 구하기 (현재 joint를 초기 추정값으로 사용)
            initial_guess = (current_hip, current_thigh, current_calf)
            ik_result = self.numerical_ik_leg(foot_pos_hip, leg, initial_guess=initial_guess)
            
            if ik_result is None:
                self.get_logger().warn(f'{leg}: IK 해를 구할 수 없습니다.')
                continue
            
            ik_hip, ik_thigh, ik_calf = ik_result
            
            # IK 해를 검증: FK로 다시 계산해서 차이 확인
            ik_foot_pos = self.forward_kinematics_leg(ik_hip, ik_thigh, ik_calf, leg)
            ik_error = np.linalg.norm(foot_pos_hip - ik_foot_pos)
            
            # 4. 차이 계산
            diff_hip = abs(current_hip - ik_hip)
            diff_thigh = abs(current_thigh - ik_thigh)
            diff_calf = abs(current_calf - ik_calf)
            
            # 각도 차이를 라디안에서 도로 변환
            diff_hip_deg = math.degrees(diff_hip)
            diff_thigh_deg = math.degrees(diff_thigh)
            diff_calf_deg = math.degrees(diff_calf)
            
            total_diff += diff_hip_deg + diff_thigh_deg + diff_calf_deg
            total_count += 3
            
            # 출력
            self.get_logger().info(f'\n{leg} Leg:')
            self.get_logger().info(f'  발 끝 위치(hip 기준): [{foot_pos_hip[0]:.4f}, {foot_pos_hip[1]:.4f}, {foot_pos_hip[2]:.4f}] m')
            self.get_logger().info(f'  Hip 위치(world): [{hip_positions_world[leg][0]:.4f}, {hip_positions_world[leg][1]:.4f}, {hip_positions_world[leg][2]:.4f}] m')
            self.get_logger().info(f'  현재 joint: hip={current_hip:.4f}, thigh={current_thigh:.4f}, calf={current_calf:.4f} rad')
            self.get_logger().info(f'  IK 해:      hip={ik_hip:.4f}, thigh={ik_thigh:.4f}, calf={ik_calf:.4f} rad')
            self.get_logger().info(f'  IK 검증:    FK 오차={ik_error*1000:.3f} mm')
            self.get_logger().info(f'  차이:       hip={diff_hip_deg:.2f}°, thigh={diff_thigh_deg:.2f}°, calf={diff_calf_deg:.2f}°')
            
            if diff_hip_deg > 5.0 or diff_thigh_deg > 5.0 or diff_calf_deg > 5.0:
                self.get_logger().warn(f'  ⚠️  차이가 큽니다! (5도 이상)')
            else:
                self.get_logger().info(f'  ✅ 차이가 작습니다 (5도 이하)')
        
        if total_count > 0:
            avg_diff = total_diff / total_count
            self.get_logger().info('='*60)
            self.get_logger().info(f'평균 차이: {avg_diff:.2f}°')
            self.get_logger().info('='*60)
        
        self.get_logger().info('')


def main(args=None):
    rclpy.init(args=args)
    
    node = FKIKComparison()
    
    try:
        node.get_logger().info('\n🔄 FK-IK 비교 시작... (Ctrl+C로 종료)\n')
        node.get_logger().info('   Joint state를 기다리는 중...\n')
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('\n\n✅ FK-IK 비교 노드 종료됨')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

