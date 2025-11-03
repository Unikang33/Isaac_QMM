#!/usr/bin/env python3
"""
FK-IK 기반 Joint Command 컨트롤러

1. 현재 joint state 수신
2. FK로 발 끝 위치 계산 (hip 좌표계)
3. Numerical IK로 joint 값 재계산
4. 계산된 joint 값을 joint_command 토픽으로 발행
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import numpy as np
import math
from typing import Dict, Tuple, Optional


class FKIKController(Node):
    """FK-IK 기반 제어 노드"""
    
    def __init__(self):
        super().__init__('fk_ik_controller')
        
        # Joint state 구독
        self.joint_state_subscriber = self.create_subscription(
            JointState,
            'joint_states',
            self.joint_state_callback,
            10
        )
        
        # Joint command 발행
        self.joint_command_publisher = self.create_publisher(
            JointState,
            'joint_command',
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
        
        # K1 조인트 (home position 유지)
        self.k1_joints = [
            'joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6',
            'joint_gripper_left', 'joint_gripper_right'
        ]
        
        self.all_joints = self.go1_joints + self.k1_joints
        
        # K1 home positions
        self.k1_home_positions = {
            'joint1': 0.0, 'joint2': 0.0, 'joint3': 0.0, 'joint4': 0.0,
            'joint5': 0.0, 'joint6': 0.0, 'joint_gripper_left': 0.0, 'joint_gripper_right': 0.0
        }
        
        # 타이머 설정 (30Hz)
        self.timer = self.create_timer(1.0/30.0, self.control_loop)
        
        # 발행 카운터
        self.publish_count = 0
        
        self.get_logger().info('='*60)
        self.get_logger().info('✅ FK-IK Controller 시작됨')
        self.get_logger().info('='*60)
        self.get_logger().info('  - Joint state → FK → Numerical IK → Joint Command')
        self.get_logger().info('  - 제어 주기: 30Hz')
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
        """
        l1 = self.upper_leg_length  # Thigh
        l2 = self.lower_leg_length  # Calf
        
        # Thigh 벡터
        thigh_local = np.array([
            l1 * math.cos(thigh_angle),
            0.0,
            -l1 * math.sin(thigh_angle)
        ])
        
        # Calf 벡터
        total_angle = thigh_angle - calf_angle
        calf_local = np.array([
            l2 * math.cos(total_angle),
            0.0,
            -l2 * math.sin(total_angle)
        ])
        
        # Hip rotation (Y축 회전) 적용
        cos_h = math.cos(hip_angle)
        sin_h = math.sin(hip_angle)
        
        R_y = np.array([
            [cos_h, 0, sin_h],
            [0, 1, 0],
            [-sin_h, 0, cos_h]
        ])
        
        thigh_rotated = R_y @ thigh_local
        calf_rotated = R_y @ calf_local
        
        foot_pos = thigh_rotated + calf_rotated
        
        return foot_pos
    
    def numerical_ik_leg(self, foot_pos_hip: np.ndarray, leg: str, 
                         initial_guess: Optional[Tuple[float, float, float]] = None,
                         max_iterations: int = 100, tolerance: float = 1e-5) -> Optional[Tuple[float, float, float]]:
        """
        Numerical IK using Gradient Descent
        """
        if initial_guess is None:
            q = np.array([0.0, 0.67, -1.3])
        else:
            q = np.array(initial_guess)
        
        alpha = 0.1  # 학습률
        
        for iteration in range(max_iterations):
            current_foot = self.forward_kinematics_leg(q[0], q[1], q[2], leg)
            error = foot_pos_hip - current_foot
            error_norm = np.linalg.norm(error)
            
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
            
            J = np.array(gradients).T
            delta_q = alpha * J.T @ error
            
            q_new = q + delta_q
            
            # 각도 제한
            q_new[0] = np.clip(q_new[0], -math.pi/2, math.pi/2)  # hip
            q_new[1] = np.clip(q_new[1], 0, math.pi)  # thigh
            q_new[2] = np.clip(q_new[2], -math.pi, 0)  # calf
            
            q = q_new
        
        return None
    
    def joint_state_callback(self, msg: JointState):
        """Joint state 수신"""
        for i, joint_name in enumerate(msg.name):
            if i < len(msg.position):
                self.current_joint_positions[joint_name] = float(msg.position[i])
    
    def control_loop(self):
        """메인 제어 루프"""
        if len(self.current_joint_positions) == 0:
            return
        
        try:
            # Hip positions 계산
            hip_positions_world = self.calculate_hip_positions_world()
            
            # GO1 조인트 값 계산
            go1_joint_positions = []
            leg_names = ['FR', 'FL', 'RR', 'RL']
            
            for leg in leg_names:
                hip_joint = f'{leg}_hip_joint'
                thigh_joint = f'{leg}_thigh_joint'
                calf_joint = f'{leg}_calf_joint'
                
                if not all(j in self.current_joint_positions for j in [hip_joint, thigh_joint, calf_joint]):
                    # 기본값 사용
                    go1_joint_positions.extend([0.0, 0.67, -1.3])
                    continue
                
                # 현재 joint 값
                current_hip = self.current_joint_positions[hip_joint]
                current_thigh = self.current_joint_positions[thigh_joint]
                current_calf = self.current_joint_positions[calf_joint]
                
                # FK로 발 끝 위치 계산
                foot_pos_hip = self.forward_kinematics_leg(current_hip, current_thigh, current_calf, leg)
                
                # Numerical IK로 joint 값 재계산
                initial_guess = (current_hip, current_thigh, current_calf)
                ik_result = self.numerical_ik_leg(foot_pos_hip, leg, initial_guess=initial_guess)
                
                if ik_result is None:
                    # IK 실패 시 현재 값 사용
                    go1_joint_positions.extend([current_hip, current_thigh, current_calf])
                    self.get_logger().warn(f'{leg}: IK 실패, 현재 값 사용')
                else:
                    ik_hip, ik_thigh, ik_calf = ik_result
                    go1_joint_positions.extend([ik_hip, ik_thigh, ik_calf])
            
            # K1 조인트 위치 (home position)
            k1_joint_positions = [self.k1_home_positions[joint] for joint in self.k1_joints]
            
            # 전체 조인트 위치
            all_joint_positions = go1_joint_positions + k1_joint_positions
            
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
            self.publish_count += 1
            if self.publish_count % 150 == 0:  # 30Hz * 5초
                self.get_logger().info('📊 FK-IK 제어 중...')
                self.get_logger().info(f'   FR: hip={go1_joint_positions[0]:.3f}, thigh={go1_joint_positions[1]:.3f}, calf={go1_joint_positions[2]:.3f} rad')
                self.get_logger().info(f'   발행 중... (총 {self.publish_count}번)')
                
        except Exception as e:
            self.get_logger().error(f'Control loop error: {e}')


def main(args=None):
    rclpy.init(args=args)
    
    node = FKIKController()
    
    try:
        node.get_logger().info('\n🔄 FK-IK Controller 실행 중... (Ctrl+C로 종료)\n')
        node.get_logger().info('   Joint state를 기다리는 중...\n')
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('\n\n✅ FK-IK Controller 종료됨')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

