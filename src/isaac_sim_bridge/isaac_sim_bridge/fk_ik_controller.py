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
        
        # GO1 다리 파라미터 (go1_kinematics.py와 동일)
        self.L1 = 0.08   # Hip to thigh length (hip offset)
        self.L2 = 0.213  # Thigh length  
        self.L3 = 0.213  # Calf length
        
        # 하위 호환성을 위한 별칭
        self.upper_leg_length = self.L2
        self.lower_leg_length = self.L3
        
        # Hip positions in body frame (base 중심 기준)
        self.hip_positions_body = {
            'FR': np.array([0.1881, -self.hip_offset, 0.0]),
            'FL': np.array([0.1881, self.hip_offset, 0.0]),
            'RR': np.array([-0.1881, -self.hip_offset, 0.0]),
            'RL': np.array([-0.1881, self.hip_offset, 0.0])
        }
        
        # Base position (기본값)
        self.base_position = np.array([0.0, 0.0, 0.33])  # 기본 높이
        self.base_orientation = np.array([0.0, 0.0, 0.0])  # [roll, pitch, yaw]
        
        # Base offset (수정 가능)
        self.base_z_offset = -0.05  # 5cm 낮아짐 (음수 = 아래)
        
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
        
        # Joint state 수신 플래그
        self.joint_state_received = False
        
        # 명령 발행 임계값 (1도 = 약 0.0175 rad)
        self.joint_diff_threshold = math.radians(1.0)  # 1도
        
        # 주기적 command 발행을 위한 타이머 (예: 100Hz = 10ms)
        self.timer = self.create_timer(0.01, self.timer_callback)  # 100Hz
        
        self.get_logger().info('='*60)
        self.get_logger().info('✅ FK-IK Controller 시작됨')
        self.get_logger().info('='*60)
        self.get_logger().info('  - Joint state → FK → Numerical IK → Joint Command')
        self.get_logger().info('  - 지속적으로 자세 유지 (100Hz)')
        self.get_logger().info(f'  - Base Z offset: {self.base_z_offset*100:.1f} cm')
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
        """
        World frame 기준으로 hip positions 계산 (base offset 적용)
        
        Base position에 z offset을 적용한 후 hip positions를 계산합니다.
        """
        # Base position에 offset 적용
        base_position_with_offset = self.base_position.copy()
        base_position_with_offset[2] += self.base_z_offset  # Z offset 적용
        
        roll, pitch, yaw = self.base_orientation
        R = self.rotation_matrix_from_euler(roll, pitch, yaw)
        
        hip_positions_world = {}
        for leg, hip_pos_body in self.hip_positions_body.items():
            hip_pos_world = base_position_with_offset + R @ hip_pos_body
            hip_positions_world[leg] = hip_pos_world
        
        return hip_positions_world
    
    def forward_kinematics_leg(self, hip_angle: float, thigh_angle: float, calf_angle: float, leg: str) -> np.ndarray:
        """
        Forward Kinematics: 조인트 각도에서 발 끝 위치 계산 (hip 좌표계 기준)
        go1_kinematics.py와 동일한 기하학적 모델 사용
        """
        # L1: Hip to thigh offset (Y축 방향)
        # L2: Thigh length
        # L3: Calf length
        
        # Step 1: Hip joint rotation (Y축 회전)
        cos_h = math.cos(hip_angle)
        sin_h = math.sin(hip_angle)
        
        # Step 2: Hip offset 위치 (hip joint에서 Y축으로 L1만큼)
        hip_offset_local = np.array([0.0, self.L1, 0.0])
        
        # Step 3: Thigh 벡터 (hip offset 끝에서 시작, XZ 평면에서 회전)
        thigh_local = np.array([
            self.L2 * math.cos(thigh_angle),
            0.0,
            -self.L2 * math.sin(thigh_angle)
        ])
        
        # Step 4: Calf 벡터 (thigh 끝에서 시작)
        total_angle = thigh_angle - calf_angle
        calf_local = np.array([
            self.L3 * math.cos(total_angle),
            0.0,
            -self.L3 * math.sin(total_angle)
        ])
        
        # Step 5: Hip rotation 적용
        R_y = np.array([
            [cos_h, 0, sin_h],
            [0, 1, 0],
            [-sin_h, 0, cos_h]
        ])
        
        hip_offset_rotated = R_y @ hip_offset_local
        thigh_rotated = R_y @ thigh_local
        calf_rotated = R_y @ calf_local
        
        # 발 끝 위치 = hip offset + thigh + calf
        foot_pos = hip_offset_rotated + thigh_rotated + calf_rotated
        
        return foot_pos
    
    def analytical_ik_leg(self, target_pos: np.ndarray, leg: str) -> Optional[Tuple[float, float, float]]:
        """
        Analytical inverse kinematics for GO1 leg using geometric approach
        (from go1_kinematics.py)
        
        Args:
            target_pos: 3D target position relative to hip joint [x, y, z]
            leg: Leg name ('FL', 'FR', 'RL', 'RR')
            
        Returns:
            3-DOF joint angles [hip, thigh, calf] or None if failed
        """
        x, y, z = target_pos[0], target_pos[1], target_pos[2]
        
        try:
            # Step 1: Calculate D = sqrt(z^2 + y^2 - L1^2)
            D_squared = z*z + y*y - self.L1*self.L1
            if D_squared < 0:
                # Target unreachable in hip joint range
                return None
            
            D = np.sqrt(D_squared)
            
            # Step 2: Calculate q1 (hip joint)
            # Determine if this is a left leg (FL, RL) or right leg (FR, RR)
            is_left_leg = leg in ['FL', 'RL']
            
            if abs(y) < 1e-6:  # y ≈ 0, special case
                q1 = 0.0
            elif y > 0:  # Left side
                if is_left_leg:
                    q1 = np.arctan2(y, abs(z)) + np.arctan2(D, self.L1) - math.pi/2
                else:  # Right leg on left side (unusual case)
                    q1 = np.arctan2(y, abs(z)) + np.arctan2(D, self.L1) - math.pi/2
            else:  # y < 0, Right side
                if is_left_leg:  # Left leg on right side (unusual case)
                    q1 = math.pi/2 - (np.arctan2(D, self.L1) + np.arctan2(abs(y), abs(z)))
                else:  # Right leg
                    q1 = math.pi/2 - (np.arctan2(D, self.L1) + np.arctan2(abs(y), abs(z)))
            
            # Step 3: Calculate G = sqrt(D^2 + x^2)
            G = np.sqrt(D*D + x*x)
            
            # Check reachability
            max_reach = self.L2 + self.L3
            min_reach = abs(self.L2 - self.L3)
            if G > max_reach or G < min_reach:
                return None
            
            # Step 4: Calculate gamma using law of cosines
            cos_gamma = (G*G - self.L2*self.L2 - self.L3*self.L3) / (-2 * self.L2 * self.L3)
            if abs(cos_gamma) > 1.0:
                return None
            
            gamma = np.arccos(cos_gamma)
            
            # Step 5: Calculate q3 (calf joint)
            q3 = gamma - math.pi
            
            # Step 6: Calculate beta
            sin_gamma = np.sin(gamma)
            if abs(sin_gamma) < 1e-6:  # gamma ≈ 0 or pi
                beta = 0.0
            else:
                sin_beta = self.L3 * sin_gamma / G
                if abs(sin_beta) > 1.0:
                    return None
                beta = np.arcsin(sin_beta)
            
            # Step 7: Calculate alpha
            alpha = np.arctan2(x, D)
            
            # Step 8: Calculate q2 (thigh joint)
            q2 = beta - alpha
            
            solution = np.array([q1, q2, q3])
            
            # Check joint limits
            joint_limits = {
                'hip': [-0.8, 0.8],      # ±45 degrees
                'thigh': [-1.0, 4.0],    # -60 to 230 degrees  
                'calf': [-2.7, -0.9]     # -155 to -50 degrees
            }
            
            limits = [joint_limits['hip'], joint_limits['thigh'], joint_limits['calf']]
            for i, (q, (q_min, q_max)) in enumerate(zip(solution, limits)):
                if not (q_min <= q <= q_max):
                    # Joint limit violation
                    return None
            
            return tuple(solution)
            
        except Exception as e:
            self.get_logger().debug(f'Analytical IK failed for {leg}: {e}')
            return None
    
    def numerical_ik_leg(self, foot_pos_hip: np.ndarray, leg: str, 
                         initial_guess: Optional[Tuple[float, float, float]] = None,
                         max_iterations: int = 200, tolerance: float = 1e-4) -> Optional[Tuple[float, float, float]]:
        """
        Numerical IK using Gradient Descent
        """
        if initial_guess is None:
            q = np.array([0.0, 0.67, -1.3])
        else:
            q = np.array(initial_guess)
        
        alpha = 0.05  # 학습률 (더 작게 조정)
        
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
            
            # Damped least squares로 더 안정적인 업데이트
            damping = 0.01
            JtJ = J.T @ J
            damped_JtJ = JtJ + damping * np.eye(3)
            delta_q = alpha * np.linalg.solve(damped_JtJ, J.T @ error)
            
            q_new = q + delta_q
            
            # 각도 제한
            q_new[0] = np.clip(q_new[0], -math.pi/2, math.pi/2)  # hip
            q_new[1] = np.clip(q_new[1], 0, math.pi)  # thigh
            q_new[2] = np.clip(q_new[2], -math.pi, 0)  # calf
            
            q = q_new
        
        # 최종 오차 확인
        final_foot = self.forward_kinematics_leg(q[0], q[1], q[2], leg)
        final_error = np.linalg.norm(foot_pos_hip - final_foot)
        if final_error < 0.01:  # 1cm 이내면 허용
            return tuple(q)
        
        return None
    
    def joint_state_callback(self, msg: JointState):
        """Joint state 수신 및 업데이트"""
        # Joint state 업데이트
        for i, joint_name in enumerate(msg.name):
            if i < len(msg.position):
                self.current_joint_positions[joint_name] = float(msg.position[i])
        
        # Joint state가 충분히 수신되었는지 확인
        if len(self.current_joint_positions) >= len(self.go1_joints):
            self.joint_state_received = True
    
    def timer_callback(self):
        """타이머 콜백: 주기적으로 joint command 계산 및 조건부 발행"""
        if self.joint_state_received:
            self.calculate_and_send_joint_command()
    
    def calculate_and_send_joint_command(self):
        """Joint command 계산 및 조건부 발행 (차이가 1도 이상일 때만)"""
        try:
            # 현재 base position에서 hip positions 계산 (offset 적용 전)
            roll, pitch, yaw = self.base_orientation
            R = self.rotation_matrix_from_euler(roll, pitch, yaw)
            current_hip_positions_world = {}
            for leg, hip_pos_body in self.hip_positions_body.items():
                current_hip_positions_world[leg] = self.base_position + R @ hip_pos_body
            
            # Base offset이 적용된 새로운 hip positions 계산
            new_hip_positions_world = self.calculate_hip_positions_world()
            
            # GO1 조인트 값 계산
            go1_joint_positions = []
            current_go1_joints = []
            leg_names = ['FR', 'FL', 'RR', 'RL']
            has_significant_change = False  # 1도 이상 변화 여부
            
            for leg in leg_names:
                hip_joint = f'{leg}_hip_joint'
                thigh_joint = f'{leg}_thigh_joint'
                calf_joint = f'{leg}_calf_joint'
                
                if not all(j in self.current_joint_positions for j in [hip_joint, thigh_joint, calf_joint]):
                    # 기본값 사용
                    go1_joint_positions.extend([0.0, 0.67, -1.3])
                    current_go1_joints.extend([0.0, 0.67, -1.3])
                    continue
                
                # 현재 joint 값
                current_hip = self.current_joint_positions[hip_joint]
                current_thigh = self.current_joint_positions[thigh_joint]
                current_calf = self.current_joint_positions[calf_joint]
                current_go1_joints.extend([current_hip, current_thigh, current_calf])
                
                # 1. FK로 현재 발 끝 위치 계산 (현재 hip 좌표계 기준)
                current_foot_pos_hip = self.forward_kinematics_leg(current_hip, current_thigh, current_calf, leg)
                
                # 2. 현재 발 끝의 world frame 위치 계산
                # 현재 hip position 기준으로 world frame 변환
                current_hip_pos_world = current_hip_positions_world[leg]
                current_foot_pos_world = current_hip_pos_world + R @ current_foot_pos_hip
                
                # 3. 새로운 hip position (base offset 적용된) 기준으로
                #    현재 발 끝 world 위치를 새로운 hip 좌표계로 변환
                new_hip_pos_world = new_hip_positions_world[leg]
                foot_pos_relative_to_new_hip = current_foot_pos_world - new_hip_pos_world
                
                # World frame에서 hip 좌표계로 변환 (회전 역변환)
                R_inv = R.T  # 회전 행렬의 역행렬 = 전치 행렬
                foot_pos_new_hip = R_inv @ foot_pos_relative_to_new_hip
                
                # 4. IK로 새로운 joint 값 계산 (Analytical IK 우선, 실패 시 Numerical IK)
                ik_result = None
                
                # 먼저 Analytical IK 시도
                ik_result = self.analytical_ik_leg(foot_pos_new_hip, leg)
                
                # Analytical IK 실패 시 Numerical IK로 폴백
                if ik_result is None or np.allclose(ik_result, [0.0, 0.0, 0.0]):
                    initial_guess = (current_hip, current_thigh, current_calf)
                    ik_result = self.numerical_ik_leg(foot_pos_new_hip, leg, initial_guess=initial_guess)
                
                if ik_result is None or np.allclose(ik_result, [0.0, 0.0, 0.0]):
                    # IK 실패 시 현재 값 사용
                    go1_joint_positions.extend([current_hip, current_thigh, current_calf])
                else:
                    ik_hip, ik_thigh, ik_calf = ik_result
                    go1_joint_positions.extend([ik_hip, ik_thigh, ik_calf])
                    
                    # 현재 값과 IK 해의 차이 확인 (1도 이상 차이 체크)
                    hip_diff = abs(ik_hip - current_hip)
                    thigh_diff = abs(ik_thigh - current_thigh)
                    calf_diff = abs(ik_calf - current_calf)
                    
                    if (hip_diff > self.joint_diff_threshold or 
                        thigh_diff > self.joint_diff_threshold or 
                        calf_diff > self.joint_diff_threshold):
                        has_significant_change = True
                        
                        # 검증: IK 해가 올바른지 확인
                        verify_foot = self.forward_kinematics_leg(ik_hip, ik_thigh, ik_calf, leg)
                        verify_error = np.linalg.norm(foot_pos_new_hip - verify_foot)
                        if verify_error > 0.01:  # 1cm 이상 오차
                            self.get_logger().warn(f'{leg}: IK 검증 오차={verify_error*1000:.1f}mm')
            
            # 1도 이상 차이가 있는 경우에만 명령 발행
            if has_significant_change:
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
                
                # 결과 출력 (처음 한 번만, 이후에는 debug 레벨)
                if not hasattr(self, '_first_command'):
                    self.get_logger().info('='*60)
                    self.get_logger().info('✅ Joint Command 발행 시작')
                    self.get_logger().info('='*60)
                    self.get_logger().info(f'  Base Z offset 적용: {self.base_z_offset*100:.1f} cm')
                    self.get_logger().info(f'  조정된 Base 높이: {self.base_position[2] + self.base_z_offset:.3f} m')
                    self.get_logger().info(f'  Joint 차이 임계값: {math.degrees(self.joint_diff_threshold):.1f}도')
                    self.get_logger().info('  지속적으로 자세 유지 중...')
                    self.get_logger().info('='*60)
                    self._first_command = True
                else:
                    # 변경된 joint 정보 로그 (debug 레벨)
                    changes = []
                    for i, leg in enumerate(leg_names):
                        idx = i * 3
                        for j, joint_name in enumerate(['hip', 'thigh', 'calf']):
                            curr = current_go1_joints[idx + j]
                            new = go1_joint_positions[idx + j]
                            diff = abs(new - curr)
                            if diff > self.joint_diff_threshold:
                                changes.append(f'{leg}_{joint_name}: {math.degrees(diff):.1f}°')
                    
                    if changes:
                        self.get_logger().debug(f'Joint command 발행 (변경: {", ".join(changes)})')
            # else:
            #     # 1도 미만 차이는 발행하지 않음 (로그도 출력하지 않음)
            #     pass
                
        except Exception as e:
            self.get_logger().error(f'Error calculating/sending joint command: {e}')


def main(args=None):
    rclpy.init(args=args)
    
    node = FKIKController()
    
    try:
        node.get_logger().info('\n🔄 FK-IK Controller 실행 중...\n')
        node.get_logger().info('   Joint state를 기다리는 중...\n')
        
        # Joint state가 올 때까지 대기 (최대 10초)
        timeout = 10.0  # seconds
        start_time = node.get_clock().now()
        
        while not node.joint_state_received:
            rclpy.spin_once(node, timeout_sec=0.1)
            
            # 타임아웃 확인
            elapsed = (node.get_clock().now() - start_time).nanoseconds / 1e9
            if elapsed > timeout:
                node.get_logger().error(f'❌ 타임아웃: {timeout}초 내에 joint state를 받지 못했습니다.')
                break
        
        if node.joint_state_received:
            node.get_logger().info('\n✅ Joint state 수신됨, 지속적으로 자세 유지 중...\n')
            node.get_logger().info('   종료하려면 Ctrl+C를 누르세요.\n')
            
            # 지속적으로 실행 (타이머가 주기적으로 command 발행)
            rclpy.spin(node)
        else:
            node.get_logger().error('\n❌ Joint state 수신 실패')
            
    except KeyboardInterrupt:
        node.get_logger().info('\n\n⚠️  사용자에 의해 중단됨')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

