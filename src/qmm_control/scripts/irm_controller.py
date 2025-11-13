#!/usr/bin/env python3
"""
IRM 기반 Joint Command 컨트롤러 (단일 자세)
~/secret_usd/QMM_final.usd 사용

1. 현재 joint state 수신
2. TF에서 base pose와 foot position 수신
3. 단일 offset을 적용한 목표 자세 계산
4. Analytical IK로 joint 값 계산
5. 계산된 joint 값을 joint_command 토픽으로 발행
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from tf2_ros import TransformListener, Buffer
from geometry_msgs.msg import TransformStamped
import numpy as np
import math
from typing import Dict, Tuple, Optional
import os


class IRMController(Node):
    """IRM 기반 단일 자세 제어 노드"""
    
    def __init__(self):
        super().__init__('irm_controller')
        
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
        
        # 디버깅 출력 제어
        self.last_debug_time = 0.0
        self.debug_interval = 1.0  # 1초마다 출력
        self.last_pose_debug_time = 0.0
        self.pose_debug_interval = 2.0  # 2초마다 base pose 출력
        
        # TF Buffer와 Listener 생성
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # TF frame 이름
        self.world_frame = 'world'
        self.base_frame = 'base'
        
        # 다리별 hip과 foot frame 이름
        self.hip_frames = {
            'FR': 'FR_hip',
            'FL': 'FL_hip',
            'RR': 'RR_hip',
            'RL': 'RL_hip'
        }
        self.foot_frames = {
            'FR': ['FR_foot', 'FR_foot_link', 'go1_FR_foot'],
            'FL': ['FL_foot', 'FL_foot_link', 'go1_FL_foot'],
            'RR': ['RR_foot', 'RR_foot_link', 'go1_RR_foot'],
            'RL': ['RL_foot', 'RL_foot_link', 'go1_RL_foot']
        }
        
        # TF에서 받은 실제 hip 위치를 저장
        self.hip_positions_body_from_tf = {}
        self.hip_positions_initialized = False
        
        # 로봇 파라미터 (GO1 기하학적 파라미터)
        self.hip_offset = 0.08505
        self.L1 = 0.08   # Hip to thigh length
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
        
        # Base position과 orientation (TF에서 수신)
        self.base_position = np.array([0.0, 0.0, 0.33])
        self.base_orientation = np.array([0.0, 0.0, 0.0])  # [roll, pitch, yaw]
        self.tf_received = False
        
        # ============================================================
        # 목표 base pose 설정
        # ============================================================
        self.initial_base_position = None  # TF에서 받은 초기 위치
        self.initial_base_orientation = None  # TF에서 받은 초기 방향
        
        # ===== 옵션 1: Offset 방식 (초기 자세 기준 상대적 변화) =====
        # self.use_absolute_target = False
        # self.base_position_offset = np.array([0.0, 0.0, 0.0])  # [x, y, z] in meters
        # self.base_orientation_offset = np.array([0.0, 0.0, 0.0])  # [roll, pitch, yaw] in radians
        
        # ===== 옵션 2: 절대 좌표 방식 (World frame 기준 목표 자세) =====
        self.use_absolute_target = True
        self.desired_base_position = np.array([-0.100000, 0.250000, 0.395835])  # [x, y, z] in meters (world frame)
        self.desired_base_orientation = np.array([-0.155593, 0.021954, 0.0])  # [roll, pitch, yaw] in radians
        
        # 내부 계산용 (자동 계산됨)
        self.base_position_offset = np.array([0.0, 0.0, 0.0])
        self.base_orientation_offset = np.array([0.0, 0.0, 0.0])
        
        # 목표 base pose (초기화 시 설정)
        self.target_base_position = None
        self.target_base_orientation = None
        self.target_pose_initialized = False
        
        # 목표 foot position
        self.target_foot_positions_world = {}
        self.target_foot_positions_initialized = False
        
        # GO1 조인트 정의
        self.go1_joints = [
            'FR_hip_joint', 'FR_thigh_joint', 'FR_calf_joint',
            'FL_hip_joint', 'FL_thigh_joint', 'FL_calf_joint',
            'RR_hip_joint', 'RR_thigh_joint', 'RR_calf_joint',
            'RL_hip_joint', 'RL_thigh_joint', 'RL_calf_joint',
        ]
        
        # K1 조인트
        self.k1_joints = [
            'joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6',
            'joint_gripper_left', 'joint_gripper_right'
        ]
        
        self.all_joints = self.go1_joints + self.k1_joints
        
        # ============================================================
        # K1 목표 조인트 값 (직접 입력, 단위: radians)
        # ============================================================
        self.k1_target_positions = {
            'joint1': 1.595481,  # 91.414328°
            'joint2': -1.010969,  # -57.924242°
            'joint3': -0.080150,  # -4.592231°
            'joint4': 1.599731,  # 91.657859°
            'joint5': 1.561074,  # 89.442971°
            'joint6': -0.323965,  # -18.561848°
            'joint_gripper_left': 0.0,
            'joint_gripper_right': 0.0
        }


        # self.k1_target_positions = {
        #     'joint1': 2.208885,  # 126.559770°
        #     'joint2': 0.016313,  # 0.934653°
        #     'joint3': -0.225558,  # -12.923504°
        #     'joint4': -1.664043,  # -95.342621°
        #     'joint5': -2.190093,  # -125.483083°
        #     'joint6': 1.485634,  # 85.120535°
        #     'joint_gripper_left': 0.0,
        #     'joint_gripper_right': 0.0
        # }




# 'joint_gripper_left': 0.0,
# 'joint_gripper_right': 0.0
        self.get_logger().info('='*60)
        self.get_logger().info('🤖 K1 매니퓰레이터 목표 조인트 값:')
        for joint_name, value in self.k1_target_positions.items():
            if 'joint' in joint_name and 'gripper' not in joint_name:
                self.get_logger().info(f'  {joint_name}: {math.degrees(value):.2f}° ({value:.4f} rad)')
        self.get_logger().info('='*60)
        
        # Joint state 수신 플래그
        self.joint_state_received = False
        
        # 명령 발행 임계값
        self.joint_diff_threshold = math.radians(1.0)  # 1도
        
        # Joint 값 변화량 제한
        self.max_joint_change_per_step = math.radians(0.5)  # 0.5도/스텝 (GO1용 - 천천히)
        self.max_k1_joint_change_per_step = math.radians(0.5)  # 0.5도/스텝 (K1용 - 천천히)
        
        # 이전 joint positions 저장
        self.previous_joint_positions = {}
        
        # K1 매니퓰레이터 제어 관련
        self.k1_start_delay = 3.0  # 3초 대기 후 K1 동작 시작
        self.k1_control_start_time = None  # K1 제어 시작 시간
        self.k1_enabled = False  # K1 제어 활성화 플래그
        
        # K1 현재 조인트 값 (home position에서 시작)
        self.k1_current_positions = {
            'joint1': 0.0,
            'joint2': 0.0,
            'joint3': 0.0,
            'joint4': 0.0,
            'joint5': 0.0,
            'joint6': 0.0,
            'joint_gripper_left': 0.0,
            'joint_gripper_right': 0.0
        }
        
        # 주기적 command 발행을 위한 타이머 (10Hz)
        self.timer = self.create_timer(0.1, self.timer_callback)
        
        self.get_logger().info('='*60)
        self.get_logger().info('✅ IRM Controller 시작됨 (단일 자세)')
        self.get_logger().info('='*60)
        self.get_logger().info('  - Joint state → TF → Analytical IK → Joint Command')
        self.get_logger().info('  - 목표 자세 유지 (10Hz)')
        self.get_logger().info(f'  - K1 매니퓰레이터: {self.k1_start_delay}초 대기 후 천천히 동작 시작')
        
        if self.use_absolute_target:
            self.get_logger().info('  - 모드: 절대 좌표 방식 (World frame 기준)')
            self.get_logger().info(f'  - 목표 위치: [{self.desired_base_position[0]:.3f}, {self.desired_base_position[1]:.3f}, {self.desired_base_position[2]:.3f}] m')
            self.get_logger().info(f'  - 목표 회전: [{math.degrees(self.desired_base_orientation[0]):.1f}°, {math.degrees(self.desired_base_orientation[1]):.1f}°, {math.degrees(self.desired_base_orientation[2]):.1f}°]')
        else:
            self.get_logger().info('  - 모드: Offset 방식 (초기 자세 기준)')
            self.get_logger().info(f'  - Position offset: [{self.base_position_offset[0]:.3f}, {self.base_position_offset[1]:.3f}, {self.base_position_offset[2]:.3f}] m')
            self.get_logger().info(f'  - Orientation offset: [{math.degrees(self.base_orientation_offset[0]):.1f}°, {math.degrees(self.base_orientation_offset[1]):.1f}°, {math.degrees(self.base_orientation_offset[2]):.1f}°]')
        
        self.get_logger().info('='*60)
    
    def quaternion_to_euler(self, x, y, z, w):
        """쿼터니언을 오일러 각도(roll, pitch, yaw)로 변환"""
        # Roll (x-axis rotation)
        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = math.atan2(sinr_cosp, cosr_cosp)
        
        # Pitch (y-axis rotation)
        sinp = 2 * (w * y - z * x)
        if abs(sinp) >= 1:
            pitch = math.copysign(math.pi / 2, sinp)
        else:
            pitch = math.asin(sinp)
        
        # Yaw (z-axis rotation)
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        
        return roll, pitch, yaw
    
    def get_tf_position(self, target_frame: str, source_frame: str = 'world', timeout: float = 0.1) -> Optional[np.ndarray]:
        """TF에서 특정 frame의 world 위치를 가져옴"""
        try:
            transform: TransformStamped = self.tf_buffer.lookup_transform(
                source_frame,
                target_frame,
                rclpy.time.Time()
            )
            trans = transform.transform.translation
            return np.array([trans.x, trans.y, trans.z])
        except Exception:
            try:
                transform: TransformStamped = self.tf_buffer.lookup_transform(
                    source_frame,
                    target_frame,
                    rclpy.time.Time(seconds=0)
                )
                trans = transform.transform.translation
                return np.array([trans.x, trans.y, trans.z])
            except Exception:
                return None
    
    def get_tf_position_try_multiple(self, frame_names: list, source_frame: str = 'world') -> tuple[Optional[np.ndarray], Optional[str]]:
        """여러 frame 이름을 시도하여 TF 위치를 가져옴"""
        for frame_name in frame_names:
            pos = self.get_tf_position(frame_name, source_frame)
            if pos is not None:
                return pos, frame_name
        return None, None
    
    def update_base_pose_from_tf(self):
        """TF에서 base pose를 읽어와서 업데이트"""
        try:
            transform: TransformStamped = self.tf_buffer.lookup_transform(
                self.world_frame,
                self.base_frame,
                rclpy.time.Time()
            )
            
            # Translation
            trans = transform.transform.translation
            self.base_position = np.array([trans.x, trans.y, trans.z])
            
            # Rotation
            rot = transform.transform.rotation
            roll, pitch, yaw = self.quaternion_to_euler(rot.x, rot.y, rot.z, rot.w)
            self.base_orientation = np.array([roll, pitch, yaw])
            
            if not self.tf_received:
                self.tf_received = True
                self.get_logger().info('='*60)
                self.get_logger().info('✅ TF에서 Base Pose 수신 시작')
                self.get_logger().info(f'  현재 위치: [{self.base_position[0]:.4f}, {self.base_position[1]:.4f}, {self.base_position[2]:.4f}] m')
                self.get_logger().info(f'  현재 회전: [{math.degrees(roll):.2f}°, {math.degrees(pitch):.2f}°, {math.degrees(yaw):.2f}°]')
                self.get_logger().info('='*60)
            
            # 초기 base pose 저장 및 목표 pose 설정 (처음 한 번만)
            if not self.target_pose_initialized:
                self.initial_base_position = self.base_position.copy()
                self.initial_base_orientation = self.base_orientation.copy()
                
                if self.use_absolute_target:
                    # 절대 좌표 방식: desired target을 사용하고 offset을 자동 계산
                    self.target_base_position = self.desired_base_position.copy()
                    self.target_base_orientation = self.desired_base_orientation.copy()
                    
                    # Offset 자동 계산 (target - initial)
                    self.base_position_offset = self.target_base_position - self.initial_base_position
                    self.base_orientation_offset = self.target_base_orientation - self.initial_base_orientation
                    
                    self.get_logger().info('='*60)
                    self.get_logger().info('🎯 절대 좌표 방식: 목표 Base Pose 설정 완료')
                    self.get_logger().info(f'  초기 위치: [{self.initial_base_position[0]:.4f}, {self.initial_base_position[1]:.4f}, {self.initial_base_position[2]:.4f}] m')
                    self.get_logger().info(f'  초기 회전: [{math.degrees(self.initial_base_orientation[0]):.2f}°, {math.degrees(self.initial_base_orientation[1]):.2f}°, {math.degrees(self.initial_base_orientation[2]):.2f}°]')
                    self.get_logger().info(f'  목표 위치 (절대): [{self.target_base_position[0]:.4f}, {self.target_base_position[1]:.4f}, {self.target_base_position[2]:.4f}] m')
                    self.get_logger().info(f'  목표 회전 (절대): [{math.degrees(self.target_base_orientation[0]):.2f}°, {math.degrees(self.target_base_orientation[1]):.2f}°, {math.degrees(self.target_base_orientation[2]):.2f}°]')
                    self.get_logger().info(f'  계산된 Position offset: [{self.base_position_offset[0]:.4f}, {self.base_position_offset[1]:.4f}, {self.base_position_offset[2]:.4f}] m')
                    self.get_logger().info(f'  계산된 Orientation offset: [{math.degrees(self.base_orientation_offset[0]):.2f}°, {math.degrees(self.base_orientation_offset[1]):.2f}°, {math.degrees(self.base_orientation_offset[2]):.2f}°]')
                    self.get_logger().info('='*60)
                else:
                    # Offset 방식: 초기 pose + offset
                    self.target_base_position = self.initial_base_position + self.base_position_offset
                    self.target_base_orientation = self.initial_base_orientation + self.base_orientation_offset
                    
                    self.get_logger().info('='*60)
                    self.get_logger().info('🎯 Offset 방식: 목표 Base Pose 설정 완료')
                    self.get_logger().info(f'  초기 위치: [{self.initial_base_position[0]:.4f}, {self.initial_base_position[1]:.4f}, {self.initial_base_position[2]:.4f}] m')
                    self.get_logger().info(f'  초기 회전: [{math.degrees(self.initial_base_orientation[0]):.2f}°, {math.degrees(self.initial_base_orientation[1]):.2f}°, {math.degrees(self.initial_base_orientation[2]):.2f}°]')
                    self.get_logger().info(f'  Position offset: [{self.base_position_offset[0]:.4f}, {self.base_position_offset[1]:.4f}, {self.base_position_offset[2]:.4f}] m')
                    self.get_logger().info(f'  Orientation offset: [{math.degrees(self.base_orientation_offset[0]):.2f}°, {math.degrees(self.base_orientation_offset[1]):.2f}°, {math.degrees(self.base_orientation_offset[2]):.2f}°]')
                    self.get_logger().info(f'  목표 위치: [{self.target_base_position[0]:.4f}, {self.target_base_position[1]:.4f}, {self.target_base_position[2]:.4f}] m')
                    self.get_logger().info(f'  목표 회전: [{math.degrees(self.target_base_orientation[0]):.2f}°, {math.degrees(self.target_base_orientation[1]):.2f}°, {math.degrees(self.target_base_orientation[2]):.2f}°]')
                    self.get_logger().info('='*60)
                
                self.target_pose_initialized = True
            
            return True
        except Exception as e:
            if not hasattr(self, '_tf_warning_logged'):
                self.get_logger().debug(f'TF 변환 대기 중: {e}')
                self._tf_warning_logged = True
            return False
    
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
        # 목표 base pose 사용 (초기화되지 않은 경우 현재 pose + offset 사용)
        if self.target_pose_initialized and self.target_base_position is not None:
            base_pos = self.target_base_position
            base_orient = self.target_base_orientation
        else:
            # 아직 초기화되지 않은 경우 현재 값 + offset 사용
            base_pos = self.base_position + self.base_position_offset
            base_orient = self.base_orientation + self.base_orientation_offset
        
        roll, pitch, yaw = base_orient
        R = self.rotation_matrix_from_euler(roll, pitch, yaw)
        
        hip_positions_world = {}
        for leg in ['FR', 'FL', 'RR', 'RL']:
            if self.hip_positions_initialized and leg in self.hip_positions_body_from_tf:
                hip_pos_body = self.hip_positions_body_from_tf[leg]
            else:
                hip_pos_body = self.hip_positions_body[leg]
            
            hip_pos_world = base_pos + R @ hip_pos_body
            hip_positions_world[leg] = hip_pos_world
        
        return hip_positions_world
    
    def forward_kinematics_leg(self, hip_angle: float, thigh_angle: float, calf_angle: float, leg: str) -> np.ndarray:
        """Forward Kinematics: 조인트 각도에서 발 끝 위치 계산"""
        cos_h = math.cos(hip_angle)
        sin_h = math.sin(hip_angle)
        
        hip_offset_local = np.array([0.0, self.L1, 0.0])
        
        thigh_local = np.array([
            self.L2 * math.cos(thigh_angle),
            0.0,
            -self.L2 * math.sin(thigh_angle)
        ])
        
        total_angle = thigh_angle - calf_angle
        calf_local = np.array([
            self.L3 * math.cos(total_angle),
            0.0,
            -self.L3 * math.sin(total_angle)
        ])
        
        R_y = np.array([
            [cos_h, 0, sin_h],
            [0, 1, 0],
            [-sin_h, 0, cos_h]
        ])
        
        hip_offset_rotated = R_y @ hip_offset_local
        thigh_rotated = R_y @ thigh_local
        calf_rotated = R_y @ calf_local
        
        foot_pos = hip_offset_rotated + thigh_rotated + calf_rotated
        
        return foot_pos
    
    def analytical_ik_leg(self, target_pos: np.ndarray, leg: str) -> Optional[Tuple[float, float, float]]:
        """Analytical inverse kinematics for GO1 leg"""
        x, y, z = target_pos[0], target_pos[1], target_pos[2]
        
        try:
            # Step 1: Calculate D
            D_squared = z*z + y*y - self.L1*self.L1
            if D_squared < 0:
                return None
            
            D = np.sqrt(D_squared)
            
            # Step 2: Calculate q1 (hip joint)
            is_left_leg = leg in ['FL', 'RL']
            
            if abs(y) < 1e-6:
                q1 = 0.0
            elif y > 0:
                if is_left_leg:
                    q1 = np.arctan2(y, abs(z)) + np.arctan2(D, self.L1) - math.pi/2
                else:
                    q1 = np.arctan2(y, abs(z)) + np.arctan2(D, self.L1) - math.pi/2
            else:
                if is_left_leg:
                    q1 = math.pi/2 - (np.arctan2(D, self.L1) + np.arctan2(abs(y), abs(z)))
                else:
                    q1 = math.pi/2 - (np.arctan2(D, self.L1) + np.arctan2(abs(y), abs(z)))
            
            # Step 3: Calculate G
            G = np.sqrt(D*D + x*x)
            
            # Check reachability
            max_reach = self.L2 + self.L3
            min_reach = abs(self.L2 - self.L3)
            if G > max_reach or G < min_reach:
                return None
            
            # Step 4: Calculate gamma
            cos_gamma = (G*G - self.L2*self.L2 - self.L3*self.L3) / (-2 * self.L2 * self.L3)
            if abs(cos_gamma) > 1.0:
                return None
            
            gamma = np.arccos(cos_gamma)
            
            # Step 5: Calculate q3 (calf joint)
            q3 = gamma - math.pi
            
            # Step 6: Calculate beta
            sin_gamma = np.sin(gamma)
            if abs(sin_gamma) < 1e-6:
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
                'hip': [-0.8, 0.8],
                'thigh': [-1.0, 4.0],
                'calf': [-2.7, -0.9]
            }
            
            limits = [joint_limits['hip'], joint_limits['thigh'], joint_limits['calf']]
            for i, (q, (q_min, q_max)) in enumerate(zip(solution, limits)):
                if not (q_min <= q <= q_max):
                    return None
            
            return tuple(solution)
            
        except Exception as e:
            self.get_logger().debug(f'Analytical IK failed for {leg}: {e}')
            return None
    
    def joint_state_callback(self, msg: JointState):
        """Joint state 수신 및 업데이트"""
        for i, joint_name in enumerate(msg.name):
            if i < len(msg.position):
                self.current_joint_positions[joint_name] = float(msg.position[i])
        
        if len(self.current_joint_positions) >= len(self.go1_joints):
            self.joint_state_received = True
    
    def timer_callback(self):
        """타이머 콜백: 주기적으로 joint command 계산 및 발행"""
        self.update_base_pose_from_tf()
        
        if self.joint_state_received:
            self.calculate_and_send_joint_command()
    
    def calculate_and_send_joint_command(self):
        """Joint command 계산 및 발행"""
        try:
            # 주기적으로 현재 base pose와 목표 base pose 비교
            current_time = self.get_clock().now().nanoseconds / 1e9
            if self.target_pose_initialized and current_time - self.last_pose_debug_time >= self.pose_debug_interval:
                pos_error = np.linalg.norm(self.base_position - self.target_base_position)
                orient_error = np.linalg.norm(self.base_orientation - self.target_base_orientation)
                
                self.get_logger().info('='*80)
                self.get_logger().info('📍 Base Pose 상태:')
                self.get_logger().info(f'  현재 위치: [{self.base_position[0]:.4f}, {self.base_position[1]:.4f}, {self.base_position[2]:.4f}] m')
                self.get_logger().info(f'  목표 위치: [{self.target_base_position[0]:.4f}, {self.target_base_position[1]:.4f}, {self.target_base_position[2]:.4f}] m')
                self.get_logger().info(f'  위치 오차: {pos_error*1000:.2f} mm')
                self.get_logger().info(f'  현재 회전: [{math.degrees(self.base_orientation[0]):.2f}°, {math.degrees(self.base_orientation[1]):.2f}°, {math.degrees(self.base_orientation[2]):.2f}°]')
                self.get_logger().info(f'  목표 회전: [{math.degrees(self.target_base_orientation[0]):.2f}°, {math.degrees(self.target_base_orientation[1]):.2f}°, {math.degrees(self.target_base_orientation[2]):.2f}°]')
                self.get_logger().info(f'  회전 오차: {math.degrees(orient_error):.2f}°')
                self.get_logger().info('='*80)
                self.last_pose_debug_time = current_time
            
            hip_positions_world = self.calculate_hip_positions_world()
            
            # GO1 조인트 값 계산
            go1_joint_positions = []
            leg_names = ['FR', 'FL', 'RR', 'RL']
            
            for leg in leg_names:
                hip_joint = f'{leg}_hip_joint'
                thigh_joint = f'{leg}_thigh_joint'
                calf_joint = f'{leg}_calf_joint'
                
                if not all(j in self.current_joint_positions for j in [hip_joint, thigh_joint, calf_joint]):
                    go1_joint_positions.extend([0.0, 0.67, -1.3])
                    continue
                
                # 현재 joint 값
                current_hip = self.current_joint_positions[hip_joint]
                current_thigh = self.current_joint_positions[thigh_joint]
                current_calf = self.current_joint_positions[calf_joint]
                
                # TF에서 hip과 foot 위치 가져오기
                hip_frame_name = self.hip_frames.get(leg, f'{leg}_hip')
                foot_frame_names = self.foot_frames.get(leg, [f'{leg}_foot'])
                
                tf_hip_pos_world = self.get_tf_position(hip_frame_name, self.world_frame)
                tf_foot_pos_world, actual_foot_frame = self.get_tf_position_try_multiple(foot_frame_names, self.world_frame)
                
                # TF에서 base → hip 변환으로 hip_positions_body 업데이트
                base_to_hip_tf = self.get_tf_position(hip_frame_name, self.base_frame)
                if base_to_hip_tf is not None and not self.hip_positions_initialized:
                    self.hip_positions_body_from_tf[leg] = base_to_hip_tf.copy()
                    if len(self.hip_positions_body_from_tf) == 4:
                        self.hip_positions_initialized = True
                        self.get_logger().info('='*80)
                        self.get_logger().info('✅ TF에서 실제 hip 위치 수신 완료')
                        for l, pos in self.hip_positions_body_from_tf.items():
                            self.get_logger().info(f'  {l}: [{pos[0]:.6f}, {pos[1]:.6f}, {pos[2]:.6f}] m')
                        self.get_logger().info('='*80)
                
                # 목표 base pose 기반 hip position 계산
                if self.target_pose_initialized and self.target_base_position is not None:
                    target_base_pos = self.target_base_position
                    target_base_orient = self.target_base_orientation
                else:
                    # 아직 초기화되지 않은 경우 현재 값 + offset 사용
                    target_base_pos = self.base_position + self.base_position_offset
                    target_base_orient = self.base_orientation + self.base_orientation_offset
                
                if self.hip_positions_initialized and leg in self.hip_positions_body_from_tf:
                    hip_pos_body_actual = self.hip_positions_body_from_tf[leg]
                    roll, pitch, yaw = target_base_orient
                    R = self.rotation_matrix_from_euler(roll, pitch, yaw)
                    calculated_hip_pos_world = target_base_pos + R @ hip_pos_body_actual
                else:
                    calculated_hip_pos_world = hip_positions_world[leg]
                
                # 디버그 출력 (주석 처리됨)
                # if tf_hip_pos_world is not None:
                #     hip_diff = np.linalg.norm(calculated_hip_pos_world - tf_hip_pos_world)
                #     current_time = self.get_clock().now().nanoseconds / 1e9
                #     if current_time - self.last_debug_time >= self.debug_interval:
                #         self.get_logger().info(f'{leg} hip: 목표[{calculated_hip_pos_world[0]:.4f}, {calculated_hip_pos_world[1]:.4f}, {calculated_hip_pos_world[2]:.4f}] '
                #                               f'현재TF[{tf_hip_pos_world[0]:.4f}, {tf_hip_pos_world[1]:.4f}, {tf_hip_pos_world[2]:.4f}] '
                #                               f'차이[{hip_diff*1000:.2f}mm]')
                #         if leg == 'RL':
                #             self.last_debug_time = current_time
                
                fk_hip_pos_world = calculated_hip_pos_world
                
                # 회전 행렬
                roll, pitch, yaw = target_base_orient
                R = self.rotation_matrix_from_euler(roll, pitch, yaw)
                R_inv = R.T
                
                # 목표 foot position 초기화 (처음 한 번만)
                if tf_foot_pos_world is not None and not self.target_foot_positions_initialized:
                    self.target_foot_positions_world[leg] = tf_foot_pos_world.copy()
                    if len(self.target_foot_positions_world) == 4:
                        self.target_foot_positions_initialized = True
                        self.get_logger().info('='*80)
                        self.get_logger().info('🎯 목표 Foot Position 설정 완료')
                        for l, pos in self.target_foot_positions_world.items():
                            self.get_logger().info(f'  {l}: [{pos[0]:.4f}, {pos[1]:.4f}, {pos[2]:.4f}] m')
                        self.get_logger().info('='*80)
                
                # IK를 위한 target 위치 계산
                if self.target_foot_positions_initialized and leg in self.target_foot_positions_world:
                    target_foot_pos_world = self.target_foot_positions_world[leg]
                    foot_relative_world = target_foot_pos_world - fk_hip_pos_world
                    foot_pos_hip = R_inv @ foot_relative_world
                elif tf_foot_pos_world is not None:
                    foot_relative_world = tf_foot_pos_world - fk_hip_pos_world
                    foot_pos_hip = R_inv @ foot_relative_world
                else:
                    go1_joint_positions.extend([current_hip, current_thigh, current_calf])
                    if tf_foot_pos_world is None:
                        self.get_logger().warn(f'⚠️ {leg} 다리: TF에서 foot 위치를 받지 못해 현재 joint 값 유지')
                    continue
                
                # IK 계산
                ik_result = self.analytical_ik_leg(foot_pos_hip, leg)
                
                if ik_result is None or np.allclose(ik_result, [0.0, 0.0, 0.0]):
                    go1_joint_positions.extend([current_hip, current_thigh, current_calf])
                    if not hasattr(self, '_ik_warnings_logged'):
                        self._ik_warnings_logged = {}
                    if leg not in self._ik_warnings_logged:
                        self.get_logger().warn(f'❌ {leg}: Analytical IK 실패, 현재 joint 값 유지')
                        self.get_logger().warn(f'   Target foot (hip frame): [{foot_pos_hip[0]:.3f}, {foot_pos_hip[1]:.3f}, {foot_pos_hip[2]:.3f}]')
                        self._ik_warnings_logged[leg] = True
                else:
                    ik_hip, ik_thigh, ik_calf = ik_result
                    
                    # IK 성공 시 디버깅 (처음 한 번만)
                    if not hasattr(self, '_ik_success_logged'):
                        self._ik_success_logged = {}
                    if leg not in self._ik_success_logged:
                        self.get_logger().info(f'✅ {leg}: Analytical IK 성공')
                        self.get_logger().info(f'   Target foot (hip frame): [{foot_pos_hip[0]:.3f}, {foot_pos_hip[1]:.3f}, {foot_pos_hip[2]:.3f}]')
                        self.get_logger().info(f'   IK result: hip={math.degrees(ik_hip):.1f}°, thigh={math.degrees(ik_thigh):.1f}°, calf={math.degrees(ik_calf):.1f}°')
                        self._ik_success_logged[leg] = True
                    
                    # Joint 값 변화량 제한
                    def limit_joint_change(current, target, max_change):
                        diff = target - current
                        if abs(diff) > max_change:
                            return current + np.sign(diff) * max_change
                        return target
                    
                    leg_key = f'{leg}_joints'
                    if leg_key not in self.previous_joint_positions:
                        self.previous_joint_positions[leg_key] = {
                            'hip': current_hip,
                            'thigh': current_thigh,
                            'calf': current_calf
                        }
                    
                    prev_hip = self.previous_joint_positions[leg_key]['hip']
                    prev_thigh = self.previous_joint_positions[leg_key]['thigh']
                    prev_calf = self.previous_joint_positions[leg_key]['calf']
                    
                    safe_ik_hip = limit_joint_change(prev_hip, ik_hip, self.max_joint_change_per_step)
                    safe_ik_thigh = limit_joint_change(prev_thigh, ik_thigh, self.max_joint_change_per_step)
                    safe_ik_calf = limit_joint_change(prev_calf, ik_calf, self.max_joint_change_per_step)
                    
                    self.previous_joint_positions[leg_key] = {
                        'hip': safe_ik_hip,
                        'thigh': safe_ik_thigh,
                        'calf': safe_ik_calf
                    }
                    
                    go1_joint_positions.extend([safe_ik_hip, safe_ik_thigh, safe_ik_calf])
            
            # Joint command 발행
            if len(go1_joint_positions) == len(self.go1_joints):
                current_time = self.get_clock().now().nanoseconds / 1e9
                
                # K1 제어 시작 시간 설정 (처음 한 번만)
                if self.k1_control_start_time is None:
                    self.k1_control_start_time = current_time
                    self.get_logger().info('='*60)
                    self.get_logger().info('⏱️  K1 매니퓰레이터 대기 시작')
                    self.get_logger().info(f'   {self.k1_start_delay}초 후 목표 자세로 이동 시작')
                    self.get_logger().info('='*60)
                
                # K1 제어 활성화 확인
                elapsed_time = current_time - self.k1_control_start_time
                if not self.k1_enabled and elapsed_time >= self.k1_start_delay:
                    self.k1_enabled = True
                    self.get_logger().info('='*60)
                    self.get_logger().info('✅ K1 매니퓰레이터 동작 시작!')
                    self.get_logger().info('   목표 자세로 천천히 이동합니다...')
                    self.get_logger().info('='*60)
                
                # K1 조인트 값 점진적 업데이트
                k1_joint_positions = []
                k1_reached_target = True  # 모든 조인트가 목표에 도달했는지 확인
                
                for joint_name in self.k1_joints:
                    target_value = self.k1_target_positions[joint_name]
                    current_value = self.k1_current_positions[joint_name]
                    
                    if self.k1_enabled:
                        # 목표 값으로 천천히 이동
                        diff = target_value - current_value
                        if abs(diff) > self.max_k1_joint_change_per_step:
                            new_value = current_value + np.sign(diff) * self.max_k1_joint_change_per_step
                            k1_reached_target = False
                        else:
                            new_value = target_value
                        
                        self.k1_current_positions[joint_name] = new_value
                        k1_joint_positions.append(new_value)
                    else:
                        # 대기 중에는 home position (0) 유지
                        k1_joint_positions.append(current_value)
                        k1_reached_target = False
                
                # K1이 목표에 도달했을 때 알림 (한 번만)
                if k1_reached_target and self.k1_enabled and not hasattr(self, '_k1_target_reached'):
                    self.get_logger().info('='*60)
                    self.get_logger().info('🎯 K1 매니퓰레이터가 목표 자세에 도달했습니다!')
                    self.get_logger().info('='*60)
                    self._k1_target_reached = True
                
                all_joint_positions = go1_joint_positions + k1_joint_positions
                
                # K1 joint 값 디버깅 (주기적으로 출력)
                if not hasattr(self, '_k1_debug_time'):
                    self._k1_debug_time = 0.0
                    self._k1_values_logged = False
                
                if current_time - self._k1_debug_time >= 5.0 and not self._k1_values_logged:  # 5초마다
                    self.get_logger().info('='*60)
                    self.get_logger().info('🤖 K1 매니퓰레이터 조인트 현재 값:')
                    for i, joint_name in enumerate(self.k1_joints):
                        value = k1_joint_positions[i]
                        target = self.k1_target_positions[joint_name]
                        error = abs(target - value)
                        self.get_logger().info(f'  {joint_name}: {math.degrees(value):.2f}° (목표: {math.degrees(target):.2f}°, 오차: {math.degrees(error):.2f}°)')
                    self.get_logger().info('='*60)
                    self._k1_debug_time = current_time
                    if current_time > 15.0:  # 15초 후에는 더 이상 출력하지 않음
                        self._k1_values_logged = True
                
                msg = JointState()
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.header.frame_id = 'base_link'
                msg.name = self.all_joints
                msg.position = all_joint_positions
                msg.velocity = []
                msg.effort = []
                
                self.joint_command_publisher.publish(msg)
            
            # 첫 실행 시 정보 출력
            if not hasattr(self, '_control_mode_logged') and self.target_pose_initialized:
                self.get_logger().info('='*80)
                self.get_logger().info('✅ 제어 모드: 목표 Base Pose 추종 중')
                self.get_logger().info('='*80)
                self.get_logger().info(f'  목표 Base position: [{self.target_base_position[0]:.4f}, {self.target_base_position[1]:.4f}, {self.target_base_position[2]:.4f}] m')
                self.get_logger().info(f'  목표 Base orientation: [{math.degrees(self.target_base_orientation[0]):.2f}°, {math.degrees(self.target_base_orientation[1]):.2f}°, {math.degrees(self.target_base_orientation[2]):.2f}°]')
                self.get_logger().info('='*80)
                self._control_mode_logged = True
                
        except Exception as e:
            self.get_logger().error(f'Error calculating/sending joint command: {e}')


def main(args=None):
    rclpy.init(args=args)
    
    node = IRMController()
    
    try:
        node.get_logger().info('\n🔄 IRM Controller 실행 중...\n')
        node.get_logger().info('   Joint state를 기다리는 중...\n')
        
        # Joint state가 올 때까지 대기 (최대 10초)
        timeout = 10.0
        start_time = node.get_clock().now()
        
        while not node.joint_state_received:
            rclpy.spin_once(node, timeout_sec=0.1)
            
            elapsed = (node.get_clock().now() - start_time).nanoseconds / 1e9
            if elapsed > timeout:
                node.get_logger().error(f'❌ 타임아웃: {timeout}초 내에 joint state를 받지 못했습니다.')
                break
        
        if node.joint_state_received:
            node.get_logger().info('\n✅ Joint state 수신됨, 단일 자세 유지 중...\n')
            node.get_logger().info('   종료하려면 Ctrl+C를 누르세요.\n')
            
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

