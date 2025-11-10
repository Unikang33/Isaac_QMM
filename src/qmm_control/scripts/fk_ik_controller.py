#!/usr/bin/env python3
"""
FK-IK 기반 Joint Command 컨트롤러
~/secret_usd/QMM_final.usd 사용

1. 현재 joint state 수신
2. TF에서 base pose와 foot position 수신
3. Analytical IK로 joint 값 계산
4. 계산된 joint 값을 joint_command 토픽으로 발행
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from tf2_ros import TransformListener, Buffer
from geometry_msgs.msg import TransformStamped
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
        
        # 디버깅 출력 제어
        self.last_debug_time = 0.0
        self.debug_interval = 1.0  # 1초마다 출력
        
        # TF Buffer와 Listener 생성
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # TF frame 이름
        self.world_frame = 'world'
        self.base_frame = 'base'  # 실제 base frame 이름 (TF tree 확인 결과)
        
        # 다리별 hip과 foot frame 이름 (TF에서 사용)
        self.hip_frames = {
            'FR': 'FR_hip',
            'FL': 'FL_hip',
            'RR': 'RR_hip',
            'RL': 'RL_hip'
        }
        # Foot frame 이름 (여러 가능한 이름 시도, calf는 제외)
        self.foot_frames = {
            'FR': ['FR_foot', 'FR_foot_link', 'go1_FR_foot'],
            'FL': ['FL_foot', 'FL_foot_link', 'go1_FL_foot'],
            'RR': ['RR_foot', 'RR_foot_link', 'go1_RR_foot'],
            'RL': ['RL_foot', 'RL_foot_link', 'go1_RL_foot']
        }
        
        # TF에서 받은 실제 hip 위치를 저장 (초기화 후 업데이트)
        self.hip_positions_body_from_tf = {}
        self.hip_positions_initialized = False
        
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
        
        # Base position과 orientation (TF에서 받아옴, 초기값 설정)
        self.base_position = np.array([0.0, 0.0, 0.33])  # 초기값 (TF 수신 전까지)
        self.base_orientation = np.array([0.0, 0.0, 0.0])  # [roll, pitch, yaw]
        self.tf_received = False  # TF 수신 플래그
        
        # Base position과 orientation offset (시간에 따라 동적으로 변경)
        self.base_position_offset = np.array([0.0, 0.0, 0.0])  # [x, y, z] in meters
        self.base_orientation_offset = np.array([0.0, 0.0, 0.0])  # [roll, pitch, yaw] in radians
        
        # 시간 기반 offset 시퀀스 제어
        self.start_time = None  # 제어 시작 시간
        self.stage_duration = 3.0  # 각 단계 지속 시간 (초)
        self.current_stage = 0  # 현재 단계
        self.offset_stages = [
            # 1. offset 없음 (0-3초)
            {'pos': [0.0, 0.0, 0.0], 'orient': [0.0, 0.0, 0.0], 'name': 'offset 없음'},
            # 2. z offset -0.1 (3-6초)
            {'pos': [0.0, 0.0, -0.1], 'orient': [0.0, 0.0, 0.0], 'name': 'z offset -0.1'},
            # 3. offset 없음 (6-9초)
            {'pos': [0.0, 0.0, 0.0], 'orient': [0.0, 0.0, 0.0], 'name': 'offset 없음'},
            # 4. roll offset 0.2 (9-12초)
            {'pos': [0.0, 0.0, 0.0], 'orient': [0.2, 0.0, 0.0], 'name': 'roll offset 0.2'},
            # 5. offset 없음 (12-15초)
            {'pos': [0.0, 0.0, 0.0], 'orient': [0.0, 0.0, 0.0], 'name': 'offset 없음'},
            # 6. pitch offset 0.4 (15-18초)
            {'pos': [0.0, 0.0, 0.0], 'orient': [0.0, 0.4, 0.0], 'name': 'pitch offset 0.4'},
            # 7. offset 없음 (18-21초)
            {'pos': [0.0, 0.0, 0.0], 'orient': [0.0, 0.0, 0.0], 'name': 'offset 없음'},
            # 8. z offset -0.1, roll offset 0.2, pitch offset 0.4 (21-24초)
            {'pos': [0.0, 0.0, -0.1], 'orient': [0.2, 0.4, 0.0], 'name': 'z offset -0.1, roll offset 0.2, pitch offset 0.4'},
        ]
        
        # 목표 base pose (offset 변경 시 재계산됨)
        self.target_base_position = None  # 초기화 시 설정됨
        self.target_base_orientation = None  # 초기화 시 설정됨
        self.target_pose_initialized = False  # 목표 pose 초기화 플래그
        self.initial_base_position = None  # 초기 base position 저장
        self.initial_base_orientation = None  # 초기 base orientation 저장
        
        # 목표 foot position (초기 실행 시 한 번만 설정, 각 다리별)
        self.target_foot_positions_world = {}  # {leg: np.array([x, y, z])}
        self.target_foot_positions_initialized = False  # 목표 foot position 초기화 플래그
        
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
        
        # Joint 값 변화량 제한 (한 스텝에서 최대 변화량)
        self.max_joint_change_per_step = math.radians(2.0)  # 2도/스텝 (10Hz 기준)
        
        # 주기적 command 발행을 위한 타이머 (10Hz로 변경하여 부드러운 제어)
        self.timer = self.create_timer(0.1, self.timer_callback)  # 10Hz
        
        self.get_logger().info('='*60)
        self.get_logger().info('✅ FK-IK Controller 시작됨')
        self.get_logger().info('='*60)
        self.get_logger().info('  - Joint state → TF → Analytical IK → Joint Command')
        self.get_logger().info('  - 지속적으로 자세 유지 (10Hz)')
        self.get_logger().info(f'  - Base position: TF에서 받아옴 ({self.world_frame} → {self.base_frame})')
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
            pitch = math.copysign(math.pi / 2, sinp)  # use 90 degrees if out of range
        else:
            pitch = math.asin(sinp)
        
        # Yaw (z-axis rotation)
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        
        return roll, pitch, yaw
    
    def get_tf_position(self, target_frame: str, source_frame: str = 'world', timeout: float = 0.1) -> Optional[np.ndarray]:
        """TF에서 특정 frame의 world 위치를 가져옴 (재시도 포함)"""
        try:
            # 최신 시간 사용
            transform: TransformStamped = self.tf_buffer.lookup_transform(
                source_frame,
                target_frame,
                rclpy.time.Time()
            )
            trans = transform.transform.translation
            return np.array([trans.x, trans.y, trans.z])
        except Exception as e:
            # 재시도: 과거 시간 사용
            try:
                transform: TransformStamped = self.tf_buffer.lookup_transform(
                    source_frame,
                    target_frame,
                    rclpy.time.Time(seconds=0)  # 과거 시간 사용
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
            # TF 변환 가져오기
            transform: TransformStamped = self.tf_buffer.lookup_transform(
                self.world_frame,
                self.base_frame,
                rclpy.time.Time()
            )
            
            # Translation (위치)
            trans = transform.transform.translation
            self.base_position = np.array([trans.x, trans.y, trans.z])
            
            # Rotation (회전) - 쿼터니언을 오일러 각도로 변환
            rot = transform.transform.rotation
            roll, pitch, yaw = self.quaternion_to_euler(rot.x, rot.y, rot.z, rot.w)
            self.base_orientation = np.array([roll, pitch, yaw])
            
            if not self.tf_received:
                self.tf_received = True
                self.get_logger().info('='*60)
                self.get_logger().info('✅ TF에서 Base Pose 수신 시작')
                self.get_logger().info(f'  위치: [{self.base_position[0]:.4f}, {self.base_position[1]:.4f}, {self.base_position[2]:.4f}] m')
                self.get_logger().info(f'  회전: [{math.degrees(roll):.2f}°, {math.degrees(pitch):.2f}°, {math.degrees(yaw):.2f}°]')
                self.get_logger().info('='*60)
            
            # 초기 base pose 저장 (처음 한 번만)
            if not self.target_pose_initialized:
                self.initial_base_position = self.base_position.copy()
                self.initial_base_orientation = self.base_orientation.copy()
                self.start_time = self.get_clock().now()
                self.target_pose_initialized = True
                self.get_logger().info('='*60)
                self.get_logger().info('🎯 초기 Base Pose 저장 완료')
                self.get_logger().info(f'  초기 위치: [{self.initial_base_position[0]:.4f}, {self.initial_base_position[1]:.4f}, {self.initial_base_position[2]:.4f}] m')
                self.get_logger().info(f'  초기 회전: [{math.degrees(self.initial_base_orientation[0]):.2f}°, {math.degrees(self.initial_base_orientation[1]):.2f}°, {math.degrees(self.initial_base_orientation[2]):.2f}°]')
                self.get_logger().info('='*60)
            
            # 시간 기반 offset 업데이트 및 목표 pose 재계산
            if self.target_pose_initialized and self.start_time is not None:
                elapsed_time = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
                stage_index = int(elapsed_time / self.stage_duration)
                
                # 현재 단계가 변경되었는지 확인
                if stage_index != self.current_stage and stage_index < len(self.offset_stages):
                    self.current_stage = stage_index
                    stage = self.offset_stages[stage_index]
                    
                    # Offset 업데이트
                    self.base_position_offset = np.array(stage['pos'])
                    self.base_orientation_offset = np.array(stage['orient'])
                    
                    # 목표 pose 재계산 (초기 pose + 현재 offset)
                    self.target_base_position = self.initial_base_position.copy() + self.base_position_offset
                    self.target_base_orientation = self.initial_base_orientation.copy() + self.base_orientation_offset
                    
                    # 로그 출력
                    self.get_logger().info('='*60)
                    self.get_logger().info(f'🔄 단계 {stage_index + 1}/{len(self.offset_stages)}: {stage["name"]}')
                    self.get_logger().info(f'  경과 시간: {elapsed_time:.2f}초')
                    self.get_logger().info(f'  Position offset: [{self.base_position_offset[0]:.4f}, {self.base_position_offset[1]:.4f}, {self.base_position_offset[2]:.4f}] m')
                    self.get_logger().info(f'  Orientation offset: [{math.degrees(self.base_orientation_offset[0]):.2f}°, {math.degrees(self.base_orientation_offset[1]):.2f}°, {math.degrees(self.base_orientation_offset[2]):.2f}°]')
                    self.get_logger().info(f'  목표 위치: [{self.target_base_position[0]:.4f}, {self.target_base_position[1]:.4f}, {self.target_base_position[2]:.4f}] m')
                    self.get_logger().info(f'  목표 회전: [{math.degrees(self.target_base_orientation[0]):.2f}°, {math.degrees(self.target_base_orientation[1]):.2f}°, {math.degrees(self.target_base_orientation[2]):.2f}°]')
                    self.get_logger().info('='*60)
                elif stage_index >= len(self.offset_stages):
                    # 모든 단계 완료 후 마지막 단계 유지
                    if self.current_stage < len(self.offset_stages) - 1:
                        self.current_stage = len(self.offset_stages) - 1
                        stage = self.offset_stages[-1]
                        self.base_position_offset = np.array(stage['pos'])
                        self.base_orientation_offset = np.array(stage['orient'])
                        self.target_base_position = self.initial_base_position.copy() + self.base_position_offset
                        self.target_base_orientation = self.initial_base_orientation.copy() + self.base_orientation_offset
                        self.get_logger().info('='*60)
                        self.get_logger().info('✅ 모든 offset 시퀀스 완료, 마지막 단계 유지')
                        self.get_logger().info('='*60)
                else:
                    # 목표 pose 유지 (현재 offset 기반)
                    if self.initial_base_position is not None:
                        self.target_base_position = self.initial_base_position.copy() + self.base_position_offset
                        self.target_base_orientation = self.initial_base_orientation.copy() + self.base_orientation_offset
            
            return True
        except Exception as e:
            # TF가 아직 발행되지 않았거나 변환 실패
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
        """
        World frame 기준으로 hip positions 계산
        
        목표 base pose를 사용하여 hip positions를 계산합니다.
        TF에서 실제 hip 위치를 받은 경우 그것을 사용하고, 그렇지 않으면 기본값을 사용합니다.
        목표 base pose는 초기 실행 시 한 번만 설정되고 계속 추종합니다.
        """
        # 목표 base pose 사용 (초기 설정된 값)
        if self.target_pose_initialized:
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
            # TF에서 받은 실제 hip 위치를 우선 사용
            if self.hip_positions_initialized and leg in self.hip_positions_body_from_tf:
                hip_pos_body = self.hip_positions_body_from_tf[leg]
            else:
                # 기본값 사용
                hip_pos_body = self.hip_positions_body[leg]
            
            hip_pos_world = base_pos + R @ hip_pos_body
            hip_positions_world[leg] = hip_pos_world
        
        return hip_positions_world
    
    def calculate_hip_positions_world_from_tf(self) -> Dict[str, np.ndarray]:
        """
        World frame 기준으로 hip positions 계산 (TF 기반, base pose 변화 대응)
        
        매번 TF에서 base → hip 변환을 받아서 계산하므로,
        base pose가 바뀌어도 항상 최신 값을 사용합니다.
        """
        hip_positions_world = {}
        
        for leg in ['FR', 'FL', 'RR', 'RL']:
            hip_frame_name = self.hip_frames.get(leg, f'{leg}_hip')
            
            # TF에서 world → hip 변환을 직접 받아서 사용
            tf_hip_pos_world = self.get_tf_position(hip_frame_name, self.world_frame)
            
            if tf_hip_pos_world is not None:
                # TF에서 직접 받은 hip 위치 사용
                hip_positions_world[leg] = tf_hip_pos_world
            else:
                # TF를 받지 못한 경우 기존 방식 사용 (fallback)
                roll, pitch, yaw = self.base_orientation
                R = self.rotation_matrix_from_euler(roll, pitch, yaw)
                
                if self.hip_positions_initialized and leg in self.hip_positions_body_from_tf:
                    hip_pos_body = self.hip_positions_body_from_tf[leg]
                else:
                    hip_pos_body = self.hip_positions_body[leg]
                
                hip_pos_world = self.base_position + R @ hip_pos_body
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
        # TF에서 base pose 업데이트
        self.update_base_pose_from_tf()
        
        if self.joint_state_received:
            self.calculate_and_send_joint_command()
    
    def calculate_and_send_joint_command(self):
        """Joint command 계산 및 조건부 발행 (차이가 1도 이상일 때만)"""
        try:
            # TF에서 받아온 base position과 orientation으로 hip positions 계산
            roll, pitch, yaw = self.base_orientation
            R = self.rotation_matrix_from_euler(roll, pitch, yaw)
            hip_positions_world = self.calculate_hip_positions_world()
            
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
                
                # ========== FK 검증: TF와 FK 계산값 비교 ==========
                # TF에서 hip과 foot 위치 가져오기
                hip_frame_name = self.hip_frames.get(leg, f'{leg}_hip')
                foot_frame_names = self.foot_frames.get(leg, [f'{leg}_foot'])
                
                tf_hip_pos_world = self.get_tf_position(hip_frame_name, self.world_frame)
                tf_foot_pos_world, actual_foot_frame = self.get_tf_position_try_multiple(foot_frame_names, self.world_frame)
                
                if actual_foot_frame:
                    foot_frame_name = actual_foot_frame
                else:
                    foot_frame_name = foot_frame_names[0] if isinstance(foot_frame_names, list) else foot_frame_names
                
                # TF에서 base → hip 변환을 받아서 hip_positions_body 업데이트 (초기화용)
                base_to_hip_tf = self.get_tf_position(hip_frame_name, self.base_frame)
                if base_to_hip_tf is not None and not self.hip_positions_initialized:
                    self.hip_positions_body_from_tf[leg] = base_to_hip_tf.copy()
                    # 모든 다리의 hip 위치를 받으면 초기화 완료
                    if len(self.hip_positions_body_from_tf) == 4:
                        self.hip_positions_initialized = True
                        self.get_logger().info('='*80)
                        self.get_logger().info('✅ TF에서 실제 hip 위치를 받아 hip_positions_body 업데이트')
                        for l, pos in self.hip_positions_body_from_tf.items():
                            self.get_logger().info(f'  {l}: [{pos[0]:.6f}, {pos[1]:.6f}, {pos[2]:.6f}] m')
                        self.get_logger().info('='*80)
                
                # 목표 base pose 기반으로 계산한 hip position (검증 후 사용)
                # 목표 base pose 사용 (초기 설정된 값)
                if self.target_pose_initialized:
                    target_base_pos = self.target_base_position
                    target_base_orient = self.target_base_orientation
                else:
                    # 아직 초기화되지 않은 경우 현재 값 + offset 사용
                    target_base_pos = self.base_position + self.base_position_offset
                    target_base_orient = self.base_orientation + self.base_orientation_offset
                
                if self.hip_positions_initialized and leg in self.hip_positions_body_from_tf:
                    # 초기화된 경우, 목표 base pose와 body frame 위치로 계산
                    hip_pos_body_actual = self.hip_positions_body_from_tf[leg]
                    roll, pitch, yaw = target_base_orient
                    R = self.rotation_matrix_from_euler(roll, pitch, yaw)
                    calculated_hip_pos_world = target_base_pos + R @ hip_pos_body_actual
                else:
                    # 기본값 사용 (이미 목표 base pose가 적용된 hip_positions_world 사용)
                    calculated_hip_pos_world = hip_positions_world[leg]
                
                # 검증: 계산된 hip position과 TF에서 받은 hip position 비교
                if tf_hip_pos_world is not None:
                    hip_diff = np.linalg.norm(calculated_hip_pos_world - tf_hip_pos_world)
                    # 1초마다 출력
                    current_time = self.get_clock().now().nanoseconds / 1e9
                    if current_time - self.last_debug_time >= self.debug_interval:
                        self.get_logger().info(f'{leg} hip: 목표[{calculated_hip_pos_world[0]:.4f}, {calculated_hip_pos_world[1]:.4f}, {calculated_hip_pos_world[2]:.4f}] '
                                              f'현재TF[{tf_hip_pos_world[0]:.4f}, {tf_hip_pos_world[1]:.4f}, {tf_hip_pos_world[2]:.4f}] '
                                              f'차이[{hip_diff*1000:.2f}mm]')
                        if leg == 'RL':  # 마지막 다리 출력 후 시간 업데이트
                            self.last_debug_time = current_time
                
                # 계산된 hip position 사용 (목표 base pose 기반)
                fk_hip_pos_world = calculated_hip_pos_world
                
                # World frame에서 hip 좌표계로 변환을 위한 회전 역행렬
                # 목표 base pose의 orientation 사용
                roll, pitch, yaw = target_base_orient
                R = self.rotation_matrix_from_euler(roll, pitch, yaw)
                R_inv = R.T  # 회전 행렬의 역행렬 = 전치 행렬
                
                # 목표 foot position 초기화 (처음 한 번만)
                if tf_foot_pos_world is not None and not self.target_foot_positions_initialized:
                    self.target_foot_positions_world[leg] = tf_foot_pos_world.copy()
                    # 모든 다리의 foot 위치를 받으면 초기화 완료
                    if len(self.target_foot_positions_world) == 4:
                        self.target_foot_positions_initialized = True
                        self.get_logger().info('='*80)
                        self.get_logger().info('🎯 목표 Foot Position 설정 완료')
                        for l, pos in self.target_foot_positions_world.items():
                            self.get_logger().info(f'  {l}: [{pos[0]:.4f}, {pos[1]:.4f}, {pos[2]:.4f}] m')
                        self.get_logger().info('='*80)
                
                # IK를 위한 target 위치 계산
                # 목표 foot position 사용 (초기 설정된 값)
                if self.target_foot_positions_initialized and leg in self.target_foot_positions_world:
                    # 목표 foot position 사용
                    target_foot_pos_world = self.target_foot_positions_world[leg]
                    # World frame에서 hip 좌표계로 변환
                    foot_relative_world = target_foot_pos_world - fk_hip_pos_world
                    # Hip frame 기준 상대좌표로 변환
                    foot_pos_hip = R_inv @ foot_relative_world
                elif tf_foot_pos_world is not None:
                    # 아직 초기화되지 않은 경우 TF에서 받은 값 사용 (임시)
                    foot_relative_world = tf_foot_pos_world - fk_hip_pos_world
                    foot_pos_hip = R_inv @ foot_relative_world
                else:
                    # TF를 받지 못한 경우 현재 joint 값 유지
                    go1_joint_positions.extend([current_hip, current_thigh, current_calf])
                    if tf_foot_pos_world is None:
                        self.get_logger().warn(f'⚠️ {leg} 다리: TF에서 foot 위치를 받지 못해 현재 joint 값 유지')
                    continue
                
                # 4. IK로 새로운 joint 값 계산 (Analytical IK만 사용)
                ik_result = self.analytical_ik_leg(foot_pos_hip, leg)
                
                if ik_result is None or np.allclose(ik_result, [0.0, 0.0, 0.0]):
                    # IK 실패 시 현재 값 사용
                    go1_joint_positions.extend([current_hip, current_thigh, current_calf])
                    if not hasattr(self, '_ik_warnings_logged'):
                        self._ik_warnings_logged = {}
                    if leg not in self._ik_warnings_logged:
                        self.get_logger().warn(f'{leg}: Analytical IK 실패, 현재 joint 값 유지')
                        self._ik_warnings_logged[leg] = True
                else:
                    ik_hip, ik_thigh, ik_calf = ik_result
                    
                    # Joint 값 변화량 제한 (부드러운 전환)
                    def limit_joint_change(current, target, max_change):
                        diff = target - current
                        if abs(diff) > max_change:
                            return current + np.sign(diff) * max_change
                        return target
                    
                    # Smooth interpolation을 위한 이전 값 저장 (처음 실행 시)
                    if not hasattr(self, 'previous_joint_positions'):
                        self.previous_joint_positions = {}
                    
                    leg_key = f'{leg}_joints'
                    if leg_key not in self.previous_joint_positions:
                        self.previous_joint_positions[leg_key] = {
                            'hip': current_hip,
                            'thigh': current_thigh,
                            'calf': current_calf
                        }
                    
                    # 이전 값 가져오기
                    prev_hip = self.previous_joint_positions[leg_key]['hip']
                    prev_thigh = self.previous_joint_positions[leg_key]['thigh']
                    prev_calf = self.previous_joint_positions[leg_key]['calf']
                    
                    # 변화량 제한 적용 (이전 값 기준)
                    safe_ik_hip = limit_joint_change(prev_hip, ik_hip, self.max_joint_change_per_step)
                    safe_ik_thigh = limit_joint_change(prev_thigh, ik_thigh, self.max_joint_change_per_step)
                    safe_ik_calf = limit_joint_change(prev_calf, ik_calf, self.max_joint_change_per_step)
                    
                    # 이전 값 업데이트
                    self.previous_joint_positions[leg_key] = {
                        'hip': safe_ik_hip,
                        'thigh': safe_ik_thigh,
                        'calf': safe_ik_calf
                    }
                    
                    go1_joint_positions.extend([safe_ik_hip, safe_ik_thigh, safe_ik_calf])
                    
                    # Joint 차이 계산
                    hip_diff = abs(safe_ik_hip - current_hip)
                    thigh_diff = abs(safe_ik_thigh - current_thigh)
                    calf_diff = abs(safe_ik_calf - current_calf)
                    
                    # Joint 차이 확인
                    if (hip_diff > self.joint_diff_threshold or 
                        thigh_diff > self.joint_diff_threshold or 
                        calf_diff > self.joint_diff_threshold):
                        has_significant_change = True
            
            # Offset을 반영한 hip에서 발 끝을 이용해 IK를 구한 값을 joint command로 발행
            if len(go1_joint_positions) == len(self.go1_joints):
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
            
            # 첫 실행 시 정보 출력
            if not hasattr(self, '_control_mode_logged') and self.target_pose_initialized:
                self.get_logger().info('='*80)
                self.get_logger().info('✅ 제어 모드: 목표 Base Pose 추종 중 (IK 계산값을 joint command로 발행)')
                self.get_logger().info('='*80)
                self.get_logger().info(f'  목표 Base position: [{self.target_base_position[0]:.4f}, {self.target_base_position[1]:.4f}, {self.target_base_position[2]:.4f}] m')
                self.get_logger().info(f'  목표 Base orientation: [{math.degrees(self.target_base_orientation[0]):.2f}°, {math.degrees(self.target_base_orientation[1]):.2f}°, {math.degrees(self.target_base_orientation[2]):.2f}°]')
                self.get_logger().info('='*80)
                self._control_mode_logged = True
                
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

