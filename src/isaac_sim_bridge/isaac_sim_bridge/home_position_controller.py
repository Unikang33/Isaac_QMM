#!/usr/bin/env python3
"""
GO1과 K1을 home 자세로 position 제어하는 노드

GO1을 웅크리기 자세에서 home position으로 부드럽게 이동시킵니다.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import math
import sys


class HomePositionController(Node):
    """Home 자세로 제어하는 노드"""
    
    def __init__(self):
        super().__init__('home_position_controller')
        
        # Joint command 발행 (두 토픽 모두 발행)
        self.joint_command_publisher = self.create_publisher(
            JointState,
            '/joint_command',  # fk_ik_controller에서 사용하는 토픽
            10
        )
        self.joint_command_publisher_alt = self.create_publisher(
            JointState,
            '/quadmani/joint_commands',  # README에 명시된 토픽
            10
        )
        
        # GO1 조인트 정의
        self.go1_joints = [
            'FR_hip_joint', 'FR_thigh_joint', 'FR_calf_joint',
            'FL_hip_joint', 'FL_thigh_joint', 'FL_calf_joint',
            'RR_hip_joint', 'RR_thigh_joint', 'RR_calf_joint',
            'RL_hip_joint', 'RL_thigh_joint', 'RL_calf_joint',
        ]
        
        # K1 조인트 정의
        self.k1_joints = [
            'joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6',
            'joint_gripper_left', 'joint_gripper_right'
        ]
        
        self.all_joints = self.go1_joints + self.k1_joints
        
        # GO1 웅크리기 자세 (초기 자세)
        # hip: 0.0, thigh: 0.3 rad (약 17도), calf: -0.8 rad (약 -46도)
        self.go1_crouch_positions = {
            'FR_hip_joint': 0.0,
            'FR_thigh_joint': 0.3,
            'FR_calf_joint': -0.8,
            'FL_hip_joint': 0.0,
            'FL_thigh_joint': 0.3,
            'FL_calf_joint': -0.8,
            'RR_hip_joint': 0.0,
            'RR_thigh_joint': 0.3,
            'RR_calf_joint': -0.8,
            'RL_hip_joint': 0.0,
            'RL_thigh_joint': 0.3,
            'RL_calf_joint': -0.8,
        }
        
        # GO1 home positions (서 있는 자세)
        # hip: 0.0, thigh: 0.67 rad (약 38도), calf: -1.3 rad (약 -74도)
        self.go1_home_positions = {
            'FR_hip_joint': -0.0,
            'FR_thigh_joint': 0.67,
            'FR_calf_joint': -1.3,
            'FL_hip_joint': -0.0,
            'FL_thigh_joint': 0.67,
            'FL_calf_joint': -1.3,
            'RR_hip_joint': 0.0,
            'RR_thigh_joint': 0.67,
            'RR_calf_joint': -1.3,
            'RL_hip_joint': 0.0,
            'RL_thigh_joint': 0.67,
            'RL_calf_joint': -1.3,
        }
        
        # K1 home positions (모든 조인트 0.0)
        self.k1_home_positions = {
            'joint1': 0.0,
            'joint2': 0.0,
            'joint3': 0.0,
            'joint4': 0.0,
            'joint5': 0.0,
            'joint6': 0.0,
            'joint_gripper_left': 0.0,
            'joint_gripper_right': 0.0
        }
        
        # 주기적으로 home position 명령 발행 (10Hz)
        self.timer = self.create_timer(0.1, self.timer_callback)
        
        # 발행 횟수 카운터
        self.publish_count = 0
        
        # 이동 단계 설정
        self.standup_duration = 3.0  # 웅크리기 → 서기 이동 시간 (초)
        self.stabilize_duration = 2.0  # 안정화 시간 (초)
        self.total_duration = self.standup_duration + self.stabilize_duration
        self.max_publish_count = int(self.total_duration * 10)  # 10Hz 기준
        
        self.get_logger().info('='*60)
        self.get_logger().info('✅ Home Position Controller 시작됨')
        self.get_logger().info('='*60)
        self.get_logger().info('  - GO1: 웅크리기 → 서기 자세로 이동')
        self.get_logger().info(f'    웅크리기: thigh=0.3, calf=-0.8')
        self.get_logger().info(f'    서기: thigh=0.67, calf=-1.3')
        self.get_logger().info(f'  - 이동 시간: {self.standup_duration}초')
        self.get_logger().info(f'  - 안정화 시간: {self.stabilize_duration}초')
        self.get_logger().info('  - K1: home position (모든 조인트 0.0)')
        self.get_logger().info('  - 10Hz로 명령 발행')
        self.get_logger().info('='*60)
    
    def smooth_interpolation(self, t):
        """
        Smooth interpolation function (ease-in-out)
        t: 0.0 ~ 1.0
        Returns: 0.0 ~ 1.0
        """
        # Ease-in-out cubic function
        if t < 0.5:
            return 4 * t * t * t
        else:
            return 1 - pow(-2 * t + 2, 3) / 2
    
    def get_current_go1_positions(self):
        """
        현재 시간에 따른 GO1 조인트 위치 계산 (보간)
        """
        current_time = self.publish_count * 0.1  # 10Hz 기준
        
        if current_time <= self.standup_duration:
            # 웅크리기 → 서기 이동 단계
            t = current_time / self.standup_duration
            t = max(0.0, min(1.0, t))  # 0.0 ~ 1.0으로 제한
            alpha = self.smooth_interpolation(t)  # 부드러운 보간
            
            # 각 조인트에 대해 보간
            current_positions = {}
            for joint in self.go1_joints:
                crouch_pos = self.go1_crouch_positions[joint]
                home_pos = self.go1_home_positions[joint]
                current_positions[joint] = crouch_pos + alpha * (home_pos - crouch_pos)
        else:
            # 안정화 단계 (home position 유지)
            current_positions = self.go1_home_positions.copy()
        
        return current_positions
    
    def timer_callback(self):
        """타이머 콜백: 주기적으로 home position 명령 발행"""
        if self.publish_count >= self.max_publish_count:
            self.get_logger().info('✅ Home position 명령 발행 완료')
            self.get_logger().info(f'   총 {self.total_duration}초 동안 실행 (이동: {self.standup_duration}초, 안정화: {self.stabilize_duration}초)')
            self.timer.cancel()
            # 노드 종료를 위한 플래그 설정
            if not hasattr(self, '_shutdown_requested'):
                self._shutdown_requested = True
                # 약간의 지연 후 노드 종료
                self.create_timer(0.1, self.shutdown_timer_callback)
            return
        
        try:
            # 현재 시간에 따른 GO1 조인트 위치 계산 (보간)
            current_go1_positions = self.get_current_go1_positions()
            go1_joint_positions = [current_go1_positions[joint] for joint in self.go1_joints]
            
            # K1 조인트 위치
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
            
            # 발행 (두 토픽 모두)
            self.joint_command_publisher.publish(msg)
            self.joint_command_publisher_alt.publish(msg)
            
            current_time = self.publish_count * 0.1
            self.publish_count += 1
            
            # 첫 번째 발행 시 상세 정보 출력
            if self.publish_count == 1:
                self.get_logger().info('='*60)
                self.get_logger().info('📤 웅크리기 → 서기 이동 시작')
                self.get_logger().info('='*60)
                self.get_logger().info('GO1 초기 자세 (웅크리기):')
                for leg in ['FR', 'FL', 'RR', 'RL']:
                    hip = math.degrees(self.go1_crouch_positions[f'{leg}_hip_joint'])
                    thigh = math.degrees(self.go1_crouch_positions[f'{leg}_thigh_joint'])
                    calf = math.degrees(self.go1_crouch_positions[f'{leg}_calf_joint'])
                    self.get_logger().info(f'  {leg}: hip={hip:.1f}°, thigh={thigh:.1f}°, calf={calf:.1f}°')
                self.get_logger().info('GO1 목표 자세 (서기):')
                for leg in ['FR', 'FL', 'RR', 'RL']:
                    hip = math.degrees(self.go1_home_positions[f'{leg}_hip_joint'])
                    thigh = math.degrees(self.go1_home_positions[f'{leg}_thigh_joint'])
                    calf = math.degrees(self.go1_home_positions[f'{leg}_calf_joint'])
                    self.get_logger().info(f'  {leg}: hip={hip:.1f}°, thigh={thigh:.1f}°, calf={calf:.1f}°')
                self.get_logger().info('K1 조인트: 모두 0.0 rad (home position)')
                self.get_logger().info('='*60)
            elif self.publish_count % 10 == 0:  # 1초마다
                if current_time <= self.standup_duration:
                    progress = (current_time / self.standup_duration) * 100
                    self.get_logger().info(f'🔄 웅크리기 → 서기 이동 중... ({current_time:.1f}s/{self.standup_duration:.1f}s, {progress:.0f}%)')
                else:
                    stabilize_time = current_time - self.standup_duration
                    self.get_logger().info(f'✅ 서기 자세 안정화 중... ({stabilize_time:.1f}s/{self.stabilize_duration:.1f}s)')
                
        except Exception as e:
            self.get_logger().error(f'Error publishing home position command: {e}')
    
    def shutdown_timer_callback(self):
        """종료 타이머 콜백"""
        sys.exit(0)


def main(args=None):
    rclpy.init(args=args)
    
    node = HomePositionController()
    
    try:
        node.get_logger().info('\n🔄 Home Position Controller 실행 중...\n')
        node.get_logger().info('   웅크리기 → 서기 자세로 이동 중...\n')
        node.get_logger().info('   종료하려면 Ctrl+C를 누르세요.\n')
        
        # 타이머가 완료될 때까지 실행
        try:
            rclpy.spin(node)
        except Exception:
            pass  # shutdown 예외 무시
        
        node.get_logger().info('\n✅ Home position 제어 완료\n')
        
    except KeyboardInterrupt:
        node.get_logger().info('\n\n⚠️  사용자에 의해 중단됨')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

