#!/usr/bin/env python3
"""
Home position 제어 확인 스크립트

joint_command와 joint_states를 비교하여 home position이 제대로 적용되는지 확인합니다.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import math
import time


class HomePositionChecker(Node):
    """Home position 제어 확인 노드"""
    
    def __init__(self):
        super().__init__('home_position_checker')
        
        # Joint command 구독
        self.joint_command_subscriber = self.create_subscription(
            JointState,
            '/joint_command',
            self.joint_command_callback,
            10
        )
        
        # Joint state 구독
        self.joint_state_subscriber = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )
        
        # GO1 home positions
        self.go1_home_positions = {
            'FR_hip_joint': 0.0,
            'FR_thigh_joint': 0.67,
            'FR_calf_joint': -1.3,
            'FL_hip_joint': 0.0,
            'FL_thigh_joint': 0.67,
            'FL_calf_joint': -1.3,
            'RR_hip_joint': 0.0,
            'RR_thigh_joint': 0.67,
            'RR_calf_joint': -1.3,
            'RL_hip_joint': 0.0,
            'RL_thigh_joint': 0.67,
            'RL_calf_joint': -1.3,
        }
        
        # K1 home positions
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
        
        self.all_home_positions = {**self.go1_home_positions, **self.k1_home_positions}
        
        # 현재 상태 저장
        self.current_command = None
        self.current_state = None
        
        # 확인 횟수
        self.check_count = 0
        self.max_checks = 10
        
        # 주기적으로 확인 (1Hz)
        self.timer = self.create_timer(1.0, self.timer_callback)
        
        self.get_logger().info('='*60)
        self.get_logger().info('✅ Home Position Checker 시작됨')
        self.get_logger().info('='*60)
        self.get_logger().info('  - /joint_command 구독 중...')
        self.get_logger().info('  - /joint_states 구독 중...')
        self.get_logger().info('  - Home position과 비교 중...')
        self.get_logger().info('='*60)
    
    def joint_command_callback(self, msg: JointState):
        """Joint command 수신"""
        self.current_command = msg
        self.get_logger().debug(f'Joint command 수신: {len(msg.name)}개 조인트')
    
    def joint_state_callback(self, msg: JointState):
        """Joint state 수신"""
        self.current_state = msg
        self.get_logger().debug(f'Joint state 수신: {len(msg.name)}개 조인트')
    
    def timer_callback(self):
        """타이머 콜백: 주기적으로 확인"""
        if self.check_count >= self.max_checks:
            self.get_logger().info('✅ 확인 완료')
            return
        
        self.check_count += 1
        
        if self.current_command is None:
            self.get_logger().warn(f'[{self.check_count}/{self.max_checks}] Joint command를 아직 받지 못했습니다.')
            return
        
        if self.current_state is None:
            self.get_logger().warn(f'[{self.check_count}/{self.max_checks}] Joint state를 아직 받지 못했습니다.')
            return
        
        # Command와 State 비교
        self.compare_command_and_state()
    
    def compare_command_and_state(self):
        """Command와 State 비교"""
        command_dict = {}
        for i, name in enumerate(self.current_command.name):
            if i < len(self.current_command.position):
                command_dict[name] = self.current_command.position[i]
        
        state_dict = {}
        for i, name in enumerate(self.current_state.name):
            if i < len(self.current_state.position):
                state_dict[name] = self.current_state.position[i]
        
        # Home position 확인
        self.get_logger().info('='*60)
        self.get_logger().info(f'📊 Home Position 확인 ({self.check_count}/{self.max_checks})')
        self.get_logger().info('='*60)
        
        # GO1 조인트 확인
        go1_errors = []
        for joint_name, home_pos in self.go1_home_positions.items():
            cmd_pos = command_dict.get(joint_name, None)
            state_pos = state_dict.get(joint_name, None)
            
            if cmd_pos is None:
                self.get_logger().warn(f'  {joint_name}: Command에 없음')
                continue
            
            if state_pos is None:
                self.get_logger().warn(f'  {joint_name}: State에 없음')
                continue
            
            # Command와 Home position 비교
            cmd_error = abs(cmd_pos - home_pos)
            # State와 Command 비교
            state_error = abs(state_pos - cmd_pos)
            # State와 Home position 비교
            home_error = abs(state_pos - home_pos)
            
            cmd_deg = math.degrees(cmd_pos)
            state_deg = math.degrees(state_pos)
            home_deg = math.degrees(home_pos)
            
            if cmd_error > 0.01 or state_error > 0.1 or home_error > 0.1:
                status = '❌'
                go1_errors.append(joint_name)
            else:
                status = '✅'
            
            if self.check_count == 1 or status == '❌':
                self.get_logger().info(f'  {status} {joint_name}:')
                self.get_logger().info(f'    Command: {cmd_pos:.4f} rad ({cmd_deg:.2f}°)')
                self.get_logger().info(f'    State:   {state_pos:.4f} rad ({state_deg:.2f}°)')
                self.get_logger().info(f'    Home:    {home_pos:.4f} rad ({home_deg:.2f}°)')
                self.get_logger().info(f'    오차: Command-Home={math.degrees(cmd_error):.2f}°, State-Cmd={math.degrees(state_error):.2f}°')
        
        # K1 조인트 확인
        k1_errors = []
        for joint_name, home_pos in self.k1_home_positions.items():
            cmd_pos = command_dict.get(joint_name, None)
            state_pos = state_dict.get(joint_name, None)
            
            if cmd_pos is None:
                continue
            if state_pos is None:
                continue
            
            cmd_error = abs(cmd_pos - home_pos)
            state_error = abs(state_pos - cmd_pos)
            home_error = abs(state_pos - home_pos)
            
            if cmd_error > 0.01 or state_error > 0.1 or home_error > 0.1:
                status = '❌'
                k1_errors.append(joint_name)
            else:
                status = '✅'
            
            if self.check_count == 1 or status == '❌':
                self.get_logger().info(f'  {status} {joint_name}:')
                self.get_logger().info(f'    Command: {cmd_pos:.4f} rad')
                self.get_logger().info(f'    State:   {state_pos:.4f} rad')
                self.get_logger().info(f'    Home:    {home_pos:.4f} rad')
        
        # 결과 요약
        total_go1 = len(self.go1_home_positions)
        total_k1 = len(self.k1_home_positions)
        ok_go1 = total_go1 - len(go1_errors)
        ok_k1 = total_k1 - len(k1_errors)
        
        self.get_logger().info('='*60)
        self.get_logger().info(f'📈 결과 요약:')
        self.get_logger().info(f'  GO1: {ok_go1}/{total_go1} 조인트 정확')
        self.get_logger().info(f'  K1:  {ok_k1}/{total_k1} 조인트 정확')
        
        if len(go1_errors) > 0:
            self.get_logger().warn(f'  GO1 오차 조인트: {", ".join(go1_errors)}')
        if len(k1_errors) > 0:
            self.get_logger().warn(f'  K1 오차 조인트: {", ".join(k1_errors)}')
        
        if len(go1_errors) == 0 and len(k1_errors) == 0:
            self.get_logger().info('  ✅ 모든 조인트가 home position에 있습니다!')
        
        self.get_logger().info('='*60)


def main(args=None):
    rclpy.init(args=args)
    
    node = HomePositionChecker()
    
    try:
        node.get_logger().info('\n🔄 Home Position Checker 실행 중...\n')
        node.get_logger().info('   Joint command와 joint state를 비교 중...\n')
        node.get_logger().info('   종료하려면 Ctrl+C를 누르세요.\n')
        
        rclpy.spin(node)
        
    except KeyboardInterrupt:
        node.get_logger().info('\n\n⚠️  사용자에 의해 중단됨')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()


