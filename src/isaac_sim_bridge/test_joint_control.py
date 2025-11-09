#!/usr/bin/env python3
"""
Joint Command 발행 후 로봇 제어 확인 스크립트
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import time
import math


class JointControlTester(Node):
    def __init__(self):
        super().__init__('joint_control_tester')
        
        # Command 구독
        self.cmd_sub = self.create_subscription(
            JointState, '/joint_command', self.cmd_callback, 10)
        
        # State 구독
        self.state_sub = self.create_subscription(
            JointState, '/joint_states', self.state_callback, 10)
        
        self.last_cmd = None
        self.last_state = None
        self.check_count = 0
        
        self.timer = self.create_timer(1.0, self.compare)
        
    def cmd_callback(self, msg):
        self.last_cmd = msg
        
    def state_callback(self, msg):
        self.last_state = msg
        
    def compare(self):
        self.check_count += 1
        
        if self.last_cmd is None:
            if self.check_count <= 3:
                self.get_logger().warn(f'[{self.check_count}] Command 대기 중...')
            return
            
        if self.last_state is None:
            if self.check_count <= 3:
                self.get_logger().warn(f'[{self.check_count}] State 대기 중...')
            return
        
        # 딕셔너리 생성
        cmd_dict = {}
        for i, name in enumerate(self.last_cmd.name):
            if i < len(self.last_cmd.position):
                cmd_dict[name] = self.last_cmd.position[i]
        
        state_dict = {}
        for i, name in enumerate(self.last_state.name):
            if i < len(self.last_state.position):
                state_dict[name] = self.last_state.position[i]
        
        print('='*60)
        print(f'Joint Command vs State 비교 ({self.check_count})')
        print('='*60)
        
        # GO1 조인트 확인
        go1_joints = ['FR_hip_joint', 'FR_thigh_joint', 'FR_calf_joint']
        total_error = 0.0
        max_error = 0.0
        
        for joint in go1_joints:
            cmd = cmd_dict.get(joint)
            state = state_dict.get(joint)
            
            if cmd is None or state is None:
                continue
                
            diff = abs(cmd - state)
            total_error += diff
            max_error = max(max_error, diff)
            
            status = '✅' if diff < 0.1 else '❌'
            print(f'{status} {joint}:')
            print(f'  Command: {cmd:.4f} rad ({cmd*57.3:.2f}°)')
            print(f'  State:   {state:.4f} rad ({state*57.3:.2f}°)')
            print(f'  차이:    {diff:.4f} rad ({diff*57.3:.2f}°)')
        
        print('='*60)
        print(f'평균 오차: {total_error/len(go1_joints):.4f} rad ({total_error/len(go1_joints)*57.3:.2f}°)')
        print(f'최대 오차: {max_error:.4f} rad ({max_error*57.3:.2f}°)')
        
        if max_error < 0.1:
            print('✅ 로봇이 명령을 잘 따르고 있습니다!')
        else:
            print('⚠️  로봇이 명령과 차이가 있습니다.')
        
        print('='*60)
        print()
        
        if self.check_count >= 10:
            self.timer.cancel()
            rclpy.shutdown()


def main():
    rclpy.init()
    node = JointControlTester()
    
    try:
        print('Joint Command와 State를 모니터링 중...')
        print('(최대 10초간 확인)')
        print()
        rclpy.spin(node)
    except:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

