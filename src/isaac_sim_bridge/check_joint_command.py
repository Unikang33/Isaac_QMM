#!/usr/bin/env python3
"""
Joint Command와 Joint States를 비교하는 간단한 스크립트
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import time


class JointChecker(Node):
    def __init__(self):
        super().__init__('joint_checker')
        
        self.cmd_sub = self.create_subscription(
            JointState, '/joint_command', self.cmd_callback, 10)
        self.state_sub = self.create_subscription(
            JointState, '/joint_states', self.state_callback, 10)
        
        self.last_cmd = None
        self.last_state = None
        self.check_count = 0
        
        self.timer = self.create_timer(1.0, self.check)
        
    def cmd_callback(self, msg):
        self.last_cmd = msg
        
    def state_callback(self, msg):
        self.last_state = msg
        
    def check(self):
        self.check_count += 1
        
        if self.last_cmd is None:
            self.get_logger().warn(f'[{self.check_count}] Command 없음')
            return
            
        if self.last_state is None:
            self.get_logger().warn(f'[{self.check_count}] State 없음')
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
        for joint in go1_joints:
            cmd = cmd_dict.get(joint)
            state = state_dict.get(joint)
            
            if cmd is None:
                print(f'❌ {joint}: Command에 없음')
                continue
            if state is None:
                print(f'❌ {joint}: State에 없음')
                continue
                
            diff = abs(cmd - state)
            status = '✅' if diff < 0.1 else '❌'
            print(f'{status} {joint}:')
            print(f'  Command: {cmd:.4f} rad ({cmd*57.3:.2f}°)')
            print(f'  State:   {state:.4f} rad ({state*57.3:.2f}°)')
            print(f'  차이:    {diff:.4f} rad ({diff*57.3:.2f}°)')
        
        print('='*60)
        
        if self.check_count >= 5:
            self.timer.cancel()
            rclpy.shutdown()


def main():
    rclpy.init()
    node = JointChecker()
    
    try:
        rclpy.spin(node)
    except:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

