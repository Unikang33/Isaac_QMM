#!/usr/bin/env python3
"""
Joint state 확인 스크립트
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


class JointStateChecker(Node):
    def __init__(self):
        super().__init__('joint_state_checker')
        self.subscription = self.create_subscription(
            JointState,
            'joint_states',
            self.joint_state_callback,
            10)
        self.message_count = 0
        self.get_logger().info('✅ joint_states 토픽 구독 시작')
    
    def joint_state_callback(self, msg: JointState):
        self.message_count += 1
        
        if self.message_count == 1:
            self.get_logger().info('\n' + '='*60)
            self.get_logger().info('✅ Joint State 수신 성공!')
            self.get_logger().info('='*60)
            self.get_logger().info(f'조인트 개수: {len(msg.name)}')
            self.get_logger().info(f'타임스탬프: {msg.header.stamp.sec}.{msg.header.stamp.nanosec}')
            
            # GO1 조인트 값 확인
            go1_joints = ['FR_hip_joint', 'FR_thigh_joint', 'FR_calf_joint',
                         'FL_hip_joint', 'FL_thigh_joint', 'FL_calf_joint',
                         'RR_hip_joint', 'RR_thigh_joint', 'RR_calf_joint',
                         'RL_hip_joint', 'RL_thigh_joint', 'RL_calf_joint']
            
            self.get_logger().info('\n=== GO1 조인트 상태 ===')
            for joint_name in go1_joints:
                if joint_name in msg.name:
                    idx = msg.name.index(joint_name)
                    pos = msg.position[idx] if idx < len(msg.position) else 0.0
                    vel = msg.velocity[idx] if idx < len(msg.velocity) else 0.0
                    self.get_logger().info(f'  {joint_name:20s}: pos={pos:7.4f} rad, vel={vel:7.4f} rad/s')
            
            # K1 조인트 값 확인
            k1_joints = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6',
                        'joint_gripper_left', 'joint_gripper_right']
            
            self.get_logger().info('\n=== K1 조인트 상태 ===')
            for joint_name in k1_joints:
                if joint_name in msg.name:
                    idx = msg.name.index(joint_name)
                    pos = msg.position[idx] if idx < len(msg.position) else 0.0
                    vel = msg.velocity[idx] if idx < len(msg.velocity) else 0.0
                    self.get_logger().info(f'  {joint_name:20s}: pos={pos:7.4f} rad, vel={vel:7.4f} rad/s')
            
            self.get_logger().info('\n' + '='*60)
            self.get_logger().info('계속 수신 중... (Ctrl+C로 종료)')
            self.get_logger().info('='*60 + '\n')


def main():
    rclpy.init()
    node = JointStateChecker()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print(f'\n✅ 총 {node.message_count}개 메시지 수신 완료')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

