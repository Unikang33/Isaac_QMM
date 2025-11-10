#!/usr/bin/env python3
"""
Joint State Subscriber 데모 노드

/joint_states 토픽을 구독하여 조인트 상태를 콘솔에 출력하는 데모입니다.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


class JointStateSubscriberDemo(Node):
    """조인트 상태 구독 데모 노드"""
    
    def __init__(self):
        super().__init__('joint_state_subscriber_demo')
        
        # Joint state 구독
        self.subscription = self.create_subscription(
            JointState,
            'joint_states',
            self.joint_state_callback,
            10
        )
        
        # 메시지 수신 카운터
        self.message_count = 0
        self.last_print_time = 0.0
        
        self.get_logger().info('='*60)
        self.get_logger().info('✅ Joint State Subscriber 데모 시작됨')
        self.get_logger().info('='*60)
        self.get_logger().info('  - 토픽: /joint_states')
        self.get_logger().info('  - 메시지 타입: sensor_msgs/JointState')
        self.get_logger().info('  - 조인트 상태를 1초마다 출력합니다')
        self.get_logger().info('='*60)
        self.get_logger().info('📥 Joint State 구독 시작...')
        self.get_logger().info('')
    
    def joint_state_callback(self, msg):
        """조인트 상태 메시지 수신 콜백"""
        self.message_count += 1
        current_time = self.get_clock().now().seconds_nanoseconds()[0]
        
        # 1초마다 출력
        if current_time - self.last_print_time >= 1.0:
            self.get_logger().info('='*60)
            self.get_logger().info(f'📊 Joint State 수신 (총 {self.message_count}회)')
            self.get_logger().info(f'  - 시간: {msg.header.stamp.sec}.{msg.header.stamp.nanosec:09d}')
            self.get_logger().info(f'  - 조인트 수: {len(msg.name)}개')
            
            if len(msg.name) > 0:
                self.get_logger().info('  - 조인트 목록:')
                for i, name in enumerate(msg.name):
                    pos = msg.position[i] if i < len(msg.position) else 0.0
                    vel = msg.velocity[i] if i < len(msg.velocity) else 0.0
                    eff = msg.effort[i] if i < len(msg.effort) else 0.0
                    self.get_logger().info(f'    [{i:2d}] {name:20s} | pos: {pos:8.4f} | vel: {vel:8.4f} | eff: {eff:8.4f}')
            
            self.get_logger().info('='*60)
            self.get_logger().info('')
            
            self.last_print_time = current_time


def main(args=None):
    rclpy.init(args=args)
    
    node = JointStateSubscriberDemo()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('\n⚠️  사용자에 의해 중단됨')
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()

