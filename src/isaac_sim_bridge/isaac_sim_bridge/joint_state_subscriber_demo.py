#!/usr/bin/env python3
"""
joint_states 토픽을 구독하여 조인트 상태를 출력하는 데모 노드
Isaac Sim의 ros2_publish_joint_state 노드가 발행하는 토픽을 구독합니다.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


class JointStateSubscriberDemo(Node):
    """조인트 상태를 구독하여 콘솔에 출력하는 데모"""
    
    def __init__(self):
        super().__init__('joint_state_subscriber_demo')
        
        self.subscription = self.create_subscription(
            JointState,
            'joint_states',  # Isaac Sim의 ros2_publish_joint_state 노드가 발행하는 토픽
            self.joint_state_callback,
            10)
        
        self.subscription  # 사용되지 않음 경고 방지
        
        self.get_logger().info('joint_states 토픽 구독 시작')
    
    def joint_state_callback(self, msg: JointState):
        """조인트 상태 콜백"""
        self.get_logger().info(
            f'\n=== 조인트 상태 ===\n'
            f'타임스탬프: {msg.header.stamp.sec}.{msg.header.stamp.nanosec}\n'
            f'조인트 개수: {len(msg.name)}\n'
            f'조인트 목록: {", ".join(msg.name[:5])}{"..." if len(msg.name) > 5 else ""}\n'
            f'위치: {[f"{p:.3f}" for p in msg.position[:5]]}{"..." if len(msg.position) > 5 else ""}\n'
            f'속도: {[f"{v:.3f}" for v in msg.velocity[:5]]}{"..." if len(msg.velocity) > 5 else ""}\n'
        )
        
        # 특정 조인트 값 출력
        if len(msg.name) > 0:
            for i, name in enumerate(msg.name):
                if i < len(msg.position):
                    pos = msg.position[i]
                    if abs(pos) > 0.001:  # 0이 아닌 조인트만 출력
                        self.get_logger().info(
                            f'  {name}: position={pos:.4f} rad '
                            f'({pos*180/3.14159:.2f} deg)',
                            throttle_duration_sec=1.0)  # 1초에 한 번만 출력


def main(args=None):
    rclpy.init(args=args)
    
    node = JointStateSubscriberDemo()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

