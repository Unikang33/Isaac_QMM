#!/usr/bin/env python3
"""
TF를 통해 로봇의 world 좌표계 기준 현재 pose를 받아오는 노드

Isaac Sim에서 발행하는 TF를 구독하여 로봇의 base_link pose를 world 좌표계 기준으로 출력합니다.
"""

import rclpy
from rclpy.node import Node
from tf2_ros import TransformListener, Buffer
from geometry_msgs.msg import TransformStamped
import math


class TFPoseListener(Node):
    """TF를 통해 로봇 pose를 구독하는 노드"""
    
    def __init__(self):
        super().__init__('tf_pose_listener')
        
        # TF Buffer와 Listener 생성
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # 로봇 base frame 이름 (Isaac Sim에서 사용하는 이름)
        self.robot_base_frame = 'sgr532_base_link'  # 또는 'base_link' 등
        self.world_frame = 'world'
        
        # 주기적으로 TF를 읽어서 pose 출력 (10Hz)
        self.timer = self.create_timer(0.1, self.timer_callback)
        
        self.get_logger().info('='*60)
        self.get_logger().info('✅ TF Pose Listener 시작됨')
        self.get_logger().info('='*60)
        self.get_logger().info(f'  - World frame: {self.world_frame}')
        self.get_logger().info(f'  - Robot base frame: {self.robot_base_frame}')
        self.get_logger().info('  - 10Hz로 pose 정보 출력')
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
    
    def timer_callback(self):
        """타이머 콜백: TF에서 pose를 읽어서 출력"""
        try:
            # TF 변환 가져오기
            transform: TransformStamped = self.tf_buffer.lookup_transform(
                self.world_frame,
                self.robot_base_frame,
                rclpy.time.Time()
            )
            
            # Translation (위치)
            trans = transform.transform.translation
            x, y, z = trans.x, trans.y, trans.z
            
            # Rotation (회전) - 쿼터니언
            rot = transform.transform.rotation
            qx, qy, qz, qw = rot.x, rot.y, rot.z, rot.w
            
            # 쿼터니언을 오일러 각도로 변환
            roll, pitch, yaw = self.quaternion_to_euler(qx, qy, qz, qw)
            
            # Pose 정보 출력
            self.get_logger().info('='*60)
            self.get_logger().info(f'🤖 로봇 Pose (world 좌표계 기준)')
            self.get_logger().info('='*60)
            self.get_logger().info(f'  위치 (Translation):')
            self.get_logger().info(f'    x: {x:.4f} m')
            self.get_logger().info(f'    y: {y:.4f} m')
            self.get_logger().info(f'    z: {z:.4f} m')
            self.get_logger().info(f'  회전 (Rotation):')
            self.get_logger().info(f'    Roll:  {math.degrees(roll):.2f}° ({roll:.4f} rad)')
            self.get_logger().info(f'    Pitch: {math.degrees(pitch):.2f}° ({pitch:.4f} rad)')
            self.get_logger().info(f'    Yaw:   {math.degrees(yaw):.2f}° ({yaw:.4f} rad)')
            self.get_logger().info(f'  쿼터니언 (xyzw):')
            self.get_logger().info(f'    x: {qx:.4f}, y: {qy:.4f}, z: {qz:.4f}, w: {qw:.4f}')
            self.get_logger().info('='*60)
            
        except Exception as e:
            self.get_logger().warn(f'TF 변환 실패: {e}')
            self.get_logger().warn(f'  - World frame: {self.world_frame}')
            self.get_logger().warn(f'  - Robot base frame: {self.robot_base_frame}')
            self.get_logger().warn('  - TF가 아직 발행되지 않았을 수 있습니다.')


def main(args=None):
    rclpy.init(args=args)
    
    node = TFPoseListener()
    
    try:
        node.get_logger().info('\n🔄 TF Pose Listener 실행 중...\n')
        node.get_logger().info('   TF를 기다리는 중...\n')
        node.get_logger().info('   종료하려면 Ctrl+C를 누르세요.\n')
        
        rclpy.spin(node)
        
    except KeyboardInterrupt:
        node.get_logger().info('\n\n⚠️  사용자에 의해 중단됨')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

