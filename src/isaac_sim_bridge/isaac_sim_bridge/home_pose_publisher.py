#!/usr/bin/env python3
"""
GO1과 K1 모두 home 자세를 유지하는 joint command를 publish하는 ROS2 노드

사용법:
    ros2 run isaac_sim_bridge home_pose_publisher
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


class HomePosePublisher(Node):
    """GO1과 K1 모두 home 자세를 유지하는 명령을 퍼블리시하는 ROS2 노드"""
    
    def __init__(self):
        super().__init__('home_pose_publisher')
        
        self.publisher_ = self.create_publisher(
            JointState, 
            'joint_command',  # Isaac Sim의 ros2_subscribe_joint_state 노드가 구독하는 토픽
            10)
        
        # 조인트 정의 (isaac_sim_bridge와 동일)
        self.go1_joints = [
            'FR_hip_joint', 'FR_thigh_joint', 'FR_calf_joint',
            'FL_hip_joint', 'FL_thigh_joint', 'FL_calf_joint',
            'RR_hip_joint', 'RR_thigh_joint', 'RR_calf_joint',
            'RL_hip_joint', 'RL_thigh_joint', 'RL_calf_joint',
        ]
        
        self.k1_joints = [
            'joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6',
            'joint_gripper_left', 'joint_gripper_right'
        ]
        
        self.all_joints = self.go1_joints + self.k1_joints
        
        # GO1 Stand 자세 정의 (라디안)
        # go1_python_controller.py의 standing_pos를 참고:
        # [0.0, 0.67, -1.3] - hip, thigh, calf
        # 모든 다리가 동일한 stand 자세 사용
        self.go1_stand_positions = {
            'hip': 0.0,
            'thigh': 0.67,
            'calf': -1.3
        }
        
        # K1 Home 자세 정의 (모든 조인트 0 또는 중립 위치)
        # 그리퍼는 닫힌 상태 (0)
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
        
        # 타이머 설정 (10Hz로 명령 전송 - 충분히 빠르게)
        timer_period = 0.1  # 10Hz
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        self.get_logger().info('='*60)
        self.get_logger().info('✅ Home Pose Publisher 시작됨 (Stand 자세)')
        self.get_logger().info('='*60)
        self.get_logger().info(f'  - 퍼블리시 토픽: joint_command')
        self.get_logger().info(f'  - 조인트 개수: {len(self.all_joints)} (GO1: {len(self.go1_joints)}, K1: {len(self.k1_joints)})')
        self.get_logger().info(f'  - 발행 주기: 10Hz')
        self.get_logger().info(f'  - 모드: Stand 자세 유지')
        self.get_logger().info(f'  - GO1 Stand: hip=0.0, thigh=0.67, calf=-1.3 rad')
        self.get_logger().info('='*60)
        
        # 첫 번째 명령 즉시 전송
        positions = self.publish_stand_command()
        self.get_logger().info('✅ 첫 번째 stand 자세 명령 전송 완료')
        # 디버그: 실제 전송된 값 출력
        self.get_logger().info(f'   GO1 FR: hip={positions[0]:.3f}, thigh={positions[1]:.3f}, calf={positions[2]:.3f} rad')
        self.get_logger().info(f'   GO1 FL: hip={positions[3]:.3f}, thigh={positions[4]:.3f}, calf={positions[5]:.3f} rad')
    
    def get_go1_position(self, joint_name: str) -> float:
        """GO1 조인트 이름에서 stand 위치 반환"""
        if 'hip' in joint_name:
            return self.go1_stand_positions['hip']
        elif 'thigh' in joint_name:
            return self.go1_stand_positions['thigh']
        elif 'calf' in joint_name:
            return self.go1_stand_positions['calf']
        else:
            return 0.0
    
    def publish_stand_command(self):
        """Stand 자세 명령 퍼블리시 (go1_python_controller.py의 standing_pos 참고)"""
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.name = self.all_joints
        
        positions = []
        
        # GO1 조인트: stand 자세 (모든 다리 동일: [0.0, 0.67, -1.3])
        for joint_name in self.go1_joints:
            positions.append(self.get_go1_position(joint_name))
        
        # K1 조인트: home 자세 (모두 0 또는 정의된 값)
        for joint_name in self.k1_joints:
            positions.append(self.k1_home_positions.get(joint_name, 0.0))
        
        msg.position = positions
        msg.velocity = []  # 속도 제어는 선택적
        msg.effort = []   # 토크 제어는 선택적
        
        self.publisher_.publish(msg)
        return positions
    
    def timer_callback(self):
        """주기적으로 stand 자세 명령 퍼블리시"""
        positions = self.publish_stand_command()
        
        # 주기적으로 로그 출력 (5초마다)
        current_time = self.get_clock().now()
        sec = current_time.nanoseconds / 1e9
        if int(sec) % 5 == 0 and (current_time.nanoseconds % 1e9) < 1e8:
            # 실제 전송되는 값 확인을 위한 상세 로그
            self.get_logger().info('✅ Stand 자세 명령 전송 중...')
            self.get_logger().info(f'   GO1 FR: [{positions[0]:.3f}, {positions[1]:.3f}, {positions[2]:.3f}] rad')
            self.get_logger().info(f'   GO1 FL: [{positions[3]:.3f}, {positions[4]:.3f}, {positions[5]:.3f}] rad')
            self.get_logger().info(f'   K1 첫 조인트: {positions[len(self.go1_joints)]:.3f} rad')


def main(args=None):
    rclpy.init(args=args)
    
    node = HomePosePublisher()
    
    try:
        node.get_logger().info('\n🔄 Stand 자세 명령 전송 시작... (Ctrl+C로 종료)\n')
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('\n\n✅ Home Pose Publisher 종료됨')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

