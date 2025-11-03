#!/usr/bin/env python3
"""
ROS2 /quadmani/joint_commands 토픽을 구독하여 Isaac Sim 로봇의 조인트를 제어하는 노드

Isaac Sim에서 실행되어야 합니다.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from typing import Dict, Optional, List
import numpy as np

try:
    from omni.isaac.core import World
    from omni.isaac.core.robots import Robot
    ISAAC_SIM_AVAILABLE = True
except ImportError:
    ISAAC_SIM_AVAILABLE = False
    print("Warning: Isaac Sim modules not available. This node should run in Isaac Sim environment.")


class IsaacJointCommandSubscriber(Node):
    """
    ROS2 조인트 명령을 구독하여 Isaac Sim 로봇을 제어합니다.
    """
    
    def __init__(self):
        super().__init__('isaac_joint_command_subscriber')
        
        # ROS2 Subscriber
        self.subscription = self.create_subscription(
            JointState,
            '/quadmani/joint_commands',
            self.command_callback,
            10)
        
        # 조인트 이름 정의
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
        
        # Isaac Sim 로봇 참조
        self.robot: Optional[Robot] = None
        
        # 조인트 인덱스 매핑 (캐싱용)
        self.joint_index_map: Dict[str, int] = {}
        
        # 최근 명령 저장
        self.last_command: Optional[JointState] = None
        
        self.get_logger().info(
            f'Isaac Joint Command Subscriber 시작됨. '
            f'{len(self.all_joints)}개 조인트 제어 준비 완료')
    
    def set_robot(self, robot: Robot):
        """로봇 참조 설정 및 조인트 인덱스 매핑 생성"""
        self.robot = robot
        
        if ISAAC_SIM_AVAILABLE and robot is not None:
            try:
                # 조인트 이름으로 인덱스 매핑 생성
                for i, joint_name in enumerate(robot.dof_names):
                    self.joint_index_map[joint_name] = i
                
                self.get_logger().info(
                    f'로봇 설정 완료. {len(self.joint_index_map)}개 조인트 매핑됨')
            except Exception as e:
                self.get_logger().error(f'로봇 설정 실패: {e}')
    
    def command_callback(self, msg: JointState):
        """조인트 명령 콜백"""
        if self.robot is None:
            self.get_logger().warn('로봇이 설정되지 않았습니다. 명령을 무시합니다.')
            return
        
        if not ISAAC_SIM_AVAILABLE:
            self.get_logger().warn('Isaac Sim 모듈을 사용할 수 없습니다.')
            return
        
        try:
            # 명령을 로봇의 조인트 순서에 맞게 변환
            joint_positions = np.zeros(len(self.robot.dof_names))
            
            # 메시지의 조인트 명령을 파싱
            for i, joint_name in enumerate(msg.name):
                if joint_name in self.joint_index_map:
                    idx = self.joint_index_map[joint_name]
                    if i < len(msg.position):
                        joint_positions[idx] = msg.position[i]
                else:
                    self.get_logger().debug(
                        f'알 수 없는 조인트: {joint_name}')
            
            # Isaac Sim에 조인트 위치 명령 전송
            self.robot.set_joint_positions(joint_positions)
            
            self.last_command = msg
            self.get_logger().debug(
                f'조인트 명령 적용됨: {len(msg.name)}개 조인트')
            
        except Exception as e:
            self.get_logger().error(f'조인트 명령 처리 실패: {e}')
    
    def get_last_command(self) -> Optional[JointState]:
        """마지막 명령 반환"""
        return self.last_command


def main(args=None):
    rclpy.init(args=args)
    
    node = IsaacJointCommandSubscriber()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

