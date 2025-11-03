#!/usr/bin/env python3
"""
Isaac Sim에서 조인트 상태를 읽어서 ROS2 joint_states 토픽으로 퍼블리시하는 노드
Isaac Sim의 ros2_publish_joint_state 노드와 동일한 토픽 이름을 사용합니다.

Isaac Sim에서 실행되어야 합니다.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import numpy as np
from typing import Dict, Optional

try:
    from omni.isaac.core import World
    from omni.isaac.core.robots import Robot
    ISAAC_SIM_AVAILABLE = True
except ImportError:
    ISAAC_SIM_AVAILABLE = False
    print("Warning: Isaac Sim modules not available. This node should run in Isaac Sim environment.")


class IsaacJointStatePublisher(Node):
    """
    Isaac Sim의 로봇 조인트 상태를 읽어서 ROS2 토픽으로 퍼블리시합니다.
    """
    
    def __init__(self):
        super().__init__('isaac_joint_state_publisher')
        
        # ROS2 Publisher
        self.publisher_ = self.create_publisher(JointState, 'joint_states', 10)  # Isaac Sim과 동일한 토픽 이름
        
        # 조인트 이름 정의 (README.md에서 정의된 조인트)
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
        
        # Isaac Sim 로봇 참조 (나중에 설정됨)
        self.robot: Optional[Robot] = None
        
        # 타이머 설정 (30Hz로 퍼블리시)
        timer_period = 1.0 / 30.0  # 30Hz
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        # Isaac Sim 초기화 (런타임에서 호출)
        self._init_isaac_sim()
        
        self.get_logger().info(
            f'Isaac Joint State Publisher 시작됨. {len(self.all_joints)}개 조인트 모니터링')
    
    def _init_isaac_sim(self):
        """Isaac Sim 로봇 초기화"""
        if not ISAAC_SIM_AVAILABLE:
            self.get_logger().warn(
                'Isaac Sim 모듈을 사용할 수 없습니다. '
                '이 노드는 Isaac Sim 환경에서 실행되어야 합니다.')
            return
        
        try:
            # World가 이미 초기화되어 있다고 가정
            # 실제로는 Isaac Sim 스크립트에서 로봇을 전달받아야 함
            self.get_logger().info('Isaac Sim 초기화 완료 (로봇 참조는 별도로 설정 필요)')
        except Exception as e:
            self.get_logger().error(f'Isaac Sim 초기화 실패: {e}')
    
    def set_robot(self, robot: Robot):
        """로봇 참조 설정 (Isaac Sim 스크립트에서 호출)"""
        self.robot = robot
        self.get_logger().info('로봇 참조가 설정되었습니다.')
    
    def timer_callback(self):
        """주기적으로 조인트 상태를 읽어서 퍼블리시"""
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.name = self.all_joints
        
        if self.robot is not None and ISAAC_SIM_AVAILABLE:
            try:
                # Isaac Sim에서 조인트 상태 읽기
                positions = []
                velocities = []
                efforts = []
                
                for joint_name in self.all_joints:
                    try:
                        # 조인트 인덱스 찾기
                        joint_idx = self.robot.dof_names.index(joint_name)
                        
                        # 조인트 상태 읽기
                        positions.append(float(self.robot.get_joint_positions()[joint_idx]))
                        velocities.append(float(self.robot.get_joint_velocities()[joint_idx]))
                        efforts.append(float(self.robot.get_joint_efforts()[joint_idx]))
                    except (ValueError, IndexError) as e:
                        # 조인트를 찾을 수 없는 경우 0으로 채움
                        self.get_logger().debug(
                            f'조인트 {joint_name}을 찾을 수 없습니다: {e}')
                        positions.append(0.0)
                        velocities.append(0.0)
                        efforts.append(0.0)
                
                msg.position = positions
                msg.velocity = velocities
                msg.effort = efforts
                
            except Exception as e:
                self.get_logger().error(f'조인트 상태 읽기 실패: {e}')
                # 에러 시 0으로 채움
                msg.position = [0.0] * len(self.all_joints)
                msg.velocity = [0.0] * len(self.all_joints)
                msg.effort = [0.0] * len(self.all_joints)
        else:
            # Isaac Sim이 없는 경우 더미 데이터
            msg.position = [0.0] * len(self.all_joints)
            msg.velocity = [0.0] * len(self.all_joints)
            msg.effort = [0.0] * len(self.all_joints)
        
        self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    
    node = IsaacJointStatePublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

