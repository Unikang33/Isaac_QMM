#!/usr/bin/env python3
"""
Isaac Sim에서 hip position을 읽어서 ROS2 토픽으로 퍼블리시하는 노드

Isaac Sim Script Editor 또는 Extension에서 실행되어야 합니다.
Isaac Sim의 Python API를 사용하여 FR_hip, FL_hip, RR_hip, RL_hip의
world frame 기준 position을 읽어옵니다.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from std_msgs.msg import Header
import numpy as np
from typing import Optional

# Isaac Sim API
try:
    from omni.isaac.core import World
    from omni.isaac.core.utils.stage import get_current_stage
    from pxr import UsdGeom, Usd, Gf
    ISAAC_SIM_AVAILABLE = True
except ImportError:
    ISAAC_SIM_AVAILABLE = False
    print("⚠️  Isaac Sim API를 사용할 수 없습니다. Isaac Sim Script Editor에서 실행해야 합니다.")


class IsaacHipPositionPublisher(Node):
    """Isaac Sim에서 hip position을 읽어서 ROS2로 발행하는 노드"""
    
    def __init__(self):
        super().__init__('isaac_hip_position_publisher')
        
        # 각 hip의 position을 발행하는 퍼블리셔
        self.hip_publishers = {}
        hip_names = ['FR_hip', 'FL_hip', 'RR_hip', 'RL_hip']
        
        for hip_name in hip_names:
            self.hip_publishers[hip_name] = self.create_publisher(
                Point,
                f'{hip_name.lower()}_position',
                10
            )
        
        # 모든 hip position을 하나의 메시지로 발행하는 퍼블리셔
        self.all_hips_publisher = self.create_publisher(
            Point,  # 배열 대신 Point를 여러 개 발행하거나, 커스텀 메시지 사용 가능
            'all_hip_positions',
            10
        )
        
        # 타이머 설정 (30Hz)
        self.timer = self.create_timer(1.0/30.0, self.publish_hip_positions)
        
        # Isaac Sim stage 접근 및 hip prim 저장소 초기화
        self.stage = None
        self.hip_prims = {}  # 캐시된 prim 참조
        self.hip_prim_paths = {}  # 찾은 경로 저장
        
        if ISAAC_SIM_AVAILABLE:
            try:
                self.stage = get_current_stage()
                self.get_logger().info('✅ Isaac Sim stage 접근 성공')
                # Hip prim 찾기
                self.find_hip_prims()
            except Exception as e:
                self.get_logger().error(f'Isaac Sim stage 접근 실패: {e}')
        
        self.get_logger().info('='*60)
        self.get_logger().info('✅ Isaac Hip Position Publisher 시작됨')
        self.get_logger().info('='*60)
        self.get_logger().info('  - Isaac Sim에서 hip position 읽기')
        self.get_logger().info('  - ROS2 토픽으로 발행')
        self.get_logger().info('='*60)
    
    def find_hip_prims(self):
        """Isaac Sim stage에서 hip prim들을 찾기"""
        if not ISAAC_SIM_AVAILABLE or self.stage is None:
            return
        
        hip_names = ['FR_hip', 'FL_hip', 'RR_hip', 'RL_hip']
        
        # 이미 찾았으면 스킵
        if len(self.hip_prims) == 4:
            return
        
        # Stage를 순회하면서 hip prim 찾기
        for prim in Usd.PrimRange.Stage(self.stage):
            prim_name = prim.GetName()
            
            if prim_name in hip_names and prim not in self.hip_prims.values():
                self.hip_prims[prim_name] = prim
                self.hip_prim_paths[prim_name] = prim.GetPath()
                self.get_logger().info(f'✅ {prim_name} 찾음: {prim.GetPath()}')
        
        if len(self.hip_prims) == 4:
            self.get_logger().info('✅ 모든 hip prim 찾기 완료!')
        else:
            self.get_logger().warn(f'⚠️  일부 hip prim을 찾지 못했습니다. (찾은 개수: {len(self.hip_prims)}/4)')
    
    def get_hip_position_from_isaac_sim(self, hip_name: str) -> Optional[np.ndarray]:
        """
        Isaac Sim에서 특정 hip의 world frame 기준 position 읽기
        
        Args:
            hip_name: 'FR_hip', 'FL_hip', 'RR_hip', 'RL_hip'
        
        Returns:
            [x, y, z] position 또는 None
        """
        if not ISAAC_SIM_AVAILABLE or self.stage is None:
            return None
        
        try:
            # Hip prim 찾기 (캐시된 것 사용)
            if hip_name not in self.hip_prims:
                self.find_hip_prims()
            
            if hip_name not in self.hip_prims:
                return None
            
            prim = self.hip_prims[hip_name]
            
            if not prim.IsValid():
                return None
            
            # Xform에서 world transform 가져오기
            xform = UsdGeom.Xformable(prim)
            if not xform:
                return None
            
            # Local to world transform 계산
            time_code = Usd.TimeCode.Default()
            world_transform = xform.ComputeLocalToWorldTransform(time_code)
            
            # Translation 추출 (Gf.Vec3d)
            translation = world_transform.ExtractTranslation()
            
            return np.array([translation[0], translation[1], translation[2]])
            
        except Exception as e:
            self.get_logger().error(f'{hip_name} position 읽기 오류: {e}')
            return None
    
    def publish_hip_positions(self):
        """모든 hip position을 읽어서 ROS2 토픽으로 발행"""
        try:
            hip_positions = {}
            
            for hip_name in ['FR_hip', 'FL_hip', 'RR_hip', 'RL_hip']:
                # Isaac Sim에서 position 읽기
                position = self.get_hip_position_from_isaac_sim(hip_name)
                
                if position is not None:
                    hip_positions[hip_name] = position
                    
                    # 개별 토픽으로 발행
                    msg = Point()
                    msg.x = float(position[0])
                    msg.y = float(position[1])
                    msg.z = float(position[2])
                    self.hip_publishers[hip_name].publish(msg)
                else:
                    self.get_logger().warn(f'{hip_name} position을 읽을 수 없습니다.')
            
            # 모든 hip position이 성공적으로 읽혔는지 확인
            if len(hip_positions) == 4:
                # 성공적으로 읽은 경우 로그 출력 (5초마다)
                current_time = self.get_clock().now()
                sec = current_time.nanoseconds / 1e9
                if int(sec) % 5 == 0 and (current_time.nanoseconds % 1e9) < 1e8:
                    self.get_logger().info('📊 Hip Positions 발행 중...')
                    for hip_name, pos in hip_positions.items():
                        self.get_logger().info(f'   {hip_name}: [{pos[0]:.4f}, {pos[1]:.4f}, {pos[2]:.4f}] m')
            
        except Exception as e:
            self.get_logger().error(f'Hip position 발행 오류: {e}')


def main(args=None):
    if not ISAAC_SIM_AVAILABLE:
        print("="*60)
        print("⚠️  경고: Isaac Sim API를 사용할 수 없습니다.")
        print("   이 노드는 Isaac Sim Script Editor 또는 Extension에서 실행해야 합니다.")
        print("="*60)
        return
    
    rclpy.init(args=args)
    
    node = IsaacHipPositionPublisher()
    
    try:
        node.get_logger().info('\n🔄 Isaac Hip Position Publisher 실행 중... (Ctrl+C로 종료)\n')
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('\n\n✅ Isaac Hip Position Publisher 종료됨')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

