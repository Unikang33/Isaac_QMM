"""
Isaac Sim에서 실행할 스크립트 예제

이 스크립트는 Isaac Sim의 Extension 스크립트 에디터나 Python Scripting에서 실행됩니다.
Isaac Sim에서 로봇을 로드하고 ROS2 브리지 노드를 초기화합니다.
"""

from omni.isaac.core import World
from omni.isaac.core.robots import Robot
import omni.isaac.core.utils.nucleus as nucleus_utils

# ROS2 노드 임포트 (Isaac Sim 환경에서 실행되어야 함)
# 실제로는 ROS2 노드가 별도 프로세스로 실행되므로, 
# 여기서는 Isaac Sim 내에서 직접 ROS2 브리지를 구성하는 방법을 보여줍니다.

class IsaacSimQuadManiBridge:
    """Isaac Sim과 ROS2 브리지를 구성하는 클래스"""
    
    def __init__(self, usd_path: str = None):
        """
        Args:
            usd_path: 로봇 USD 파일 경로
        """
        self.world = World()
        self.robot = None
        self.usd_path = usd_path or "/home/oem/unitree_quadmani/unitree_quadmani.usd"
    
    def setup_scene(self):
        """씬 설정 및 로봇 로드"""
        # USD 파일에서 로봇 로드
        # 실제 경로는 프로젝트에 맞게 수정해야 합니다
        self.robot = self.world.scene.add(
            Robot(
                prim_path="/World/quadmani",  # Isaac Sim에서 설정한 경로
                name="quadmani",
                usd_path=self.usd_path
            )
        )
        
        # 물리 시뮬레이션 시작
        self.world.reset()
        
        print(f"로봇 로드 완료: {len(self.robot.dof_names)}개 조인트")
        print(f"조인트 목록: {self.robot.dof_names[:5]}...")  # 처음 5개만 출력
    
    def run(self):
        """시뮬레이션 실행"""
        # Isaac Sim의 스텝 함수를 사용하여 시뮬레이션 진행
        # 실제로는 Isaac Sim의 애플리케이션 루프에서 호출됩니다
        
        # 예제: 조인트 상태 읽기
        if self.robot is not None:
            positions = self.robot.get_joint_positions()
            print(f"현재 조인트 위치 (처음 5개): {positions[:5]}")


# 사용 예제 (Isaac Sim Extension 스크립트에서 실행):
"""
from isaac_sim_bridge.isaac_sim_script import IsaacSimQuadManiBridge

bridge = IsaacSimQuadManiBridge()
bridge.setup_scene()

# Isaac Sim 애플리케이션 루프에서 계속 실행
while True:
    bridge.run()
    # Isaac Sim 스텝 진행
"""

