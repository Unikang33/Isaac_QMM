# Isaac Sim ROS2 Bridge Extension 설정 확인 가이드

## 문제 상황
- ✅ `/joint_states` 토픽이 발행되고 있음 (Isaac Sim → ROS2)
- ❌ `/joint_command` 토픽이 발행되어도 로봇이 움직이지 않음 (ROS2 → Isaac Sim)

## 가능한 원인
Isaac Sim ROS2 Bridge Extension이 다음 중 하나일 수 있습니다:
1. **Extension이 활성화되지 않음**
2. **Extension이 활성화되었지만 Joint Command Publisher가 설정되지 않음**
3. **Extension 설정에서 토픽 이름이 다름**
4. **제어 모드(Position Control)가 활성화되지 않음**

## 확인 방법

### 1. Isaac Sim Extension Manager 확인
1. Isaac Sim 실행
2. **Window → Extensions** 메뉴 열기
3. **ROS2 Bridge** 또는 **omni.isaac.ros_bridge** 확장 확인
4. 확장이 **Enabled** 상태인지 확인

### 2. Isaac Sim에서 Joint Command Publisher 설정 확인
1. Isaac Sim에서 로봇 선택
2. **Properties** 패널에서 ROS2 관련 설정 확인
3. **ROS2 Bridge** 또는 **Joint Command Publisher** 설정 확인
4. 다음 설정 확인:
   - **Topic Name**: `/joint_command` 또는 `joint_command`
   - **Control Mode**: `Position` 또는 `Velocity`
   - **Enabled**: `True`

### 3. 토픽 이름 확인
Isaac Sim에서 기대하는 토픽 이름이 다를 수 있습니다:
- `/joint_command` (현재 사용 중)
- `/quadmani/joint_commands` (README에 언급됨)
- `/robot/joint_commands`
- 다른 네임스페이스

### 4. Isaac Sim Python Script 확인
Isaac Sim에서 ROS2 Bridge를 설정하는 Python 스크립트가 있는지 확인:
- `*.py` 파일에서 `JointCommandPublisher` 또는 `ros_bridge` 관련 코드 찾기
- Extension 설정 스크립트 확인

## 해결 방법

### 방법 1: Extension에서 Joint Command Publisher 추가
1. Isaac Sim Extension Manager에서 ROS2 Bridge Extension 확인
2. Extension 설정에서 **Joint Command Publisher** 추가
3. 다음 설정 입력:
   - **Topic Name**: `joint_command` 또는 `/joint_command`
   - **Control Mode**: `Position`
   - **Joint Names**: 모든 조인트 이름 (자동으로 감지되거나 수동 입력)

### 방법 2: Isaac Sim Python Script로 설정
Isaac Sim Script Editor에서 다음 코드 실행:

```python
from omni.isaac.core_nodes.scripts.utils import set_target_prims
from omni.isaac.ros_bridge import ROSBridgeExtension

# ROS Bridge Extension 가져오기
ros_bridge = ROSBridgeExtension()

# Joint Command Publisher 추가
ros_bridge.add_joint_command_publisher(
    topic_name="joint_command",
    robot_prim_path="/World/quadmani",  # 또는 실제 로봇 경로
    joint_names=["FR_hip_joint", "FR_thigh_joint", ...],  # 모든 조인트
    control_mode="position"
)
```

### 방법 3: 다른 토픽 시도
README에 `/quadmani/joint_commands`가 언급되어 있으므로, 이 토픽도 발행해보세요:

```python
pub = node.create_publisher(JointState, '/quadmani/joint_commands', 10)
```

## 확인 체크리스트
- [ ] ROS2 Bridge Extension이 활성화되어 있는가?
- [ ] Joint Command Publisher가 Extension에 추가되어 있는가?
- [ ] 토픽 이름이 `/joint_command`로 설정되어 있는가?
- [ ] 제어 모드가 Position으로 설정되어 있는가?
- [ ] 로봇이 Physics 시뮬레이션 모드인가?
- [ ] 로봇의 모든 조인트가 제어 가능한 상태인가?

## 참고
- Isaac Sim ROS2 Bridge 공식 문서: https://docs.omniverse.nvidia.com/app_isaacsim/app_isaacsim/ext_omni_isaac_ros_bridge.html
- Extension API: `omni.isaac.ros_bridge`

