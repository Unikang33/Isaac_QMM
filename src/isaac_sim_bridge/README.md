# Isaac Sim ROS2 Bridge

Isaac Sim에서 QuadMani 로봇(Unitree GO1 + K1 Arm)을 ROS2로 제어하기 위한 브리지 패키지입니다.

## 개요

이 패키지는 Isaac Sim 시뮬레이션 환경에서 ROS2 토픽을 통해 로봇을 제어하고 모니터링할 수 있도록 합니다.

### 주요 기능

- **조인트 상태 퍼블리시**: Isaac Sim의 로봇 조인트 상태를 `/joint_states` 토픽으로 퍼블리시
- **조인트 명령 구독**: `/quadmani/joint_commands` 토픽을 구독하여 로봇 제어
- **데모 노드**: 구독 및 퍼블리시 테스트용 데모 코드

## 환경 구성

### 1. 필수 요구사항

- ROS2 (Humble 또는 Iron 권장)
- Isaac Sim (2023.1 이상)
- Python 3.8+

### 2. Isaac Sim ROS2 브리지 설정

Isaac Sim에서 ROS2 브리지를 활성화해야 합니다:

1. **Isaac Sim Extension Manager**에서 `ROS2 Bridge` 확장을 활성화
2. 또는 Isaac Sim에서 다음 명령 실행:
   ```
   Extensions > Search > "ROS2 Bridge" > Enable
   ```

### 3. 패키지 빌드

```bash
cd ~/kiro_ws  # 또는 ROS2 워크스페이스 경로
colcon build --packages-select isaac_sim_bridge --symlink-install
source install/setup.bash
```

## 사용 방법

### 방법 1: Isaac Sim 내에서 ROS2 노드 실행

Isaac Sim은 별도 프로세스이므로, 일반적으로 다음 두 가지 방법을 사용합니다:

#### A. 외부 터미널에서 ROS2 노드 실행

1. **Isaac Sim 시작** (로봇 로드 완료)

2. **터미널 1**: 조인트 상태 구독 데모 실행
   ```bash
   source ~/kiro_ws/install/setup.bash
   ros2 run isaac_sim_bridge joint_state_subscriber_demo
   ```

3. **터미널 2**: 조인트 명령 퍼블리시 데모 실행
   ```bash
   source ~/kiro_ws/install/setup.bash
   ros2 run isaac_sim_bridge joint_command_publisher_demo
   ```

#### B. Isaac Sim Extension Script로 통합

Isaac Sim의 Extension 스크립트에서 직접 ROS2 브리지를 구성할 수 있습니다.
예제는 `isaac_sim_bridge/isaac_sim_bridge/isaac_sim_script.py`를 참조하세요.

### 방법 2: Isaac Sim Python Scripting 사용

Isaac Sim의 Script Editor에서 다음 코드를 실행:

```python
from omni.isaac.core import World
from omni.isaac.core.robots import Robot
import rclpy
from isaac_sim_bridge.isaac_joint_state_publisher import IsaacJointStatePublisher

# World 초기화
world = World()

# 로봇 로드
robot = world.scene.add(
    Robot(
        prim_path="/World/quadmani",
        name="quadmani"
    )
)

# ROS2 초기화 (별도 스레드 권장)
rclpy.init()

# 노드 생성
joint_state_publisher = IsaacJointStatePublisher()
joint_state_publisher.set_robot(robot)

# Isaac Sim 루프에서 ROS2 스핀 (별도 스레드로 실행 권장)
```

## 노드 설명

### 1. `isaac_joint_state_publisher`

Isaac Sim에서 조인트 상태를 읽어 `/joint_states` 토픽으로 퍼블리시합니다.

**특징:**
- 30Hz로 조인트 상태 퍼블리시
- 모든 조인트의 위치, 속도, 토크 정보 포함

**사용법:**
```bash
ros2 run isaac_sim_bridge isaac_joint_state_publisher
```

**주의**: 이 노드는 Isaac Sim 환경 내에서 실행되어야 하며, Isaac Sim의 로봇 객체 참조가 필요합니다.

### 2. `isaac_joint_command_subscriber`

`/quadmani/joint_commands` 토픽을 구독하여 로봇 조인트를 제어합니다.

**특징:**
- 조인트 위치 명령 지원
- Isaac Sim 로봇 객체를 직접 제어

**사용법:**
```bash
ros2 run isaac_sim_bridge isaac_joint_command_subscriber
```

### 3. `joint_state_subscriber_demo`

`/joint_states` 토픽을 구독하여 조인트 상태를 콘솔에 출력하는 데모 노드입니다.

**사용법:**
```bash
ros2 run isaac_sim_bridge joint_state_subscriber_demo
```

### 4. `joint_command_publisher_demo`

간단한 조인트 움직임 시퀀스를 `/quadmani/joint_commands` 토픽으로 퍼블리시하는 데모입니다.

**특징:**
- K1 Arm 조인트에 사인파 움직임 적용
- GO1은 홈 포지션 유지

**사용법:**
```bash
ros2 run isaac_sim_bridge joint_command_publisher_demo
```

## ROS2 토픽

### 퍼블리시되는 토픽

- `/joint_states` (sensor_msgs/JointState)
  - Isaac Sim의 로봇 조인트 상태
  - 모든 조인트의 위치, 속도, 토크 정보

### 구독하는 토픽

- `/quadmani/joint_commands` (sensor_msgs/JointState)
  - 조인트 명령
  - `name`, `position` 필드 필수
  - `velocity`, `effort` 필드는 선택적

## 조인트 이름

### GO1 다리 (12개)
- `FR_hip_joint`, `FR_thigh_joint`, `FR_calf_joint`
- `FL_hip_joint`, `FL_thigh_joint`, `FL_calf_joint`
- `RR_hip_joint`, `RR_thigh_joint`, `RR_calf_joint`
- `RL_hip_joint`, `RL_thigh_joint`, `RL_calf_joint`

### K1 Arm (8개)
- `joint1`, `joint2`, `joint3`, `joint4`, `joint5`, `joint6`
- `joint_gripper_left`, `joint_gripper_right`

## 문제 해결

### 1. Isaac Sim 모듈을 찾을 수 없음

**에러**: `ImportError: cannot import name 'World' from 'omni.isaac.core'`

**해결**: 
- Isaac Sim 환경에서 실행하고 있는지 확인
- Isaac Sim Extension이 올바르게 로드되었는지 확인

### 2. 조인트를 찾을 수 없음

**에러**: `ValueError: joint_name not in dof_names`

**해결**:
- Isaac Sim에서 로봇이 올바르게 로드되었는지 확인
- 로봇의 실제 조인트 이름 확인:
  ```python
  print(robot.dof_names)
  ```

### 3. ROS2 토픽이 보이지 않음

**해결**:
```bash
# 토픽 목록 확인
ros2 topic list

# 특정 토픽 확인
ros2 topic echo /joint_states
ros2 topic echo /quadmani/joint_commands
```

### 4. Isaac Sim과 ROS2 통신 안 됨

**해결**:
- Isaac Sim ROS2 Bridge Extension이 활성화되었는지 확인
- ROS2 도메인 ID 확인:
  ```bash
  echo $ROS_DOMAIN_ID  # 기본값: 0
  ```
- Isaac Sim과 같은 ROS_DOMAIN_ID 사용

## 통합 예제

### 전체 시나리오 테스트

1. **터미널 1**: Isaac Sim 시작 (로봇 로드)
2. **터미널 2**: 조인트 상태 구독
   ```bash
   ros2 run isaac_sim_bridge joint_state_subscriber_demo
   ```
3. **터미널 3**: 조인트 명령 퍼블리시
   ```bash
   ros2 run isaac_sim_bridge joint_command_publisher_demo
   ```
4. **터미널 4**: QuadMani Controller (RQT) 실행 (선택)
   ```bash
   rqt --force-discover --standalone quadmani_controller
   ```

## 참고

- Isaac Sim ROS2 Bridge 공식 문서: https://docs.omniverse.nvidia.com/app_isaacsim/app_isaacsim/ext_omni_isaac_ros_bridge.html
- ROS2 토픽 모니터링: `ros2 topic hz /joint_states`
- 조인트 명령 테스트: `ros2 topic pub` 사용

## 라이센스

MIT License

