# Isaac Sim ROS2 브리지 설정 가이드

이 가이드는 Isaac Sim에서 QuadMani 로봇을 ROS2로 제어하기 위한 환경 구성 방법을 설명합니다.

## 1. 사전 준비

### Isaac Sim 설치 확인
```bash
# Isaac Sim이 설치되어 있는지 확인
which isaac-sim
# 또는
echo $ISAAC_SIM_PATH
```

### ROS2 설치 확인
```bash
# ROS2 환경 확인
printenv | grep ROS
source /opt/ros/humble/setup.bash  # 또는 iron, foxy 등
```

## 2. Isaac Sim ROS2 Bridge Extension 활성화

### 방법 A: Extension Manager 사용 (권장)

1. Isaac Sim 실행
2. `Window` > `Extensions` 메뉴 선택
3. 검색창에 "ROS2" 입력
4. `omni.isaac.ros2_bridge` 확장 찾아서 `Enable` 클릭
5. Isaac Sim 재시작

### 방법 B: 명령줄에서 활성화

```bash
# Isaac Sim Extension 활성화 (경로는 실제 설치 경로로 변경)
isaac-sim --enable-extension omni.isaac.ros2_bridge
```

## 3. 패키지 빌드

```bash
# ROS2 워크스페이스로 이동 (예: ~/kiro_ws)
cd ~/kiro_ws

# 패키지 추가 (이미 추가된 경우 생략)
# colcon build --packages-select isaac_sim_bridge --symlink-install

# 빌드
colcon build --packages-select isaac_sim_bridge --symlink-install

# 환경 소싱
source install/setup.bash
```

## 4. Isaac Sim에서 로봇 로드

### USD 파일 로드

1. Isaac Sim 실행
2. `File` > `Open` 메뉴
3. `/home/oem/unitree_quadmani/unitree_quadmani.usd` 파일 선택
4. 로봇이 씬에 로드되었는지 확인

### 로봇 프림 경로 확인

Isaac Sim의 Stage Tree에서 로봇의 프림 경로를 확인하세요.
예: `/World/quadmani` 또는 `/quadmani`

## 5. ROS2 도메인 설정

Isaac Sim과 ROS2 노드가 같은 도메인을 사용해야 합니다.

```bash
# 터미널 1: Isaac Sim
export ROS_DOMAIN_ID=0
isaac-sim

# 터미널 2: ROS2 노드
export ROS_DOMAIN_ID=0
source ~/kiro_ws/install/setup.bash
```

## 6. 테스트 실행

### 단계별 테스트

#### 1단계: 조인트 상태 구독 테스트

**터미널 1**: 조인트 상태 구독 데모 실행
```bash
export ROS_DOMAIN_ID=0
source ~/kiro_ws/install/setup.bash
ros2 run isaac_sim_bridge joint_state_subscriber_demo
```

**터미널 2**: Isaac Sim에서 조인트 상태 퍼블리시 확인
```bash
ros2 topic list | grep joint
ros2 topic echo /joint_states
```

#### 2단계: 조인트 명령 퍼블리시 테스트

**터미널 3**: 조인트 명령 퍼블리시 데모 실행
```bash
export ROS_DOMAIN_ID=0
source ~/kiro_ws/install/setup.bash
ros2 run isaac_sim_bridge joint_command_publisher_demo
```

**터미널 4**: 명령 확인
```bash
ros2 topic echo /quadmani/joint_commands
```

## 7. Isaac Sim Extension Script 사용 (고급)

Isaac Sim의 Script Editor에서 직접 브리지를 구성할 수 있습니다.

### Script Editor 열기

1. Isaac Sim에서 `Window` > `Script Editor` 선택
2. 새 스크립트 생성 또는 기존 스크립트 열기

### 예제 스크립트

```python
from omni.isaac.core import World
from omni.isaac.core.robots import Robot

# World 초기화
world = World()

# 로봇 로드
robot = world.scene.add(
    Robot(
        prim_path="/World/quadmani",  # 실제 프림 경로로 변경
        name="quadmani",
        usd_path="/home/oem/unitree_quadmani/unitree_quadmani.usd"
    )
)

# 물리 시뮬레이션 시작
world.reset()

# 조인트 이름 확인
print("조인트 목록:")
print(robot.dof_names)
```

## 8. 실제 사용 시나리오

### 전체 시나리오: RQT Controller + Isaac Sim

1. **터미널 1**: Isaac Sim 실행 (로봇 로드 완료)
   ```bash
   export ROS_DOMAIN_ID=0
   isaac-sim
   ```

2. **터미널 2**: RQT QuadMani Controller 실행
   ```bash
   export ROS_DOMAIN_ID=0
   source ~/kiro_ws/install/setup.bash
   rqt --force-discover --standalone quadmani_controller
   ```

3. **터미널 3**: 조인트 상태 모니터링 (선택)
   ```bash
   export ROS_DOMAIN_ID=0
   source ~/kiro_ws/install/setup.bash
   ros2 topic echo /joint_states
   ```

4. **RQT Controller에서**:
   - 슬라이더로 조인트 조정
   - "현재 위치 전송" 버튼 클릭
   - Isaac Sim에서 로봇이 움직이는지 확인

## 문제 해결

### 문제 1: ROS2 토픽이 보이지 않음

**해결책**:
```bash
# 도메인 ID 확인
echo $ROS_DOMAIN_ID

# 토픽 목록 확인
ros2 topic list

# Isaac Sim ROS2 Bridge가 활성화되었는지 확인
# Isaac Sim > Extensions > omni.isaac.ros2_bridge
```

### 문제 2: 조인트를 찾을 수 없음

**해결책**:
1. Isaac Sim에서 로봇이 올바르게 로드되었는지 확인
2. Stage Tree에서 로봇 프림 경로 확인
3. Script Editor에서 조인트 이름 확인:
   ```python
   print(robot.dof_names)
   ```

### 문제 3: Isaac Sim 모듈 Import 오류

**해결책**:
- 이 노드들은 Isaac Sim 환경 내에서 실행되어야 합니다
- 일반 ROS2 환경에서는 데모 노드만 사용 가능합니다
- `joint_state_subscriber_demo`, `joint_command_publisher_demo`는 일반 ROS2 환경에서 실행 가능

### 문제 4: 조인트 명령이 적용되지 않음

**해결책**:
1. Isaac Sim에서 로봇이 물리 시뮬레이션 모드인지 확인
2. Isaac Sim Extension Script에서 로봇 객체 참조 확인
3. 토픽 메시지 확인:
   ```bash
   ros2 topic echo /quadmani/joint_commands
   ```

## 다음 단계

- [ ] Isaac Sim에서 로봇 로드 완료 확인
- [ ] ROS2 Bridge Extension 활성화 확인
- [ ] 조인트 상태 구독 테스트 성공
- [ ] 조인트 명령 퍼블리시 테스트 성공
- [ ] RQT Controller와 연동 테스트

## 참고 자료

- [Isaac Sim ROS2 Bridge 문서](https://docs.omniverse.nvidia.com/app_isaacsim/app_isaacsim/ext_omni_isaac_ros_bridge.html)
- [ROS2 토픽 튜토리얼](https://docs.ros.org/en/humble/Tutorials/Topics.html)
- QuadMani Controller README: `quadmani_controller/README.md`

