# 로봇이 움직이지 않는 문제 진단

## ✅ 현재 상태

### 1. 명령 전송 확인
- ✅ **토픽 발행 중**: `/quadmani/joint_commands` (10Hz)
- ✅ **값 정확**: GO1 Stand 자세 `[0.0, 0.67, -1.3]` rad
- ✅ **메시지 형식**: `sensor_msgs/JointState` 정상

### 2. 문제점
- ⚠️ **구독자 없음**: `Subscription count: 0`
- ⚠️ **로봇 미동작**: Isaac Sim에서 로봇이 움직이지 않음

## 🔍 확인 사항

### Isaac Sim 설정 확인

1. **Isaac Sim이 Play 모드인지 확인**
   - Isaac Sim 창에서 ▶️ (Play) 버튼이 눌러져 있어야 함
   - 시뮬레이션이 실행 중이어야 함

2. **ActionGraph 설정 확인**
   - Isaac Sim의 Scene Explorer에서 `ActionGraph` 확인
   - `ros2_subscribe_joint_state` 노드가 존재하는지 확인
   - **구독 토픽 이름**: `/quadmani/joint_commands`인지 확인
     - 만약 다른 토픽(예: `/joint_command`)을 구독한다면 변경 필요

3. **노드 연결 확인**
   - `ros2_subscribe_joint_state` → `articulation_controller` 연결
   - `articulation_controller` → `go1` 로봇 연결

## 🛠️ 해결 방법

### 방법 1: Isaac Sim ActionGraph에서 토픽 이름 확인/변경

1. Isaac Sim에서 ActionGraph 열기
2. `ros2_subscribe_joint_state` 노드 선택
3. Properties에서 구독 토픽 확인
4. `/quadmani/joint_commands`로 설정되어 있는지 확인
5. 다르다면 `/quadmani/joint_commands`로 변경

### 방법 2: 토픽 이름 변경 (다른 토픽을 구독하는 경우)

만약 Isaac Sim이 다른 토픽을 구독한다면:
- 예: `/joint_command` 또는 `/go1/joint_commands`

코드를 수정하여 해당 토픽으로 발행:
```python
# home_pose_publisher.py에서
self.publisher_ = self.create_publisher(
    JointState, 
    '/joint_command',  # 또는 Isaac Sim이 구독하는 토픽 이름
    10)
```

### 방법 3: 토픽 확인 명령어

```bash
source ~/kiro_ws/install/setup.bash

# 구독자 확인
ros2 topic info /quadmani/joint_commands

# 메시지 내용 확인
ros2 topic echo /quadmani/joint_commands

# 발행 주기 확인
ros2 topic hz /quadmani/joint_commands
```

## 📊 현재 전송 중인 값

**GO1 Stand 자세** (모든 다리 동일):
- FR: `[0.000, 0.670, -1.300]` rad (hip, thigh, calf)
- FL: `[0.000, 0.670, -1.300]` rad
- RR: `[0.000, 0.670, -1.300]` rad
- RL: `[0.000, 0.670, -1.300]` rad

**K1 Home 자세**:
- 모든 조인트: `0.0` rad

## 💡 다음 단계

1. Isaac Sim에서 ActionGraph의 토픽 이름 확인
2. 구독자 수가 1 이상이 되면 로봇이 움직여야 함
3. 필요시 토픽 이름을 맞춤

