# QuadMani Controller - RQT Plugin

Unitree GO1 (4족 보행 로봇)과 K1 Arm (6축 로봇팔)을 통합 제어하는 RQT 플러그인입니다.

## 기능

- **20개 조인트 제어**: GO1의 12개 다리 조인트 + K1 Arm의 6개 팔 조인트 + 2개 그리퍼 조인트
- **실시간 슬라이더 제어**: 각 조인트를 슬라이더와 스핀박스로 제어
- **프리셋 포지션**: 홈 포지션, 제로 포지션 등
- **ROS2 Topic 퍼블리시**: `/quadmani/joint_commands` 토픽으로 명령 전송

## 제어 가능한 조인트

### GO1 다리 (12개)
- Front Right: `FR_hip_joint`, `FR_thigh_joint`, `FR_calf_joint`
- Front Left: `FL_hip_joint`, `FL_thigh_joint`, `FL_calf_joint`
- Rear Right: `RR_hip_joint`, `RR_thigh_joint`, `RR_calf_joint`
- Rear Left: `RL_hip_joint`, `RL_thigh_joint`, `RL_calf_joint`

### K1 Arm (8개)
- Arm: `joint1`, `joint2`, `joint3`, `joint4`, `joint5`, `joint6`
- Gripper: `joint_gripper_left`, `joint_gripper_right`

## 설치 및 빌드

```bash
cd ~/kiro_ws
colcon build --packages-select quadmani_controller --symlink-install
source install/setup.bash
```

## 실행 방법

### ✅ 방법 1: 쉬운 실행 (추천!)
```bash
cd ~/kiro_ws
./run_quadmani_controller.sh
```

### 방법 2: 직접 실행
```bash
source ~/kiro_ws/install/setup.bash
rqt --force-discover --standalone quadmani_controller
```

### 방법 3: RQT 메뉴에서 실행
```bash
source ~/kiro_ws/install/setup.bash
rqt --force-discover
```
그 다음 RQT 창에서: `Plugins` > `Robot Tools` > `QuadMani Controller`

**⚠️ 중요**: 처음 실행 시 반드시 `--force-discover` 옵션을 사용해야 합니다!

## 사용 방법

1. **조인트 제어**
   - 슬라이더를 움직이거나 스핀박스에 값을 입력하여 각 조인트 위치 설정
   - 각 조인트는 URDF에 정의된 한계값 내에서만 움직입니다

2. **프리셋 버튼**
   - `모든 조인트 0으로`: 가능한 조인트를 0 위치로 설정
   - `홈 포지션`: GO1이 서있는 자세, 팔은 수직 자세로 이동
   - `현재 위치 전송`: 현재 설정된 조인트 값을 ROS2 토픽으로 전송

3. **ROS2 토픽**
   - **퍼블리시**: `/quadmani/joint_commands` (sensor_msgs/JointState)
   - **구독**: `/joint_states` (sensor_msgs/JointState) - 현재 로봇 상태 표시

## 실제 로봇과 연동

이 플러그인은 조인트 명령을 `/quadmani/joint_commands` 토픽으로 퍼블리시합니다.
실제 로봇 제어를 위해서는 이 토픽을 구독하여 로봇 하드웨어로 전달하는 노드가 필요합니다.

### 예제: joint_command_relay 노드

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

class JointCommandRelay(Node):
    def __init__(self):
        super().__init__('joint_command_relay')
        self.subscription = self.create_subscription(
            JointState,
            '/quadmani/joint_commands',
            self.command_callback,
            10)
    
    def command_callback(self, msg):
        # 여기에 실제 로봇 제어 코드 작성
        self.get_logger().info(f'Received joint command: {len(msg.name)} joints')
        # 예: GO1 SDK로 다리 제어, K1 SDK로 팔 제어

if __name__ == '__main__':
    rclpy.init()
    node = JointCommandRelay()
    rclpy.spin(node)
```

## 문제 해결

### 1️⃣ RQT에서 플러그인이 보이지 않을 때

**해결 방법 A**: 캐시 삭제 후 재실행
```bash
rm -rf ~/.config/ros.org/rqt_gui.ini
source ~/kiro_ws/install/setup.bash
rqt --force-discover
```

**해결 방법 B**: 쉬운 실행 스크립트 사용
```bash
cd ~/kiro_ws
./run_quadmani_controller.sh
```

### 2️⃣ Import 에러가 발생할 때
```bash
# 워크스페이스 다시 빌드
cd ~/kiro_ws
colcon build --packages-select quadmani_controller --symlink-install
source install/setup.bash
```

### 3️⃣ 플러그인이 등록되었는지 확인
```bash
source ~/kiro_ws/install/setup.bash
rqt --list-plugins | grep quadmani
```
출력 예상: `quadmani_controller.quadmani_controller.QuadManiController`

### 4️⃣ PyQt5가 없다는 에러가 나올 때
```bash
pip3 install PyQt5
```

## 라이센스

MIT License

## 저자

robotics22

