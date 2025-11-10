# QMM Control Package

QuadMani Motion Control 패키지 - FK/IK 기반 제어 노드들

## 📦 패키지 정보

- **패키지 이름**: `qmm_control`
- **빌드 타입**: ament_cmake
- **의존성**: rclcpp, rclpy, sensor_msgs, geometry_msgs, tf2_ros

## 🎯 제공 노드

### 1. `home_position_controller.py`
GO1과 K1을 home 자세로 position 제어하는 노드

**기능**:
- GO1을 웅크리기 자세에서 서기 자세로 부드럽게 이동
- K1 매니퓰레이터를 home position(모든 조인트 0.0)으로 제어
- 3초 이동 + 2초 안정화 = 총 5초 실행 후 자동 종료

**발행 토픽**:
- `/joint_command` (sensor_msgs/JointState)
- `/quadmani/joint_commands` (sensor_msgs/JointState)

**실행 방법**:
```bash
# 환경 소싱
source /opt/ros/humble/setup.bash
source ~/kiro_ws/install/setup.bash

# 노드 실행
ros2 run qmm_control home_position_controller.py
```

### 2. `fk_ik_controller.py`
FK-IK 기반 Joint Command 컨트롤러

**기능**:
- 현재 joint state 수신
- TF에서 base pose와 foot position 수신
- Analytical IK로 joint 값 계산
- 계산된 joint 값을 joint_command 토픽으로 발행
- 시간에 따른 base pose offset 변화 (높이, roll, pitch 제어)

**구독 토픽**:
- `joint_states` (sensor_msgs/JointState)
- TF: world → base, world → hip, world → foot

**발행 토픽**:
- `joint_command` (sensor_msgs/JointState)

**실행 방법**:
```bash
# 환경 소싱
source /opt/ros/humble/setup.bash
source ~/kiro_ws/install/setup.bash

# 노드 실행
ros2 run qmm_control fk_ik_controller.py
```

## 🔧 빌드 방법

```bash
cd ~/kiro_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select qmm_control
source install/setup.bash
```

## 📁 패키지 구조

```
qmm_control/
├── CMakeLists.txt          # CMake 빌드 설정
├── package.xml             # 패키지 메타데이터
├── README.md               # 이 파일
├── scripts/                # Python 스크립트
│   ├── home_position_controller.py
│   └── fk_ik_controller.py
├── src/                    # C++ 소스 (현재 비어있음)
└── include/                # C++ 헤더 (현재 비어있음)
```

## 🚀 사용 예시

### 1. Home Position으로 이동
```bash
# 터미널 1: Isaac Sim 실행 (사전에 실행되어 있어야 함)
# 터미널 2: Home position 제어
source /opt/ros/humble/setup.bash
source ~/kiro_ws/install/setup.bash
ros2 run qmm_control home_position_controller.py
```

### 2. FK-IK 기반 자세 제어
```bash
# 터미널 1: Isaac Sim 실행 (사전에 실행되어 있어야 함)
# 터미널 2: FK-IK 제어
source /opt/ros/humble/setup.bash
source ~/kiro_ws/install/setup.bash
ros2 run qmm_control fk_ik_controller.py
```

## ⚠️ 주의사항

1. **Isaac Sim 실행**: 노드 실행 전에 Isaac Sim이 실행되어 있어야 합니다
2. **TF 발행**: `fk_ik_controller.py`는 TF 데이터가 필요합니다
3. **Joint State**: 두 노드 모두 `/joint_states` 토픽을 구독하거나 발행합니다
4. **Python 버전**: ROS 2 Humble의 Python 버전과 호환되어야 합니다

## 📝 개발 정보

- **원본 패키지**: isaac_sim_bridge
- **이전 위치**: `src/Isaac_QMM/src/isaac_sim_bridge/`
- **현재 위치**: `src/qmm_control/`

## 🔗 관련 패키지

- `isaac_sim_bridge`: Isaac Sim과의 통신
- `sensor_msgs`: ROS 2 센서 메시지
- `geometry_msgs`: ROS 2 지오메트리 메시지
- `tf2_ros`: TF 변환

## 📄 라이센스

MIT License

