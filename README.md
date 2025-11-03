# kiro_ws ROS2 워크스페이스

QuadMani 로봇 (Unitree GO1 + K1 Arm) 제어를 위한 ROS2 워크스페이스입니다.

## 구조

```
kiro_ws/
├── src/
│   ├── quadmani_controller/      # RQT 플러그인 (심볼릭 링크)
│   └── isaac_sim_bridge/         # Isaac Sim ROS2 브리지 (심볼릭 링크)
├── build/                        # 빌드 파일
├── install/                      # 설치 파일
└── log/                          # 빌드 로그
```

## 빠른 시작

### 1. 워크스페이스 설정 (최초 1회)

프로젝트 루트에서 실행:
```bash
cd /home/oem/unitree_quadmani
chmod +x setup_kiro_ws.sh
./setup_kiro_ws.sh
```

또는 수동으로:
```bash
mkdir -p ~/kiro_ws/src
cd ~/kiro_ws/src
ln -sf /home/oem/unitree_quadmani/quadmani_controller .
ln -sf /home/oem/unitree_quadmani/isaac_sim_bridge .
```

### 2. 패키지 빌드

```bash
cd ~/kiro_ws
# 자동 빌드 스크립트 사용
chmod +x /home/oem/unitree_quadmani/build_kiro_ws.sh
/home/oem/unitree_quadmani/build_kiro_ws.sh

# 또는 수동으로
source /opt/ros/humble/setup.bash  # 또는 iron, foxy
colcon build --symlink-install
source install/setup.bash
```

### 3. 데모 실행

```bash
cd /home/oem/unitree_quadmani

# 조인트 상태 구독 데모
chmod +x run_demo.sh
./run_demo.sh subscriber

# 또는 조인트 명령 퍼블리시 데모
./run_demo.sh publisher
```

## 패키지 목록

### 1. quadmani_controller
RQT 플러그인으로 20개 조인트를 GUI로 제어할 수 있습니다.

**실행:**
```bash
source ~/kiro_ws/install/setup.bash
rqt --force-discover --standalone quadmani_controller
```

**토픽:**
- 퍼블리시: `/quadmani/joint_commands`
- 구독: `/joint_states`

### 2. isaac_sim_bridge
Isaac Sim과 ROS2를 연동하는 브리지 패키지입니다.

**데모 노드:**
- `joint_state_subscriber_demo` - 조인트 상태 구독
- `joint_command_publisher_demo` - 조인트 명령 퍼블리시

**실행:**
```bash
source ~/kiro_ws/install/setup.bash

# 조인트 상태 구독
ros2 run isaac_sim_bridge joint_state_subscriber_demo

# 조인트 명령 퍼블리시
ros2 run isaac_sim_bridge joint_command_publisher_demo
```

## 전체 시나리오 테스트

### 준비 사항
1. Isaac Sim 실행 (로봇 로드 완료)
2. Isaac Sim ROS2 Bridge Extension 활성화
3. ROS2 도메인 ID 설정 (모든 터미널에서 동일하게)

### 실행 단계

**터미널 1**: Isaac Sim 실행
```bash
export ROS_DOMAIN_ID=0
isaac-sim  # 또는 Isaac Sim 실행
```

**터미널 2**: 조인트 상태 구독
```bash
export ROS_DOMAIN_ID=0
cd /home/oem/unitree_quadmani
./run_demo.sh subscriber
```

**터미널 3**: 조인트 명령 퍼블리시
```bash
export ROS_DOMAIN_ID=0
cd /home/oem/unitree_quadmani
./run_demo.sh publisher
```

**터미널 4**: RQT Controller (선택)
```bash
export ROS_DOMAIN_ID=0
source ~/kiro_ws/install/setup.bash
rqt --force-discover --standalone quadmani_controller
```

## 환경 설정

### ROS2 도메인 ID 설정

모든 터미널에서 동일한 도메인 ID를 사용해야 합니다:
```bash
export ROS_DOMAIN_ID=0
```

또는 `~/.bashrc`에 추가:
```bash
echo "export ROS_DOMAIN_ID=0" >> ~/.bashrc
```

### 자동 환경 소싱

`~/.bashrc`에 다음 추가:
```bash
# ROS2 환경
source /opt/ros/humble/setup.bash  # 또는 iron, foxy

# kiro_ws 워크스페이스
if [ -f ~/kiro_ws/install/setup.bash ]; then
    source ~/kiro_ws/install/setup.bash
fi
```

## 문제 해결

### 빌드 오류
```bash
# 전체 재빌드
cd ~/kiro_ws
rm -rf build install log
colcon build --symlink-install
source install/setup.bash
```

### 패키지를 찾을 수 없음
```bash
# 환경 확인
source ~/kiro_ws/install/setup.bash
ros2 pkg list | grep -E "(quadmani|isaac)"
```

### 토픽이 보이지 않음
```bash
# 도메인 ID 확인
echo $ROS_DOMAIN_ID

# 토픽 목록 확인
ros2 topic list

# 토픽 내용 확인
ros2 topic echo /joint_states
ros2 topic echo /quadmani/joint_commands
```

## 참고

- 프로젝트 루트: `/home/oem/unitree_quadmani`
- 워크스페이스: `~/kiro_ws`
- 상세 문서:
  - QuadMani Controller: `quadmani_controller/README.md`
  - Isaac Sim Bridge: `isaac_sim_bridge/README.md`
  - 설정 가이드: `isaac_sim_bridge/SETUP_GUIDE.md`

