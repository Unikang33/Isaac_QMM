#!/bin/bash
# Isaac Sim 브리지 데모 실행 스크립트

set -e

WORKSPACE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DEMO_TYPE="${1:-subscriber}"  # subscriber 또는 publisher

# ROS2 환경 확인 및 소싱
if [ -z "$ROS_DISTRO" ]; then
    if [ -f "/opt/ros/humble/setup.bash" ]; then
        source /opt/ros/humble/setup.bash
    elif [ -f "/opt/ros/iron/setup.bash" ]; then
        source /opt/ros/iron/setup.bash
    else
        echo "오류: ROS2를 찾을 수 없습니다."
        echo "다음 중 하나를 실행하세요:"
        echo "  source /opt/ros/humble/setup.bash"
        echo "  source /opt/ros/iron/setup.bash"
        exit 1
    fi
fi

# 워크스페이스 환경 소싱
if [ -f "$WORKSPACE_DIR/install/setup.bash" ]; then
    source "$WORKSPACE_DIR/install/setup.bash"
else
    echo "오류: 워크스페이스를 먼저 빌드하세요:"
    echo "  cd $WORKSPACE_DIR"
    echo "  source /opt/ros/humble/setup.bash  # 또는 iron"
    echo "  colcon build --symlink-install"
    exit 1
fi

# ROS2 도메인 ID 설정 (기본값: 0)
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}

echo "=========================================="
echo "Isaac Sim 브리지 데모 실행"
echo "=========================================="
echo "워크스페이스: $WORKSPACE_DIR"
echo "ROS2 도메인 ID: $ROS_DOMAIN_ID"
echo "데모 타입: $DEMO_TYPE"
echo ""

case "$DEMO_TYPE" in
    subscriber|sub)
        echo "조인트 상태 구독 데모 실행 중..."
        echo "(Ctrl+C로 종료)"
        echo ""
        # install/bin의 실행 파일 직접 실행
        if [ -f "$WORKSPACE_DIR/install/isaac_sim_bridge/bin/joint_state_subscriber_demo" ]; then
            exec "$WORKSPACE_DIR/install/isaac_sim_bridge/bin/joint_state_subscriber_demo"
        else
            echo "오류: 실행 파일을 찾을 수 없습니다."
            echo "먼저 빌드하세요: colcon build --packages-select isaac_sim_bridge"
            exit 1
        fi
        ;;
    publisher|pub)
        echo "조인트 명령 퍼블리시 데모 실행 중..."
        echo "(Ctrl+C로 종료)"
        echo ""
        # install/bin의 실행 파일 직접 실행
        if [ -f "$WORKSPACE_DIR/install/isaac_sim_bridge/bin/joint_command_publisher_demo" ]; then
            exec "$WORKSPACE_DIR/install/isaac_sim_bridge/bin/joint_command_publisher_demo"
        else
            echo "오류: 실행 파일을 찾을 수 없습니다."
            echo "먼저 빌드하세요: colcon build --packages-select isaac_sim_bridge"
            exit 1
        fi
        ;;
    *)
        echo "사용법: $0 [subscriber|publisher]"
        echo ""
        echo "옵션:"
        echo "  subscriber, sub  - 조인트 상태 구독 데모 (기본값)"
        echo "  publisher, pub  - 조인트 명령 퍼블리시 데모"
        exit 1
        ;;
esac

