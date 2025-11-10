#!/bin/bash
# QuadMani Controller RQT 플러그인 실행 스크립트

set -e

WORKSPACE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

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
    echo "  colcon build --packages-select quadmani_controller --symlink-install"
    exit 1
fi

# ROS2 도메인 ID 설정 (기본값: 0)
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}

echo "=========================================="
echo "QuadMani Controller RQT 플러그인 실행"
echo "=========================================="
echo "워크스페이스: $WORKSPACE_DIR"
echo "ROS2 도메인 ID: $ROS_DOMAIN_ID"
echo ""
echo "RQT 플러그인을 시작합니다..."
echo ""

# RQT 플러그인 실행
rqt --force-discover --standalone quadmani_controller

