#!/bin/bash

# IRM 테스트 프로그램 빌드 및 실행 스크립트

echo "========================================="
echo "IRM 테스트 프로그램 빌드"
echo "========================================="

# 워크스페이스 디렉토리로 이동
cd /home/taegyeom/kiro_ws

# 빌드
echo ""
echo "1. 패키지 빌드 중..."
colcon build --packages-select qmm_control --symlink-install

if [ $? -ne 0 ]; then
    echo "빌드 실패!"
    exit 1
fi

echo ""
echo "빌드 완료!"
echo ""
echo "========================================="
echo "테스트 프로그램 실행"
echo "========================================="

# 환경 설정
source install/setup.bash

# 실행
echo ""
ros2 run qmm_control test_irm

echo ""
echo "========================================="
echo "완료"
echo "========================================="



