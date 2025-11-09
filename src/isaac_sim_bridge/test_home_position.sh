#!/bin/bash
# Home position 제어 테스트 스크립트

source ~/kiro_ws/install/setup.bash

echo "=========================================="
echo "Home Position 제어 테스트"
echo "=========================================="
echo ""

echo "1. Home position controller 실행 중..."
timeout 6 python3 ~/kiro_ws/src/isaac_sim_bridge/isaac_sim_bridge/home_position_controller.py &
CONTROLLER_PID=$!
sleep 2

echo ""
echo "2. Joint command 확인:"
echo "----------------------------------------"
timeout 2 ros2 topic echo /joint_command --once 2>&1 | grep -A 25 "name:" | head -30

echo ""
echo "3. Joint states 확인:"
echo "----------------------------------------"
timeout 2 ros2 topic echo /joint_states --once 2>&1 | grep -A 25 "name:" | head -30

echo ""
echo "4. Home position 확인:"
echo "----------------------------------------"
echo "GO1 Home Position:"
echo "  - hip: 0.0 rad (0.0°)"
echo "  - thigh: 0.67 rad (38.4°)"
echo "  - calf: -1.3 rad (-74.5°)"
echo ""
echo "K1 Home Position:"
echo "  - 모든 조인트: 0.0 rad"

wait $CONTROLLER_PID 2>/dev/null

echo ""
echo "=========================================="
echo "테스트 완료"
echo "=========================================="


