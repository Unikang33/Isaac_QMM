#!/bin/bash
# Home Position Controller 실행 스크립트

# ROS 2 환경 소싱
source /opt/ros/humble/setup.bash
source /home/taegyeom/kiro_ws/install/setup.bash

# 노드 실행
echo "=========================================="
echo "  Home Position Controller 실행 중..."
echo "=========================================="
ros2 run qmm_control home_position_controller.py

