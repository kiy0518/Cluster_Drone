#!/bin/bash
# ROS2 환경 로드 스크립트
# Cluster_Drone 프로젝트용

# 스크립트 위치 확인
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

echo "=========================================="
echo "ROS2 환경 로드"
echo "=========================================="
echo ""

# .bashrc 로드
echo "1. .bashrc 로드 중..."
source ~/.bashrc 2>/dev/null || {
    echo "  ⚠ .bashrc 로드 실패 (계속 진행)"
}

# Micro-ROS Agent 작업 공간 로드
echo "2. Micro-ROS Agent 작업 공간 로드 중..."
if [ -f ~/Cluster_Drone/micro_ros_ws/install/setup.bash ]; then
    source ~/Cluster_Drone/micro_ros_ws/install/setup.bash
    echo "  ✓ Micro-ROS Agent 작업 공간 로드됨"
else
    echo "  ⚠ Micro-ROS Agent 작업 공간 없음"
fi

# PX4 ROS2 메시지 패키지 로드
echo "3. PX4 ROS2 메시지 패키지 로드 중..."
if [ -f ~/Cluster_Drone/px4_ros2_ws/install/setup.bash ]; then
    source ~/Cluster_Drone/px4_ros2_ws/install/setup.bash
    echo "  ✓ PX4 ROS2 메시지 패키지 로드됨"
else
    echo "  ⚠ PX4 ROS2 메시지 패키지 없음"
fi

echo ""
echo "=========================================="
echo "환경 확인"
echo "=========================================="
echo "  ROS_DOMAIN_ID: ${ROS_DOMAIN_ID:-0}"
echo "  RMW_IMPLEMENTATION: ${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
if command -v ros2 > /dev/null 2>&1; then
    echo "  ROS2: $(which ros2)"
    echo "  ✓ ROS2 환경 준비 완료"
else
    echo "  ROS2: 없음"
    echo "  ✗ ROS2가 설치되지 않았습니다"
fi
echo ""

