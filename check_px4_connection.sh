#!/bin/bash
# PX4 연결 상태 종합 확인 스크립트
# Cluster_Drone 프로젝트용

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

echo "=========================================="
echo "PX4 연결 상태 종합 확인"
echo "=========================================="
echo ""

# 1. 네트워크 상태
echo "1. 네트워크 상태:"
echo "   eth0 IP:"
ip addr show eth0 | grep "inet " || echo "     (IP 없음)"
echo "   eth0 상태:"
ip link show eth0 | grep -E "state|UP" || echo "     (상태 확인 불가)"

# 2. DHCP 서버 상태
echo ""
echo "2. DHCP 서버 (dnsmasq) 상태:"
if systemctl is-active --quiet dnsmasq-px4; then
    echo "   ✓ dnsmasq 실행 중"
    echo "   최근 DHCP 활동:"
    sudo journalctl -u dnsmasq-px4 -n 20 --no-pager 2>&1 | grep -i "dhcp\|lease\|request" | tail -5 || echo "     (DHCP 활동 없음)"
else
    echo "   ✗ dnsmasq 실행 안됨"
fi

# 3. 연결된 장치 확인
echo ""
echo "3. eth0에 연결된 장치:"
NEIGH=$(ip neigh show dev eth0 2>/dev/null)
if [ -n "$NEIGH" ]; then
    echo "$NEIGH" | while read line; do
        echo "   $line"
    done
else
    echo "   (ARP 테이블에 장치 없음)"
    echo "   → PX4가 연결되지 않았거나 아직 통신하지 않았습니다"
fi

# 4. Micro-ROS Agent 상태
echo ""
echo "4. Micro-ROS Agent 상태:"
# systemd 서비스 상태 확인
SERVICE_STATUS=$(systemctl is-active micro-ros-agent 2>&1)
if [ "$SERVICE_STATUS" = "active" ]; then
    echo "   ✓ systemd 서비스 실행 중"
elif [ "$SERVICE_STATUS" = "activating" ] || [ "$SERVICE_STATUS" = "auto-restart" ]; then
    echo "   ⚠ systemd 서비스 재시작 중 (문제 가능성)"
    echo "   → 로그 확인: sudo journalctl -u micro-ros-agent -n 30"
else
    echo "   ✗ systemd 서비스 실행 안됨 (상태: $SERVICE_STATUS)"
fi

# 포트 8888이 리스닝 중인지 확인
AGENT_PID=$(netstat -ulnp 2>/dev/null | grep ":8888" | awk '{print $NF}' | cut -d'/' -f1 | head -1)
if [ -z "$AGENT_PID" ]; then
    AGENT_PID=$(ss -ulnp 2>/dev/null | grep ":8888" | grep -oP 'pid=\K[0-9]+' | head -1)
fi
if [ -n "$AGENT_PID" ]; then
    echo "   ✓ Agent 실행 중 (PID: $AGENT_PID)"
    echo "   ✓ 포트 8888 리스닝 중"
else
    # 프로세스 이름으로도 확인
    AGENT_PID=$(pgrep -f "micro_ros_agent" | head -1)
    if [ -n "$AGENT_PID" ]; then
        echo "   ⚠ Agent 프로세스 발견 (PID: $AGENT_PID) - 포트 미확인"
        echo "   포트 확인:"
        netstat -ulnp 2>/dev/null | grep 8888 || ss -ulnp 2>/dev/null | grep 8888 || echo "     ✗ 포트 8888이 열려있지 않음"
        echo "   → Agent가 제대로 시작되지 않았을 수 있습니다"
        echo "   → 로그 확인: sudo journalctl -u micro-ros-agent -n 50"
    else
        echo "   ✗ Agent 실행 안됨"
        echo "   → 서비스 시작: sudo systemctl start micro-ros-agent"
        echo "   → 로그 확인: sudo journalctl -u micro-ros-agent -n 50"
    fi
fi

# 5. ROS2 토픽 확인
echo ""
echo "5. ROS2 토픽 확인:"
source ~/.bashrc 2>/dev/null || true
if [ -f ~/Cluster_Drone/px4_ros2_ws/install/setup.bash ]; then
    source ~/Cluster_Drone/px4_ros2_ws/install/setup.bash
fi

if command -v ros2 > /dev/null 2>&1; then
    TOPIC_LIST=$(ros2 topic list 2>/dev/null)
    TOPICS=$(echo "$TOPIC_LIST" | grep -c "fmu" 2>/dev/null || echo "0")
    TOPICS=$(echo "$TOPICS" | tr -d '\n' | head -c 1)
    if [ -z "$TOPICS" ] || [ "$TOPICS" = "" ]; then
        TOPICS=0
    fi
    echo "   PX4 토픽 개수: $TOPICS"
    if [ "$TOPICS" -gt 0 ] 2>/dev/null; then
        echo "   ✓ PX4 연결됨!"
        echo "   주요 토픽:"
        echo "$TOPIC_LIST" | grep "fmu" | head -5 | sed 's/^/     /'
    else
        echo "   ✗ PX4 토픽 없음"
    fi
else
    echo "   ✗ ROS2 명령어 없음"
    TOPICS=0
fi

# 6. 종합 진단
echo ""
echo "=========================================="
echo "종합 진단"
echo "=========================================="

ISSUES=0

# 네트워크 확인
if ! ip addr show eth0 | grep -q "10.0.0.1"; then
    echo "✗ eth0에 IP가 할당되지 않음"
    echo "  해결: ./setup_complete.sh"
    ISSUES=$((ISSUES + 1))
fi

# DHCP 확인
if ! systemctl is-active --quiet dnsmasq-px4; then
    echo "✗ dnsmasq 실행 안됨"
    echo "  해결: sudo systemctl start dnsmasq-px4"
    ISSUES=$((ISSUES + 1))
fi

# Agent 확인
if [ -z "$AGENT_PID" ]; then
    echo "✗ Micro-ROS Agent 실행 안됨"
    echo "  해결: ./start_micro_ros_agent.sh"
    ISSUES=$((ISSUES + 1))
fi

# PX4 연결 확인
TOPICS_NUM=${TOPICS:-0}
if [ "$TOPICS_NUM" -eq 0 ] 2>/dev/null; then
    echo "✗ PX4 토픽 없음"
    echo ""
    echo "  확인 사항:"
    echo "    1. PX4가 eth0 케이블로 연결되어 있는지 확인"
    echo "    2. PX4를 재부팅하여 DHCP로 IP를 받도록 함"
    echo "    3. PX4 파라미터 확인 (QGroundControl):"
    echo "       - UXRCE_DDS_AG_IP = 167772161 (10.0.0.1)"
    echo "       - UXRCE_DDS_PRT = 8888"
    echo "       - UXRCE_DDS_DOM_ID = 0"
    echo "    4. DHCP 로그 확인: sudo tail -f /var/log/dnsmasq-px4.log"
    ISSUES=$((ISSUES + 1))
fi

if [ $ISSUES -eq 0 ]; then
    echo "✓ 모든 설정이 정상입니다!"
else
    echo ""
    echo "발견된 문제: $ISSUES 개"
fi

echo ""

