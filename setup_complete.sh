#!/bin/bash
# PX4-ROS2 XRCE-DDS 완전 자동 설정 스크립트
# 재시작 후에도 자동으로 작동하도록 모든 설정을 적용
# 다른 VIM4에 복사 후 이 스크립트만 실행하면 됨

# set -e 제거 (일부 명령어 실패해도 계속 진행)

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

echo "=========================================="
echo "PX4-ROS2 XRCE-DDS 완전 자동 설정"
echo "=========================================="
echo ""

# 1. 네트워크 설정 (eth0 IP 할당)
echo "1. 네트워크 설정 중..."
ETH0_CONN=$(nmcli -t -f NAME,DEVICE connection show | grep ":eth0$" | cut -d: -f1 | head -1)

if [ -z "$ETH0_CONN" ]; then
    echo "   새 연결 생성 중..."
    sudo nmcli connection add type ethernet ifname eth0 con-name "eth0-px4" \
        ipv4.addresses 10.0.0.1/24 \
        ipv4.method manual \
        ipv4.dhcp-client-id "" \
        ipv6.method ignore
    ETH0_CONN="eth0-px4"
else
    echo "   기존 연결 수정 중: $ETH0_CONN"
    sudo nmcli connection modify "$ETH0_CONN" \
        ipv4.addresses 10.0.0.1/24 \
        ipv4.method manual \
        ipv4.dhcp-client-id "" \
        ipv6.method ignore \
        connection.autoconnect yes
fi

# 연결 활성화
sudo nmcli connection up "$ETH0_CONN" || {
    sudo nmcli device disconnect eth0
    sleep 2
    sudo nmcli connection up "$ETH0_CONN"
}

echo "   ✓ eth0 네트워크 설정 완료 (10.0.0.1)"

# 2. dnsmasq 설정
echo ""
echo "2. dnsmasq DHCP 서버 설정 중..."
sudo mkdir -p /etc/dnsmasq.d
sudo tee /etc/dnsmasq.d/px4.conf > /dev/null << 'EOF'
# dnsmasq configuration for PX4
interface=eth0
bind-interfaces
dhcp-range=10.0.0.10,10.0.0.50,255.255.255.0,12h
dhcp-option=3,10.0.0.1
dhcp-option=6,10.0.0.1
log-queries
log-dhcp
log-facility=/var/log/dnsmasq-px4.log
EOF

# dnsmasq 서비스 설정
sudo cp "$SCRIPT_DIR/dnsmasq-px4.service" /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable dnsmasq-px4
sudo systemctl restart dnsmasq-px4

echo "   ✓ dnsmasq DHCP 서버 설정 완료"

# 3. Micro-ROS Agent 서비스 설정
echo ""
echo "3. Micro-ROS Agent 서비스 설정 중..."

# wrapper 스크립트 확인
if [ ! -f "$SCRIPT_DIR/start_micro_ros_agent_wrapper.sh" ]; then
    echo "   ✗ wrapper 스크립트를 찾을 수 없습니다"
    exit 1
fi

# 서비스 파일 확인
if [ ! -f "$SCRIPT_DIR/micro-ros-agent.service" ]; then
    echo "   ✗ 서비스 파일을 찾을 수 없습니다"
    exit 1
fi

# 기존 Agent 프로세스 종료
pkill -f "micro_ros_agent.*udp4" 2>/dev/null || true
pkill -f "start_micro_ros_agent_wrapper" 2>/dev/null || true
sleep 1

# 서비스 파일 복사 및 활성화
sudo cp "$SCRIPT_DIR/micro-ros-agent.service" /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable micro-ros-agent
sudo systemctl restart micro-ros-agent

sleep 2
if systemctl is-active --quiet micro-ros-agent; then
    echo "   ✓ Micro-ROS Agent 서비스 설정 완료"
else
    echo "   ⚠ Micro-ROS Agent 서비스 시작 실패"
    echo "   로그 확인: sudo journalctl -u micro-ros-agent -n 30"
fi

# 4. 환경 변수 설정 (.bashrc)
echo ""
echo "4. 환경 변수 설정 중..."
if ! grep -q "Cluster_Drone" ~/.bashrc 2>/dev/null; then
    cat >> ~/.bashrc << 'BASHRC_EOF'

# Cluster_Drone PX4-ROS2 환경 설정
export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

# ROS2 환경 로드
if [ -f /opt/ros/humble/setup.bash ]; then
    source /opt/ros/humble/setup.bash
fi

# PX4 ROS2 메시지 패키지 로드
if [ -f ~/Cluster_Drone/px4_ros2_ws/install/setup.bash ]; then
    source ~/Cluster_Drone/px4_ros2_ws/install/setup.bash
fi
BASHRC_EOF
    echo "   ✓ .bashrc에 환경 변수 추가됨"
else
    echo "   ✓ .bashrc에 이미 설정되어 있음"
fi

# 5. 상태 확인
echo ""
echo "5. 설정 상태 확인 중..."
echo ""

# 네트워크 확인
if ip addr show eth0 | grep -q "10.0.0.1"; then
    echo "   ✓ eth0 IP: 10.0.0.1"
else
    echo "   ✗ eth0 IP 할당 안됨"
fi

# dnsmasq 확인
if systemctl is-active --quiet dnsmasq-px4; then
    echo "   ✓ dnsmasq-px4: 실행 중"
else
    echo "   ✗ dnsmasq-px4: 실행 안됨"
fi

# Micro-ROS Agent 확인
if systemctl is-active --quiet micro-ros-agent; then
    echo "   ✓ micro-ros-agent: 실행 중"
    AGENT_PID=$(ss -ulnp 2>/dev/null | grep ":8888" | grep -oP 'pid=\K[0-9]+' | head -1)
    if [ -n "$AGENT_PID" ]; then
        echo "   ✓ 포트 8888 리스닝 중 (PID: $AGENT_PID)"
    fi
else
    echo "   ✗ micro-ros-agent: 실행 안됨"
fi

echo ""
echo "=========================================="
echo "설정 완료"
echo "=========================================="
echo ""
echo "재부팅 후 자동으로 시작되는 서비스:"
echo "  - dnsmasq-px4 (DHCP 서버)"
echo "  - micro-ros-agent (Micro-ROS Agent)"
echo "  - eth0 네트워크 연결 (10.0.0.1)"
echo ""
echo "다음 단계:"
echo "  1. PX4를 이더넷 케이블로 연결"
echo "  2. PX4 파라미터 설정 (QGroundControl):"
echo "     - UXRCE_DDS_AG_IP = 167772161 (10.0.0.1)"
echo "     - UXRCE_DDS_PRT = 8888"
echo "     - UXRCE_DDS_DOM_ID = 0"
echo "  3. PX4 재부팅"
echo "  4. 연결 확인: ros2 topic list"
echo ""
echo "진단 도구:"
echo "  cd ~/Cluster_Drone"
echo "  ./check_px4_connection.sh"
echo ""

