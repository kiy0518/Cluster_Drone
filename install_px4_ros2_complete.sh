#!/bin/bash
# PX4-ROS2 XRCE-DDS 완전 자동 설치 스크립트
# Ubuntu 22.04 ARM64 (Khadas VIM4)용
# 
# 이 스크립트는 새로 설치한 Ubuntu VIM4에서 한 번만 실행하면
# 모든 설정이 자동으로 완료됩니다.
#
# 실행 방법:
#   chmod +x install_px4_ros2_complete.sh
#   ./install_px4_ros2_complete.sh

set -e

echo "=========================================="
echo "PX4-ROS2 XRCE-DDS 완전 자동 설치"
echo "=========================================="
echo ""

# 아키텍처 확인
ARCH=$(uname -m)
if [ "$ARCH" != "aarch64" ]; then
    echo "⚠ 경고: ARM64 아키텍처가 아닙니다 ($ARCH)"
    read -p "계속하시겠습니까? (y/n): " CONTINUE
    if [ "$CONTINUE" != "y" ]; then
        exit 1
    fi
fi

# 1. ROS2 Humble 설치
echo "=========================================="
echo "1단계: ROS2 Humble 설치"
echo "=========================================="

if command -v ros2 > /dev/null 2>&1; then
    echo "✓ ROS2가 이미 설치되어 있습니다"
else
    echo "ROS2 Humble 설치 중..."
    
    # 시스템 업데이트
    sudo apt update
    sudo apt install -y software-properties-common curl gnupg lsb-release
    
    # ROS2 저장소 추가
    sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc -o /tmp/ros.asc
    sudo gpg --dearmor < /tmp/ros.asc | sudo tee /usr/share/keyrings/ros-archive-keyring.gpg > /dev/null
    sudo rm -f /tmp/ros.asc
    
    ARCH=$(dpkg --print-architecture)
    CODENAME=$(lsb_release -cs)
    echo "deb [arch=${ARCH} signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu ${CODENAME} main" | sudo tee /etc/apt/sources.list.d/ros2-latest.list > /dev/null
    
    # ROS2 Humble 설치
    sudo apt update
    sudo apt install -y ros-humble-desktop
    
    echo "✓ ROS2 Humble 설치 완료"
fi

# 2. 필수 패키지 설치
echo ""
echo "=========================================="
echo "2단계: 필수 패키지 설치"
echo "=========================================="

sudo apt install -y \
    python3-pip \
    python3-colcon-common-extensions \
    git \
    build-essential \
    cmake \
    python3-rosdep \
    python3-vcstool \
    ros-humble-rmw-fastrtps-cpp \
    dnsmasq

# rosdep 초기화
echo "rosdep 초기화 중..."
sudo rosdep init || echo "rosdep 이미 초기화됨"
rosdep update

echo "✓ 필수 패키지 설치 완료"

# 3. Micro-ROS Agent 빌드
echo ""
echo "=========================================="
echo "3단계: Micro-ROS Agent 빌드"
echo "=========================================="

# Micro-ROS Agent 빌드 디렉토리 확인 및 정리
if [ -d ~/Cluster_Drone/micro_ros_ws ]; then
    echo "기존 빌드 디렉토리 확인 중..."
    # 빌드 디렉토리가 있지만 setup.bash가 없거나 손상된 경우 재빌드
    if [ ! -f ~/Cluster_Drone/micro_ros_ws/install/setup.bash ]; then
        echo "빌드 디렉토리 정리 중 (재빌드 필요)..."
        cd ~/Cluster_Drone/micro_ros_ws
        rm -rf build install log
    fi
fi

if [ -f ~/Cluster_Drone/micro_ros_ws/install/setup.bash ] && [ -f ~/Cluster_Drone/micro_ros_ws/install/micro_ros_agent/lib/micro_ros_agent/micro_ros_agent ]; then
    echo "✓ Micro-ROS Agent가 이미 빌드되어 있습니다"
else
    echo "Micro-ROS Agent 빌드 중..."
    
    mkdir -p ~/Cluster_Drone/micro_ros_ws/src
    cd ~/Cluster_Drone/micro_ros_ws/src
    
    if [ ! -d "micro-ROS-Agent" ]; then
        echo "Micro-ROS-Agent 저장소 클론 중..."
        git clone -b humble https://github.com/micro-ROS/micro-ROS-Agent.git
    fi
    
    cd ~/Cluster_Drone/micro_ros_ws
    
    # 기존 빌드 디렉토리 정리 (경로 문제 방지)
    if [ -d build ] || [ -d install ] || [ -d log ]; then
        echo "기존 빌드 디렉토리 정리 중..."
        rm -rf build install log
    fi
    
    source /opt/ros/humble/setup.bash
    
    # 의존성 설치
    echo "의존성 패키지 설치 중..."
    sudo apt install -y \
        python3-rosinstall-generator \
        python3-wstool \
        libasio-dev \
        libtinyxml2-dev \
        libcunit1-dev \
        ros-humble-micro-ros-msgs
    
    echo "rosdep 의존성 설치 중..."
    rosdep install --from-paths src --ignore-src -y || echo "일부 의존성 설치 실패 (계속 진행)"
    
    # 빌드 (의존성 포함하여 전체 빌드)
    echo "Micro-ROS Agent 빌드 중 (시간이 걸릴 수 있습니다)..."
    colcon build
    
    # 빌드 결과 확인
    if [ -f ~/Cluster_Drone/micro_ros_ws/install/micro_ros_agent/lib/micro_ros_agent/micro_ros_agent ]; then
        echo "✓ Micro-ROS Agent 빌드 완료"
    else
        echo "✗ Micro-ROS Agent 빌드 실패"
        echo "로그 확인: ~/Cluster_Drone/micro_ros_ws/log/latest_build/micro_ros_agent"
        exit 1
    fi
fi

# 4. PX4 ROS2 메시지 패키지 빌드
echo ""
echo "=========================================="
echo "4단계: PX4 ROS2 메시지 패키지 빌드"
echo "=========================================="

if [ -f ~/Cluster_Drone/px4_ros2_ws/install/setup.bash ]; then
    echo "✓ PX4 ROS2 메시지 패키지가 이미 빌드되어 있습니다"
else
    echo "PX4 ROS2 메시지 패키지 빌드 중..."
    
    mkdir -p ~/Cluster_Drone/px4_ros2_ws/src
    cd ~/Cluster_Drone/px4_ros2_ws/src
    
    if [ ! -d "px4_msgs" ]; then
        git clone https://github.com/PX4/px4_msgs.git
        cd px4_msgs
        git checkout main
        cd ..
    fi
    
    cd ~/Cluster_Drone/px4_ros2_ws
    source /opt/ros/humble/setup.bash
    
    rosdep install --from-paths src --ignore-src -r -y || echo "일부 의존성 설치 실패 (계속 진행)"
    
    # 빌드
    colcon build
    
    echo "✓ PX4 ROS2 메시지 패키지 빌드 완료"
fi

# 5. 네트워크 설정 (이더넷 직접 연결)
echo ""
echo "=========================================="
echo "5단계: 네트워크 설정 (PX4-SBC 직접 연결)"
echo "=========================================="

# Netplan을 사용한 영구 네트워크 설정
echo "eth0 영구 네트워크 설정 중 (Netplan)..."

# netplan 디렉토리 생성
sudo mkdir -p /etc/netplan

# 기존 netplan 설정 백업
if [ -f /etc/netplan/01-netcfg.yaml ]; then
    sudo cp /etc/netplan/01-netcfg.yaml /etc/netplan/01-netcfg.yaml.backup.$(date +%Y%m%d_%H%M%S) 2>/dev/null || true
fi

# eth0를 10.0.0.1/24로 고정 설정하는 netplan 파일 생성
sudo tee /etc/netplan/01-eth0-px4.yaml > /dev/null << 'NETPLAN_EOF'
network:
  version: 2
  renderer: networkd
  ethernets:
    eth0:
      dhcp4: no
      addresses:
        - 10.0.0.1/24
      nameservers:
        addresses: [8.8.8.8, 8.8.4.4]
NETPLAN_EOF

# netplan 설정 적용
echo "Netplan 설정 적용 중..."
sudo netplan try --timeout 10 || {
    echo "⚠ netplan try 실패, netplan apply 시도 중..."
    sudo netplan apply
}

sleep 2

# eth0 IP 확인
ETH0_IP=$(ip addr show eth0 2>/dev/null | grep "inet " | awk '{print $2}' | cut -d/ -f1)

if [ "$ETH0_IP" = "10.0.0.1" ]; then
    echo "✓ eth0 IP 설정 완료: 10.0.0.1 (재부팅 후에도 유지됨)"
else
    echo "⚠ eth0 IP 설정 확인 실패 (현재: $ETH0_IP)"
    echo "수동으로 설정: sudo ip addr add 10.0.0.1/24 dev eth0"
fi

# dnsmasq 설정
echo "dnsmasq DHCP 서버 설정 중..."

# dnsmasq 설정 디렉토리 생성
sudo mkdir -p /etc/dnsmasq.d

# 기존 설정 백업
sudo cp /etc/dnsmasq.conf /etc/dnsmasq.conf.backup.$(date +%Y%m%d_%H%M%S) 2>/dev/null || true

# dnsmasq 기본 설정 (PX4 전용 설정 파일 사용)
sudo tee /etc/dnsmasq.conf > /dev/null << EOF
# PX4-SBC 직접 연결용 DHCP 서버 설정
# 추가 설정은 /etc/dnsmasq.d/px4.conf에서 관리
conf-dir=/etc/dnsmasq.d/,*.conf
EOF

# PX4 전용 설정 파일
sudo tee /etc/dnsmasq.d/px4.conf > /dev/null << EOF
# PX4-SBC 직접 연결용 DHCP 서버 설정
interface=eth0
bind-interfaces
dhcp-range=10.0.0.10,10.0.0.50,12h
dhcp-option=3,10.0.0.1
dhcp-option=6,8.8.8.8
log-dhcp
EOF

# dnsmasq systemd 서비스 파일 생성
sudo tee /etc/systemd/system/dnsmasq-px4.service > /dev/null << 'DNSMASQ_SERVICE_EOF'
[Unit]
Description=dnsmasq DHCP server for PX4
After=network.target
Wants=network-online.target

[Service]
Type=simple
ExecStart=/usr/sbin/dnsmasq --conf-file=/etc/dnsmasq.d/px4.conf --no-daemon
Restart=on-failure
RestartSec=5
StandardOutput=journal
StandardError=journal

[Install]
WantedBy=multi-user.target
DNSMASQ_SERVICE_EOF

# 기존 dnsmasq 프로세스 종료
sudo pkill -TERM dnsmasq 2>/dev/null || true
sleep 1

# systemd 데몬 재로드
sudo systemctl daemon-reload

# dnsmasq 서비스 활성화 및 시작
echo "dnsmasq 서비스 활성화 및 시작 중..."
sudo systemctl enable dnsmasq-px4.service
sudo systemctl restart dnsmasq-px4.service

sleep 2

if systemctl is-active --quiet dnsmasq-px4.service; then
    echo "✓ dnsmasq DHCP 서버 실행 중 (부팅 시 자동 시작됨)"
else
    echo "⚠ dnsmasq 시작 실패"
    echo "상태 확인: sudo systemctl status dnsmasq-px4"
    echo "로그 확인: sudo journalctl -u dnsmasq-px4 -n 50"
fi

# 6. 환경 설정
echo ""
echo "=========================================="
echo "6단계: 환경 설정"
echo "=========================================="

# .bashrc에 추가 (중복 방지)
if ! grep -q "PX4 ROS2" ~/.bashrc; then
    cat >> ~/.bashrc << 'BASHRC_EOF'

# ==========================================
# PX4 ROS2 XRCE-DDS 환경 설정
# ==========================================

# ROS2 Humble 설정
source /opt/ros/humble/setup.bash

# Micro-ROS Agent 작업 공간 설정
if [ -f ~/Cluster_Drone/micro_ros_ws/install/setup.bash ]; then
    source ~/Cluster_Drone/micro_ros_ws/install/setup.bash
fi

# PX4 ROS2 작업 공간 설정
if [ -f ~/Cluster_Drone/px4_ros2_ws/install/setup.bash ]; then
    source ~/Cluster_Drone/px4_ros2_ws/install/setup.bash
fi

# ROS2 도메인 ID 설정
export ROS_DOMAIN_ID=0

# RMW 구현체 설정 (ARM64에서는 FastRTPS 사용)
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

# Micro-ROS Agent 실행 함수
micro-ros-agent() {
    if [ -f ~/Cluster_Drone/micro_ros_ws/install/setup.bash ]; then
        source ~/Cluster_Drone/micro_ros_ws/install/setup.bash
        ros2 run micro_ros_agent micro_ros_agent "$@"
    else
        echo "Micro-ROS Agent가 설치되지 않았습니다."
    fi
}
BASHRC_EOF
    echo "✓ 환경 설정 추가 완료"
else
    echo "✓ 환경 설정 이미 추가됨"
fi

# 7. Micro-ROS Agent systemd 서비스 설정
echo ""
echo "=========================================="
echo "7단계: Micro-ROS Agent systemd 서비스 설정"
echo "=========================================="

# 현재 사용자 정보 가져오기
CURRENT_USER=$(whoami)
CURRENT_HOME=$(eval echo ~$CURRENT_USER)

# Micro-ROS Agent 실행 스크립트 생성
MICRO_ROS_SCRIPT="/usr/local/bin/micro-ros-agent-service.sh"
sudo tee "$MICRO_ROS_SCRIPT" > /dev/null << SCRIPT_EOF
#!/bin/bash
# Micro-ROS Agent 실행 스크립트

export HOME=$CURRENT_HOME
export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

# Source ROS2 environment
source /opt/ros/humble/setup.bash

# Source Micro-ROS Agent workspace (필수!)
if [ -f $CURRENT_HOME/Cluster_Drone/micro_ros_ws/install/setup.bash ]; then
    source $CURRENT_HOME/Cluster_Drone/micro_ros_ws/install/setup.bash
fi

# Source PX4 ROS2 messages workspace
if [ -f $CURRENT_HOME/Cluster_Drone/px4_ros2_ws/install/setup.bash ]; then
    source $CURRENT_HOME/Cluster_Drone/px4_ros2_ws/install/setup.bash
fi

# Set library path for Micro-ROS Agent (모든 필요한 경로 포함)
export LD_LIBRARY_PATH=$CURRENT_HOME/Cluster_Drone/micro_ros_ws/install/micro_ros_agent/lib:/opt/ros/humble/lib:/opt/ros/humble/lib/aarch64-linux-gnu:\${LD_LIBRARY_PATH}

# Start Micro-ROS Agent directly
exec $CURRENT_HOME/Cluster_Drone/micro_ros_ws/install/micro_ros_agent/lib/micro_ros_agent/micro_ros_agent udp4 --port 8888
SCRIPT_EOF

sudo chmod +x "$MICRO_ROS_SCRIPT"

# systemd 서비스 파일 생성
sudo tee /etc/systemd/system/micro-ros-agent.service > /dev/null << SERVICE_EOF
[Unit]
Description=Micro-ROS Agent for PX4 XRCE-DDS
After=network.target
Wants=network-online.target

[Service]
Type=simple
User=$CURRENT_USER
Group=$CURRENT_USER
WorkingDirectory=$CURRENT_HOME
Environment="HOME=$CURRENT_HOME"
ExecStart=/usr/local/bin/micro-ros-agent-service.sh
Restart=always
RestartSec=5
StandardOutput=journal
StandardError=journal

[Install]
WantedBy=multi-user.target
SERVICE_EOF

# systemd 데몬 재로드
sudo systemctl daemon-reload

# 기존 프로세스 종료 (있다면)
if pgrep -f "micro_ros_agent" > /dev/null; then
    echo "기존 Micro-ROS Agent 프로세스 종료 중..."
    pkill -f "micro_ros_agent" || true
    sleep 2
fi

# 서비스 활성화 및 시작
echo "Micro-ROS Agent 서비스 활성화 및 시작 중..."
sudo systemctl enable micro-ros-agent.service
sudo systemctl restart micro-ros-agent.service

sleep 3

# 서비스 상태 확인
sleep 3
if systemctl is-active --quiet micro-ros-agent.service; then
    echo "✓ Micro-ROS Agent 서비스 실행 중 (포트 8888)"
    echo "  - 부팅 시 자동 시작됨"
    echo "  - 로그 확인: sudo journalctl -u micro-ros-agent -f"
    
    # 포트 확인
    sleep 1
    if ss -ulnp 2>/dev/null | grep -q ":8888" || netstat -ulnp 2>/dev/null | grep -q ":8888"; then
        echo "  - 포트 8888 리스닝 확인됨"
    else
        echo "  ⚠ 포트 8888이 아직 열리지 않았을 수 있습니다 (잠시 후 확인)"
    fi
else
    echo "⚠ Micro-ROS Agent 서비스 시작 실패"
    echo "상태 확인: sudo systemctl status micro-ros-agent"
    echo "로그 확인: sudo journalctl -u micro-ros-agent -n 50"
    echo ""
    echo "수동 진단:"
    echo "  1. 실행 파일 확인: ls -la $CURRENT_HOME/Cluster_Drone/micro_ros_ws/install/micro_ros_agent/lib/micro_ros_agent/micro_ros_agent"
    echo "  2. 수동 실행 테스트: sudo -u $CURRENT_USER $MICRO_ROS_SCRIPT"
fi

# 8. 설치 검증
echo ""
echo "=========================================="
echo "8단계: 설치 검증"
echo "=========================================="

VERIFICATION_FAILED=0

# ROS2 확인
if command -v ros2 > /dev/null 2>&1; then
    echo "✓ ROS2 설치 확인됨"
else
    echo "✗ ROS2 설치 확인 실패"
    VERIFICATION_FAILED=1
fi

# Micro-ROS Agent 실행 파일 확인
if [ -f ~/Cluster_Drone/micro_ros_ws/install/micro_ros_agent/lib/micro_ros_agent/micro_ros_agent ]; then
    echo "✓ Micro-ROS Agent 실행 파일 확인됨"
else
    echo "✗ Micro-ROS Agent 실행 파일 없음"
    VERIFICATION_FAILED=1
fi

# PX4 메시지 패키지 확인
if [ -f ~/Cluster_Drone/px4_ros2_ws/install/setup.bash ]; then
    echo "✓ PX4 ROS2 메시지 패키지 확인됨"
else
    echo "✗ PX4 ROS2 메시지 패키지 없음"
    VERIFICATION_FAILED=1
fi

# eth0 IP 확인
ETH0_IP=$(ip addr show eth0 2>/dev/null | grep "inet " | awk '{print $2}' | cut -d/ -f1)
if [ "$ETH0_IP" = "10.0.0.1" ]; then
    echo "✓ eth0 IP 설정 확인됨 (10.0.0.1)"
else
    echo "⚠ eth0 IP 설정 확인 실패 (현재: $ETH0_IP)"
    echo "  재부팅 후 자동으로 설정됩니다"
fi

# 서비스 상태 확인
if systemctl is-enabled --quiet dnsmasq-px4.service 2>/dev/null; then
    echo "✓ dnsmasq 서비스 활성화됨"
else
    echo "⚠ dnsmasq 서비스 활성화 확인 실패"
fi

if systemctl is-enabled --quiet micro-ros-agent.service 2>/dev/null; then
    echo "✓ micro-ros-agent 서비스 활성화됨"
else
    echo "⚠ micro-ros-agent 서비스 활성화 확인 실패"
fi

# 9. 설치 완료 및 요약
echo ""
echo "=========================================="
echo "설치 완료!"
echo "=========================================="
echo ""

if [ $VERIFICATION_FAILED -eq 0 ]; then
    echo "✓ 모든 구성요소가 정상적으로 설치되었습니다!"
else
    echo "⚠ 일부 구성요소 설치에 문제가 있을 수 있습니다"
    echo "  위의 검증 결과를 확인하세요"
fi

echo ""
echo "설치된 구성요소:"
echo "  ✓ ROS2 Humble"
echo "  ✓ Micro-ROS Agent (systemd 서비스, 부팅 시 자동 시작)"
echo "  ✓ PX4 ROS2 메시지 패키지"
echo "  ✓ DHCP 서버 (dnsmasq, systemd 서비스, 부팅 시 자동 시작)"
echo "  ✓ 네트워크 설정 (eth0: 10.0.0.1, Netplan으로 부팅 시 자동 설정)"
echo ""
echo "다음 단계:"
echo "  1. 시스템 재부팅 (권장):"
echo "     sudo reboot"
echo ""
echo "  2. 재부팅 후 확인:"
echo "     cd ~/Cluster_Drone"
echo "     ./check_px4_connection.sh"
echo ""
echo "  3. PX4 파라미터 설정 (QGroundControl 또는 PX4 콘솔):"
echo "     - UXRCE_DDS_AG_IP = 167772161 (10.0.0.1)"
echo "     - UXRCE_DDS_PRT = 8888"
echo "     - UXRCE_DDS_DOM_ID = 0"
echo ""
echo "  4. PX4 재부팅 후 연결 확인:"
echo "     ros2 topic list"
echo ""
echo "유용한 명령:"
echo "  cd ~/Cluster_Drone"
echo "  ./check_px4_connection.sh                    # 연결 상태 종합 확인"
echo "  ros2 topic list                              # 토픽 목록"
echo "  ros2 topic echo /fmu/out/vehicle_status_v1  # PX4 상태"
echo "  sudo systemctl status micro-ros-agent       # Agent 상태 확인"
echo "  sudo journalctl -u micro-ros-agent -f       # Agent 로그"
echo "  sudo journalctl -u dnsmasq-px4 -f           # DHCP 로그"
echo ""
echo "문제 해결:"
echo "  자세한 내용은 README.md의 '문제 해결' 섹션을 참고하세요"
echo ""

