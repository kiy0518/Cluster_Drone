# Cluster_Drone

PX4-ROS2 XRCE-DDS 연결 프로젝트

## 개요

이 프로젝트는 Khadas VIM4 (Ubuntu 22.04 ARM64)에서 PX4와 ROS2를 XRCE-DDS로 연결하는 완전 자동화 솔루션입니다.

## 빠른 시작 (새로 설치한 Ubuntu VIM4)

### 단 한 번의 명령으로 모든 설정 완료!

```bash
# 1. 파일 복사 (다른 VIM4에 설치하는 경우)
# 방법 A: 전체 복사 (빠름, 같은 사용자 이름인 경우)
scp -r Cluster_Drone user@new-vim4:~/Cluster_Drone

# 방법 B: 소스 코드만 복사 (권장, 더 안전)
# 자세한 내용은 COPY_GUIDE.md 참고

# 2. 설치 스크립트 실행 (모든 것이 자동으로 설정됩니다)
cd ~/Cluster_Drone
chmod +x install_px4_ros2_complete.sh
./install_px4_ros2_complete.sh
```

**참고:** 복사 방법에 대한 자세한 가이드는 [COPY_GUIDE.md](COPY_GUIDE.md)를 참고하세요.

**이 스크립트 하나로 다음이 모두 자동 설정됩니다:**
- ✓ ROS2 Humble 설치
- ✓ Micro-ROS Agent 빌드 및 systemd 서비스 설정
- ✓ PX4 ROS2 메시지 패키지 빌드
- ✓ 네트워크 설정 (eth0: 10.0.0.1, 부팅 시 자동 설정)
- ✓ DHCP 서버 (dnsmasq) 설정 및 systemd 서비스
- ✓ 환경 변수 설정 (.bashrc)

**설치 후:**
```bash
# 시스템 재부팅 (권장)
sudo reboot

# 재부팅 후 연결 상태 확인
cd ~/Cluster_Drone
./check_px4_connection.sh
```

### 3. PX4 설정

QGroundControl 또는 PX4 콘솔에서 다음 파라미터 설정:

```
UXRCE_DDS_AG_IP = 167772161  (10.0.0.1의 int32 값)
UXRCE_DDS_PRT = 8888
UXRCE_DDS_DOM_ID = 1  (단일 기체 기본값, 군집 비행 시 기체별로 1, 2, 3)
UXRCE_DDS_CFG = Ethernet
```

**IP 변환 도구:**
```bash
cd ~/Cluster_Drone
./ip_to_decimal.sh 10.0.0.1
```

### 4. 전원 켜는 순서

**권장 순서:**
1. **VIM4 (SBC) 먼저 켜기** - DHCP 서버와 Micro-ROS Agent가 준비됨
2. **FC (PX4) 나중에 켜기** - DHCP로 IP를 받고 Agent에 연결

**확인:**
```bash
# VIM4 부팅 후 확인
ip addr show eth0              # 10.0.0.1 확인
systemctl status dnsmasq-px4   # DHCP 서버 확인
ss -ulnp | grep 8888           # Micro-ROS Agent 확인

# PX4 부팅 후 확인
ros2 topic list                # PX4 토픽 확인
```

## 파일 구조

```
Cluster_Drone/
├── install_px4_ros2_complete.sh    # 완전 자동 설치 스크립트 (이것 하나로 모든 설정 완료!)
├── check_px4_connection.sh          # 연결 상태 진단 도구
├── ip_to_decimal.sh                 # IP 주소 변환 도구
├── start_micro_ros_agent_wrapper.sh # Agent wrapper 스크립트 (자동 생성됨)
├── README.md                        # 이 파일
├── QUICK_START.md                   # 빠른 시작 가이드
└── [빌드 디렉토리]
    ├── micro_ros_ws/                # Micro-ROS Agent 작업 공간
    └── px4_ros2_ws/                 # PX4 ROS2 메시지 작업 공간
```

## 주요 명령어

### ROS2 토픽

```bash
# 모든 토픽 목록
ros2 topic list

# 토픽 정보 확인
ros2 topic info /fmu/out/vehicle_status_v1

# 토픽 모니터링
ros2 topic echo /fmu/out/vehicle_status_v1

# 토픽 발행 속도 확인
ros2 topic hz /fmu/out/vehicle_status_v1
```

### 주요 PX4 토픽

```bash
# 상태 정보
ros2 topic echo /fmu/out/vehicle_status_v1

# 위치 정보
ros2 topic echo /fmu/out/vehicle_odometry
ros2 topic echo /fmu/out/vehicle_local_position

# 자세 정보
ros2 topic echo /fmu/out/vehicle_attitude

# 배터리 상태
ros2 topic echo /fmu/out/battery_status
```

### 서비스 관리

```bash
# DHCP 서버 상태
sudo systemctl status dnsmasq-px4
sudo systemctl restart dnsmasq-px4

# Micro-ROS Agent 상태
sudo systemctl status micro-ros-agent
sudo systemctl restart micro-ros-agent

# 로그 확인
sudo journalctl -u dnsmasq-px4 -f
sudo journalctl -u micro-ros-agent -f
```

### 네트워크

```bash
# eth0 IP 확인
ip addr show eth0

# 연결된 장치 확인
ip neigh show dev eth0

# DHCP 로그 확인
sudo tail -f /var/log/dnsmasq-px4.log
```

### 진단

```bash
# 연결 상태 종합 확인
cd ~/Cluster_Drone
./check_px4_connection.sh
```

## 네트워크 구성

### 기본 구성 (단일 SBC)

```
PX4 (클라이언트)  <--LAN 케이블-->  SBC (서버)
10.0.0.10-50                       10.0.0.1
                                     DHCP 서버
                                     Micro-ROS Agent (포트 8888)
```

- **SBC**: DHCP 서버 + Micro-ROS Agent (서버)
- **PX4**: DHCP 클라이언트 + XRCE-DDS 클라이언트

### 멀티 SBC 구성 (ROS2 DDS 멀티캐스트)

```
                    WiFi 네트워크
        ┌───────────────┼───────────────┐
        │               │               │
    vim4-A          vim4-B          다른 노드
   (WiFi)      eth0←→PX4 (직접)      (WiFi)
        │               │
        └─────── ROS2 DDS 멀티캐스트 ────┘
              (같은 ROS_DOMAIN_ID)
```

**중요:** ROS2 DDS는 멀티캐스트를 사용하므로, 같은 WiFi 네트워크와 같은 `ROS_DOMAIN_ID`를 사용하는 모든 노드가 토픽을 공유할 수 있습니다.

자세한 내용은 [NETWORK_ARCHITECTURE.md](NETWORK_ARCHITECTURE.md)를 참고하세요.

## 재시작 후 자동 시작

`install_px4_ros2_complete.sh`를 실행하면 다음이 자동으로 설정되어 **재시작 후에도 자동으로 작동**합니다:

- **dnsmasq-px4**: DHCP 서버 (systemd 서비스, 부팅 시 자동 시작)
- **micro-ros-agent**: Micro-ROS Agent (systemd 서비스, 부팅 시 자동 시작)
- **eth0 네트워크**: Netplan으로 설정 (10.0.0.1, 부팅 시 자동 설정)

**재시작 후 확인:**
```bash
# 서비스 상태 확인
systemctl status dnsmasq-px4
systemctl status micro-ros-agent

# 네트워크 확인
ip addr show eth0

# 연결 상태 종합 확인
cd ~/Cluster_Drone
./check_px4_connection.sh
```

## 문제 해결

### PX4 토픽이 보이지 않는 경우

1. **연결 상태 확인:**
   ```bash
   cd ~/Cluster_Drone
   ./check_px4_connection.sh
   ```

2. **서비스 확인:**
   ```bash
   systemctl status dnsmasq-px4
   systemctl status micro-ros-agent
   ```

3. **PX4 재부팅:**
   - PX4를 재부팅하여 DHCP로 IP를 받도록 함

4. **PX4 파라미터 확인:**
   - `UXRCE_DDS_AG_IP`가 올바른지 확인
   - PX4 재부팅 확인

5. **네트워크 연결 확인:**
   ```bash
   ip neigh show dev eth0
   sudo tail -f /var/log/dnsmasq-px4.log
   ```

### DHCP가 작동하지 않는 경우

```bash
# dnsmasq 상태 확인
sudo systemctl status dnsmasq-px4

# 로그 확인
sudo journalctl -u dnsmasq-px4 -n 50

# 재시작
sudo systemctl restart dnsmasq-px4
```

### Micro-ROS Agent가 시작되지 않는 경우

```bash
# 서비스 상태 확인
sudo systemctl status micro-ros-agent

# 로그 확인
sudo journalctl -u micro-ros-agent -n 50

# 재빌드가 필요한 경우
cd ~/Cluster_Drone/micro_ros_ws
rm -rf build install log
source /opt/ros/humble/setup.bash
colcon build

# 서비스 재시작
sudo systemctl restart micro-ros-agent
```

### eth0에 IP가 할당되지 않는 경우

```bash
# Netplan 설정 확인
cat /etc/netplan/01-eth0-px4.yaml

# Netplan 설정 적용
sudo netplan apply

# 또는 재부팅
sudo reboot
```

## IP 주소 변환

PX4 파라미터에 설정할 IP 주소를 int32 값으로 변환:

```bash
cd ~/Cluster_Drone
./ip_to_decimal.sh <IP주소>
```

예:
```bash
./ip_to_decimal.sh 10.0.0.1
# 결과: -1062731519
```

## 시스템 요구사항

- **OS**: Ubuntu 22.04 LTS (새로 설치한 깨끗한 시스템 권장)
- **아키텍처**: ARM64 (aarch64)
- **SBC**: Khadas VIM4 (Amlogic A311D2)
- **PX4 펌웨어**: v1.16.0 (XRCE-DDS 지원)
- **QGroundControl**: v5.0.8 (64-bit)
- **연결**: PX4와 SBC가 이더넷 케이블로 직접 연결
- **인터넷 연결**: 설치 중 패키지 다운로드를 위해 필요

## 추가 문서

- [PROJECT_SUMMARY.md](PROJECT_SUMMARY.md) - **프로젝트 종합 정리** (패키지, 파라미터 전체 정리)
- [INSTALLATION.md](INSTALLATION.md) - 상세 설치 가이드
- [COPY_GUIDE.md](COPY_GUIDE.md) - 파일 복사 가이드
- [NETWORK_ARCHITECTURE.md](NETWORK_ARCHITECTURE.md) - 네트워크 아키텍처 설명
- [SWARM_FLIGHT.md](SWARM_FLIGHT.md) - 군집 비행 설정 가이드
- [QGC_MULTI_VEHICLE.md](QGC_MULTI_VEHICLE.md) - QGroundControl 다중 기체 모니터링
- [QGC_NETWORK_SETUP.md](QGC_NETWORK_SETUP.md) - QGC 네트워크 연결 설정
- [QGC_CONNECTION_GUIDE.md](QGC_CONNECTION_GUIDE.md) - QGC 연결 추가 가이드

## 참고 자료

- [PX4 공식 문서](https://docs.px4.io/main/en/ros/ros2_comm.html)
- [Micro-ROS 문서](https://micro.ros.org/)
- [ROS2 Humble 문서](https://docs.ros.org/en/humble/)
- [QGroundControl 문서](https://docs.qgroundcontrol.com/)

## 지원

문제가 발생하면:
1. `./check_px4_connection.sh`로 진단
2. 로그 파일 확인: `/var/log/dnsmasq-px4.log`
3. 시스템 로그: `journalctl -u dnsmasq-px4`, `journalctl -u micro-ros-agent`
4. ROS2 환경: `ros2 doctor`
