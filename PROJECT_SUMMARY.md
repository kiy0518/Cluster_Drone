# 3대 드론 동시 제어 프로젝트 종합 정리

## 프로젝트 개요

이 프로젝트는 **화재 진압용 군집 드론 시스템**입니다. 화재 발생 시 3대의 드론이 자동으로 발화점으로 이동하여 열화상 카메라로 타겟팅하고, 소화탄을 순차적으로 격발한 후 복귀하는 완전 자동화 시스템입니다.

**핵심 기능:**
- 화재 감지 및 위치 파악
- 자동 경로 계획 및 군집 비행
- 열화상 카메라 기반 타겟팅
- 소화탄 순차 격발
- 자동 복귀 및 착륙

**하드웨어:**
- **플랫폼**: Khadas VIM4 (Ubuntu 22.04 ARM64)
- **FC**: Pixhawk 6X(Standard v2A PM02D)
- **PX4 펌웨어**: v1.16.0
- **QGroundControl**: v5.0.8 (64-bit)
- **통신**: ROS2 XRCE-DDS, MAVLink
- **제어**: ROS2 기반 분산 제어 시스템

**자세한 기술 스택은 [FIREFIGHTING_SYSTEM.md](FIREFIGHTING_SYSTEM.md)를 참고하세요.**

---

## 1. 설치된 패키지 종류

### 1.1 KHADAS VIM4 시스템 패키지

#### 기본 시스템 도구
```bash
software-properties-common  # 소프트웨어 저장소 관리
curl                        # HTTP 클라이언트
gnupg                       # 암호화 키 관리
lsb-release                 # 리눅스 표준 기반 정보
```

#### 네트워크 관리
```bash
dnsmasq                     # DHCP 서버 (PX4에 IP 할당)
netplan                     # 네트워크 설정 관리 (Ubuntu 기본)
```

#### 개발 도구
```bash
build-essential            # 컴파일러 및 빌드 도구
cmake                      # 빌드 시스템
git                        # 버전 관리
python3-pip                # Python 패키지 관리자
```

**용도:**
- 시스템 기본 기능 제공
- 네트워크 설정 및 DHCP 서버 운영
- 소스 코드 빌드 및 개발

---

### 1.2 ROS2 관련 설치 패키지

#### ROS2 핵심 패키지
```bash
ros-humble-desktop         # ROS2 Humble 전체 데스크톱 버전
  ├── ros-humble-ros-base
  ├── ros-humble-rclcpp
  ├── ros-humble-rclpy
  ├── ros-humble-common-interfaces
  └── 기타 ROS2 핵심 패키지
```

#### ROS2 개발 도구
```bash
python3-colcon-common-extensions  # ROS2 빌드 시스템
python3-rosdep                    # ROS2 의존성 관리
python3-vcstool                   # ROS2 워크스페이스 관리
python3-rosinstall-generator      # ROS2 패키지 설치 생성기
python3-wstool                    # ROS2 워크스페이스 도구
```

#### ROS2 RMW 구현체
```bash
ros-humble-rmw-fastrtps-cpp      # FastRTPS DDS 구현체 (ARM64용)
```

#### ROS2 메시지 패키지
```bash
ros-humble-micro-ros-msgs        # Micro-ROS 메시지 정의
```

**용도:**
- ROS2 통신 프레임워크
- 분산 시스템 제어
- 토픽 기반 메시지 교환
- 서비스 및 액션 호출

---

### 1.3 PX4 통신 관련 설치 패키지

#### Micro-ROS Agent
```bash
# 소스에서 빌드
micro-ros-agent              # Micro-ROS Agent 실행 파일
  ├── micro-ROS-Agent (GitHub 저장소)
  └── 의존성:
      ├── libasio-dev        # 비동기 I/O 라이브러리
      ├── libtinyxml2-dev    # XML 파서
      └── libcunit1-dev      # 단위 테스트 프레임워크
```

#### PX4 ROS2 메시지 패키지
```bash
# 소스에서 빌드
px4_msgs                     # PX4 ROS2 메시지 정의
  └── px4_msgs (GitHub 저장소)
```

**용도:**
- PX4와 ROS2 간 브리지 역할
- XRCE-DDS 프로토콜 변환
- PX4 데이터를 ROS2 토픽으로 변환

---

### 1.4 Systemd 서비스

#### Micro-ROS Agent 서비스
```bash
/etc/systemd/system/micro-ros-agent.service
/usr/local/bin/micro-ros-agent-service.sh
```

#### DHCP 서버 서비스
```bash
/etc/systemd/system/dnsmasq-px4.service
/etc/dnsmasq.d/px4.conf
```

**용도:**
- 부팅 시 자동 시작
- 서비스 자동 재시작
- 로그 관리

---

## 2. FC (Flight Controller) 파라미터 설정

### 2.1 XRCE-DDS 통신 파라미터 (ROS2 통신용)

#### 필수 파라미터

**기체 A:**
```
UXRCE_DDS_AG_IP = 167772161      # 10.0.0.1의 int32 값
UXRCE_DDS_PRT = 8888             # Micro-ROS Agent 포트
UXRCE_DDS_DOM_ID = 1             # ROS2 도메인 ID (기체 A = 1)
UXRCE_DDS_CFG = Ethernet         # 이더넷 통신 설정
```

**기체 B:**
```
UXRCE_DDS_AG_IP = 167772161      # 10.0.0.1의 int32 값
UXRCE_DDS_PRT = 8888             # Micro-ROS Agent 포트
UXRCE_DDS_DOM_ID = 2             # ROS2 도메인 ID (기체 B = 2)
UXRCE_DDS_CFG = Ethernet         # 이더넷 통신 설정
```

**기체 C:**
```
UXRCE_DDS_AG_IP = 167772161      # 10.0.0.1의 int32 값
UXRCE_DDS_PRT = 8888             # Micro-ROS Agent 포트
UXRCE_DDS_DOM_ID = 3             # ROS2 도메인 ID (기체 C = 3)
UXRCE_DDS_CFG = Ethernet         # 이더넷 통신 설정
```

**주의:** 
- 각 기체의 `UXRCE_DDS_DOM_ID`는 연결된 SBC의 `ROS_DOMAIN_ID`와 **일치**해야 합니다
- 기체 A = 도메인 1, 기체 B = 도메인 2, 기체 C = 도메인 3

**설정 방법:**
1. QGroundControl에서 기체에 연결
2. Parameters → XRCE-DDS
3. 위 파라미터 값 입력
4. 기체 재부팅

**IP 주소 변환:**
```bash
cd ~/Cluster_Drone
./ip_to_decimal.sh 10.0.0.1
# 결과: 167772161
```

---

### 2.2 MAVLink 통신 파라미터 (QGroundControl 통신용)

#### 필수 파라미터

**기체 A:**
```
MAV_SYS_ID = 1                   # 고유 시스템 ID (필수!)
MAV_1_CONFIG = TELEM 2 (UDP)     # UDP 통신 설정
MAV_1_MODE = 2 (Onboard)         # Onboard 모드
MAV_1_RATE = 1000000             # 전송률 (1MHz)
MAV_1_FORWARD = 1                # 포워딩 활성화
MAV_1_BROADCAST = 1              # 브로드캐스트 활성화
```

**기체 B:**
```
MAV_SYS_ID = 2                   # 고유 시스템 ID (필수!)
MAV_1_CONFIG = TELEM 2 (UDP)     # UDP 통신 설정
MAV_1_MODE = 2 (Onboard)         # Onboard 모드
MAV_1_RATE = 1000000             # 전송률 (1MHz)
MAV_1_FORWARD = 1                # 포워딩 활성화
MAV_1_BROADCAST = 1              # 브로드캐스트 활성화
```

**기체 C:**
```
MAV_SYS_ID = 3                   # 고유 시스템 ID (필수!)
MAV_1_CONFIG = TELEM 2 (UDP)     # UDP 통신 설정
MAV_1_MODE = 2 (Onboard)         # Onboard 모드
MAV_1_RATE = 1000000             # 전송률 (1MHz)
MAV_1_FORWARD = 1                # 포워딩 활성화
MAV_1_BROADCAST = 1              # 브로드캐스트 활성화
```

**설정 방법:**
1. QGroundControl에서 기체에 연결
2. Parameters → MAVLink
3. 위 파라미터 값 입력
4. 기체 재부팅

**중요:**
- `MAV_SYS_ID`는 각 기체마다 **반드시 고유**해야 함
- 같은 ID를 사용하면 QGC에서 구분 불가

---

### 2.3 네트워크 파라미터

#### DHCP 설정 (자동 IP 할당)

각 PX4 기체는 DHCP로 자동으로 IP를 받습니다.

**기체 A:**
- MAC 주소: AA:BB:CC:DD:EE:01
- 할당 IP: 10.0.0.11 (vim4-A의 DHCP 서버에서 할당)

**기체 B:**
- MAC 주소: AA:BB:CC:DD:EE:02
- 할당 IP: 10.0.0.21 (vim4-B의 DHCP 서버에서 할당)

**기체 C:**
- MAC 주소: AA:BB:CC:DD:EE:03
- 할당 IP: 10.0.0.31 (vim4-C의 DHCP 서버에서 할당)

**설정 방법:**
- 각 SBC의 `/etc/dnsmasq.d/px4.conf`에서 MAC 주소별 IP 할당 설정
- PX4 재부팅 시 자동으로 IP 할당

---

## 3. 네트워크 구성

### 3.1 네트워크 인터페이스

#### 각 VIM4 보드

```
eth0: 10.0.0.1/24
  └─ PX4와 직접 연결 (LAN 케이블)
     └─ DHCP 서버 운영
     └─ Micro-ROS Agent (포트 8888)

wlan0: 192.168.100.x/24
  └─ WiFi 네트워크 연결
     └─ ROS2 DDS 멀티캐스트 통신
     └─ QGroundControl 연결
```

### 3.2 네트워크 토폴로지

```
                    WiFi 네트워크 (192.168.100.0/24)
                           │
        ┌──────────────────┼──────────────────┐
        │                  │                  │
    QGC PC            vim4-A/B/C          중앙 제어 노드
(192.168.100.x)   (192.168.100.x)      (192.168.100.x)
        │                  │                  │
        │            wlan0 (WiFi)             │
        │                  │                  │
        │            eth0 (10.0.0.1)          │
        │                  │                  │
        │              LAN 케이블             │
        │                  │                  │
        │         PX4-A/B/C (10.0.0.x)       │
        │                  │                  │
        └────────── MAVLink UDP ──────────────┘
              (포트 14550, WiFi 네트워크)
```

---

## 4. ROS2 도메인 구성

### 4.1 도메인 ID 분리 (기체 구분)

**기체 A:**
- ROS_DOMAIN_ID = 1 (MAV_SYS_ID = 1과 일치)
- vim4-A의 Micro-ROS Agent가 도메인 1 사용

**기체 B:**
- ROS_DOMAIN_ID = 2 (MAV_SYS_ID = 2와 일치)
- vim4-B의 Micro-ROS Agent가 도메인 2 사용

**기체 C:**
- ROS_DOMAIN_ID = 3 (MAV_SYS_ID = 3과 일치)
- vim4-C의 Micro-ROS Agent가 도메인 3 사용

**설정 방법:**
```bash
# 각 vim4 보드에서
sudo systemctl edit micro-ros-agent
# 다음 내용 추가:
[Service]
Environment="ROS_DOMAIN_ID=1"  # 기체별로 1, 2, 3 (MAV_SYS_ID와 일치)
```

**중요:**
- `ROS_DOMAIN_ID`는 `MAV_SYS_ID`와 **일치**시켜 혼란을 방지합니다
- 기체 A: MAV_SYS_ID=1, ROS_DOMAIN_ID=1
- 기체 B: MAV_SYS_ID=2, ROS_DOMAIN_ID=2
- 기체 C: MAV_SYS_ID=3, ROS_DOMAIN_ID=3

---

## 5. 시스템 서비스 구성

### 5.1 Micro-ROS Agent 서비스

**서비스 파일:** `/etc/systemd/system/micro-ros-agent.service`

**설정:**
- 부팅 시 자동 시작
- 프로세스 종료 시 자동 재시작 (5초 후)
- 로그는 journalctl로 확인

**관리 명령:**
```bash
sudo systemctl status micro-ros-agent
sudo systemctl restart micro-ros-agent
sudo journalctl -u micro-ros-agent -f
```

### 5.2 DHCP 서버 서비스

**서비스 파일:** `/etc/systemd/system/dnsmasq-px4.service`

**설정:**
- 부팅 시 자동 시작
- eth0 인터페이스에서만 동작
- MAC 주소별 고유 IP 할당

**관리 명령:**
```bash
sudo systemctl status dnsmasq-px4
sudo systemctl restart dnsmasq-px4
sudo journalctl -u dnsmasq-px4 -f
```

---

## 6. 설치 및 설정 순서

### 6.1 초기 설치 (각 VIM4 보드에서)

```bash
cd ~/Cluster_Drone
chmod +x install_px4_ros2_complete.sh
./install_px4_ros2_complete.sh
```

**자동으로 설치되는 것:**
1. ROS2 Humble
2. 필수 시스템 패키지
3. Micro-ROS Agent 빌드
4. PX4 ROS2 메시지 패키지 빌드
5. 네트워크 설정 (eth0: 10.0.0.1)
6. DHCP 서버 설정
7. Micro-ROS Agent 서비스 등록

### 6.2 군집 비행 설정 (각 VIM4 보드별로)

**vim4-A (기체 A):**
```bash
# DHCP 서버 설정
sudo tee /etc/dnsmasq.d/px4.conf > /dev/null << EOF
interface=eth0
bind-interfaces
dhcp-range=10.0.0.10,10.0.0.19,12h
dhcp-host=AA:BB:CC:DD:EE:01,10.0.0.11,px4-drone-a
dhcp-option=3,10.0.0.1
dhcp-option=6,8.8.8.8
EOF

# ROS2 도메인 ID 설정
sudo systemctl edit micro-ros-agent
# [Service]
# Environment="ROS_DOMAIN_ID=1"  # 기체 A = 1

sudo systemctl restart dnsmasq-px4
sudo systemctl restart micro-ros-agent
```

**vim4-B (기체 B):**
```bash
# DHCP 서버 설정
sudo tee /etc/dnsmasq.d/px4.conf > /dev/null << EOF
interface=eth0
bind-interfaces
dhcp-range=10.0.0.20,10.0.0.29,12h
dhcp-host=AA:BB:CC:DD:EE:02,10.0.0.21,px4-drone-b
dhcp-option=3,10.0.0.1
dhcp-option=6,8.8.8.8
EOF

# ROS2 도메인 ID 설정
sudo systemctl edit micro-ros-agent
# [Service]
# Environment="ROS_DOMAIN_ID=2"  # 기체 B = 2

sudo systemctl restart dnsmasq-px4
sudo systemctl restart micro-ros-agent
```

**vim4-C (기체 C):**
```bash
# DHCP 서버 설정
sudo tee /etc/dnsmasq.d/px4.conf > /dev/null << EOF
interface=eth0
bind-interfaces
dhcp-range=10.0.0.30,10.0.0.39,12h
dhcp-host=AA:BB:CC:DD:EE:03,10.0.0.31,px4-drone-c
dhcp-option=3,10.0.0.1
dhcp-option=6,8.8.8.8
EOF

# ROS2 도메인 ID 설정
sudo systemctl edit micro-ros-agent
# [Service]
# Environment="ROS_DOMAIN_ID=3"  # 기체 C = 3

sudo systemctl restart dnsmasq-px4
sudo systemctl restart micro-ros-agent
```

### 6.3 PX4 파라미터 설정 (각 기체별로)

**QGroundControl에서:**

1. **기체 A:**
   - Parameters → XRCE-DDS:
     - `UXRCE_DDS_AG_IP = 167772161`
     - `UXRCE_DDS_PRT = 8888`
     - `UXRCE_DDS_DOM_ID = 1`  (MAV_SYS_ID와 일치)
   - Parameters → MAVLink:
     - `MAV_SYS_ID = 1`
     - `MAV_1_CONFIG = TELEM 2 (UDP)`
     - `MAV_1_MODE = 2 (Onboard)`
     - `MAV_1_RATE = 1000000`

2. **기체 B:**
   - Parameters → XRCE-DDS:
     - `UXRCE_DDS_AG_IP = 167772161`
     - `UXRCE_DDS_PRT = 8888`
     - `UXRCE_DDS_DOM_ID = 2`  (MAV_SYS_ID와 일치)
   - Parameters → MAVLink:
     - `MAV_SYS_ID = 2`
     - `MAV_1_CONFIG = TELEM 2 (UDP)`
     - `MAV_1_MODE = 2 (Onboard)`
     - `MAV_1_RATE = 1000000`

3. **기체 C:**
   - Parameters → XRCE-DDS:
     - `UXRCE_DDS_AG_IP = 167772161`
     - `UXRCE_DDS_PRT = 8888`
     - `UXRCE_DDS_DOM_ID = 3`  (MAV_SYS_ID와 일치)
   - Parameters → MAVLink:
     - `MAV_SYS_ID = 3`
     - `MAV_1_CONFIG = TELEM 2 (UDP)`
     - `MAV_1_MODE = 2 (Onboard)`
     - `MAV_1_RATE = 1000000`

4. **각 기체 재부팅**

---

## 7. 확인 및 테스트

### 7.1 연결 상태 확인

**각 VIM4 보드에서:**
```bash
cd ~/Cluster_Drone
./check_px4_connection.sh
```

**확인 항목:**
- eth0 IP 설정 (10.0.0.1)
- DHCP 서버 실행 상태
- Micro-ROS Agent 실행 상태
- 연결된 PX4 IP

### 7.2 ROS2 토픽 확인

**각 VIM4 보드에서:**
```bash
# 기체 A (도메인 1)
export ROS_DOMAIN_ID=1
ros2 topic list

# 기체 B (도메인 2)
export ROS_DOMAIN_ID=2
ros2 topic list

# 기체 C (도메인 3)
export ROS_DOMAIN_ID=3
ros2 topic list
```

### 7.3 QGroundControl 연결 확인

**QGC에서:**
1. Settings → Comm Links → UDP 추가 (포트 14550)
2. 상단에 3대의 기체 아이콘 표시 확인
3. 각 기체를 클릭하여 선택 가능한지 확인

---

## 8. 주요 파일 및 디렉토리

### 8.1 설치 스크립트
```
~/Cluster_Drone/
├── install_px4_ros2_complete.sh    # 완전 자동 설치 스크립트
├── check_px4_connection.sh        # 연결 상태 진단 도구
└── ip_to_decimal.sh                # IP 주소 변환 도구
```

### 8.2 빌드 디렉토리
```
~/Cluster_Drone/
├── micro_ros_ws/                   # Micro-ROS Agent 작업 공간
│   ├── src/micro-ROS-Agent/        # 소스 코드
│   ├── build/                      # 빌드 파일
│   └── install/                    # 설치 파일
└── px4_ros2_ws/                    # PX4 ROS2 메시지 작업 공간
    ├── src/px4_msgs/               # 소스 코드
    ├── build/                      # 빌드 파일
    └── install/                    # 설치 파일
```

### 8.3 설정 파일
```
/etc/
├── netplan/01-eth0-px4.yaml        # 네트워크 설정
├── dnsmasq.d/px4.conf              # DHCP 서버 설정
└── systemd/system/
    ├── micro-ros-agent.service     # Micro-ROS Agent 서비스
    └── dnsmasq-px4.service         # DHCP 서버 서비스
```

---

## 9. 요약 표

### 설치된 패키지 요약

| 카테고리 | 패키지 | 용도 |
|---------|--------|------|
| **시스템** | dnsmasq, netplan | 네트워크 관리 |
| **시스템** | build-essential, cmake, git | 개발 도구 |
| **ROS2** | ros-humble-desktop | ROS2 프레임워크 |
| **ROS2** | ros-humble-rmw-fastrtps-cpp | DDS 구현체 |
| **ROS2** | ros-humble-micro-ros-msgs | Micro-ROS 메시지 |
| **ROS2** | python3-colcon-common-extensions | 빌드 시스템 |
| **PX4 통신** | micro-ros-agent (빌드) | XRCE-DDS 브리지 |
| **PX4 통신** | px4_msgs (빌드) | PX4 메시지 정의 |

### FC 파라미터 요약

| 파라미터 그룹 | 파라미터 | 기체 A | 기체 B | 기체 C |
|-------------|---------|--------|--------|--------|
| **XRCE-DDS** | UXRCE_DDS_AG_IP | 167772161 | 167772161 | 167772161 |
| **XRCE-DDS** | UXRCE_DDS_PRT | 8888 | 8888 | 8888 |
| **XRCE-DDS** | UXRCE_DDS_DOM_ID | **1** | **2** | **3** |
| **MAVLink** | MAV_SYS_ID | **1** | **2** | **3** |
| **MAVLink** | MAV_1_CONFIG | UDP | UDP | UDP |
| **MAVLink** | MAV_1_MODE | Onboard | Onboard | Onboard |
| **네트워크** | IP 주소 | 10.0.0.11 | 10.0.0.21 | 10.0.0.31 |
| **ROS2** | ROS_DOMAIN_ID | **1** | **2** | **3** |

**중요:** `MAV_SYS_ID`와 `ROS_DOMAIN_ID` (및 `UXRCE_DDS_DOM_ID`)를 **일치**시켜 혼란을 방지합니다.

---

## 10. 테스트 환경

### 소프트웨어 버전

- **PX4 펌웨어**: v1.16.0
- **QGroundControl**: v5.0.8 (64-bit)
- **ROS2**: Humble Hawksbill
- **Ubuntu**: 22.04 LTS
- **Micro-ROS Agent**: humble 브랜치 (최신)

### 하드웨어

- **SBC**: Khadas VIM4 (Amlogic A311D2, ARM64)
- **Flight Controller**: PX4 호환 보드 (XRCE-DDS 지원)

---

## 11. 참고 문서

### 기본 문서
- [INSTALLATION.md](INSTALLATION.md) - 상세 설치 가이드
- [SWARM_FLIGHT.md](SWARM_FLIGHT.md) - 군집 비행 설정 가이드
- [NETWORK_ARCHITECTURE.md](NETWORK_ARCHITECTURE.md) - 네트워크 아키텍처 설명
- [ID_MAPPING.md](ID_MAPPING.md) - 기체 ID 매핑 가이드

### QGroundControl 관련
- [QGC_MULTI_VEHICLE.md](QGC_MULTI_VEHICLE.md) - QGroundControl 다중 기체 모니터링
- [QGC_NETWORK_SETUP.md](QGC_NETWORK_SETUP.md) - QGC 네트워크 연결 설정
- [QGC_CONNECTION_GUIDE.md](QGC_CONNECTION_GUIDE.md) - QGC 연결 추가 가이드
- [QGC_DEVELOPMENT.md](QGC_DEVELOPMENT.md) - QGC 개발 및 수정 가이드

### 화재 진압 시스템
- [FIREFIGHTING_SYSTEM.md](FIREFIGHTING_SYSTEM.md) - **화재 진압 시스템 전체 기술 스택**
- [RTK_FORMATION.md](RTK_FORMATION.md) - RTK GPS 기반 편대 비행 및 충돌 방지
- [RTK_PRECISION_HOVERING.md](RTK_PRECISION_HOVERING.md) - RTK GPS 기반 정밀 호버링 ("말뚝처럼 고정")
- [LIDAR_TARGETING.md](LIDAR_TARGETING.md) - LiDAR 기반 타겟 거리 측정 및 타겟팅

---

## 12. 문제 해결 체크리스트

### 설치 확인
- [ ] ROS2 Humble 설치됨 (`ros2 --version`)
- [ ] Micro-ROS Agent 빌드됨 (`ls ~/Cluster_Drone/micro_ros_ws/install`)
- [ ] PX4 메시지 패키지 빌드됨 (`ls ~/Cluster_Drone/px4_ros2_ws/install`)

### 네트워크 확인
- [ ] eth0 IP 설정됨 (`ip addr show eth0`)
- [ ] DHCP 서버 실행 중 (`systemctl status dnsmasq-px4`)
- [ ] Micro-ROS Agent 실행 중 (`systemctl status micro-ros-agent`)

### PX4 연결 확인
- [ ] PX4 IP 할당됨 (`ip neigh show dev eth0`)
- [ ] ROS2 토픽 수신됨 (`ros2 topic list`)
- [ ] QGC에서 기체 표시됨

### 군집 비행 확인
- [ ] 각 기체의 MAV_SYS_ID 고유함
- [ ] 각 SBC의 ROS_DOMAIN_ID 다름
- [ ] 각 기체의 IP 주소 고유함

