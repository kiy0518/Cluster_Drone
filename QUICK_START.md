# 빠른 시작 가이드

## 새로 설치한 Ubuntu VIM4에서 완전 자동 설치

### 1단계: 파일 복사
```bash
# 방법 A: 전체 복사 (빠름, 같은 사용자 이름인 경우)
scp -r Cluster_Drone user@new-vim4-ip:~/Cluster_Drone

# 방법 B: 소스 코드만 복사 (권장, 더 안전)
# 자세한 내용은 COPY_GUIDE.md 참고

# 방법 C: GITHUB에서 다운로드
git clone https://github.com/kiy0518/Cluster_Drone.git

# 또는 USB/네트워크 드라이브로 복사
```

**참고:** 복사 방법 선택 가이드는 [COPY_GUIDE.md](COPY_GUIDE.md)를 참고하세요.

### 2단계: 설치 (단 한 번의 명령!)
```bash
cd ~/Cluster_Drone
chmod +x install_px4_ros2_complete.sh
./install_px4_ros2_complete.sh
```

**이 스크립트 하나로 모든 것이 자동 설정됩니다!**
- ROS2 Humble 설치
- Micro-ROS Agent 빌드 및 서비스 설정
- PX4 ROS2 메시지 패키지 빌드
- 네트워크 설정 (eth0: 10.0.0.1)
- DHCP 서버 설정
- 모든 systemd 서비스 등록

**설치 시간:** 약 30-60분

### 3단계: 시스템 재부팅 (권장)
```bash
sudo reboot
```

재부팅 후 모든 서비스가 자동으로 시작됩니다.

### 4단계: 재부팅 후 확인
```bash
# 연결 상태 종합 확인
cd ~/Cluster_Drone
./check_px4_connection.sh

# 서비스 상태 확인
systemctl status dnsmasq-px4
systemctl status micro-ros-agent
```

### 5단계: PX4 설정
QGroundControl 또는 PX4 콘솔에서:
- `UXRCE_DDS_AG_IP` = `167772161` (10.0.0.1)
- `UXRCE_DDS_PRT` = `8888`
- `UXRCE_DDS_DOM_ID` = `1` (단일 기체 기본값, 군집 비행 시 기체별로 1, 2, 3)

**IP 주소 변환:**
```bash
cd ~/Cluster_Drone
./ip_to_decimal.sh 10.0.0.1
```

### 6단계: PX4 재부팅 및 연결 확인
```bash
# PX4 재부팅 후
ros2 topic list

# PX4 토픽이 보이면 성공!
```

## 전원 켜는 순서

1. **VIM4 먼저 켜기** (DHCP 서버와 Agent 준비)
2. **PX4 나중에 켜기** (IP 받고 연결)

## 확인 명령어

```bash
# 연결 상태 확인
cd ~/Cluster_Drone
./check_px4_connection.sh

# ROS2 토픽 확인
ros2 topic list

# 서비스 상태
systemctl status dnsmasq-px4
systemctl status micro-ros-agent
```

## 문제 해결

### 빠른 진단
```bash
cd ~/Cluster_Drone
./check_px4_connection.sh
```

### 로그 확인
```bash
# DHCP 서버 로그
sudo journalctl -u dnsmasq-px4 -f

# Micro-ROS Agent 로그
sudo journalctl -u micro-ros-agent -f
```

### 서비스 재시작
```bash
sudo systemctl restart dnsmasq-px4
sudo systemctl restart micro-ros-agent
```

## 자세한 내용

- **완전한 설치 가이드**: [INSTALLATION.md](INSTALLATION.md)
- **상세 사용 설명서**: [README.md](README.md)

 