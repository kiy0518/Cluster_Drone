# 설치 가이드

다른 VIM4에 이 프로젝트를 설치하는 방법

## 1. 파일 복사

### 방법 1: SCP 사용
```bash
# 다른 컴퓨터에서
scp -r Cluster_Drone user@new-vim4-ip:~/Cluster_Drone
```

### 방법 2: USB/네트워크 드라이브
```bash
# USB나 네트워크 드라이브로 Cluster_Drone 폴더 복사
cp -r /path/to/Cluster_Drone ~/Cluster_Drone
```

### 방법 3: Git (선택사항)
```bash
# Git 저장소가 있다면
git clone <repository-url> ~/Cluster_Drone
```

## 2. 설치

### 첫 설치 (ROS2, Micro-ROS Agent, PX4 메시지 패키지)

```bash
cd ~/Cluster_Drone
chmod +x install_px4_ros2_complete.sh
./install_px4_ros2_complete.sh
```

이 스크립트는 다음을 설치합니다:
- ROS2 Humble
- Micro-ROS Agent (소스 빌드)
- PX4 ROS2 메시지 패키지
- 필요한 의존성 패키지

**소요 시간**: 약 30분~1시간 (인터넷 속도에 따라 다름)

### 완전 자동 설정 (네트워크, DHCP, Agent 서비스)

```bash
cd ~/Cluster_Drone
chmod +x setup_complete.sh
./setup_complete.sh
```

이 스크립트는 다음을 설정합니다:
- eth0 네트워크 설정 (10.0.0.1)
- dnsmasq DHCP 서버 설정 및 자동 시작
- Micro-ROS Agent systemd 서비스 설정 및 자동 시작
- 환경 변수 설정 (.bashrc)

**중요**: 이 스크립트를 실행하면 재시작 후에도 자동으로 작동합니다!

## 3. 확인

설치 및 설정이 완료되면:

```bash
# 네트워크 확인
ip addr show eth0
# 출력에 10.0.0.1이 있어야 함

# 서비스 확인
systemctl status dnsmasq-px4
systemctl status micro-ros-agent

# 포트 확인
ss -ulnp | grep 8888
# Micro-ROS Agent가 포트 8888에서 리스닝 중이어야 함

# ROS2 환경 확인
source ~/.bashrc
ros2 topic list
```

## 4. PX4 설정

QGroundControl 또는 PX4 콘솔에서 다음 파라미터 설정:

```
UXRCE_DDS_AG_IP = 167772161  (10.0.0.1)
UXRCE_DDS_PRT = 8888
UXRCE_DDS_DOM_ID = 0
UXRCE_DDS_CFG = Ethernet
```

**IP 변환:**
```bash
cd ~/Cluster_Drone
./ip_to_decimal.sh 10.0.0.1
```

## 5. 테스트

1. **VIM4 재시작:**
   ```bash
   sudo reboot
   ```

2. **재시작 후 확인:**
   ```bash
   # 서비스 자동 시작 확인
   systemctl status dnsmasq-px4
   systemctl status micro-ros-agent
   
   # 네트워크 확인
   ip addr show eth0
   ```

3. **PX4 연결:**
   - PX4를 이더넷 케이블로 연결
   - PX4 재부팅
   - 연결 확인:
     ```bash
     cd ~/Cluster_Drone
     ./check_px4_connection.sh
     ros2 topic list
     ```

## 문제 해결

### 설치 중 오류 발생

```bash
# 로그 확인
tail -f /tmp/install_*.log

# 부분적으로 설치된 경우 재시도
./install_px4_ros2_complete.sh
```

### 설정 스크립트 오류

```bash
# 서비스 로그 확인
sudo journalctl -u dnsmasq-px4 -n 50
sudo journalctl -u micro-ros-agent -n 50

# 수동 설정
cd ~/Cluster_Drone
./setup_complete.sh
```

### 재시작 후 작동 안 함

```bash
# 서비스 상태 확인
systemctl status dnsmasq-px4
systemctl status micro-ros-agent

# 자동 시작 확인
systemctl is-enabled dnsmasq-px4
systemctl is-enabled micro-ros-agent

# 수동 시작
sudo systemctl start dnsmasq-px4
sudo systemctl start micro-ros-agent
```

## 완료

설치 및 설정이 완료되면:
- 재시작 후 자동으로 모든 서비스가 시작됩니다
- PX4를 연결하고 재부팅하면 자동으로 토픽이 나타납니다
- `ros2 topic list`로 PX4 토픽을 확인할 수 있습니다

