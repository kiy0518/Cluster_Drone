# 설치 가이드

## 새로 설치한 Ubuntu VIM4에서 완전 자동 설치

이 가이드는 새로 설치한 Ubuntu 22.04 VIM4에서 PX4-ROS2 XRCE-DDS 연결을 완전 자동으로 설정하는 방법을 설명합니다.

## 사전 준비

1. **Ubuntu 22.04 LTS 설치**
   - Khadas VIM4에 Ubuntu 22.04 LTS를 설치합니다
   - 기본 사용자 계정을 생성합니다 (예: `khadas`)

2. **인터넷 연결 확인**
   - 설치 중 패키지 다운로드를 위해 인터넷 연결이 필요합니다
   ```bash
   ping -c 3 8.8.8.8
   ```

3. **파일 복사**
   - 이 `Cluster_Drone` 폴더를 VIM4에 복사합니다
   ```bash
   # SCP 사용
   scp -r Cluster_Drone user@vim4-ip:~/Cluster_Drone
   
   # 또는 USB 드라이브 사용
   # USB를 마운트하고 파일 복사
   ```

## 설치 실행

### 1단계: 스크립트 실행 권한 부여

```bash
cd ~/Cluster_Drone
chmod +x install_px4_ros2_complete.sh
```

### 2단계: 설치 스크립트 실행

```bash
./install_px4_ros2_complete.sh
```

**설치 시간:** 약 30-60분 (인터넷 속도에 따라 다름)

**설치 중 자동으로 수행되는 작업:**
1. 시스템 패키지 업데이트
2. ROS2 Humble 설치
3. 필수 개발 도구 설치
4. Micro-ROS Agent 빌드 (시간이 가장 오래 걸림)
5. PX4 ROS2 메시지 패키지 빌드
6. 네트워크 설정 (eth0: 10.0.0.1)
7. DHCP 서버 설정 및 systemd 서비스 등록
8. Micro-ROS Agent systemd 서비스 등록
9. 환경 변수 설정 (.bashrc)

### 3단계: 설치 검증

스크립트가 자동으로 설치를 검증합니다. 마지막에 다음 메시지가 표시됩니다:

```
✓ 모든 구성요소가 정상적으로 설치되었습니다!
```

### 4단계: 시스템 재부팅 (권장)

```bash
sudo reboot
```

재부팅 후 모든 서비스가 자동으로 시작됩니다.

## 재부팅 후 확인

### 1. 서비스 상태 확인

```bash
# DHCP 서버 확인
systemctl status dnsmasq-px4

# Micro-ROS Agent 확인
systemctl status micro-ros-agent

# 둘 다 "active (running)" 상태여야 합니다
```

### 2. 네트워크 확인

```bash
# eth0 IP 확인 (10.0.0.1이어야 함)
ip addr show eth0

# 포트 8888 리스닝 확인
ss -ulnp | grep 8888
```

### 3. 종합 진단

```bash
cd ~/Cluster_Drone
./check_px4_connection.sh
```

이 스크립트는 다음을 확인합니다:
- ✓ eth0 IP 설정
- ✓ DHCP 서버 실행 상태
- ✓ Micro-ROS Agent 실행 상태
- ✓ 연결된 장치 (PX4)

## PX4 설정

### 1. PX4 파라미터 설정

QGroundControl 또는 PX4 콘솔에서 다음 파라미터를 설정합니다:

```
UXRCE_DDS_AG_IP = 167772161  (10.0.0.1의 int32 값)
UXRCE_DDS_PRT = 8888
UXRCE_DDS_DOM_ID = 0
UXRCE_DDS_CFG = Ethernet
```

**IP 주소 변환:**
```bash
cd ~/Cluster_Drone
./ip_to_decimal.sh 10.0.0.1
# 결과: 167772161
```

### 2. PX4 재부팅

파라미터 설정 후 PX4를 재부팅합니다.

### 3. 연결 확인

```bash
# ROS2 토픽 목록 확인
ros2 topic list

# PX4 토픽이 보이면 성공!
# 예: /fmu/out/vehicle_status_v1
```

## 문제 해결

### 설치 실패

**문제:** 스크립트 실행 중 오류 발생

**해결:**
1. 인터넷 연결 확인
2. sudo 권한 확인
3. 로그 확인:
   ```bash
   # Micro-ROS Agent 빌드 로그
   cat ~/Cluster_Drone/micro_ros_ws/log/latest_build/micro_ros_agent/stdout_stderr.log
   ```

### 서비스가 시작되지 않음

**문제:** 재부팅 후 서비스가 실행되지 않음

**해결:**
```bash
# 서비스 상태 확인
sudo systemctl status dnsmasq-px4
sudo systemctl status micro-ros-agent

# 로그 확인
sudo journalctl -u dnsmasq-px4 -n 50
sudo journalctl -u micro-ros-agent -n 50

# 서비스 재시작
sudo systemctl restart dnsmasq-px4
sudo systemctl restart micro-ros-agent
```

### eth0 IP가 설정되지 않음

**문제:** 재부팅 후 eth0에 IP가 없음

**해결:**
```bash
# Netplan 설정 확인
cat /etc/netplan/01-eth0-px4.yaml

# Netplan 적용
sudo netplan apply

# 또는 재부팅
sudo reboot
```

### PX4 토픽이 보이지 않음

**문제:** `ros2 topic list`에 PX4 토픽이 없음

**해결:**
1. 연결 상태 확인:
   ```bash
   cd ~/Cluster_Drone
   ./check_px4_connection.sh
   ```

2. PX4 파라미터 확인:
   - `UXRCE_DDS_AG_IP`가 올바른지 확인
   - PX4 재부팅

3. 네트워크 연결 확인:
   ```bash
   # 연결된 장치 확인
   ip neigh show dev eth0
   
   # DHCP 로그 확인
   sudo journalctl -u dnsmasq-px4 -f
   ```

## 완전 재설치

모든 것을 처음부터 다시 설치하려면:

```bash
# 1. 빌드 디렉토리 삭제
rm -rf ~/Cluster_Drone/micro_ros_ws
rm -rf ~/Cluster_Drone/px4_ros2_ws

# 2. 서비스 중지 및 제거
sudo systemctl stop micro-ros-agent dnsmasq-px4
sudo systemctl disable micro-ros-agent dnsmasq-px4
sudo rm /etc/systemd/system/micro-ros-agent.service
sudo rm /etc/systemd/system/dnsmasq-px4.service
sudo systemctl daemon-reload

# 3. Netplan 설정 제거 (선택사항)
sudo rm /etc/netplan/01-eth0-px4.yaml
sudo netplan apply

# 4. 설치 스크립트 다시 실행
cd ~/Cluster_Drone
./install_px4_ros2_complete.sh
```

## 다음 단계

설치가 완료되면:
1. [README.md](README.md)의 "주요 명령어" 섹션 참고
2. [QUICK_START.md](QUICK_START.md) 참고
3. PX4와 ROS2 개발 시작!

