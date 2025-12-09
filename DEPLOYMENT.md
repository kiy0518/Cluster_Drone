# 배포 가이드

다른 VIM4에 이 프로젝트를 배포하는 방법

## 배포 파일 목록

다음 파일들을 다른 VIM4에 복사하면 됩니다:

### 필수 파일
- `install_px4_ros2_complete.sh` - 첫 설치 스크립트
- `setup_complete.sh` - 완전 자동 설정 스크립트
- `check_px4_connection.sh` - 진단 도구
- `ip_to_decimal.sh` - IP 변환 도구
- `load_environment.sh` - 환경 로드 스크립트
- `dnsmasq-px4.service` - DHCP 서버 systemd 서비스
- `micro-ros-agent.service` - Micro-ROS Agent systemd 서비스
- `start_micro_ros_agent_wrapper.sh` - Agent wrapper 스크립트

### 문서 (선택사항)
- `README.md` - 메인 문서
- `INSTALL_GUIDE.md` - 설치 가이드
- `QUICK_START.md` - 빠른 시작

### 작업 공간 (이미 설치된 경우)
- `micro_ros_ws/` - Micro-ROS Agent 작업 공간
- `px4_ros2_ws/` - PX4 ROS2 메시지 패키지 작업 공간

## 배포 방법

### 방법 1: 전체 폴더 복사 (권장)

```bash
# 다른 컴퓨터에서
scp -r Cluster_Drone user@new-vim4-ip:~/Cluster_Drone

# 또는 tar로 압축 후 복사
tar czf Cluster_Drone.tar.gz Cluster_Drone
scp Cluster_Drone.tar.gz user@new-vim4-ip:~/
# VIM4에서
tar xzf Cluster_Drone.tar.gz
```

### 방법 2: 필수 파일만 복사

```bash
# 필수 파일만 복사
scp install_px4_ros2_complete.sh setup_complete.sh check_px4_connection.sh \
    ip_to_decimal.sh load_environment.sh dnsmasq-px4.service \
    micro-ros-agent.service start_micro_ros_agent_wrapper.sh \
    user@new-vim4-ip:~/Cluster_Drone/
```

## 설치 순서

1. **파일 복사**
   ```bash
   scp -r Cluster_Drone user@new-vim4-ip:~/Cluster_Drone
   ```

2. **SSH 접속**
   ```bash
   ssh user@new-vim4-ip
   ```

3. **첫 설치**
   ```bash
   cd ~/Cluster_Drone
   chmod +x *.sh
   ./install_px4_ros2_complete.sh
   ```
   (약 30분~1시간 소요)

4. **완전 자동 설정**
   ```bash
   ./setup_complete.sh
   ```

5. **재시작**
   ```bash
   sudo reboot
   ```

6. **확인**
   ```bash
   systemctl status dnsmasq-px4
   systemctl status micro-ros-agent
   ip addr show eth0
   ```

## 주의사항

- 작업 공간(`micro_ros_ws/`, `px4_ros2_ws/`)은 복사하지 않아도 됩니다
- `install_px4_ros2_complete.sh`가 자동으로 빌드합니다
- 하지만 이미 빌드된 작업 공간이 있다면 복사하면 시간을 절약할 수 있습니다

## 빠른 배포 (이미 빌드된 작업 공간 포함)

```bash
# 전체 폴더 압축
tar czf Cluster_Drone_complete.tar.gz Cluster_Drone

# 복사
scp Cluster_Drone_complete.tar.gz user@new-vim4-ip:~/

# VIM4에서
tar xzf Cluster_Drone_complete.tar.gz
cd ~/Cluster_Drone
./setup_complete.sh
```

이 경우 `install_px4_ros2_complete.sh`를 실행할 필요가 없습니다.

