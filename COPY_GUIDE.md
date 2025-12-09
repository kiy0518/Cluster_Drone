# 파일 복사 가이드

## 다른 VIM4에 Cluster_Drone 폴더 복사하기

### 방법 1: 소스 코드만 복사 (권장) ⭐

**가장 안전하고 권장하는 방법입니다.**

```bash
# 1. 소스 코드와 스크립트만 복사
# 빌드 디렉토리(build, install, log)는 제외

# 원본에서 복사할 파일/폴더:
# - 모든 .sh 스크립트
# - 모든 .md 문서
# - micro_ros_ws/src/ (소스 코드만)
# - px4_ros2_ws/src/ (소스 코드만)

# 예시 (원본 VIM4에서):
cd ~/Cluster_Drone
tar --exclude='micro_ros_ws/build' \
    --exclude='micro_ros_ws/install' \
    --exclude='micro_ros_ws/log' \
    --exclude='px4_ros2_ws/build' \
    --exclude='px4_ros2_ws/install' \
    --exclude='px4_ros2_ws/log' \
    -czf Cluster_Drone_source.tar.gz .

# 새 VIM4로 전송
scp Cluster_Drone_source.tar.gz user@new-vim4:~

# 새 VIM4에서:
cd ~
tar -xzf Cluster_Drone_source.tar.gz
cd Cluster_Drone
chmod +x install_px4_ros2_complete.sh
./install_px4_ros2_complete.sh
```

**장점:**
- ✅ 경로 문제 없음
- ✅ 깨끗한 빌드
- ✅ 모든 의존성이 올바르게 설정됨

**단점:**
- ⏱️ 빌드 시간이 필요함 (30-60분)

---

### 방법 2: 전체 복사 (빠른 방법)

**같은 아키텍처(ARM64)이고 같은 사용자 이름인 경우에만 사용하세요.**

```bash
# 전체 폴더 복사
scp -r Cluster_Drone user@new-vim4:~/Cluster_Drone

# 또는 tar로 압축해서 복사
cd ~
tar -czf Cluster_Drone.tar.gz Cluster_Drone
scp Cluster_Drone.tar.gz user@new-vim4:~
# 새 VIM4에서:
tar -xzf Cluster_Drone.tar.gz
```

**주의사항:**
- ⚠️ 사용자 이름이 같아야 함 (경로에 사용자 이름이 포함됨)
- ⚠️ 같은 아키텍처(ARM64)여야 함
- ⚠️ 스크립트가 빌드 디렉토리를 확인하고 재빌드할 수 있음

**스크립트 실행 시:**
- 스크립트가 빌드 디렉토리를 확인하고, 문제가 있으면 자동으로 재빌드합니다
- 경로 문제가 발생하면 빌드 디렉토리를 정리하고 재빌드합니다

---

### 방법 3: 최소 복사 (인터넷 연결이 있는 경우)

**인터넷 연결이 있으면 소스 코드도 다시 다운로드할 수 있습니다.**

```bash
# 스크립트와 문서만 복사
# 새 VIM4에서:
mkdir -p ~/Cluster_Drone
# 스크립트 파일들만 복사:
# - install_px4_ros2_complete.sh
# - check_px4_connection.sh
# - ip_to_decimal.sh
# - README.md
# - 등등

# 스크립트 실행 시 소스 코드를 자동으로 다운로드합니다
cd ~/Cluster_Drone
chmod +x install_px4_ros2_complete.sh
./install_px4_ros2_complete.sh
```

**장점:**
- ✅ 최소한의 파일만 복사
- ✅ 항상 최신 소스 코드 사용

---

## 복사 후 확인 사항

### 1. 파일 권한 확인
```bash
cd ~/Cluster_Drone
chmod +x *.sh
```

### 2. 경로 확인
```bash
# 현재 사용자 이름 확인
whoami

# 경로에 사용자 이름이 포함되어 있는지 확인
grep -r "$(whoami)" ~/Cluster_Drone/*.sh
```

### 3. 스크립트 실행
```bash
cd ~/Cluster_Drone
./install_px4_ros2_complete.sh
```

스크립트가 자동으로:
- 빌드 디렉토리 확인
- 문제가 있으면 재빌드
- 모든 설정 자동 완료

---

## 복사 방법 비교

| 방법 | 복사 시간 | 빌드 시간 | 안정성 | 권장도 |
|------|----------|----------|--------|--------|
| 방법 1 (소스만) | 빠름 | 30-60분 | ⭐⭐⭐⭐⭐ | ⭐⭐⭐⭐⭐ |
| 방법 2 (전체) | 느림 | 0-30분 | ⭐⭐⭐ | ⭐⭐⭐ |
| 방법 3 (최소) | 매우 빠름 | 30-60분 | ⭐⭐⭐⭐⭐ | ⭐⭐⭐⭐ |

---

## 문제 해결

### 빌드 디렉토리 경로 문제

**증상:** 스크립트 실행 시 경로 오류 발생

**해결:**
```bash
cd ~/Cluster_Drone
rm -rf micro_ros_ws/build micro_ros_ws/install micro_ros_ws/log
rm -rf px4_ros2_ws/build px4_ros2_ws/install px4_ros2_ws/log
./install_px4_ros2_complete.sh
```

### 사용자 이름이 다른 경우

**증상:** 경로에 다른 사용자 이름이 포함됨

**해결:**
```bash
# 빌드 디렉토리 삭제 후 재빌드
cd ~/Cluster_Drone
rm -rf micro_ros_ws px4_ros2_ws
./install_px4_ros2_complete.sh
```

---

## 권장 워크플로우

**새로 설치한 Ubuntu VIM4의 경우:**

1. **방법 1 사용** (소스 코드만 복사)
2. 스크립트 실행
3. 재부팅
4. 완료!

**이미 설정된 VIM4를 복제하는 경우:**

1. **방법 2 사용** (전체 복사)
2. 스크립트 실행 (자동으로 문제 해결)
3. 재부팅
4. 완료!

