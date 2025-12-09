# 기체 ID 매핑 가이드

## ID 통일 규칙

**중요:** 혼란을 방지하기 위해 모든 ID를 **일치**시킵니다.

## 기체별 ID 매핑

| 기체 | MAV_SYS_ID | ROS_DOMAIN_ID | UXRCE_DDS_DOM_ID | IP 주소 |
|------|-----------|---------------|------------------|---------|
| **기체 A** | **1** | **1** | **1** | 10.0.0.11 |
| **기체 B** | **2** | **2** | **2** | 10.0.0.21 |
| **기체 C** | **3** | **3** | **3** | 10.0.0.31 |

**규칙:**
- 모든 ID를 **1, 2, 3**으로 통일
- 기체 A = 모든 ID가 1
- 기체 B = 모든 ID가 2
- 기체 C = 모든 ID가 3

## 각 ID의 역할

### MAV_SYS_ID
- **용도:** QGroundControl에서 기체를 구분
- **범위:** 1-255 (0은 브로드캐스트용)
- **설정 위치:** PX4 Parameters → MAVLink

### ROS_DOMAIN_ID
- **용도:** ROS2 DDS 도메인 분리 (기체별 토픽 분리)
- **범위:** 0-232 (일반적으로 0-101 사용)
- **설정 위치:** SBC의 Micro-ROS Agent 서비스

### UXRCE_DDS_DOM_ID
- **용도:** PX4의 XRCE-DDS 도메인 설정
- **범위:** 0-232
- **설정 위치:** PX4 Parameters → XRCE-DDS
- **주의:** 연결된 SBC의 ROS_DOMAIN_ID와 **일치**해야 함

## 설정 예시

### 기체 A 설정

**PX4 파라미터:**
```
MAV_SYS_ID = 1
UXRCE_DDS_DOM_ID = 1
```

**SBC (vim4-A) 설정:**
```bash
sudo systemctl edit micro-ros-agent
# [Service]
# Environment="ROS_DOMAIN_ID=1"
```

### 기체 B 설정

**PX4 파라미터:**
```
MAV_SYS_ID = 2
UXRCE_DDS_DOM_ID = 2
```

**SBC (vim4-B) 설정:**
```bash
sudo systemctl edit micro-ros-agent
# [Service]
# Environment="ROS_DOMAIN_ID=2"
```

### 기체 C 설정

**PX4 파라미터:**
```
MAV_SYS_ID = 3
UXRCE_DDS_DOM_ID = 3
```

**SBC (vim4-C) 설정:**
```bash
sudo systemctl edit micro-ros-agent
# [Service]
# Environment="ROS_DOMAIN_ID=3"
```

## 왜 통일하는가?

### 이전 방식 (혼란스러움)
- MAV_SYS_ID: 1, 2, 3
- ROS_DOMAIN_ID: 0, 1, 2
- ❌ 기체 A의 MAV_SYS_ID=1인데 ROS_DOMAIN_ID=0
- ❌ 기체 B의 MAV_SYS_ID=2인데 ROS_DOMAIN_ID=1
- ❌ 혼란 발생!

### 통일된 방식 (명확함)
- MAV_SYS_ID: 1, 2, 3
- ROS_DOMAIN_ID: 1, 2, 3
- ✅ 기체 A = 모든 ID가 1
- ✅ 기체 B = 모든 ID가 2
- ✅ 기체 C = 모든 ID가 3
- ✅ 명확하고 기억하기 쉬움!

## 확인 방법

### 각 기체의 ID 확인

**PX4에서:**
```bash
# QGroundControl에서
Parameters → MAVLink → MAV_SYS_ID
Parameters → XRCE-DDS → UXRCE_DDS_DOM_ID
```

**SBC에서:**
```bash
# ROS2 도메인 ID 확인
echo $ROS_DOMAIN_ID

# Micro-ROS Agent 서비스 확인
sudo systemctl show micro-ros-agent | grep ROS_DOMAIN_ID
```

## 요약

**핵심 원칙:**
- ✅ 모든 ID를 **일치**시킴
- ✅ 기체 A = 1, 기체 B = 2, 기체 C = 3
- ✅ 혼란 방지 및 설정 단순화

**기억하기 쉬운 규칙:**
- 기체 번호 = 모든 ID 번호
- 기체 A (첫 번째) = ID 1
- 기체 B (두 번째) = ID 2
- 기체 C (세 번째) = ID 3

