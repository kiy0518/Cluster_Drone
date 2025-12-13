# 네트워크 아키텍처 설명

## 현재 네트워크 구성

```
                    WiFi 네트워크 (192.168.100.0/24)
                           │
                        wlan0 (WiFi)
        ┌──────────────────┼──────────────────┐
        │                  │                  │
    vim4-A            vim4-B               vim4-B
(192.168.100.11)  (192.168.100.21)      (192.168.100.31)
        │                  │                  │
        │                  │                  │
        │                  │                  │        
        │                  │                  │
eth0 (10.0.0.11/24)  eth0 (10.0.0.21/24) eth0 (10.0.0.31/24)
        │                  │
        │              LAN 케이블
        │                  │
        │                 FC (PX4)
        │            (10.0.0.10-50)
        │
    ROS2 DDS 멀티캐스트 통신
    (같은 ROS_DOMAIN_ID 사용)
```

## 왜 vim4-A에서도 토픽을 읽을 수 있나요?

### ROS2 DDS 멀티캐스트 통신

ROS2는 **DDS (Data Distribution Service)**를 사용하며, **멀티캐스트**를 통해 통신합니다.

**동작 원리:**

1. **PX4 → vim4-B (직접 연결)**
   - PX4는 eth0(10.0.0.1)를 통해 vim4-B의 Micro-ROS Agent에 연결
   - Micro-ROS Agent는 ROS2 토픽으로 변환하여 발행

2. **vim4-B → vim4-A (멀티캐스트)**
   - vim4-B의 Micro-ROS Agent가 토픽을 발행하면
   - ROS2 DDS가 **멀티캐스트**를 사용하여 같은 네트워크 세그먼트의 모든 노드에 전송
   - vim4-A와 vim4-B가 같은 WiFi 네트워크(192.168.100.0/24)에 있으므로
   - vim4-A도 같은 `ROS_DOMAIN_ID`를 사용하면 토픽을 받을 수 있음

### 핵심 포인트

- **PX4 연결**: eth0 (10.0.0.1) - 직접 연결, 유니캐스트
- **ROS2 통신**: WiFi (192.168.100.0/24) - 멀티캐스트
- **같은 ROS_DOMAIN_ID**: 모든 노드가 같은 도메인 ID를 사용해야 함

## 네트워크 인터페이스 역할

### vim4-B 보드

```
eth0 (10.0.0.1/24)
  └─ PX4와 직접 연결 (유니캐스트)
     └─ Micro-ROS Agent가 PX4 데이터 수신

wlan0 (192.168.100.166/24)
  └─ WiFi 네트워크 연결
     └─ ROS2 DDS 멀티캐스트 통신
        └─ vim4-A와 다른 노드들이 토픽 수신
```

### vim4-A 보드

```
wlan0 (192.168.100.x/24)
  └─ WiFi 네트워크 연결
     └─ ROS2 DDS 멀티캐스트로 토픽 수신
        └─ vim4-B가 발행한 PX4 토픽 수신 가능
```

## ROS2 DDS 도메인 ID

모든 노드가 **같은 ROS_DOMAIN_ID**를 사용해야 통신할 수 있습니다.

```bash
# vim4-B (Micro-ROS Agent 실행 중)
export ROS_DOMAIN_ID=1  # 기본값 (단일 기체 사용 시)

# vim4-A (토픽 수신)
export ROS_DOMAIN_ID=1  # 같은 도메인 ID 사용 (같은 WiFi 네트워크)

# 이제 vim4-A에서도 토픽을 볼 수 있음
ros2 topic list
```

**참고:** 단일 기체 사용 시 기본값은 1입니다. 군집 비행 시 각 기체별로 1, 2, 3을 사용합니다.

## 실제 확인 방법

### vim4-B에서 확인

```bash
# PX4 연결 확인
ip addr show eth0
ip neigh show dev eth0

# ROS2 토픽 확인
ros2 topic list

# Micro-ROS Agent 상태
systemctl status micro-ros-agent
```

### vim4-A에서 확인

```bash
# WiFi 연결 확인
ip addr show wlan0

# ROS2 도메인 ID 확인
echo $ROS_DOMAIN_ID

# ROS2 토픽 확인 (vim4-B가 발행한 토픽이 보여야 함)
ros2 topic list

# 토픽 수신 테스트
ros2 topic echo /fmu/out/vehicle_status_v1
```

## 네트워크 격리

만약 vim4-A가 토픽을 받지 못하게 하려면:

### 방법 1: 다른 ROS_DOMAIN_ID 사용

```bash
# vim4-A에서
export ROS_DOMAIN_ID=1  # 다른 도메인 ID 사용
```

### 방법 2: 네트워크 분리

- vim4-A를 다른 WiFi 네트워크에 연결
- 또는 vim4-B의 WiFi를 끄고 eth0만 사용

## 멀티캐스트 확인

ROS2 DDS가 멀티캐스트를 사용하는지 확인:

```bash
# 멀티캐스트 그룹 확인
netstat -gn | grep 239.255  # FastRTPS 기본 멀티캐스트 주소

# 또는
ss -ulnp | grep 239.255
```

## 요약

**질문: 왜 vim4-A에서도 토픽을 읽을 수 있나요?**

**답변:**
1. PX4는 eth0를 통해 vim4-B에 직접 연결 (10.0.0.1)
2. vim4-B의 Micro-ROS Agent가 ROS2 토픽으로 변환
3. ROS2 DDS가 멀티캐스트를 사용하여 같은 WiFi 네트워크의 모든 노드에 전송
4. vim4-A가 같은 WiFi 네트워크와 같은 ROS_DOMAIN_ID를 사용하면 토픽 수신 가능

**이것은 ROS2 DDS의 정상적인 동작입니다!** 🎉

---

## 군집 비행 (Swarm Flight)

여러 PX4 기체를 사용할 때는 다음을 고려해야 합니다:

1. **각 기체는 고유한 IP 주소 필요** (네트워크 충돌 방지)
2. **각 기체는 다른 ROS_DOMAIN_ID 사용** (토픽 구분)

자세한 내용은 [SWARM_FLIGHT.md](SWARM_FLIGHT.md)를 참고하세요.

