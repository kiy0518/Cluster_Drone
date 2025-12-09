# 군집 비행 (Swarm Flight) 설정 가이드

## 문제 상황

여러 PX4 기체를 사용할 때:
- ✅ 모든 기체의 데이터를 읽을 수 있음 (ROS2 DDS 멀티캐스트)
- ❌ 기체를 구분할 수 없음 (모두 같은 토픽 이름 사용)
- ❌ IP 주소가 같으면 네트워크 충돌 발생

## 현재 문제점

### 1. 토픽 이름 충돌

모든 PX4 기체가 같은 토픽 이름을 사용합니다:
```
/fmu/out/vehicle_status_v1
/fmu/out/vehicle_odometry
/fmu/out/vehicle_local_position
...
```

여러 기체가 있으면 어떤 기체의 데이터인지 구분할 수 없습니다.

### 2. IP 주소 충돌

만약 여러 PX4 기체가 모두 같은 IP(예: 10.0.0.20)를 사용하면:
- 네트워크 충돌 발생
- 통신 불안정
- 일부 기체만 연결됨

## 해결 방법

### 방법 1: 각 기체에 고유한 ROS_DOMAIN_ID 할당 (권장) ⭐

각 PX4 기체와 연결된 SBC에 다른 ROS_DOMAIN_ID를 할당합니다.

```
기체 A: ROS_DOMAIN_ID=1  (MAV_SYS_ID=1과 일치)
기체 B: ROS_DOMAIN_ID=2  (MAV_SYS_ID=2와 일치)
기체 C: ROS_DOMAIN_ID=3  (MAV_SYS_ID=3과 일치)
```

**중요:** `ROS_DOMAIN_ID`는 `MAV_SYS_ID`와 **일치**시켜 혼란을 방지합니다.

**장점:**
- ✅ 각 기체의 토픽이 완전히 분리됨
- ✅ 네트워크 충돌 없음
- ✅ 설정이 간단함

**단점:**
- ❌ 다른 도메인의 토픽은 보이지 않음
- ❌ 중앙 제어 노드가 모든 도메인을 구독해야 함

**설정 방법:**

#### vim4-A (기체 A 연결)
```bash
# .bashrc 또는 환경 설정
export ROS_DOMAIN_ID=1  # MAV_SYS_ID=1과 일치

# Micro-ROS Agent 서비스 설정
sudo systemctl edit micro-ros-agent
# 다음 내용 추가:
[Service]
Environment="ROS_DOMAIN_ID=1"
```

#### vim4-B (기체 B 연결)
```bash
export ROS_DOMAIN_ID=2  # MAV_SYS_ID=2와 일치
# Micro-ROS Agent 서비스에도 동일하게 설정
```

#### vim4-C (기체 C 연결)
```bash
export ROS_DOMAIN_ID=3  # MAV_SYS_ID=3과 일치
# Micro-ROS Agent 서비스에도 동일하게 설정
```

#### 중앙 제어 노드 (모든 기체 데이터 수신)
```bash
# 여러 도메인을 구독하려면 별도 노드 필요
# 또는 각 도메인별로 별도 터미널/프로세스 사용
```

---

### 방법 2: 각 기체에 고유한 IP 주소 할당 (필수)

각 PX4 기체는 **반드시 고유한 IP 주소**를 가져야 합니다.

#### DHCP 서버 설정 (dnsmasq)

각 SBC의 DHCP 서버에서 MAC 주소별로 고유 IP를 할당:

**vim4-A의 /etc/dnsmasq.d/px4.conf:**
```
interface=eth0
bind-interfaces
dhcp-range=10.0.0.10,10.0.0.19,12h
dhcp-host=AA:BB:CC:DD:EE:01,10.0.0.11,px4-drone-a  # 기체 A MAC 주소
dhcp-option=3,10.0.0.1
dhcp-option=6,8.8.8.8
```

**vim4-B의 /etc/dnsmasq.d/px4.conf:**
```
interface=eth0
bind-interfaces
dhcp-range=10.0.0.20,10.0.0.29,12h
dhcp-host=AA:BB:CC:DD:EE:02,10.0.0.21,px4-drone-b  # 기체 B MAC 주소
dhcp-option=3,10.0.0.1
dhcp-option=6,8.8.8.8
```

**vim4-C의 /etc/dnsmasq.d/px4.conf:**
```
interface=eth0
bind-interfaces
dhcp-range=10.0.0.30,10.0.0.39,12h
dhcp-host=AA:BB:CC:DD:EE:03,10.0.0.31,px4-drone-c  # 기체 C MAC 주소
dhcp-option=3,10.0.0.1
dhcp-option=6,8.8.8.8
```

**PX4 MAC 주소 확인:**
```bash
# SBC에서 연결된 PX4의 MAC 주소 확인
ip neigh show dev eth0
```

---

### 방법 3: ROS2 네임스페이스 사용 (고급)

PX4의 토픽을 네임스페이스로 감싸서 구분합니다.

**장점:**
- ✅ 모든 기체를 같은 도메인에서 사용 가능
- ✅ 토픽 이름으로 기체 구분 가능

**단점:**
- ❌ Micro-ROS Agent 수정 필요
- ❌ 복잡한 설정

**예시:**
```
/drone_a/fmu/out/vehicle_status_v1
/drone_b/fmu/out/vehicle_status_v1
/drone_c/fmu/out/vehicle_status_v1
```

이 방법은 Micro-ROS Agent를 수정하거나 remap을 사용해야 합니다.

---

### 방법 4: 토픽 remap 사용 (중앙 제어 노드)

중앙 제어 노드에서 토픽을 remap하여 구분합니다.

```python
# Python 예시
import rclpy
from rclpy.node import Node
from px4_msgs.msg import VehicleStatus

class SwarmController(Node):
    def __init__(self):
        super().__init__('swarm_controller')
        
        # 각 기체의 토픽을 다른 이름으로 구독
        self.drone_a_status = self.create_subscription(
            VehicleStatus,
            '/fmu/out/vehicle_status_v1',  # 도메인 0
            self.drone_a_callback,
            10
        )
        
        # 도메인 1의 토픽은 별도 프로세스나 노드에서 처리
```

---

## 권장 구성

### 구성 1: 도메인 분리 (간단, 권장)

```
기체 A: vim4-A (eth0: 10.0.0.1) → PX4-A (10.0.0.11) → ROS_DOMAIN_ID=1 (MAV_SYS_ID=1과 일치)
기체 B: vim4-B (eth0: 10.0.0.1) → PX4-B (10.0.0.21) → ROS_DOMAIN_ID=2 (MAV_SYS_ID=2와 일치)
기체 C: vim4-C (eth0: 10.0.0.1) → PX4-C (10.0.0.31) → ROS_DOMAIN_ID=3 (MAV_SYS_ID=3과 일치)

중앙 제어: 모든 도메인을 구독하는 노드
```

**중요:** `ROS_DOMAIN_ID`는 `MAV_SYS_ID`와 **일치**시켜 혼란을 방지합니다.

**설정 스크립트 예시:**

각 vim4 보드에서:
```bash
# vim4-A (기체 A)
sudo systemctl edit micro-ros-agent
# [Service]
# Environment="ROS_DOMAIN_ID=1"  # MAV_SYS_ID=1과 일치

# vim4-B (기체 B)
sudo systemctl edit micro-ros-agent
# [Service]
# Environment="ROS_DOMAIN_ID=2"  # MAV_SYS_ID=2와 일치

# vim4-C (기체 C)
sudo systemctl edit micro-ros-agent
# [Service]
# Environment="ROS_DOMAIN_ID=3"  # MAV_SYS_ID=3과 일치
```

---

## 실제 설정 예시

### 1. 각 SBC의 eth0 IP 설정

각 SBC는 같은 IP(10.0.0.1)를 사용해도 됩니다 (다른 네트워크 인터페이스이므로).

```bash
# 모든 vim4 보드에서 동일
sudo netplan apply  # eth0: 10.0.0.1/24
```

### 2. DHCP 서버 설정 (각 SBC별로 다름)

**vim4-A:**
```bash
sudo tee /etc/dnsmasq.d/px4.conf > /dev/null << EOF
interface=eth0
bind-interfaces
dhcp-range=10.0.0.10,10.0.0.19,12h
dhcp-host=AA:BB:CC:DD:EE:01,10.0.0.11,px4-drone-a
dhcp-option=3,10.0.0.1
dhcp-option=6,8.8.8.8
EOF
```

**vim4-B:**
```bash
sudo tee /etc/dnsmasq.d/px4.conf > /dev/null << EOF
interface=eth0
bind-interfaces
dhcp-range=10.0.0.20,10.0.0.29,12h
dhcp-host=AA:BB:CC:DD:EE:02,10.0.0.21,px4-drone-b
dhcp-option=3,10.0.0.1
dhcp-option=6,8.8.8.8
EOF
```

### 3. Micro-ROS Agent 서비스 설정

**vim4-A (기체 A):**
```bash
sudo systemctl edit micro-ros-agent
# 다음 내용 추가:
[Service]
Environment="ROS_DOMAIN_ID=1"  # MAV_SYS_ID=1과 일치
```

**vim4-B (기체 B):**
```bash
sudo systemctl edit micro-ros-agent
# 다음 내용 추가:
[Service]
Environment="ROS_DOMAIN_ID=2"  # MAV_SYS_ID=2와 일치
```

### 4. 중앙 제어 노드 (모든 기체 데이터 수신)

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from px4_msgs.msg import VehicleStatus
import os

class SwarmController(Node):
    def __init__(self):
        super().__init__('swarm_controller')
        
        # 도메인 1 (기체 A)
        os.environ['ROS_DOMAIN_ID'] = '1'
        # 도메인 1의 토픽 구독 (별도 프로세스 필요)
        
    def drone_a_callback(self, msg):
        self.get_logger().info(f'Drone A: {msg}')
        
    def drone_b_callback(self, msg):
        self.get_logger().info(f'Drone B: {msg}')

# 실제로는 각 도메인별로 별도 프로세스나 노드 필요
```

---

## 확인 방법

### 각 기체별 토픽 확인

```bash
# vim4-A에서 (도메인 1, 기체 A)
export ROS_DOMAIN_ID=1
ros2 topic list

# vim4-B에서 (도메인 2, 기체 B)
export ROS_DOMAIN_ID=2
ros2 topic list

# vim4-C에서 (도메인 3, 기체 C)
export ROS_DOMAIN_ID=3
ros2 topic list

# 중앙 제어 노드에서
# 각 도메인별로 별도 터미널에서 확인
```

### IP 주소 확인

```bash
# 각 SBC에서 연결된 PX4 IP 확인
ip neigh show dev eth0

# 각 PX4의 IP가 고유한지 확인
# vim4-A: 10.0.0.11
# vim4-B: 10.0.0.21
# vim4-C: 10.0.0.31
```

---

## 요약

**질문: 여러 기체를 어떻게 구분하나요?**

**답변:**

1. **IP 주소**: 각 PX4 기체는 반드시 고유한 IP 주소를 가져야 함 (DHCP로 자동 할당)
2. **ROS_DOMAIN_ID**: 각 기체를 다른 도메인 ID로 분리 (권장)
3. **토픽 구분**: 도메인 ID로 자동 분리되므로 같은 토픽 이름 사용 가능

**핵심:**
- ✅ 각 PX4는 고유 IP (10.0.0.11, 10.0.0.21, 10.0.0.31...)
- ✅ 각 SBC는 다른 ROS_DOMAIN_ID (1, 2, 3, MAV_SYS_ID와 일치)
- ✅ MAV_SYS_ID와 ROS_DOMAIN_ID를 일치시켜 혼란 방지
- ✅ 중앙 제어 노드는 모든 도메인을 구독

**주의:**
- ❌ 같은 IP를 사용하면 네트워크 충돌
- ❌ 같은 도메인 ID를 사용하면 토픽 구분 불가
- ✅ MAV_SYS_ID와 ROS_DOMAIN_ID를 일치시키면 기체 식별이 쉬워짐

---

## QGroundControl 다중 기체 모니터링

3대의 기체를 QGC에서 동시에 모니터링하고 카메라 스트리밍을 받을 수 있습니다.

**필수 설정:**
1. 각 기체에 고유한 `MAV_SYS_ID` 설정 (1, 2, 3...)
2. QGC 멀티 비히클 모드 활성화
3. 각 기체를 네트워크로 연결

자세한 내용은 [QGC_MULTI_VEHICLE.md](QGC_MULTI_VEHICLE.md)를 참고하세요.

