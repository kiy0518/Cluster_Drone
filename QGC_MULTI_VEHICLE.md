# QGroundControl 다중 기체 모니터링 가이드

## 개요

QGroundControl(QGC)은 **여러 기체를 동시에 모니터링**할 수 있습니다. 3대의 드론을 동시에 모니터링하고 카메라 스트리밍을 받을 수 있습니다.

**테스트 환경:**
- **QGroundControl**: v5.0.8 (64-bit)
- **PX4 펌웨어**: v1.16.0

## QGroundControl 다중 기체 지원

### 기본 기능

- ✅ **다중 기체 동시 연결**: 여러 PX4 기체를 동시에 연결
- ✅ **위치 모니터링**: 각 기체의 위치를 지도에 표시
- ✅ **상태 모니터링**: 각 기체의 배터리, 모드, 상태 등
- ✅ **카메라 스트리밍**: 각 기체의 카메라 영상 수신 (설정 필요)

## 설정 방법

### 1. 각 PX4 기체에 고유한 MAVLink 시스템 ID 설정

각 PX4 기체는 **고유한 MAV_SYS_ID**를 가져야 합니다.

**QGroundControl에서 설정:**
1. 각 기체에 연결
2. Parameters → MAV_SYS_ID 설정:
   - 기체 A: `MAV_SYS_ID = 1`
   - 기체 B: `MAV_SYS_ID = 2`
   - 기체 C: `MAV_SYS_ID = 3`

**또는 PX4 파라미터 파일에서:**
```bash
# 기체 A
MAV_SYS_ID = 1

# 기체 B
MAV_SYS_ID = 2

# 기체 C
MAV_SYS_ID = 3
```

### 2. QGroundControl 멀티 비히클 모드 활성화

**QGroundControl 설정:**
1. QGC 실행
2. Settings → General → Multi-Vehicle
3. "Enable Multi-Vehicle" 체크

또는 QGC 시작 시:
```bash
qgroundcontrol --multi-vehicle
```

### 3. 네트워크 연결 설정

각 기체를 QGC에 연결하는 방법:

#### 방법 A: UDP 브로드캐스트 (같은 네트워크)

**PX4 파라미터 설정:**
```
MAV_1_CONFIG = TELEM 2 (UDP)
MAV_1_MODE = 2 (Onboard)
MAV_1_RATE = 1000000
MAV_1_FORWARD = 1
```

**QGC 연결:**
- Add Vehicle → UDP
- 포트: 14550 (기본)
- 모든 기체가 같은 네트워크에 있으면 자동으로 감지됨

#### 방법 B: TCP/IP (직접 연결)

각 기체를 개별적으로 연결:
- 기체 A: TCP → 10.0.0.11:5760
- 기체 B: TCP → 10.0.0.21:5760
- 기체 C: TCP → 10.0.0.31:5760

### 4. 카메라 스트리밍 설정

#### PX4에서 비디오 스트리밍 활성화

**각 기체의 PX4 파라미터:**
```
VIDEO_ONLY = 1 (비디오만)
VIDEO_TCP_PORT = 5600 (기체별로 다르게)
```

**기체별 포트:**
- 기체 A: 5600
- 기체 B: 5601
- 기체 C: 5602

#### QGC에서 비디오 스트림 수신

**QGC 설정:**
1. Settings → Video
2. Video Source: UDP
3. UDP Port: 각 기체별로 설정
   - 기체 A: 5600
   - 기체 B: 5601
   - 기체 C: 5602

**또는 GStreamer 파이프라인 사용:**
```bash
# 기체 A
gst-launch-1.0 udpsrc port=5600 ! application/x-rtp,encoding-name=H264 ! rtph264depay ! h264parse ! avdec_h264 ! videoconvert ! autovideosink

# 기체 B
gst-launch-1.0 udpsrc port=5601 ! application/x-rtp,encoding-name=H264 ! rtph264depay ! h264parse ! avdec_h264 ! videoconvert ! autovideosink
```

## ROS2를 통한 통합 모니터링

ROS2를 통해 모든 기체의 데이터를 통합하고 QGC에 전달할 수 있습니다.

### MAVROS/MAVSDK를 통한 ROS2-MAVLink 브리지

#### 방법 1: MAVROS 사용

각 기체별로 MAVROS 노드 실행:

```bash
# 기체 A (도메인 1, MAV_SYS_ID=1과 일치)
export ROS_DOMAIN_ID=1
ros2 run mavros mavros_node \
    --ros-args \
    -p fcu_url:=udp://:14550@10.0.0.11:14550 \
    -p system_id:=1

# 기체 B (도메인 2, MAV_SYS_ID=2와 일치)
export ROS_DOMAIN_ID=2
ros2 run mavros mavros_node \
    --ros-args \
    -p fcu_url:=udp://:14551@10.0.0.21:14550 \
    -p system_id:=2

# 기체 C (도메인 3, MAV_SYS_ID=3과 일치)
export ROS_DOMAIN_ID=3
ros2 run mavros mavros_node \
    --ros-args \
    -p fcu_url:=udp://:14552@10.0.0.31:14550 \
    -p system_id:=3
```

#### 방법 2: MAVSDK-Python 사용

```python
#!/usr/bin/env python3
import asyncio
from mavsdk import System
from mavsdk.mission import MissionItem, MissionPlan

async def monitor_vehicle(system_id, connection_string):
    """단일 기체 모니터링"""
    drone = System()
    await drone.connect(connection_string)
    
    print(f"기체 {system_id} 연결됨")
    
    async for position in drone.telemetry.position():
        print(f"기체 {system_id}: {position.latitude_deg}, {position.longitude_deg}")

async def monitor_all_vehicles():
    """모든 기체 모니터링"""
    tasks = [
        monitor_vehicle(1, "udp://:14550"),
        monitor_vehicle(2, "udp://:14551"),
        monitor_vehicle(3, "udp://:14552"),
    ]
    await asyncio.gather(*tasks)

if __name__ == "__main__":
    asyncio.run(monitor_all_vehicles())
```

### ROS2 토픽을 MAVLink로 변환

모든 기체의 ROS2 토픽을 수집하여 QGC에 전달:

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from px4_msgs.msg import VehicleOdometry, VehicleStatus
from pymavlink import mavutil

class ROS2ToMAVLinkBridge(Node):
    def __init__(self, vehicle_id, domain_id):
        super().__init__(f'ros2_mavlink_bridge_{vehicle_id}')
        
        # ROS2 도메인 설정
        import os
        os.environ['ROS_DOMAIN_ID'] = str(domain_id)
        
        # MAVLink 연결
        self.mavlink = mavutil.mavlink_connection('udpout:127.0.0.1:14550')
        self.vehicle_id = vehicle_id
        
        # ROS2 토픽 구독
        self.odometry_sub = self.create_subscription(
            VehicleOdometry,
            '/fmu/out/vehicle_odometry',
            self.odometry_callback,
            10
        )
        
    def odometry_callback(self, msg):
        # ROS2 메시지를 MAVLink로 변환
        # 실제 변환 로직 구현 필요
        pass

# 각 기체별로 브리지 실행
# 기체 A: ROS2ToMAVLinkBridge(1, 0)
# 기체 B: ROS2ToMAVLinkBridge(2, 1)
# 기체 C: ROS2ToMAVLinkBridge(3, 2)
```

## 실제 구성 예시

### 네트워크 구성

```
                    WiFi 네트워크 (192.168.100.0/24)
                           │
        ┌──────────────────┼──────────────────┐
        │                  │                  │
    QGC PC            vim4-A/B/C          다른 노드
(192.168.100.x)   (192.168.100.x)      (같은 WiFi)
        │                  │
        │            wlan0 (WiFi)
        │                  │
        │            eth0 (10.0.0.1)
        │                  │
        │              LAN 케이블
        │                  │
        │         PX4-A/B/C (10.0.0.x)
        │
    MAVLink UDP 브로드캐스트
    (포트 14550, 14551, 14552)
```

### 설정 요약

**기체 A:**
- MAV_SYS_ID = 1
- IP: 10.0.0.11
- ROS_DOMAIN_ID = 1 (MAV_SYS_ID와 일치)
- MAVLink 포트: 14550
- 비디오 포트: 5600

**기체 B:**
- MAV_SYS_ID = 2
- IP: 10.0.0.21
- ROS_DOMAIN_ID = 2 (MAV_SYS_ID와 일치)
- MAVLink 포트: 14551
- 비디오 포트: 5601

**기체 C:**
- MAV_SYS_ID = 3
- IP: 10.0.0.31
- ROS_DOMAIN_ID = 3 (MAV_SYS_ID와 일치)
- MAVLink 포트: 14552
- 비디오 포트: 5602

## QGC에서 확인

### 다중 기체 표시

1. **지도 뷰**: 각 기체의 위치가 다른 색상/아이콘으로 표시됨
2. **비히클 선택**: 상단에서 기체 선택 가능
3. **비디오 뷰**: 각 기체의 카메라 영상 표시 (설정 필요)

### 모니터링 정보

각 기체별로 다음 정보를 확인할 수 있습니다:
- 위치 (GPS 좌표)
- 고도
- 배터리 상태
- 비행 모드
- 속도
- 자세 (롤, 피치, 요)
- 카메라 영상 (설정 시)

## 문제 해결

### 기체가 QGC에 나타나지 않는 경우

1. **MAV_SYS_ID 확인**: 각 기체가 고유한 ID를 가지는지 확인
2. **네트워크 연결 확인**: 모든 기체가 같은 네트워크에 있는지 확인
3. **포트 확인**: MAVLink 포트가 올바르게 설정되었는지 확인
4. **방화벽 확인**: UDP 포트가 열려있는지 확인

### 카메라 스트리밍이 작동하지 않는 경우

1. **비디오 포트 확인**: 각 기체의 비디오 포트가 올바른지 확인
2. **GStreamer 설치**: QGC에서 비디오를 받으려면 GStreamer 필요
3. **네트워크 대역폭**: 여러 스트림은 높은 대역폭 필요

### ROS2와 QGC 동시 사용

ROS2와 QGC를 동시에 사용하려면:
- ROS2: XRCE-DDS를 통해 데이터 수신
- QGC: MAVLink를 통해 데이터 수신
- 두 프로토콜이 동시에 작동 가능

## 요약

**질문: 3대의 기체를 QGC에서 모니터링할 수 있나요?**

**답변:**
- ✅ **가능합니다!** QGC는 다중 기체 모니터링을 지원합니다.
- ✅ 각 기체에 고유한 `MAV_SYS_ID` 설정 필요
- ✅ QGC의 멀티 비히클 모드 활성화
- ✅ 각 기체를 네트워크로 연결 (UDP 또는 TCP)
- ✅ 카메라 스트리밍도 가능 (각 기체별 포트 설정)

**추가 기능:**
- ROS2와 QGC를 동시에 사용 가능
- ROS2로 데이터 처리, QGC로 모니터링
- MAVROS/MAVSDK로 통합 가능

