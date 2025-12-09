# QGroundControl 네트워크 연결 설정 가이드

## 개요

QGroundControl(QGC)에서 여러 PX4 기체를 네트워크로 연결하는 방법을 설명합니다.

## 네트워크 연결 방법

### 방법 1: UDP 브로드캐스트 (권장) ⭐

**장점:**
- ✅ 설정이 간단함
- ✅ 여러 기체를 자동으로 감지
- ✅ 같은 네트워크의 모든 기체에 연결 가능

**단점:**
- ⚠️ 모든 기체가 같은 네트워크에 있어야 함

#### PX4 파라미터 설정

각 기체의 QGroundControl에서:

1. **기체에 연결** (USB 또는 시리얼)
2. **Parameters → MAVLink** 메뉴로 이동
3. **다음 파라미터 설정:**

```
MAV_1_CONFIG = TELEM 2 (UDP)
MAV_1_MODE = 2 (Onboard)
MAV_1_RATE = 1000000
MAV_1_FORWARD = 1
MAV_1_BROADCAST = 1
```

4. **파라미터 저장 후 기체 재부팅**

#### QGC에서 연결 설정

**방법 A: Settings 메뉴에서 연결 추가 (권장)**

1. **QGroundControl 실행**

2. **Settings 메뉴 열기**
   - 우측 상단의 **톱니바퀴 아이콘** 클릭
   - 또는 메뉴: **Settings** (상단 메뉴바)

3. **Comm Links 메뉴 선택**
   - Settings 창에서 **"Comm Links"** 또는 **"Connections"** 선택
   - 또는 Settings → **General** → **Comm Links**

4. **Add 버튼 클릭**
   - "Add" 또는 "+" 버튼 클릭

5. **Connection Type 선택**
   - **"UDP"** 선택

6. **포트 설정**
   ```
   Name: UDP Connection (원하는 이름)
   Listening Port: 14550
   Target Host: (비워두기 - 브로드캐스트)
   Target Port: 14550
   ```

7. **OK 또는 Save 클릭**

8. **연결 활성화**
   - 연결 목록에서 방금 추가한 연결을 **체크**하여 활성화

9. **연결 확인**
   - 상단에 기체가 나타나면 성공
   - 여러 기체가 있으면 각각 다른 아이콘으로 표시됨

**방법 B: 자동 연결 설정 (최신 버전)**

최신 QGC 버전에서는 자동 감지 기능이 있습니다:

1. **Settings → General**
2. **"AutoConnect to the following devices"** 섹션 확인
3. **"UDP"** 체크박스 선택
4. **포트 설정** (기본: 14550)
5. **OK**

이렇게 하면 QGC가 자동으로 네트워크의 기체를 감지합니다.

**참고:**
- QGC 버전에 따라 메뉴 위치가 다를 수 있습니다
- "Add Vehicle" 대신 "Comm Links" 또는 "Connections" 메뉴를 찾아보세요

### 방법 2: TCP/IP (직접 연결)

**장점:**
- ✅ 특정 기체만 연결 가능
- ✅ 더 안정적인 연결

**단점:**
- ❌ 각 기체를 개별적으로 추가해야 함

#### PX4 파라미터 설정

각 기체의 QGroundControl에서:

```
MAV_1_CONFIG = TELEM 2 (TCP)
MAV_1_MODE = 2 (Onboard)
MAV_1_RATE = 1000000
```

#### QGC에서 연결 설정

**Settings 메뉴에서 TCP 연결 추가:**

1. **Settings → Comm Links** (또는 Connections)

2. **Add 버튼 클릭**

3. **Connection Type: TCP 선택**

4. **연결 정보 입력:**

   **기체 A:**
   ```
   Name: TCP - Drone A
   Server Address: 10.0.0.11
   Server Port: 5760
   ```

   **기체 B:**
   ```
   Name: TCP - Drone B
   Server Address: 10.0.0.21
   Server Port: 5760
   ```

   **기체 C:**
   ```
   Name: TCP - Drone C
   Server Address: 10.0.0.31
   Server Port: 5760
   ```

5. **각 연결마다 OK/Save 클릭**

6. **연결 활성화**
   - 연결 목록에서 각 연결을 **체크**하여 활성화

### 방법 3: UDP (특정 IP 지정)

특정 기체만 연결하려면:

1. **Settings → Comm Links → Add**
2. **Connection Type: UDP 선택**
3. **포트 설정:**
   ```
   Name: UDP - Drone A
   Listening Port: 14550
   Target Host: 10.0.0.11  (기체 A의 IP)
   Target Port: 14550
   ```
4. **OK/Save 클릭**
5. **연결 활성화** (체크박스)

이 방법으로 각 기체를 개별적으로 추가할 수 있습니다.

## 네트워크 구성 예시

### 시나리오 1: 모든 기체가 같은 WiFi 네트워크

```
                    WiFi 네트워크 (192.168.100.0/24)
                           │
        ┌──────────────────┼──────────────────┐
        │                  │                  │
    QGC PC            vim4-A/B/C          PX4 기체들
(192.168.100.x)   (192.168.100.x)      (10.0.0.x)
        │                  │                  │
        │                  │            eth0 (직접 연결)
        │                  │                  │
        │            wlan0 (WiFi)             │
        │                  │                  │
        │            eth0 (10.0.0.1)          │
        │                  │                  │
        │              LAN 케이블             │
        │                  │                  │
        │         PX4-A/B/C (10.0.0.11/21/31)│
        │                                     │
        └────────── UDP 브로드캐스트 ──────────┘
              (포트 14550, WiFi 네트워크)
```

**설정:**
- 모든 기체의 PX4에서 UDP 브로드캐스트 활성화
- QGC에서 UDP 연결 추가 (포트 14550)
- 모든 기체가 자동으로 감지됨

### 시나리오 2: 각 기체를 개별적으로 연결

```
QGC PC (192.168.100.10)
    │
    ├─ TCP → 10.0.0.11:5760 (기체 A)
    ├─ TCP → 10.0.0.21:5760 (기체 B)
    └─ TCP → 10.0.0.31:5760 (기체 C)
```

**설정:**
- 각 기체의 PX4에서 TCP 서버 활성화
- QGC에서 각 기체를 개별적으로 TCP로 연결

## 실제 설정 단계

### 1단계: PX4 파라미터 확인 및 설정

**각 기체별로:**

1. QGroundControl에서 기체에 연결 (USB/시리얼)
2. Parameters → MAVLink
3. 다음 확인:
   - `MAV_SYS_ID`: 각 기체마다 고유 (1, 2, 3...)
   - `MAV_1_CONFIG`: UDP 또는 TCP 선택
   - `MAV_1_MODE`: 2 (Onboard)
   - `MAV_1_RATE`: 1000000 (1MHz)

### 2단계: 네트워크 연결 확인

**각 SBC에서:**

```bash
# PX4 IP 확인
ip neigh show dev eth0

# 포트 확인
ss -ulnp | grep 14550  # UDP
ss -tlnp | grep 5760   # TCP
```

### 3단계: QGC에서 연결 추가

**UDP 브로드캐스트 (권장):**

1. QGC 실행
2. **Settings** (우측 상단 톱니바퀴 아이콘) → **Comm Links**
3. **Add** 버튼 클릭
4. Connection Type: **UDP** 선택
5. Listening Port: `14550`
6. Target Host: (비워두기 - 브로드캐스트)
7. Target Port: `14550`
8. **OK/Save** 클릭
9. 연결 목록에서 **체크박스**로 활성화

**또는 자동 연결:**
1. Settings → General
2. "AutoConnect to the following devices" → **UDP** 체크
3. 포트: `14550`

**TCP (개별 연결):**

1. QGC 실행
2. **Settings → Comm Links → Add**
3. Connection Type: **TCP** 선택
4. Server Address: `10.0.0.11` (기체 A)
5. Server Port: `5760`
6. **OK/Save** 클릭
7. 연결 목록에서 **체크박스**로 활성화
8. 반복 (기체 B, C)

### 4단계: 연결 확인

**QGC에서:**
- 상단에 기체 아이콘이 나타나면 성공
- 여러 기체가 있으면 각각 표시됨
- 각 기체를 클릭하여 선택 가능

**터미널에서 확인:**

```bash
# MAVLink 메시지 확인
mavlink-routerd -e 127.0.0.1:14550

# 또는
mavproxy.py --master=udp:127.0.0.1:14550
```

## 문제 해결

### 기체가 QGC에 나타나지 않는 경우

1. **네트워크 연결 확인:**
   ```bash
   # QGC PC에서 PX4 IP로 ping
   ping 10.0.0.11
   ```

2. **포트 확인:**
   ```bash
   # PX4에서 포트 리스닝 확인
   ss -ulnp | grep 14550
   ```

3. **방화벽 확인:**
   ```bash
   # UDP 포트 14550 열기
   sudo ufw allow 14550/udp
   ```

4. **MAV_SYS_ID 확인:**
   - 각 기체가 고유한 ID를 가지는지 확인

### 연결이 끊어지는 경우

1. **네트워크 안정성 확인:**
   ```bash
   # 패킷 손실 확인
   ping -c 100 10.0.0.11
   ```

2. **MAVLink 전송률 조정:**
   - `MAV_1_RATE`를 낮춤 (예: 500000)

3. **네트워크 대역폭 확인:**
   - 여러 기체가 같은 네트워크를 사용하면 대역폭 부족 가능

### 여러 기체가 같은 것으로 보이는 경우

1. **MAV_SYS_ID 확인:**
   - 각 기체가 고유한 ID를 가지는지 확인
   - Parameters → MAVLink → MAV_SYS_ID

2. **기체 재부팅:**
   - 파라미터 변경 후 재부팅 필요

## 요약

**QGC 네트워크 연결 설정:**

1. **PX4 파라미터 설정:**
   - `MAV_1_CONFIG = TELEM 2 (UDP 또는 TCP)`
   - `MAV_SYS_ID = 고유 ID (1, 2, 3...)`

2. **QGC에서 연결 추가:**
   - **Settings (⚙️) → Comm Links → Add**
   - **UDP (권장):** UDP 선택 → 포트 14550
   - **TCP:** TCP 선택 → IP:포트 입력
   - 연결 목록에서 **체크박스로 활성화**

3. **연결 확인:**
   - QGC 상단에 기체 아이콘 표시 확인

**"Add Vehicle"이 보이지 않을 때:**
- Settings → Comm Links 메뉴 사용
- 자세한 내용은 [QGC_CONNECTION_GUIDE.md](QGC_CONNECTION_GUIDE.md) 참고

**권장 방법:**
- 같은 네트워크: UDP 브로드캐스트
- 개별 연결: TCP/IP

