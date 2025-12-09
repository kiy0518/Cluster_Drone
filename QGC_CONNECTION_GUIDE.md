# QGroundControl 연결 추가 가이드 (실제 화면 기준)

## "Add Vehicle"이 보이지 않는 경우

**테스트 환경:**
- **QGroundControl**: v5.0.8 (64-bit)

최신 QGroundControl 버전(v5.0.8)에서는 "Add Vehicle" 메뉴가 보이지 않을 수 있습니다. 대신 **Settings 메뉴**에서 연결을 추가해야 합니다.

## 연결 추가 방법

### 방법 1: Settings → Comm Links (가장 일반적)

**단계별 가이드:**

1. **QGroundControl 실행**

2. **Settings 메뉴 열기**
   - 우측 상단의 **톱니바퀴 아이콘 (⚙️)** 클릭
   - 또는 상단 메뉴바에서 **"Settings"** 클릭

3. **Comm Links 선택**
   - Settings 창 왼쪽 메뉴에서 **"Comm Links"** 선택
   - 또는 **"Connections"** (버전에 따라 다름)

4. **Add 버튼 클릭**
   - 우측 상단 또는 하단의 **"Add"** 또는 **"+"** 버튼 클릭

5. **연결 타입 선택**
   - **UDP** 또는 **TCP** 선택

6. **연결 정보 입력**
   - UDP: 포트 번호 입력
   - TCP: IP 주소와 포트 입력

7. **저장**
   - **"OK"** 또는 **"Save"** 버튼 클릭

8. **연결 활성화**
   - 연결 목록에서 방금 추가한 연결의 **체크박스**를 체크하여 활성화

### 방법 2: Settings → General → AutoConnect

**자동 연결 설정:**

1. **Settings → General**

2. **"AutoConnect to the following devices"** 섹션 찾기

3. 원하는 연결 타입 체크:
   - ✅ **UDP** (네트워크 기체 자동 감지)
   - ✅ **Serial** (USB/시리얼 연결)
   - ✅ **TCP** (TCP 연결)

4. **포트 설정** (UDP의 경우):
   - 기본 포트: `14550`

5. **OK** 클릭

이렇게 하면 QGC가 자동으로 네트워크의 기체를 감지합니다.

## 실제 화면 위치

### QGC 메뉴 구조

```
QGroundControl
├── 상단 메뉴바
│   ├── File
│   ├── Settings (⚙️ 아이콘) ← 여기!
│   └── ...
│
└── Settings 창
    ├── General
    │   └── AutoConnect ← 자동 연결 설정
    ├── Comm Links ← 연결 추가/관리
    ├── Plan View
    └── ...
```

### Comm Links 화면

```
┌─────────────────────────────────────┐
│ Comm Links                          │
├─────────────────────────────────────┤
│ [Add] [Remove] [Edit]               │
│                                     │
│ ☑ UDP Connection                   │
│   Port: 14550                      │
│                                     │
│ ☐ TCP - Drone A                    │
│   10.0.0.11:5760                   │
│                                     │
│ ☐ TCP - Drone B                    │
│   10.0.0.21:5760                   │
└─────────────────────────────────────┘
```

## UDP 연결 추가 (상세)

1. **Settings → Comm Links → Add**

2. **연결 정보 입력:**
   ```
   Name: UDP Connection
   Type: UDP
   Listening Port: 14550
   Target Host: (비워두기)
   Target Port: 14550
   ```

3. **OK** 클릭

4. **체크박스로 활성화**

## TCP 연결 추가 (상세)

1. **Settings → Comm Links → Add**

2. **연결 정보 입력:**
   ```
   Name: TCP - Drone A
   Type: TCP
   Server Address: 10.0.0.11
   Server Port: 5760
   ```

3. **OK** 클릭

4. **체크박스로 활성화**

5. **기체 B, C도 반복**

## 연결 확인

### 연결이 활성화되면:

1. **상단 바에 기체 아이콘 표시**
   - 여러 기체가 있으면 각각 다른 아이콘
   - 각 기체를 클릭하여 선택 가능

2. **지도에 기체 위치 표시**
   - 각 기체가 다른 색상/아이콘으로 표시

3. **HUD에 기체 정보 표시**
   - 선택한 기체의 정보가 표시됨

## 문제 해결

### Settings 메뉴가 보이지 않는 경우

1. **QGC 버전 확인:**
   ```bash
   qgroundcontrol --version
   ```

2. **최신 버전으로 업데이트:**
   - [QGC 다운로드 페이지](https://qgroundcontrol.com/downloads/)에서 최신 버전 다운로드

### Comm Links 메뉴가 보이지 않는 경우

1. **Settings 창 크기 조정**
   - Settings 창이 작으면 메뉴가 숨겨져 있을 수 있음

2. **스크롤 확인**
   - 왼쪽 메뉴를 스크롤하여 "Comm Links" 찾기

3. **검색 기능 사용**
   - Settings 창에 검색창이 있으면 "Comm" 또는 "Link" 검색

### 연결이 작동하지 않는 경우

1. **연결이 활성화되었는지 확인**
   - Comm Links 목록에서 체크박스가 체크되어 있는지 확인

2. **포트 확인**
   - UDP: 14550
   - TCP: 5760 (PX4 기본 포트)

3. **네트워크 연결 확인**
   ```bash
   # QGC PC에서 PX4 IP로 ping
   ping 10.0.0.11
   ```

4. **방화벽 확인**
   ```bash
   # UDP 포트 열기
   sudo ufw allow 14550/udp
   ```

## 요약

**"Add Vehicle"이 보이지 않을 때:**

1. ✅ **Settings (⚙️) → Comm Links → Add**
2. ✅ **Settings → General → AutoConnect** (자동 연결)

**핵심:**
- "Add Vehicle" 대신 **"Comm Links"** 메뉴 사용
- 연결 추가 후 **체크박스로 활성화** 필수
- 최신 QGC는 자동 감지 기능 제공

