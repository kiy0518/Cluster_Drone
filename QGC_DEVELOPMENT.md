# QGroundControl 개발 및 수정 가이드

## 개요

QGroundControl(QGC)을 수정하려면 소스 코드를 빌드해야 합니다. QGC는 **Qt 기반** 애플리케이션이므로 Qt 개발 환경이 필요합니다.

**테스트 환경:**
- **QGroundControl**: v5.0.8 (64-bit)
- **PX4 펌웨어**: v5.0.8
- **FC**: Pixhawk 6X (Standard v2A PM02D)

---

## 개발 환경 선택

### 방법 1: Qt Creator 사용 (권장) ⭐

**장점:**
- ✅ Qt 공식 IDE
- ✅ Qt 디자이너 통합
- ✅ 디버깅 도구 우수
- ✅ UI 수정이 쉬움

**단점:**
- ❌ Qt 설치 필요
- ❌ 용량이 큼

### 방법 2: VS Code 사용

**장점:**
- ✅ 가벼움
- ✅ 익숙한 환경
- ✅ 확장 기능 풍부

**단점:**
- ❌ Qt 디자이너 없음 (UI 수정 어려움)
- ❌ Qt 설정 필요

**결론:** VS Code에서도 가능하지만, Qt Creator가 더 편리합니다.

---

## 개발 환경 설정

### 1. 필수 요구사항

#### Qt 설치

**Qt 5.15 또는 Qt 6.x 필요**

**Ubuntu에서 Qt 설치:**

```bash
# Qt 5.15 설치 (QGC v5.0.8 권장)
sudo apt update
sudo apt install -y \
    qtbase5-dev \
    qtbase5-dev-tools \
    qttools5-dev \
    qttools5-dev-tools \
    qtdeclarative5-dev \
    qtquickcontrols2-5-dev \
    qml-module-qtquick2 \
    qml-module-qtquick-controls2 \
    qml-module-qtquick-layouts \
    qml-module-qtquick-window2 \
    qml-module-qtgraphicaleffects \
    qml-module-qtqml-models2 \
    libqt5serialport5-dev \
    libqt5svg5-dev \
    libqt5location5-dev \
    libqt5positioning5-dev \
    libqt5webchannel5-dev \
    libqt5webengine5-dev

# 또는 Qt 6.x (최신 버전)
sudo apt install -y \
    qt6-base-dev \
    qt6-tools-dev \
    qt6-declarative-dev \
    qt6-quickcontrols2-dev \
    qt6-serialport-dev \
    qt6-svg-dev \
    qt6-location-dev \
    qt6-positioning-dev \
    qt6-webchannel-dev \
    qt6-webengine-dev
```

#### 기타 필수 패키지

```bash
sudo apt install -y \
    build-essential \
    cmake \
    git \
    libssl-dev \
    libsdl2-dev \
    libgstreamer1.0-dev \
    libgstreamer-plugins-base1.0-dev \
    gstreamer1.0-plugins-base \
    gstreamer1.0-plugins-good \
    gstreamer1.0-plugins-bad \
    gstreamer1.0-plugins-ugly \
    gstreamer1.0-libav \
    libqt5opengl5-dev \
    libqt5serialport5-dev \
    libqt5svg5-dev \
    libqt5location5-dev \
    libqt5positioning5-dev \
    libqt5webchannel5-dev \
    libqt5webengine5-dev
```

### 2. QGroundControl 소스 코드 다운로드

```bash
cd ~
git clone https://github.com/mavlink/qgroundcontrol.git
cd qgroundcontrol
git checkout v5.0.8  # 또는 최신 태그
```

### 3. 빌드 방법

#### 방법 A: Qt Creator 사용

1. **Qt Creator 실행**
   ```bash
   qtcreator
   ```

2. **프로젝트 열기**
   - File → Open File or Project
   - `qgroundcontrol.pro` 선택

3. **빌드 설정**
   - Release 또는 Debug 모드 선택
   - Build → Build Project

4. **실행**
   - Run → Run (Ctrl+R)

#### 방법 B: 명령줄 빌드

```bash
cd ~/qgroundcontrol

# 빌드 디렉토리 생성
mkdir build
cd build

# CMake 빌드
cmake ..
make -j$(nproc)

# 실행
./qgroundcontrol
```

#### 방법 C: qmake 사용 (Qt 5)

```bash
cd ~/qgroundcontrol

# qmake 실행
qmake qgroundcontrol.pro

# 빌드
make -j$(nproc)

# 실행
./qgroundcontrol
```

---

## VS Code에서 개발하기

### VS Code 설정

#### 1. 필수 확장 설치

- **C/C++** (Microsoft)
- **Qt Tools** (Qt Company)
- **CMake Tools** (Microsoft)
- **QML** (Qt Company)

#### 2. VS Code 설정 파일 생성

**`.vscode/settings.json`:**
```json
{
    "cmake.configureOnOpen": true,
    "cmake.buildDirectory": "${workspaceFolder}/build",
    "C_Cpp.default.includePath": [
        "${workspaceFolder}/**",
        "/usr/include/qt5/**",
        "/usr/include/qt5/QtCore",
        "/usr/include/qt5/QtGui",
        "/usr/include/qt5/QtQml",
        "/usr/include/qt5/QtQuick"
    ],
    "C_Cpp.default.defines": [
        "QT_CORE_LIB",
        "QT_GUI_LIB",
        "QT_QML_LIB"
    ],
    "files.associations": {
        "*.qml": "qml",
        "*.ui": "xml"
    }
}
```

**`.vscode/tasks.json`:**
```json
{
    "version": "2.0.0",
    "tasks": [
        {
            "label": "build",
            "type": "shell",
            "command": "cmake --build build",
            "group": {
                "kind": "build",
                "isDefault": true
            }
        },
        {
            "label": "clean",
            "type": "shell",
            "command": "rm -rf build && mkdir build"
        }
    ]
}
```

**`.vscode/launch.json`:**
```json
{
    "version": "0.2.0",
    "configurations": [
        {
            "name": "QGC Debug",
            "type": "cppdbg",
            "request": "launch",
            "program": "${workspaceFolder}/build/qgroundcontrol",
            "args": [],
            "stopAtEntry": false,
            "cwd": "${workspaceFolder}",
            "environment": [],
            "externalConsole": false,
            "MIMode": "gdb",
            "setupCommands": [
                {
                    "description": "Enable pretty-printing for gdb",
                    "text": "-enable-pretty-printing",
                    "ignoreFailures": true
                }
            ]
        }
    ]
}
```

### VS Code에서 빌드 및 실행

1. **CMake 구성**
   - Ctrl+Shift+P → "CMake: Configure"
   - 빌드 타입 선택 (Debug/Release)

2. **빌드**
   - Ctrl+Shift+P → "CMake: Build"
   - 또는 F7

3. **실행**
   - F5 (디버그 모드)
   - 또는 터미널에서: `./build/qgroundcontrol`

---

## 주요 수정 포인트

### 1. UI 수정 (QML 파일)

**위치:** `src/QmlControls/`, `src/ui/`

**예시:**
```qml
// src/ui/MainWindow.qml 수정
import QtQuick 2.15
import QtQuick.Controls 2.15

ApplicationWindow {
    // UI 수정
}
```

**Qt Creator에서:**
- `.qml` 파일을 열면 디자이너 모드로 편집 가능

**VS Code에서:**
- 텍스트 편집만 가능 (디자이너 없음)

### 2. C++ 코드 수정

**위치:** `src/`

**예시:**
```cpp
// src/Vehicle/Vehicle.h 수정
class Vehicle : public QObject {
    // 코드 수정
};
```

**VS Code에서:**
- IntelliSense 지원
- 디버깅 가능

### 3. 파라미터 수정

**위치:** `src/FirmwarePlugin/PX4/PX4ParameterFactMetaData.xml`

**예시:**
```xml
<parameter>
    <name>MAV_SYS_ID</name>
    <default>1</default>
    <!-- 수정 -->
</parameter>
```

---

## 빌드 문제 해결

### Qt 버전 확인

```bash
qmake --version
# Qt version 5.15.x 또는 6.x 필요
```

### CMake 오류

```bash
# CMake 캐시 삭제 후 재구성
rm -rf build
mkdir build
cd build
cmake ..
```

### 의존성 오류

```bash
# 누락된 패키지 확인
ldd build/qgroundcontrol | grep "not found"

# 필요한 패키지 설치
sudo apt install -y [패키지명]
```

### ARM64 빌드 이슈

ARM64에서는 일부 패키지가 다를 수 있습니다:

```bash
# ARM64 특정 패키지 확인
dpkg -l | grep qt

# 필요시 소스에서 빌드
```

---

## 커스터마이징 예시

### 1. UI 색상 변경

**파일:** `src/ui/MainWindow.qml`

```qml
// 기본 색상 변경
palette {
    window: "#2C3E50"  // 배경색
    button: "#3498DB"  // 버튼 색상
}
```

### 2. 파라미터 기본값 변경

**파일:** `src/FirmwarePlugin/PX4/PX4ParameterFactMetaData.xml`

```xml
<parameter>
    <name>MAV_SYS_ID</name>
    <default>1</default>  <!-- 기본값 변경 -->
</parameter>
```

### 3. 기능 추가

**파일:** `src/Vehicle/Vehicle.h`, `src/Vehicle/Vehicle.cpp`

```cpp
// 새로운 기능 추가
class Vehicle : public QObject {
    Q_OBJECT
public:
    void customFunction();  // 커스텀 함수 추가
};
```

---

## 빌드 시간 단축

### 병렬 빌드

```bash
make -j$(nproc)  # CPU 코어 수만큼 병렬 빌드
```

### 증분 빌드

```bash
# 변경된 파일만 재빌드
make
```

### 릴리즈 빌드

```bash
cmake -DCMAKE_BUILD_TYPE=Release ..
make -j$(nproc)
```

---

## 배포용 빌드

### 실행 파일 패키징

```bash
# Linux AppImage 생성
./build/qgroundcontrol --appimage

# 또는 직접 실행 파일 복사
cp build/qgroundcontrol ~/qgc-custom
```

### 의존성 포함

```bash
# ldd로 의존성 확인
ldd build/qgroundcontrol

# 필요한 라이브러리 복사
```

---

## 요약

### 개발 환경 선택

| 방법 | UI 수정 | 코드 수정 | 디버깅 | 권장도 |
|------|---------|----------|--------|--------|
| **Qt Creator** | ⭐⭐⭐⭐⭐ | ⭐⭐⭐⭐⭐ | ⭐⭐⭐⭐⭐ | ⭐⭐⭐⭐⭐ |
| **VS Code** | ⭐⭐ | ⭐⭐⭐⭐ | ⭐⭐⭐⭐ | ⭐⭐⭐ |

### 필수 설치

1. **Qt 5.15 또는 Qt 6.x**
2. **CMake**
3. **빌드 도구** (gcc, make)
4. **QGC 소스 코드**

### 권장 워크플로우

1. **Qt Creator 설치** (UI 수정이 많을 경우)
2. **VS Code 설치** (코드 수정 위주일 경우)
3. **소스 코드 다운로드**
4. **빌드 및 테스트**
5. **수정 및 재빌드**

---

## 참고 자료

- [QGroundControl 개발 문서](https://dev.qgroundcontrol.com/)
- [Qt 공식 문서](https://doc.qt.io/)
- [QGC GitHub 저장소](https://github.com/mavlink/qgroundcontrol)

