# 문서 구조 및 역할 정리

이 문서는 프로젝트의 모든 문서 파일의 역할과 중복 여부를 정리합니다.

## 📚 문서 분류

### 1. 시작 가이드 (Getting Started)

#### [README.md](README.md) ⭐ **메인 문서**
- **역할**: 프로젝트 전체 개요 및 빠른 시작
- **대상**: 모든 사용자
- **내용**: 
  - 프로젝트 소개
  - 빠른 시작 가이드
  - 주요 명령어
  - 문제 해결
- **중요도**: ⭐⭐⭐⭐⭐

#### [QUICK_START.md](QUICK_START.md)
- **역할**: 최소한의 단계로 빠르게 시작
- **대상**: 경험 있는 사용자
- **내용**: 
  - 설치 스크립트 실행
  - 기본 확인
- **중요도**: ⭐⭐⭐⭐

### 2. 설치 가이드

#### [INSTALLATION.md](INSTALLATION.md)
- **역할**: 상세한 설치 가이드 (새로 설치한 Ubuntu VIM4)
- **대상**: 처음 설치하는 사용자
- **내용**: 
  - 사전 준비사항
  - 단계별 설치 과정
  - 설치 검증
  - 재부팅 후 확인
- **중요도**: ⭐⭐⭐⭐

#### [INSTALL_GUIDE.md](INSTALL_GUIDE.md)
- **역할**: 다른 VIM4에 프로젝트 복사 및 설치
- **대상**: 기존 프로젝트를 다른 보드에 설치하는 사용자
- **내용**: 
  - 파일 복사 방법
  - 설치 스크립트 실행
  - 설정 확인
- **중복**: INSTALLATION.md와 일부 중복, 하지만 "다른 보드에 복사"에 초점
- **중요도**: ⭐⭐⭐

**권장**: INSTALLATION.md를 메인으로 하고, INSTALL_GUIDE.md는 "복사" 부분만 강조

### 3. 프로젝트 종합 문서

#### [PROJECT_SUMMARY.md](PROJECT_SUMMARY.md) ⭐ **종합 정리**
- **역할**: 프로젝트 전체를 한눈에 보는 종합 문서
- **대상**: 프로젝트 전체를 이해하려는 사용자
- **내용**: 
  - 설치된 패키지 종류
  - FC 파라미터 설정
  - 네트워크 구성
  - ROS2 도메인 구성
  - 시스템 서비스 구성
- **중요도**: ⭐⭐⭐⭐⭐

#### [ID_MAPPING.md](ID_MAPPING.md)
- **역할**: 기체 ID 매핑 가이드
- **대상**: 군집 비행 설정하는 사용자
- **내용**: 
  - MAV_SYS_ID, ROS_DOMAIN_ID, UXRCE_DDS_DOM_ID 통일 규칙
  - 기체별 ID 설정 예시
- **중요도**: ⭐⭐⭐⭐

### 4. 네트워크 및 통신

#### [NETWORK_ARCHITECTURE.md](NETWORK_ARCHITECTURE.md)
- **역할**: 네트워크 아키텍처 설명
- **대상**: 네트워크 구조를 이해하려는 사용자
- **내용**: 
  - ROS2 DDS 멀티캐스트 설명
  - 네트워크 토폴로지
  - 도메인 ID 역할
- **중요도**: ⭐⭐⭐

#### [SWARM_FLIGHT.md](SWARM_FLIGHT.md)
- **역할**: 군집 비행 설정 가이드
- **대상**: 다중 기체를 제어하는 사용자
- **내용**: 
  - 기체 구분 방법
  - ROS_DOMAIN_ID 설정
  - IP 주소 할당
  - 편대 비행, 충돌 방지
- **중요도**: ⭐⭐⭐⭐

### 5. QGroundControl 관련

#### [QGC_MULTI_VEHICLE.md](QGC_MULTI_VEHICLE.md)
- **역할**: QGC에서 다중 기체 모니터링
- **대상**: QGC 사용자
- **내용**: 
  - MAV_SYS_ID 설정
  - 다중 기체 연결
  - 비디오 스트리밍
- **중요도**: ⭐⭐⭐⭐

#### [QGC_NETWORK_SETUP.md](QGC_NETWORK_SETUP.md)
- **역할**: QGC 네트워크 연결 설정
- **대상**: QGC 네트워크 설정하는 사용자
- **내용**: 
  - UDP/TCP 연결 설정
  - 파라미터 설정
- **중요도**: ⭐⭐⭐

#### [QGC_CONNECTION_GUIDE.md](QGC_CONNECTION_GUIDE.md)
- **역할**: QGC 연결 추가 가이드
- **대상**: QGC에서 연결을 추가하는 사용자
- **내용**: 
  - "Add Vehicle" 버튼 찾기
  - 연결 추가 방법
- **중복**: QGC_NETWORK_SETUP.md와 일부 중복
- **중요도**: ⭐⭐⭐

#### [QGC_DEVELOPMENT.md](QGC_DEVELOPMENT.md)
- **역할**: QGC 개발 및 수정 가이드
- **대상**: QGC를 수정하려는 개발자
- **내용**: 
  - Qt Creator 설치
  - VS Code 개발
  - 빌드 방법
- **중요도**: ⭐⭐

### 6. 화재 진압 시스템

#### [FIREFIGHTING_SYSTEM.md](FIREFIGHTING_SYSTEM.md) ⭐ **화재 진압 시스템 전체**
- **역할**: 화재 진압 시스템 전체 기술 스택
- **대상**: 화재 진압 시스템 개발자
- **내용**: 
  - 하드웨어 구성
  - 소프트웨어 스택
  - 화재 감지
  - 타겟팅 시스템
  - 소화탄 격발
  - 군집 제어
- **중요도**: ⭐⭐⭐⭐⭐

#### [RTK_FORMATION.md](RTK_FORMATION.md)
- **역할**: RTK GPS 기반 편대 비행 및 충돌 방지
- **대상**: 정밀 편대 비행 구현하는 개발자
- **내용**: 
  - RTK GPS 설정
  - 편대 형성 알고리즘
  - 충돌 방지 시스템
- **중요도**: ⭐⭐⭐⭐

#### [RTK_PRECISION_HOVERING.md](RTK_PRECISION_HOVERING.md)
- **역할**: RTK GPS 기반 정밀 호버링
- **대상**: 정밀 호버링 구현하는 개발자
- **내용**: 
  - RTK GPS 정밀도
  - 호버링 제어 알고리즘
  - 바람 보상
- **중요도**: ⭐⭐⭐⭐

#### [LIDAR_TARGETING.md](LIDAR_TARGETING.md)
- **역할**: LiDAR 기반 타겟 거리 측정 및 타겟팅
- **대상**: 정밀 타겟팅 구현하는 개발자
- **내용**: 
  - LiDAR 통합
  - 거리 측정
  - 타겟팅 알고리즘
- **중요도**: ⭐⭐⭐⭐

### 7. 유틸리티 가이드

#### [COPY_GUIDE.md](COPY_GUIDE.md)
- **역할**: 프로젝트 파일 복사 가이드
- **대상**: 다른 보드에 프로젝트를 복사하는 사용자
- **내용**: 
  - 전체 복사 vs 소스만 복사
  - 빌드 아티팩트 처리
- **중요도**: ⭐⭐⭐

#### [GITHUB_UPLOAD.md](GITHUB_UPLOAD.md)
- **역할**: GitHub 업로드 가이드
- **대상**: 프로젝트를 GitHub에 업로드하는 사용자
- **내용**: 
  - Git 초기화
  - GitHub 저장소 생성
  - 푸시 방법
- **중요도**: ⭐⭐

#### [GIT_WORKFLOW.md](GIT_WORKFLOW.md)
- **역할**: Git 워크플로우 가이드
- **대상**: 정기적으로 GitHub에 업데이트하는 사용자
- **내용**: 
  - 업데이트 스크립트 사용
  - Git 명령어
  - 문제 해결
- **중요도**: ⭐⭐

#### [CURSOR_GIT_GUIDE.md](CURSOR_GIT_GUIDE.md)
- **역할**: Cursor Git 확장 기능 사용 가이드
- **대상**: Cursor IDE 사용자
- **내용**: 
  - GUI로 커밋/푸시
  - 소스 제어 패널 사용
- **중복**: GIT_WORKFLOW.md와 일부 중복, 하지만 Cursor 특화
- **중요도**: ⭐⭐

## 🔍 중복 및 개선 사항

### 중복 문서

1. **INSTALLATION.md vs INSTALL_GUIDE.md**
   - **상태**: 일부 중복
   - **권장**: INSTALLATION.md를 메인으로 하고, INSTALL_GUIDE.md는 "복사" 부분만 강조
   - **조치**: INSTALL_GUIDE.md에 "INSTALLATION.md 참고" 링크 추가

2. **QGC_NETWORK_SETUP.md vs QGC_CONNECTION_GUIDE.md**
   - **상태**: 일부 중복
   - **권장**: QGC_NETWORK_SETUP.md를 메인으로 하고, QGC_CONNECTION_GUIDE.md는 "Add Vehicle" 찾기 부분만 강조
   - **조치**: 각 문서에 상호 참조 링크 추가

3. **GIT_WORKFLOW.md vs CURSOR_GIT_GUIDE.md**
   - **상태**: 일부 중복
   - **권장**: GIT_WORKFLOW.md는 명령어 중심, CURSOR_GIT_GUIDE.md는 GUI 중심으로 역할 분리
   - **조치**: 각 문서에 상호 참조 링크 추가

### 수정 완료 사항

1. ✅ **ID 불일치 수정**
   - 모든 문서에서 기본값을 1로 통일 (단일 기체 사용 시)
   - 군집 비행 시 1, 2, 3으로 통일
   - MAV_SYS_ID와 ROS_DOMAIN_ID 일치

2. ✅ **스크립트 수정**
   - `start_micro_ros_agent_wrapper.sh`: ROS_DOMAIN_ID 기본값 1로 변경
   - `install_px4_ros2_complete.sh`: ROS_DOMAIN_ID 기본값 1로 변경

3. ✅ **구버전 예시 제거**
   - SWARM_FLIGHT.md: ROS_DOMAIN_ID=0,1,2 예시를 1,2,3으로 수정
   - QGC_MULTI_VEHICLE.md: ROS_DOMAIN_ID=0,1,2 예시를 1,2,3으로 수정
   - NETWORK_ARCHITECTURE.md: ROS_DOMAIN_ID=0 예시를 1로 수정

## 📋 문서 읽기 순서 추천

### 처음 시작하는 사용자
1. [README.md](README.md) - 프로젝트 개요
2. [QUICK_START.md](QUICK_START.md) - 빠른 시작
3. [INSTALLATION.md](INSTALLATION.md) - 상세 설치

### 프로젝트 전체 이해
1. [PROJECT_SUMMARY.md](PROJECT_SUMMARY.md) - 종합 정리
2. [ID_MAPPING.md](ID_MAPPING.md) - ID 매핑 규칙
3. [NETWORK_ARCHITECTURE.md](NETWORK_ARCHITECTURE.md) - 네트워크 구조

### 군집 비행 설정
1. [SWARM_FLIGHT.md](SWARM_FLIGHT.md) - 군집 비행 가이드
2. [QGC_MULTI_VEHICLE.md](QGC_MULTI_VEHICLE.md) - QGC 다중 기체
3. [RTK_FORMATION.md](RTK_FORMATION.md) - RTK 편대 비행

### 화재 진압 시스템 개발
1. [FIREFIGHTING_SYSTEM.md](FIREFIGHTING_SYSTEM.md) - 전체 기술 스택
2. [RTK_PRECISION_HOVERING.md](RTK_PRECISION_HOVERING.md) - 정밀 호버링
3. [LIDAR_TARGETING.md](LIDAR_TARGETING.md) - 타겟팅 시스템

## ✅ 체크리스트

- [x] ID 불일치 수정 완료
- [x] 스크립트 기본값 수정 완료
- [x] 구버전 예시 제거 완료
- [ ] 중복 문서 정리 (상호 참조 추가)
- [ ] 문서 간 일관성 확인

---

**마지막 업데이트**: 2025-12-09
**다음 검토**: 문서 간 상호 참조 링크 추가

