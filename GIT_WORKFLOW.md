# Git 워크플로우 가이드

이 문서는 프로젝트를 주기적으로 GitHub에 업데이트하는 방법을 설명합니다.

## 🚀 빠른 업데이트 (권장)

### 방법 1: 업데이트 스크립트 사용 (가장 간단)

```bash
cd ~/Cluster_Drone

# 커밋 메시지와 함께 업데이트
./update_github.sh "RTK GPS 정밀 호버링 기능 추가"

# 또는 커밋 메시지를 입력하지 않으면 프롬프트가 나타남
./update_github.sh
```

**장점:**
- 한 번의 명령으로 모든 작업 완료
- 변경사항 확인 후 업로드
- 안전한 확인 절차 포함

### 방법 2: Git 명령어 직접 사용

```bash
cd ~/Cluster_Drone

# 1. 변경사항 확인
git status

# 2. 변경된 파일 추가
git add .

# 또는 특정 파일만 추가
git add 파일명.md

# 3. 커밋 생성
git commit -m "변경사항 설명"

# 4. GitHub에 푸시
git push
```

**푸시 시 인증:**
- Username: `kiy0518`
- Password: Personal Access Token 입력

## 📋 일반적인 워크플로우

### 1. 작업 전: 최신 상태 확인 (다른 컴퓨터에서 작업하는 경우)

```bash
cd ~/Cluster_Drone
git pull
```

### 2. 작업 중: 변경사항 확인

```bash
# 현재 상태 확인
git status

# 변경된 내용 확인
git diff

# 특정 파일의 변경 내용 확인
git diff 파일명.md
```

### 3. 작업 후: 커밋 및 푸시

```bash
# 모든 변경사항 추가
git add .

# 커밋
git commit -m "기능 설명: 상세 내용"

# 푸시
git push
```

## 💡 유용한 Git 명령어

### 상태 확인

```bash
# 현재 상태 확인
git status

# 커밋 히스토리 확인
git log --oneline -10

# 원격 저장소 확인
git remote -v
```

### 변경사항 관리

```bash
# 특정 파일만 추가
git add 파일명.md

# 특정 파일 제외
git reset HEAD 파일명.md

# 모든 변경사항 취소 (주의!)
git reset --hard HEAD

# 마지막 커밋 수정 (아직 푸시하지 않은 경우)
git commit --amend -m "수정된 메시지"
```

### 브랜치 관리

```bash
# 새 브랜치 생성
git checkout -b feature/new-feature

# 브랜치 목록 확인
git branch

# 브랜치 전환
git checkout 브랜치명

# 브랜치 병합
git merge 브랜치명
```

## 🔄 자주 사용하는 시나리오

### 시나리오 1: 문서 수정 후 업로드

```bash
cd ~/Cluster_Drone
./update_github.sh "README.md 업데이트: 설치 방법 개선"
```

### 시나리오 2: 스크립트 수정 후 업로드

```bash
cd ~/Cluster_Drone
git add install_px4_ros2_complete.sh
git commit -m "설치 스크립트: 네트워크 설정 개선"
git push
```

### 시나리오 3: 여러 파일 수정 후 업로드

```bash
cd ~/Cluster_Drone
git add .
git commit -m "프로젝트 문서화: RTK GPS 가이드 추가"
git push
```

### 시나리오 4: 실수로 잘못된 파일 추가한 경우

```bash
# 스테이징 취소
git reset HEAD 잘못된파일.txt

# 커밋 수정 (아직 푸시하지 않은 경우)
git commit --amend
```

## ⚠️ 주의사항

### 1. 커밋 전 확인

```bash
# 커밋 전에 항상 변경사항 확인
git status
git diff
```

### 2. 민감한 정보 제외

다음은 **절대** 커밋하지 마세요:
- 비밀번호
- API 키
- 개인 인증 정보
- 시스템 특정 경로 (필요시 수정)

### 3. 의미 있는 커밋 메시지

```bash
# 좋은 예
git commit -m "RTK GPS: 정밀 호버링 기능 추가"
git commit -m "설치 스크립트: 네트워크 설정 자동화 개선"
git commit -m "문서: QGroundControl 연결 가이드 추가"

# 나쁜 예
git commit -m "수정"
git commit -m "업데이트"
git commit -m "asdf"
```

### 4. 정기적인 백업

```bash
# 주기적으로 푸시하여 백업
./update_github.sh "일일 백업: $(date +%Y-%m-%d)"
```

## 🔧 문제 해결

### "Your branch is ahead of 'origin/main' by X commits"

```bash
# 단순히 푸시하지 않은 경우
git push
```

### "Updates were rejected because the remote contains work"

```bash
# 원격 저장소의 변경사항을 먼저 가져옴
git pull

# 충돌 해결 후 다시 푸시
git push
```

### "Authentication failed"

```bash
# Personal Access Token 확인
# GitHub → Settings → Developer settings → Personal access tokens

# 토큰이 만료되었으면 새로 생성
```

### "fatal: not a git repository"

```bash
# 올바른 디렉토리로 이동
cd ~/Cluster_Drone
```

## 📅 권장 업데이트 주기

- **작업 완료 시**: 기능 추가, 버그 수정 후 즉시
- **일일 백업**: 하루 작업 종료 시
- **주간 백업**: 주말에 전체 프로젝트 백업
- **중요 변경 전**: 큰 변경사항 전에 백업

## 🎯 빠른 참조

```bash
# 가장 자주 사용하는 명령어
cd ~/Cluster_Drone
./update_github.sh "변경사항 설명"

# 또는
git add .
git commit -m "변경사항 설명"
git push
```

---

**추가 도움말:**
- Git 공식 문서: https://git-scm.com/doc
- GitHub 가이드: https://docs.github.com/
- 프로젝트 README: [README.md](README.md)

