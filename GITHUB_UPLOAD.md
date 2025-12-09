# GitHub 업로드 가이드

이 문서는 Cluster_Drone 프로젝트를 GitHub에 업로드하는 방법을 설명합니다.

## 📋 사전 준비사항

1. **Git 설치 확인**
   ```bash
   git --version
   ```
   설치되어 있지 않다면:
   ```bash
   sudo apt update
   sudo apt install git -y
   ```

2. **GitHub 계정 및 저장소 준비**
   - GitHub 계정이 있어야 합니다
   - 새 저장소를 생성하거나 기존 저장소를 사용할 수 있습니다

## 🚀 단계별 업로드 방법

### 1단계: Git 초기화

프로젝트 디렉토리에서 Git 저장소를 초기화합니다:

```bash
cd ~/Cluster_Drone
git init
```

### 2단계: Git 사용자 정보 설정 (처음 한 번만)

```bash
git config --global user.name "Your Name"
git config --global user.email "your.email@example.com"
```

또는 이 프로젝트에만 적용하려면 `--global` 없이:

```bash
git config user.name "Your Name"
git config user.email "your.email@example.com"
```

### 3단계: 파일 추가 및 커밋

모든 파일을 스테이징하고 첫 커밋을 생성합니다:

```bash
# 모든 파일 추가
git add .

# 커밋 메시지와 함께 커밋
git commit -m "Initial commit: PX4-ROS2 XRCE-DDS swarm firefighting drone project"
```

### 4단계: GitHub 저장소 생성

1. GitHub 웹사이트(https://github.com)에 로그인
2. 우측 상단의 **"+"** 버튼 클릭 → **"New repository"** 선택
3. 저장소 정보 입력:
   - **Repository name**: `Cluster_Drone` (또는 원하는 이름)
   - **Description**: `3-drone swarm firefighting system with PX4-ROS2 XRCE-DDS integration`
   - **Visibility**: Public 또는 Private 선택
   - **⚠️ 중요**: "Initialize this repository with a README" 체크하지 않기 (이미 로컬에 파일이 있으므로)
4. **"Create repository"** 클릭

### 5단계: 원격 저장소 추가 및 푸시

GitHub에서 생성된 저장소의 URL을 복사합니다 (예: `https://github.com/yourusername/Cluster_Drone.git`)

```bash
# 원격 저장소 추가 (URL을 실제 저장소 URL로 변경)
git remote add origin https://github.com/yourusername/Cluster_Drone.git

# 기본 브랜치 이름을 main으로 설정 (GitHub 기본값)
git branch -M main

# GitHub에 푸시
git push -u origin main
```

**인증 방법:**

#### 방법 1: Personal Access Token (권장)

1. GitHub → Settings → Developer settings → Personal access tokens → Tokens (classic)
2. "Generate new token (classic)" 클릭
3. 권한 선택:
   - `repo` (전체 저장소 권한)
4. 토큰 생성 후 복사
5. 푸시 시 비밀번호 대신 토큰 사용:
   ```bash
   git push -u origin main
   # Username: your_github_username
   # Password: <복사한 토큰>
   ```

#### 방법 2: SSH 키 사용 (더 편리함)

```bash
# SSH 키 생성 (이미 있다면 생략)
ssh-keygen -t ed25519 -C "your.email@example.com"

# 공개 키 복사
cat ~/.ssh/id_ed25519.pub

# GitHub → Settings → SSH and GPG keys → New SSH key
# 위에서 복사한 공개 키를 붙여넣기

# 원격 저장소 URL을 SSH 형식으로 변경
git remote set-url origin git@github.com:yourusername/Cluster_Drone.git

# 푸시
git push -u origin main
```

## 📝 이후 업데이트 방법

프로젝트를 수정한 후 GitHub에 업데이트하려면:

```bash
cd ~/Cluster_Drone

# 변경사항 확인
git status

# 변경된 파일 추가
git add .

# 또는 특정 파일만 추가
git add 파일명

# 커밋
git commit -m "변경사항 설명"

# GitHub에 푸시
git push
```

## 🔍 현재 상태 확인

```bash
# Git 상태 확인
git status

# 커밋 히스토리 확인
git log --oneline

# 원격 저장소 확인
git remote -v
```

## ⚠️ 주의사항

### 1. 민감한 정보 제외

다음과 같은 정보는 **절대** 커밋하지 마세요:

- 비밀번호
- API 키
- 개인 인증 정보
- 시스템 특정 경로 (예: `/home/khadas` 대신 `~` 사용 고려)

### 2. .gitignore 확인

`.gitignore` 파일에 다음이 포함되어 있습니다:
- 빌드 아티팩트 (`build/`, `install/`, `log/`)
- 임시 파일
- IDE 설정 파일

### 3. 대용량 파일

GitHub는 단일 파일 100MB, 저장소 1GB 제한이 있습니다. 대용량 바이너리 파일은 제외하세요.

## 🛠️ 문제 해결

### "fatal: remote origin already exists" 오류

```bash
# 기존 원격 저장소 제거 후 다시 추가
git remote remove origin
git remote add origin https://github.com/yourusername/Cluster_Drone.git
```

### "Permission denied" 오류

- SSH 키가 제대로 설정되었는지 확인
- Personal Access Token이 올바른지 확인
- 저장소 권한이 있는지 확인

### "Large files detected" 오류

```bash
# Git LFS 설치 (대용량 파일용)
sudo apt install git-lfs
git lfs install

# 또는 대용량 파일을 .gitignore에 추가
```

## 📚 추가 리소스

- [Git 공식 문서](https://git-scm.com/doc)
- [GitHub 가이드](https://docs.github.com/)
- [Git 튜토리얼](https://www.atlassian.com/git/tutorials)

## ✅ 체크리스트

업로드 전 확인사항:

- [ ] `.gitignore` 파일이 올바르게 설정되었는가?
- [ ] 민감한 정보가 포함되지 않았는가?
- [ ] README.md가 최신 상태인가?
- [ ] 모든 문서 파일이 포함되었는가?
- [ ] 빌드 아티팩트가 제외되었는가?
- [ ] Git 사용자 정보가 설정되었는가?

---

**업로드 완료 후**: GitHub 저장소의 README.md가 자동으로 표시됩니다. 프로젝트 설명과 설치 방법을 확인할 수 있습니다.

