#!/bin/bash
set -euo pipefail

# 1. 경로 설정
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$SCRIPT_DIR"
REPOS_FILE="$PROJECT_ROOT/dependencies.repos"
WS_SRC="$PROJECT_ROOT/ros2_ws/src"
EXTERNAL_SRC="$WS_SRC/external"
ROS_DISTRO="humble"

# 2. 출력 색상 정의
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NO_COLOR='\033[0m'

# 3. 명령 존재 여부 확인 함수
require_cmd() {
  if ! command -v "$1" &> /dev/null; then
    echo -e "${RED}Error: '$1' 명령어를 찾을 수 없습니다. 설치 후 다시 시도해주세요.${NO_COLOR}"
    exit 1
  fi
}

echo -e "\n${YELLOW}[1/4] 필수 도구 확인 및 디렉토리 준비...${NO_COLOR}"
require_cmd vcs
require_cmd git
require_cmd python3
require_cmd pip3

mkdir -p "$EXTERNAL_SRC"

echo -e "\n${YELLOW}[2/4] 시스템 의존성 설치 (ROS 2 Apt packages)...${NO_COLOR}"
sudo apt update
sudo apt install -y \
    ros-$ROS_DISTRO-cartographer \
    ros-$ROS_DISTRO-cartographer-ros

echo -e "\n${YELLOW}[3/4] 외부 저장소 가져오기 (vcs import)...${NO_COLOR}"
if [ ! -f "$REPOS_FILE" ]; then
  echo -e "${RED}Error: 리포지토리 설정 파일이 없습니다: $REPOS_FILE${NO_COLOR}"
  exit 1
fi

vcs import "$EXTERNAL_SRC" < "$REPOS_FILE"

echo -e "\n${YELLOW}[4/4] Python 의존성 라이브러리 설치...${NO_COLOR}"
# qrcode 라이브러리 설치 (실행은 하지 않음)
pip3 install --upgrade --user qrcode

echo -e "\n${GREEN}==========================================${NO_COLOR}"
echo -e "${GREEN}환경 설정이 완료되었습니다!${NO_COLOR}"
echo -e "${GREEN}==========================================${NO_COLOR}"
echo ""
echo "다음 명령어를 통해 빌드를 진행하세요:"
echo -e "  1. cd $PROJECT_ROOT/ros2_ws"
echo -e "  2. rosdep install --from-paths src --ignore-src -r -y"
echo -e "  3. colcon build --symlink-install"
echo ""
echo -e "참고: 'make_qr.py'는 환경 준비가 끝났으므로 필요할 때 직접 실행하시면 됩니다."