#!/bin/bash
set -euo pipefail

# 1. Path Configuration
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$SCRIPT_DIR"
REPOS_FILE="$PROJECT_ROOT/dependencies.repos"
WS_ROOT="$PROJECT_ROOT/ros2_ws"

YELLOW='\033[1;33m'
GREEN='\033[0;32m'
RED='\033[0;31m'
NO_COLOR='\033[0m'

# 3. Check for Required Tools (Assumed to be pre-installed)
require_cmd() {
  if ! command -v "$1" &> /dev/null; then
    echo -e "${RED}Error: Command '$1' not found. (Please run: sudo apt install $1)${NO_COLOR}"
    exit 1
  fi
}
require_cmd vcs
require_cmd git
require_cmd python3
require_cmd pip3

echo -e "\n${YELLOW}[1/3] Preparing directories...${NO_COLOR}"
mkdir -p "$WS_ROOT"

echo -e "\n${YELLOW}[2/3] Fetching external repositories (vcs import)...${NO_COLOR}"
if [ ! -f "$REPOS_FILE" ]; then
  echo -e "${RED}Error: Repository configuration file missing: $REPOS_FILE${NO_COLOR}"
  exit 1
fi

# Install packages into ros2_ws/external/ as defined in dependencies.repos
# Run within WS_ROOT to avoid nested path issues
vcs import "$WS_ROOT" < "$REPOS_FILE"

echo -e "\n${YELLOW}[3/3] Installing Python dependencies...${NO_COLOR}"
# Use --user option to install without sudo
pip3 install --upgrade --user qrcode

echo -e "\n${GREEN}==========================================${NO_COLOR}"
echo -e "${GREEN}External Environment setup completed!${NO_COLOR}"
echo -e "${GREEN}Location: $WS_ROOT/external${NO_COLOR}"
echo -e "==========================================${NO_COLOR}"