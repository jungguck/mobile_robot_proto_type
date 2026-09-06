#!/usr/bin/env bash
#
# ROS 2 Humble 설치 + 프로젝트 의존성 셋업 (Jetson Orin Nano / JetPack 6.x / Ubuntu 22.04 arm64)
#
# 사용법:  ./scripts/setup_humble.sh          <- sudo 없이 실행 (스크립트가 내부에서 sudo 호출)
#
# 이 스크립트가 하는 일:
#   1. 환경 검증 (jammy / arm64 인지)
#   2. ROS 2 apt 저장소 등록 (GPG 키 + jammy 소스)
#   3. ros-humble-desktop + 프로젝트 필수 ROS 패키지 설치
#   4. 빌드 도구(colcon, rosdep) 설치 + rosdep 초기화
#   5. Python 의존성 설치 (cvxpy / polytope / osqp / cvxopt / transforms3d / pyserial)
#   6. dialout 그룹 등록 (시리얼 포트 접근 권한)
#
# 워크스페이스 빌드와 udev 규칙 적용은 별도 단계입니다. (README 참고)

set -euo pipefail

RED='\033[0;31m'; GRN='\033[0;32m'; YLW='\033[1;33m'; BLU='\033[0;34m'; NC='\033[0m'
step() { echo -e "\n${BLU}==> $*${NC}"; }
ok()   { echo -e "${GRN}  OK  $*${NC}"; }
warn() { echo -e "${YLW}  !!  $*${NC}"; }
die()  { echo -e "${RED}  XX  $*${NC}" >&2; exit 1; }

[ "$(id -u)" -eq 0 ] && die "sudo 없이 일반 사용자로 실행하세요. (스크립트가 필요할 때 알아서 sudo 를 씁니다)"

# ── 1. 환경 검증 ──────────────────────────────────────────────────────────────
step "1/6  환경 검증"
. /etc/os-release
[ "${VERSION_CODENAME:-}" = "jammy" ] || die "Ubuntu 22.04(jammy) 가 아닙니다: ${VERSION_CODENAME:-unknown}. Humble 은 jammy 전용입니다."
ARCH="$(dpkg --print-architecture)"
ok "Ubuntu ${VERSION_ID} (${VERSION_CODENAME}) / ${ARCH}"
[ -f /etc/nv_tegra_release ] && ok "$(head -1 /etc/nv_tegra_release | cut -d, -f1-2)"

if [ -d /opt/ros/humble ]; then
  warn "/opt/ros/humble 이 이미 있습니다. 이후 단계는 갱신/보완으로 동작합니다."
fi

step "sudo 권한 확인 (비밀번호를 한 번 입력하세요)"
sudo -v || die "sudo 실패"
# 스크립트가 도는 동안 sudo 타임스탬프를 계속 갱신
( while true; do sudo -n true; sleep 50; kill -0 "$$" 2>/dev/null || exit; done ) 2>/dev/null &
SUDO_KEEPALIVE_PID=$!
trap 'kill "$SUDO_KEEPALIVE_PID" 2>/dev/null || true' EXIT

# ── 2. ROS 2 apt 저장소 등록 ──────────────────────────────────────────────────
step "2/6  ROS 2 apt 저장소 등록"
sudo apt-get update -qq
sudo apt-get install -y -qq curl gnupg ca-certificates software-properties-common
sudo add-apt-repository -y universe >/dev/null

KEYRING=/usr/share/keyrings/ros-archive-keyring.gpg
if [ ! -f "$KEYRING" ]; then
  curl -fsSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    | sudo gpg --dearmor -o "$KEYRING"
  ok "GPG 키 설치: $KEYRING"
else
  ok "GPG 키 이미 존재"
fi

echo "deb [arch=${ARCH} signed-by=${KEYRING}] http://packages.ros.org/ros2/ubuntu jammy main" \
  | sudo tee /etc/apt/sources.list.d/ros2.list >/dev/null
ok "저장소 등록: /etc/apt/sources.list.d/ros2.list"

sudo apt-get update -qq
# 주의: `cmd | grep -q` 는 pipefail 과 함께 쓰면 SIGPIPE(141) 로 오작동한다.
#       반드시 변수에 담은 뒤 검사할 것.
CAND="$(apt-cache policy ros-humble-desktop | awk '/Candidate:/{print $2}')"
case "$CAND" in
  ""|"(none)") die "ros-humble-desktop 을 찾을 수 없습니다. 저장소 등록을 확인하세요." ;;
esac
ok "ros-humble-desktop 후보: $CAND"

# ── 3. ROS 패키지 설치 ────────────────────────────────────────────────────────
step "3/6  ROS 2 Humble + 프로젝트 필수 패키지 설치 (수 분~십수 분 소요)"
sudo apt-get install -y \
  ros-humble-desktop \
  ros-humble-robot-localization \
  ros-humble-cartographer-ros \
  ros-humble-tf-transformations \
  ros-humble-teleop-twist-keyboard \
  ros-humble-nav2-map-server \
  ros-humble-xacro \
  ros-humble-robot-state-publisher \
  ros-humble-joint-state-publisher \
  ros-humble-foxglove-bridge
ok "ROS 패키지 설치 완료"
# xacro: real_robot_260519.launch.py 가 `import xacro` 로 URDF 를 파싱한다 (desktop 에 미포함)
# foxglove-bridge: 젯슨에서 RViz 를 띄우지 않고 PC 로 시각화를 넘기기 위한 원격 운용 필수품

# ── 4. 빌드 도구 ──────────────────────────────────────────────────────────────
step "4/6  빌드 도구(colcon / rosdep) 설치"
sudo apt-get install -y \
  python3-colcon-common-extensions \
  python3-rosdep \
  python3-pip \
  build-essential \
  git \
  tmux
# tmux: 원격(SSH) 운용 시 필수 — 무선이 끊겨도 ROS 노드가 안 죽는다

if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
  sudo rosdep init
  ok "rosdep 초기화"
else
  ok "rosdep 이미 초기화됨"
fi
rosdep update || warn "rosdep update 실패 (네트워크 문제일 수 있음 — 나중에 다시 실행하세요)"

# ── 5. Python 의존성 ──────────────────────────────────────────────────────────
step "5/6  Python 의존성 설치 (MPC 용)"
# 22.04 는 시스템 pip 이 보호돼 있지 않으므로 --break-system-packages 불필요
python3 -m pip install --user --upgrade pip
python3 -m pip install --user \
  numpy scipy cvxpy polytope osqp cvxopt transforms3d pyserial
ok "Python 의존성 설치 완료"

# ── 6. 시리얼 포트 권한 ───────────────────────────────────────────────────────
step "6/6  시리얼 포트 접근 권한(dialout 그룹)"
USER_GROUPS=" $(id -nG "$USER") "
if [ "${USER_GROUPS#* dialout }" != "$USER_GROUPS" ]; then
  ok "$USER 는 이미 dialout 그룹 소속"
else
  sudo usermod -aG dialout "$USER"
  warn "$USER 를 dialout 그룹에 추가했습니다 — 적용하려면 재로그인(또는 재부팅) 필요"
fi

# ── 마무리 ────────────────────────────────────────────────────────────────────
echo
echo -e "${GRN}================================================================${NC}"
echo -e "${GRN} ROS 2 Humble 셋업 완료${NC}"
echo -e "${GRN}================================================================${NC}"
echo
echo "다음 단계:"
echo "  1) 워크스페이스 빌드:"
echo "       source /opt/ros/humble/setup.bash"
echo "       cd ~/mobile_robot_proto_type"
echo "       colcon build --symlink-install --parallel-workers 2"
echo
echo "  2) alias 등록 (README 2절):"
echo "       echo 'alias ros_setup=\"source /opt/ros/humble/setup.bash && source ~/mobile_robot_proto_type/install/setup.bash && echo ROS2 환경 로드 완료\"' >> ~/.bashrc"
echo
echo "  3) 하드웨어 연결 후 udev 규칙 적용 (README 3절 Step 2):"
echo "       sudo cp ~/mobile_robot_proto_type/src/relayrobot_description/scripts/99-robot-devices.rules /etc/udev/rules.d/"
echo "       sudo udevadm control --reload-rules && sudo udevadm trigger"
echo "       ~/mobile_robot_proto_type/check_devices.sh"
echo
