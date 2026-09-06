#!/usr/bin/env bash
#
# 원격 운용 준비 — USB 사전 등록 + 안전 종료
#
# 사용법:  ./scripts/setup_remote_ops.sh      <- sudo 없이 실행
#
# 하는 일:
#   1. udev 규칙 설치  → USB 를 아직 안 꽂았어도, 나중에 아무 순서로 꽂아도 바로 인식
#   2. 원격 종료 권한  → PC 에서 비밀번호 없이 `ssh robot robot-off` 로 안전하게 끔
#   3. robot-off 설치  → 모터 정지 → 노드 종료 → sync → 전원 차단 순서 보장
#   4. 파일시스템 점검 → SD 카드 저널 상태 확인
#
# 왜 필요한가: 이 젯슨은 SD 카드(/dev/mmcblk0p1)로 부팅한다. SD 카드는 SSD 보다
#              갑작스런 전원 차단에 훨씬 약하다. 실제로 전원을 그냥 꺼서 저장장치가
#              깨진 적이 있다. 반드시 종료 절차를 밟고 전원을 뽑아야 한다.
#
# ROS 설치(setup_humble.sh)와 분리돼 있다. 이 스크립트는 ROS 가 없어도 동작하고,
# 장치를 교체하거나 규칙을 고쳤을 때 다시 실행하면 된다.

set -euo pipefail

RED='\033[0;31m'; GRN='\033[0;32m'; YLW='\033[1;33m'; BLU='\033[0;34m'; NC='\033[0m'
step() { echo -e "\n${BLU}==> $*${NC}"; }
ok()   { echo -e "${GRN}  OK  $*${NC}"; }
warn() { echo -e "${YLW}  !!  $*${NC}"; }
die()  { echo -e "${RED}  XX  $*${NC}" >&2; exit 1; }

[ "$(id -u)" -eq 0 ] && die "sudo 없이 일반 사용자로 실행하세요."

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
TARGET_USER="$USER"

step "sudo 권한 확인 (비밀번호를 한 번 입력하세요)"
sudo -v || die "sudo 실패"
( while true; do sudo -n true; sleep 50; kill -0 "$$" 2>/dev/null || exit; done ) 2>/dev/null &
KEEPALIVE=$!
trap 'kill "$KEEPALIVE" 2>/dev/null || true' EXIT

# ── 1. udev 규칙 ──────────────────────────────────────────────────────────────
step "1/4  udev 규칙 설치 (USB 를 꽂기 전에 미리 등록)"
RULES="$REPO/src/relayrobot_description/scripts/99-robot-devices.rules"
[ -f "$RULES" ] || die "규칙 파일을 찾을 수 없습니다: $RULES"

sudo cp "$RULES" /etc/udev/rules.d/99-robot-devices.rules
sudo udevadm control --reload-rules
sudo udevadm trigger
ok "규칙 설치 완료 → /etc/udev/rules.d/99-robot-devices.rules"
echo "      모터   1a86:55d3                  -> /dev/motor"
echo "      라이다 10c4:ea60 (CP2102N)        -> /dev/rplidar"
echo "      IMU    10c4:ea60 (CP2102)         -> /dev/ttyimu"
echo "      이제 USB 를 아무 순서로 꽂아도 위 이름으로 잡힙니다."

# 시리얼 포트 접근 권한
if id -nG "$TARGET_USER" | tr ' ' '\n' | grep -x dialout >/dev/null 2>&1; then
  ok "$TARGET_USER 는 이미 dialout 그룹 소속"
else
  sudo usermod -aG dialout "$TARGET_USER"
  warn "$TARGET_USER 를 dialout 그룹에 추가 — 적용하려면 재로그인(또는 재부팅) 필요"
fi

# ── 2. 원격 종료 권한 ─────────────────────────────────────────────────────────
step "2/4  원격 종료 권한 (비밀번호 없이 '종료만' 허용)"
SUDOERS_TMP="$(mktemp)"
{
  echo "# 원격(SSH)에서 비밀번호 없이 안전 종료할 수 있도록 허용한다."
  echo "# 전원 차단으로 SD 카드가 깨지는 것을 막기 위한 조치이며,"
  echo "# 허용 범위는 종료/재시작 명령으로만 한정한다."
  echo "$TARGET_USER ALL=(root) NOPASSWD: /usr/sbin/poweroff, /usr/sbin/reboot, /usr/sbin/shutdown, /usr/bin/systemctl poweroff, /usr/bin/systemctl reboot"
} > "$SUDOERS_TMP"

# sudoers 는 문법이 틀리면 sudo 자체가 막힌다. 반드시 검증한 뒤에만 설치한다.
if sudo visudo -cf "$SUDOERS_TMP" >/dev/null 2>&1; then
  sudo install -m 0440 -o root -g root "$SUDOERS_TMP" /etc/sudoers.d/010-robot-poweroff
  rm -f "$SUDOERS_TMP"
  ok "설치 완료 → /etc/sudoers.d/010-robot-poweroff (문법 검증 통과)"
else
  rm -f "$SUDOERS_TMP"
  die "sudoers 문법 검증 실패 — 설치하지 않았습니다 (기존 sudo 설정은 그대로입니다)"
fi

# ── 3. robot-off 헬퍼 ─────────────────────────────────────────────────────────
step "3/4  robot-off 설치 (안전 종료 명령)"
SRC="$REPO/scripts/robot-off.sh"
[ -f "$SRC" ] || die "종료 스크립트를 찾을 수 없습니다: $SRC"

# 실행 전에 문법부터 본다. 깨진 스크립트를 설치하면 정작 끌 때 못 끈다.
bash -n "$SRC" || die "robot-off.sh 문법 오류 — 설치하지 않았습니다"

# 저장소가 윈도우 PC 를 거치면 CRLF 로 체크아웃될 수 있고, 그러면 셰뱅이 깨져서
# 정작 끌 때 robot-off 가 실행되지 않는다. 조건을 따지지 말고 항상 LF 로 정규화한다
# (이미 LF 면 tr 이 아무것도 하지 않는다).
ROBOTOFF_TMP="$(mktemp)"
tr -d '\r' < "$SRC" > "$ROBOTOFF_TMP"
sudo install -m 0755 -o root -g root "$ROBOTOFF_TMP" /usr/local/bin/robot-off
rm -f "$ROBOTOFF_TMP"

bash -n /usr/local/bin/robot-off || die "설치된 robot-off 문법 오류"
ok "설치 완료 → /usr/local/bin/robot-off (원본: scripts/robot-off.sh)"

# ── 4. 파일시스템 점검 ────────────────────────────────────────────────────────
step "4/4  SD 카드 파일시스템 점검"
ROOTDEV="$(findmnt -no SOURCE /)"
echo "      루트 장치: $ROOTDEV ($(findmnt -no FSTYPE /))"
if sudo tune2fs -l "$ROOTDEV" 2>/dev/null | grep -q "has_journal"; then
  ok "ext4 저널 활성 — 전원이 갑자기 끊겨도 복구 가능성이 높습니다"
else
  warn "ext4 저널이 없습니다 — 전원 차단에 매우 취약합니다"
fi
STATE="$(sudo tune2fs -l "$ROOTDEV" 2>/dev/null | awk -F: '/Filesystem state/{gsub(/ /,"",$2);print $2}')"
case "$STATE" in
  clean) ok "파일시스템 상태: clean" ;;
  "")    warn "파일시스템 상태를 읽지 못했습니다" ;;
  *)     warn "파일시스템 상태: $STATE — 재부팅 시 fsck 를 돌리세요" ;;
esac

# ── 검증 ──────────────────────────────────────────────────────────────────────
step "검증"
echo "  무비밀번호로 허용된 명령:"
sudo -l 2>/dev/null | grep -i "NOPASSWD" | sed 's/^/    /' || echo "    (확인 실패)"
echo "  robot-off: $(command -v robot-off || echo '없음')"
echo "  udev 규칙: $(ls /etc/udev/rules.d/99-robot-devices.rules 2>/dev/null || echo '없음')"

echo
echo -e "${GRN}================================================================${NC}"
echo -e "${GRN} 원격 운용 준비 완료${NC}"
echo -e "${GRN}================================================================${NC}"
cat <<'DONE'

■ USB 연결
    이제 라이다·IMU·모터를 아무 순서로 꽂아도 됩니다. 꽂은 뒤 확인:
        ~/mobile_robot_proto_type/check_devices.sh

■ 원격 종료 (PC 에서)
    PowerShell:  .\scripts\robot-off.ps1      <- ping 끊길 때까지 자동 대기
obot-off.ps1      <- ping 끊길 때까지 자동 대기
    수동:        ssh robot robot-off

    ★ 절대 그냥 전원을 뽑지 마세요. 이 젯슨은 SD 카드로 부팅합니다.
      LED 가 꺼지고 팬이 멈춘 뒤에 어댑터를 뽑으세요.

DONE
