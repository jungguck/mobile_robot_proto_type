#!/usr/bin/env bash
#
# 로봇 노드 상시 기동 — tmux 세션 안에서 띄운다
#
# 설치 위치: 저장소 안에서 바로 실행 (젯슨)
# 사용법:
#     ssh robot '~/mobile_robot_proto_type/scripts/robot-up.sh'          # 센서만 (기본)
#     ssh robot '~/mobile_robot_proto_type/scripts/robot-up.sh full'     # 메인 launch 전체
#     ssh robot '~/mobile_robot_proto_type/scripts/robot-up.sh stop'     # 전부 종료
#     ssh robot '~/mobile_robot_proto_type/scripts/robot-up.sh status'   # 상태만 확인
#
#     ssh -t robot 'tmux attach -t robot'    # 화면 직접 붙어서 로그 보기 (Ctrl-b d 로 빠져나옴)
#
# 왜 tmux 인가: ssh 로 그냥 띄우면 연결이 끊기는 순간 노드가 통째로 죽는다. 무선은
#              반드시 끊긴다. tmux 안에서 돌리면 SSH 가 끊겨도 노드는 계속 산다.
#              (nohup 도 살긴 하지만 로그를 다시 볼 수가 없다. tmux 는 붙어서 볼 수 있다.)
#
# 모드:
#   sensors (기본) — 라이다 + IMU 만. 모터를 건드리지 않으므로 로봇이 움직일 일이 없다.
#   full           — real_robot_260519.launch.py 전체 (모터 드라이버 + EKF + robot_state_publisher)
#                    ★ 모터가 연결돼 있으면 /cmd_vel 에 따라 실제로 움직인다. 주변을 확인할 것.

set -uo pipefail

SESSION="robot"
WS="$HOME/mobile_robot_proto_type"
MODE="${1:-sensors}"

GRN='\033[0;32m'; YLW='\033[1;33m'; RED='\033[0;31m'; BLU='\033[0;34m'; NC='\033[0m'
step() { echo -e "\n${BLU}==> $*${NC}"; }
ok()   { echo -e "${GRN}  OK  $*${NC}"; }
warn() { echo -e "${YLW}  !!  $*${NC}"; }
die()  { echo -e "${RED}  XX  $*${NC}" >&2; exit 1; }

# ROS 환경. tmux 안의 셸은 .bashrc 를 타지만, 이 스크립트 자신도 필요하다.
set +u
source /opt/ros/humble/setup.bash
[ -f "$WS/install/setup.bash" ] || die "워크스페이스가 빌드되지 않았습니다. colcon build 를 먼저 하세요."
source "$WS/install/setup.bash"
set -u

# ── status ────────────────────────────────────────────────────────────────────
show_status() {
  step "상태"
  if tmux has-session -t "$SESSION" 2>/dev/null; then
    ok "tmux 세션 '$SESSION' 살아있음"
    tmux list-windows -t "$SESSION" -F "      [#{window_index}] #{window_name}  (#{window_panes} pane)"
  else
    warn "tmux 세션 '$SESSION' 없음"
  fi

  echo "  실행 중인 노드:"
  pgrep -af "$WS/install/[^ ]*/lib/" | sed "s|$WS|~|" | sed 's/^/      /' || echo "      (없음)"

  echo "  발행 중인 토픽:"
  for t in /scan /ebimu_data /odom_raw /odometry/filtered /joint_states; do
    # 타임아웃이 짧으면 10Hz 토픽의 첫 샘플을 놓쳐 "안 나온다" 고 오해하게 된다.
    hz=$(timeout 8 ros2 topic hz "$t" 2>/dev/null | grep -oE "average rate: [0-9.]+" | head -1 | grep -oE "[0-9.]+")
    if [ -n "$hz" ]; then
      printf "      %-22s %6.1f Hz\n" "$t" "$hz"
    fi
  done
}

# ── stop ──────────────────────────────────────────────────────────────────────
if [ "$MODE" = "stop" ]; then
  step "노드 종료"
  tmux kill-session -t "$SESSION" 2>/dev/null && ok "tmux 세션 종료" || warn "tmux 세션이 없었음"
  # tmux 밖에서 뜬 것(nohup 등)도 정리한다
  pkill -TERM -f "$WS/install/[^ ]*/lib/" 2>/dev/null && ok "남은 노드 정리" || true
  sleep 2
  pkill -KILL -f "$WS/install/[^ ]*/lib/" 2>/dev/null || true
  ok "완료"
  exit 0
fi

if [ "$MODE" = "status" ]; then
  show_status
  exit 0
fi

# ── 기동 ──────────────────────────────────────────────────────────────────────
[ "$MODE" = "sensors" ] || [ "$MODE" = "full" ] || die "모드는 sensors | full | stop | status 중 하나입니다 (받은 값: $MODE)"

step "기존 노드 정리 (중복 기동 방지)"
tmux kill-session -t "$SESSION" 2>/dev/null && ok "이전 tmux 세션 종료" || true
pkill -TERM -f "$WS/install/[^ ]*/lib/" 2>/dev/null && { ok "이전 노드 종료"; sleep 2; } || true

# 장치 확인 — 없는 채로 띄우면 노드가 조용히 죽고 원인 찾기가 번거롭다
step "장치 확인"
for d in /dev/rplidar /dev/ttyimu; do
  [ -e "$d" ] && ok "$d -> $(readlink -f $d)" || die "$d 가 없습니다. USB 연결과 udev 규칙을 확인하세요."
done
if [ "$MODE" = "full" ]; then
  [ -e /dev/motor ] && ok "/dev/motor -> $(readlink -f /dev/motor)" || die "/dev/motor 가 없습니다."
fi

ENV_SETUP="source /opt/ros/humble/setup.bash && source $WS/install/setup.bash"

step "tmux 세션 '$SESSION' 생성 (모드: $MODE)"
tmux new-session -d -s "$SESSION" -n shell -c "$WS"

if [ "$MODE" = "sensors" ]; then
  # 라이다 — ★ serial_port 를 반드시 넘긴다. launch 기본값 /dev/ttyUSB0 는 IMU 다.
  # ★ 이 라이다는 A1 이 아니라 S2 계열(1000000 보드)이다. a1 launch 로는 타임아웃으로 죽는다.
  tmux new-window -t "$SESSION" -n lidar -c "$WS" \
    "$ENV_SETUP && ros2 launch sllidar_ros2 sllidar_s2_launch.py serial_port:=/dev/rplidar; exec bash"
  tmux new-window -t "$SESSION" -n imu -c "$WS" \
    "$ENV_SETUP && ros2 run ebimu_pkg ebimu_publisher; exec bash"
  ok "lidar / imu 창 생성"
else
  tmux new-window -t "$SESSION" -n robot -c "$WS" \
    "$ENV_SETUP && ros2 launch relayrobot_description real_robot_260519.launch.py; exec bash"
  ok "robot 창 생성 (메인 launch)"
  warn "모터가 연결돼 있습니다. /cmd_vel 을 보내면 실제로 움직입니다."
fi

step "기동 대기 (IMU 캘리브레이션 10초 포함)"
sleep 14

show_status

cat <<DONE

──────────────────────────────────────────────────────────────
 노드가 tmux 안에서 계속 돌고 있습니다. SSH 를 끊어도 안 죽습니다.
──────────────────────────────────────────────────────────────

  로그 직접 보기 :  ssh -t robot 'tmux attach -t robot'
                    (빠져나올 때 Ctrl-b 누르고 d — Ctrl-c 로 끄지 말 것)
  창 이동         :  Ctrl-b 다음 숫자 (0=shell, 1=lidar, 2=imu)
  상태만 확인     :  ssh robot '~/mobile_robot_proto_type/scripts/robot-up.sh status'
  종료            :  ssh robot '~/mobile_robot_proto_type/scripts/robot-up.sh stop'

DONE
