#!/usr/bin/env bash
#
# 로봇 안전 종료 — 젯슨을 끄는 표준 방법
#
# 설치 위치: /usr/local/bin/robot-off  (setup_remote_ops.sh 가 여기서 복사해 간다)
# 사용법:    ssh robot robot-off        <- PC 에서
#            robot-off                  <- 젯슨에서 직접
#
# 순서:  1) 모터 정지  2) 노드 종료  3) 종료 확인  4) 디스크 flush  5) 전원 차단
#
# 왜 필요한가: 이 젯슨은 SD 카드(/dev/mmcblk0p1)로 부팅한다. SD 카드는 SSD 보다
#              갑작스런 전원 차단에 훨씬 약하고, 실제로 한 번 깨진 적이 있다.
#              그리고 모터가 도는 중에 전원이 끊기면 로봇이 그대로 굴러간다.
#
# ※ 이 파일을 고쳤으면 젯슨에 다시 설치해야 반영된다:
#      ssh -tt robot 'cd ~/mobile_robot_proto_type && ./scripts/setup_remote_ops.sh'

set -uo pipefail   # -e 는 쓰지 않는다. 한 단계가 실패해도 종료까지는 가야 한다.

WS="$HOME/mobile_robot_proto_type"

# ── 1. 모터 정지 ──────────────────────────────────────────────────────────────
echo "[1/5] 모터 정지 명령 (/cmd_vel = 0)"
if [ -f /opt/ros/humble/setup.bash ]; then
  set +u
  source /opt/ros/humble/setup.bash
  [ -f "$WS/install/setup.bash" ] && source "$WS/install/setup.bash"
  set -u
  if timeout 6 ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
       "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" \
       --once >/dev/null 2>&1; then
    echo "      정지 명령 전송됨"
  else
    echo "      (노드 미실행으로 보임 — 건너뜀)"
  fi
else
  echo "      (ROS 미설치 — 건너뜀)"
fi

# ── 2. 노드 종료 ──────────────────────────────────────────────────────────────
# 노드 이름을 하나씩 나열하면 패키지가 늘 때마다 목록이 낡는다. 실제로 그렇게 돼서
# main_driver 가 빠져 있었다. 그래서 "이 워크스페이스에서 뜬 프로세스" 를 경로로
# 통째로 잡는다. 새 노드를 추가해도 이 스크립트를 고칠 필요가 없다.
echo "[2/5] ROS 노드 종료"

WS_PATTERN="$WS/install/"
# 워크스페이스 밖(apt)에서 오는 노드는 여기에만 적어두면 된다.
EXTERNAL_NODES="robot_state_publisher ekf_node cartographer_node cartographer_occupancy_grid_node joint_state_publisher"

kill_pattern() {   # $1=패턴  $2=표시이름
  local pids
  pids="$(pgrep -f "$1" 2>/dev/null | tr '\n' ' ')"
  [ -z "${pids// /}" ] && return 1
  echo "      TERM: $2 (pid: ${pids% })"
  pkill -TERM -f "$1" 2>/dev/null
  return 0
}

found=0
kill_pattern "$WS_PATTERN" "워크스페이스 노드" && found=1
for n in $EXTERNAL_NODES; do
  kill_pattern "$n" "$n" && found=1
done
[ "$found" -eq 0 ] && echo "      (실행 중인 노드 없음)"

# ── 3. 종료 확인 ──────────────────────────────────────────────────────────────
# SIGTERM 은 즉시가 아니다. 안 죽은 게 있으면 KILL 로 확실히 끝낸다.
echo "[3/5] 종료 확인"
for i in 1 2 3 4 5; do
  sleep 1
  remaining="$(pgrep -f "$WS_PATTERN" 2>/dev/null | wc -l)"
  [ "$remaining" -eq 0 ] && break
done

remaining="$(pgrep -f "$WS_PATTERN" 2>/dev/null | wc -l)"
if [ "$remaining" -gt 0 ]; then
  echo "      $remaining 개가 안 죽음 — KILL 로 강제 종료"
  pkill -KILL -f "$WS_PATTERN" 2>/dev/null
  sleep 1
fi
echo "      정리 완료"

# ── 4. 디스크 flush ───────────────────────────────────────────────────────────
echo "[4/5] 디스크 캐시 flush (sync)"
sync; sync

# ── 5. 전원 차단 ──────────────────────────────────────────────────────────────
echo "[5/5] 시스템 종료"
echo
echo "  ★ 전원 어댑터를 지금 뽑지 마세요."
echo "    LED 가 꺼지고 팬이 완전히 멈춘 뒤에 뽑아야 합니다."
echo "    PC 에서는 ping 이 끊기고 20초 더 기다리면 안전합니다."
echo "    (PC 에서 scripts/robot-off.ps1 로 끄면 이 대기를 자동으로 해줍니다)"
echo

sudo -n /usr/sbin/poweroff
