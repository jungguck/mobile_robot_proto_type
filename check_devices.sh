#!/usr/bin/env bash
# USB 기기 연결 확인 — 라이다 / 모터드라이버 / IMU
# 사용법: ./check_devices.sh
#
# 동작: udev 심볼릭링크(/dev/rplidar, /dev/motor, /dev/ttyimu)를 먼저 확인하고,
#       링크가 없으면 실제 USB 칩(vid:pid + product 문자열)으로 기기를 찾아 보고한다.
#
# 칩 매핑 (정본: src/relayrobot_description/scripts/99-robot-devices.rules):
#   - 모터  : QinHeng USB Single Serial (1a86:55d3) — 시스템에서 유일, 보통 /dev/ttyACM0
#   - 라이다: Silicon Labs CP2102N (10c4:ea60, product "CP2102N ...") — /dev/ttyUSB*
#   - IMU   : Silicon Labs CP2102  (10c4:ea60, product "CP2102 ...")  — /dev/ttyUSB*
#     ※ 라이다·IMU는 같은 칩(10c4:ea60)이라 반드시 product 문자열로 구분한다.
set -u

MOTOR_ID="1a86:55d3"   # 모터드라이버 (QinHeng USB Single Serial)
CP210X="10c4:ea60"     # 라이다 & IMU (동일 칩 CP210x — product 문자열로 구분)

# match <node> <vid:pid> : 해당 tty 노드가 주어진 vid:pid 인지 확인
match() {
  local props; props=$(udevadm info -q property -n "$1" 2>/dev/null)
  echo "$props" | grep -q "ID_VENDOR_ID=${2%%:*}" && \
  echo "$props" | grep -q "ID_MODEL_ID=${2##*:}"
}

# has_product <node> <문자열> : 장치 트리에서 ATTRS{product} 에 문자열이 있는지 확인
has_product() {
  udevadm info -a -n "$1" 2>/dev/null | grep -q "ATTRS{product}==\"$2"
}

echo "===== 모바일 로봇 USB 기기 연결 확인 ====="
echo
result=0

# --- 라이다 (CP2102N) ---
LIDAR=""
if [ -e /dev/rplidar ]; then
  echo "✅ 라이다      : 연결됨  (/dev/rplidar → $(readlink -f /dev/rplidar))"
else
  for d in /dev/ttyUSB*; do
    [ -e "$d" ] && match "$d" "$CP210X" && has_product "$d" "CP2102N" && { LIDAR="$d"; break; }
  done
  if [ -n "$LIDAR" ]; then
    echo "⚠️  라이다      : 연결됨($LIDAR) — 심볼릭링크 /dev/rplidar 없음 (udev 규칙 재적용 필요)"
  else
    echo "❌ 라이다      : 연결 안 됨"; result=1
  fi
fi

# --- IMU (CP2102, 구형) : CP2102N(라이다)와 구분하려고 product 문자열까지 확인 ---
IMU=""
if [ -e /dev/ttyimu ]; then
  echo "✅ IMU         : 연결됨  (/dev/ttyimu → $(readlink -f /dev/ttyimu))"
else
  for d in /dev/ttyUSB*; do
    [ -e "$d" ] || continue
    match "$d" "$CP210X" && has_product "$d" "CP2102 " && { IMU="$d"; break; }
  done
  if [ -n "$IMU" ]; then
    echo "⚠️  IMU         : 연결됨($IMU) — 심볼릭링크 /dev/ttyimu 없음 (udev 규칙 재적용 필요)"
  else
    echo "❌ IMU         : 연결 안 됨"; result=1
  fi
fi

# --- 모터드라이버 (QinHeng 1a86:55d3, 보통 ttyACM0) ---
MOTOR=""
if [ -e /dev/motor ]; then
  echo "✅ 모터드라이버: 연결됨  (/dev/motor → $(readlink -f /dev/motor))"
else
  for d in /dev/ttyACM* /dev/ttyUSB*; do
    [ -e "$d" ] && match "$d" "$MOTOR_ID" && { MOTOR="$d"; break; }
  done
  if [ -n "$MOTOR" ]; then
    echo "⚠️  모터드라이버: 연결됨($MOTOR) — 심볼릭링크 /dev/motor 없음 (udev 규칙 재적용 필요)"
  else
    echo "❌ 모터드라이버: 연결 안 됨"; result=1
  fi
fi

echo
if [ "$result" -eq 0 ]; then
  echo "결과: 3개 기기 모두 연결 확인됨 ✅"
else
  echo "결과: 일부 기기 누락 ❌  (연결/전원 확인 후, udev 규칙 재적용: Step 2 참고)"
fi
exit $result
