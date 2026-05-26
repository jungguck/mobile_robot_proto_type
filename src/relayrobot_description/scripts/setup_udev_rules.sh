#!/bin/bash
# =====================================================
#  로봇 USB 장치 udev 규칙 자동 설정 스크립트
#  실행 방법: sudo bash setup_udev_rules.sh
# =====================================================

RULES_FILE="/etc/udev/rules.d/99-robot-devices.rules"

echo "=========================================="
echo "  로봇 USB 장치 udev 규칙 설정 스크립트"
echo "=========================================="
echo ""
echo "라이다, IMU, 모터 컨트롤러를 모두 연결한 후 실행하세요."
echo ""

# 장치 정보 추출 함수
get_vendor()  { udevadm info -a -n "$1" 2>/dev/null | grep 'ATTRS{idVendor}'  | head -1 | grep -oP '"\K[^"]+'; }
get_product() { udevadm info -a -n "$1" 2>/dev/null | grep 'ATTRS{idProduct}' | head -1 | grep -oP '"\K[^"]+'; }
get_serial()  { udevadm info -a -n "$1" 2>/dev/null | grep 'ATTRS{serial}'    | head -1 | grep -oP '"\K[^"]+'; }

# 연결된 장치 목록 출력
echo "[ 현재 연결된 USB 시리얼 장치 ]"
echo "-------------------------------------------"
found=0
for dev in /dev/ttyUSB* /dev/ttyACM*; do
    [ -e "$dev" ] || continue
    vendor=$(get_vendor "$dev")
    product=$(get_product "$dev")
    serial=$(get_serial "$dev")
    echo "  $dev  |  vendor=$vendor  product=$product  serial=$serial"
    found=1
done

if [ $found -eq 0 ]; then
    echo "  USB 장치가 감지되지 않았습니다. 장치를 연결하고 다시 실행하세요."
    exit 1
fi
echo ""

# ---- 라이다 ----
echo "[ 1/3 ] 라이다(RPLidar)의 포트를 입력하세요 (예: /dev/ttyUSB0):"
read -r LIDAR_DEV
LIDAR_VENDOR=$(get_vendor "$LIDAR_DEV")
LIDAR_PRODUCT=$(get_product "$LIDAR_DEV")
LIDAR_SERIAL=$(get_serial "$LIDAR_DEV")

# ---- IMU ----
echo "[ 2/3 ] IMU(EB-IMU)의 포트를 입력하세요 (예: /dev/ttyUSB1):"
read -r IMU_DEV
IMU_VENDOR=$(get_vendor "$IMU_DEV")
IMU_PRODUCT=$(get_product "$IMU_DEV")
IMU_SERIAL=$(get_serial "$IMU_DEV")

# ---- 모터 ----
echo "[ 3/3 ] 모터 컨트롤러의 포트를 입력하세요 (예: /dev/ttyACM0):"
read -r MOTOR_DEV
MOTOR_VENDOR=$(get_vendor "$MOTOR_DEV")
MOTOR_PRODUCT=$(get_product "$MOTOR_DEV")
MOTOR_SERIAL=$(get_serial "$MOTOR_DEV")

echo ""
echo "[ 감지된 장치 정보 ]"
echo "  라이다  : vendor=$LIDAR_VENDOR  product=$LIDAR_PRODUCT  serial=$LIDAR_SERIAL"
echo "  IMU     : vendor=$IMU_VENDOR    product=$IMU_PRODUCT    serial=$IMU_SERIAL"
echo "  모터    : vendor=$MOTOR_VENDOR  product=$MOTOR_PRODUCT  serial=$MOTOR_SERIAL"
echo ""

# 라이다와 IMU가 같은 vendor/product일 때 serial로 구분 (CP2102 칩 중복 방지)
if [ "$LIDAR_VENDOR" = "$IMU_VENDOR" ] && [ "$LIDAR_PRODUCT" = "$IMU_PRODUCT" ]; then
    echo "  라이다와 IMU가 같은 USB 칩을 사용합니다. 시리얼 번호로 구분합니다."
    LIDAR_RULE='KERNEL=="ttyUSB*", ATTRS{idVendor}=="'"$LIDAR_VENDOR"'", ATTRS{idProduct}=="'"$LIDAR_PRODUCT"'", ATTRS{serial}=="'"$LIDAR_SERIAL"'", MODE:="0666", SYMLINK+="rplidar"'
    IMU_RULE='KERNEL=="ttyUSB*", ATTRS{idVendor}=="'"$IMU_VENDOR"'", ATTRS{idProduct}=="'"$IMU_PRODUCT"'", ATTRS{serial}=="'"$IMU_SERIAL"'", MODE:="0666", SYMLINK+="ttyimu"'
else
    LIDAR_RULE='KERNEL=="ttyUSB*", ATTRS{idVendor}=="'"$LIDAR_VENDOR"'", ATTRS{idProduct}=="'"$LIDAR_PRODUCT"'", MODE:="0666", SYMLINK+="rplidar"'
    IMU_RULE='KERNEL=="ttyUSB*", ATTRS{idVendor}=="'"$IMU_VENDOR"'", ATTRS{idProduct}=="'"$IMU_PRODUCT"'", MODE:="0666", SYMLINK+="ttyimu"'
fi

if [ -n "$MOTOR_SERIAL" ]; then
    MOTOR_RULE='KERNEL=="ttyACM*", ATTRS{idVendor}=="'"$MOTOR_VENDOR"'", ATTRS{idProduct}=="'"$MOTOR_PRODUCT"'", ATTRS{serial}=="'"$MOTOR_SERIAL"'", MODE:="0666", SYMLINK+="motor"'
else
    MOTOR_RULE='KERNEL=="ttyACM*", ATTRS{idVendor}=="'"$MOTOR_VENDOR"'", ATTRS{idProduct}=="'"$MOTOR_PRODUCT"'", MODE:="0666", SYMLINK+="motor"'
fi

# 규칙 파일 작성
cat > "$RULES_FILE" <<EOF
# 로봇 USB 장치 udev 규칙
# 자동 생성: $(date)
# 재생성: sudo bash $(realpath "$0")

# RPLidar → /dev/rplidar
$LIDAR_RULE

# EB-IMU → /dev/ttyimu
$IMU_RULE

# 모터 컨트롤러 → /dev/motor
$MOTOR_RULE
EOF

echo "[ 규칙 파일 생성 완료: $RULES_FILE ]"
echo ""

# udev 재로드
udevadm control --reload-rules
udevadm trigger
echo "[ udev 규칙 적용 완료 ]"
echo ""
echo "장치를 한 번 뽑았다가 다시 꽂으면 심볼릭 링크가 자동 생성됩니다:"
echo "  /dev/rplidar  → 라이다"
echo "  /dev/ttyimu   → IMU"
echo "  /dev/motor    → 모터 컨트롤러"
echo ""
echo "확인 명령: ls -la /dev/rplidar /dev/ttyimu /dev/motor"
