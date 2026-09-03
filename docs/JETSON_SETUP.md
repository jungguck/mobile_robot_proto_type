# Jetson Orin Nano 셋업 + 원격 개발 환경

**작성:** 2026-09-03 · **대상:** Jetson Orin Nano 개발자 키트 + Windows 11 PC

---

## 0. 결론 먼저

| | JetPack 5.1.x | **JetPack 6.2.1 (목표)** |
|---|---|---|
| Jetson Linux | L4T 35.x | L4T 36.4.x |
| **Ubuntu** | 20.04 | **22.04** |
| **ROS 2** | **Foxy** — 2023-05 EOL | **Humble** — LTS, 2027-05 까지 |
| 6개 패키지 빌드 | ✅ | ✅ |
| 실기 파이프라인 | ✅ | ✅ |
| `gazebo.launch.py` | ✅ | ✅ |
| Super 모드 (MAXN) | ❌ | ✅ |
| ROS apt 설치 난이도 | 높음 (EOL + 키 로테이션) | 낮음 |

> **JetPack 6.2.1 + Ubuntu 22.04 + ROS 2 Humble 이 최종 목표다.**
> 모든 항목에서 우위이고, 이 저장소도 Humble 을 전부 지원한다. Foxy 를 고를 이유가 없다.
>
> **JetPack 5.1.3 은 목적지가 아니라 QSPI 펌웨어를 올리기 위한 경유지(bridge)일 뿐이다.**
> 부팅 실패 원인과 bridge 절차는 `docs/DEBUG_LOG_2026-09-03.md` 4절 참고.

### JetPack ↔ Ubuntu ↔ ROS 2 대응
```
JetPack 5.1.1 / 5.1.3  →  L4T 35.x  →  Ubuntu 20.04  →  ROS 2 Foxy    (EOL)
JetPack 6.0 ~ 6.2.x    →  L4T 36.x  →  Ubuntu 22.04  →  ROS 2 Humble  ← 여기
```
JetPack 6 은 24.04 가 아니라 **22.04** 다. 따라서 Jazzy 가 아니라 **Humble** 이고,
README 호환표에서 Humble 은 Gazebo 시뮬레이션까지 ✅ 다. (Jazzy 만 ❌)

---

## 1. 부팅시키기

`docs/DEBUG_LOG_2026-09-03.md` 4절의 순서대로:

1. JetPack 5.1.3 SD 이미지로 부팅 (펌웨어가 구버전이면 6.2.1 은 UEFI Shell 로 떨어진다)
2. `sudo reboot` × 2 + `nvidia-l4t-jetson-orin-nano-qspi-updater` 로 QSPI 펌웨어 갱신
3. JetPack 6.2.1 SD 카드로 교체 → 정상 부팅
4. 아래 2절부터 진행

> ⚠️ 펌웨어 업데이트 중 전원 차단 금지. **19V 정품 어댑터** 사용.

---

## 2. 젯슨에서 준비할 것 (첫 부팅 후, 순서대로)

### 2-1. 기본 시스템
```bash
sudo apt update && sudo apt upgrade -y
sudo apt install -y git tmux curl nano python3-pip net-tools
```

### 2-2. 네트워크 — 고정 IP + 이름으로 접속
```bash
hostnamectl set-hostname jetson        # 원하는 이름
sudo apt install -y avahi-daemon       # → PC 에서 ssh jetson.local 로 접속 가능
ip addr show | grep "inet "            # 현재 IP 확인
```
공유기에서 **MAC 주소 기반 DHCP 고정 할당**을 걸어두는 게 젯슨 설정을 만지는 것보다 안전하다.

### 2-3. SSH 서버
JetPack 이미지는 `openssh-server` 가 기본 설치·활성화돼 있다. 확인만:
```bash
sudo systemctl status ssh
```

### 2-4. 시리얼 포트 권한 (모터·IMU·라이다)
```bash
sudo usermod -aG dialout $USER     # 로그아웃 후 재로그인해야 적용
groups | grep dialout              # 확인
```

### 2-5. ROS 2 Humble 설치

> ⚠️ **2025년 6월에 ROS apt 서명 키가 교체됐다.** 인터넷에 있는 예전 설치 안내
> (`apt-key adv ...` 또는 `curl ... ros-archive-keyring.gpg`)를 그대로 따라 하면
> **`NO_PUBKEY` 오류**가 난다. 아래 `ros2-apt-source` 패키지 방식을 쓸 것.

```bash
sudo apt install -y software-properties-common curl
sudo add-apt-repository universe

# 새 방식: 키 + 저장소 설정을 .deb 하나로
export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest \
  | grep -F "tag_name" | awk -F\" '{print $4}')
curl -L -o /tmp/ros2-apt-source.deb \
  "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo $VERSION_CODENAME)_all.deb"
sudo apt install -y /tmp/ros2-apt-source.deb

sudo apt update
sudo apt install -y ros-humble-desktop python3-colcon-common-extensions python3-rosdep
```

### 2-6. 이 프로젝트가 쓰는 ROS 패키지
```bash
sudo apt install -y \
  ros-humble-robot-localization \
  ros-humble-cartographer-ros \
  ros-humble-tf-transformations \
  ros-humble-xacro \
  ros-humble-joint-state-publisher \
  ros-humble-robot-state-publisher \
  ros-humble-foxglove-bridge
pip3 install transforms3d
```

### 2-7. 저장소 클론 + 빌드
```bash
cd ~
git clone https://github.com/jungguck/mobile_robot_proto_type.git
cd mobile_robot_proto_type

sudo rosdep init && rosdep update
rosdep install --from-paths src --ignore-src -r -y

colcon build --symlink-install
```

> **`--symlink-install` 은 필수다.** 두 가지 이유:
> 1. `bridge_node.py` 가 소스 경로 기반으로 `TubeMPCPlanner` 를 import 한다 (README 480행)
> 2. **`.py` 를 고쳐도 재빌드 없이 노드 재시작만으로 반영된다** — 드라이버 튜닝 때 체감이 크다
>    (단, **새 파일 추가 / `setup.py` entry_points / launch / config yaml** 변경은 재빌드 필요)

### 2-8. 환경 로드 alias
```bash
echo 'alias ros_setup="source /opt/ros/humble/setup.bash && source ~/mobile_robot_proto_type/install/setup.bash && echo ROS2 환경 로드 완료"' >> ~/.bashrc
source ~/.bashrc
```

### 2-9. udev 규칙 — USB 포트 고정
포트가 뒤바뀌면 모터 대신 라이다에 명령을 보내게 된다. 반드시 할 것.
```bash
sudo cp ~/mobile_robot_proto_type/src/relayrobot_description/scripts/99-robot-devices.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules && sudo udevadm trigger

cd ~/mobile_robot_proto_type && ./check_devices.sh    # /dev/motor, /dev/rplidar, /dev/ttyimu 확인
```
> 라이다와 IMU 는 같은 칩(`10c4:ea60`)이라 **product 문자열로만 구분된다.**
> 규칙에 `MODE="0666"` 이 있어야 하고 사용자가 `dialout` 그룹이어야 포트가 열린다.

### 2-10. 성능 / 안정성
```bash
sudo nvpmodel -q                  # 현재 전원 모드
sudo nvpmodel -m 0                # MAXN (Super) — Cartographer 가 CPU 를 많이 쓴다
sudo jetson_clocks                # 클럭 고정

sudo pip3 install -U jetson-stats && sudo reboot
jtop                              # CPU/GPU/온도/전력 모니터
```

**스왑 추가** — `colcon build` 가 메모리를 많이 써서 8GB 로도 OOM 이 날 수 있다:
```bash
sudo fallocate -l 8G /swapfile && sudo chmod 600 /swapfile
sudo mkswap /swapfile && sudo swapon /swapfile
echo '/swapfile none swap sw 0 0' | sudo tee -a /etc/fstab
```
빌드가 자꾸 죽으면 병렬도를 낮춘다: `colcon build --symlink-install --parallel-workers 2`

### 2-11. GUI 를 원격에서 띄우려면
```bash
sudo apt install -y xauth        # ssh -X 포워딩에 필요
```
`/etc/ssh/sshd_config` 에 `X11Forwarding yes` 확인 후 `sudo systemctl restart ssh`.

---

## 3. PC(Windows 11) 쪽 준비

### 3-1. SSH 키 + 접속 별칭
```powershell
ssh-keygen -t ed25519
type $env:USERPROFILE\.ssh\id_ed25519.pub | ssh <user>@<젯슨IP> "mkdir -p ~/.ssh && cat >> ~/.ssh/authorized_keys"
```
`C:\Users\<사용자>\.ssh\config`:
```
Host robot
    HostName jetson.local        # 또는 고정 IP
    User <user>
    ServerAliveInterval 30
    ForwardX11 yes
```
→ 이후 `ssh robot` 만 치면 된다. `ServerAliveInterval` 은 무선 유휴 끊김을 막는다.

### 3-2. 도구

| 도구 | 용도 | 비고 |
|---|---|---|
| **VS Code + Remote-SSH** | 코드 편집 · 터미널 · git | 주력 |
| **VcXsrv** 또는 MobaXterm | X 서버 | `ssh -X` 로 `hw_test` GUI 띄울 때 |
| **Foxglove Studio** | `/scan` `/map` `/tf` `/odom` 시각화 | ROS 설치 불필요 |

### 3-3. 원격 코드 편집 방법 비교

| 방법 | 장점 | 단점 |
|---|---|---|
| **VS Code Remote-SSH** | 로컬처럼 편집, 터미널 다중, Source Control 로 젯슨에서 바로 커밋 | 젯슨 메모리 조금 씀 |
| `nano` (SSH) | 준비물 0, 항상 가능 | 큰 수정에 불편 |
| `git pull` | 이력이 깔끔 | 한 글자 고치는데 커밋 1개 — 루프가 길다 |
| SSHFS-Win | Windows 편집기 아무거나 | 무선 끊기면 편집기가 멈춤 — 비권장 |

`nano` 단축키: `Ctrl-O` 저장 · `Ctrl-X` 종료 · `Ctrl-W` 검색 · `Alt-U` 실행취소

---

## 4. 실행 흐름

### tmux — 선택이 아니라 필수
detach 해두면 **무선이 끊겨도 ROS 노드가 안 죽는다.** 그냥 SSH 로 띄우면
연결이 끊기는 순간 주행 중에 드라이버가 같이 죽는다.
```bash
tmux new -s robot
#  Ctrl-b %   좌우 분할        Ctrl-b "   상하 분할
#  Ctrl-b ←→  창 이동          Ctrl-b d   분리(detach)
tmux attach -t robot
```

### SLAM 모니터링 — RViz 를 젯슨에서 띄우지 말 것
젯슨이 렌더링과 영상 인코딩을 둘 다 하게 되어 SLAM 에 쓸 CPU 를 뺏긴다.
젯슨은 데이터만 보내고 PC 가 그린다:
```bash
# 젯슨
ros2 launch foxglove_bridge foxglove_bridge_launch.xml
```
PC 의 Foxglove Studio → `ws://<젯슨IP>:8765`

### 하드웨어 점검 GUI
`hw_test` 는 tkinter(OpenGL 미사용)라 X11 포워딩으로 가볍게 넘어온다.
```bash
ssh -X robot
ros_setup && ros2 run gui_py hw_test
```

---

## 5. ⚠️ 저장소가 두 곳에 있다

Windows PC 와 젯슨(`~/mobile_robot_proto_type`) 양쪽에 클론이 있다.
**양쪽에서 따로 고치면 코드가 갈라진다.** 문서가 어긋나는 것보다 훨씬 고약하다.

```bash
# 젯슨에서 고치기 전에 항상
git pull

# 고치고 테스트가 끝나면 그날 바로
git add -A && git commit -m "..." && git push
```

> **젯슨에서 고쳤으면 젯슨에서 커밋한다.** VS Code Remote-SSH 의 Source Control 패널이면 클릭 몇 번이다.
> "나중에 정리해서 올려야지" 가 바로 2026-09-03 에 한 라운드를 통째로 날린 원인이다.
> (`docs/DEBUG_LOG_2026-09-03.md` 1절)

---

## 6. 셋업 완료 확인

```bash
ros_setup
ros2 doctor                     # ROS 환경 점검
./check_devices.sh              # /dev/motor · /dev/rplidar · /dev/ttyimu
groups | grep dialout           # 시리얼 권한
ls install/                     # colcon build 산출물
ros2 pkg list | grep relayrobot # 패키지 인식
```

전부 통과하면 README 의 **STAGE 1** 부터 진행한다.
