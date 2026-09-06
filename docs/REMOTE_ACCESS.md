# 원격 접속 (SSH) — 실측 정보

**작성:** 2026-09-06 · **대상 젯슨:** Jetson Orin Nano (JetPack 6.2.x / L4T R36.4.7 / Ubuntu 22.04)

> `docs/JETSON_SETUP.md` 3절이 일반론이라면, 이 문서는 **이 젯슨의 실제 값**이다.
> 값은 측정해서 적은 것이고, 바뀔 수 있는 항목은 따로 표시했다.

---

## 0. 한 줄 요약

```bash
ssh frlab@172.30.1.45
```

**키 등록 완료 (2026-09-06)** — 비밀번호 없이 붙는다. PC 에 `robot` 별칭도 있으므로
실제로는 `ssh robot` 한 줄이면 된다. 경위는 `docs/DEBUG_LOG_2026-09-06.md` 1절.

---

## 1. 접속 정보 (2026-09-06 실측)

| 항목 | 값 | 비고 |
|---|---|---|
| 사용자 | `frlab` | |
| 호스트명 | `frlab` | 문서 예시의 `jetson` 이 아니다 |
| **IP** | **`172.30.1.45`** | ⚠️ **DHCP 동적** — 바뀔 수 있다 (5절) |
| 인터페이스 | `wlP1p1s0` (WiFi) | 유선 `enP8p1s0` 은 현재 down |
| 게이트웨이 | `172.30.1.254` | 공유기 관리페이지도 보통 여기 |
| 서브넷 | `172.30.1.0/24` | **PC 도 같은 대역이어야 한다** |
| MAC (WiFi) | 젯슨에서 `cat /sys/class/net/wlP1p1s0/address` | 공유기 고정IP 예약용 (5절) |
| SSH 포트 | `22` (기본) | `PasswordAuthentication` 기본 허용 |
| X11 포워딩 | ✅ `X11Forwarding yes` + `xauth` 설치됨 | `hw_test` GUI 원격 표시 가능 |

### SSH 호스트키 지문 — 첫 접속 시 대조용

첫 접속 때 `Are you sure you want to continue connecting?` 가 뜬다.
그때 보이는 지문이 아래와 같은지 확인하고 `yes` 를 입력한다.

```
ED25519  SHA256:AFn/uztFpWxRrgUMtKYhYbqs7A1J53ceCGUM2skCFqk
RSA      SHA256:WWZTDj+kzX33oHc44tMo7EF4y0ghGg5bFS0u0C31Buc
```

---

## 2. 접속 경로 3가지

### 2-A. WiFi IP 직접 (지금 되는 방법)
```bash
ssh frlab@172.30.1.45
```
가장 확실하다. 단 IP 가 바뀌면 못 쓴다.

### 2-B. `frlab.local` (mDNS) — ⚠️ 지금은 고장나 있다

이론상 `ssh frlab@frlab.local` 로 IP 를 몰라도 접속할 수 있어야 한다. 그런데:

```bash
$ avahi-resolve -4 -n frlab.local
frlab.local     172.17.0.1        # ← docker0 브리지 주소!
```

**avahi 가 WiFi 주소가 아니라 `docker0` 주소를 광고하고 있다.** 172.17.0.1 은 젯슨
내부에서만 의미가 있는 주소라, PC 에서 `frlab.local` 로 접속하면 엉뚱한 데로 가서 실패한다.
(`/etc/avahi/avahi-daemon.conf` 에 인터페이스 제한이 없어 avahi 가 도커 브리지까지 잡은 것)

**고치는 법 — 광고할 인터페이스를 WiFi 로 못박는다:**
```bash
sudo sed -i '/^\[server\]/a allow-interfaces=wlP1p1s0\ndeny-interfaces=docker0,l4tbr0' \
  /etc/avahi/avahi-daemon.conf
sudo systemctl restart avahi-daemon

# 확인 — 172.30.1.x 가 나와야 정상
avahi-resolve -4 -n frlab.local
```
고치고 나면 IP 가 바뀌어도 `ssh frlab@frlab.local` 로 계속 접속된다.

> Windows 10/11 은 mDNS 를 기본 지원하므로 PC 쪽에 따로 깔 건 없다.

### 2-C. USB-C 직결 (`192.168.55.1`) — 네트워크가 죽어도 되는 최후 수단

JetPack 은 `nv-l4t-usb-device-mode` 가 **기본 활성화**돼 있다 (이 젯슨도 `enabled` 확인함).
젯슨의 USB-C 포트를 PC 에 케이블로 연결하면 젯슨이 가상 이더넷 장치로 잡히고,
**공유기·WiFi 와 무관하게** 고정 주소로 접속된다.

```bash
ssh frlab@192.168.55.1
```

| 주소 | 의미 |
|---|---|
| `192.168.55.1` | 젯슨 (항상 고정) |
| `192.168.55.100` | PC 에 할당되는 주소 |

WiFi 설정이 날아갔거나 IP 를 모를 때 **이 경로로 들어가서 IP 를 확인**하면 된다:
```bash
ssh frlab@192.168.55.1 "ip -4 addr show wlP1p1s0"
```

---

## 3. PC 쪽 준비 — SSH 키 + 접속 별칭

> ✅ **이 절은 2026-09-06 에 완료됐다.** 아래는 새 PC 를 붙일 때 다시 쓰는 절차다.
> 등록됐는지 확인하려면 비밀번호 입력 자체를 금지하는 옵션으로 찔러본다:
> ```bash
> ssh -o BatchMode=yes robot hostname     # frlab 이 나오면 키 인증 성공
> ```

키를 등록하면 접속할 때마다 비밀번호를 치지 않아도 된다.

**Windows PowerShell:**
```powershell
ssh-keygen -t ed25519                      # 이미 있으면 건너뛴다
type $env:USERPROFILE\.ssh\id_ed25519.pub | ssh frlab@172.30.1.45 "mkdir -p ~/.ssh && chmod 700 ~/.ssh && cat >> ~/.ssh/authorized_keys && chmod 600 ~/.ssh/authorized_keys"
```

**Linux / macOS:**
```bash
ssh-copy-id frlab@172.30.1.45
```

**`~/.ssh/config` (Windows 는 `C:\Users\<사용자>\.ssh\config`):**
```
Host robot
    HostName 172.30.1.45          # 2-B 를 고쳤다면 frlab.local 로 바꾸는 게 낫다
    User frlab
    ServerAliveInterval 30        # 무선 유휴 끊김 방지 — 원격 주행 중엔 필수급
    ServerAliveCountMax 3
    ForwardX11 yes                # hw_test GUI 를 PC 로 띄우려면
```
→ 이후 `ssh robot` 만 치면 된다.

---

## 4. 원격에서 로봇을 돌릴 때 — tmux 는 선택이 아니다

**그냥 SSH 로 런치를 띄우면 WiFi 가 끊기는 순간 드라이버가 같이 죽는다.**
문제는 그 시점에 모터에는 이미 속도 명령이 들어가 있다는 것이다 —
**정지 명령을 보낼 주체가 사라진 채 로봇만 굴러간다.** 반드시 tmux 안에서 띄운다.

```bash
tmux new -s robot        # 세션 시작
#   Ctrl-b %   좌우 분할       Ctrl-b "   상하 분할
#   Ctrl-b ←→  창 이동         Ctrl-b d   분리(detach)
tmux attach -t robot     # 끊겼다 다시 들어올 때
```

끊겨도 노드는 tmux 안에서 계속 살아 있고, 다시 붙어서 `Ctrl-C` 로 정지시킬 수 있다.

**비상 정지 (다른 창에서):**
```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0}, angular: {z: 0.0}}" --once
```

---

## 5. IP 가 바뀌는 문제

현재 WiFi 는 **DHCP 동적 할당**이다 (리스 약 45분마다 갱신). 재부팅하거나 공유기가
리스를 재배정하면 `172.30.1.45` 가 아닐 수 있다.

**권장 해결 순서:**
1. **공유기에서 MAC 기반 고정 IP 예약** — 젯슨의 WiFi MAC 을 `172.30.1.45` 에 묶는다.
   MAC 확인: `cat /sys/class/net/wlP1p1s0/address`
   공유기 관리페이지: `http://172.30.1.254` (게이트웨이 주소)
   젯슨 네트워크 설정을 직접 만지는 것보다 안전하다.
2. **2-B 의 avahi 수정** — 이름(`frlab.local`)으로 접속하면 IP 를 몰라도 된다.
3. 둘 다 안 됐을 때 → **2-C 의 USB-C 직결**로 들어가서 IP 확인.

---

## 6. 시각화 — RViz 를 젯슨에서 띄우지 말 것

젯슨이 렌더링까지 하면 SLAM 에 쓸 CPU 를 뺏긴다. 젯슨은 데이터만 보내고 PC 가 그린다.

```bash
# 젯슨 (tmux 안에서)
ros2 launch foxglove_bridge foxglove_bridge_launch.xml
```
PC 의 **Foxglove Studio** → `Open connection` → `ws://172.30.1.45:8765`
→ `/scan` `/map` `/tf` `/odom` 을 PC 에서 그린다. PC 에 ROS 를 깔 필요가 없다.

**하드웨어 점검 GUI (`hw_test`)** 는 tkinter 라 X11 포워딩으로 가볍게 넘어온다:
```bash
ssh -X robot
ros_setup && ros2 run gui_py hw_test
```
Windows 에서는 **VcXsrv** 나 **MobaXterm** 같은 X 서버가 PC 쪽에 떠 있어야 한다.

---

## 7. PC 에서도 ROS 를 돌릴 경우

같은 `ROS_DOMAIN_ID` 를 써야 토픽이 보인다. **양쪽 모두** 설정한다.
```bash
echo 'export ROS_DOMAIN_ID=30' >> ~/.bashrc && source ~/.bashrc
```
확인: PC 에서 `ros2 topic list` 에 젯슨 토픽이 뜨면 성공.

---

## 8. 접속 안 될 때 점검 순서

```bash
# PC 에서
ping 172.30.1.45                 # 1) 네트워크 도달?   실패 → 같은 공유기인지 / IP 바뀌었는지
ssh -v frlab@172.30.1.45         # 2) 상세 로그로 어디서 막히는지

# 젯슨에서 (모니터 연결 또는 USB-C 직결)
systemctl status ssh             # 3) SSH 데몬 살아있나
ip -4 addr show wlP1p1s0         # 4) 실제 IP 확인
```

| 증상 | 원인 | 조치 |
|---|---|---|
| `ping` 실패 | IP 바뀜 / 다른 공유기 | 5절 — USB-C 로 들어가 IP 확인 |
| `frlab.local` 만 실패 | avahi 가 docker0 광고 | 2-B 수정 |
| 접속은 되는데 GUI 안 뜸 | PC 에 X 서버 없음 | VcXsrv/MobaXterm 실행 후 `ssh -X` |
| 주행 중 끊기며 노드 사망 | tmux 미사용 | 4절 |
