# 로봇 안전 종료 (PC 쪽) — 젯슨을 끄고, 전원을 뽑아도 되는 시점까지 대기한다.
#
# 사용법 — 이건 PC 에서 도는 PowerShell 스크립트다. ssh 뒤에 붙이는 게 아니다.
#
#     .\scripts\robot-off.ps1
#     .\scripts\robot-off.ps1 -RobotHost robot   # ~/.ssh/config 의 Host 이름
#
#     젯슨만 끄고 대기는 직접 하겠다면:  ssh robot robot-off
#
# 실행 정책에 막히면 (윈도우 기본값이 Restricted 라 대부분 막힌다):
#     powershell -ExecutionPolicy Bypass -File .\scripts\robot-off.ps1
#
# 매번 치기 싫으면 한 번만 풀어둔다 (사용자 범위 / 관리자 권한 불필요):
#     Set-ExecutionPolicy -Scope CurrentUser RemoteSigned
#
# ※ 이 파일은 UTF-8 BOM 으로 저장해야 한다. Windows PowerShell 5.1 은 BOM 이 없으면
#    ANSI(CP949)로 읽어서 한글 주석이 깨지고 파서가 죽는다. 편집기 설정 주의.
#
# 하는 일:
#     1. 젯슨 IP 를 먼저 알아둔다 (끄고 나면 물어볼 수 없다)
#     2. ssh 로 robot-off 실행  -> 모터 정지 / 노드 종료 / sync / poweroff
#     3. ping 이 끊길 때까지 대기  -> 커널이 실제로 내려간 시점
#     4. 20초 더 대기            -> 전원 회로가 완전히 내려가는 시간
#     5. "이제 뽑아도 됩니다" 표시
#
# 왜 필요한가: 젯슨이 SD 카드로 부팅해서 갑작스런 전원 차단에 약하다. ping 이 끊긴
#              직후에도 잠깐은 디스크에 쓰고 있을 수 있어서 여유를 둬야 한다.

param(
    [string]$RobotHost = "robot",
    [int]$PowerOffGraceSeconds = 20,   # ping 끊긴 뒤 추가 대기
    [int]$ShutdownTimeoutSeconds = 90  # 이 시간 안에 안 꺼지면 경고
)

$ErrorActionPreference = "Stop"

function Write-Step($msg) { Write-Host "==> $msg" -ForegroundColor Cyan }
function Write-Ok($msg)   { Write-Host "  OK  $msg" -ForegroundColor Green }
function Write-Warn($msg) { Write-Host "  !!  $msg" -ForegroundColor Yellow }
function Write-Err($msg)  { Write-Host "  XX  $msg" -ForegroundColor Red }

# ── 1. IP 확보 ────────────────────────────────────────────────────────────────
# 끄고 나면 젯슨에 물어볼 수 없으니 반드시 먼저 알아둔다.
Write-Step "젯슨 주소 확인"
$ip = (ssh -o BatchMode=yes -o ConnectTimeout=10 $RobotHost "hostname -I | awk '{print `$1}'" 2>$null)
if ($LASTEXITCODE -ne 0 -or [string]::IsNullOrWhiteSpace($ip)) {
    Write-Err "$RobotHost 에 접속할 수 없습니다. 이미 꺼져 있거나 네트워크가 끊긴 상태입니다."
    Write-Host "      확인: ssh $RobotHost hostname"
    exit 1
}
$ip = $ip.Trim()
Write-Ok "$RobotHost = $ip"

# ── 2. 종료 실행 ──────────────────────────────────────────────────────────────
Write-Step "안전 종료 실행 (robot-off)"
# poweroff 로 SSH 연결이 끊기면서 비정상 종료 코드가 나오는 게 정상이다.
ssh -o BatchMode=yes $RobotHost "robot-off" 2>&1 | ForEach-Object { Write-Host "      $_" }

# ── 3. ping 이 끊길 때까지 ────────────────────────────────────────────────────
Write-Step "커널이 내려갈 때까지 대기 (ping 감시)"
$deadline = (Get-Date).AddSeconds($ShutdownTimeoutSeconds)
$downStreak = 0
$isDown = $false

while ((Get-Date) -lt $deadline) {
    # 연속 3회 실패해야 인정한다. 무선은 한두 번 놓치는 게 흔하다.
    if (Test-Connection -ComputerName $ip -Count 1 -Quiet -ErrorAction SilentlyContinue) {
        $downStreak = 0
        Write-Host "      아직 응답함..." -ForegroundColor DarkGray
    } else {
        $downStreak++
        Write-Host "      무응답 $downStreak/3" -ForegroundColor DarkGray
        if ($downStreak -ge 3) { $isDown = $true; break }
    }
    Start-Sleep -Seconds 2
}

if (-not $isDown) {
    Write-Warn "$ShutdownTimeoutSeconds 초가 지나도 응답이 있습니다."
    Write-Warn "종료가 걸렸을 수 있습니다. 전원을 뽑지 말고 상태를 확인하세요:"
    Write-Host "      ssh $RobotHost 'uptime; systemctl is-system-running'"
    exit 2
}
Write-Ok "ping 끊김 — 커널이 내려갔습니다"

# ── 4. 여유 대기 ──────────────────────────────────────────────────────────────
Write-Step "전원 회로가 완전히 내려갈 때까지 $PowerOffGraceSeconds 초 대기"
for ($i = $PowerOffGraceSeconds; $i -gt 0; $i--) {
    Write-Host -NoNewline "`r      남은 시간: $i 초   "
    Start-Sleep -Seconds 1
}
Write-Host "`r                                   "

# ── 5. 완료 ───────────────────────────────────────────────────────────────────
Write-Host ""
Write-Host "================================================================" -ForegroundColor Green
Write-Host " 이제 전원 어댑터를 뽑아도 됩니다" -ForegroundColor Green
Write-Host "================================================================" -ForegroundColor Green
Write-Host ""
Write-Host "  눈으로도 한 번 확인하세요 — LED 가 꺼지고 팬이 멈춰 있어야 합니다."
Write-Host "  아직 돌고 있으면 뽑지 말고 30초 더 기다리세요."
Write-Host ""
