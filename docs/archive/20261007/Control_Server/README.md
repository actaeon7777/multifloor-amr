> **이력 문서** · 2026-10-07 정리 시점의 이전 안내입니다. 현재 실행은 [최신 운영 문서](../../../README.md), 검증은 [검증 문서](../../../VALIDATION.md)를 사용합니다.

# 2026-10-06 위치추정 웹 업데이트 적용본

2026-10-07: 로컬 배포본에 컨셉 A Clarity 웹 UI와 차량 상태 화면을 반영했습니다.
새 `dashboard.js`·`battery-profile.js` 제공, 중첩 차량 센서·시간 감시·목표 피드백의
전송 지연 처리를 함께 적용했습니다. AMR IP 대신 현재 웹 서버 주소를 표시하며
미연동 전압·드라이버·BMS 정보는 예시 수치 없이 표시합니다.
호스트 PC에는 이 로컬 배포본을 복사하고 서버를 다시 시작해야 적용됩니다.

Jetson의 새 PC 배포본과 연결하는 중앙 서버 업데이트다.
[업데이트 적용·복귀 안내](UPDATE_20261006.md)를 먼저 따른다.
웹 정적 파일과 vendor 브리지에 자동 위치 찾기 및 새 위치 준비 상태 처리를 반영했다.
호스트 PC에 이 파일이 복사되어 서버가 다시 시작되기 전에는 배포가 완료된 것이 아니다.
아래 ‘현재 실행 상태’는 기존 구축 시점의 기록이다.

---

# AMR 중앙 관제 서버

프로젝트: `/home/kim/Control_Server`

기존 `amr_remote_bridge`의 Registry와 변경하지 않은 Agent, 기존 웹 UI를
연결하는 FastAPI + Uvicorn 호스트 서버다. ROS 설치나 colcon 빌드는 호스트에 필요 없다.

## 현재 실행 상태

- 호스트 Tailscale 등록·로그인 확인: **100.88.249.77**.
- AMR Tailscale IP: **100.106.98.122**. 호스트 → AMR peer ping 성공
  (direct 연결, 확인 시 4ms). 이는 현재 경로의 단일 측정이며 모바일 회선 성능 검증 결과는 아니다.
- 서버 바인딩: **100.88.249.77:8081**, Uvicorn worker 1개, reload 비활성화.
- 호스트 공통 인증 파일: `/home/kim/.config/amr_remote_bridge/token`.
  디렉터리 700, 파일 600. 키 내용은 소스·설정·로그에 저장하지 않는다.
- 호스트 자신의 Tailscale 주소에서 인증·HTML·API 응답을 확인했다.
- AMR Agent의 실제 교환은 아직 확인하지 않았다. 현재 등록 로봇 0대이며 상태 API의
  HTTP 503, 지도 대기 화면과 fleet의 빈 목록이 정상이다.

브라우저 주소:

- 기본 지도 UI: <http://100.88.249.77:8081/>
- amr-01 지도 UI: <http://100.88.249.77:8081/robots/amr-01/>
- 로봇 목록: <http://100.88.249.77:8081/fleet>

로그인 사용자 이름은 `operator`, 비밀번호는 공통 인증 파일 내용이다.
호스트에서 직접 접속할 때도 위 주소를 사용한다. 다른 장치는 같은 tailnet과
해당 포트 접근 권한이 필요하다.

## 환경 및 재현

검증 환경: Ubuntu 22.04.5 LTS, Python 3.10.12, Tailscale 1.102.5.

| 패키지 | 검증 버전 |
|---|---|
| FastAPI | 0.142.2 |
| Uvicorn | 0.54.0 |
| Starlette | 1.7.0 |
| AnyIO | 4.15.1 |
| pytest | 9.1.1 |
| httpx | 0.28.1 |
| Pillow | 12.3.0 |
| PyYAML | 6.0.3 |
| Playwright | 1.63.0 |

새 환경에서는 프로젝트 디렉터리에서 실행한다.

```bash
cd /home/kim/Control_Server
python3 -m venv .venv
.venv/bin/python -m pip install -r requirements.lock.txt
```

`requirements.txt`는 운영 직접 의존성, `requirements-dev.txt`는 시험 의존성,
`requirements.lock.txt`는 이번에 설치·검증한 전체 고정 버전이다.
현재 호스트에는 ensurepip가 없어 `venv --without-pip`를 만든 뒤 공식 PyPA
get-pip 스크립트로 프로젝트 가상환경에만 pip를 설치했다. 시스템 Python은 변경하지 않았다.
같은 문제가 발생하면 일반 터미널에서 OS의 `python3.10-venv`를 준비하거나 다음을 사용한다.

```bash
python3 -m venv --without-pip .venv
curl -fsSL https://bootstrap.pypa.io/get-pip.py -o /tmp/amr-host-get-pip.py
.venv/bin/python /tmp/amr-host-get-pip.py
.venv/bin/python -m pip install -r requirements.lock.txt
```

## 실행·종료·재실행

관리 스크립트는 이 프로젝트에서 시작한 프로세스의 PID·실행 파일·작업 경로를
확인한 뒤 종료한다. 다른 서버를 종료하지 않는다. PID 상태는 `.run/server.json`,
로그는 `diagnostics/server.log`다. 이 방식은 재부팅 후 자동 시작하는 서비스는 아니다.

```bash
cd /home/kim/Control_Server
.venv/bin/python scripts/serverctl.py status
.venv/bin/python scripts/serverctl.py start
.venv/bin/python scripts/serverctl.py stop
.venv/bin/python scripts/serverctl.py restart
```

start·restart는 현재 `tailscale ip -4`의 실제 주소를 사용한다.
이미 실행 중이면 start는 중복 실행하지 않는다. 설정 변경 후에는 restart를 사용한다.

터미널에서 직접 실행하려면 관리 프로세스를 먼저 종료한다.

```bash
.venv/bin/python scripts/serverctl.py stop
scripts/run_server.sh --tailscale
```

직접 실행 중인 서버는 해당 터미널에서 `Ctrl+C`로 종료한다.
로컬 개발·시험은 `scripts/run_server.sh --local` 또는
`.venv/bin/python scripts/serverctl.py start --local`로 127.0.0.1:8081에 바인딩한다.
Tailscale 주소를 사용한 운영 서버는 127.0.0.1:8081에서도 함께 듣는 구성이 아니다.

필요하면 `config/server.env.example`을 `config/server.env`로 복사하여
포트·기본 로봇·토큰 **파일 경로**·시간 제한을 설정한다. 토큰 값 자체를 넣지 않는다.
UI 기본 위치는 `vendor/amr_web_ui/web`, fleet는 `vendor/amr_remote_bridge/web`이다.
다른 위치라면 `AMR_ASSETS_DIR`, `AMR_FLEET_DIR`을 지정한다.

## 인증 파일 준비·AMR 전달

이번 호스트 키는 이미 생성했다. 다음 명령은 정상 기존 키를 덮어쓰지 않고 재사용한다.

```bash
python3 scripts/create_token.py
```

AMR에 이미 운영 중인 키가 있으면 연결 전에 어느 키를 사용할지 맞춘다.
AMR의 기존 키를 확인하지 않고 덮어쓰지 않는다. 신규 AMR 키 파일이 없는 경우,
실제 SSH 대상 이름으로 `AMR_SSH_ALIAS`를 바꾸고 호스트 일반 터미널에서 다음을 실행할 수 있다.
SSH 암호화 통신으로 파일 내용을 전달하며, 원격 파일이 이미 있으면 생성이 실패한다.

```bash
ssh AMR_SSH_ALIAS 'umask 077; mkdir -p "$HOME/.config/amr_remote_bridge"; chmod 700 "$HOME/.config/amr_remote_bridge"; set -C; cat > "$HOME/.config/amr_remote_bridge/token"' < "$HOME/.config/amr_remote_bridge/token"
```

전달 후 AMR에서 파일 존재와 권한만 확인한다. 대화나 서버 로그에 키를 출력하지 않는다.

```bash
stat -c '%a %n' "$HOME/.config/amr_remote_bridge/token"
```

## AMR 실행 명령

AMR의 기존 stack·localization·navigation은 기존 현장 절차로 준비한다.
로컬 웹 API가 이미 실행 중이면 중복 실행하지 않는다.

```bash
# AMR 터미널 1
cd /home/acus03/amr_ws
source install/setup.bash
ros2 launch amr_web_ui web_ui.launch.py host:=127.0.0.1 port:=8080
```

```bash
# AMR 터미널 2: 양쪽 공통 인증 파일 준비 후 실행
cd /home/acus03/amr_ws
source install/setup.bash
ros2 run amr_remote_bridge remote_client \
  --server-url http://100.88.249.77:8081 \
  --token-file "$HOME/.config/amr_remote_bridge/token" \
  --robot-id amr-01
```

빌드 없이 모듈로 실행하는 대안:

```bash
cd /home/acus03/amr_ws
PYTHONPATH=src/amr_system/amr_remote_bridge python3 -m amr_remote_bridge.agent \
  --server-url http://100.88.249.77:8081 \
  --token-file "$HOME/.config/amr_remote_bridge/token" \
  --robot-id amr-01
```

위 워크스페이스 경로는 제공된 AMR 문서 기준이다. 실제 AMR 경로가 다르면 맞춰야 한다.
AMR 로컬 API의 읽기 확인은 AMR 터미널에서 `curl -fsS http://127.0.0.1:8080/api/state`로 한다.
실제 연결 시 상태·지도부터 확인하며, 실차 이동은 사용자 요청과 기존 운용 절차가 준비된 뒤 진행한다.

## Tailscale 설치와 연결 점검

호스트는 사용자가 설치·로그인했고 에이전트가 상태를 확인했다. AMR 미설치 시에는
[공식 Linux 설치 절차](https://tailscale.com/docs/install/linux)를 사용하고 같은 tailnet에 등록한다.
sudo 비밀번호와 Tailscale 인증 키를 대화로 보내지 않는다.

```bash
curl -fsSL https://tailscale.com/install.sh | sh
sudo tailscale up
tailscale status
tailscale ip -4
```

호스트 점검:

```bash
cd /home/kim/Control_Server
scripts/check_connection.sh
.venv/bin/python scripts/verify_host.py
# 호스트 → AMR peer ping과 호스트의 서버 HTTP 응답을 각각 점검
scripts/check_connection.sh 100.106.98.122
```

`check_connection.sh`의 HTTP 401은 인증 없는 포트 접근이 성공했다는 의미다.
AMR에서 호스트 포트까지의 실제 접근도 별도로 확인한다.

```bash
# AMR에서 실행: peer ping과 HTTP 접근은 별개의 시험이다.
tailscale ping 100.88.249.77
curl --noproxy '*' --connect-timeout 3 -i http://100.88.249.77:8081/fleet
```

이 curl의 기대 결과도 HTTP 401이며 키를 명령줄에 넣을 필요가 없다.
접근이 안 되면 tailnet 정책·호스트 방화벽의 tailscale0/8081 접근을 확인한다.
방화벽 전체 비활성화·포트포워딩·Funnel·exit node·subnet router는 구성에 포함하지 않는다.

## 검증 결과와 재실행

| 구분 | 결과 | 범위 |
|---|---|---|
| 원본 서버 기준 | 19개 통과 | 원본 ThreadingHTTPServer + 원본 Agent + 로컬 API |
| 새 FastAPI 서버 | 48개 통과 | 실제 Uvicorn TCP + 원본 Agent + 로컬 API; 정상·장애·재시작·동시성·입력 검증 |
| 실제 Chromium UI | 통과 | 생성 지도 PNG, 목표·초기 위치·취소, 단절 제어 차단, 재접속 미재실행, fleet |
| 호스트 Tailscale HTTP | 통과 | 본인 주소의 인증·UI·목록 API; 미접속 상태 503 |
| 호스트 → AMR peer ping | 통과 | 100.106.98.122, direct 연결, 확인 시 4ms |
| 실제 AMR Agent 교환 | 미확인 | 키 전달·로컬 API·Agent 실행 확인 필요; 현재 등록 0대 |

시험 전용 키와 임시 포트를 사용하며 운영 Registry나 실제 AMR에 명령을 보내지 않는다.
기존 테스트의 시나리오는 `tests/test_compatibility.py`가 재사용하지만, fixture는
`tests/conftest.py`의 **새 FastAPI/Uvicorn 서버**다. 원본 서버 시험과 구분된다.
브라우저 지도는 80×60의 생성 Grid이며 실제 AMR 건물 지도를 시험한 결과는 아니다.

```bash
cd /home/kim/Control_Server
mkdir -p diagnostics
# 원본 기준 시험: vendor 내부 pytest 설정 대신 호스트 설정을 명시
.venv/bin/python -m pytest -c pytest.ini -q vendor/amr_remote_bridge/test/test_remote_bridge.py \
  --junitxml=diagnostics/baseline.xml
# 새 서버의 실제 TCP 시험
.venv/bin/python -m pytest -c pytest.ini -q tests --junitxml=diagnostics/fastapi.xml
# 실제 브라우저 준비 및 시험
PLAYWRIGHT_BROWSERS_PATH="$PWD/.playwright-browsers" .venv/bin/python -m playwright install chromium --only-shell
.venv/bin/python scripts/browser_check.py
```

실제 지도 미리보기를 별도로 시험하려면 YAML과 참조 이미지 파일을 준비하고
`scripts/browser_check.py --map /path/to/map.yaml`을 사용한다.
ROS 전용 vendor 시험과 복사되지 않은 amr_stack 지도 의존 시험은 호스트 시험 범위에서 제외했다.

결과 파일:

- `diagnostics/baseline.xml`, `diagnostics/fastapi.xml`: 자동 시험 결과.
- `diagnostics/browser_result.json`, `central_ui.png`, `fleet_ui.png`: 브라우저 결과와 화면.
- `diagnostics/host_result.json`: 운영 주소에서 수행한 읽기 전용 HTTP 확인.
- `diagnostics/peer_result.json`: 호스트 → AMR Tailscale peer ping 결과.
- `diagnostics/verification.json`: 환경·시험·외부 연결 상태 요약.

## 유지한 규약과 제한

`POST /api/agent/exchange`의 Bearer 인증, 브라우저 Basic 인증, protocol 1,
루트 API 별칭과 로봇별 경로, `{"error": ...}`, no-store 및 원본 CSP를 유지한다.
FastAPI 자동 요청 형 변환과 trailing-slash redirect를 사용하지 않는다.
정수 protocol 검증과 비유한 JSON 숫자 거절을 강화했으며 정상 기존 Agent의 요청은 그대로 동작한다.

기본 상태 신선도는 2초, 명령 TTL은 3초다. 원본 Registry의 state_age 범위는 0~3600초다.
명령 전달 전 만료는 408, 전달 후 응답 불명은 504다. 목표를 새 ID로 자동 재요청하지 않는다.
로컬 API의 QUEUED는 웹 API 접수이며 ROS 접수·주행 성공은 이후 상태로 확인한다.
센서 age·지도 중앙/원본 버전·PNG SHA-256 검증은 원본 구현을 사용한다.

상태·큐·이미지와 최근 256개 명령 기록은 메모리 보관이다. worker를 늘리면 Registry가
분리되므로 현재는 단일 worker로 실행한다. 동기 명령 결과 대기는 별도의 thread pool
용량으로 처리하며 상태 교환을 계속할 수 있다.
[FastAPI 실행 방식](https://fastapi.tiangolo.com/async/),
[프로세스별 메모리](https://fastapi.tiangolo.com/deployment/concepts/)

현재는 단일 운영자·공통 키 모델이다. 연결 단절이 이미 AMR에서 접수한 목표를
자동 정지·취소하지 않는다. 취소는 기존 웹 UI가 보유한 목표 범위다.
기존 수동/자동·조이스틱 RB·mux·CAN 정책은 AMR에서 유지된다.
WebSocket 전환, 자동 작업 배정, 사용자별 권한, 영구 작업 복원은 이번 범위에 포함되지 않는다.

실제 연결을 마무리하려면 동일 키 전달 여부와 AMR 로컬 API·Agent 실행 상태를
확인해야 한다. 원격 점검·파일 전달을 에이전트가 수행하려면 AMR의 SSH 사용자 이름 또는
접속 별칭과 사용 가능한 인증 방법이 필요하다. 호스트 IP가 바뀌면 README의
AMR server-url과 접속 주소도 새 주소로 갱신한다.
