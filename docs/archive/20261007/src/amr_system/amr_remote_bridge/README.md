> **이력 문서** · 2026-10-07 정리 시점의 이전 안내입니다. 현재 실행은 [최신 운영 문서](../../../../../README.md), 검증은 [검증 문서](../../../../../VALIDATION.md)를 사용합니다.

# 2026-10-06 새 위치추정 웹 API 연동

2026-10-07 컨셉 A 호환: 중앙 서버가 `dashboard.js`와 `battery-profile.js`를 제공합니다.
중첩 차량 텔레메트리, 시간 감시, 목표 피드백의 경과 시간에 관제 전송 지연을 더합니다.
새 UI를 중앙에서 사용할 때 호스트의 웹 정적 파일과 `amr_remote_bridge/server.py`도 함께 업데이트해야 합니다.
원격 프로토콜 버전은 유지하며 이전 로컬 API에 차량 텔레메트리가 없으면 미수신으로 표시합니다.

현재 PC 배포본의 `amr_web_ui`와 연결한다. 기존 protocol 1을 유지하면서
`POST /api/relocalize`와 중앙 `/robots/<robot_id>/api/relocalize`를 추가했다.
요청은 `request_id`와 지도가 표시된 경우 `map_version`을 사용하며 좌표는 필요 없다.
기존 지도 버전 변환·내용 검증·명령 만료·중복 실행 방지 규칙을 적용한다.
위치 검증 상태의 `age`에도 관제 전송 지연을 반영하며 2초 이상 오래된 준비 상태는 해제한다.
최종 목표/정지/위치 검증은 AMR의 로컬 웹 API와 주행 스택에서 수행한다.

AMR 브리지, 호스트의 vendor 브리지, 호스트 웹 정적 파일을 함께 업데이트한다.
호스트 HTTP 서버는 ROS 없이 동작하며, 호스트의 로컬 API 호환성 테스트에는
`vendor/amr_control`의 순수 Python 시간 검증 모듈을 함께 사용한다.

현재 실행 순서는 [중앙 관제 실행 안내](../../../docs/AMR_REMOTE_REAL_DRIVE_TEST.md)를 따른다.
아래 내용은 기존 네트워크 구성과 프로토콜 참고이며 현재 접속 상태를 보장하지 않는다.

---

# AMR Remote Bridge — 중앙 관제 서버와 외부 접속 클라이언트

기존 `amr_web_ui`를 변경하지 않고, AMR이 호스트 PC에 먼저 연결하여 상태·지도를
전송하고 중앙 서버의 목표·초기 위치·취소 요청을 기존 로컬 API로 전달합니다.
Tailscale은 사설 통신 경로를 제공합니다. 관제 서버와 AMR 클라이언트는 이 패키지에서 실행합니다.

## 현재 할당된 네트워크 주소

2026-10-06 사용자 공유 기준입니다. 호스트 중앙 서버는 구현 중입니다.

| 장치 | Tailscale IPv4 |
|---|---|
| 호스트 PC / 중앙 관제 서버 | `100.88.249.77` |
| AMR | `100.106.98.122` |

현재 프로토타입의 기본 서버 포트는 `8081`이며, 아래 명령에 실제 호스트 IP를 반영했습니다.
호스트 구현에서 포트를 변경하면 클라이언트 URL도 맞춰야 합니다. 주소 할당 정보와
실제 서버·peer 접속 검증은 구분합니다.
값은 [네트워크 설정 기록](../../../../../reference/AMR_REMOTE_NETWORK.env)에도 저장했습니다.
이 파일은 프로그램이 자동으로 읽지 않습니다.

```text
호스트 PC 브라우저 → 중앙 서버 → 다음 클라이언트 응답에 명령 포함
                         ↑                    ↓
            Tailscale 가상망의 HTTP 요청·응답 (AMR이 연결 시작)
                         ↑                    ↓
AMR remote_client → localhost:8080의 기존 amr_web_ui → 기존 navigation_manager
                                                    → 기존 control_mux → CAN
```

## 구현 범위

- Python 표준 라이브러리만 사용합니다. WebSocket·MQTT·rosbridge·npm 의존성이 없습니다.
- AMR은 기본 5Hz로 중앙 서버에 HTTP POST를 시작합니다. 중앙 서버가 AMR에 새 TCP 연결을
  여는 방식이 아니므로, 테더링·LTE NAT 환경에서도 양쪽 Tailscale 연결이 확보되면 동작합니다.
- 로컬 `/api/state`의 로봇 위치·방향, AMCL, LiDAR, 전역·지역 경로, footprint,
  속도, 주행 모드, 목표 상태, notice, pending_request를 전달합니다.
- `/api/layers/{map,local_costmap,global_costmap}.png`를 버전별로 전달합니다.
  기본 5초마다 재확인하여 로컬 웹 서버가 같은 지도 버전을 재사용하는 경우도 확인합니다.
- 중앙 서버는 기존 `amr_web_ui/web` 파일을 읽어 제공합니다. 브라우저 API 주소만
  `/robots/<robot_id>/api/...`로 바꾸어 제공하며 원본 파일은 수정하지 않습니다.
- `/api/goal`, `/api/initialpose`, `/api/cancel`은 기존 API로 전달합니다.
  실물 조이스틱의 AUTO 허용과 기존 목표 관리·mux·CAN 정책이 그대로 적용됩니다.
- 중앙 `/fleet` 페이지는 등록된 로봇을 보여주며 각 로봇의 기존 지도 UI로 이동합니다.
  이 단계는 수동 로봇 선택을 지원합니다. 자동 작업 배정·교통 관제는 포함하지 않습니다.

## 준비

AMR에는 기존 주행 시스템과 `amr_web_ui`가 있어야 합니다. 중앙 서버는 Python 3.10 이상과
기존 웹 정적 파일만 있으면 실행되며 ROS 2가 필요하지 않습니다.

PC와 Jetson에 Tailscale을 설치하고 같은 tailnet에 등록합니다.
AMR은 스마트폰 모바일 데이터의 USB/Wi-Fi 테더링에, PC는 연구실 유선망 등 다른 인터넷에
연결하여 서로 다른 네트워크 조건을 시험합니다. 스마트폰에는 Tailscale 설치가 필요 없습니다.
외부 연결 확인은 두 장치에서 다음과 같이 합니다.

```bash
tailscale status
tailscale ip -4
tailscale ping <상대 장치의 Tailscale IP>
```

Tailscale 접속·승인과 교내 방화벽 허용은 실행 환경에서 준비해야 합니다.
이 패키지는 Tailscale 설치·로그인·회선 설정을 자동으로 변경하지 않습니다.

## 인증 파일

호스트 PC에서 키를 한 번 만들고, 같은 내용을 AMR의 동일 경로에 안전하게 복사합니다.
아래 생성 명령은 최초 한 번만 실행합니다. 운영 중 키를 덮어쓰면 양쪽 키가 달라집니다.

```bash
mkdir -p ~/.config/amr_remote_bridge
chmod 700 ~/.config/amr_remote_bridge
umask 077
openssl rand -hex 32 > ~/.config/amr_remote_bridge/token
```

토큰은 최소 32자입니다. 명령줄 값이나 Git에 넣지 않고 `--token-file`로 읽습니다.
브라우저는 사용자 이름 **operator**, 비밀번호 **토큰 파일의 내용**으로 로그인합니다.
AMR은 같은 토큰으로 Bearer 인증을 합니다. 이 단계는 개발용 공통 키와 단일 운영자 역할을
사용합니다. 사용자별 권한·로봇별 키 분리는 후속 관제 기능입니다.

서버의 기본 바인딩은 `127.0.0.1`입니다. 원격 시험에서는 **호스트 PC의 Tailscale IP에만
바인딩**하세요. 아래 HTTP 연결은 Tailscale의 암호화된 가상망 안에서 사용합니다.
인터넷 포트포워딩이나 `0.0.0.0` 공개 바인딩은 이 배포 절차에 포함하지 않습니다.

## 호스트 PC에서 중앙 서버 실행

호스트에도 이 소스 트리가 있다면 ROS 환경 없이 실행할 수 있습니다.

```bash
cd /home/acus03/amr_ws
PYTHONPATH=src/amr_system/amr_remote_bridge python3 -m amr_remote_bridge.server \
  --host 100.88.249.77 --port 8081 \
  --token-file ~/.config/amr_remote_bridge/token --robot-id amr-01
```

다른 PC로 복사하는 최소 구성은 새 `amr_remote_bridge` 폴더 전체와 기존
`amr_web_ui/web` 폴더입니다. 복사한 패키지 폴더에서 실행할 때 정적 파일 위치를 지정합니다.

```bash
cd /path/to/amr_remote_bridge
python3 -m amr_remote_bridge.server \
  --host 100.88.249.77 --port 8081 \
  --token-file /path/to/token --assets-dir /path/to/amr_web_ui/web
```

Windows PC도 Python 모듈로 실행할 수 있습니다. 터미널의 Python 실행 명령과 경로를
해당 환경에 맞게 사용합니다. ROS 전용 빌드·실행 명령은 AMR 측에서 사용할 수 있습니다.

## AMR에서 기존 웹 UI와 새 클라이언트 실행

새 패키지만 빌드합니다. 기존 웹 UI는 이미 설치되어 있어야 합니다.

```bash
cd /home/acus03/amr_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select amr_remote_bridge
source install/setup.bash
```

터미널 1 — 기존 웹 UI를 로컬 API로 실행합니다. 기존 stack·localization·navigation은
평소 실행 절차로 별도 준비합니다. 동일 포트에 기존 웹 UI가 실행 중이면 중복 실행하지 않습니다.

```bash
amr
ros2 launch amr_web_ui web_ui.launch.py host:=127.0.0.1 port:=8080
```

터미널 2 — 클라이언트가 중앙 서버에 접속합니다.

```bash
amr
ros2 run amr_remote_bridge remote_client \
  --server-url http://100.88.249.77:8081 \
  --token-file ~/.config/amr_remote_bridge/token --robot-id amr-01
```

또는 런치 파일을 사용할 수 있습니다.

```bash
amr
ros2 launch amr_remote_bridge remote_client.launch.py \
  server_url:=http://100.88.249.77:8081 robot_id:=amr-01
```

빌드 없이 모듈로 실행하는 방법:

```bash
cd /home/acus03/amr_ws
PYTHONPATH=src/amr_system/amr_remote_bridge python3 -m amr_remote_bridge.agent \
  --server-url http://100.88.249.77:8081 \
  --token-file ~/.config/amr_remote_bridge/token --robot-id amr-01
```

## 브라우저 접속

- 기본 로봇 화면: `http://100.88.249.77:8081/`
- 로봇 지정: `http://100.88.249.77:8081/robots/amr-01/`
- 등록 로봇 목록: `http://100.88.249.77:8081/fleet`

호스트 PC에서 직접 볼 때에도 Tailscale 주소로 접속합니다. 다른 PC도 같은 tailnet에 등록되어
해당 서버 포트에 접근할 권한이 있으면 같은 주소로 접속할 수 있습니다.
AMR ID는 영문·숫자·`_`·`-`로 1~64자입니다. 다른 실제 AMR에는 다른 ID를 지정합니다.
동일한 ID로 동시에 접속하는 두 클라이언트는 허용하지 않습니다.

## 명령·연결 정책

- 중앙 서버는 로컬 웹 API의 응답을 받아야 HTTP 202를 반환합니다.
  이는 기존 API의 요청 접수이며 ROS 목표 접수·주행 성공은 이후 상태 데이터로 확인합니다.
- 브라우저와 서버가 사용하는 지도 버전은 중앙 서버가 관리합니다. AMR으로 전달할 때 로컬
  지도 버전으로 변환하고, 클라이언트는 PNG 해시도 비교한 뒤 위치 명령을 전달합니다.
- 중앙 상태는 기본 2초가 지나면 HTTP 503으로 처리합니다. 기존 UI가 연결 끊김으로 표시하고
  조작 버튼을 비활성화합니다. 센서 데이터의 age에 수집·전송·중앙 보관 지연을 반영합니다.
- 클라이언트는 연결 복구를 재시도합니다. **명령은 자동 재전송하지 않습니다.**
  전달 전 만료는 HTTP 408, 전달 후 결과를 확인하지 못한 경우는 HTTP 504로 반환합니다.
  통신 시간은 AMR의 명령 TTL에서 보수적으로 차감합니다.
- 명령 ID 중복은 다시 실행하지 않습니다. 같은 ID에 다른 명령은 거부합니다.
  중앙과 클라이언트의 기록은 최근 256개를 메모리에 유지합니다. ID를 새 요청에 재사용하지 마세요.
- 통신 복구 시 과거 목표를 자동 실행하거나 서버 재시작 후 작업을 자동 복원하지 않습니다.
- **연결 단절은 새 원격 명령을 차단하지만, 이미 로컬 주행 시스템이 접수한 목표를 자동 정지·취소하지
  않습니다.** 단절 시 일시정지·명시적 재개는 기존 목표 관리기와 연동할 별도 정책입니다.
- 취소 대상은 기존 `amr_web_ui`가 보유한 목표입니다. 다른 ROS 클라이언트의 목표나 하드웨어
  비상정지를 대신하지 않습니다. 원격 수동 속도 조작·웹 AUTO 전환은 이번 범위에 포함하지 않습니다.

## 검증

실제 외부망에서 AMR 측 시험을 진행할 때는 [AMR 통신 테스트 실행법](../../../docs/AMR_REMOTE_COMM_TEST.md)을 사용합니다.

물리 CAN·센서·ROS 노드를 시작하지 않고, 기존 웹 API의 미리보기 서버와 새 중앙 서버를
실제 loopback HTTP로 연결합니다.

```bash
cd /home/acus03/amr_ws
PYTHONPATH=src/amr_system/amr_remote_bridge:src/amr_system/amr_web_ui \
  python3 -m pytest -q src/amr_system/amr_remote_bridge/test/test_remote_bridge.py
```

상태·PNG·기존 UI 제공, 세 명령의 왕복 전달, 로컬 검증 오류, 인증·Origin 검사,
센서 age·상태 만료, 로컬 API 장애, 단절 후 명령 만료·재전송 방지, 동일 버전의 지도 교체,
중복 AMR 세션, 로봇별 경로 분리를 검증합니다.

테더링/Tailscale 실제 시험에서는 상태 표시 → 무구동 명령 확인 → 기존 현장 절차에 따른 주행 순서로
확인하고, 테더링을 끊었다 복원하여 재연결·명령 미재전송을 확인합니다. 실제 Tailscale 경로의
지연·대역폭·층간 통신 품질은 loopback 자동 시험으로 검증되지 않습니다.

실제 Chromium WebDriver를 사용할 수 있는 이 개발 환경에서는 다음 화면 시험도 제공합니다.
새 프로세스는 미리보기 지도만 사용하며 물리 주행 명령을 보내지 않습니다.

```bash
PYTHONPATH=src/amr_system/amr_remote_bridge:src/amr_system/amr_web_ui \
  python3 src/amr_system/amr_remote_bridge/test/browser_check.py
```

결과는 `diagnostics/amr_remote_bridge/browser_result.json`과 `central_ui.png`에 저장합니다.
지도 렌더링·터치 목표 전달·연결 단절 시 버튼 비활성화·재접속 후 목표 미재전송·취소를 확인합니다.
