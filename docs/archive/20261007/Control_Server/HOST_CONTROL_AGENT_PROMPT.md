> **이력 문서** · 2026-10-07 정리 시점의 이전 안내입니다. 현재 실행은 [최신 운영 문서](../../../README.md), 검증은 [검증 문서](../../../VALIDATION.md)를 사용합니다.

# AMR 중앙 관제 서버 구축 — 호스트 PC 에이전트용 프롬프트

이 문서 전체를 호스트 PC의 에이전트에게 작업 지시로 전달한다. 아래의 현재 구현 설명은
2026-10-06 기준이며, 복사된 실제 소스와 차이가 있으면 소스를 확인하여 차이를 먼저 설명한다.

## 1. 수행할 작업과 호스트 환경

너는 AMR 중앙 관제 서버를 구축하는 개발 에이전트다. 아래 요구사항을 읽고 호스트 PC에서
환경 준비, 서버 구현, 실행, 테스트, 실행 문서 작성까지 수행해라.

- 호스트 OS는 Ubuntu 22 계열이다. 먼저 `/etc/os-release`로 정확한 버전을 확인해라.
  Ubuntu 22.04라면 시스템 Python 3.10을 기준으로 작업할 수 있다.
- 서버는 Python으로 구축한다. 앞선 설계 논의에 따라 **FastAPI + Uvicorn을 기본 선택**으로 한다.
- 현재 호스트 PC에는 **Tailscale이 설치되지 않았다**. 설치와 같은 tailnet 등록을 준비해야 한다.
- AMR은 Jetson에서 기존 ROS 2 주행 시스템을 실행하며, 새 Python 클라이언트가 이미 작성되어 있다.
- 1차 목표는 **기존 클라이언트를 수정하지 않고 중앙 서버에서 기존 UI로 모니터링하고
  목표·초기 위치·목표 취소 명령을 요청할 수 있게 만드는 것**이다.
- 먼저 AMR 1대, ID `amr-01`, 중앙 서버 포트 `8081`을 사용한다.
- 초기 AMR 회선은 스마트폰 모바일 데이터의 USB/Wi-Fi 테더링이다. 이후 LAN 연결이 가능한
  LTE/5G 유심 라우터로 교체한다. 호스트 PC는 연구실 인터넷 등 별도 회선을 사용한다.
- AMR과 호스트 PC 모두 Tailscale에 등록한다. 스마트폰은 인터넷을 제공하면 된다.
- 호스트 PC에는 ROS 2를 설치할 필요가 없다. ROS 분산 발견이나 DDS를 외부망으로 연결하는
  구현을 만들지 말고, 아래 HTTP 규약으로 통신해라.

기존 프로젝트와 `AGENTS.md`가 있으면 먼저 확인하고 작업 위치를 정해라. 새 프로젝트가 필요하면
`~/amr_control_host`를 기본 후보로 사용하되, 현재 호스트의 저장소 구조를 우선한다.
이 문서의 `/home/acus03/amr_ws`는 **AMR 개발 환경의 원본 경로**이며 호스트에 있다고 가정하지 마라.

## 2. 먼저 확보하고 읽을 소스

원본 환경에서 필요한 파일은 다음 위치에 있다.

```text
/home/acus03/amr_ws/src/amr_system/amr_remote_bridge/
  amr_remote_bridge/
    agent.py             AMR 측 송신·명령 실행 클라이언트
    server.py            이미 구현된 표준 라이브러리 중앙 서버와 Registry
    common.py            프로토콜 상수·HTTP 도구·검증
    __init__.py
  web/                   기존 중앙 서버의 로봇 목록 페이지
  launch/                AMR용 클라이언트 런치 파일
  test/test_remote_bridge.py
  test/browser_check.py
  README.md
  setup.py
  setup.cfg
  package.xml
  resource/

/home/acus03/amr_ws/src/amr_system/amr_web_ui/
  amr_web_ui/server.py    기존 로컬 HTTP API와 SharedState
  amr_web_ui/geometry.py  지도 PNG 생성·좌표 검증
  web/                   기존 지도·주행 화면의 정적 파일
  README.md
```

중앙 서버 실행에 필요한 최소 자료는 `amr_remote_bridge` 폴더 전체와 `amr_web_ui/web`이다.
기존 클라이언트와 로컬 API를 이용한 호환성 테스트까지 하려면 `amr_web_ui` 폴더도 전체 복사해라.
테스트의 상대 경로가 유지되도록 두 패키지를 같은 부모 디렉터리에 배치하는 것이 좋다.

```text
~/amr_control_host/
  vendor/
    amr_remote_bridge/
    amr_web_ui/
  app/                   새 FastAPI 호스트 애플리케이션
  tests/                 새 서버에 대한 호환성 테스트
  scripts/
  README.md
```

실제 소스가 아직 없다면 해당 파일의 위치를 먼저 검색해라. 없으면 사용자에게 위 두 폴더의
전달이 필요함을 구체적으로 알려라. 그동안 OS 확인, Tailscale 준비, Python 환경 준비를 진행해라.
임의로 프로토콜을 만들어 호환된다고 보고하지 마라.

읽는 순서는 `common.py` → `agent.py` → `server.py` → 원본 UI의 `app.js` → 테스트다.
이 문서는 전달 자료이고 **실제 소스가 구현의 최종 기준**이다.

## 3. 현재 구현과 데이터 흐름

현재 중앙 서버는 FastAPI가 아니라 Python `ThreadingHTTPServer`로 구현되어 있으며,
AMR 클라이언트도 Python 표준 라이브러리만 사용한다.

```text
AMR 내부
  ROS 2 / Nav2 / navigation_manager / control_mux
              ↕
  기존 amr_web_ui: http://127.0.0.1:8080
              ↕ 로컬 HTTP API
  새 remote_client
              │ AMR이 먼저 HTTP POST를 시작
              │ 스마트폰 테더링 → 인터넷 → Tailscale 가상망
              ▼
호스트 PC
  중앙 서버: http://<호스트 Tailscale IP>:8081
              ↕
  브라우저: 기존 지도 UI / 로봇 목록
```

현재는 **WebSocket, MQTT, 서버에서 AMR로의 역방향 새 TCP 연결을 사용하지 않는다.**
AMR이 기본 5Hz로 `POST /api/agent/exchange`를 보내고 중앙 서버가 그 HTTP 응답에
명령을 넣으면 AMR이 로컬 API로 전달한다. 다음 교환 요청에 로컬 API의 처리 결과를 담는다.

클라이언트는 ROS 토픽을 직접 중복 구독하지 않는다. 이미 ROS 데이터를 모으는 기존
`amr_web_ui`에서 상태와 PNG를 읽는다. 원본 `amr_web_ui`와 AMR의 주행 제어 정책은 유지한다.

클라이언트 실행 옵션의 현재 기본값:

| 옵션 | 값·의미 |
|---|---|
| `--server-url` | 필수. 호스트의 기본 URL, 예: `http://100.x.y.z:8081` |
| `--token-file` | 필수. 공통 인증 키 파일 |
| `--robot-id` | `amr-01` |
| `--local-url` | `http://127.0.0.1:8080`; loopback 주소만 허용 |
| `--rate` | `5`, Hz |
| `--timeout` | `1.5`, 초; 기본 교환 HTTP 타임아웃 |
| `--layer-refresh` | `5`, 초; 같은 지도 버전도 주기적으로 다시 확인 |

실패하면 지수 백오프로 연결을 재시도하며 최대 대기는 기본 10초다.
클라이언트 프로세스 시작 때 새 `session_id`를 생성하고, 같은 프로세스의 재연결에는 이를 유지한다.

## 4. 중앙 서버에서 재사용할 구현

기존 `server.py`의 `Registry`, `Robot`, `Command`, `ApiError`와 `common.py`의 검증 코드는
ROS와 분리되어 있다. **가능하면 Registry를 재사용하고 FastAPI 라우팅·인증·정적 파일 응답만
새 호스트 애플리케이션으로 감싸라.** 동일한 큐·지도 버전·만료 규칙을 처음부터 재작성하지 마라.

Registry에서 사용하는 핵심 메서드:

```text
exchange(message)                 AMR 상태·PNG·결과 수신, ACK와 명령 반환
snapshot(robot_id)                센서 age와 중앙 지도 버전을 반영한 상태 조회
layer(robot_id, name)             PNG bytes와 중앙 버전 반환
queue(robot_id, kind, payload)    사용자 명령 검증·등록·중복 확인
wait_result(robot_id, command)    AMR이 보낸 로컬 API 응답 또는 만료 결과 대기
roster()                         등록 로봇과 연결·상태 갱신 정보 조회
```

기존 Registry는 메모리 상태, `threading.RLock`, `threading.Event`를 사용한다.
`wait_result()` 같은 대기 함수를 FastAPI의 `async def` 안에서 직접 호출하여 이벤트 루프를
막지 마라. 동기 라우트 또는 명시적인 thread pool 실행을 사용하고, 명령 응답을 기다리는 동안에도
교환 요청과 상태 조회가 계속 처리되는지 테스트해라. [FastAPI 동기·비동기 실행 문서](https://fastapi.tiangolo.com/async/)

초기 Uvicorn은 **worker 1개**로 실행한다. 여러 worker는 메모리 Registry·큐·이미지를
분리하여 서로 다른 상태를 볼 수 있으므로, 공유 저장소를 설계하기 전에는 늘리지 마라.
실제 연결 검증에서는 자동 reload도 사용하지 않는다. [FastAPI 프로세스·메모리 설명](https://fastapi.tiangolo.com/deployment/concepts/)

## 5. 유지할 AMR 교환 규약: protocol 1

### 5.1 요청

정확한 경로는 **`POST /api/agent/exchange`**이며 마지막 `/`가 없다.

```http
Authorization: Bearer <공통 인증 키>
Content-Type: application/json
```

| JSON 필드 | 형식·처리 |
|---|---|
| `protocol` | 정수 `1` |
| `robot_id` | `[A-Za-z0-9_-]{1,64}` |
| `session_id` | 소문자 16진수 32자, 클라이언트 시작 때 생성 |
| `sequence` | 1부터 증가하는 정수. 같은 세션에서 재사용·역순이면 거부 |
| `state` | 기존 `/api/state`의 JSON 객체 또는 로컬 API 장애 시 `null` |
| `state_age` | 초 단위의 유한한 비음수 숫자. 수집 소요 시간과 이전 교환 RTT를 포함 |
| `layers` | 변경·재확인된 PNG의 사전. 키는 `map`, `local_costmap`, `global_costmap` |
| `results` | 아직 중앙 ACK를 받지 못한 로컬 명령 응답의 배열 |
| `local_error` | 로컬 API 장애 안내 문자열 또는 `null` |

다음 JSON은 **로컬 API 장애 시 보내는 교환 형식**이다. 통신·인증 시험용이며 지도·제어가
정상이라는 의미는 없다. 실제 모니터링에는 클라이언트가 수집한 `state`와 PNG가 필요하다.

```json
{
  "protocol": 1,
  "robot_id": "amr-01",
  "session_id": "0123456789abcdef0123456789abcdef",
  "sequence": 1,
  "state": null,
  "state_age": 0.0,
  "layers": {},
  "results": [],
  "local_error": "local web API unavailable"
}
```

이 패킷도 유효한 교환으로 ACK할 수 있지만, 상태 조회와 주행 명령은 503으로 막아야 한다.
교환 요청이 들어온다는 이유만으로 오래된 상태를 최신으로 표시하지 마라.

`state`에는 다음 데이터가 포함된다. 알려지지 않은 정상 필드를 임의로 삭제하지 말고
기존 UI가 사용할 수 있도록 보존해라.

```text
preview, navigation, mode, action_ready,
robot, amcl, scan, global_path, local_path, footprint, velocity,
goal, notice, pending_request, tf_error, layers
```

위치·스캔·경로·속도 등의 객체에는 `age`가 있다. 서로 다른 장치의 monotonic 시각을
그대로 비교하지 말고 `state_age`와 호스트 수신 후 경과 시간을 사용해 지연을 반영한다.
PNG 업데이트는 다음 구조이며 `version`은 AMR의 원본 버전이다.

```text
layers[레이어 이름] = {"version": 원본 정수 버전, "png": 실제 PNG bytes의 base64 문자열}
```

state의 레이어 메타데이터와 PNG 버전이 일치해야 한다. 실제 PNG를 사용해라.
교환 본문 제한은 8 MiB, 디코딩된 개별 PNG 제한은 2 MiB다.
JSON의 NaN·Infinity를 허용하지 않는다.

### 5.2 성공 응답

항상 HTTP 200의 JSON으로 즉시 반환해라. 현재 클라이언트의 타임아웃이 기본 1.5초이므로
교환 요청을 장시간 long polling으로 보관하지 마라.

```json
{
  "protocol": 1,
  "session_id": "0123456789abcdef0123456789abcdef",
  "sequence": 1,
  "commands": [],
  "accepted_results": [],
  "accepted_layers": [],
  "needed_layers": []
}
```

- `protocol`, `session_id`, `sequence`는 요청과 일치해야 한다. 불일치하면 클라이언트가 실패로 처리한다.
- `commands`: 이번 응답에 전달할 명령 배열. 기존 구현은 교환당 최대 1개를 전달한다.
- `accepted_results`: 확인한 `command_id`의 문자열 배열. 결과가 처리된 후 ACK하고 반복 수신해도
  명령을 다시 실행하지 않는다. 기존 호스트는 재시작 후 알 수 없는 과거 결과도 ACK하여 버릴 수 있다.
- `accepted_layers`: 수신·검증한 레이어 **이름 문자열** 배열이다. 이미지 객체나 버전 사전이 아니다.
- `needed_layers`: 캐시가 없거나 버전이 다른 레이어 이름 배열. 호스트 재시작 뒤 클라이언트가
  이미지를 다시 보내도록 사용한다.

클라이언트의 HTTP 도구는 redirect를 따르지 않으며 환경 HTTP proxy를 사용하지 않는다.
FastAPI의 trailing-slash redirect나 다른 로그인 주소로의 301/302/307/308 응답을 만들지 마라.
현재 `--server-url`은 경로·쿼리·URL 인증정보가 없는 기본 URL만 허용한다.
호환 엔드포인트를 `/api/v1/...`로 옮기거나 WebSocket만 제공하면 기존 클라이언트가 작동하지 않는다.

## 6. 사용자 명령과 결과 규약

명령 종류는 `goal`, `initialpose`, `cancel` 세 가지다. 서버가 교환 응답에 넣는 구조는 다음과 같다.

```text
{
  "command_id": "host_goal_0001",
  "kind": "goal",
  "payload": {
    "request_id": "host_goal_0001",
    "map_version": AMR의 원본 지도 버전,
    "x": 목표 X 좌표(m),
    "y": 목표 Y 좌표(m),
    "yaw": 목표 방향(rad)
  },
  "map_digest": AMR 원본 지도 PNG bytes의 실제 SHA-256 hexdigest,
  "ttl": 호스트에서 남은 유효 시간(초)
}
```

- `command_id`와 `payload.request_id`는 같아야 한다. 형식은 `[A-Za-z0-9_-]{8,80}`이다.
- `ttl`은 남은 초 단위 숫자이며, 클라이언트는 전체 교환 RTT와 로컬 확인 소요 시간을 차감한다.
  서로 다른 PC의 wall clock 동기화에 의존하지 않는다.
- `goal`과 `initialpose`에는 위치·방향·원본 지도 버전·실제 PNG 해시가 필요하다.
- `cancel`의 payload에는 `request_id`만 있으면 된다. 기존 서버는 빈 `map_digest`를 사용한다.
- 좌표는 `map` 프레임, yaw는 라디안이다. UI 입력 단위와 혼동하지 마라.
- 클라이언트는 지도 버전과 PNG SHA-256을 로컬에서 확인한 뒤 위치 명령을 전달한다.
- 최종 로컬 호출은 `POST /api/{kind}`이며 `X-AMR-Request: 1` 헤더를 추가한다.

AMR이 다음 교환 요청의 `results` 배열로 보내는 예:

```json
{
  "command_id": "host_goal_0001",
  "http_status": 202,
  "body": {
    "request_id": "host_goal_0001",
    "kind": "goal",
    "status": "QUEUED"
  }
}
```

이 결과는 **로컬 웹 API가 요청을 큐에 넣었다는 뜻**이다. ROS의 목표 접수나 주행 성공과
구분해라. 이후 `state.goal`, `state.notice`, `state.navigation`, `state.pending_request`로
ROS 접수·거절·도착·실패를 표시한다. 미리보기 모드에서는 로컬 결과가 `PREVIEW`일 수 있다.

사용자 POST 요청에는 로컬 API의 응답이 확인된 뒤 해당 HTTP 상태와 JSON body를 반환해라.
호스트 큐 등록만으로 202나 주행 성공을 반환하지 마라. 로컬 오류도 그대로 전달한다.

### 지도 버전은 두 개다

브라우저가 보는 **중앙 버전**과 AMR의 **원본 버전**을 분리한다.

1. 원본 버전·PNG 해시·세션이 달라지면 중앙 버전을 갱신한다.
2. 브라우저가 보낸 `map_version`이 현재 중앙 버전과 같은지 확인한다.
3. 명령 payload를 AMR 원본 버전으로 변환하고, 원본 PNG 해시를 `map_digest`에 넣는다.
4. PNG 응답의 `X-Map-Version`에는 중앙 버전을 반환한다.
5. 메타데이터와 일치하는 PNG가 준비되지 않은 레이어는 상태에서 잠시 제외한다.

예를 들어 브라우저 버전이 `1`, AMR 버전이 `17`이면 클라이언트에 전달하는 payload는 `17`이다.
중앙 버전 `1`을 그대로 보내면 AMR의 로컬 API가 거절한다. 기존 Registry가 이 변환을 구현한다.

## 7. 세션·만료·재접속 정책

기존 동작을 유지해라.

| 항목 | 현재 기준 |
|---|---|
| 상태 신선도 | 기본 2초. 수집 age와 마지막 교환 시각 모두 확인 |
| 신선하지 않은 상태·로컬 API 장애 | 상태 조회·새 제어 명령 HTTP 503 |
| 명령 유효 시간 | 기본 3초 |
| 전달 전 만료 | HTTP 408; 이후 재접속해도 전달하지 않음 |
| 전달 후 응답 불명 | HTTP 504; 실행 여부 불명으로 안내, 자동 재전송 안 함 |
| 요청 ID 기록 | 로봇별 최근 256개, 메모리 보관 |
| 같은 ID·같은 요청 | 기존 결과 재사용, 다시 전달하지 않음 |
| 같은 ID·다른 요청 | HTTP 409 |
| 처리 중 명령 | 로봇당 미완료 원격 요청 1개 |
| 같은 robot_id의 새 세션 | 기존 세션 마지막 교환 후 기본 3초 이내면 HTTP 409 |
| sequence 재사용·역순 | HTTP 409 |
| 서버·클라이언트 재시작 | 과거 명령을 자동 복원·재실행하지 않음 |

목표 전송 후 응답이 끊겼다고 곧바로 새 ID로 재시도하지 마라. 상태 조회로 실제 접수를 확인한다.
서버 재시작 시 지도 캐시를 다시 채우고, 전송 중 명령을 새 작업처럼 재등록하지 않는다.

**현재 연결 단절은 새 원격 명령만 막는다. 이미 AMR 내부에서 접수한 목표는 자동 정지·취소하지
않는다.** 통신 단절 시 일시정지·재개는 아직 구현되지 않은 별도 기능이다. 구현되어 있다고
설명하지 말고 이번 호환 서버 구축 중 임의로 추가하지 마라.

기존 수동·자동 모드와 조이스틱 RB 허용 정책도 유지한다. 중앙 서버 명령이 로컬 mux 정책을
우회하거나 직접 CAN·모터 명령을 보내도록 만들지 마라. 목표 취소는 기존 웹 UI가 보유한
GoalHandle 범위이며 다른 ROS 클라이언트의 목표를 일괄 취소하는 기능이 아니다.

## 8. 제공할 UI와 HTTP 경로

| 경로 | 역할 |
|---|---|
| `POST /api/agent/exchange` | AMR Bearer 인증, 교환 규약 |
| `GET /` | 기본 로봇의 기존 지도 UI |
| `GET /robots/{robot_id}/` | 해당 로봇 지도 UI |
| `GET /fleet` | 등록 로봇 목록 |
| `GET /api/robots` | `{"robots": [...]}` 목록 |
| `GET /robots/{robot_id}/api/state` | 기존 UI 상태 JSON |
| `GET /robots/{robot_id}/api/layers/{name}.png` | 지도 PNG와 `X-Map-Version` |
| `POST /robots/{robot_id}/api/goal` | 목적지·도착 방향 요청 |
| `POST /robots/{robot_id}/api/initialpose` | 초기 위치 설정 요청 |
| `POST /robots/{robot_id}/api/cancel` | 현재 웹 목표 취소 요청 |
| 기본 로봇의 `/api/state`, `/api/layers/...`, `/api/goal` 등 | 기존 서버의 루트 API 별칭 유지 |

기존 `app.js`는 `/api/...` 절대 경로를 사용한다. 서버에서 제공할 때 해당 로봇의
`/robots/{robot_id}/api/...`로 바꾸고 index의 app.js 주소도 맞춰라. 원본 정적 파일은 유지한다.
`geometry.js`, `style.css`, `favicon.svg`와 로봇 목록의 `fleet.js`, `fleet.css`도 제공해야 한다.
한 로봇의 화면 조작이 다른 로봇에게 전달되지 않는지 확인해라.

브라우저 인증은 기존과 같이 **HTTP Basic, 사용자 `operator`, 비밀번호 공통 토큰**으로 유지한다.
AMR 교환은 **Bearer 인증**이다. 첫 단계의 단일 운영자·공통 키 모델이며, 사용자별 세밀한
권한과 로봇별 키가 이미 구현되어 있다고 설명하지 마라.

브라우저 POST는 `Content-Type: application/json`, `X-AMR-Request: 1`을 요구한다.
`Origin`이 있으면 `Host`와 일치하는지 검사한다. 오류 JSON은 `{"error": "설명"}`으로 반환한다.
FastAPI의 기본 `{"detail": ...}`만 반환하면 기존 UI의 구체적인 오류 표시가 달라질 수 있다.
400·401·403·408·409·413·415·422·429·503·504의 기존 의미를 유지한다.
브라우저 명령 body는 4096 bytes 이하, 캐시 정책은 `no-store`이며 기존 CSP도 유지해라.

## 9. 호스트 PC 작업 순서

### A. 환경과 파일 확인

OS, Python, 실행 중 서버, 8081 포트 사용 여부, sudo 가능 여부를 확인해라.
기존 프로젝트 파일을 덮어쓰지 않고 필요한 소스를 찾는다. 기존 서버를 종료해야 한다면
대상이 이번 AMR 서버인지 확인해라.

### B. Tailscale 설치·로그인

공식 Linux 설치 절차를 사용해라. 임의 버전·미러를 추측하지 않는다.
공식 안내의 기본 설치와 시작 명령은 다음과 같다. [Tailscale Linux 설치](https://tailscale.com/docs/install/linux)

```bash
curl -fsSL https://tailscale.com/install.sh | sh
sudo tailscale up
```

설치·권한 승인은 현재 에이전트 환경의 도구 정책을 따른다. 사용자 로그인이 필요하면
생성된 로그인 절차를 안내하고, 계정 비밀번호·Tailscale 인증 키를 대화나 로그에 노출하지 마라.
사용자가 AMR과 동일한 tailnet을 선택해야 한다. 로그인 대기 중에도 독립적인 서버 코드·테스트
작업을 진행할 수 있다.

```bash
tailscale status
tailscale ip -4
tailscale ping <AMR의 실제 Tailscale IP>
```

AMR IP가 아직 없으면 호스트 등록과 로컬 검증까지 진행하고 AMR의 주소 확인이 남았다고 보고해라.
peer ping 성공과 서버 포트 접근 성공은 별도로 확인한다.

호스트 서버는 실제 **Tailscale IP에만 바인딩**한다. 로컬 자동 시험은 loopback을 사용한다.
공인 포트포워딩·Funnel·exit node·subnet router는 이 구성에 필요하지 않다.
방화벽 조정이 필요하면 기존 규칙을 확인하여 필요한 Tailscale 인터페이스·포트 범위에 한정한다.
방화벽 전체 비활성화나 교내 네트워크 정책 변경을 하지 마라.

### C. Python 개발 환경

Ubuntu 시스템 Python을 교체하지 않고 프로젝트 venv를 사용해라.

```bash
python3 -m venv .venv
source .venv/bin/activate
python -m pip install fastapi uvicorn pytest httpx pillow pyyaml
```

venv 모듈 등 OS 패키지가 없으면 필요한 패키지만 준비한다. 설치된 실제 의존성 버전을 기록한다.
Pillow·PyYAML은 기존 로컬 웹 API를 사용하는 호환성 테스트용이며, 호스트에 ROS를 설치할 이유가 아니다.
현재 프로젝트의 의존성 관리 방식이 있으면 이에 맞춰라.

### D. 인증 키 준비

기존에 사용 중인 키가 있는지 먼저 확인한다. 정상 키가 있으면 재사용하고 덮어쓰지 않는다.
없으면 `~/.config/amr_remote_bridge/token`에 공백 없는 32자 이상의 난수 키를 한 번 생성한다.
디렉터리 권한은 700, 파일은 600으로 설정한다.

서버가 읽는 키와 AMR이 읽는 키는 동일해야 한다. 사용자에게 안전한 파일 전달 방법과
AMR의 저장 경로를 안내한다. 키를 프롬프트·Git·접속 URL·로그에 넣지 않는다.
AMR에 별도 접근 권한이 없다면 파일 전달을 완료했다고 가정하지 마라.

### E. 기존 서버 기준 동작 확인 후 FastAPI 구현

복사된 `amr_remote_bridge` 모듈로 표준 라이브러리 서버를 loopback에서 실행하여
기준 API를 확인할 수 있다. 이는 FastAPI 변환의 비교 기준이다.

```bash
PYTHONPATH=vendor/amr_remote_bridge python -m amr_remote_bridge.server \
  --host 127.0.0.1 --port 8081 \
  --token-file "$HOME/.config/amr_remote_bridge/token" \
  --assets-dir vendor/amr_web_ui/web --robot-id amr-01
```

프로젝트 경로가 다르면 위 경로를 실제 배치에 맞춘다. 이후 해당 기준 서버만 정상 종료하고
FastAPI 서버가 같은 포트·규약을 제공하도록 구현한다.

- Registry를 프로세스 내 한 인스턴스로 공유한다.
- 교환·상태·PNG·명령·UI·인증·예외 처리의 transport 계층을 만든다.
- FastAPI 요청 모델의 자동 형 변환으로 기존 검증을 우회하지 않는다.
- startup/lifespan에서 경로·키·설정을 검증하고 오류 원인을 명확히 표시한다.
- 실행 스크립트에 실제 Tailscale IP, 포트, 키 파일, UI 파일 위치, 기본 로봇 ID를 연결한다.
- Uvicorn `--workers 1`로 실행하며, 서버가 먼저 접속되지 않는 AMR에도 빈 대기 화면을 제공한다.

기존 클라이언트의 HTTP 규약을 유지하는 호스트 구현에 집중한다. 이번 단계에서 WebSocket 전환,
새 UI 프레임워크, 자동 작업 배정, 충전·도킹, ROS 제어 정책 변경을 함께 수행하지 마라.

### F. 검증하고 실제 연결 확인

먼저 loopback의 모의/미리보기 로컬 API와 **변경하지 않은 Agent**를 새 FastAPI 서버에 연결한다.
실제 TCP HTTP를 사용하여 urllib 클라이언트까지 포함해 검증해라. FastAPI TestClient만 사용하면
redirect·인증·네트워크 타임아웃·동시 대기 문제를 놓칠 수 있다.

기존 19개 테스트는 기존 `create_server()`를 생성한다. 그대로 통과했다고 **새 FastAPI 서버가
검증됐다고 보고하지 마라.** 기존 테스트를 참고하여 새 서버 fixture 또는 별도의 호환성 테스트를 작성한다.

필수 검증:

1. 상태 JSON·센서 age·지도 PNG·`X-Map-Version`이 원본과 호환된다.
2. 기존 UI가 로봇별 API 주소를 사용하며 지도와 목표 상태를 표시한다.
3. goal·initialpose·cancel이 로컬 API에 정확히 한 번 전달된다.
4. 로컬 API의 QUEUED 접수와 이후 ROS 접수·거절·성공 상태가 구분된다.
5. 중앙 지도 버전과 원본 버전 변환 및 PNG 해시 검증이 맞다.
6. 인증 실패·교차 Origin·잘못된 ID·잘못된 좌표·잘못된 지도 버전이 거절된다.
7. state=null 또는 상태 지연 시 503과 UI 조작 비활성화가 적용된다.
8. 응답 분실·중복 요청·단절·재접속·서버 재시작에서 명령이 재실행되지 않는다.
9. 다른 AMR ID·동일 ID의 충돌 세션·역순 sequence를 올바르게 처리한다.
10. 사용자 명령 응답 대기 중에도 exchange와 상태 조회가 처리된다.

실제 AMR이 준비되면 Tailscale IP와 인증 파일을 맞추고 실제 상태·지도부터 확인한다.
실차 이동 시험은 사용자가 해당 시험을 요청하고 AMR 측 기존 실행 절차가 준비된 경우에 진행한다.
원격에서 stack·모터·CAN을 자동 시작하여 움직임을 만들지 마라.

소스 전체 트리가 복사되어 있으면 AMR 개발 환경의 기존 미리보기·자동 시험을 참고할 수 있다.
브라우저 시험 스크립트는 실제 지도 파일과 해당 환경의 Chromium 경로를 사용하므로,
호스트에서 파일·브라우저 경로를 확인한 뒤 적용한다. ROS 없는 미리보기와 실차 결과를 구분한다.

## 10. AMR 측에 전달할 실행 정보

호스트 작업 후 **실제 호스트 IP와 포트가 채워진 명령**을 사용자에게 전달해라.
AMR 측 기존 로컬 API와 클라이언트 명령의 형태는 다음과 같다.

```bash
# AMR 터미널 1: 기존 로컬 API. 이미 실행 중이면 중복 실행하지 않는다.
cd /home/acus03/amr_ws
source install/setup.bash
ros2 launch amr_web_ui web_ui.launch.py host:=127.0.0.1 port:=8080
```

```bash
# AMR 터미널 2: 아래 IP는 설명용이다. 최종 전달 시 실제 호스트 IP로 바꾼다.
cd /home/acus03/amr_ws
source install/setup.bash
ros2 run amr_remote_bridge remote_client \
  --server-url http://100.90.0.10:8081 \
  --token-file "$HOME/.config/amr_remote_bridge/token" \
  --robot-id amr-01
```

브라우저 접속은 서버의 Tailscale 주소를 사용한다. 예시 주소:

```text
http://100.90.0.10:8081/
http://100.90.0.10:8081/robots/amr-01/
http://100.90.0.10:8081/fleet
```

호스트에서 직접 보더라도 서버가 Tailscale IP에 바인딩되어 있으면 이 주소를 사용한다.
다른 장치도 해당 tailnet과 포트 접근 권한이 필요하다. 브라우저 로그인은 `operator`와 공통 키다.

AMR의 기존 stack·localization·navigation은 기존 절차로 실행한다. MANUAL에서 목표를 접수하면
출발을 기다리며 기존 RB AUTO 허용 정책을 따른다. 중앙 서버를 구축했다는 이유로 이를 변경하지 않는다.

## 11. 완료 기준과 최종 보고

필요한 파일을 만들고 테스트·실행까지 진행해라. 설명이나 계획만 제시하고 끝내지 마라.
외부 계정 로그인·소스 전달·실제 AMR 부재가 남으면 독립적으로 가능한 작업을 완료한 뒤
무엇을 사용자 또는 AMR 측에서 해야 하는지 구체적으로 남겨라.

최종 결과에는 다음을 포함한다.

- 새 호스트 프로젝트 경로와 FastAPI 실행·종료·재실행 명령.
- 실제 OS·Python·주요 의존성 버전과 재현 가능한 의존성 파일.
- Tailscale 설치·로그인 상태, 실제 호스트 Tailscale IP, 서버 바인딩 주소와 포트.
- 인증 파일의 경로와 AMR에 동일 파일을 전달할 방법. **키 내용은 출력하지 않는다.**
- 기존 UI 파일 위치, 브라우저 주소, 로그인 방법, 완성된 AMR 클라이언트 실행 명령.
- 새 FastAPI 서버 대상 테스트 결과와 실제 Tailscale/AMR 연결 결과를 별도로 보고.
- 실제 연결을 못 했다면 로컬 검증만 완료했다고 명시하고 남은 조건을 설명.
- README, 실행 스크립트, 설정 예시, 테스트 결과를 저장한다.
- 첫 단계의 메모리 상태·단일 worker·단일 운영자 인증·단절 시 기존 목표 자동 정지 없음이라는 제한.

작업 중 수정한 API가 있으면 기존 클라이언트에 미치는 영향을 설명한다. 이번 목표는
protocol 1을 유지한 실제 모니터링·명령 중계이므로, 호환성을 깨는 변경은 먼저 해결해라.

## 12. 공식 참고 자료

- [Tailscale Linux 설치](https://tailscale.com/docs/install/linux)
- [Tailscale 가상 IP 유지 방식](https://tailscale.com/docs/concepts/ip-and-dns-addresses)
- [Tailscale 직접·중계 연결](https://tailscale.com/docs/reference/connection-types)
- [FastAPI 수동 서버 실행](https://fastapi.tiangolo.com/deployment/manually/)
- [FastAPI 동기·비동기 실행](https://fastapi.tiangolo.com/async/)
- [FastAPI 배포와 프로세스 메모리](https://fastapi.tiangolo.com/deployment/concepts/)

Tailscale은 인터넷 연결이 끊긴 구간의 통신을 대신 제공하지 않는다. 스마트폰 테더링과
향후 LTE/5G 라우터의 층간 음영·재연결 품질은 별도로 실측한다.
