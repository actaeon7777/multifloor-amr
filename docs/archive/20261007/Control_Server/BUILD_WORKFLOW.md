> **이력 문서** · 2026-10-07 정리 시점의 이전 안내입니다. 현재 실행은 [최신 운영 문서](../../../README.md), 검증은 [검증 문서](../../../VALIDATION.md)를 사용합니다.

# 중앙 관제 서버 구축 워크플로

작성일: 2026-10-06
요구사항: `HOST_CONTROL_AGENT_PROMPT.md`
프로젝트 위치: `/home/kim/Control_Server`
상태: FastAPI 구현·로컬 TCP 및 브라우저 검증 완료. 호스트 Tailscale 운영 서버 실행 중.
실제 AMR 연결은 공통 키 전달·로컬 API·Agent 준비 확인 후 진행한다. 실행 정보는 `README.md` 참고.

## 목표와 설계 기준

기존 AMR Agent와 웹 UI를 유지하면서 FastAPI + Uvicorn 중앙 서버에서
상태·지도 모니터링과 goal·initialpose·cancel 명령 중계를 제공한다.
첫 대상은 `amr-01`, 포트는 `8081`이다. 실제 소스가 문서와 다르면 차이를
확인하고 실제 구현을 기준으로 호환성을 검증한다.

통신 흐름:

```text
ROS / Nav2 ↔ 기존 로컬 웹 API :8080 ↔ 기존 Agent
                                         ↓ 5Hz HTTP 교환 / Tailscale
                                  FastAPI 중앙 서버 :8081
                                         ↕
                                  기존 지도 UI / fleet
```

- 기존 Registry와 검증 로직을 우선 재사용한다.
- protocol 1, 기존 경로, 오류 JSON, 인증 방식과 명령 만료 정책을 유지한다.
- Registry는 프로세스 내 하나이며 Uvicorn worker는 1개다.
- 로컬 검증은 loopback, 운영은 실제 호스트 Tailscale IP에 바인딩한다.
- 호스트에 ROS를 설치하지 않는다.
- 연결 단절은 새 명령을 차단하며, 이미 접수한 목표를 자동 정지하지 않는다.
- 실제 이동 시험은 사용자가 요청하고 AMR 실행 절차가 준비된 뒤 수행한다.

## 현재 확인 결과

| 항목 | 확인 결과 |
|---|---|
| OS | Ubuntu 22.04.5 LTS |
| Python | 3.10.12 |
| 프로젝트 파일 | 요구사항·워크플로 문서 및 vendor 원본 두 패키지 |
| 원본 패키지 | `vendor/amr_remote_bridge`, `vendor/amr_web_ui` 전체 배치 확인 |
| Tailscale | 사용자 설치·로그인 후 직접 상태 확인; 1.102.5, 호스트 IP 100.88.249.77 |
| AMR IP·peer ping | 100.106.98.122, 호스트에서 direct ping 성공; 확인 시 4ms |
| 호스트 인증 파일 | `/home/kim/.config/amr_remote_bridge/token` 생성; 디렉터리 700, 파일 600 |
| AMR 인증·연결 상태 | AMR 측 작업 후 확인 예정; 로컬 구현의 선행 조건이 아님 |
| 실제 지도 자료 | 현재 vendor에서 지도 YAML·PNG·PGM을 발견하지 못함 |
| 8081 포트 | 새 서버가 100.88.249.77:8081에 바인딩되어 HTTP 응답 확인 |
| AGENTS.md | 확인한 상위 경로 및 프로젝트에서 발견하지 못함 |

## 실제 코드에서 확인한 설계와 재사용 범위

| 소스 | 확인 내용 | 호스트 적용 전략 |
|---|---|---|
| `vendor/amr_remote_bridge/amr_remote_bridge/common.py` | protocol 1, 8 MiB 교환·4096 bytes 명령 제한, 엄격 JSON 도구, 인증 파일 검증, redirect·환경 proxy 차단 | 상수·검증·직렬화 재사용 |
| `vendor/amr_remote_bridge/amr_remote_bridge/agent.py` | Agent가 로컬 상태·PNG를 수집하고 교환 응답 명령을 실행; 결과 ACK 및 최근 256개 실행 기록 유지 | 원본 그대로 사용하며 서버 transport만 맞춤 |
| `vendor/amr_remote_bridge/amr_remote_bridge/server.py` | Registry가 상태 신선도·지도 해시·중앙 버전·명령 큐·만료·세션 소유권을 구현 | Registry와 ApiError를 import하고 기존 HTTP Handler 동작을 FastAPI로 옮김 |
| `vendor/amr_web_ui/amr_web_ui/server.py` | SharedState, 로컬 HTTP 서버, preview 모드 제공; ROS import는 실제 운용 분기 안에 있음 | 로컬 호환 시험에서 SharedState와 create_http_server 사용 |
| `vendor/amr_web_ui/amr_web_ui/geometry.py` | Grid가 실제 PNG 생성·좌표/점유 셀 검증을 제공 | 시험용 지도 생성에 사용; 운영 호스트는 PNG를 수신·보관 |
| `vendor/amr_web_ui/web/` | 지도·센서·목표 렌더링, 200ms 주기 polling, 사용자 동작에 따른 명령, 단절 시 제어 비활성화 | 원본 유지, 응답 시 API 주소와 로봇 표시만 변환 |
| `vendor/amr_remote_bridge/web/` | fleet 목록과 로봇별 화면 링크 | 기존 정적 파일 재사용 |
| `vendor/amr_remote_bridge/test/test_remote_bridge.py` | 실제 TCP, 원본 Agent, 합성 Grid, 로컬 API를 묶는 fixture가 기존 create_server를 시험 | 같은 시나리오를 새 Uvicorn 서버 fixture로 검증 |

설계에 영향을 주는 실제 동작:

- Agent가 먼저 연결하므로 호스트는 AMR 접속 주소를 몰라도 exchange 요청을 받을 수 있다.
  AMR Tailscale IP는 후속 peer 연결 점검에 사용한다.
- 브라우저 명령 → Registry.queue → 다음 exchange 응답의 명령 → 로컬 API 호출 →
  다음 exchange의 결과 → Registry.wait_result 종료 → 브라우저 응답 순서다.
- 지도 점유 셀과 기존 운용 정책 검증은 AMR 로컬 API가 수행한다. 호스트는
  유효 좌표·중앙 지도 버전·요청 중복을 검증하고 원본 버전·해시를 전달한다.
- Registry의 기본값은 stale_after=2초, command_timeout=3초,
  lease_timeout=3초, max_robots=32다. 첫 운용 대상은 amr-01이지만
  원본의 로봇별 경로와 격리는 유지한다.
- wait_result는 threading.Event를 기다린다. FastAPI 이벤트 루프에서 직접 실행하지 않는다.
- import에 ament나 ROS가 필수인 구조가 아니다. 명시적 UI 경로와 원본 모듈 경로를
  연결하므로 호스트에 colcon 빌드·ROS 환경 source가 필요 없다.
- 로봇 미접속 시 `/api/state`는 503이지만 지도 HTML과 fleet는 제공할 수 있다.
- 중복 실행 방지는 프로세스 내 최근 256개 기록 범위의 정책이다. 서버·Agent 재시작을
  넘는 영구적인 exactly-once 보장을 추가하거나 있다고 설명하지 않는다.

요구사항 문서보다 실제 코드가 더 구체적이거나 보완이 필요한 부분:

- `state_age`는 원본 Registry에서 0~3600초로 제한한다. 이 범위를 유지한다.
- `local_error` 원문은 저장하지 않으며 state=null이면 고정 안내를 사용한다.
  초기 FastAPI 구현에서도 장애 상태 처리와 원본 동작을 유지한다.
- protocol 검사는 원본에서 값 비교만 하므로 True나 1.0도 1과 같다고 평가된다.
  새 HTTP 계층에서는 문서의 정수 1 조건을 명시적으로 검사한다.
  정상 Agent는 정수 1을 보내므로 클라이언트 변경이 필요 없다.
- common.loads는 NaN/Infinity 리터럴을 거절하지만 큰 지수 표기 숫자의 float
  overflow까지 직접 검사하지 않는다. 새 수신 계층에서 비유한 숫자를 거절하여
  상태 응답 직렬화 실패를 방지한다. 정상 상태의 미지 필드는 보존한다.
- 브라우저 스크립트는 AMR 워크스페이스 ROOT, 실제 지도 경로, Chromium·driver
  경로와 지도 폭 1014 확인을 고정한다. vendor를 수정하지 않고 호스트 시험을 작성한다.
- 웹 UI의 기존 시험 일부는 amr_stack 지도나 ROS 모듈이 필요하다. 전체 vendor
  테스트를 무조건 수집하지 않고, 호스트에 맞는 시험과 ROS 전용 시험을 구분한다.

## 1. 원본 확보와 호환성 분석 — 완료

필요한 두 패키지 전체를 같은 부모 아래 배치한다.

```text
Control_Server/
  vendor/
    amr_remote_bridge/
    amr_web_ui/
  app/
  tests/
  scripts/
  config/
  HOST_CONTROL_AGENT_PROMPT.md
  BUILD_WORKFLOW.md
  README.md
```

읽기 순서: `common.py` → `agent.py` → `server.py` → 웹 UI `app.js` → 테스트.
Registry 인터페이스, 설정 기본값, 정적 파일 변환, 오류 처리, 테스트의 외부
지도·브라우저 의존성을 확인했다. 원본은 보존하고 새 서버는 `app/`에 작성한다.

결과: 기존 프로토콜·큐·지도·세션 정책을 재사용할 수 있고, 위 소스 분석 표에
문서 대비 추가 조건과 보완점을 기록했다.

## 2. 실행 환경과 연결 준비

원본 확보와 독립적으로 진행할 수 있다.

1. sudo 사용 가능 여부, 실행 중 서버와 8081 점유 여부를 확인한다.
2. 프로젝트 `.venv`를 준비하고 FastAPI, Uvicorn, pytest, httpx, Pillow,
   PyYAML을 설치한다. 실제 설치 버전을 재현 가능한 의존성 파일에 기록한다.
3. 공식 절차를 확인하여 Tailscale을 설치하고 AMR과 동일한 tailnet에 등록한다.
   사용자 계정 로그인은 사용자가 수행한다.
4. 기존 인증 파일 유무를 내용 출력 없이 확인한다. 정상 키는 재사용한다.
   없으면 `~/.config/amr_remote_bridge/token`에 난수 키를 생성하며
   디렉터리 700, 파일 600을 적용한다. 이 경로 쓰기는 도구 권한 정책을 따른다.
5. 호스트 Tailscale IP와 AMR IP를 기록한다. 방화벽 변경이 필요하면 기존
   규칙을 확인하고 필요한 인터페이스·포트에 한정한다.

완료 기준: venv와 의존성 기록이 있고, 인증 파일 경로 및 Tailscale 로그인
상태가 확인된다. 로그인·AMR 주소가 대기 중이면 구현과 로컬 시험을 진행한다.

## 3. 기존 서버 기준 동작 확인

원본 표준 라이브러리 서버를 loopback에서 실행하여 API와 UI를 확인한다.
기존 테스트 결과는 기준 동작 확인용으로 기록한다. 사용한 기준 서버만 정상
종료하고 새 서버 시험으로 전환한다.

완료 기준: 원본 Agent·로컬 API를 이용한 시험 방법과 기준 응답이 확보되어 있다.

## 4. FastAPI 서버 구현

구현 순서:

1. 설정·키·UI 경로 검증 및 단일 Registry 생성.
2. Bearer 인증과 `POST /api/agent/exchange` 연결.
3. Basic 인증, 로봇 목록·상태·PNG 조회와 기본 로봇 API 별칭.
4. goal·initialpose·cancel 등록 및 로컬 API 결과 대기·반환.
5. 기존 지도 UI와 fleet 정적 파일 제공, 로봇별 API 주소 적용.
6. 오류 JSON, 본문 제한, JSON 엄격 검증, Origin, 요청 헤더,
   no-store, 기존 CSP와 redirect 방지 처리.
7. 실행·종료·재실행 스크립트와 비밀값 없는 설정 예시 작성.

명령 대기가 exchange·상태 조회를 막지 않도록 동기 라우트 또는 명시적
thread pool을 사용한다. FastAPI의 자동 형 변환으로 원본 검증을 우회하지 않는다.
브라우저 명령에는 로컬 API의 실제 HTTP 상태와 body를 반환하며,
중앙 큐 등록을 주행 성공으로 표시하지 않는다.

구체적인 구현 방식:

- 앱 생성 함수에 설정과 시험용 Registry를 주입할 수 있도록 구성한다.
- 요청 body는 크기를 제한하면서 읽고 common.loads로 해석한다. 전체 telemetry를
  자동 변환하는 Pydantic 모델로 덮어쓰지 않는다.
- async 라우트에서 body를 읽은 뒤 Registry 호출, 특히 queue/wait_result를
  명시적으로 thread pool에 넘긴다. 명령 대기 중 교환·조회 진행을 통합 시험으로 확인한다.
- 인증은 hmac.compare_digest와 원본 401 challenge를 유지한다.
- UI는 허용된 파일만 제공한다. HTML의 app.js 주소를 로봇별로 바꾸고
  해당 JS의 `/api/`를 `/robots/{robot_id}/api/`로 바꾼다.
- lifespan에서 token·robot_id·설정값·지도 UI·fleet 파일을 검증한다.
- 실행 스크립트가 프로젝트의 절대 경로로 vendor 모듈·UI 위치를 연결한다.
  로컬 모드는 127.0.0.1, 운영 모드는 확인한 Tailscale IPv4를 사용한다.
  Uvicorn worker 1개, 실제 연결 중 reload 비활성화로 실행한다.
- foreground 실행·Ctrl+C 종료를 기본으로 문서화하고, 재실행 시 대상 서버의
  포트 점유를 확인한다. 서비스 자동 시작 구성은 초기 구현 완료 후 필요에 따라 다룬다.

완료 기준: 기존 Agent를 수정하지 않고 새 서버와 통신하며, 기존 UI가
로봇별 상태·지도·명령 결과를 표시한다.

## 5. 새 서버 대상 로컬 호환성 검증

새 Uvicorn 서버를 실제 TCP HTTP로 실행하고 변경하지 않은 Agent를
모의 또는 미리보기 로컬 API에 연결한다. 기존 서버의 테스트 통과와
새 FastAPI 서버 검증 결과를 별도로 기록한다.

| 검증 영역 | 통과 조건 |
|---|---|
| 상태·지도 | 상태 필드 보존, 센서 age 반영, 실제 PNG 및 X-Map-Version 일치 |
| 지도 변환 | 중앙 버전 → 원본 버전 변환, PNG SHA-256 검증, 불일치 레이어 제외 |
| 명령 | 세 종류의 명령이 한 번 전달되고 로컬 오류·응답이 그대로 반환됨 |
| 결과 의미 | QUEUED/PREVIEW와 이후 ROS 접수·거절·성공이 구분됨 |
| 인증·입력 | 잘못된 인증·Origin·ID·좌표·지도 버전·JSON·본문 크기 거절 |
| 상태 장애 | state=null 또는 지연 상태에서 503 및 UI 제어 비활성화 |
| 중복·만료 | 동일 ID 재사용, 상이한 payload 충돌, 전달 전 408, 전달 후 불명 504 |
| 세션 | 충돌 세션·역순 sequence 거절, 다른 로봇 간 명령 분리 |
| 재접속·재시작 | 응답 분실·단절·재시작 시 과거 명령 자동 재실행 없음 |
| 동시성 | 명령 응답 대기 중 exchange와 상태 조회가 계속 처리됨 |
| HTTP·UI | redirect 없이 Agent 연결, 로봇별 API 주소 및 지도·목표 표시 확인 |

완료 기준: 새 FastAPI 서버를 대상으로 필수 검증을 통과하고 결과를 저장한다.
브라우저 시험에 필요한 지도와 실행 파일 경로는 호스트 환경에 맞춰 확인한다.
지도 자료 없이도 Grid로 자유 공간·장애물·미확인 셀을 포함한 시험 지도와
실제 PNG를 생성하여 로컬 API·Agent·새 서버·브라우저 경로를 검증한다.
실제 AMR 지도 시험은 사용자가 이후 지도 YAML과 참조 이미지 파일을 제공하거나
AMR이 실제 PNG를 전송할 때 수행한다.

## 6. Tailscale 및 실제 AMR 연결 검증

1. 서버를 실제 호스트 Tailscale IP:8081에 단일 worker로 실행한다.
2. 동일 인증 파일을 안전하게 AMR에 전달하고 전달 완료를 확인한다.
3. Tailscale peer ping과 서버 포트 접근을 각각 확인한다.
4. AMR의 기존 로컬 API와 Agent를 연결하여 실제 상태·지도를 먼저 확인한다.
5. 사용자가 요청한 경우에만 기존 AMR 운용 절차에 따라 실차 명령 시험을 진행한다.

완료 기준: 실제 연결 결과가 로컬 모의 시험과 구분되어 기록된다.
AMR이 없거나 로그인 대기 중이면 해당 조건을 미완료로 명시한다.

## 7. 실행 문서와 인계

README에 실제 프로젝트 경로, OS·Python·의존성 버전, 실행·종료·재실행 방법,
호스트 IP·바인딩·포트, 인증 파일 경로와 안전한 전달 절차, UI 경로·주소·로그인
방법, 실제 IP가 들어간 AMR Agent 명령, 시험 결과 및 남은 조건을 기록한다.
메모리 상태·단일 worker·단일 운영자·단절 시 기존 목표 자동 정지 없음의 제한도 명시한다.

## 구현 및 검증 결과

- 원본 서버 기준 시험 19개 통과: `diagnostics/baseline.xml`.
- 새 FastAPI/Uvicorn 대상 실제 TCP 시험 48개 통과: `diagnostics/fastapi.xml`.
- 실제 Chromium에서 지도·세 명령·단절·재접속·fleet 검증 통과:
  `diagnostics/browser_result.json`, `central_ui.png`, `fleet_ui.png`.
- 호스트 Tailscale IP 100.88.249.77:8081에서 인증·UI·API 확인:
  `diagnostics/host_result.json`. 실제 AMR peer를 통한 시험과 구분한다.
- 구현 파일·고정 의존성·실행/종료/재실행 스크립트 및 README 작성 완료.
- 현재 운영 서버는 로봇 접속을 기다린다. 실제 이동 시험은 수행하지 않았다.

## 실행 순서와 산출물

1. venv 및 필요한 의존성을 준비하고 설치 버전을 기록한다.
2. 기존 서버·포트·인증 파일을 확인한다. 호스트 키를 준비하되 AMR 측에
   기존 키가 확인되면 운영 연결 전에 사용할 공통 키를 맞춘다.
   로컬 자동 시험은 운영 키와 별도의 시험 전용 키를 사용한다.
3. 원본 서버의 ROS 없는 기준 시험을 실행하고 결과를 저장한다.
4. FastAPI 앱·실행 스크립트·설정 예시를 작성한다.
5. 새 서버용 TCP 호환 시험과 호스트용 브라우저 시험을 작성·실행한다.
6. 공식 절차로 Tailscale 설치·등록을 진행한다. 사용자 로그인이나 AMR
   준비를 기다리는 동안 위 코드·시험 작업을 계속한다.
7. 호스트 IP를 확인하면 운영 실행·접속 정보를 README에 채운다.
   실제 AMR 준비가 안 되어 있으면 로컬 검증 완료와 연결 대기를 구분하여 보고한다.

계획한 산출물:

```text
app/
  __init__.py
  config.py                 경로·토큰 파일·운용 설정
  main.py                   앱 생성·Registry·API·UI
  http.py                   인증·본문 검증·공통 응답
tests/
  conftest.py               실제 Uvicorn·로컬 API·원본 Agent fixture
  test_compatibility.py     정상 중계 및 인증·지도·세션·재접속 시험
  test_concurrency.py       명령 대기 중 교환·조회 진행 시험
scripts/
  run_server.sh             로컬/운영 실행, 단일 worker
  check_connection.sh       Tailscale 상태·peer·서버 접근 점검
  browser_check.py          호스트 환경에 맞춘 브라우저 시험
config/
  server.env.example        비밀값 없는 설정 예시
requirements*.txt           운영/시험 의존성과 실제 버전 기록
diagnostics/                기준·FastAPI·브라우저 검증 결과
README.md                   설치·실행·종료·AMR 연결 인계
```

파일 분리는 구현 규모에 맞춰 조정하되 vendor 소스와 운영 키는 수정·커밋하지 않는다.

## 이후 사용자 확인이 필요한 정보

현재 구현 전략 수립과 로컬 구현을 시작하기 위한 추가 정보는 없다.
아래 항목은 운영 연결 단계에서 확인한다.

- 호스트 등록에 사용할 Tailscale 계정/tailnet. 사용자 로그인은 설치 후 안내한다.
- AMR IP는 100.106.98.122로 확인하고 호스트에서 peer ping에 성공했다.
  기존 Agent·로컬 API·인증 파일 전달 상태는 확인이 필요하다.
- AMR 공통 인증 파일 유무와 경로. 키 내용은 대화에 보내지 않는다.
- 실제 지도 미리보기까지 원하면 지도 YAML과 YAML이 참조하는 이미지 파일.

남은 조건: AMR 로컬 API·Agent 실행과 동일 인증 파일 전달 확인.
Tailscale 설치는 sudo 비밀번호 때문에 사용자 터미널에서 수행했고, 이후 호스트 등록·IP를 직접 확인했다.

## 공식 실행 근거

- [FastAPI 동기·비동기 실행](https://fastapi.tiangolo.com/async/): 동기 대기 함수는
  동기 라우트 또는 명시적 thread pool 실행으로 처리한다.
- [FastAPI 배포와 메모리](https://fastapi.tiangolo.com/deployment/concepts/): 프로세스별
  메모리를 고려하여 원본 메모리 Registry는 단일 worker로 운영한다.
- [Tailscale Linux 설치](https://tailscale.com/docs/install/linux): Ubuntu 설치·등록은
  공식 절차를 확인하여 진행한다.
