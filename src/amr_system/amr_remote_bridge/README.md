# AMR Remote Bridge — protocol 2

로봇 에이전트가 로컬 `/api/kiosk/state`를 읽어 최대 16 KiB의 요약을 중앙 서버에 보낸다.
중앙 서버는 로봇별 카드를 표시하고 `stop`, `resume`, `return_home`, `patrol_start`, `patrol_stop`을 중계한다.
지도·costmap·실시간 카메라 스트림을 전송하지 않으며, 로봇 웹 UI 자산에 의존하지 않는다.
감지 이벤트와 저장된 스냅숏은 별도 API로 전송해 **감지 기록**에 표시한다.

- 에이전트 → 서버: `{protocol: 2, robot_id, session_id, sequence, state_age, summary, local_error, results}`.
- 서버 → 에이전트: `{protocol: 2, session_id, sequence, commands, accepted_results}`.
- 명령: `{command_id, kind, payload: {request_id: command_id}, ttl}`. `patrol_start`의 payload에만 `route`를 추가한다.
- 결과: `{command_id, http_status, body}`. 로컬 응답을 그대로 돌려준다.
- protocol 1은 서버에서 409, 에이전트에서 RuntimeError로 거부한다.

`summary.py`는 층(이동 중 출발층), 구역, x/y/yaw/age, 작업, 모드, 내비 상태,
E-stop, 관리자 정지, 목표·층간 미션·순찰 요약, 배터리, 위치 확인 상태, 알림, 대기 요청만 추린다.
누락된 블록은 null/default로 처리한다. 서버는 필드 타입과 요약 크기를 검증한다.

서버의 `GET /api/robots`는 로봇 목록 배열을 반환한다. 연결/로컬 상태가 만료되면 `online=false`,
`summary=null`이며 새 명령을 거부한다. `GET /`, `/console.js`, `/console.css`는 콘솔 자산이다.
`POST /robots/<id>/api/{stop,resume,return_home}`에는 `X-AMR-Request: 1`, JSON, 유효한
`request_id`만 필요하다. `/api/...`는 설정한 기본 로봇에 보낸다.
순찰은 `POST /robots/<id>/api/patrol_start`에 `{request_id, route}`,
`POST /robots/<id>/api/patrol_stop`에 `{request_id}`를 보낸다. 경로 이름은
`[A-Za-z0-9_-]{1,32}`만 허용하며, 시작은 로봇이 알린 등록 경로만 받는다.
에이전트는 각각 로컬 `/api/patrol/start`, `/api/patrol/stop`으로 명시적으로 매핑한다.
순찰 요약은 `active, route, phase, index, total, detail, routes, age`이며 경로 목록은 최대 32개다.
HTTP 202는 요청 접수다. 로봇의 응답은 이후 요약의 `notice`에서 확인한다. 로컬 웹은
`/patrol/status.last_command`의 요청 ID가 일치하면 성공/실패 처리하고, 발행 후 3초 동안
응답이 없으면 `순찰 노드가 응답하지 않습니다.`로 실패 처리한다.
운영자 인증은 Basic(`operator`와 공유 토큰), 에이전트는 Bearer다.

활성 세션의 robot_id를 다른 에이전트가 차지할 수 없고 시퀀스 재사용도 거부한다.
로봇마다 미완료 명령은 하나뿐이다(409). 큐에서 만료되면 408, 전달 후 결과를 확인하지
못하면 504이며 자동 재전송하지 않는다. 로컬 API와 에이전트는 요청 ID로 중복 실행을 막는다.
정지는 최대 수 초 지연될 수 있는 소프트웨어 정지이며 E-stop이 아니다.

서버 실행·Slack DM 설정은 [Control_Server](../../../Control_Server/README.md),
로봇 실행은 [운용 문서](../../../docs/README.md#4-중앙-관제-사용-시)를 참고한다.
Slack 토큰과 호실→사용자 매핑은 서버 PC에만 둔다. 알림은 비동기 큐에서 보내며,
진행 상태를 먼저 본 배송 목표의 SUCCEEDED 전환만 대상이다. 복귀·성공 상태 최초 관측은 제외한다.


## 감지 기록과 스냅숏

에이전트는 매 exchange 뒤 `--spool-dir`(기본 `~/amr_data/events/queue`)에서
`<unix_ms:013d>-<32자리 소문자 hex id>.json`을 오래된 순으로 최대 20개 읽는다.
`.tmp`는 무시하며 형식·크기·파일명 검증에 실패한 JSON은 같은 디렉터리의 `bad/`로 옮기고 로그를 남긴다.
이벤트 필드는 `id, kind, severity, text, floor, room, pose, image_path, stamp, wall_time, extra`다.
`kind`는 `security.motion` 같은 소문자 점 구분 이름, `severity`는 `info/warn/alert`,
`text`는 최대 1024자, `floor/room`은 문자열 또는 null, `pose`는 `[x,y,yaw]` 또는 null이다.
두 시각은 유한한 숫자(초), `extra`는 최대 8 KiB의 객체 또는 null(생략 시 null),
`image_path`는 로봇의 절대 경로 또는 null이다.

- Bearer `POST /api/agent/events`: `{protocol:2, robot_id, session_id, events:[...]}`.
  이벤트당 최대 64 KiB, 한 요청 최대 20건. 응답은 `{accepted:[id...], want_images:[id...]}`다.
  ID 중복은 재삽입 없이 승인하며, 이미 저장된 이미지는 다시 요청하지 않는다.
- Bearer `POST /api/agent/images/<id>`: `Content-Type: image/jpeg`, JPEG magic으로 시작하는
  최대 400 KiB의 원시 본문. 저장·중복은 204, 미등록 ID는 404다.
  한 주기에 이미지 한 장만 보내고 나머지 JSON은 다음 주기에 처리한다.
- 승인된 JSON만 삭제한다. 이미지가 요청되면 이미지 204 응답까지 기다린다.
  400 KiB 초과 이미지는 로그를 남기고 생략한 뒤 JSON을 삭제한다.
  이미지가 사라졌거나 권한 오류·디렉터리 경로로 읽을 수 없으면 경고하고 JSON을 삭제한다.
  서버에는 이미지 없는 이벤트가 남는다. 업로더는 상태 exchange와 별도의 backoff로 재시도하므로
  업로드 실패가 상태·명령 교환의 재시도 간격을 늘리지 않는다. 실패 연속 구간마다 한 번 경고한다.
  서버의 영구 거절(408/429를 제외한 4xx)은 해당 요청의 JSON을 `bad/`로 옮긴다.
  로봇의 `~/amr_data/security/` 이미지 원본은 삭제하지 않는다.
- 운영자 Basic `GET /api/events?after=0&limit=50&robot=amr-01`: `events`와 `latest_seq`.
  `robot`과 `kind_prefix`는 선택, `limit`은 1–200이다. `after` 이후 변경된 이벤트를
  `wall_time` 내림차순으로 반환한다. `kind_prefix=security.`는 서버에서 감지만 조회한다.
  `GET /api/events/latest_seq`는 `{latest_seq}`만 반환한다.
  `GET /api/events/<id>/image.jpg`도 Basic 인증이 필요하며 private 캐시는 1시간이다.

서버는 `--data-dir` / `AMR_DATA_DIR`(기본 `Control_Server/data`) 아래
`events.sqlite`와 `images/<robot_id>/<id>.jpg`를 만든다.
`AMR_MAX_EVENTS`(5000건), `AMR_MAX_IMAGE_BYTES`(1073741824바이트)를 넘으면
먼저 수신한 이벤트부터 이미지와 함께 삭제한다. 순번은 재시작·삭제 이후에도 증가하며,
늦게 도착한 이미지도 새 순번을 부여해 `after` 조회로 반영한다. 클라이언트는 ID로 병합하고 `wall_time`으로 정렬해 늦은 이미지가 기록 순서를 바꾸지 않는다.

콘솔은 2초마다 순번을 확인해 최신 50건과 썸네일을 표시한다. **전체 / 감지만**으로
서버의 전체 보관 기록에서 `security.*`를 조회하고, 썸네일을 누르면 인증된 원본을 새 탭에서 연다.
**알림음 켜기/끄기**는 기본 꺼짐이며 브라우저에 설정을 기억한다. 켠 상태에서 새 감지를 받으면
짧은 3회 알림음과 제목 깜박임으로 알린다. 다시 방문했을 때는 화면 클릭 또는 키 입력으로
브라우저 오디오를 활성화해야 한다. 최초 로딩의 과거 기록에는 소리를 내지 않는다.
감지 이벤트는 Slack에 보내지 않는다.

```bash
PYTHONDONTWRITEBYTECODE=1 python3 -m pytest -q -p no:cacheprovider src/amr_system/amr_remote_bridge/test
```

시험은 preview 상태 및 명령 기록용 로컬 HTTP 스텁과 가짜 Slack 전송만 사용한다.
`test/browser_check.py`는 이전 전체 UI 규약용 도구이므로 새 콘솔에는 사용하지 않는다.
새 콘솔의 선택적 브라우저 검증은 `Control_Server/scripts/browser_check.py`를 사용한다.
