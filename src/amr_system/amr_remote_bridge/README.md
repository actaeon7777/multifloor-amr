# AMR Remote Bridge — protocol 2

로봇 에이전트가 로컬 `/api/state`를 읽어 최대 16 KiB의 요약을 중앙 서버에 보낸다.
중앙 서버는 로봇별 카드를 표시하고 `stop`, `resume`, `return_home`만 중계한다.
지도·costmap·카메라 이미지를 읽거나 전송하지 않으며, 로봇 웹 UI 자산에 의존하지 않는다.

- 에이전트 → 서버: `{protocol: 2, robot_id, session_id, sequence, state_age, summary, local_error, results}`.
- 서버 → 에이전트: `{protocol: 2, session_id, sequence, commands, accepted_results}`.
- 명령: `{command_id, kind, payload: {request_id: command_id}, ttl}`.
- 결과: `{command_id, http_status, body}`. 로컬 응답을 그대로 돌려준다.
- protocol 1은 서버에서 409, 에이전트에서 RuntimeError로 거부한다.

`summary.py`는 층(이동 중 출발층), 구역, x/y/yaw/age, 작업, 모드, 내비 상태,
E-stop, 관리자 정지, 목표·층간 미션 요약, 배터리, 위치 확인 상태, 알림, 대기 요청만 추린다.
누락된 블록은 null/default로 처리한다. 서버는 필드 타입과 요약 크기를 검증한다.

서버의 `GET /api/robots`는 로봇 목록 배열을 반환한다. 연결/로컬 상태가 만료되면 `online=false`,
`summary=null`이며 새 명령을 거부한다. `GET /`, `/console.js`, `/console.css`는 콘솔 자산이다.
`POST /robots/<id>/api/{stop,resume,return_home}`에는 `X-AMR-Request: 1`, JSON, 유효한
`request_id`만 필요하다. `/api/...`는 설정한 기본 로봇에 보낸다.
운영자 인증은 Basic(`operator`와 공유 토큰), 에이전트는 Bearer다.

활성 세션의 robot_id를 다른 에이전트가 차지할 수 없고 시퀀스 재사용도 거부한다.
로봇마다 미완료 명령은 하나뿐이다(409). 큐에서 만료되면 408, 전달 후 결과를 확인하지
못하면 504이며 자동 재전송하지 않는다. 로컬 API와 에이전트는 요청 ID로 중복 실행을 막는다.
정지는 최대 수 초 지연될 수 있는 소프트웨어 정지이며 E-stop이 아니다.

서버 실행·Slack DM 설정은 [Control_Server](../../../Control_Server/README.md),
로봇 실행은 [운용 문서](../../../docs/README.md#4-중앙-관제-사용-시)를 참고한다.
Slack 토큰과 호실→사용자 매핑은 서버 PC에만 둔다. 알림은 비동기 큐에서 보내며,
진행 상태를 먼저 본 배송 목표의 SUCCEEDED 전환만 대상이다. 복귀·성공 상태 최초 관측은 제외한다.

```bash
PYTHONDONTWRITEBYTECODE=1 python3 -m pytest -q -p no:cacheprovider src/amr_system/amr_remote_bridge/test
```

시험은 preview 상태 및 명령 기록용 로컬 HTTP 스텁과 가짜 Slack 전송만 사용한다.
`test/browser_check.py`는 이전 전체 UI 규약용 도구이므로 새 콘솔에는 사용하지 않는다.
새 콘솔의 선택적 브라우저 검증은 `Control_Server/scripts/browser_check.py`를 사용한다.
