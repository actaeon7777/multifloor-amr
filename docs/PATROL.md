# 현재 층 고정 경로 순찰 (1단계)

`amr_patrol/patrol_manager`는 설정 파일의 경유점을 차례로 주행하고 건물 home으로 복귀한다.
기본 경로는 `4F_full`이며 4F에서만 시작할 수 있다. 탐지·영상 저장·층간 이동은 아직 없다.
관측 정지의 시작·종료와 순찰 이벤트를 발행해 후속 탐지 기능이 연결할 수 있게 한다.

`experiment.launch.py`의 `start_patrol` 기본값은 `true`이며 real과 sim 모두 적용된다.
노드 시작만으로 순찰이 시작되지는 않는다. 명시적 start 명령과 입력 인터록 통과가 필요하다.
실차 launch는 CAN 송신 기본값이 실제 송신이므로 소프트웨어 확인 목적으로 실행하지 않는다.

## 경로와 상태

경로 파일은 `amr_stack/config/patrol.yaml`, 건물·home의 기준은 `amr_stack/config/floors.yaml`이다.
설치 시 기존 `config/*.yaml` glob에 포함된다. 기본 home은 4F 401호의 `goal`이다.
경로 이름은 `[A-Za-z0-9_-]{1,32}`, floor는 건물에 등록된 층, end는 `home` 또는 `stay`다.
`end: home`은 같은 층에 유효한 home이 있어야 한다. 다른 층 home으로 이동시키지 않는다.
경유점은 하나 이상이어야 하며 모든 좌표·각도·대기 시간은 유한한 수여야 한다.
경유점에 허용되는 키는 `x y yaw dwell_s note`뿐이다. `dwell` 같은 오타는 허용 키 목록과 함께 거부한다.

| 경유점 필드 | 의미 |
|---|---|
| `x`, `y` | 해당 층 map 좌표(m), 필수 |
| `yaw` | rad. 생략 시 이전 경유점에서 해당 지점을 향하는 방향, 첫 지점은 0 |
| `dwell_s` | 0 이상. 생략 시 `default_dwell_s`(저장소 설정은 0초 = 추가 대기 없음) |
| `note` | 상태의 detail에 표시할 설명, 생략 시 빈 문자열 |

`patrol.yaml`은 `default_dwell_s: 0.0`이며 4F 경로 12지점 중 7곳에 3초의 추가 관측 대기를 둔다.
각 지점은 별도 `NavigateToPose` 목표이므로 Nav2의 `StoppedGoalChecker`와 yaw 허용 오차에 따라
모든 지점에서 정지하고 방향을 정렬한다. `dwell_s: 0`도 무정지 통과를 뜻하지 않는다.
진정한 무정지 통과에는 `NavigateThroughPoses`가 필요하며 현재 구현하지 않았다.
“통과” 지점의 yaw를 생략하면 이동 방향을 목표 방향으로 사용하므로 도착 시 정렬 회전이 작아진다.
yaw가 없는 관측 지점도 같은 방향을 사용하고, 추가 대기 동안에는 별도 회전 목표를 보내지 않는다.
Nav2 목표는 항상 유효한 quaternion으로 만들고, 피드백의 map 위치가 있으면 성공 결과와 함께
`goal_xy_tolerance` 이내인지 확인한다. 위치 피드백이 없으면 action 성공 결과를 사용한다.

`IDLE → NAV → (DWELL) → NAV … → RETURN → DONE` 순서로 진행한다.
`end: stay`는 마지막 지점에서 DONE이 된다. 실패한 지점은 지정 횟수만큼 재시도한 뒤 건너뛰고,
연속 실패 제한에 도달하면 ABORTED가 된다. 성공한 지점은 연속 실패 수를 초기화한다.
home 복귀도 재시도하지만, 복귀 실패는 건너뛰거나 DONE으로 처리하지 않고 ABORTED로 끝난다.
DONE·ABORTED는 다음 유효한 start까지 유지된다.
실패 뒤 재전송 또는 다음 지점 출발 전에는 `retry_delay_s`만큼 기다린다. MANUAL 상태에서는
`수동 조작 중: 순찰 대기`를 표시하고 목표 전송·실패 집계를 멈춘다. MANUAL 또는 hold 중 수신한
거부 결과도 실패 횟수를 소모하지 않는다. 명시적인 관리자 hold 수신은 아래의 즉시 중단 계약을 따른다.

| 파라미터 | 기본값 | 검증 |
|---|---|---|
| `config` | amr_stack share의 `config/patrol.yaml` | 비어 있지 않은 파일 경로 |
| `tick_period` | 0.2초 | 양의 유한 수; 상태 갱신 간격 상한은 1초 |
| `nav_timeout_s` | 180초 | 양의 유한 수, 목표 접수 대기 포함 |
| `max_retries` | 1 | 0 이상 정수, 최초 시도 외 추가 횟수 |
| `retry_delay_s` | 3초 | 0 이상 유한 수, 실패 후 재전송·다음 지점 출발 대기 |
| `max_consecutive_failures` | 3 | 1 이상 정수, 재시도 소진한 연속 지점 수 |
| `min_battery_voltage` | 26.1V | 0 이상 유한 수, 0이면 비활성 |
| `battery_low_s` | 10초 | 0 이상 유한 수, 조기 복귀에 필요한 연속 저전압 시간 |
| `goal_xy_tolerance` | 0.3m | 양의 유한 수 |
| `require_estop` | true | bool; experiment launch에서 real=true, sim=false |
| `cancel_result_timeout_s` | 10초 | 양의 유한 수, 비활성 순찰의 취소 결과 대기 상한(단조 시계) |

## 인터록과 목표 소유권

시작 시 알려진 현재 층이 경로와 같고 confidence가 CONFIRMED·BEACON·ASSUMED 중 하나여야 한다.
UNKNOWN·TRANSIT, 엘리베이터 내부, 관리자 hold, 정상 임대가 없는 E-stop,
RUNNING·MANEUVER·STOPPING 내비게이션, 저전압, 이미 진행 중인 순찰은 시작을 거부한다.
E-stop은 `/estop/status`의 `HealthLease`를 사용하며 정상 heartbeat가 만료되면 중단한다.
`require_estop: false`이면 순찰 노드의 E-stop 임대 검사를 생략하므로 상태 스트림 없이 시작할 수 있다.
시뮬레이션 launch는 이 값을 false로 전달한다. navigation_manager의 주행 인터록은 계속 적용된다.
층 정보가 없거나 기본 floor4 시뮬레이션에서 floor monitor를 실행하지 않으면 start가 거부된다.

stop·관리자 hold·E-stop은 현재 목표를 취소하고 각각 `순찰 정지 요청`·`관리자 정지`·`E-stop`으로
ABORTED가 된다. 자동 복귀·자동 재개는 없다. 순찰 도중 층이 달라지거나 불명확해져도 중단한다.
기준 미만의 전압이 `battery_low_s` 동안 연속 관측되면 현재 목표를 취소하고 같은 층 home으로 조기 복귀한다.
가속 중 순간 전압 강하는 이 지속시간을 채우지 않으면 복귀시키지 않는다. 완료 detail은
`배터리 부족으로 조기 복귀`다. `end: stay`도 저전압이면 home으로 복귀하며, 같은 층 home이 없으면 중단한다.
관측 중 중단·조기 복귀에도 observation end를 발행한다.

배터리는 `/vehicle/status`의 `battery_voltage_available`이 true인 유한 전압만 사용한다.
시작 시 0.5초보다 오래된 수신 값은 미확인으로 처리한다. 미확인 배터리는 저전압으로 추정하지 않는다.
정상 전압·미확인 값·0.5초를 넘는 수신 공백은 연속 저전압 타이머를 초기화한다.
시작 거부 판정은 지속시간 없이 현재의 유효 전압으로 한다.
SOC나 충전 상태를 추정하지 않는다.

주행은 navigation_manager의 `/navigate_to_pose` action으로만 보낸다. 직접 속도를 발행하지 않는다.
중단 후 늦게 도착한 목표 접수 응답도 해당 handle로 취소한다. 이전 목표의 terminal 결과를 받기 전에는
다음 목표를 전송하지 않으며, 취소 대기 시간은 다음 목표의 제한 시간에 포함하지 않는다.
실패 결과 후 자체 실행 목표가 없는데 내비게이션이 RUNNING·WAITING_NAV2·MANEUVER이면 다른 목표가
주행을 맡은 것으로 보고 `다른 주행 명령으로 순찰 중단`으로 ABORTED 처리한다. 재시도로 목표를 빼앗지 않는다.
순찰이 비활성일 때 취소 전송 후 `cancel_result_timeout_s`가 지나거나 action 서버가 사라지면
종료 결과가 없는 목표 참조를 경고 로그와 함께 해제한다. 늦은 결과는 새 순찰에 반영하지 않는다.
순찰이 활성인 동안에는 취소 결과를 기다리며 목표 중첩을 막는다.
SIGINT·SIGTERM 종료 시 ROS 컨텍스트를 유지한 채 `순찰 노드 종료`로 중단하고 최대 0.9초 동안
취소 요청·응답을 처리한 다음 컨텍스트를 종료한다.
ROS 시간 역행 시 순찰을 중단한다. 관측·재시도 대기와 저전압 지속시간은 ROS 시간을 사용한다.

## 토픽 계약

모든 JSON 메시지는 `std_msgs/String`이며 시각 필드는 ROS 초다.

| 토픽 | QoS | 내용 |
|---|---|---|
| `/patrol/command` | depth 10 | start/stop 요청 |
| `/patrol/status` | depth 1, transient local | 변경 시와 active 동안 최소 1초마다 상태 |
| `/patrol/observation` | depth 10 | 관측 begin/end |
| `/robot/event` | depth 10 | 순찰 생명주기·지점 건너뛰기 이벤트 |

명령 예시(JSON만 표시):

```json
{"command":"start","route":"4F_full","request_id":"patrol_00000001"}
{"command":"stop","request_id":"patrol_00000002"}
```

request_id는 `[A-Za-z0-9_-]{8,80}`이다. 알 수 없는 명령·경로·잘못된 JSON은 상태의
`last_command`에 `accepted: false`와 한국어 사유를 남긴다. 해석할 수 없는 ID는 null이다.
stop은 비활성 상태에서도 접수되며 기존 종료 상태를 유지한다.

상태 필드:

```text
active, route, floor, phase, index, total, detail, started_at, elapsed_s,
skipped, routes, last_command, stamp
```

phase는 IDLE·NAV·DWELL·RETURN·DONE·ABORTED다. index와 skipped는 1부터 시작하고,
RETURN에서는 `index == total + 1`이다. 아직 시작하지 않았으면 route·floor·started_at은 null,
index·total은 0이다. routes는 모든 설정 경로 이름이다. elapsed_s는 종료 후 고정된다.
last_command는 `{request_id, accepted, detail}` 또는 null이다.

관측 메시지는 `{event, route, index, x, y, yaw, dwell_s, stamp}`이며 event는 begin/end다.
0초 대기 지점에는 관측 메시지를 보내지 않는다. 좌표·yaw는 관측 지점의 목표 자세다.
이벤트 kind는 `patrol.started`, `patrol.finished`, `patrol.aborted`, `patrol.waypoint_skipped`다.
이벤트는 `{id, kind, severity, text, floor, room, pose, image_path, stamp}`로 구성된다.
id는 UUID, severity는 중단·건너뛰기에 warn이고 나머지는 info다. room은 빈 문자열,
pose는 해당 목표의 `[x,y,yaw]` 또는 null, image_path는 null이다.

## 시험

단위 시험과 문법 검사:

```bash
PYTHONDONTWRITEBYTECODE=1 PYTHONPATH=src/amr_system/amr_patrol:src/amr_system/amr_stack:src/amr_system/amr_control:$PYTHONPATH \
  python3 -m pytest -q -p no:cacheprovider src/amr_system/amr_patrol/test/test_routes.py \
  src/amr_system/amr_patrol/test/test_patrol.py src/amr_system/amr_stack/test/test_floor_config.py
python3 -m py_compile src/amr_system/amr_patrol/amr_patrol/patrol_node.py src/amr_system/amr_stack/launch/experiment.launch.py
```

격리 ROS 시험은 ROS 환경과 패키지가 준비된 Orchestrator가 실행한다. Worker는 실행하지 않는다.

```bash
ROS_DOMAIN_ID=198 ROS_LOCALHOST_ONLY=1 AMR_RUN_PATROL_TESTS=1 \
  python3 -m pytest -q src/amr_system/amr_patrol/test/test_patrol_ros.py
```

가짜 action 서버, 합성 floor·hold·E-stop·차량·내비 상태와 probe를 하나의 executor에서 사용한다.
정상 복귀·관측·retained 상태·heartbeat·명령 거부·중단·늦은 접수 취소·저전압·타임아웃을 검사한다.
층 변경·엘리베이터 진입 중단, 재시도 간격·MANUAL 대기, 다른 주행의 선점, E-stop 없는 sim 설정,
비활성 취소 목표 해제와 신호 종료 시 취소 처리 순서도 포함한다.
실제 Nav2 지도 주행과 실차 정지 성능을 검증하는 시험은 아니다. 전체 격리 러너에는 `patrol` 스위트로 등록된다.
