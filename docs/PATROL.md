# 현재 층 고정 경로 순찰과 이동 장애물 감지

`amr_patrol/patrol_manager`는 설정 파일의 경유점을 차례로 주행하고 건물 home으로 복귀한다.
기본 경로는 `4F_full`이며 4F에서만 시작할 수 있다. `motion_detector`는 순찰 중 지도에 없는
이동 장애물을 감지해 JPEG와 `security.motion` 이벤트를 저장한다. 층간 순찰은 지원하지 않는다.

`experiment.launch.py`의 `start_patrol` 기본값은 `true`이며 real과 sim 모두 적용된다.
이 옵션은 `patrol_manager`와 `motion_detector`를 함께 시작한다. 노드 시작만으로 순찰이
시작되지는 않는다. 명시적 start 명령과 입력 인터록 통과가 필요하다.
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

모든 JSON 메시지는 `std_msgs/String`이며 `stamp`는 ROS 초다. 스풀의 `wall_time`은 Unix 초다.

| 토픽 | QoS | 내용 |
|---|---|---|
| `/patrol/command` | depth 10 | start/stop 요청 |
| `/patrol/status` | depth 1, transient local | 변경 시와 active 동안 최소 1초마다 상태 |
| `/patrol/observation` | depth 10 | 관측 begin/end |
| `/robot/event` | depth 10 | 순찰 생명주기·지점 건너뛰기·`security.motion` 이벤트 |
| `/patrol/detector_status` | depth 1, transient local | 감지 활성 전환 및 활성 동안 1 Hz 상태 |

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

## 지도에 없는 이동 장애물 감지 (2단계)

`/patrol/status`가 `active: true`이고 phase가 NAV·DWELL·RETURN일 때만 감지한다.
시작 시 `/map`(transient local), `/global_costmap/costmap`(transient local),
`/global_costmap/costmap_updates`, `/camera/color/image_raw/compressed`(depth 1, best effort)를
구독한다. TF Buffer·TransformListener도 시작 시 생성하고 종료 시 `/tf`·`/tf_static`을 포함한
여섯 구독을 해제한다. 지도·영상·추적 캐시도 폐기한다. 층 변경 때는 구독을 다시
만들어 정적 지도를 재수신한다. `/robot/event`와 순찰 상태 구독은 비활성 동안에도 유지한다.

global costmap은 `nav_msgs/OccupancyGrid`의 **100만 lethal**로 취급한다. 99(inscribed),
inflation, 정적 지도 unknown은 후보가 아니다. 정적 occupied를 사각형 8방향으로 2셀 팽창한
영역도 제외한다. 정적 지도와 costmap의 원점·해상도·yaw가 다르면 셀 중심을 map 좌표로 변환해
정적 지도에 대입한다. 비용이 큰 정적 마스크·좌표 매핑은 캐시하고, updates는 로컬 배열에 적용한다.
업데이트 경계·데이터 길이가 잘못되면 배열을 변경하지 않는다.

현재 Nav2 설정은 `always_send_full_costmap: false`다. 전체 costmap이 1 Hz로 새로 온다고
가정하지 않는다. 활성화 직후 transient-local 전체 그리드는 Nav2 시작 당시 자료일 수 있으므로
coverage 마스크를 모두 false로 시작하고, 이후 받은 `costmap_updates` 사각형만 true로 표시한다.
후보는 coverage가 확인된 셀에 한정한다. 전체 그리드를 다시 받으면 coverage와 트랙을 초기화한다.
활성화 후 `warmup_s`(기본 5초) 동안에는 트랙만 구성하고 감지 이벤트·재알림 이력은 만들지 않는다.

후보의 8방향 연결 성분 중 크기·셀 수 조건에 맞는 클러스터를 추적한다. 크기는 셀 면적을 포함한
bounding box의 대각선이다(0.5×0.5m 정사각형은 약 0.707m). 거리 필터로 셀을 자르기 전에
연결 성분을 만들고, 중심점이 범위 안에 있어도 bounding box가 로봇 제외 반경이나 최대 거리 원에
닿는 클러스터는 통째로 버린다. 로봇 이동에 따라 잘린 정적 물체의 중심이 움직이는 오탐을 막는다.

직전 처리 프레임에서 관측한 트랙만 최단 중심 거리 순서로 일대일 매칭한다. 매칭 거리는
`min(match_radius, 경과 시간 × max_person_speed)`로 제한한다. 빈 프레임·크기 필터 탈락 후
다시 나타난 객체에는 새 트랙을 부여한다. 각 관측은 중심에서 가까운 후보 셀 최대 64개를 보관한다.
이동 창 안의 가장 오래된 관측(트랙이 짧으면 첫 관측)을 원점으로 삼아, 이동량이 임계값 이상이고
그 원점 셀 중 `vacated_fraction` 이상이 현재 costmap에서 더 이상 lethal이 아닐 때만 감지한다.
현재 coverage가 없는 셀은 비워졌다고 세지 않는다. 원래 자리가 그대로 점유된 채 관측 영역이
자라거나 잘려 중심만 달라지는 경우를 억제한다. 사람 크기와 움직임에 대한 기하학적 휴리스틱이며
사람을 분류하는 모델은 아니다.

TF `map → base_footprint`가 없으면 프레임을 건너뛴다. 프레임 사이 yaw의 최단 각도 차로 회전
속도를 계산하고, 회전 임계값 초과 시 프레임과 추적 이력을 버려 회전 전후 위치 흔들림을 연결하지
않는다. ROS 시간 역행·지도 교체·층 변경도 추적을 초기화한다. 타이머는 새 costmap 수신/패치가
있을 때만 프레임을 처리하므로 같은 배열을 반복 처리해 `min_frames`를 채우지 않는다.
별도의 최근 감지 위치·시각 목록은 회전·트랙 만료·지도 교체에도 유지한다. `repeat_s` 이내에
감지했던 위치의 `repeat_radius_m` 반경에서는 새 트랙도 재알림하지 않는다. 새 활성 구간·층 변경·
ROS 시간 역행은 이 억제 이력도 초기화한다.

`patrol.yaml`의 선택적 `exclusions`는 생략 시 빈 목록이다. 좌표는 유한한 수이며 min < max,
floor는 건물에 등록된 층이어야 한다. 경계의 셀 중심도 제외한다. 기본 설정:

```yaml
exclusions:
  - {floor: "4F", x_min: 8.5, x_max: 11.6, y_min: 1.8, y_max: 4.8, note: "E1 엘리베이터 문·홀"}
```

`motion_detector` 파라미터(거리 m, 시간 s, 각속도 rad/s):

| 파라미터 | 기본값 | 의미·검증 |
|---|---|---|
| `config` | amr_stack share의 `config/patrol.yaml` | 경로·제외 영역 설정 |
| `static_margin_cells` | 2 | 0 이상 정수, 정적 장애물 팽창 |
| `ignore_radius`, `max_range` | 0.6, 8.0 | 로봇 중심 제외 거리 ≥0, 최대 거리 > 제외 거리 |
| `min_diameter_m`, `max_diameter_m` | 0.25, 1.2 | 양수, 최대 ≥ 최소 |
| `min_cells` | 4 | 양의 정수 |
| `match_radius` | 0.8 | 양수, 중심점 매칭 거리 |
| `max_person_speed` | 1.5 | 양수, 경과 시간에 따른 매칭 거리 상한(m/s) |
| `vacated_fraction` | 0.6 | 0 초과 1 이하, 원점 점유 셀의 최소 비움 비율 |
| `move_threshold_m`, `move_window_s` | 0.35, 4.0 | 양수, 이동량과 이력 창 |
| `min_frames` | 2 | 2 이상 정수 |
| `track_timeout_s` | 5.0 | 양수, 마지막 관측 후 유지 시간 |
| `repeat_s`, `global_repeat_s` | 30.0, 10.0 | 0 이상, 동일 트랙/전체 재감지 간격 |
| `repeat_radius_m` | 1.0 | 양수, 트랙 소실 후에도 적용하는 위치별 재감지 억제 반경 |
| `max_yaw_rate` | 0.4 | 0 이상, 회전 중 감지 억제 |
| `detect_period` | 0.5 | 양수, 최신 그리드 검사 주기 |
| `warmup_s` | 5.0 | 0 이상, 활성화 후 트랙만 구성하고 알림하지 않는 시간 |
| `camera_max_age_s` | 3.0 | 양수, 단조 시계 기준 JPEG 수신 후 유효 시간 |
| `status_period` | 1.0 | 양수, 활성 상태 발행 주기 |
| `spool_dir` | `~/amr_data/events/queue` | 이벤트 큐 경로 |
| `image_dir` | `~/amr_data/security` | JPEG 경로 |
| `spool_max_files` | 2000 | 양의 정수, 초과 시 가장 오래된 이벤트 삭제·경고 |
| `image_keep_days` | 7.0 | 양수, 이미지 보관 일수 |

수치 파라미터는 NaN·무한대를 거부한다. 경로는 빈 문자열을 허용하지 않는다.
감지 상태 JSON은 `{active, tracks, detections, last_detection_stamp, camera_fresh}`다.
`detections`는 현재 활성 구간의 누적 수, `last_detection_stamp`는 ROS 초 또는 null이다.

## 스냅숏과 이벤트 스풀 계약

감지 시 최근 3초 이내 수신한 JPEG가 있으면 `image_dir/<YYYYmmdd-HHMMSS>-<id>.jpg`에 저장한다.
영상이 없거나 오래됐거나 파일 저장에 실패하면 이벤트의 `image_path`는 null이다. 저장 성공 시에는
절대 경로를 넣는다. 카메라는 최신 메시지 한 장만 참조하고, 스냅숏을 쓸 때만 bytes로 변환한다.

감지 이벤트의 kind는 `security.motion`, severity는 `alert`, text는
`4F (3.8, 3.2)에서 지도에 없는 움직임 감지` 형식이다. pose는 장애물 중심 `[x,y,0]`,
room은 빈 문자열이다. `extra`에는 `diameter_m`, `displacement_m`, `cells`, `track_id`,
`robot_pose: [x,y,yaw]`를 넣는다. 이 시점의 Unix 초 `wall_time`도 발행한다.

같은 프로세스의 `/robot/event` depth 10 구독이 모든 이벤트(`patrol.*` 포함)를 저장한다.
감지 발행 경로에서 JSON을 별도로 쓰지 않으므로 자체 메시지는 한 번만 저장된다. 스풀은 원래
이벤트 필드를 보존하고 `wall_time`이 없으면 저장 시 추가한다. 발행 메시지에 `extra`가 없으면
스풀에 `extra: null`을 추가해 uploader가 검증하는 전체 키 집합을 유지한다.

```text
{id, kind, severity, text, floor, room, pose, image_path, stamp, wall_time, extra}
```

id는 소문자 hex 32자리다. 큐 파일 이름은 `<unix_ms:013d>-<id>.json`이며, 같은 디렉터리의
`<name>.tmp`에 UTF-8 JSON을 쓰고 flush/fsync 후 `os.replace`한다. uploader는 `.json`만 읽는다.
저장 중 큐 디렉터리가 사라져 `FileNotFoundError`가 발생하면 디렉터리를 다시 만들고 한 번 재시도한다.
큐 파일은 전송 완료 시 별도 비-ROS uploader가 삭제한다. 스풀러는 **한도 초과 때만** 파일명
시각 순서로 가장 오래된 큐 파일을 버리고 경고한다. 나이에 따른 큐 삭제는 하지 않는다.
JPEG는 시작 시와 60초 유지관리 주기에 검사한다. 먼저 큐 JSON의 `image_path`를 수집하고,
큐에서 참조하지 않는 이미지 중 mtime 기준 7일이 지난 파일만 삭제한다. 큐를 완전히 읽지 못하면
해당 회차 이미지 삭제를 건너뛴다. 전송 대기 이벤트가 참조하는 이미지는 보관 기한이 지나도 유지한다.
네트워크 업로드와 중앙 서버 수신은 다른 컴포넌트의 책임이다.

## 시험

단위 시험과 문법 검사:

```bash
PYTHONDONTWRITEBYTECODE=1 PYTHONPATH=src/amr_system/amr_patrol:src/amr_system/amr_stack:src/amr_system/amr_control:$PYTHONPATH \
  python3 -m pytest -q -p no:cacheprovider src/amr_system/amr_patrol/test/test_routes.py \
  src/amr_system/amr_patrol/test/test_patrol.py src/amr_system/amr_patrol/test/test_detection.py \
  src/amr_system/amr_patrol/test/test_spool.py
python3 -m py_compile src/amr_system/amr_patrol/amr_patrol/detector_node.py src/amr_system/amr_stack/launch/experiment.launch.py
```

격리 ROS 시험은 ROS 환경과 패키지가 준비된 Orchestrator가 실행한다. Worker는 실행하지 않는다.

```bash
ROS_DOMAIN_ID=198 ROS_LOCALHOST_ONLY=1 AMR_RUN_PATROL_TESTS=1 \
  python3 -m pytest -q src/amr_system/amr_patrol/test/test_patrol_ros.py \
  src/amr_system/amr_patrol/test/test_detector_ros.py
```

가짜 action 서버, 합성 floor·hold·E-stop·차량·내비 상태와 probe를 하나의 executor에서 사용한다.
정상 복귀·관측·retained 상태·heartbeat·명령 거부·중단·늦은 접수 취소·저전압·타임아웃을 검사한다.
층 변경·엘리베이터 진입 중단, 재시도 간격·MANUAL 대기, 다른 주행의 선점, E-stop 없는 sim 설정,
비활성 취소 목표 해제와 신호 종료 시 취소 처리 순서도 포함한다.
실제 Nav2 지도 주행과 실차 정지 성능을 검증하는 시험은 아니다. 전체 격리 러너에는 `patrol` 스위트로 등록된다.
감지 통합 시험은 합성 지도 두 프레임/패치·static TF·압축 영상으로 단일 이벤트·이미지·스풀,
종료 시 센서·TF 구독 해제, 비활성 때 순찰 이벤트 스풀, 층 변경 시 지도 재구독과 coverage 초기화를 확인한다.
`test_detector_ros.py`는 위 직접 명령으로 실행한다(기존 격리 러너 등록 변경은 별도 범위).
단위 시험의 1014×462 합성 지도 검사는 캐시 준비 후 5프레임 중앙값 50ms 미만을 요구한다.
실제 센서·추적 밀도·Jetson 부하에서의 지연과 오탐률은 현장 검증이 필요하다.
