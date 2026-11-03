# 층간 이동 (엘리베이터)

> 상태: **시뮬레이션 구현·검증 완료 (2026-10-14), 실차 미적용**. 범위는 G동 3F·4F·5F(지도 있음), 엘리베이터 1대(E1).
> **2026-11-02 RF 비콘·버튼 장치 연동을 제거했다**(2026-11-03 실차 시험 통과). 버튼은 로봇 안내를 듣고 사람이 누르고
> (SPS-B-KOLA-0001-7700 기반이라 로봇이 엘리베이터를 물리적으로 조작하지 않는다), 하차 층은 키오스크·웹의 **도착 확인**으로 확정한다.
> 같은 확인 통로를 나중에 RGB 층인식기(비전)가 쓰게 된다. 지도가 없는 1·2·6·7F는 통과층으로 설정에 남긴다(§5).
> 그전 RF 구현과 프로토콜 문서는 커밋 a4421e6까지의 이력에 있다.
> 공용 로직은 실차 인터페이스 기준으로 작성했고, 실차 전용 노드는 기본으로 꺼져 있다.
> 시뮬레이션 수치는 실물 보정 전 모델의 결과다. 구현 단계별 기록은 [실험 기록](EXPERIMENTS.md#2026-10-1314--층간-이동엘리베이터-구현).

## 1. 요구 사항 (2026-10-13 확정)

| 항목 | 결정 |
|---|---|
| 문 열림·닫힘 | AMR이 **LiDAR로 직접 판별**. 장치는 문 상태를 알리지 않는다 |
| 버튼 호출·층 선택 | 로봇은 안내(`call`·`waiting` 음성, 키오스크 "▲/▼ 버튼을 눌러 주세요"·"N층 버튼을 눌러 주세요")와 `/elevator/request`만 낸다. **사람이 누른다.** 확인 응답(ACK)은 없다 |
| 도착 층 확정 | 카 안에서 문이 열리면 목표 층을 후보로 내고, 키오스크·웹의 "네 / 아니요" 답(`/floor/arrival_confirm`)으로 확정한다. 나중에는 RGB 층인식기가 같은 통로로 답한다 |
| 안/밖 판별 | 문 상태 + 위치 추정 + LiDAR 둘러싸임을 함께 사용 |
| 시작 층 | 운영자가 지정한다(`/floor/set_current`, 웹 층 지정) |
| 카 내부 "열림" 버튼 | 누르지 않는다. 문 열림 유지는 엘리베이터 자체 시간과 후퇴 규칙에만 기댄다 |
| 치수 | 카 내부 1.87 × 2.23 m, 출입구 1.10 m(카 왼쪽 벽에서 0.385 m), 복도 벽면 ~ 카 안쪽 벽 0.66 m ([`images/Elevator_size.jpg`](images/Elevator_size.jpg)). 지도의 `door_center`는 복도 벽면이 아니라 **0.37 m 안쪽의 홀 문 패널**이라 설정은 `hall_recess_m: 0.37`(벽면~패널) + `threshold_depth_m: 0.32`(패널~카 안쪽 벽)로 나눈다(2026-10-28. 그전 0.66을 패널 기준으로 써서 카가 0.33 m 깊게 등록돼 있었다) |
| 문 열림 유지 | 약 6~10초 |

## 2. 핵심 설계 결정

1. **하차 층은 명시적 도착 확인(운영자, 나중에 비전)으로만 확정하고, 미션 문맥(요청한 목표 층 + 문 닫힌 이동 시간)은 후보만 정한다.**
   층마다 복도 골격이 같아 지도 매칭으로 층을 구분할 수 없고, 잘못된 층에 내려도 위치 추정이 READY가 될 수 있다.
2. **엘리베이터 구간(탑승·카 안·하차)은 Nav2가 아닌 전용 저속 기동 제어기**가 맡는다. inflation 1.6 m·`robot_radius` 0.44 m로는
   1.1 m 출입구 경로가 나오지 않고, `navigation.yaml`을 바꾸면 실차 복도 주행 전체가 영향을 받으므로 **바꾸지 않는다.**
3. **카 안에서는 위치 추정을 일시 정지(SUSPENDED)** 한다. 하차 후 새 층 지도에서 `/initialpose`로 기존 수동 검증 경로를 거쳐 READY를 회복한다.
4. 문 상태는 LiDAR 전용 판별기가 맡는다(실차·시뮬레이션 공용).
5. 실차 공용 노드는 모두 기본 비활성으로 추가해 기존 실차 동작을 바꾸지 않는다.
6. 시뮬레이션 장면은 **층 수직 적층 + 카 실제 이동**(층별 장면 전환·순간이동 대신). 탑승 중 접촉, 문 타이밍, 카 안 LiDAR를 그대로 재현하고
   오도메트리·IMU가 연속이다. 측정 결과 3개 층 장면 RTF 1.0, kinematic 카 이동 시 AMR 추종 지연 2×10⁻⁵ m.

## 3. 구성

```text
            /elevator/request (사람이 누를 버튼) ◀── floor_mission_manager ◀── /floor_mission/navigate (웹 UI)
   웹·키오스크 안내 / 시뮬: elevator_sim_relay        │   ├─ Nav2 (/navigate_to_pose)          홀까지·호실까지
                                                     │   ├─ /cmd_vel_maneuver + 임대           탑승·하차
   웹·키오스크 "도착했나요?" / 시뮬 자동 확인 ── /floor/arrival_confirm ──▶ elevator_monitor ── /elevator/door, /floor/state, /floor/events
                                                     │   ├─ /localization/suspend             카 안 위치 추정 정지
                                                     │   └─ /map_server/load_map              층 지도 교체
```

| 노드 | 패키지 | 역할 | 실차 공용 |
|---|---|---|---|
| `elevator_monitor` | `amr_elevator` | LiDAR 문 상태, 현재 층, 카 안/밖 판별 | ✅ |
| `floor_mission_manager` | `amr_elevator` | 층간 미션 상태 머신 | ✅ |
| `navigation_startup` (확장) | `amr_stack` | `/localization/suspend` | ✅ |
| `navigation_manager` (확장) | `amr_stack` | `/cmd_vel_maneuver` + `/navigation/maneuver_lease` | ✅ |
| `elevator_sim_relay` | `amr_sim_bridge` | Isaac JSON 토픽 ↔ `/elevator/request`, 도착 자동 확인 | 시뮬레이션 전용 |

`experiment.sh sim world:=building`이면 monitor·mission·relay가 자동으로 실행된다. 실차(`experiment.sh real`)도 2026-10-23부터 monitor·mission을
기본으로 실행해 웹에 층 탭·호실 목록이 나온다(`elevator:=false`로 끔). `real_enabled: false` 층은 계속 거절된다.
엘리베이터 관련 CAN 송수신은 없다(0x210·0x310·0x311은 2026-11-02부터 쓰지 않는다).

### 토픽·서비스·액션

메시지 정의는 [`amr_elevator_interfaces`](../src/amr_system/amr_elevator_interfaces/)
(`ElevatorRequest`, `FloorArrivalConfirm`, `ElevatorDoorState`, `FloorState`, `FloorEvent`, `SetCurrentFloor`, `NavigateToFloorPose`).

| 이름 | 타입 | 제공 |
|---|---|---|
| `/elevator/request` (transient-local) | ElevatorRequest: `KIND_CALL`(출발 층·목표 층·방향), `KIND_CAR_FLOOR`(목표 층), `KIND_CANCEL`(미션 종료). 응답 없음. 가이드라인의 로봇출발층·목적층호출·목적층호출취소에 대응 | `floor_mission_manager` |
| `/floor/arrival_confirm` | FloorArrivalConfirm: `source`(OPERATOR·VISION·SIM), `floor`, `accepted`, `score` | 웹 UI(운영자) / `elevator_sim_relay` / 향후 층인식기 |
| `/elevator/door` | ElevatorDoorState | `elevator_monitor` |
| `/floor/state` (transient-local) | FloorState | `elevator_monitor` |
| `/floor/events` | FloorEvent | `elevator_monitor` |
| `/floor/set_current` | SetCurrentFloor | `elevator_monitor` (운영자 수동 지정) |
| `/floor_mission/navigate` | NavigateToFloorPose (action) | `floor_mission_manager` |
| `/floor_mission/status` (transient-local) | String(JSON) | `floor_mission_manager` |
| `/robot/announcement` (volatile, depth 10) | String(JSON): key, text, phase, floor, stamp | `floor_mission_manager` |
| `/localization/suspend` | std_srvs/SetBool | `navigation_startup` |
| `/navigation/maneuver_lease` | std_srvs/SetBool | `navigation_manager` |
| `/cmd_vel_maneuver` | Twist | 미션 → `navigation_manager` |
| `/sim/elevator/status`, `/sim/elevator/inject` | String(JSON) | 시뮬레이션 지상 진실, 고장 주입 |

- `/localization/suspend` true → `SUSPENDED`, `ready=false`. AMCL 결과·검증을 무시하고 Nav2 lifecycle은 건드리지 않는다.
  false → 지도 변경과 무관하게 `/initialpose`를 기다린다(수동 검증 1초·4샘플). 호출하지 않으면 기존 동작과 같다.
- `/cmd_vel_maneuver`는 임대 중(기본 30초, 갱신 가능)이고 AUTO일 때만 `/cmd_vel_auto`로 전달되며, 0.2초 끊기면 0을 낸다.
  임대 중에는 Nav2 목표를 받지 않고 LB 끼어들기(MANUAL) 시 즉시 해제된다. MANUAL에서는 임대 요청도 거절한다. 상태는 `MANEUVER`. control_mux·바퀴 제한·CAN 경로는 그대로다.
- `VehicleStatus`에는 층 필드를 넣지 않았다. 웹 UI는 `/floor/state`를 직접 구독한다.

## 4. 알고리즘

### 4.1 문 상태 — LiDAR (`door_detector.py`)

- 문 선분을 지나는 빔(문틀 가장자리 0.08 m 제외) 중 문 평면보다 0.3 m 이상 먼 빔의 비율이 `open_ratio`.
- ≥ 0.7이면 OPEN, ≤ 0.2이면 CLOSED, 그 사이는 OPENING/CLOSING. 연속 2스캔이 같아야 확정.
- 유효 빔 10개 미만이거나 문까지 4 m 넘게 떨어져 있으면 `UNKNOWN`(미션은 기다린다).

### 4.2 층·영역 판별 (`floor_state.py`, `monitor_node.py`)

- **영역** `OUTSIDE` ↔ `IN_ELEVATOR`: 문턱선 통과(위치), LiDAR 둘러싸임 비율, 문이 열려 있었는지를 함께 본다.
  문이 열리지 않았는데 문턱선을 넘으면 `UNKNOWN` + `RULE_VIOLATION`이 되어 미션이 정지한다.
- **카 안 위치**: AMCL 대신 카 진입 직전에 검증된 `map→odom`(anchor) + 오도메트리. 로봇 전체가 카 안에 있으면 카 내벽 직선 맞춤
  (`car_matcher.py`, Gauss-Newton)으로 보정한다. 층이 바뀌면 카 기준 상대 자세를 유지한 채 anchor를 새 층으로 옮긴다.
- **층 값은 카를 통해서만 바뀐다.**

  | 상황 | 층 값 | 신뢰도 |
  |---|---|---|
  | 운영자 층 지정(`/floor/set_current`) | 지정 층 | `CONFIRMED` |
  | 카 안 + 요청한 목표 층(`/elevator/request` KIND_CAR_FLOOR) + 문 1초 닫힘 | 이동 중 (`transit_from` → `transit_to`) | `TRANSIT` |
  | 이동 중 문 OPEN 1초(`arrival_settle_s`) 유지, 그전 문 닫힌 이동 ≥ 3초 | 목표 층 **후보**(새 `candidate_id`), `awaiting_confirm: true` | `ASSUMED` — **하차 금지** |
  | 위와 같으나 닫힌 시간 < 3초(문만 다시 열림) | 출발 층 후보, 확인 요청 없음 | `ASSUMED` |
  | 후보에 `accepted: true` | 목표 층 | `VERIFIED` |
  | 후보에 `accepted: false`(다른 승객 정차) | 이동 중으로 복귀. 문이 닫히고 다시 3초 이상 이동한 뒤에야 새 후보 | `TRANSIT` |
  | 확인 없이 문이 1초 닫힘 | 다시 이동 중 | `TRANSIT` |
  | `VERIFIED`인데 로봇이 카 안(문턱 안쪽·둘러싸임)에 있고 문이 1초 닫힘 | 다시 이동 중. 다음 열림에 새 후보·새 확인 필요(카가 안 움직였어도) | `TRANSIT` |
  | 하차 후 위치 추정 READY | 목표 층 | `CONFIRMED` |

- 확인은 카 안·`awaiting_confirm`·후보와 같은 `floor`와 `candidate_id`·**문이 지금 OPEN/OPENING**일 때만 받는다. 그 밖의 확인은 무시한다.
  `candidate_id`는 후보마다 1씩 늘어, 이전 후보 화면에서 누른 답이나 늦게 온 답이 같은 층 이름의 새 후보를 확정하지 못하게 한다.
- 미션은 SWITCH_MAP·ALIGHTING(출발 전)·RETURN_TO_CAR(문 열림 뒤)에서 "목표 층 + `VERIFIED`/`CONFIRMED`"를 다시 보고, 아니면 RIDING으로 돌아가 새 확인을 기다린다.
- `ASSUMED`인 채로 카를 나가면 `RULE_VIOLATION`. 판별이 불가능하면 `UNKNOWN`이 되고 운영자 `/floor/set_current`로만 복구한다.
- 시작 층은 운영자가 지정해야 `CONFIRMED`가 된다. 지정하지 않으면 `initial_floor`가 `ASSUMED`로 남고 층간 미션을 받지 않는다(같은 층 주행은 가능).

### 4.3 직선 기동 제어기 (`maneuver.py`)

- `ALIGN`(목표 방향 회전) → `DRIVE`(직진 + 방향 보정 + 감속) → `TURN`(최종 방향) → `DONE`.
- 최고 0.35 m/s, 회전 0.5 rad/s, 위치 허용 오차 0.05 m. 진행 방향 여유에 따라 `sqrt(2 × 0.3 × max(0, clearance − 0.15))`로 속도를 제한하며, 속도 한계가 최소 유효 속도 0.05 m/s 미만이 되는 여유(`0.15 + 0.05²/(2 × decel)`, 기본 약 0.15417 m)에서는 `blocked`로 판정하고 정지한다. `reverse`는 방향을 유지한 채 후진(문이 닫힐 때 후퇴).
- 진행 방향 여유는 `/scan`을 TF의 `base_footprint → laser`(실차 S2는 yaw π, 180° 돌려 장착)로 로봇 좌표계에 옮긴 뒤 계산한다.
  이 변환을 받기 전에는 여유를 0으로 보고 기동을 보류한다(5초마다 경고). 2026-10-24까지는 변환 없이 계산해 앞·뒤 여유가 뒤바뀌어 있었다(실차 미검증 수정, `test_maneuver.py`).
- DRIVE의 방향 보정 각속도는 `min(max_turn, max(0.1, speed / 0.2))` 이하로 제한해 저속에서 제자리 회전처럼 돌지 않게 한다.
- AUTO일 때만 명령을 낸다. MANUAL로 바뀌면 즉시 0을 내고 단계는 유지한다.

### 4.4 층간 미션 (`mission_node.py`)

```text
IDLE ─(같은 층)──────────────────────────────────────────────────────▶ NAV_TO_GOAL ─▶ SUCCEEDED
     └(다른 층)▶ NAV_TO_HALL ▶ CALL_ELEVATOR ▶ WAIT_DOOR_OPEN ▶ BOARDING ▶ IN_CAR_ALIGN
                ▶ SELECT_FLOOR ▶ RIDING ▶ SWITCH_MAP ▶ ALIGHTING ▶ RELOCALIZE ▶ NAV_TO_GOAL
     복구: BOARDING ─(문 닫힘·지속 막힘·시간 초과)▶ BACKING_OUT ▶ RELOCALIZE ▶ CALL_ELEVATOR
           ALIGHTING ─(문 닫힘·지속 막힘)▶ RETURN_TO_CAR ▶ ALIGHTING 또는 SELECT_FLOOR ▶ RIDING
     어느 단계든 실패 ▶ FAILED_SAFE (정지 유지, 오류 코드 표시)
```

**접수 조건**: 진행 중 미션 없음, `OUTSIDE`이고 현재 층이 `CONFIRMED`/`VERIFIED`, 위치 추정 READY. 실차에서는 목표 층이 `real_enabled: true`여야 한다.

| 단계 | 동작 | 다음 조건 | 이상 처리 (오류 코드) |
|---|---|---|---|
| NAV_TO_HALL | Nav2로 `hall_wait_pose` | Nav2 성공 | `NAV_TO_HALL_FAILED`, 위치 추정 45초 미복구 `LOCALIZATION_LOST` |
| CALL_ELEVATOR | `/elevator/request` KIND_CALL(위/아래) + `call`·`waiting` 안내. 문이 이미 열려 있으면 건너뜀 | 바로 다음 단계 | 없음(응답을 기다리지 않음) |
| WAIT_DOOR_OPEN | OPEN 후 위치 추정 정지·기동 임대, 승차 공간 확인(앞 범퍼 → 홀 → 문 터널 → 카 중앙 통로 + 회전 원. 내리는 사람이 로봇 진입 경로를 다 지나간 뒤 출발; 문틀·카 벽 제외) | 공간이 0.5초 연속 비어 있음 | 점유 시 정지·안내. 문이 연속 1.5초 동안 OPEN이 아니면 취소·RELOCALIZE → 재호출(3회 실패). 120초 → 재호출 1회 → 취소 안내·`DOOR_NOT_OPENED`; `PREPARE_FAILED` |
| BOARDING | 감속·직진 및 회전 충돌 검사로 카 중앙까지 | 카 안 판정 + 최종 회전 단계 | 전단이 홀 문턱 전이면 문 닫힘에도 BACKING_OUT으로 문에서 멀어짐. 연속 15초 막힘·30초 초과 시에는 OPEN일 때만 후퇴. 문이 안 열렸으면 정지하며 카 안이면 IN_CAR_ALIGN, 카 밖에서 30초 초과하면 `BOARDING_FAILED` |
| BACKING_OUT | 홀로 후진 → RELOCALIZE → CALL_ELEVATOR | 홀 복귀 완료 | 차체가 홀 문턱 전이면 non-OPEN에도 후퇴 가능. 문턱/카 안에서 문이 OPEN을 벗어나면 정지. 전단이 카 문을 넘었으면 IN_CAR_ALIGN, 아니면 30초 한도 후 `BACK_OUT_TIMEOUT`. 복귀 3회 → `BOARDING_FAILED` |
| IN_CAR_ALIGN | 카 중앙 정렬 + 문 쪽 180° 회전, 회전 영역 점검 | 기동 완료 | 막힘 시 정지·안내. 30초 막힘 + 출발 층 OPEN이면 BACKING_OUT, 아니면 60초 막힘 후 `ALIGN_BLOCKED`. 막힘 없는 연속 20초 초과는 `ALIGN_TIMEOUT`. 단계 전체가 `2 × turn_block_timeout + 20 s`(기본 80초)를 넘으면 출발 층 OPEN에서 후퇴, 그 외 `ALIGN_TIMEOUT` |
| SELECT_FLOOR | `/elevator/request` KIND_CAR_FLOOR(목표 층) | 바로 RIDING | 없음 |
| RIDING | 정지 유지, 문·층 감시 | 목표 층 + 문 OPEN + `VERIFIED`(도착 확인) | 아래 표 |
| SWITCH_MAP | 도착 안내 + `/map_server/load_map` (같은 미션에서 이미 로드했으면 재사용) | 로드 성공 | `MAP_SWITCH_FAILED` (카 안 정지) |
| ALIGHTING | 출구 공간 0.5초 확인 후 `hall_exit_pose`로 기동. 버튼 켰을 때 기존 열림 재무장 유지 | 기동 완료 + `OUTSIDE` | 출발 전 총 20초 공간 대기 초과는 취소·횟수 누적 후 주행 없이 RETURN_TO_CAR 도착 대기로 전환(카 중앙이므로 열림 버튼으로 문을 붙잡지 않음). 주행 중 문턱 전 문 닫힘·20초 연속 막힘·출발 후 30초 초과는 카로 복귀(30초는 출구 대기가 아닌 주행 시작부터 셈). 하차 취소 3회 초과 `ALIGHTING_FAILED`. 차체 전체가 홀 쪽으로 나간 경우에만 30초 초과 `ALIGHTING_TIMEOUT` |
| RETURN_TO_CAR | 카 중앙으로 후진, 도착 후 재개방 대기 | OPEN이면 공간 재확인 후 ALIGHTING | 복귀 주행 중에만 연속 20초 막힘 또는 30초 초과 시 `RETURN_BLOCKED`. 도착 후 대기는 이 한도에서 제외. 15초 미개방이면 SELECT_FLOOR에서 목표 층 요청을 다시 내고 RIDING으로 |
| RELOCALIZE | 위치 추정 재개 → 현재 자세로 `/initialpose`(분산 0.3 m; 감독기가 정지 후 ±1 m 보정, 일치율 50 % 기준 [위치 추정 §3](LOCALIZATION.md#3-초기-위치-검증과-빔-수-전환)) | READY | 검증이 안 되면 10초마다 같은 하차 자세를 다시 넣어 재검증(최대 5회, `relocalize_reseed_period`·`relocalize_reseeds`; 2026-11-02 5F에서 카 문이 42초 열려 있어 첫 검증 실패). 마지막 재검증 30초 뒤 TIM 1회 → `RELOCALIZE_FAILED` |
| NAV_TO_GOAL | Nav2로 호실 | 성공 | `NAV_TO_GOAL_FAILED` |

| RIDING 중 상황 | 처리 |
|---|---|
| 목표 층 후보로 문 열림(`ASSUMED`, 확인 대기) | 하차하지 않고 확인을 기다린다. "아니요"면 계속 탑승, 답 없이 문이 닫히면 다시 이동 중으로 본다. 단계 전체는 `ride_timeout`으로 제한 |
| 다른 층에서 문 열림 (다른 승객, "아니요" 뒤 포함) | 하차하지 않음. 확인 대기가 아닌데 문이 20초 넘게 계속 열려 있으면 KIND_CAR_FLOOR 요청을 다시 내고, 3회 초과 `RIDE_STALLED` |
| 180초 안에 도착 못 함 | `RIDE_TIMEOUT` |

**공통 안전 규칙**
- `/floor/state`가 3초 끊기면 `FLOOR_STATE_LOST`, 코드 예외는 `INTERNAL_ERROR`. 둘 다 즉시 정지.
- LB를 누르면(MANUAL) 기동 즉시 정지하고 미션을 `MANUAL_OVERRIDE`로 중단한다(재개 없음, MANUAL 중 새 층간 목표 거절).
  조이스틱 연결·끊김만으로는 미션이 중단되지 않는다.
  취소·중단 시 카 밖이면 위치 추정을 재개하고 카 안이면 정지 상태로 둔다.
- 실차(`require_estop`)에서 E-stop이 나거나 `/estop/status`가 끊기면 미션을 `ESTOP`으로 중단한다. 해제 뒤에도 이어 가지 않으며
  새 미션은 E-stop 인터록이 READY일 때만 받는다. 미션이 끝나면(어떤 결과든) `/elevator/request` KIND_CANCEL을 한 번 낸다.
- BOARDING·BACKING_OUT·ALIGHTING·RETURN_TO_CAR에서 미션이 끝나 차체가 문턱에 걸려 있으면 탈출은 운영자가 수동으로 한다.
- `FAILED_SAFE`에서는 정지를 유지하고 운영자 조치(MANUAL 수동 이동 또는 `/floor/set_current`)를 기다린다.
- 실제 엘리베이터의 문 끼임 방지 센서에 의존하지 않는다.

**문 열림 시간 예산**: 홀 대기 위치 → 문 평면 1.0 m + 문턱 0.32 m + 로봇 후단 0.32 m ≈ 1.64 m. 0.35~0.4 m/s 기동과 문 감지 지연을 더해
약 4.5~5초로, 최소 6초 안에 들어오지만 여유가 작다. 그래서 문턱 전 닫힘 시 후퇴 규칙을 둔다.
하차는 카 중앙에서 후단이 홀 문 평면을 지나기까지 약 2.1 m, 0.35 m/s로 약 6.5초(시뮬레이션 평균 8.1초)다. 문 열림 유지 6~10초보다
길 수 있어, 문턱 전에 닫히면 카로 돌아가 다시 열리기를 기다린다. 카 도착 뒤 15초 안에 열리지 않으면 SELECT_FLOOR에서 목표 층 요청을 다시 낸다.
문을 붙잡는 수단이 없으므로, 확인 후 하차가 늦으면 이 복귀가 잦아질 수 있다. 현장에서는 버튼을 누르는 사람이 "열림"을 눌러 주면 된다.

### 4.4.1 튜닝 파라미터와 실차 기록 (2026-11-02)

- 기동·문 판정 값은 [`amr_elevator/config/elevator_tuning.yaml`](../src/amr_system/amr_elevator/config/elevator_tuning.yaml) 한 파일에 모았다(기본값은 이전 하드코딩 값과 같다).
  `experiment.launch.py`가 이 파일을 먼저 넣고 노드별 고정값(`simulation`, `initial_floor` 등)을 위에 덮는다. 다른 파일은 `elevator_tuning:=<경로>`로 지정한다.
  값을 바꾼 뒤에는 스택만 다시 띄우면 된다(`--symlink-install`이라 빌드 불필요).
  시작 시 검사: 모두 양수·유한, `maneuver_speed ≤ 0.8`, `maneuver_max_turn ≤ 1.2`, `maneuver_min_turn ≤ maneuver_max_turn`, `maneuver_min_clearance ≥ 0.10`.
- 2026-11-02 첫 기록 주행(401→305→401, 기본값) 뒤 적용한 값: 안A(`maneuver_speed` 0.45, `accel` 0.5, `decel` 0.4, `max_turn` 0.6(0.7은 navigation_manager 상한 0.6을 넘어 회전 명령이 버려짐, 11-02 ALIGN_TIMEOUT), `turn_gain` 1.8),
  `arrival_settle_s` 0.5, `space_announce_delay` 1.0(문짝이 아직 열리는 동안 매번 나오던 "비켜주세요" 방지). 실차 미검증.
- `/floor_mission/debug`(String JSON, 미션 중에만 최대 10 Hz): 단계, 기동 단계, 명령 속도, 라이다·깊이 앞뒤 여유, 막힘, 공간 확인, scan·깊이 나이, 문, 자세,
  그리고 정지 중이면 이유 `hold`(`not_auto`, `estop`, `no_lease`, `prepare_pending`, `door_not_open`, `no_pose`, `no_laser_tf`, `scan_stale`,
  `space_occupied`, `clear_confirm`, `sweep_blocked`, `obstacle`, `waiting_confirm`, `other`).
- `elevator_recorder`(실차 `mode:=real`, `record_elevator:=true` 기본): 층간 미션이 시작되면 `bags/YYYY-MM-DD/HHMMSS_elevator_<출발>_<목표>/`에
  MCAP을 기록하고 끝나면 5초 뒤 멈춘다(최대 900초, 남은 디스크 5 GB 미만이면 기록 안 함, 기록 중 2.5 GB 미만이면 중단). `manifest.json`에 git 상태·튜닝 YAML·결과가 남는다.
  녹화기가 죽어도 bag 프로세스는 부모 종료 신호로 함께 멈춘다. 컬러 영상도 함께 기록되므로 층인식 학습 데이터로도 쓴다.
- 분석: `python3 scripts/analyze_elevator_run.py <bag 폴더> [--out report.md]` → 단계별 시간, 문 열림→출발→첫 움직임→카 진입 분해, 카 안 회전 시간,
  정지 이유별 시간, DRIVE 중 멈춤·재출발 횟수(머뭇거림), 속도 통계.

### 4.4.2 카메라 층인식 (`amr_floor_vision`, 2026-11-02, 2026-11-03 실차 시험 통과)

- `floor_vision` 노드(별도 Python `~/venvs/floorvision`, DINOv3 ViT-S, CPU 2스레드)가 카 안에서 이동 후 문이 OPEN되면 컬러 3장을 모아
  참조 사진(층별 폴더)과 코사인 1-NN 다수결로 층을 추정해 `/floor/vision_guess`를 한 번 낸다. 점수 0.85 미만이거나 2위 층과 차이 0.01 미만이면 "모름".
  실측: 문 OPEN → 추정 약 1.8초.
- `elevator_monitor`는 `vision_wait_s`(실차 2.0초) 동안 추정을 기다려, 알려진 층이면 그 층으로(`candidate_source` VISION), 모르면 목적층으로(TARGET) 묻는다.
  목적층이 아닌 층도 묻는다. 키오스크 "아니요" → 층 선택(1F~7F, 모르겠음) → `actual_floor`. 로봇은 **확정된 층이 목적층일 때만** 내린다.
  목적층이 아닌 층으로 확정되면 계속 타며, 문이 닫히면(차체 조건 없이) 다시 이동 중이 된다.
- 문이 열린 순간 사진과 모니터의 최종 판정(확정 층 / 모름)을 `datasets/floor_vision/`(`arrival/<층>/`, `labels.jsonl`)에 저장하고, 층이 확정된 사진은 즉시 참조에 추가한다.
  라벨은 처음 누른 답이 아니라 모니터가 받아들인 판정을 따른다. 참조 임베딩 캐시는 `datasets/floor_vision/gallery_cache.npz`(모델 ID가 바뀌면 재계산).
- 켜는 조건: 실차 + `start_camera:=true` + `floor_vision:=true`(기본) + 인터프리터 존재. 노드가 죽거나 늦으면 모니터는 기다림 한도 뒤 목적층 질문으로 돌아간다.
- **자동 판정**(`floor_vision_auto:=true` 기본, 2026-11-02 사용자 결정): 추정 점수 ≥ 0.93이고 2위와 차이 ≥ 0.03이면 질문 없이 확정한다.
  목적층이 아닌 층이면 계속 타고, 목적층이면 내린다. 목적층으로 자동 확정하려면 출발 층이 사람/운영자 확인 층(카메라 자동 확정이 아닌)이고
  출발~목적 사이 층과 목적층 모두 참조 사진이 있어야 한다(메시지 `reference_floors`). 아니면 화면 질문으로 돌아간다.
  **알려진 잔여 위험(사용자 수용):** 사람이 버튼을 누르므로 엘리베이터가 목적층을 지나치거나 반대 방향을 먼저 돌아 참조 없는 1·2·6·7층에 서면,
  그 층이 목적층과 닮았을 때 잘못 내릴 수 있다. 모든 층에 참조가 생기면 이 위험은 크게 준다. 자동 판정 사진은 저장만 하고 참조에는 넣지 않는다.
- 컬러 노출은 `camera_settings` 노드가 4167 µs·게인 16·WB 5000 K로 고정한다(자동 노출이 카 안 문에 맞춰져 홀이 과노출되던 문제).

### 4.5 승객·장애물 검사와 안내 (SPS-B-KOLA-0001-7700:2026)

4.1.3 a), 4.3.2 b)에 해당하는 로봇 측 승차 가능 판단을 수행한다. OPEN만으로 출발하지 않으며,
현재 지도 자세로 변환한 LiDAR·유효 깊이 점군에 승차 공간을 점유한 점이 하나라도 있으면 대기한다.
승차 공간은 홀 문 중심부터 카 중앙까지 폭 `2 × (0.307 + space_margin)`인 통로와
카 중앙의 반경 `hypot(0.348, 0.307) + space_margin` 원판의 합집합이다.
카 다각형을 `wall_margin`만큼 안쪽으로 줄여 벽·문틀을 제외한다.
하차 공간은 카 중앙부터 `hall_exit_pose`까지 같은 폭의 통로로, 홀 쪽도 포함하며 카 안쪽에서만 벽 여유를 제외한다.
복도 벽면(`hall_recess_m` 앞)부터 카 문까지의 문 통로에서는 반폭을 `min(0.307 + space_margin, door_width/2 − max(wall_margin, 0.15))`로 제한한다.
문 앞 홈의 양옆 벽이 이 통로에 들어가므로, 로봇이 대기 위치에서 옆으로 0.15 m 이상 비스듬히 서도 벽을 사람으로 보지 않는다(2026-10-28 이전에는 홀 문 평면부터만 제한해 "비켜주세요"가 끝나지 않았다).
기본 벽 여유 0.12 m만으로는 0.15 m 횡방향 자세 오차가 있는 문틀을 제외할 수 없어, 이 부분에만 최소 0.15 m를 적용한다.
승차 영역은 카 내부로 잘려 있어 문 통로 부분을 포함하지 않는다.

직진은 앞·뒤 LiDAR와 깊이 strip 여유 중 최솟값으로 감속한다(기존 기동 범퍼 기준 `HALF_LENGTH=0.32` 유지,
strip 반폭 0.307 m). 제자리 회전은 실제 차체 반치수 (0.328, 0.287)에 padding 0.02 m와
`turn_margin`을 더하고 반치수를 바깥쪽 cm 단위로 올림한 직사각형(기본 0.40 × 0.36 m)을
남은 회전각까지 2° 간격과 끝 각도에서 검사한다. 이 올림은 전방 0.40 m 점도 작은 회전부터 보수적으로 막기 위한 여유다.
자체 반사로 제외하는 영역은 실제 차체 반치수 (0.328, 0.287)에 각 0.01 m만 더한 영역이다.
확장된 안전 영역에 사람이 들어와도 제외하지 않는다. DRIVE에서 ALIGN/TURN으로 바뀔 때도 검사 전에는 회전하지 않는다.
후퇴 기동에도 동일한 직진·회전 검사를 적용한다.

`/scan`과 `/camera/depth/points_filtered`의 변환·직진 여유는 메시지 수신 시 캐시한다.
scan이 없거나 수신 후 `scan_max_age`(기본 0.5초, 단조 시계 기준)를 넘으면 직진 여유를 0으로 보고,
공간 확인을 초기화하며 제자리 회전도 보류한다. 최신 깊이 점군이 있어도 scan 누락을 대신하지 않는다.
깊이 카메라의 base←optical 정적 TF는 최초 수신 후 캐시하며 `/tf_static` 구독은 유지한다.
깊이 메시지의 역직렬화·TF 변환은 WAIT_DOOR_OPEN, BOARDING, BACKING_OUT, IN_CAR_ALIGN,
ALIGHTING, RETURN_TO_CAR에서만 수행하며 그 외 단계에서는 즉시 버린다.
상류 `amr_depth_filter`는 바닥·천장·원거리 점을 그대로 통과시키므로 이 노드가 직접 base 좌표의 유한 점만 남기고,
`depth_min_height ≤ z ≤ depth_max_height`(기본 0.06–1.0 m)와 수평 거리 `≤ depth_max_range`(3.0 m)를 적용한다.
필터링된 점만 직진 여유·회전·승하차 공간 검사에 사용한다. TF 미확보 원시 점군은 사용하지 않는다.
`use_depth=false`, 카메라 미기동, TF 미확보 또는 깊이 데이터가 0.5초보다 오래됐으면 LiDAR만 쓴다.
LiDAR TF 미확보 시 기동을 보류하고, 공간 확인에는 자세와 scan도 필요하다.
카메라 전방 약 0.55 m 바닥 사각·가림·센서 누락은 이 검사로 해소되지 않으며, 관측되지 않은 공간의 무장애를 보장하지 않는다.

시나리오 #2-1은 지속 막힘 시 정지·안내·취소 후 홀/카 복귀로, #2-2는 점유 해소 후 연속 공간 확인·진행으로 처리한다.
#2-4는 홀 대기 초과 시 취소 안내와 실패로 처리한다. 새 단계나 엘리베이터 신호를 추가하지 않는다.
문 상태가 잠깐 OPEN을 벗어나면 `door_cancel_grace` 동안 영속도로 대기한다. OPEN 회복 시 유예 타이머와
빈 공간 확인 시간을 다시 시작한다. 하차 출발 전 공간 대기는 점유가 깜박여도 총 `alight_block_timeout`으로 제한하며,
취소 시 이미 카 중앙이므로 후진 없이 도착 후 문 대기로 넘어간다.
홀에서 suspend 요청 후 실패하면 임대를 해제하고 위치 추정을 재개한 뒤 현재 자세를 `/initialpose`로 발행한다.
suspend(True) 승인이 실패 처리 뒤 늦게 도착해도 재개 요청과 자세 시드를 다시 보낸다.
카 안 실패에서는 위치 추정 정지를 유지한다. 실패 안내 발행 오류는 정지·실패 처리를 방해하지 않는다.
센서/위치 오차를 포함한 격리 ROS·시뮬레이션·실차 승객 시험은 별도 검증 대상이다.

| 새 파라미터 | 기본값 | 용도 |
|---|---|---|
| `use_depth` / `depth_topic` | `true` / `/camera/depth/points_filtered` | 선택 깊이 점군 |
| `depth_max_age` | 0.5 s | 깊이 수신 시각 기준 유효 기간 |
| `scan_max_age` | 0.5 s | scan 수신 시각 기준 유효 기간; 초과 시 공간 확인·직진·회전 보류 |
| `depth_min_height` / `depth_max_height` / `depth_max_range` | 0.06 / 1.0 / 3.0 m | base 좌표의 장애물 높이·수평 거리 한도 |
| `door_cancel_grace` | 1.5 s | 승차 전 연속 non-OPEN 취소 유예 |
| `space_margin` / `wall_margin` / `turn_margin` | 0.10 / 0.12 / 0.05 m | 공간·벽 제외·회전 추가 여유 |
| `clear_confirm_s` | 0.5 s | 연속 빈 공간 확인 |
| `announce_period` | 5.0 s | 같은 안내 키의 최소 발행 간격 |
| `board_block_timeout` / `alight_block_timeout` / `turn_block_timeout` | 15 / 20 / 30 s | 연속 막힘 허용 시간. `alight_block_timeout`은 하차 출발 전 총 공간 대기에도 적용 |
| `max_alighting_attempts` | 3 | 취소 누적 한도, 초과하면 실패 |

Annex C 표 C.1의 한국어 문구는 `announcements.py`에 있다. `/robot/announcement`에
`{"key", "text", "phase", "floor", "stamp"}` JSON을 String으로 발행한다(QoS depth 10, volatile).
`stamp`는 ROS 시각 초, `floor`는 현재 층이며 `alight_floor`에서는 목표 층 이름이다.

`announcement_player`(`experiment.launch.py mode:=real`에서 `start_announcer:=true` 기본)가 이 토픽을 받아 `tools/tts/wav/<키>.wav`를 `pw-play`로 재생한다(PipeWire 기본 출력 = HDMI 화면 스피커). `chime.wav`가 있으면 앞에 붙이고, `arrival`은 `arrival_<층>.wav`, `room_arrival`은 `room_<호실>.wav`를 쓴다. `alight_floor`는 표 C.1 각주 c에 따라 재생하지 않는다. 한 번에 하나씩 재생하며 `estop`·`robot_error`·`elevator_error`·`shock`은 재생 중인 안내를 끊고 대기열을 비운다. 5초(`max_age_s`)보다 오래 기다린 안내는 버린다. 바퀴가 움직이는 동안(`/odometry/filtered` 속도 > 0.03 m/s 또는 0.1 rad/s가 0.3초 지속)에는 `drive.wav`(5초 파일, 앞 0.7초만 소리)를 반복해 주행 알림을 낸다. 2초 정지하거나 오도메트리가 1초 끊기면 즉시 끄고, 안내가 들어오면 끊고 안내를 먼저 재생한다. 알림음과 음성은 `voice_volume`(기본 0.6, `pw-play --volume`)으로 줄여 재생하며 주행 알림은 원래 크기다.

같은 토픽에 웹 UI(`ros_bridge`)가 키오스크 배송(호실이 있는 목표)의 `delivery_start`(접수), `room_arrival`(`room` 포함, 도착), `delivery_cancel`, `received`(수령 확인 첫 1회), 같은 층 배송 실패의 `robot_error`(E-stop·LB 중단은 제외)를 낸다. 재생 노드는 스스로 두 가지를 더 낸다: `/navigation/status`가 `RUNNING`인데 3초(`nav_stall_s`) 정지해 있으면 `obstacle`, 이후 10초(`nav_stall_repeat_s`)마다 반복; `/estop/status`가 정상 상태에서 `ESTOP`으로 바뀌면 `estop`. 엘리베이터 미션의 `obstacle`은 1.5초(`obstacle_announce_delay`) 연속 막힌 뒤부터 낸다. 안전 안내(`estop` 등)는 일반 안내를 끊지만 다른 안전 안내는 끊지 않고 뒤에 이어 재생한다. wav는 PC에서 `tools/tts/generate_google_tts.py`로 만든다.

| 키 | 발행 시점 |
|---|---|
| `call`, `waiting` | CALL_ELEVATOR 진입마다(재호출 포함) |
| `board_start`, `board_space`, `board_cancel` | 승차 시작, 공간/회전 막힘, 승차 취소·대기 최종 실패 |
| `board_done`, `alight_floor` | 미션에서 최초 SELECT_FLOOR 진입 때만 발행. 하차 취소 후 층 재누름에서는 반복하지 않음 |
| `arrival`, `alight_start`, `alight_done` | SWITCH_MAP 진입, 출구 확인 후 하차 기동 시작, 하차 완료 |
| `obstacle`, `alight_cancel` | 승차·카 복귀 주행 막힘 및 하차 공간 점유/막힘, 하차 취소. ALIGHTING 진입이나 빈 공간 확인 대기만으로는 발행하지 않음 |
| `robot_error` | `fail()` 호출 |
| `elevator_error` | 표에 문구 제공; 대응하는 엘리베이터 오류 신호는 미연동 |

## 5. 층 설정 — `src/amr_system/amr_stack/config/floors.yaml`

지도를 바꿀 때는 이 파일만 고친다. 파서는 `amr_stack/floor_config.py` 하나이며 노드·스크립트·웹 UI·장면 생성기가 모두 이것을 쓴다.
로드 시 `level` 중복(통과층 포함)과 좌표 형식을 검사한다.
지도 파일 존재와 정차 지점 여유 공간은 `test_floor_config.py`가 확인한다.

| 키 | 내용 |
|---|---|
| `elevators.<id>` | 카 치수, 출입구 폭·위치, 문턱 깊이, 문 열림 시간 |
| `floors.<층>.level` | 층 순서(위/아래 호출 방향 판단) |
| `real_enabled` | 실차 층간 이동 허용. 3F·4F·5F `true`(3F는 2026-10-24 실차 시험 후, 5F는 2026-10-27 사용자 요청으로 실차 층간 이동 시험 전에 켬) |
| `provisional_copy_of` | 다른 층 지도를 복사한 임시본이면 원본 층. 현재 없음 (3F·5F는 2026-10-23 실측 지도로 교체) |
| `map`, `annotations`, `sim_map` | 실차 지도, 영역 주석 PNG, 시뮬레이션 지도 (`amr_stack` 패키지 기준 경로) |
| `sim_layer`, `spawn` | 시뮬레이션 층 USD, 시작 자세 |
| `elevators.<id>.car_interior_m`, `door_width_m`, `door_offset_from_left_m`, `hall_recess_m`, `threshold_depth_m` | 카 치수. `door_center`(홀 문 패널) 기준으로 복도 벽면은 `hall_recess_m` 앞, 카 안쪽 벽은 `threshold_depth_m` 뒤 |
| `elevators.<id>.door_center`, `car_inward_yaw`, `hall_wait_pose`, `hall_exit_pose` | 층별 엘리베이터 지점 (map 프레임) |
| `elevators.<id>.sill_ignore_height_m` | 승하차 공간 확인에서 카 문선 ±8 cm 띠(문 폭 안)의 이 높이 미만 **깊이 카메라** 점을 무시(LiDAR는 그대로). 홀이 카보다 낮은 층에서 문턱이 장애물로 찍히는 것 방지. 기본 0, 3F 0.10(2026-11-02 실측 6~8 cm 오검출로 탑승 3.4초 지연), 0~0.15 |
| `rooms` | 호실별 정차 자세 `goal`과 문 위치 `door`. 한 호실에 문이 둘이면 `"413-앞문"`처럼 나눈다 |
| `pass_floors.<층>.level` | 지도 없는 통과층(현재 1·2·6·7F). 목적지·웹 층 탭에는 나오지 않는다. 층인식기가 알아야 할 층 목록으로 남긴다. 실측 지도가 생기면 `floors`로 옮긴다 |

현재 엘리베이터 지점(3개 층 공통): 홀 문 중심 (9.975, 3.31), 호출·대기 (10.975, 3.31, π; 문에서 1.0 m, 2026-10-27 이전 0.6 m), 하차 종료 (10.775, 3.31, 0).
`sim/config/floor4_rooms.json`은 `sim_demo.py` 호환용이다.

## 6. 실차 연동 경로

엘리베이터 쪽 하드웨어 연동은 없다. 로봇이 내는 것은 음성 안내, 키오스크 표시, `/elevator/request`뿐이고, 받는 것은 LiDAR 문 상태와 `/floor/arrival_confirm`이다.
가이드라인(SPS-B-KOLA-0001-7700)의 엘리베이터 서버 연동이 생기면 `/elevator/request`를 서버로 보내고, 서버의 현재층·도어·목적층도착 신호를
같은 확인 통로나 별도 입력으로 받는 브리지를 붙인다. 2026-11-01까지 쓰던 RF 비콘·CAN 0x210/0x310/0x311 경로는 제거했다(이력: a4421e6).

## 7. 시뮬레이션

### 건물 장면 (`sim/building_G.usda`)

`sim/tools/build_building_scene.py`가 `floors.yaml`을 읽어 생성한다(`build_scene.sh`에 포함).

| 구성 | 값 / 방식 |
|---|---|
| 층 배치 | 3F z = −3.6 m, **4F z = 0**, 5F z = +3.6 m (`sim.floor_height_m`, 임시값) |
| 층 바닥 | 승강로 구멍을 뺀 **삼각형 메시** 슬래브 (Cube 충돌체는 제자리 회전 후 바퀴가 파고듦) |
| 승강로 | 모든 층을 관통하는 정적 벽 |
| 카 | kinematic rigid body, 내부 1.87 × 2.23 m, 출입구 1.10 m, 높이 2.3 m, 카 문 2패널 |
| 카 ↔ 홀 바닥 | 카 바닥판이 슬래브 아래로 2 cm 겹쳐 틈 없음 |
| 시작 위치 | 각 층 `spawn` (11.5, 3.31, 0) — 엘리베이터 홀 |

로봇·센서 모델은 [시뮬레이션](SIMULATION.md#4-제어센서-모델)과 같다.

### 엘리베이터·장치 모델

Isaac 내장 rclpy는 사용자 정의 메시지를 쓸 수 없어 Isaac은 JSON 토픽만 쓰고 `elevator_sim_relay`가 변환한다.

```text
[Isaac]  elevator_runtime.py ◀─ /sim/elevator/request_raw ── elevator_sim_relay ◀── /elevator/request
           └ ElevatorSim 모델    /sim/elevator/status (지상 진실) ─▶ 도착 자동 확인 ──▶ /floor/arrival_confirm (SIM)
           └ 카·문 USD 이동      /sim/elevator/inject (고장 주입)
```

| 동작 (`amr_sim_bridge/elevator_model.py`) | 값 |
|---|---|
| 카 이동 | 사다리꼴 속도, 1.0 m/s · 0.5 m/s² |
| 문 | 열림·닫힘 각 1.5초, 열림 유지 6~10초 난수(seed 고정). 문 평면에 물체가 있으면 다시 열림(미션은 의존하지 않음) |
| 버튼 | 사람이 누르는 것을 흉내 내 요청 `press_delay_s`(1.0초) 뒤 홀 호출·카 층 호출 등록 |
| 도착 확인 | `/floor/state`가 `awaiting_confirm`이면 `confirm_delay_s`(1.0초) 뒤 지상 진실(카가 그 층에 서서 문 열림)로 `accepted`를 정해 SIM 확인을 한 번 낸다. `auto_confirm:=false`면 웹에서 사람이 답한다 |
| 문 상태 | 알려주지 않음 (AMR이 LiDAR로 판별) |

고장 주입(`FAULT_DEFAULTS`): `stop_at_floor`(층 이름, 중간 층 정차), `hold_time_s`, `door_never_opens`, `button_fails`(아무도 버튼을 누르지 않음).

## 8. 웹 UI와 중앙 관제

- `GET /api/floors`(층 목록·현재 층·호실), `GET /api/floors/<층>/map.png`(다른 층 지도), `POST /api/goal`에 `floor`·`room`.
  현재 층과 같으면 기존 `/navigate_to_pose`, 다르면 `/floor_mission/navigate`. 다른 층 목표는 해당 층 지도의 `map_version`으로 검사한다.
- 화면: 층 탭(현재 층 표시), 호실 선택("3층 302호"), 층간 미션 단계 표시. `/floor/state`가 없으면(엘리베이터 노드 미실행) 단일 층 화면과 같다.
- 실차 모드에서는 `real_enabled: false` 층을 목적지로 고를 수 없다. 통과층은 층 탭에 나오지 않는다.
- **도착 확인**: `/floor/state.awaiting_confirm`이면 키오스크(관리자 PIN 없이)와 관리자 층간 이동 화면에 "엘리베이터가 N층에 도착했나요?" 카드가 뜬다.
  "네"/"아니요"는 `POST /api/floor/arrival_confirm {floor, candidate_id, accepted}` → `/floor/arrival_confirm`(OPERATOR). 후보가 없거나 층·후보 번호가 다르면 409.
- **버튼 안내**: 키오스크는 마지막 `/elevator/request`에 따라 "엘리베이터 ▲/▼ 버튼을 눌러 주세요", "엘리베이터 안에서 N층 버튼을 눌러 주세요"를 보여 준다
  (`/api/state`·`/api/kiosk/state`의 `elevator_request`).
- 중앙 관제 경유: `amr_remote_bridge`가 `/api/floors`와 층 지도(`floor/<층>` 레이어)를 함께 전달하고, 다른 층 목표는 서버와 AMR 양쪽에서
  그 층 지도의 버전·해시로 검증한다([amr_remote_bridge](../src/amr_system/amr_remote_bridge/README.md)).

## 9. 실차 영향

| 변경 | 실차 영향 | 기본값 |
|---|---|---|
| `navigation.yaml`, `can_telemetry` | `can_telemetry` 변경 없음. `navigation.yaml`은 2026-10-25 깊이 카메라 `depth_layer`를 전역·지역 costmap에 반영. inflation·robot_radius·footprint는 유지, 카메라가 없으면 깊이 레이어는 비어 있음 | — |
| `/localization/suspend`, 기동 임대 | 호출하지 않으면 기존과 동일 | 적용 |
| `elevator_monitor`, `floor_mission_manager` | monitor는 센서를 읽고 `/floor/*`만 발행. mission은 접수된 층간 목표에서만 움직임 | 실차·sim building `true`, 그 외 sim `false` |
| 웹 UI 층 선택 | 층 탭·호실 목록 표시. 실차 허용 층(현재 4F)만 선택 가능 | 적용 |

## 10. 시나리오

### 실행

```bash
export ROS_DOMAIN_ID=107                     # 107 이외 도메인에서는 실행 거부
./scripts/experiment.sh sim world:=building start_floor:=4F    # 수동 (웹 UI 층 탭 → 호실)
python3 scripts/run_floor_trial.py --output logs/elevator/try1 \
    --start-floor 4F --floor 3F --room 302 [--fault '{"stop_at_floor":"4F"}'] \
    [--fault-at 'RIDING={"faults":{"hold_time_s":20}}'] [--bag]
python3 scripts/run_floor_suite.py --output logs/elevator/<새 폴더> [--scenario 4F_to_3F ...]
```

`run_floor_trial.py`는 매번 Isaac과 스택을 새로 띄우고, 초기화 후 `/floor_mission/navigate` 목표를 보내고, 시작 시점(`--fault`)이나
특정 단계 진입 시(`--fault-at`) 고장을 주입한 뒤 `result.json`, `metrics.json`, `samples.json`을 저장한다.
`run_floor_suite.py`는 `sim/config/scenarios.json`의 `floor_scenarios`를 실행해 `summary.md`·`summary.json`을 만든다.

| 기대 | 통과 조건 |
|---|---|
| `success` | 목적지 도착 + 충돌 0 |
| `safe_stop` | 미션 실패 + 지정된 오류 코드 + 로봇 정지 + 충돌 0 |

수집 지표(위치 기준은 지상 진실): 도착 위치·방향 오차, 목표 층 높이 일치, 전체·단계별 소요 시간, 충돌·Nav2 복구·재시도 횟수, RTF, 층·영역 판별 일치율.

### 시나리오 결과 (2026-10-14)

원자료 `logs/elevator/final_20261014/` (시뮬레이션 PC). **11/11 통과.** RF 연동 시절 결과이며, ACK·비콘 시나리오는 2026-11-02에
`scenarios.json`에서 지웠다(`button_device_dead` → `nobody_presses_call`, 기대 `DOOR_NOT_OPENED`). 새 구성으로는 아직 다시 돌리지 않았다. 센서 주기를 실물에 맞추기 전(휠 10 Hz, LiDAR 8.8 Hz) 조건이다.

| 시나리오 | 경로 | 주입 고장 | 기대 | 결과 | 도착 오차 m | 소요 s |
|---|---|---|---|---|---|---|
| 4F_to_3F | 4F → 3F 302 | 없음 | success | 도착 | 0.49 | 129.1 |
| 4F_to_5F | 4F → 5F 502 | 없음 | success | 도착 | 0.41 | 129.6 |
| 3F_to_5F | 3F → 5F 503 | 없음 | success | 도착 | 0.11 | 119.9 |
| ack_lost_twice | 3F → 5F 505 | ACK 2회 연속 누락 | success | 도착 (버튼 재시도 2) | 0.17 | 98.5 |
| ack_loss_50 | 4F → 3F 305 | ACK 50 % 누락 | success | 도착 | 0.15 | 91.6 |
| ack_delay_4s | 4F → 5F 505 | ACK 4초 지연 | success | 도착 (재시도 1) | 0.15 | 94.3 |
| duplicate_reorder | 4F → 3F 305 | 중복·순서 뒤바뀜 각 30 % | success | 도착 (재시도 1) | 0.06 | 89.6 |
| other_passenger_stop | 3F → 5F 505 | 4F 중간 정차 | success | 4F에서 내리지 않고 5F 도착 | 0.15 | 113.3 |
| arrival_beacon_lost | 4F → 3F 305 | RIDING부터 비콘 100 % 누락 | safe_stop | `ARRIVAL_UNCONFIRMED`, 카 안 정지 | — | 64.0 |
| arrival_beacon_stale | 4F → 5F 505 | RIDING부터 수신 시각 3초 지연 | safe_stop | `ARRIVAL_UNCONFIRMED`, 카 안 정지 | — | 70.9 |
| button_device_dead | 4F → 3F 305 | 버튼 장치 고장 | safe_stop | `NO_HALL_ACK`, 홀 정지 | — | 41.6 |

성공 8건: 도착 오차 평균 0.21 m(최대 0.49 m), 소요 평균 108.2초, 엘리베이터 구간(호출~재위치) 평균 42.2초
(탑승 7.9 · 카 안 정렬 7.8 · 하차 8.1 · 재위치 4.0초), 충돌 0, Nav2 복구 0, RTF 최소 0.9999.
층·영역 판별 일치율 95.0~100 %(불일치는 탑승·하차 순간의 판정 지연, 최장 3.0초), 최종 판별은 모두 일치.
회귀: arena 0.064 m, floor4 405호 0.14 m, 402호 0.167·0.143 m, 모두 충돌 0.

## 11. 가정과 남은 일

| 항목 | 내용 |
|---|---|
| 도착 확인 | 지금은 사람이 답한다. RGB 층인식기(층마다 인테리어가 다름)를 `/floor/arrival_confirm` VISION으로 붙일 예정. 문이 열린 순간 영상과 확인 결과를 학습 데이터로 모으는 기록기는 아직 없다 |
| 범위 | 엘리베이터 1대. 승하차 공간 점유와 회전 장애물을 검사하며 다른 승객 호출에 의한 중간 정차도 처리 |
| 문 | 카 문과 홀 문이 함께 열리고 닫힌다 — 가정 |
| 기동 | 전진 진입 → 카 안 180° 회전 → 전진 하차 (회전 지름 약 0.84 m < 카 폭 1.87 m) |
| 지도 | 3F·5F 실차 지도는 실측(2026-10-06 SLAM)을 4F 좌표계로 정렬해 넣었다(`scripts/align_floor_map.py`, 엘리베이터 문 위치 차 3F 6.5 cm·5F 4.5 cm). 층 전환이 자세 값을 그대로 쓰므로 모든 층 지도는 같은 좌표계여야 한다. 시뮬레이션 3F·5F 장면·영역지정과 호실 좌표는 아직 4F 복사본 (`maps/G_3_floor/MAP_SOURCE.md`) |
| 실차 | 3F `real_enabled: true`(2026-10-24), 2026-10-26 3F→4F 전 구간 실차 통과(RF 시절). 5F `true`(2026-10-27). RF 제거 뒤 2026-11-03 실차 시험 통과 |
| 통과층 | 1·2·6·7F는 지도 없이 설정에만 있다. 시뮬레이션 장면에는 없다 |
| 수치 | 층고 3.6 m, 카 속도 1.0 m/s, 센서 노이즈 0은 임시값 |
| 긴 복도 끝 호실 | 302·502(x ≈ 33.7 m)는 하차 후 주행 중 위치 오차 0.53~0.76 m로 단일 층 시험(0.19~0.27 m)보다 컸다. 하차 직후 재위치 오차는 약 2 cm라 4층 복도 위치 추정 문제로 보인다. 반복 측정 필요 |
| 시작 경합 | 단일 층 월드에서도 Isaac·Nav2 시작 단계에서 간헐적 타임아웃. 재실행하면 통과 |

## 12. 관련 파일

| 분류 | 경로 |
|---|---|
| 층 설정 | `src/amr_system/amr_stack/config/floors.yaml`, `amr_stack/floor_config.py` |
| 공용 알고리즘 | `src/amr_system/amr_elevator/amr_elevator/` — `mission_node.py`, `monitor_node.py`, `floor_state.py`, `door_detector.py`, `car_matcher.py`, `maneuver.py` |
| 메시지 | `src/amr_system/amr_elevator_interfaces/` |
| 시뮬레이션 모델 | `src/amr_system/amr_sim_bridge/amr_sim_bridge/elevator_model.py`, `elevator_node.py` |
| Isaac 런타임·장면 | `sim/scripts/elevator_runtime.py`, `sim/scripts/run_ros_sim.py`, `sim/tools/build_building_scene.py`, `sim/config/building.json` |
| 런치 | `src/amr_system/amr_stack/launch/experiment.launch.py`, `bringup.launch.py` |
| 시험 | `scripts/run_floor_trial.py`, `scripts/run_floor_suite.py`, `sim/config/scenarios.json` |
