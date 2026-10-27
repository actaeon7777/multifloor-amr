# 층간 이동 (엘리베이터)

> 상태: **시뮬레이션 구현·검증 완료 (2026-10-14), 실차 미적용**. 범위는 G동 3F·4F·5F(지도 있음), 엘리베이터 1대(E1).
> 비콘은 1–7F 홀과 카 E1에 설치된다. 지도가 없는 1·2·6·7F는 층 판단에만 쓰는 통과층이다(§5).
> 하위제어기 RF 중계는 프로토콜 v1.1로 구현됐고 Jetson 반영은 2026-10-21에 했다. 실차 통합 시험은 아직이다(§6).
> 공용 로직은 실차 인터페이스 기준으로 작성했고, 실차 전용 노드는 기본으로 꺼져 있다.
> 시뮬레이션 수치는 실물 보정 전 모델의 결과다. 구현 단계별 기록은 [실험 기록](EXPERIMENTS.md#2026-10-1314--층간-이동엘리베이터-구현).

## 1. 요구 사항 (2026-10-13 확정)

| 항목 | 결정 |
|---|---|
| 문 열림·닫힘 | AMR이 **LiDAR로 직접 판별**. 장치는 문 상태를 알리지 않는다 |
| 장치(비콘)가 알리는 것 | ① "나는 N층 비콘"(층 식별, 홀 비콘 **1초 주기**. 카 비콘도 1초마다 알리지만 층 없이 생존 확인용) ② "버튼을 눌렀다"(확인 응답) **만** |
| 안/밖 판별 | 신호 + 위치 추정을 함께 사용 |
| 버튼 호출 | AMR이 엘리베이터 앞에서 RF로 장치에 버튼 누름 명령 송신 |
| 통신 경로 | 장치 ↔ **RF** ↔ STM32(RF 모듈) ↔ **CAN** ↔ Jetson. Jetson은 비콘을 직접 제어하지 않는다 |
| 카 내부 "열림" 버튼 | **2026-10-26부터 누르지 않는다**(`use_door_open_button` 기본 `false`). 켜면 카 비콘이 누른다. 한 명령으로 약 1초 간격 최대 15회 반복(`repeat`, RF 1.1). 버튼 없이도 동작해야 한다 |
| 치수 | 카 내부 1.87 × 2.23 m, 출입구 1.10 m(카 왼쪽 벽에서 0.385 m), 층 벽 ~ 카 안쪽 벽 **0.66 m** ([`images/Elevator_size.jpg`](images/Elevator_size.jpg)) |
| 문 열림 유지 | 약 6~10초 |

## 2. 핵심 설계 결정

1. **층 판별의 1차 근거는 비콘, 2차는 미션 문맥.** 층마다 복도 골격이 같아 지도 매칭으로 층을 구분할 수 없다.
2. **엘리베이터 구간(탑승·카 안·하차)은 Nav2가 아닌 전용 저속 기동 제어기**가 맡는다. inflation 1.6 m·`robot_radius` 0.44 m로는
   1.1 m 출입구 경로가 나오지 않고, `navigation.yaml`을 바꾸면 실차 복도 주행 전체가 영향을 받으므로 **바꾸지 않는다.**
3. **카 안에서는 위치 추정을 일시 정지(SUSPENDED)** 한다. 하차 후 새 층 지도에서 `/initialpose`로 기존 수동 검증 경로를 거쳐 READY를 회복한다.
4. 문 상태는 LiDAR 전용 판별기가 맡는다(실차·시뮬레이션 공용).
5. 실차 공용 노드는 모두 기본 비활성으로 추가해 기존 실차 동작을 바꾸지 않는다.
6. 시뮬레이션 장면은 **층 수직 적층 + 카 실제 이동**(층별 장면 전환·순간이동 대신). 탑승 중 접촉, 문 타이밍, 카 안 LiDAR를 그대로 재현하고
   오도메트리·IMU가 연속이다. 측정 결과 3개 층 장면 RTF 1.0, kinematic 카 이동 시 AMR 추종 지연 2×10⁻⁵ m.

## 3. 구성

```text
                   ┌──────────── /elevator/command ◀── floor_mission_manager ◀── /floor_mission/navigate (웹 UI)
실차: elevator_can_link (CAN 0x210/0x310/0x311)          │   ├─ Nav2 (/navigate_to_pose)          홀까지·호실까지
시뮬: elevator_sim_relay ⇄ Isaac elevator_runtime        │   ├─ /cmd_vel_maneuver + 임대           탑승·하차
                   └──────────── /elevator/beacon ──▶ elevator_monitor ── /elevator/door, /floor/state, /floor/events
                                                         │   ├─ /localization/suspend             카 안 위치 추정 정지
                                                         │   └─ /map_server/load_map              층 지도 교체
```

| 노드 | 패키지 | 역할 | 실차 공용 |
|---|---|---|---|
| `elevator_monitor` | `amr_elevator` | LiDAR 문 상태, 현재 층, 카 안/밖 판별 | ✅ |
| `floor_mission_manager` | `amr_elevator` | 층간 미션 상태 머신 | ✅ |
| `navigation_startup` (확장) | `amr_stack` | `/localization/suspend` | ✅ |
| `navigation_manager` (확장) | `amr_stack` | `/cmd_vel_maneuver` + `/navigation/maneuver_lease` | ✅ |
| `elevator_can_link` | `amr_can_bridge` | `/elevator/*` ↔ CAN | 실차 전용, 기본 꺼짐 |
| `elevator_sim_relay` | `amr_sim_bridge` | Isaac JSON 토픽 ↔ `/elevator/*` | 시뮬레이션 전용 |

`experiment.sh sim world:=building`이면 monitor·mission·relay가 자동으로 실행된다. 실차(`experiment.sh real`)도 2026-10-23부터 monitor·mission을
기본으로 실행해 웹에 층 탭·호실 목록이 나온다(`elevator:=false`로 끔). `real_enabled: false` 층은 계속 거절된다. `elevator_can_link`는 기본 꺼짐이고
`start_elevator_link:=true`로 켠다. 이때도 `elevator_link_dry_run`이 기본 `true`라 CAN을 열지 않는다(0x210 미리보기만, 0x310/0x311 비콘 수신도 없음).
비콘을 받으려면 `false`로 둔다. 그러면 미션이 보내는 버튼 명령도 실제로 송신된다.

### 토픽·서비스·액션

메시지 정의는 [`amr_elevator_interfaces`](../src/amr_system/amr_elevator_interfaces/)
(`ElevatorBeacon`, `ElevatorCommand`, `ElevatorCommandStatus`, `ElevatorDoorState`, `FloorState`, `FloorEvent`, `SetCurrentFloor`, `NavigateToFloorPose`).

| 이름 | 타입 | 제공 |
|---|---|---|
| `/elevator/beacon` | ElevatorBeacon | `elevator_can_link` / `elevator_sim_relay` |
| `/elevator/command` | ElevatorCommand (`repeat`: 열림 반복 횟수) | `floor_mission_manager` |
| `/elevator/command_status` | ElevatorCommandStatus (0x311 명령 처리 단계) | `elevator_can_link` / 시뮬레이션 |
| `/elevator/rf_status` (transient-local) | String(JSON): 링크·비콘별 배터리·플래그·마지막 명령, 1 Hz | `elevator_can_link` |
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
  | ACK 받은 목표 층 + 문 닫힘 | 이동 중 (`transit_from` → `transit_to`) | `TRANSIT` |
  | 문 OPEN + 강한 홀 비콘(≥ −70 dBm, CAN은 RPD 등급 −60만 해당) | 목표 층 | `BEACON` |
  | 이동 중 문 OPEN + 통과층(1·2·6·7F) 비콘 | 이동 중 유지 (다른 승객 정차, 계속 탑승) | `TRANSIT` |
  | 하차 후 위치 추정 READY | 목표 층 | `CONFIRMED` |
  | 비콘 없이 문만 열림 | 목표 층(추정) | `ASSUMED` — **하차 금지** |

- **비콘 필터**(장치별): 2초 넘은 신호 폐기, 5초 안 같은 시퀀스는 중복 폐기, 8비트 시퀀스 창(128) 기준 과거 번호 폐기, 카 이동 중 비콘 무시,
  밖에서 다른 층 비콘은 누설로 보고 `floor_conflict` 경고만 낸다. 다른 층 정차 시 목표를 유지한다.
  카 비콘 PRESENCE는 층 판단에 쓰지 않는다. `floors.yaml`에 없는 층 코드는 `UNKNOWN_BEACON`을 코드마다 한 번만 낸다.
  판별이 불가능하면 `UNKNOWN`이 되고 운영자 `/floor/set_current`로만 복구한다.

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

**접수 조건**: 진행 중 미션 없음, `OUTSIDE`이고 현재 층이 `BEACON`/`CONFIRMED`, 위치 추정 READY. 실차에서는 목표 층이 `real_enabled: true`여야 한다.

| 단계 | 동작 | 다음 조건 | 이상 처리 (오류 코드) |
|---|---|---|---|
| NAV_TO_HALL | Nav2로 `hall_wait_pose` | Nav2 성공 | `NAV_TO_HALL_FAILED`, 위치 추정 45초 미복구 `LOCALIZATION_LOST` |
| CALL_ELEVATOR | `HALL_CALL(위/아래)`. 문이 이미 열려 있으면 건너뜀 | 홀 장치 ACK | 3초마다 재송신, 3회 → `NO_HALL_ACK`. 0x311 `DEVICE_REJECTED`·`NO_ROUTE`·`BAD_FRAME` → 즉시 `HALL_REJECTED` |
| WAIT_DOOR_OPEN | OPEN 후 위치 추정 정지·기동 임대, 승차 공간 확인(앞 범퍼 → 홀 → 문 터널 → 카 중앙 통로 + 회전 원. 내리는 사람이 로봇 진입 경로를 다 지나간 뒤 출발; 문틀·카 벽 제외) | 공간이 0.5초 연속 비어 있음 | 점유 시 정지·안내. 문이 연속 1.5초 동안 OPEN이 아니면 취소·RELOCALIZE → 재호출(3회 실패). 120초 → 재호출 1회 → 취소 안내·`DOOR_NOT_OPENED`; `PREPARE_FAILED` |
| BOARDING | 감속·직진 및 회전 충돌 검사로 카 중앙까지 | 카 안 판정 + 최종 회전 단계 | 전단이 홀 문턱 전이면 문 닫힘에도 BACKING_OUT으로 문에서 멀어짐. 연속 15초 막힘·30초 초과 시에는 OPEN일 때만 후퇴. 문이 안 열렸으면 정지하며 카 안이면 IN_CAR_ALIGN, 카 밖에서 30초 초과하면 `BOARDING_FAILED` |
| BACKING_OUT | 열림 유지 요청 후 홀로 후진 → RELOCALIZE → CALL_ELEVATOR | 홀 복귀 완료 | 차체가 홀 문턱 전이면 non-OPEN에도 후퇴 가능. 문턱/카 안에서 문이 OPEN을 벗어나면 정지. 전단이 카 문을 넘었으면 IN_CAR_ALIGN, 아니면 30초 한도 후 `BACK_OUT_TIMEOUT`. 복귀 3회 → `BOARDING_FAILED` |
| IN_CAR_ALIGN | 카 중앙 정렬 + 문 쪽 180° 회전, 회전 영역 점검 | 기동 완료 | 막힘 시 정지·안내. 30초 막힘 + 출발 층 OPEN이면 BACKING_OUT, 아니면 60초 막힘 후 `ALIGN_BLOCKED`. 막힘 없는 연속 20초 초과는 `ALIGN_TIMEOUT`. 단계 전체가 `2 × turn_block_timeout + 20 s`(기본 80초)를 넘으면 출발 층 OPEN에서 후퇴, 그 외 `ALIGN_TIMEOUT` |
| SELECT_FLOOR | `CAR_FLOOR(목표 층)` | 카 장치 ACK | 3초 × 3회 → `NO_CAR_ACK`. 거절 → 즉시 `CAR_REJECTED` |
| RIDING | 정지 유지, 문·층 감시. 버튼 켰을 때 도착하면 "열림" repeat 10을 **한 번** 보냄 | 목표 층 + 문 OPEN + `BEACON`/`CONFIRMED` | 아래 표 |
| SWITCH_MAP | 도착 안내 + `/map_server/load_map` (같은 미션에서 이미 로드했으면 재사용) | 로드 성공 | `MAP_SWITCH_FAILED` (카 안 정지) |
| ALIGHTING | 출구 공간 0.5초 확인 후 `hall_exit_pose`로 기동. 버튼 켰을 때 기존 열림 재무장 유지 | 기동 완료 + `OUTSIDE` | 출발 전 총 20초 공간 대기 초과는 취소·횟수 누적 후 주행 없이 RETURN_TO_CAR 도착 대기로 전환(카 중앙이므로 열림 버튼으로 문을 붙잡지 않음). 주행 중 문턱 전 문 닫힘·20초 연속 막힘·출발 후 30초 초과는 카로 복귀(30초는 출구 대기가 아닌 주행 시작부터 셈). 하차 취소 3회 초과 `ALIGHTING_FAILED`. 차체 전체가 홀 쪽으로 나간 경우에만 30초 초과 `ALIGHTING_TIMEOUT` |
| RETURN_TO_CAR | 카 중앙으로 후진, 도착 후 재개방 대기 | OPEN이면 공간 재확인 후 ALIGHTING | 복귀 주행 중에만 연속 20초 막힘 또는 30초 초과 시 `RETURN_BLOCKED`. 도착 후 대기는 이 한도에서 제외. 15초 미개방이면 SELECT_FLOOR에서 목표 층 재누름·ACK 재시도 → RIDING → SWITCH_MAP. 열림 버튼을 켰으면 15초마다 열림 재누름 먼저 시도(최대 2회) |
| RELOCALIZE | 위치 추정 재개 → 현재 자세로 `/initialpose`(분산 0.3 m; 감독기가 정지 후 ±1 m 보정, 일치율 50 % 기준 [위치 추정 §3](LOCALIZATION.md#3-초기-위치-검증과-빔-수-전환)) | READY | 30초 → TIM 1회 → `RELOCALIZE_FAILED` |
| NAV_TO_GOAL | Nav2로 호실 | 성공 | `NAV_TO_GOAL_FAILED` |

| RIDING 중 상황 | 처리 |
|---|---|
| 목표 층에서 문 열림, 비콘 없음(`ASSUMED`) | 하차 금지(버튼 켰을 때 "열림" repeat 10 1회), 6초 안에 확인 안 되면 `ARRIVAL_UNCONFIRMED`(카 안 정지) |
| 다른 층에서 문 열림 (다른 승객) | 하차하지 않음. 카가 20초 넘게 멈추면 목표 층 버튼 재송신, 3회 초과 `RIDE_STALLED`. 통과층 정차도 같다 |
| 180초 안에 도착 못 함 | `RIDE_TIMEOUT` |

**공통 안전 규칙**
- `/floor/state`가 3초 끊기면 `FLOOR_STATE_LOST`, 코드 예외는 `INTERNAL_ERROR`. 둘 다 즉시 정지.
- LB를 누르면(MANUAL) 기동 즉시 정지하고 미션을 `MANUAL_OVERRIDE`로 중단한다(재개 없음, MANUAL 중 새 층간 목표 거절).
  조이스틱 연결·끊김만으로는 미션이 중단되지 않는다.
  취소·중단 시 카 밖이면 위치 추정을 재개하고 카 안이면 정지 상태로 둔다.
- 실차(`require_estop`)에서 E-stop이 나거나 `/estop/status`가 끊기면 미션을 `ESTOP`으로 중단한다. 해제 뒤에도 이어 가지 않으며
  새 미션은 E-stop 인터록이 READY일 때만 받는다. RF 중계는 E-stop과 독립이라 래치 중에도 버튼 명령이 전달된다.
- BOARDING·BACKING_OUT·ALIGHTING·RETURN_TO_CAR에서 미션이 끝나면(E-stop·LB·취소·실패) 차체가 문턱에 걸려 있으면
  버튼을 켰을 때만 "열림" repeat 15(약 15초)를 한 번 보낸다. 그 뒤의 탈출은 운영자가 수동으로 한다.
- "열림" 버튼은 2026-10-26부터 기본으로 누르지 않는다(`use_door_open_button: false`). 이때 `DOOR_OPEN`은 한 번도 송신되지 않고, 문 열림 유지는 엘리베이터 자체 시간(6~10초)과 후퇴 규칙에만 기댄다.
- 버튼을 켰을 때 "열림" 버튼 명령의 BUTTON_ACK는 "반복을 시작했다"는 뜻이다. 문 상태는 계속 LiDAR로 판단한다. ACK가 없으면 3초마다 새 번호로 최대 3회
  다시 보내고, 끝내 없으면 `door_open_unacked`만 세고 미션은 계속한다(버튼 없이도 후퇴 규칙으로 동작).
- `FAILED_SAFE`에서는 정지를 유지하고 운영자 조치(MANUAL 수동 이동 또는 `/floor/set_current`)를 기다린다.
- 실제 엘리베이터의 문 끼임 방지 센서에 의존하지 않는다.

**문 열림 시간 예산**: 홀 대기 위치 → 문 평면 0.6 m + 문턱 0.66 m + 로봇 후단 0.32 m ≈ 1.58 m. 0.35~0.4 m/s 기동과 문 감지 지연을 더해
약 4.5~5초로, 최소 6초 안에 들어오지만 여유가 작다. 그래서 문턱 전 닫힘 시 후퇴 규칙을 둔다("열림" 버튼은 기본 꺼짐).
하차는 카 중앙에서 후단이 홀 문 평면을 지나기까지 약 2.1 m, 0.35 m/s로 약 6.5초(시뮬레이션 평균 8.1초)다. 열림 repeat 10은
첫 누름(명령 후 0.3~1.5초)부터 약 9초 동안 누르고 마지막 누름 뒤 6~10초 문이 열려 있으므로, 명령 뒤 약 14초 안에 끝나는 하차를 덮는다.
그보다 느리면 ALIGHTING의 재무장이 연장한다. 버튼을 끄면(기본) 하차 약 6.5초가 문 열림 유지 6~10초보다 길 수 있어,
문턱 전에 닫히면 카로 돌아가 다시 열리기를 기다린다. 카 도착 뒤 15초 안에 열리지 않으면 SELECT_FLOOR에서 목표 층 버튼을 다시 누른다. ACK가 없으면 기존 재시도 후 `NO_CAR_ACK`로 실패하며, ACK를 받은 뒤 RIDING으로 간다.

| 파라미터 (`floor_mission_manager`) | 기본값 |
|---|---|
| `use_door_open_button` | `false` (2026-10-26. 켜야 아래 반복 값이 쓰인다) |
| `door_open_repeat_board` / `door_open_repeat_exit` / `door_open_repeat_hold` | 6 / 10 / 15 |
| `door_open_rearm_s` / `door_open_rearm_max` | 8.0 s / 2 |

### 4.5 승객·장애물 검사와 안내 (SPS-B-KOLA-0001-7700:2026)

4.1.3 a), 4.3.2 b)에 해당하는 로봇 측 승차 가능 판단을 수행한다. OPEN만으로 출발하지 않으며,
현재 지도 자세로 변환한 LiDAR·유효 깊이 점군에 승차 공간을 점유한 점이 하나라도 있으면 대기한다.
승차 공간은 홀 문 중심부터 카 중앙까지 폭 `2 × (0.307 + space_margin)`인 통로와
카 중앙의 반경 `hypot(0.348, 0.307) + space_margin` 원판의 합집합이다.
카 다각형을 `wall_margin`만큼 안쪽으로 줄여 벽·문틀을 제외한다.
하차 공간은 카 중앙부터 `hall_exit_pose`까지 같은 폭의 통로로, 홀 쪽도 포함하며 카 안쪽에서만 벽 여유를 제외한다.
홀 문과 카 문 사이의 문 통로에서는 반폭을 `min(0.307 + space_margin, door_width/2 − max(wall_margin, 0.15))`로 제한한다.
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
`stamp`는 ROS 시각 초, `floor`는 현재 층이며 `alight_floor`에서는 목표 층 이름이다. 실제 스피커 출력은 아직 없다.

| 키 | 발행 시점 |
|---|---|
| `call`, `waiting` | 홀 호출 ACK 확인 |
| `board_start`, `board_space`, `board_cancel` | 승차 시작, 공간/회전 막힘, 승차 취소·대기 최종 실패 |
| `board_done`, `alight_floor` | 미션에서 최초 SELECT_FLOOR 진입 때만 발행. 하차 취소 후 층 재누름에서는 반복하지 않음 |
| `arrival`, `alight_start`, `alight_done` | SWITCH_MAP 진입, 출구 확인 후 하차 기동 시작, 하차 완료 |
| `obstacle`, `alight_cancel` | 승차·카 복귀 주행 막힘 및 하차 공간 점유/막힘, 하차 취소. ALIGHTING 진입이나 빈 공간 확인 대기만으로는 발행하지 않음 |
| `robot_error` | `fail()` 호출 |
| `elevator_error` | 표에 문구 제공; 대응하는 엘리베이터 오류 신호는 미연동 |

## 5. 층 설정 — `src/amr_system/amr_stack/config/floors.yaml`

지도를 바꿀 때는 이 파일만 고친다. 파서는 `amr_stack/floor_config.py` 하나이며 노드·스크립트·웹 UI·장면 생성기가 모두 이것을 쓴다.
로드 시 층 코드·`level` 중복(통과층 포함), 층 코드 범위(−8…20, 홀 비콘 RF 주소 `0x40 + 층`), 좌표 형식을 검사한다.
지도 파일 존재와 정차 지점 여유 공간은 `test_floor_config.py`가 확인한다.

| 키 | 내용 |
|---|---|
| `elevators.<id>` | 카 치수, 출입구 폭·위치, 문턱 깊이, 문 열림 시간 |
| `floors.<층>.level`, `beacon_floor_code` | 층 순서, 비콘 층 코드 |
| `real_enabled` | 실차 층간 이동 허용. 3F·4F·5F `true`(3F는 2026-10-24 실차 시험 후, 5F는 2026-10-27 사용자 요청으로 실차 층간 이동 시험 전에 켬) |
| `provisional_copy_of` | 다른 층 지도를 복사한 임시본이면 원본 층. 현재 없음 (3F·5F는 2026-10-23 실측 지도로 교체) |
| `map`, `annotations`, `sim_map` | 실차 지도, 영역 주석 PNG, 시뮬레이션 지도 (`amr_stack` 패키지 기준 경로) |
| `sim_layer`, `spawn` | 시뮬레이션 층 USD, 시작 자세 |
| `elevators.<id>.door_center`, `car_inward_yaw`, `hall_wait_pose`, `hall_exit_pose` | 층별 엘리베이터 지점 (map 프레임) |
| `rooms` | 호실별 정차 자세 `goal`과 문 위치 `door`. 한 호실에 문이 둘이면 `"413-앞문"`처럼 나눈다 |
| `pass_floors.<층>.level`, `beacon_floor_code` | 지도 없는 통과층(현재 1·2·6·7F). 카가 서도 계속 탑승하고, 목적지·웹 층 탭에는 나오지 않는다. 실측 지도가 생기면 `floors`로 옮긴다 |

현재 엘리베이터 지점(3개 층 공통): 홀 문 중심 (9.975, 3.31), 호출·대기 (10.975, 3.31, π; 문에서 1.0 m, 2026-10-27 이전 0.6 m), 하차 종료 (10.775, 3.31, 0).
`sim/config/floor4_rooms.json`은 `sim_demo.py` 호환용이다.

## 6. 실차 CAN 경로 (**프로토콜 v1.1 · 하위제어기 구현 · Jetson 반영 · 홀 호출~카 안 정렬 실차 통과, 카 비콘 이후 미시험**)

확정 명세는 두 문서로 나뉜다. 무선 구간(nRF24 설정, RF 프레임, 비콘 동작)은 [엘리베이터 RF 프로토콜](ELEVATOR_RF_PROTOCOL.md),
하위제어기 ↔ Jetson CAN 프레임은 [엘리베이터 RF 중계 CAN 프로토콜](ELEVATOR_CAN_PROTOCOL.md)이다. 이 절은 요약이다.

```text
비콘·버튼 장치 ◀── RF ──▶ RF 모듈 ── STM32 ◀── CAN(can0) ──▶ Jetson: elevator_can_link ◀──▶ /elevator/command, /elevator/beacon,
                                                                                    /elevator/command_status, /elevator/rf_status
```

- 실차·시뮬레이션의 경계는 ROS 토픽 `/elevator/command`·`/elevator/beacon`이다.
- `elevator_can_link`는 자체 SocketCAN 소켓을 열고 커널 필터 하나(`0x310`, mask `0x7FE`)로 0x310·0x311만 받는다.
  `can_telemetry`·`cmd_vel_to_can` 소켓과 독립이다. bringup `start_elevator_link:=false`(기본), `elevator_link_dry_run:=true`(기본).
  통합 시험을 통과하면 실차 기본값을 바꾼다.
- 기존 사용 ID 0x001–0x038(ODrive), 0x201(주행), 0x202(E-stop 상태, [ESTOP_CAN_PROTOCOL.md](ESTOP_CAN_PROTOCOL.md)), 0x301·0x302(IMU)와 겹치지 않는다:

| ID | 방향 | 바이트 배치 (DLC 8) |
|---|---|---|
| **0x210** | Jetson → STM32 → RF | [0] 프로토콜 버전 [1] sequence [2] command [3] floor_code(int8) [4] direction [5] repeat(DOOR_OPEN만 0–15) [6] 예약 [7] CRC-8 |
| **0x310** | RF → STM32 → Jetson | [0] 프로토콜 버전 [1] sequence [2] event [3] floor_code(int8) [4] location [5] ack_sequence [6] rssi(int8, nRF24 RPD 등급값 −60/−80) [7] CRC-8. 홀·카 PRESENCE와 BUTTON_ACK |
| **0x311** | STM32 → Jetson | 무선 진단: COMMAND_STATUS → `/elevator/command_status`, LINK_STATUS·DEVICE_STATUS → `/elevator/rf_status`·`/diagnostics` |

- Jetson 쪽 ID·바이트 배치는 `amr_can_bridge/elevator_protocol.py` 한 곳에만 정의한다. [CAN 프로토콜 §9](ELEVATOR_CAN_PROTOCOL.md#9-시험-벡터) 벡터 16개를
  `test_elevator_protocol.py`가 바이트 그대로 검사한다. 비콘 표·링크 상태 집계는 `elevator_rf_status.py`.
- `/diagnostics`(`amr/elevator_can_link`): 0x311 LINK_STATUS가 3초 넘게 없거나 RADIO_OK=0이면 WARN/ERROR, 비콘 `BATT_LOW` WARN,
  `BUTTON_FAULT`·`CONFIG_ERR` ERROR. 디코드·송신 오류는 10초 동안만 WARN으로 남는다.
- 하위제어기(펌웨어 담당, 구현 완료): 수신 필터 뱅크 1의 4번째 슬롯(중복 `0x021`)을 0x210으로 바꿨다. 실 펌웨어는 `C:\STM32\AMR`이며
  이 저장소의 `firmware/stm32/main.c`는 그 프로젝트의 `Core/Src/main.c` 사본이다(`elevator_rf.c`·`nrf24.c`는 포함하지 않음).
- 버스 부하: 0x310은 들리는 비콘 수 × 1 Hz + 응답, 0x311은 1 Hz + 명령당 2–3개 + 비콘당 0.1 Hz로, 주행 명령(0x201, 20 Hz)에 비해 무시할 수준이다.

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
[Isaac]  elevator_runtime.py ── /sim/elevator/beacon_raw ──▶ elevator_sim_relay ──▶ /elevator/beacon
           │                   ── /sim/elevator/command_status_raw ─▶            ──▶ /elevator/command_status
           └ ElevatorSim 모델 ◀─ /sim/elevator/command_raw ─                    ◀── /elevator/command
           └ 카·문 USD 이동      /sim/elevator/status (지상 진실), /sim/elevator/inject (고장 주입)
```

| 동작 (`amr_sim_bridge/elevator_model.py`) | 값 |
|---|---|
| 카 이동 | 사다리꼴 속도, 1.0 m/s · 0.5 m/s² |
| 문 | 열림·닫힘 각 1.5초, 열림 유지 6~10초 난수(seed 고정). 문 평면에 물체가 있으면 다시 열림(미션은 의존하지 않음) |
| 버튼 장치 | 명령 0.2초 뒤 버튼을 누르고 `BUTTON_ACK`와 `COMPLETED` 명령 상태. 모르는 층 홀 호출은 `NO_ROUTE` |
| 열림 반복 | `repeat` N이면 1초(PRESENCE 주기)마다 N회 누름, 새 열림 명령은 횟수를 다시 셈 |
| 홀 비콘 | 1초마다 `PRESENCE` (지도 있는 층만. 통과층은 시뮬레이션에 없다) |
| 카 비콘 | 로봇이 카 안이거나 카가 로봇 층에 서서 문이 열렸을 때 1초마다 `PRESENCE`(층 0) |
| RF 수신 세기 | 같은 층 −50 dBm, 인접 층 20 % 확률 −82 dBm 누설, 카 안은 도착 층 문이 절반 이상 열렸을 때 −58 dBm |
| 문 상태 | 알려주지 않음 (AMR이 LiDAR로 판별) |

고장 주입(`FAULT_DEFAULTS`): `drop_ack_probability`/`drop_next_acks`(ACK 누락), `ack_extra_delay_s`, `duplicate_probability`/`reorder_probability`,
`drop_presence_probability`, `wrong_floor_code`, `stamp_lag_s`(오래된 데이터), `stop_at_floor_code`(중간 층 정차), `hold_time_s`,
`door_never_opens`, `button_fails`(장치 고장).

## 8. 웹 UI와 중앙 관제

- `GET /api/floors`(층 목록·현재 층·호실), `GET /api/floors/<층>/map.png`(다른 층 지도), `POST /api/goal`에 `floor`·`room`.
  현재 층과 같으면 기존 `/navigate_to_pose`, 다르면 `/floor_mission/navigate`. 다른 층 목표는 해당 층 지도의 `map_version`으로 검사한다.
- 화면: 층 탭(현재 층 표시), 호실 선택("3층 302호"), 층간 미션 단계 표시. `/floor/state`가 없으면(엘리베이터 노드 미실행) 단일 층 화면과 같다.
- 실차 모드에서는 `real_enabled: false` 층을 목적지로 고를 수 없다. 통과층은 층 탭에 나오지 않는다.
- 차량 상태 화면의 "엘리베이터 RF · 비콘" 패널: 1–7F·E1 비콘별 마지막 수신·세기(RPD 강/약)·배터리·고장 플래그, nRF24 링크 카운터,
  마지막 버튼 명령 상태(`/elevator/rf_status`, `/elevator/command_status`). 층간 이동 줄에는 버튼 전달 지연·거절을 함께 표시한다.
- 중앙 관제 경유: `amr_remote_bridge`가 `/api/floors`와 층 지도(`floor/<층>` 레이어)를 함께 전달하고, 다른 층 목표는 서버와 AMR 양쪽에서
  그 층 지도의 버전·해시로 검증한다([amr_remote_bridge](../src/amr_system/amr_remote_bridge/README.md)).

## 9. 실차 영향

| 변경 | 실차 영향 | 기본값 |
|---|---|---|
| `navigation.yaml`, `can_telemetry` | `can_telemetry` 변경 없음. `navigation.yaml`은 2026-10-25 깊이 카메라 `depth_layer`를 전역·지역 costmap에 반영. inflation·robot_radius·footprint는 유지, 카메라가 없으면 깊이 레이어는 비어 있음 | — |
| `elevator_can_link` | 별도 소켓. 통합 시험 전까지 실행하지 않음 | `start_elevator_link:=false` |
| `/localization/suspend`, 기동 임대 | 호출하지 않으면 기존과 동일 | 적용 |
| `elevator_monitor`, `floor_mission_manager` | monitor는 센서를 읽고 `/floor/*`만 발행. mission은 접수된 층간 목표에서만 움직임 | 실차·sim building `true`, 그 외 sim `false` |
| 웹 UI 층 선택 | 층 탭·호실 목록 표시. 실차 허용 층(현재 4F)만 선택 가능 | 적용 |

## 10. 시나리오

### 실행

```bash
export ROS_DOMAIN_ID=107                     # 107 이외 도메인에서는 실행 거부
./scripts/experiment.sh sim world:=building start_floor:=4F    # 수동 (웹 UI 층 탭 → 호실)
python3 scripts/run_floor_trial.py --output logs/elevator/try1 \
    --start-floor 4F --floor 3F --room 302 [--fault '{"drop_next_acks":2}'] \
    [--fault-at 'RIDING={"faults":{"stamp_lag_s":3.0}}'] [--bag]
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

원자료 `logs/elevator/final_20261014/` (시뮬레이션 PC). **11/11 통과.** 센서 주기를 실물에 맞추기 전(휠 10 Hz, LiDAR 8.8 Hz) 조건이다.

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
| 장치 구성 | 1–7F 홀 비콘(1F UP만, 7F DOWN만), 카 E1(층 버튼·열림, 층을 알리지 않음 → 도착 층은 홀 비콘으로만 확인) — RF §6.2 |
| RF 누설 | 승강로를 따라 인접 층 비콘이 수신될 수 있다고 보고 RSSI·응답 기반으로만 층 확정 — 가정 |
| 범위 | 엘리베이터 1대. 승하차 공간 점유와 회전 장애물을 검사하며 다른 승객 호출에 의한 중간 정차도 처리 |
| 문 | 카 문과 홀 문이 함께 열리고 닫힌다 — 가정 |
| 기동 | 전진 진입 → 카 안 180° 회전 → 전진 하차 (회전 지름 약 0.84 m < 카 폭 1.87 m) |
| 지도 | 3F·5F 실차 지도는 실측(2026-10-06 SLAM)을 4F 좌표계로 정렬해 넣었다(`scripts/align_floor_map.py`, 엘리베이터 문 위치 차 3F 6.5 cm·5F 4.5 cm). 층 전환이 자세 값을 그대로 쓰므로 모든 층 지도는 같은 좌표계여야 한다. 시뮬레이션 3F·5F 장면·영역지정과 호실 좌표는 아직 4F 복사본 (`maps/G_3_floor/MAP_SOURCE.md`) |
| 실차 | 3F `real_enabled: true`(2026-10-24 홀 호출~카 안 정렬 실차 2회 통과 후). 카 비콘이 없어 카 안 `SELECT_FLOOR`에서 `NO_CAR_ACK`로 안전 정지하며, LB로 수동 회수한다. 2026-10-26 3F→4F 탑승·이동·하차·도착까지 실차 통과. 5F `true`(2026-10-27, 미시험) |
| CAN·RF | 프로토콜 v1.1([RF](ELEVATOR_RF_PROTOCOL.md), [CAN](ELEVATOR_CAN_PROTOCOL.md)). 하위제어기 구현·단독 검증, Jetson 반영. 실차에서 4F 홀 호출 → 실제 버튼 눌림(램프 점등) → ACK 확인(2026-10-24). 현장 채널·RPD 보정, 카 비콘 남음 |
| 카 "열림" 버튼 | `repeat` 정의(1.1), 카 비콘 펌웨어 미구현. 없어도 후퇴·재호출 규칙으로 동작 |
| 통과층 | 1·2·6·7F는 지도 없이 비콘만 쓴다. 시뮬레이션 장면에는 없다 |
| 수치 | 층고 3.6 m, 카 속도 1.0 m/s, 센서 노이즈 0은 임시값 |
| 긴 복도 끝 호실 | 302·502(x ≈ 33.7 m)는 하차 후 주행 중 위치 오차 0.53~0.76 m로 단일 층 시험(0.19~0.27 m)보다 컸다. 하차 직후 재위치 오차는 약 2 cm라 4층 복도 위치 추정 문제로 보인다. 반복 측정 필요 |
| 시작 경합 | 단일 층 월드에서도 Isaac·Nav2 시작 단계에서 간헐적 타임아웃. 재실행하면 통과 |

## 12. 관련 파일

| 분류 | 경로 |
|---|---|
| 층 설정 | `src/amr_system/amr_stack/config/floors.yaml`, `amr_stack/floor_config.py` |
| 공용 알고리즘 | `src/amr_system/amr_elevator/amr_elevator/` — `mission_node.py`, `monitor_node.py`, `floor_state.py`, `door_detector.py`, `car_matcher.py`, `maneuver.py` |
| 메시지 | `src/amr_system/amr_elevator_interfaces/` |
| 실차 CAN | `src/amr_system/amr_can_bridge/amr_can_bridge/elevator_can_link.py`, `elevator_protocol.py` |
| 시뮬레이션 모델 | `src/amr_system/amr_sim_bridge/amr_sim_bridge/elevator_model.py`, `elevator_node.py` |
| Isaac 런타임·장면 | `sim/scripts/elevator_runtime.py`, `sim/scripts/run_ros_sim.py`, `sim/tools/build_building_scene.py`, `sim/config/building.json` |
| 런치 | `src/amr_system/amr_stack/launch/experiment.launch.py`, `bringup.launch.py` |
| 시험 | `scripts/run_floor_trial.py`, `scripts/run_floor_suite.py`, `sim/config/scenarios.json` |
