# 층간 이동 (엘리베이터)

> 상태: **시뮬레이션 구현·검증 완료 (2026-10-14), 실차 미적용**. 범위는 G동 3F·4F·5F, 엘리베이터 1대(E1).
> 공용 로직은 실차 인터페이스 기준으로 작성했고, 실차 전용 노드는 기본으로 꺼져 있다.
> 시뮬레이션 수치는 실물 보정 전 모델의 결과다. 구현 단계별 기록은 [실험 기록](EXPERIMENTS.md#2026-10-1314--층간-이동엘리베이터-구현).

## 1. 요구 사항 (2026-10-13 확정)

| 항목 | 결정 |
|---|---|
| 문 열림·닫힘 | AMR이 **LiDAR로 직접 판별**. 장치는 문 상태를 알리지 않는다 |
| 장치(비콘)가 알리는 것 | ① "나는 N층 비콘"(층 식별, 홀 비콘 **1초 주기**) ② "버튼을 눌렀다"(확인 응답) **만** |
| 안/밖 판별 | 신호 + 위치 추정을 함께 사용 |
| 버튼 호출 | AMR이 엘리베이터 앞에서 RF로 장치에 버튼 누름 명령 송신 |
| 통신 경로 | 장치 ↔ **RF** ↔ STM32(RF 모듈) ↔ **CAN** ↔ Jetson. Jetson은 비콘을 직접 제어하지 않는다 |
| 카 내부 "열림" 버튼 | 장치가 누를 수 있도록 **계획 중**. 없어도 동작해야 한다 |
| 치수 | 카 내부 1.87 × 2.23 m, 출입구 1.10 m(카 왼쪽 벽에서 0.385 m), 층 벽 ~ 카 안쪽 벽 **0.66 m** ([`images/Elevator_size.jpg`](images/Elevator_size.jpg)) |
| 문 열림 유지 | 약 6~10초 |

## 2. 핵심 설계 결정

1. **층 판별의 1차 근거는 비콘, 2차는 미션 문맥.** 3F·5F 지도가 4F 복사본이라 지도 매칭으로 층을 구분할 수 없다.
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
실차: elevator_can_link (CAN 0x210/0x310)                │   ├─ Nav2 (/navigate_to_pose)          홀까지·호실까지
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

`experiment.sh sim world:=building`이면 monitor·mission·relay가 자동으로 실행된다. 실차는 `elevator:=true`로 켤 수 있으며 기본은 꺼져 있다.

### 토픽·서비스·액션

메시지 정의는 [`amr_elevator_interfaces`](../src/amr_system/amr_elevator_interfaces/)
(`ElevatorBeacon`, `ElevatorCommand`, `ElevatorDoorState`, `FloorState`, `FloorEvent`, `SetCurrentFloor`, `NavigateToFloorPose`).

| 이름 | 타입 | 제공 |
|---|---|---|
| `/elevator/beacon` | ElevatorBeacon | `elevator_can_link` / `elevator_sim_relay` |
| `/elevator/command` | ElevatorCommand | `floor_mission_manager` |
| `/elevator/door` | ElevatorDoorState | `elevator_monitor` |
| `/floor/state` (transient-local) | FloorState | `elevator_monitor` |
| `/floor/events` | FloorEvent | `elevator_monitor` |
| `/floor/set_current` | SetCurrentFloor | `elevator_monitor` (운영자 수동 지정) |
| `/floor_mission/navigate` | NavigateToFloorPose (action) | `floor_mission_manager` |
| `/floor_mission/status` (transient-local) | String(JSON) | `floor_mission_manager` |
| `/localization/suspend` | std_srvs/SetBool | `navigation_startup` |
| `/navigation/maneuver_lease` | std_srvs/SetBool | `navigation_manager` |
| `/cmd_vel_maneuver` | Twist | 미션 → `navigation_manager` |
| `/sim/elevator/status`, `/sim/elevator/inject` | String(JSON) | 시뮬레이션 지상 진실, 고장 주입 |

- `/localization/suspend` true → `SUSPENDED`, `ready=false`. AMCL 결과·검증을 무시하고 Nav2 lifecycle은 건드리지 않는다.
  false → 지도 변경과 무관하게 `/initialpose`를 기다린다(수동 검증 1초·4샘플). 호출하지 않으면 기존 동작과 같다.
- `/cmd_vel_maneuver`는 임대 중(기본 30초, 갱신 가능)이고 AUTO일 때만 `/cmd_vel_auto`로 전달되며, 0.2초 끊기면 0을 낸다.
  임대 중에는 Nav2 목표를 받지 않고 RB(MANUAL) 전환 시 즉시 해제된다. 상태는 `MANEUVER`. control_mux·바퀴 제한·CAN 경로는 그대로다.
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
  | 문 OPEN + 강한 홀 비콘(≥ −70 dBm) | 목표 층 | `BEACON` |
  | 하차 후 위치 추정 READY | 목표 층 | `CONFIRMED` |
  | 비콘 없이 문만 열림 | 목표 층(추정) | `ASSUMED` — **하차 금지** |

- **비콘 필터**(장치별): 2초 넘은 신호 폐기, 5초 안 같은 시퀀스는 중복 폐기, 8비트 시퀀스 창(128) 기준 과거 번호 폐기, 카 이동 중 비콘 무시,
  밖에서 다른 층 비콘은 누설로 보고 `floor_conflict` 경고만 낸다. 다른 층 정차 시 목표를 유지한다.
  판별이 불가능하면 `UNKNOWN`이 되고 운영자 `/floor/set_current`로만 복구한다.

### 4.3 직선 기동 제어기 (`maneuver.py`)

- `ALIGN`(목표 방향 회전) → `DRIVE`(직진 + 방향 보정 + 감속) → `TURN`(최종 방향) → `DONE`.
- 최고 0.35 m/s, 회전 0.5 rad/s, 위치 허용 오차 0.05 m, 전방 여유 0.15 m 미만이면 정지. `reverse`는 방향을 유지한 채 후진(문이 닫힐 때 후퇴).
- AUTO일 때만 명령을 낸다. MANUAL로 바뀌면 즉시 0을 내고 단계는 유지한다.

### 4.4 층간 미션 (`mission_node.py`)

```text
IDLE ─(같은 층)──────────────────────────────────────────────────────▶ NAV_TO_GOAL ─▶ SUCCEEDED
     └(다른 층)▶ NAV_TO_HALL ▶ CALL_ELEVATOR ▶ WAIT_DOOR_OPEN ▶ BOARDING ▶ IN_CAR_ALIGN
                ▶ SELECT_FLOOR ▶ RIDING ▶ SWITCH_MAP ▶ ALIGHTING ▶ RELOCALIZE ▶ NAV_TO_GOAL
     복구: BOARDING ─(문 닫힘)▶ BACKING_OUT ▶ RELOCALIZE ▶ CALL_ELEVATOR
           ALIGHTING ─(문 닫힘)▶ RETURN_TO_CAR ▶ ALIGHTING
     어느 단계든 실패 ▶ FAILED_SAFE (정지 유지, 오류 코드 표시)
```

**접수 조건**: 진행 중 미션 없음, `OUTSIDE`이고 현재 층이 `BEACON`/`CONFIRMED`, 위치 추정 READY. 실차에서는 목표 층이 `real_enabled: true`여야 한다.

| 단계 | 동작 | 다음 조건 | 이상 처리 (오류 코드) |
|---|---|---|---|
| NAV_TO_HALL | Nav2로 `hall_wait_pose` | Nav2 성공 | `NAV_TO_HALL_FAILED`, 위치 추정 45초 미복구 `LOCALIZATION_LOST` |
| CALL_ELEVATOR | `HALL_CALL(위/아래)`. 문이 이미 열려 있으면 건너뜀 | 홀 장치 ACK | 3초마다 재송신, 3회 → `NO_HALL_ACK` |
| WAIT_DOOR_OPEN | 문 감시. OPEN이면 위치 추정 정지 + 기동 임대 + "열림" 버튼 | 준비 완료 | 120초 → 재호출 1회 → `DOOR_NOT_OPENED`; `PREPARE_FAILED` |
| BOARDING | 직선 기동으로 카 중앙까지 | 카 안 판정 + 기동 완료 | 문턱 전 문 닫힘 → BACKING_OUT; 30초 `BOARDING_TIMEOUT` |
| BACKING_OUT | 홀 대기 위치로 후진 → 재위치 → 재호출 | — | 3회 → `BOARDING_FAILED` |
| IN_CAR_ALIGN | 카 중앙 정렬 + 문 쪽 180° 회전 | 기동 완료 | 20초 `ALIGN_TIMEOUT` |
| SELECT_FLOOR | `CAR_FLOOR(목표 층)` | 카 장치 ACK | 3초 × 3회 → `NO_CAR_ACK` |
| RIDING | 정지 유지, 문·층 감시 | 목표 층 + 문 OPEN + `BEACON`/`CONFIRMED` | 아래 표 |
| SWITCH_MAP | `/map_server/load_map` | 로드 성공 | `MAP_SWITCH_FAILED` (카 안 정지) |
| ALIGHTING | 직선 기동으로 `hall_exit_pose` | 기동 완료 + `OUTSIDE` | 문턱 전 문 닫힘 → RETURN_TO_CAR(2회 초과 `ALIGHTING_FAILED`); 30초 `ALIGHTING_TIMEOUT` |
| RETURN_TO_CAR | 카 중앙으로 후진, "열림" 버튼 | 문 OPEN | 15초마다 버튼, 초과 시 `DOOR_NOT_REOPENED` |
| RELOCALIZE | 위치 추정 재개 → 현재 자세로 `/initialpose` | READY | 30초 → TIM 1회 → `RELOCALIZE_FAILED` |
| NAV_TO_GOAL | Nav2로 호실 | 성공 | `NAV_TO_GOAL_FAILED` |

| RIDING 중 상황 | 처리 |
|---|---|
| 목표 층에서 문 열림, 비콘 없음(`ASSUMED`) | 하차 금지, "열림" 버튼 1회, 6초 안에 확인 안 되면 `ARRIVAL_UNCONFIRMED`(카 안 정지) |
| 다른 층에서 문 열림 (다른 승객) | 하차하지 않음. 카가 20초 넘게 멈추면 목표 층 버튼 재송신, 3회 초과 `RIDE_STALLED` |
| 180초 안에 도착 못 함 | `RIDE_TIMEOUT` |

**공통 안전 규칙**
- `/floor/state`가 3초 끊기면 `FLOOR_STATE_LOST`, 코드 예외는 `INTERNAL_ERROR`. 둘 다 즉시 정지.
- RB로 MANUAL 전환 시 기동 즉시 정지, 상태 보존. 취소 시 카 밖이면 위치 추정을 재개하고 카 안이면 정지 상태로 둔다.
- `FAILED_SAFE`에서는 정지를 유지하고 운영자 조치(MANUAL 수동 이동 또는 `/floor/set_current`)를 기다린다.
- 실제 엘리베이터의 문 끼임 방지 센서에 의존하지 않는다.

**문 열림 시간 예산**: 홀 대기 위치 → 문 평면 0.6 m + 문턱 0.66 m + 로봇 후단 0.32 m ≈ 1.58 m. 0.35~0.4 m/s 기동과 문 감지 지연을 더해
약 4.5~5초로, 최소 6초 안에 들어오지만 여유가 작다. 그래서 "열림" 버튼과 문턱 전 닫힘 시 후퇴 규칙을 함께 둔다.

## 5. 층 설정 — `src/amr_system/amr_stack/config/floors.yaml`

지도를 바꿀 때는 이 파일만 고친다. 파서는 `amr_stack/floor_config.py` 하나이며 노드·스크립트·웹 UI·장면 생성기가 모두 이것을 쓴다.
로드 시 지도 파일 존재, 층 코드 중복, 좌표 범위를 검사한다.

| 키 | 내용 |
|---|---|
| `elevators.<id>` | 카 치수, 출입구 폭·위치, 문턱 깊이, 문 열림 시간 |
| `floors.<층>.level`, `beacon_floor_code` | 층 순서, 비콘 층 코드 |
| `real_enabled` | 실차 층간 이동 허용. 3F·5F는 실측 지도 전까지 `false` |
| `provisional_copy_of` | 다른 층 지도를 복사한 임시본이면 원본 층 (3F·5F = `"4F"`, 각 지도 폴더에 `COPY_OF_4F.md`) |
| `map`, `annotations`, `sim_map` | 실차 지도, 영역 주석 PNG, 시뮬레이션 지도 (`amr_stack` 패키지 기준 경로) |
| `sim_layer`, `spawn` | 시뮬레이션 층 USD, 시작 자세 |
| `elevators.<id>.door_center`, `car_inward_yaw`, `hall_wait_pose`, `hall_exit_pose` | 층별 엘리베이터 지점 (map 프레임) |
| `rooms` | 호실별 정차 자세 `goal`과 문 위치 `door`. 한 호실에 문이 둘이면 `"413-앞문"`처럼 나눈다 |

현재 엘리베이터 지점(3개 층 공통): 홀 문 중심 (9.975, 3.31), 호출·대기 (10.575, 3.31, π), 하차 종료 (10.775, 3.31, 0).
`sim/config/floor4_rooms.json`은 `sim_demo.py` 호환용이다.

## 6. 실차 CAN 경로 (**ID·페이로드 미확정 · TODO**)

```text
비콘·버튼 장치 ◀── RF ──▶ RF 모듈 ── STM32 ◀── CAN(can0) ──▶ Jetson: elevator_can_link ◀──▶ /elevator/command, /elevator/beacon
```

- 실차·시뮬레이션의 경계는 ROS 토픽 `/elevator/command`·`/elevator/beacon`이다.
- `elevator_can_link`는 자체 SocketCAN 소켓을 열고 커널 필터를 0x310으로만 둔다. 기존 `can_telemetry`·`cmd_vel_to_can`은 바꾸지 않았다.
  bringup `start_elevator_link:=false`(기본), `elevator_link_dry_run:=true`(기본).
- 기존 사용 ID 0x001–0x038(ODrive), 0x201(주행), 0x301·0x302(IMU)와 겹치지 않는 후보:

| ID (후보) | 방향 | 바이트 배치 (후보, DLC 8) |
|---|---|---|
| **0x210** | Jetson → STM32 → RF | [0] 프로토콜 버전 [1] sequence [2] command [3] floor_code(int8) [4] direction [5] 대상 장치 ID [6] 예약 [7] CRC-8 |
| **0x310** | RF → STM32 → Jetson | [0] 프로토콜 버전 [1] sequence [2] event [3] floor_code(int8) [4] location [5] ack_sequence [6] rssi(int8) [7] CRC-8 |

- ID·바이트 배치는 `amr_can_bridge/elevator_protocol.py` 한 곳에만 정의한다. 확정 시 이 파일과 시험만 고친다.
- STM32 쪽 필요 작업(펌웨어 담당): 수신 필터에 0x210 추가(뱅크 1 빈 슬롯이 1개라 새 뱅크 권장), `HAL_CAN_RxFifo0MsgPendingCallback`에 0x210 분기,
  RF 모듈 송수신, 비콘 수신분을 0x310으로 전달. `firmware/stm32/main.c`는 아직 수정하지 않았다.
- 0x310 부하는 비콘 1 Hz × 수신 층 수 + 버튼 응답으로, 주행 명령(0x201, 20 Hz)에 비해 무시할 수준이다.

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
           └ ElevatorSim 모델 ◀─ /sim/elevator/command_raw ─                    ◀── /elevator/command
           └ 카·문 USD 이동      /sim/elevator/status (지상 진실), /sim/elevator/inject (고장 주입)
```

| 동작 (`amr_sim_bridge/elevator_model.py`) | 값 |
|---|---|
| 카 이동 | 사다리꼴 속도, 1.0 m/s · 0.5 m/s² |
| 문 | 열림·닫힘 각 1.5초, 열림 유지 6~10초 난수(seed 고정). 문 평면에 물체가 있으면 다시 열림(미션은 의존하지 않음) |
| 버튼 장치 | 명령 0.2초 뒤 버튼을 누르고 `BUTTON_ACK` |
| 홀 비콘 | 1초마다 `PRESENCE` |
| RF 수신 세기 | 같은 층 −50 dBm, 인접 층 20 % 확률 −82 dBm 누설, 카 안은 도착 층 문이 절반 이상 열렸을 때 −58 dBm |
| 문 상태 | 알려주지 않음 (AMR이 LiDAR로 판별) |

고장 주입(`FAULT_DEFAULTS`): `drop_ack_probability`/`drop_next_acks`(ACK 누락), `ack_extra_delay_s`, `duplicate_probability`/`reorder_probability`,
`drop_presence_probability`, `wrong_floor_code`, `stamp_lag_s`(오래된 데이터), `stop_at_floor_code`(중간 층 정차), `hold_time_s`,
`door_never_opens`, `button_fails`(장치 고장).

## 8. 웹 UI와 중앙 관제

- `GET /api/floors`(층 목록·현재 층·호실), `GET /api/floors/<층>/map.png`(다른 층 지도), `POST /api/goal`에 `floor`·`room`.
  현재 층과 같으면 기존 `/navigate_to_pose`, 다르면 `/floor_mission/navigate`. 다른 층 목표는 해당 층 지도의 `map_version`으로 검사한다.
- 화면: 층 탭(현재 층 표시), 호실 선택("3층 302호"), 층간 미션 단계 표시. `/floor/state`가 없으면(엘리베이터 노드 미실행) 단일 층 화면과 같다.
- 실차 모드에서는 `real_enabled: false` 층을 목적지로 고를 수 없다.
- 중앙 관제 경유: `amr_remote_bridge`가 `/api/floors`와 층 지도(`floor/<층>` 레이어)를 함께 전달하고, 다른 층 목표는 서버와 AMR 양쪽에서
  그 층 지도의 버전·해시로 검증한다([amr_remote_bridge](../src/amr_system/amr_remote_bridge/README.md)).

## 9. 실차 영향

| 변경 | 실차 영향 | 기본값 |
|---|---|---|
| `navigation.yaml`, CAN 송수신 노드, `firmware/stm32/main.c` | **변경 없음** | — |
| `elevator_can_link` | 별도 소켓. 펌웨어 준비 전까지 실행하지 않음 | `start_elevator_link:=false` |
| `/localization/suspend`, 기동 임대 | 호출하지 않으면 기존과 동일 | 적용 |
| `elevator_monitor`, `floor_mission_manager` | 실행하지 않으면 영향 없음 | `elevator:=false` (sim building은 true) |
| 웹 UI 층 선택 | 실차 허용 층이 1개면 기존 화면과 같음 | 적용 |

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
| 장치 구성 | 홀마다 1대(상·하 호출), 카 안 1대(층 버튼, 층을 알리지 않음 → 도착 층은 홀 비콘으로만 확인) — 가정 |
| RF 누설 | 승강로를 따라 인접 층 비콘이 수신될 수 있다고 보고 RSSI·응답 기반으로만 층 확정 — 가정 |
| 범위 | 엘리베이터 1대, 카는 비어 있다고 가정. 다른 승객 호출에 의한 중간 정차는 처리 |
| 문 | 카 문과 홀 문이 함께 열리고 닫힌다 — 가정 |
| 기동 | 전진 진입 → 카 안 180° 회전 → 전진 하차 (회전 지름 약 0.84 m < 카 폭 1.87 m) |
| 지도 | 3F·5F는 4F 복사본. 실측 지도로 바꾸면 층마다 평면 좌표 오프셋을 장면 생성기와 런타임에 넣어야 한다 |
| 실차 | 3F·5F `real_enabled: false`. 실측 지도 교체, 엘리베이터 실측(문턱 높이·틈) 후 활성화 |
| CAN | 0x210/0x310 ID·페이로드 확정, STM32 펌웨어 작업 |
| 카 "열림" 버튼 | 장치 계획 단계. 없어도 후퇴·재호출 규칙으로 동작 |
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
