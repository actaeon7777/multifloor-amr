# 층간 이동(엘리베이터) 설계안

> 상태: **구현·시뮬레이션 검증 완료 (2026-10-14, 결과 logs/elevator/REPORT.md), 실차 미적용** · 작성 2026-10-13 · 개정 2026-10-13 (RF→STM32→CAN 경로, 가정 A3·A6·A8 반영)
> 범위: 3F·4F·5F 엘리베이터 층간 이동. 시뮬레이션에서 구현·검증하고, 실차 공용 로직은 실차 인터페이스 기준으로 작성한다.
> 작업 전 백업: `backups/amr_ws_pre_elevator_20261013_230836.tar.gz` (+ `.sha256`)

## 0. 확정된 요구 사항 (2026-10-13 사용자 답변)

| 항목 | 결정 |
|---|---|
| 문 열림·닫힘 | AMR이 **LiDAR로 직접 판별**. 장치는 문 상태를 알리지 않는다 |
| 장치(비콘)가 알리는 것 | ① "나는 N층 비콘" (층 식별) ② "버튼을 눌렀다" (확인 응답) **만** |
| 안/밖 판별 | 신호 + 위치 추정을 **함께** 사용 |
| 버튼 호출 | AMR이 **엘리베이터 앞에서** RF로 장치에 버튼 누름 명령 송신 |
| 비콘 통신 경로 | 비콘·버튼 장치 ↔ **RF** ↔ STM32(RF 모듈 연결) ↔ **CAN** ↔ Jetson. Jetson은 비콘을 직접 제어하지 않고 STM32가 중계한다 (확인) → 4.3 |
| 홀 비콘 층 신호 | **1초 주기** 발신 (확인) |
| 카 내부 "열림" 버튼 | 장치가 누를 수 있도록 **계획 중** (확인) |
| 문턱 깊이 | 층 벽 ~ 카 안쪽 벽 **66 cm** (확인) |
| 문 열림 유지 시간 | 약 **6~10초** |

엘리베이터 치수 ([`docs/images/Elevator_size.jpg`](images/Elevator_size.jpg), 단위 cm): 카 내부 폭 187 × 깊이 223, 출입구 폭 110 (카 왼쪽 벽에서 38.5),
문턱 구간 8 / 10.5 / 21 / 37 (층 벽 ~ 카 안쪽 벽 66 cm, 확인됨).

## 1. 현재 구조에서 출발점

| 기존 요소 | 활용 방식 |
|---|---|
| `navigation_startup`의 `/map` 변경 감지 (`on_map`) | 지도 교체 시 자동으로 `WAITING_INITIAL_POSE` 진입 → `/initialpose` 수동 검증(1 s·4샘플) 경로 재사용 |
| `/localization/relocalize` (TIM) | 수동 위치 검증 실패 시 대체 경로 |
| Nav2 `map_server`의 `/map_server/load_map` | 층 전환 시 지도 교체 (현재 코드에서 미사용, Jazzy 표준 서비스) |
| `navigation_manager` `/navigate_to_pose` | 층 내 구간 주행은 그대로 사용 |
| 4층 USD의 `ElevatorDoor_01` | 중심 (9.975, 3.31), 벽면 x≈9.975, ±y 방향 슬라이드, 개구 **1.0 m** (실측 1.10 m와 다름) |
| 4층 시작 위치 (11.5, 3.31) | 엘리베이터 앞 홀, 문에서 동쪽 1.5 m |

**현재 없는 것**: 엘리베이터 카(내부 공간)·승강로 형상, 문 제어 ROS 인터페이스(GUI 키보드 `P`만), 로봇 순간이동 API, 층/호실 개념, 웹 UI의 이름 있는 목적지.

## 2. 핵심 설계 결정

1. **층 판별의 1차 근거는 비콘, 2차는 미션 문맥.** 3F·5F 지도가 4F 복사본이므로 지도 매칭으로는 층을 구분할 수 없다.
   실측 지도로 교체되면 지도 일치율을 추가 확인 근거로 쓸 수 있도록 판별 노드에 자리를 남긴다.
2. **엘리베이터 구간(탑승·카 내부·하차)은 Nav2가 아닌 전용 저속 기동 제어기**가 맡는다.
   현재 inflation 1.6 m·`robot_radius` 0.44 m로는 1.1 m 출입구를 통과하는 경로가 나오지 않는다.
   `navigation.yaml`을 바꾸면 실차 복도 주행 전체가 영향을 받으므로 **변경하지 않는다.**
3. **카 내부에서는 위치 추정을 일시 정지(SUSPENDED)** 하고 EKF 오도메트리 + LiDAR 카 벽 기준으로 기동한다.
   하차 후 홀에서 새 층 지도로 `/initialpose`를 넣어 기존 수동 검증 경로로 READY를 회복한다.
4. **문 상태는 LiDAR 전용 노드**가 판별한다 (실차·시뮬레이션 공용).
5. 실차 공용 노드는 모두 **기본 비활성(launch 인자 false)** 으로 추가해 기존 실차 동작을 바꾸지 않는다.

## 3. 상태 머신

### 3.1 위치 영역(zone)·층 판별 — `floor_state_estimator` (실차 공용)

출력 영역은 사양대로 두 가지가 주 상태이며, 판별 불가 시 `UNKNOWN`을 둔다.

```text
            문 OPEN + 문턱선 안쪽 통과(odom) + 카 둘러싸임 확인(LiDAR)
 OUTSIDE ───────────────────────────────────────────────────────▶ IN_ELEVATOR
    ▲                                                                   │
    └──────────── 문 OPEN + 문턱선 바깥쪽 통과(odom) + 홀 개방 확인 ─────┘
 (어느 상태든) 규칙 위반·근거 상실 ──▶ UNKNOWN ──(운영자 /floor/set_current 또는 재검증)──▶ OUTSIDE
```

| 판별 근거 | 사용 방식 |
|---|---|
| 문턱선 통과 | 진입 직전(홀, AMCL 양호) 위치를 기준으로 EKF 오도메트리 변위로 문턱선(문 평면) 통과 판정 |
| 카 둘러싸임 (LiDAR) | 2.5 m 이내 빔 비율 ≥ 85 %, 측정 사각형이 카 치수(1.87×2.23)와 ±0.15 m 일치 |
| 홀 개방 (LiDAR) | 카 둘러싸임 해제, 전방 개방 거리 > 3 m |
| 문 상태 | `/elevator/door` (아래 3.3) |

**층(floor) 값 규칙**

| 상황 | 층 값 | 신뢰도 |
|---|---|---|
| OUTSIDE, 지도 로드된 층 | 현재 층 | `CONFIRMED` |
| IN_ELEVATOR, 문 닫힘 이후 | `IN_TRANSIT(출발→목표)` | `TRANSIT` |
| 도착 후 문 OPEN, 목표 층 홀 비콘 수신 | 목표 층 | `BEACON` |
| 하차 후 새 지도 위치 검증 READY | 목표 층 | `CONFIRMED` |
| 비콘 미수신 상태로 문 OPEN | 목표 층(추정) | `ASSUMED` — **하차 금지** |

**신호 이상 처리 규칙**

| 이상 | 처리 |
|---|---|
| 중복 (같은 `sequence`) | 무시, 카운터 증가 |
| 순서 꼬임 (8비트 시퀀스 창 128 기준 과거) | 폐기, 카운터 증가 |
| 지연 (수신 시각 − 생성 시각 > 2.0 s 또는 2.0 s 이상 오래된 상태) | 폐기 |
| 인접 층 비콘 누설 (OUTSIDE에서 현재 층과 다른 층 비콘) | 층 변경하지 않음. `floor_conflict` 경고만 |
| 카 이동 중(IN_TRANSIT) 층 비콘 변경 | 무시하고 로그. 도착 판정에 쓰지 않음 |
| 문 OPEN인데 비콘 층 ≠ 목표 층 (다른 승객 호출 등) | 하차 금지, 목표 층 버튼 재송신, 카 내부 대기 |
| 문 OPEN 없이 IN_ELEVATOR ↔ OUTSIDE 전환 조건 발생 | 불법 전이 → `UNKNOWN`, 미션 안전 정지 |
| 엘리베이터 영역이 아닌 곳에서 층 변경 요구 | 거부 |

### 3.2 층간 미션 — `floor_mission_manager` (실차 공용)

```text
IDLE
 ├─(같은 층 목표)──────────────────────────────▶ NAV_TO_GOAL ─▶ SUCCEEDED
 └─(다른 층 목표)─▶ NAV_TO_HALL ─▶ CALL_ELEVATOR ─▶ WAIT_DOOR_OPEN ─▶ BOARDING
                                                                  │
   ┌──────────────────────────────────────────────────────────────┘
   ▼
 IN_CAR_ALIGN ─▶ SELECT_FLOOR ─▶ RIDING ─▶ WAIT_ARRIVAL ─▶ SWITCH_MAP ─▶ ALIGHTING
                                                                          │
   ┌──────────────────────────────────────────────────────────────────────┘
   ▼
 RELOCALIZE ─▶ NAV_TO_GOAL ─▶ SUCCEEDED
```

| 상태 | 동작 | 진행 조건 | 시간 초과·이상 |
|---|---|---|---|
| NAV_TO_HALL | 현재 층 `hall_wait_pose`로 Nav2 주행 | Nav2 SUCCEEDED | Nav2 실패 → FAILED_SAFE |
| CALL_ELEVATOR | `HALL_CALL(방향)` 송신 | 해당 층 홀 비콘 `BUTTON_ACK` | 3 s 무응답 → 재송신, 3회 실패 → FAILED_SAFE |
| WAIT_DOOR_OPEN | LiDAR 문 감시 | 문 `OPENING`/`OPEN` | 120 s → 재호출 1회 → FAILED_SAFE |
| BOARDING | localization **SUSPEND**, 카 내 `OPEN 유지` 명령(가정 A6), 기동 제어기로 직진 진입 | 문턱선 통과 + 카 둘러싸임 | 로봇이 문턱선 **도달 전** 문 `CLOSING` 감지 → 정지·후진해 홀 대기 위치 복귀 → CALL 재시도 |
| IN_CAR_ALIGN | 카 중앙 정렬 + 180° 회전(문 방향) | 정렬 오차 < 0.1 m·5° | 20 s → FAILED_SAFE (정지 유지) |
| SELECT_FLOOR | `CAR_FLOOR(목표 층)` 송신 | 카 비콘 `BUTTON_ACK` | 3 s × 3회 |
| RIDING | 정지 유지, 문 CLOSED 확인 | 문 CLOSED 지속 ≥ 2 s | — |
| WAIT_ARRIVAL | 문 감시 | 문 OPEN + 목표 층 홀 비콘 | 다른 층에서 열림 → 하차 금지·재선택. 180 s → FAILED_SAFE |
| SWITCH_MAP | `/map_server/load_map(목표 층 지도)` | 새 `/map` 수신(지도 키 변경) | 실패 → FAILED_SAFE (카 안 정지) |
| ALIGHTING | 기동 제어기로 직진 하차 → `hall_exit_pose` | 문턱선 바깥 통과 + 홀 개방 | 문 CLOSING 감지 시 문턱선 이전이면 카 중앙 복귀 |
| RELOCALIZE | localization **RESUME**, `/initialpose` = 목표 층 `hall_exit_pose` (+odom 보정) | `/localization/status` READY | 실패 → `/localization/relocalize`(TIM) 1회 → FAILED_SAFE |
| NAV_TO_GOAL | Nav2 주행 | SUCCEEDED | 기존 Nav2 처리 |

- **PAUSED**: RB로 MANUAL 전환 시 어느 단계든 기동 즉시 정지, 상태 보존. AUTO 복귀 시 해당 단계 처음부터 재개
  (BOARDING/ALIGHTING 중이었다면 문 상태를 다시 확인).
- **FAILED_SAFE**: 정지 유지, 웹 UI에 사유 표시, 운영자 조치(MANUAL 수동 이동 또는 `/floor/set_current`) 대기.

**문 열림 시간 예산 (6~10 s)** — 가장 빡빡한 구간이다.

| 항목 | 값 |
|---|---|
| 홀 대기 위치 → 문 평면 | 0.6 m (설정값) |
| 문턱 구간 | 0.66 m (해석값, 확인 필요) |
| 로봇 후단이 카 문 안쪽 통과까지 | 0.6 + 0.66 + 0.32 ≈ 1.58 m |
| 0.4 m/s 기동 시 | ≈ 4.0 s + 문 감지 지연 ≈ 0.5 s → **≈ 4.5 s** |

최소 6 s 안에 들어오지만 여유가 작다. 그래서 카 장치의 "열림" 버튼 누름(가정 A6)과
문턱선 이전 닫힘 감지 시 후퇴 규칙을 함께 둔다. 실제 엘리베이터의 문 끼임 방지 센서에 의존하지 않는다.

### 3.3 문 상태 — `elevator_door_detector` (실차 공용)

- 입력: `/scan`, TF(`map→laser`, 카 내부에서는 `odom→laser`), 층 설정의 문 선분.
- 문 선분을 지나는 빔 중 거리가 `문 평면 + 0.3 m`를 넘는 비율 = `open_ratio`.
- 상태: `UNKNOWN / CLOSED / OPENING / OPEN / CLOSING`. 히스테리시스(OPEN ≥ 0.7, CLOSED ≤ 0.2), 연속 3스캔 확인.
- 문 선분이 시야각 밖이거나 유효 빔 < 10 이면 `UNKNOWN` (미션은 정지 대기).

### 3.4 위치 추정 일시 정지 (`navigation_startup` 확장)

- 신규 서비스 `/localization/suspend` (`std_srvs/SetBool`): true → 상태 `SUSPENDED`, `ready=false`, detail "엘리베이터 구간".
  - AMCL 결과·검증을 무시하고 READY 판정을 멈춘다. Nav2 lifecycle은 건드리지 않는다.
  - `navigation_manager`는 READY가 아니므로 Nav2 목표를 받지 않는다 (의도된 동작).
- false → 지도 변경 여부와 무관하게 `WAITING_INITIAL_POSE`로 가서 `/initialpose`를 기다린다 (기존 수동 검증 1 s·4샘플).
- 이 서비스를 호출하지 않으면 기존 동작과 동일하다.

### 3.5 기동 명령 경로 (`navigation_manager` 확장)

- 신규 입력 `/cmd_vel_maneuver` (Twist). `/navigation/maneuver_lease` 서비스로 임대(기본 30 s, 갱신 가능)한 동안만
  `/cmd_vel_auto`로 전달한다. AUTO 모드일 때만 통과, 0.3 s 이상 입력 끊기면 0 출력.
- 임대 중에는 Nav2 목표를 받지 않는다. RB(MANUAL) 전환 시 임대를 즉시 해제한다.
- `control_mux`·바퀴 속도 제한·CAN 경로는 변경하지 않는다.

## 4. 인터페이스

### 4.1 신규 패키지 `amr_elevator_interfaces` (ament_cmake + rosidl, 기존 `*_interfaces` 관례)

```text
msg/ElevatorBeacon.msg       # 비콘 → AMR
  std_msgs/Header header
  uint8 EVENT_PRESENCE=1     # "나는 N층 비콘"
  uint8 EVENT_BUTTON_ACK=2   # "버튼을 눌렀다"
  uint8 LOCATION_HALL=0
  uint8 LOCATION_CAR=1
  uint8 event
  uint8 location
  int8 floor_code            # 홀 비콘의 층 (카 비콘은 0)
  uint8 sequence
  uint8 ack_sequence         # BUTTON_ACK가 응답한 명령 시퀀스
  int8 rssi_dbm              # 없으면 0
  string source              # "can" | "sim"

msg/ElevatorCommand.msg      # AMR → 장치
  std_msgs/Header header
  uint8 CMD_HALL_CALL=1
  uint8 CMD_CAR_FLOOR=2
  uint8 CMD_DOOR_OPEN=3      # 카 내부 "열림" 버튼 (가정 A6)
  uint8 DIR_NONE=0
  uint8 DIR_UP=1
  uint8 DIR_DOWN=2
  uint8 command
  int8 floor_code
  uint8 direction
  uint8 sequence

msg/ElevatorDoorState.msg
  std_msgs/Header header
  uint8 UNKNOWN=0
  uint8 CLOSED=1
  uint8 OPENING=2
  uint8 OPEN=3
  uint8 CLOSING=4
  uint8 state
  float32 open_ratio
  uint16 valid_beams
  string elevator_id

msg/FloorState.msg
  std_msgs/Header header
  uint8 ZONE_UNKNOWN=0
  uint8 ZONE_OUTSIDE=1
  uint8 ZONE_IN_ELEVATOR=2
  uint8 zone
  string floor               # "4F", 이동 중이면 "" (from/to 사용)
  string transit_from
  string transit_to
  string confidence          # CONFIRMED | BEACON | TRANSIT | ASSUMED | UNKNOWN
  string last_event
  uint32 duplicate_count
  uint32 out_of_order_count
  uint32 stale_count
  bool floor_conflict

msg/FloorEvent.msg           # 상태 전이 이벤트
  std_msgs/Header header
  string event               # ENTERED_ELEVATOR | EXITED_ELEVATOR | DEPARTED | ARRIVED | FLOOR_CONFIRMED | RULE_VIOLATION ...
  string from_floor
  string to_floor
  string detail

srv/SetCurrentFloor.srv      # 운영자 수동 지정
  string floor
  ---
  bool success
  string message

action/NavigateToFloorPose.action
  string floor
  string destination_id      # 예: "302" (선택)
  geometry_msgs/PoseStamped pose
  ---
  bool success
  string error_code
  string message
  ---
  string phase
  string current_floor
  uint8 zone
  string detail
```

### 4.2 토픽·서비스·액션

| 이름 | 타입 | 발행/제공 | 실차 | 시뮬레이션 |
|---|---|---|---|---|
| `/elevator/beacon` | ElevatorBeacon | elevator_can_link / sim | CAN 0x310(안) 수신 | `elevator_sim` |
| `/elevator/command` | ElevatorCommand | floor_mission_manager | `elevator_can_link` → CAN 0x210(안) | `elevator_sim`이 구독 |
| `/elevator/door` | ElevatorDoorState | elevator_door_detector | 공용 | 공용 |
| `/floor/state` (TRANSIENT_LOCAL) | FloorState | floor_state_estimator | 공용 | 공용 |
| `/floor/events` | FloorEvent | floor_state_estimator | 공용 | 공용 |
| `/floor/set_current` | SetCurrentFloor | floor_state_estimator | 공용 | 공용 |
| `/floor_mission/navigate` | NavigateToFloorPose (action) | floor_mission_manager | 공용 | 공용 |
| `/floor_mission/status` (TRANSIENT_LOCAL) | String(JSON) | floor_mission_manager | 공용 | 공용 |
| `/localization/suspend` | std_srvs/SetBool | navigation_startup | 공용 | 공용 |
| `/cmd_vel_maneuver`, `/navigation/maneuver_lease` | Twist, 신규 srv | navigation_manager | 공용 | 공용 |
| `/sim/elevator/status` | String(JSON) | elevator_sim | — | 실제 카 위치·문 상태(진단·평가용) |
| `/sim/elevator/inject` | String(JSON) 서비스 | elevator_sim | — | 고장 주입 |

`/floor_mission/status`는 기존 `/localization/status`, `/navigation/status`와 같은 JSON String 관례를 따른다.
`VehicleStatus` 메시지에는 이번 단계에서 층 필드를 추가하지 않는다 (중앙 관제 스키마 영향 회피). 웹 UI는 `/floor/state`를 직접 구독한다.
중앙 관제 경유는 `amr_remote_bridge`가 `/api/floors`와 층 지도(`floor/<층>` 레이어)를 함께 전달하고, 다른 층 목표는 그 층 지도의 버전·해시로 검증한다 (2026-10-16).

### 4.3 실차 전송 경로와 CAN 프레임 제안 (**ID·페이로드 미확정 · TODO**)

```text
비콘·버튼 장치 ◀── RF ──▶ RF 모듈 ── STM32 ◀── CAN(can0, 500 kbit/s) ──▶ Jetson: elevator_can_link ◀──▶ /elevator/command, /elevator/beacon
```

- Jetson은 비콘을 직접 제어하지 않는다. STM32가 RF 내용을 CAN으로 중계한다.
- 실차·시뮬레이션의 경계는 ROS 토픽 `/elevator/command`·`/elevator/beacon`이다. 공용 노드는 이 토픽만 본다.
- Jetson 측은 **신규 노드 `elevator_can_link`**(`amr_can_bridge` 패키지) 하나가 송수신을 맡는다.
  자체 SocketCAN 소켓을 열고 커널 필터를 0x310으로만 둔다. 기존 `can_telemetry`·`cmd_vel_to_can`은 **변경하지 않는다**
  (`can_telemetry` 시험의 필터 집합 검사도 그대로 유지된다).
- 기존 사용 ID: 0x001–0x038(ODrive 노드 0·1), 0x201(주행 명령), 0x301·0x302(IMU). 아래 후보는 이들과 겹치지 않는다.

| ID (후보) | 방향 | DLC | 바이트 배치 (후보) |
|---|---|---|---|
| **0x210** | Jetson → STM32 → RF | 8 | [0] 프로토콜 버전 [1] sequence [2] command [3] floor_code(int8) [4] direction [5] 대상 장치 ID [6] 예약 [7] CRC-8 |
| **0x310** | RF → STM32 → Jetson | 8 | [0] 프로토콜 버전 [1] sequence [2] event [3] floor_code(int8) [4] location [5] ack_sequence [6] rssi(int8) [7] CRC-8 |

- STM32 쪽 필요 작업(펌웨어 담당, TODO): 수신 필터에 0x210 추가(뱅크 1의 빈 슬롯은 1개뿐이라 새 뱅크 권장),
  `HAL_CAN_RxFifo0MsgPendingCallback`에 0x210 분기, RF 모듈 송수신, 1초 비콘 수신분을 0x310으로 전달.
  **이번 작업에서 `firmware/stm32/main.c`는 수정하지 않는다.**
- 0x310 수신 빈도: 비콘 1 Hz × 수신 가능한 층 수 + 버튼 응답. 주행 명령(0x201, 20 Hz) 대비 부하는 무시할 수준이다.
- ID·바이트 배치는 코드에서 한 곳(`elevator_protocol.py`)에만 정의해, 확정 시 이 파일과 시험만 고치면 되게 한다.

## 5. 층 설정 파일 — `src/amr_system/amr_stack/config/floors.yaml`

지도 교체 시 이 파일만 고친다. 경로는 워크스페이스 기준이다.

```yaml
schema_version: 1
building: G
default_floor: "4F"
elevators:
  E1:
    car_interior_m: [1.87, 2.23]     # [문 벽 방향 폭, 깊이]  Elevator_size.jpg
    door_width_m: 1.10
    door_offset_from_left_m: 0.385
    threshold_depth_m: 0.66          # 층 벽 ~ 카 안쪽 벽 (확인)
    door_open_time_s: [6.0, 10.0]
    car_device_id: 0x10              # TODO
floors:
  "4F":
    level: 4
    beacon_floor_code: 4
    hall_device_id: 0x04             # TODO
    real_map: maps/G_4_floor/4_floor_0522.yaml
    sim_map: maps/sim_floor4/floor4.yaml
    sim_scene_layer: sim/layers/4_floor_0522_map.usda
    real_enabled: true               # 실차 층간 이동 허용 여부
    elevators:
      E1:
        door_center: [9.975, 3.31]
        car_inward_yaw: 3.1416       # 홀 → 카 진입 방향 (map 기준)
        hall_wait_pose: [10.575, 3.31, 3.1416]
        hall_exit_pose: [10.775, 3.31, 0.0]
    rooms:
      "401": {goal: [41.08, -0.2, -1.5708], door: [41.08, -1.17]}
      # ... 402~406
  "3F":
    level: 3
    beacon_floor_code: 3
    real_map: maps/G_3_floor/3_floor_copy_of_4F_0522.yaml   # 임시 복사본
    sim_map: maps/sim_floor3/floor3.yaml
    sim_scene_layer: sim/layers/3_floor_map.usda
    real_enabled: false              # 실측 지도 전까지 실차 층간 이동 금지
    elevators: {E1: {...}}           # 복사본이므로 4F와 같은 좌표
    rooms: {"301": ..., "306": ...}
  "5F": { ... }
sim:
  floor_height_m: 3.6                # TODO 실측
  car_speed_m_s: 1.0
  car_accel_m_s2: 0.5
```

- 3F·5F 지도·장면은 4F를 복사하고, 폴더에 `COPY_OF_4F.md`를 둬 임시본임을 표시한다.
- 기존 `sim/config/floor4_rooms.json`은 `sim_demo.py` 호환을 위해 유지하고, `floors.yaml`에서 생성하도록 바꾼다.
- 파서는 `amr_stack/floor_config.py` 하나로 두고 모든 노드·스크립트·웹 UI가 이것을 쓴다. 로드할 때 지도 파일 존재 여부, 층 코드 중복, 좌표 범위를 검사한다.

## 6. Isaac Sim 장면 방식 비교

| 기준 | A. 층 수직 적층 + 카 실제 이동 | B. 층별 장면 전환(순간이동) |
|---|---|---|
| 구성 | 3개 층 레이어를 z = 0, 3.6, 7.2 m에 배치, 승강로 + kinematic 카가 수직 이동 | 한 층만 활성. 도착 시 로봇·카를 다른 층 복사본으로 순간이동 |
| 물리 충실도 | 높음. 카 바닥 위 로봇 접촉, 가감속, IMU z 가속도 자연 발생 | 낮음. 수직 운동 없음, 순간이동 순간 접촉·속도 리셋 필요 |
| RTF | 정적 충돌체 약 3배. 광역 단계·BVH 비용 증가는 로그 수준으로 예상 → **측정 필요** | 현재와 동일 |
| LiDAR | 광선이 수평이고 층 간격이 3.6 m라 다른 층은 기울기 약 6° 이상에서만 보임. 카 벽·승강로가 그대로 보임 | 영향 없음 |
| 기존 코드 영향 | 접촉 필터 경로(`/World/GMap/*`), 지도 추출 z 단면(0.1946 m), 지상 진실 z 값을 층별로 처리해야 함 | 순간이동 API 신규(현재 없음). "일시정지·리셋 시 시험 무효" 규칙과 충돌 가능 |
| 시험 신뢰성 | 오도메트리·IMU가 연속 | 순간이동 시점에 오도메트리·EKF 불연속 처리 필요 |

**제안: A (수직 적층).** 층간 이동의 핵심 위험(탑승 중 접촉, 문 타이밍, 카 내부 LiDAR)을 그대로 재현할 수 있다.
4단계 시작 시 RTF와 카 위 로봇 안정성을 먼저 측정하고, RTF < 0.9이거나 카 이동 중 로봇이 미끄러지면 B로 전환한다.

**장면 구성 (A 기준)**

- `sim/layers/building_G.usda`: 3F·4F·5F 맵 레이어를 z 오프셋으로 참조하고, 각 층 엘리베이터 문 위치에 승강로 벽을 둔다.
- 카: 실측 치수 내부 1.87 × 2.23 m, 출입구 1.10 m, kinematic rigid body, 카 문 2패널 (홀 문과 동기 동작).
  `generate_occupancy_usd.py`의 문 생성 방식을 재사용하고, 치수는 `floors.yaml`에서 읽는다.
- 4층 홀 문 개구를 1.0 m에서 1.10 m로 맞춘다 (`floors.yaml` 기준. 원본 주석 PNG는 수정하지 않음).
- `elevator_sim` (Isaac 프로세스 내부, `sim/scripts/elevator_sim.py` + `amr_sim_bridge`):
  - 버튼 장치 모델: `/elevator/command` 수신 → 지연(기본 0.2 s) 후 `BUTTON_ACK` 발행.
  - 홀 비콘: 1 Hz `PRESENCE`. 카가 있는 층의 홀 비콘은 강하게, 인접 층은 약하게(RSSI)·확률적으로 수신해 누설을 재현.
  - 카 제어: 호출 → 이동(사다리꼴 속도) → 도착 → 문 열림 → 6~10 s 유지(시드 기반 난수) → 닫힘.
    문 평면에 물체가 있으면 다시 연다 (시뮬레이션 문 보호 동작. 미션 로직은 이것에 의존하지 않음).
  - 고장 주입 (`scenarios.json`의 `faults` 또는 `/sim/elevator/inject`): ACK 누락·지연·중복·순서 뒤바뀜,
    잘못된 층 비콘, 다른 층에서 문 열림, 조기 닫힘.

## 7. 웹 UI 확장

- `GET /api/floors`: 층 목록, 현재 층, 호실 목록 (`floors.yaml`).
- `GET /api/floors/<floor>/map.png`: 다른 층 지도 미리보기 (목적지 지정용). 현재 층은 기존 `/api/layers/map.png`.
- `POST /api/goal` 확장: `floor`, `room`(선택) 필드. 현재 층과 같으면 기존 `/navigate_to_pose`, 다르면 `/floor_mission/navigate`.
  기존 필드·검증(request_id, map_version, 점유 셀 검사)은 유지하며, 다른 층 목표는 해당 층 지도의 `map_version`으로 검사한다.
- 화면: 상단 층 선택 탭(3F/4F/5F), 호실 목록 드롭다운("3층 302호"), 층간 미션 단계와 엘리베이터 상태 표시.
  하드코딩된 "G-BUILDING · 4TH FLOOR" 라벨을 현재 층으로 바꾼다.
- `real_enabled: false`인 층은 실차 모드에서 목적지로 선택할 수 없게 막는다.

## 8. 실차 영향

| 변경 | 실차 영향 | 기본값 |
|---|---|---|
| `navigation.yaml` | **변경 없음** | — |
| CAN 경로 (`can_telemetry`, `cmd_vel_to_can`) | **변경 없음** | — |
| `elevator_can_link` 신규 노드 (0x210 송신·0x310 수신) | 별도 소켓. STM32 펌웨어 준비 전까지 실행하지 않음 | `start_elevator_link:=false` |
| `navigation_startup` `/localization/suspend` | 호출하지 않으면 기존과 동일 | 적용 |
| `navigation_manager` 기동 임대 입력 | 임대하지 않으면 기존과 동일 | 적용 |
| `floor_state_estimator`·`floor_mission_manager`·`elevator_door_detector` | 실행하지 않으면 영향 없음 | `start_elevator:=false` (sim에서는 building 월드에서 true) |
| 웹 UI 층 선택 | `floors.yaml`에서 실차 허용 층이 1개면 기존 화면과 같음 | 적용 |
| `firmware/stm32/main.c` | **수정하지 않음** | — |

## 9. 단계별 작업 계획

| 단계 | 내용 | 검증 |
|---|---|---|
| 1 | 3F·5F 지도·장면 복제, `floors.yaml` + `floor_config.py` 파서, rooms 생성 | 파서 단위 시험, 기존 floor4 장면·지도 해시 불변 확인, colcon 빌드 |
| 2 | `amr_elevator_interfaces`, `elevator_protocol.py`(0x210/0x310 인코더·디코더, TODO 표시), `elevator_can_link`, `elevator_sim` 신호 발생기(ROS 단독, 1 Hz 비콘·버튼 응답·고장 주입) | 프레임 왕복·CRC 단위 시험, FakeBus 시험(실제 CAN 미사용), 도메인 193 ROS 시험 |
| 3 | `floor_state_estimator`, `elevator_door_detector` | 정상 시퀀스 + 누락·중복·순서·지연·누설·불법 전이 시퀀스 단위 시험, 합성 스캔 문 판별 시험 (도메인 195) |
| 4 | 수직 적층 장면, 카·승강로, 카 이동, 기동 제어기, 탑승·하차 | RTF·카 위 안정성 측정 → A/B 결정, 4F 단일 탑승·하차 반복 시험 (도메인 107) |
| 5 | `floor_mission_manager`, `load_map` 전환, localization suspend/resume, 기동 임대, 웹 UI | 미션 상태 머신 단위 시험, 웹 HTTP·ROS 시험(193), 브라우저 확인 |
| 6 | `scenarios.json`에 4F→3F, 4F→5F, 3F→5F + 고장 주입 시나리오, `run_sim_trial.py` 층간 지원 | 시나리오별 결과를 `logs/elevator/<시나리오>/`에 저장, 요약 보고 |

각 단계 끝에 기존 단위 테스트 전체와 arena·floor4 통합 시험(`run_sim_trial.py`)을 다시 돌린다. 결과와 변경 사항은 아래 부록과 커밋 메시지에 기록한다.

**평가 지표 (6단계)**: 목표 도착 오차(지상 진실 기준), 전체·구간별 소요 시간(대기·탑승·이동·하차), 충돌 수,
복구·재시도 횟수(버튼 재송신, 재호출, TIM 재위치), RTF, 층·영역 판별과 실제 위치(`/sim/elevator/status`, 지상 진실 z) 일치율.

## 10. 확인이 필요한 가정

| # | 가정 | 영향 |
|---|---|---|
| A1 | 장치는 각 층 홀에 1대, 카 내부에 1대. 홀 장치는 상·하 호출 버튼, 카 장치는 층 버튼을 누른다 | 명령 종류 |
| A2 | 카 내부 장치는 층을 알리지 않는다 (`floor_code=0`, `location=CAR`) | 도착 층은 홀 비콘으로만 확인 |
| A3 | ✅ 확인: 홀 비콘은 "N층 비콘" 신호를 1초 주기로 보낸다 | 층 확인 |
| A4 | RF가 승강로를 따라 인접 층 비콘까지 수신될 수 있다 → RSSI 또는 응답 기반으로만 층 확정 | 누설 처리 |
| A5 | 엘리베이터 1대, 카 안은 비어 있다고 가정. 다른 승객 호출로 중간 층에서 문이 열리는 경우는 처리 | 범위 |
| A6 | 🔶 계획 중: 카 장치가 "열림" 버튼도 누른다. 미션은 이 기능 없이도 동작하도록 하고(후퇴·재호출 규칙), 있으면 사용 | 문 타이밍 |
| A7 | 카 문과 홀 문은 함께 열리고 닫힌다 | 문 판별 |
| A8 | ✅ 확인: 층 벽 ~ 카 안쪽 벽 0.66 m | 기동 거리 |
| A9 | 층고 3.6 m, 카 속도 1.0 m/s (시뮬레이션 전용 임시값) | 장면·소요 시간 |
| A10 | 탑승은 전진 진입 → 카 안 180° 회전 → 전진 하차 (회전 지름 약 0.84 m < 카 폭 1.87 m) | 기동 방식 |
| A11 | 층 이름 `3F/4F/5F`, 호실은 4층 좌표 복사(301~306, 501~506) | 웹 UI·시나리오 |
| A12 | 실차 3F·5F 층간 이동은 실측 지도 전까지 비활성 (`real_enabled: false`) | 실차 안전 |
| A13 | RF 모듈은 STM32에 연결, STM32가 CAN으로 중계 (확인). CAN ID 0x210/0x310과 페이로드는 후보이며 펌웨어 담당과 확정 필요 | 펌웨어·`elevator_protocol.py` |

시뮬레이션 수치(소요 시간, 위치 오차 등)는 실물 보정 전 모델 결과이며 실차 성능으로 확정하지 않는다.

## 11. 구현 반영 사항 (2026-10-14)

설계와 달라진 점과 구현 중 확인한 사항이다.

| 항목 | 설계 | 구현 |
|---|---|---|
| 문·층 판별 노드 | `elevator_door_detector` + `floor_state_estimator` | 한 노드 `elevator_monitor` (같은 위치 추정을 공유해야 해서 합침) |
| 카 안 위치 | 오도메트리 + LiDAR 카 벽 | 마지막으로 검증된 map→odom(anchor) + 오도메트리, 로봇 전체가 카 안에 있을 때 카 내부 벽 맞춤(`car_matcher.py`)으로 보정. 층이 바뀌면 카 기준 상대 자세를 유지해 anchor 이동 |
| 기동 명령 | `/cmd_vel_maneuver` + 임대 | 동일 (`navigation_manager`, `/navigation/maneuver_lease` = `std_srvs/SetBool`, 상태 `MANEUVER`) |
| 위치 추정 일시 정지 | `/localization/suspend` | 동일 (상태 `SUSPENDED`; 웹 UI·vehicle_status 상태 목록에 추가) |
| Isaac ↔ ROS | `amr_sim_bridge`가 직접 발행 | Isaac 내장 rclpy는 사용자 메시지를 쓸 수 없어 Isaac은 JSON 토픽(`/sim/elevator/*_raw`)만 쓰고, 시스템 Python의 `elevator_sim_relay`가 `/elevator/command`·`/elevator/beacon`으로 변환 |
| 장면 방식 | A(수직 적층) 우선 | A 채택. USD 변환으로 kinematic 카를 움직였을 때 AMR이 함께 이동(지연 2×10⁻⁵ m, 기울기 0.015°), 3개 층 장면 RTF 1.0 |
| 층 바닥 | 4개 슬래브 | **삼각형 메시**로 생성. Cube 충돌체에서는 제자리 회전 뒤 바퀴가 바닥에 파고들어 하차 중 기울어지고 멈춤(재현·원인 확인) |
| 카 바닥 ↔ 홀 바닥 | 1.5 cm 틈 | 카 바닥판이 슬래브 아래로 2 cm 겹치게 해서 틈을 없앰 (kinematic과 static은 충돌하지 않음) |
| 시험 도구 | — | `scripts/run_floor_trial.py`(1회), `scripts/run_floor_suite.py` + `sim/config/scenarios.json`의 `floor_scenarios` |


## 부록: 단계별 구현 기록

설계: `docs/ELEVATOR_DESIGN.md`. 작업 전 백업: `backups/amr_ws_pre_elevator_20261013_230836.tar.gz`.

#### 1단계 — 3F·5F 임시 지도·장면, 층 설정 파일
- 층 설정 `src/amr_system/amr_stack/config/floors.yaml` + 파서 `amr_stack/floor_config.py` (ROS 비의존).
  층 → 지도(실차·sim) → USD 레이어 → 엘리베이터 대기·하차 지점 → 호실. 지도 교체 시 이 파일만 수정.
- 3F·5F는 4F 복사본(`provisional_copy_of: "4F"`, 실차 `real_enabled: false`): `src/amr_system/amr_stack/maps/G_{3,5}_floor/`,
  `.../maps/sim_floor{3,5}/` (각 폴더 `COPY_OF_4F.md`), `sim/layers/{3,5}_floor_tmp_from_4F_map.usda`.
- `sim/tools/build_floor_layers.py`: floors.yaml의 모든 층 레이어 생성. `build_scene.sh` 첫 단계를 이것으로 교체
  (4F 레이어는 바이트 단위 동일 확인).
- 엘리베이터 E1(4F 문 중심 9.975, 3.31, 카는 서쪽): 카 내부 1.87×2.23 m, 문 1.10 m, 층 벽~카 안쪽 벽 0.66 m.
  카 중심 (8.2, 3.31). 홀 대기 (10.575, 3.31, π), 하차 (10.775, 3.31, 0).
- 실차 영향: 없음 (기존 4F 지도·launch·navigation.yaml 불변, 신규 지도 폴더 설치만 추가).
- 확인: `test_floor_config.py` 32개 통과 (지점 여유 0.3 m 실차·sim 지도 모두, 카 내부 벽 없음,
  USD 문 위치 일치, 임시본=원본 해시, 잘못된 설정 18종 거부). amr_stack 67→99 통과, 나머지 패키지 기준과 동일.
  회귀: arena 성공(도착 0.032 m, 충돌 0, 복구 0), floor4 405호 성공(도착 0.164 m, 충돌 0, 복구 0) —
  `logs/elevator/stage1/`.

#### 2단계 — 엘리베이터 신호 인터페이스, 시뮬레이션 신호 발생기
- 신규 `amr_elevator_interfaces`: msg `ElevatorBeacon`·`ElevatorCommand`·`ElevatorDoorState`·`FloorState`·`FloorEvent`,
  srv `SetCurrentFloor`, action `NavigateToFloorPose`.
- 실차 경로(RF 모듈 → STM32 → CAN → Jetson): `amr_can_bridge/elevator_protocol.py`(0x210 명령·0x310 이벤트,
  CRC-8, **ID·바이트 배치는 제안값 TODO**), 노드 `elevator_can_link`(자체 소켓, 0x310 필터, 기본 dry_run).
  `bringup.launch.py`에 `start_elevator_link:=false`(기본 꺼짐), `elevator_link_dry_run:=true` 추가.
  기존 `can_telemetry`·`cmd_vel_to_can`·`firmware/stm32/main.c`는 변경 없음.
- 시뮬레이션: `amr_sim_bridge/elevator_model.py`(카 이동·문·버튼 장치·1초 홀 비콘·인접 층 누설·고장 주입),
  `elevator_node.py`(`elevator_sim` 실행 파일, `/elevator/beacon`·`/sim/elevator/status`·`/sim/elevator/inject`).
  `amr_sim_bridge/setup.cfg` 추가: `ros2 run` 경로 + launch_testing pytest 플러그인 비활성
  (없으면 모듈 단위 skip 하나가 디렉터리 전체를 skip시킴).
- 실차 영향: 기본값으로는 없음 (새 노드는 꺼진 상태).
- 확인: can_bridge 47→68, sim_bridge 11→44 통과. ROS 시험 `test_elevator_ros.py` 2개 통과(도메인 193,
  `AMR_RUN_ELEVATOR_TESTS=1`): sim 비콘·응답·지상 진실 기반 카 탑승 판정, CAN 링크 왕복(가짜 버스).
  `scripts/test_ros_integration.py` 러너에는 등록하지 않음(러너·문서 검사가 스위트별 고유 도메인을 요구하는데
  193/195는 web_ui·vehicle_status가 사용 중). 회귀: arena 성공(0.027 m), floor4 405호 성공(0.163 m), 충돌·복구 0
  — `logs/elevator/stage2/`.

#### 3단계 — 층·상태 판별 (실차 공용)
- 신규 `amr_elevator` 패키지, 노드 `elevator_monitor` 하나에 LiDAR 문 판별과 층·영역 판별을 둔다
  (설계안의 `elevator_door_detector`·`floor_state_estimator`를 위치 추정 공유를 위해 한 노드로 합침).
  - `door_detector.py`: 문 선분을 지나는 빔의 통과 비율, 히스테리시스·연속 2스캔, 문틀 0.08 m 제외.
  - `floor_state.py`: 비콘 필터(지연 2 s·중복·순서 꼬임, 장치별 시퀀스), 층 변경은 카를 통해서만
    (닫힘+확인된 목표층 → DEPARTED, 열림+강한 홀 비콘 → ARRIVED), 이동 중 비콘 무시, 밖에서 다른 층 비콘은
    FLOOR_CONFLICT만, 다른 층 정차는 목표 유지, 비콘 없으면 ASSUMED(하차 시 RULE_VIOLATION),
    문 열림 없이 문턱 통과·이동 중 하차는 UNKNOWN(운영자 `/floor/set_current`로만 복구).
  - 위치: 마지막으로 검증된 map→odom(anchor) + 오도메트리. 층이 바뀌면 카 기준 상대 자세를 유지해 anchor 이동.
- 실차 영향: 없음 (launch에 아직 포함하지 않음).
- 확인: amr_elevator 단위 16개, ROS `test_monitor_ros.py` 1개(도메인 195, `AMR_RUN_ELEVATOR_MONITOR_TESTS=1`,
  4F→3F 전체 탑승·하차·운영자 지정) 통과. 회귀: arena·floor4 405호 성공 — `logs/elevator/stage3/`.

#### 4단계 — 엘리베이터 장면, 탑승·하차 주행
- 장면 방식 A(수직 적층) 채택: `sim/tools/build_building_scene.py` → `sim/building_G.usda`(+ `sim/config/building.json`).
  층 z = (level − 4) × 3.6 m. 층 바닥은 승강로 구멍을 뺀 **삼각형 메시**, 승강로 벽(static), kinematic 카(바닥·벽·천장)와
  카 문 2짝. `build_scene.sh`에 추가. 실행: `./scripts/experiment.sh sim world:=building [start_floor:=3F]`.
- Isaac 런타임 `sim/scripts/elevator_runtime.py`: `ElevatorSim` 모델로 카 높이·카 문·해당 층 홀 문을 매 물리 단계 USD로 갱신,
  로봇이 문 평면에 있으면 문 닫힘 보류(시뮬레이션 문 보호). Isaac은 사용자 메시지를 못 쓰므로 JSON 토픽만 사용하고
  `elevator_sim_relay`(시스템 Python)가 `/elevator/command`·`/elevator/beacon`으로 변환.
- `run_ros_sim.py`: `--world building`, `--start-floor`; 충돌 판정 경로를 `physics_parameters.obstacle_path()`로 일반화
  (바닥 메시·카 바닥 제외). 기존 floor4·합성 월드의 판정 결과는 동일.
- 탑승·하차: `amr_elevator/maneuver.py`(직선 저속 기동, 0.35 m/s), `navigation_manager`의 `/navigation/maneuver_lease`
  + `/cmd_vel_maneuver`(임대 중에만, AUTO에서만, 0.2 s 끊기면 0), `navigation_startup`의 `/localization/suspend`
  (`SUSPENDED`, 재개 시 `/initialpose` 대기). 카 안에서는 카 내부 벽 맞춤으로 위치 보정(`car_matcher.py`).
- 디버깅 기록: (1) 슬래브를 Cube로 만들면 카 안에서 제자리 회전한 뒤 하차할 때 x≈9.8 m에서 3.7° 기울며 멈춤 →
  프로브로 재현(회전 없으면 정상, 원래 메시 바닥이면 정상) → 슬래브를 메시로 바꿔 해결. (2) 카 바닥과 슬래브 사이 1.5 cm 틈 제거.
  (3) 문 근처에서 카 벽 맞춤이 홀 벽을 잘못 맞춰 추정이 0.2 m 뒤처짐 → 로봇 전체가 카 안일 때만 보정.
  (4) 1회: Nav2 lifecycle의 map_server 설정 응답 타임아웃으로 지도가 활성화되지 않음(시작 시 DDS 경합, 엘리베이터와 무관).
- 실차 영향: `navigation_manager`·`navigation_startup`에 서비스가 추가됐지만 호출하지 않으면 동작 동일
  (기존 ROS 시험 navigation_manager 19개·navigation_startup 90개 통과, 각 스위트 지정 도메인 190·194).
  웹 UI·vehicle_status 상태 목록에 `SUSPENDED`·`MANEUVER` 추가. `navigation.yaml` 변경 없음.

#### 5단계 — 층간 미션, 지도 전환·재위치, 웹 UI
- `amr_elevator/mission_node.py`(`floor_mission_manager`, action `/floor_mission/navigate`): 홀 이동(Nav2) → 호출(응답 3 s×3) →
  문 열림 대기(LiDAR) → 위치 추정 정지·임대 → 탑승(문턱 전 닫힘 시 후퇴·재호출, 최대 3회) → 카 안 정렬 → 층 버튼 →
  탑승(다른 층 정차 시 대기·재선택) → `/map_server/load_map` → 하차(카 문 전 닫힘 시 카 안 복귀·열림 버튼) →
  `/initialpose`로 기존 수동 검증(실패 시 TIM 1회) → 목적지(Nav2). 처리 불가 상황은 정지 상태로 `FAILED_SAFE`.
  `experiment.launch.py`: `world:=building`이면 monitor·mission·relay 자동 실행(`elevator:=true`로 실차에서도 가능, 기본 꺼짐).
  실차에서는 `real_enabled: false` 층(3F·5F 임시 지도)을 목적지로 받지 않음.
- 웹 UI: `GET /api/floors`, `GET /api/floors/<층>/map.png`, `POST /api/goal`에 `floor`·`room`(다른 층이면 층간 미션으로 전달).
  화면에 층 탭(현재 층 표시)·호실 선택·층간 이동 단계 표시. `/floor/state`가 없으면(엘리베이터 노드 미실행) 기존 단일 층 화면과 같음.
- 확인: amr_web_ui 73→78(+ROS 시험 3개 통과, 도메인 193), JS 시험 2개 통과(`test_tracking_status.js`는 `/api/floors` 조회 허용으로 갱신).
  Isaac: 4F→3F 302호 0.032 m, 4F→5F 502호 0.268 m, 3F→5F 503호 0.78/0.17/0.33 m(0.78 m 1회는 503호 앞 긴 복도에서
  AMCL이 진행 방향으로 0.8 m 뒤처진 기존 유형의 오차, 하차 직후 재위치 오차는 2 cm), 충돌 0 — `logs/elevator/stage4/`.

#### 6단계 — 층간 시나리오 통합 시험
- `scripts/run_floor_trial.py`(1회 시험, 고장 주입·적용 확인, bag 선택), `scripts/run_floor_suite.py` +
  `sim/config/scenarios.json`의 `floor_scenarios` 11종(정상 3 + 신호 누락·지연·중복·다른 층 정차 5 + 안전 정지 3).
- 최종 결과 11/11 통과 — 요약 `logs/elevator/REPORT.md`, 원자료 `logs/elevator/final_20261014/`.
  성공 8건 도착 오차 평균 0.21 m(최대 0.49 m), 충돌 0, Nav2 복구 0, RTF 1.0, 층·영역 일치 95~100 %.
- 시험 중 수정한 결함: `elevator_monitor`가 경고 이벤트 기록 시 rclpy 로거 예외로 종료(실차에도 해당) → 수정·회귀 시험 추가;
  미션에 `/floor/state` 3 s 단절 시 안전 정지 추가; 늦게 온 이전 버튼 누름 응답 인정; Nav2 구간에서 위치 추정이
  잠시 풀리면 재시도를 소모하지 않고 최대 45 s 회복 대기; 시험 도구의 시작 시 고장 주입 미적용.
- 격리 ROS 시험(193/195): monitor 1, mission 6, lease·suspend 2, sim+CAN 2, web_ui 3, vehicle_status 10 통과
  (`logs/elevator/final_ros/`). 단위 시험: amr_elevator 23, amr_sim_bridge 45, amr_can_bridge 68, amr_stack 99,
  amr_control 122, amr_web_ui 78, amr_remote_bridge 28 통과.
- 회귀: arena 0.064 m, floor4 405호 0.14 m, 402호 0.167·0.143 m 성공(`logs/elevator/final_regression/`).
  Nav2 시작 경합으로 2회 재실행(단일 층 월드에서도 발생, 엘리베이터와 무관).
- 남은 확인: 긴 복도 끝 호실(302·502) 하차 후 주행 위치 오차가 단일 층 비교보다 큼(표본 적음).
  실차 반영 전 STM32 0x210/0x310 펌웨어 확정, 실측 3F·5F 지도 교체 후 `real_enabled`, 엘리베이터 실측(문턱 높이·틈).
  `docs/VALIDATION.md` 3.8에 재현 명령.
