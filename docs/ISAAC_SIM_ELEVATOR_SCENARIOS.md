# Isaac Sim 층간 이동(엘리베이터) — 환경·알고리즘·시나리오 정리

> 기준일 2026-10-14 · 상태: **시뮬레이션 구현·검증 완료, 실차 미적용**
> 설계 원문: [ELEVATOR_DESIGN.md](ELEVATOR_DESIGN.md) · 결과: [logs/elevator/REPORT.md](../logs/elevator/REPORT.md)
> 시뮬레이션 수치는 실물 보정 전 모델(`calibration_status: uncalibrated`)의 결과이며 실차 성능으로 확정하지 않는다.

## 1. 한눈에 보기

| 항목 | 내용 |
|---|---|
| 목적 | G동 3F·4F·5F 사이를 엘리베이터로 이동해 다른 층 호실까지 자율 주행 |
| 시뮬레이터 | Isaac Sim 6.1.0, 물리 200 Hz (TGS), 측정 RTF ≈ 1.0 |
| 장면 | `sim/building_G.usda` — 3개 층 **수직 적층** + 실제로 오르내리는 kinematic 엘리베이터 카 |
| ROS | ROS 2 Jazzy, `ROS_DOMAIN_ID=107` (시뮬레이션 전용) |
| 실행 | `./scripts/experiment.sh sim world:=building [start_floor:=3F]` |
| 시나리오 일괄 실행 | `scripts/run_floor_suite.py` (`sim/config/scenarios.json` → `floor_scenarios`) |
| 최종 결과 | 11/11 통과 (정상 3 · 신호 이상 회복 5 · 안전 정지 3), 충돌 0 |

## 2. 시뮬레이션 환경

### 2.1 건물 장면 (`sim/building_G.usda`)

`sim/tools/build_building_scene.py`가 [floors.yaml](../src/amr_system/amr_stack/config/floors.yaml)을 읽어 생성한다.

| 구성 | 값 / 방식 |
|---|---|
| 층 배치 | 3F z = −3.6 m, **4F z = 0** (기준층), 5F z = +3.6 m (`sim.floor_height_m`, 임시값) |
| 층 지도 | 4F 실측 지도 기반. **3F·5F는 4F 복사본**(`provisional_copy_of: "4F"`)이라 평면 좌표가 모두 같다 |
| 층 바닥 | 승강로 구멍을 뺀 **삼각형 메시** 슬래브 (Cube 충돌체는 제자리 회전 후 바퀴가 파고드는 문제가 있어 교체) |
| 승강로 | 모든 층을 관통하는 정적 벽(뒤·옆·문 옆 문턱 벽) |
| 엘리베이터 카 | kinematic rigid body. 내부 1.87 × 2.23 m, 출입구 1.10 m, 높이 2.3 m, 카 문 2패널 |
| 카 ↔ 홀 바닥 | 카 바닥판이 슬래브 아래로 2 cm 겹쳐 틈 없음 (1.5 cm 틈에서 캐스터가 걸리는 문제 수정) |
| 조명 | 층마다 시각용 천장 + RectLight, 카 내부 조명 |
| 시작 위치 | 각 층 `spawn` = (11.5, 3.31, 0) — 엘리베이터 홀, 문에서 동쪽 1.5 m |

엘리베이터 지점 좌표 (3개 층 공통, map 프레임):

| 이름 | 좌표 [x, y, yaw] | 의미 |
|---|---|---|
| `door_center` | (9.975, 3.31) | 홀 문 중심 |
| `hall_wait_pose` | (10.575, 3.31, π) | 호출·대기 위치 (문 평면에서 0.6 m, 카 쪽을 바라봄) |
| `hall_exit_pose` | (10.775, 3.31, 0) | 하차 종료 위치 (복도 쪽을 바라봄) |
| `threshold_depth_m` | 0.66 | 층 벽 ~ 카 안쪽 벽 |

호실은 층마다 같은 좌표이고 번호만 다르다. 예: 302·402·502 = (33.73, −0.2, −π/2), 305 = (11.08, −0.2, −π/2).

### 2.2 로봇·센서 모델

| 항목 | 값 | 출처 |
|---|---|---|
| 차량 | 차동구동 AMR (`sim/amr_flat_test.usda`), 실차와 같은 토픽 인터페이스 | `amr_sim_bridge` |
| LiDAR | 8.8 Hz, 3,240점, 0.05~30 m, 노이즈 0 | `sim/config/sensors.json` |
| 휠 오도메트리 / IMU | 10 Hz / 10 Hz | 〃 |
| 주행 명령 | 0.05 s 주기, 0.3 s 끊기면 정지 | 〃 |
| 물리 | 200 Hz, TGS, 위치 반복 32회 | `sim/config/physics.json` |

### 2.3 엘리베이터·RF 장치 모델

실차의 `RF 장치 ↔ STM32 ↔ CAN ↔ Jetson` 경로를 시뮬레이션에서는 아래 구조로 대체한다.
공용 노드는 실차와 똑같이 `/elevator/command`·`/elevator/beacon`만 본다.

```text
[Isaac 프로세스]                                    [시스템 Python / ROS 2]
elevator_runtime.py ── /sim/elevator/beacon_raw ──▶ elevator_sim_relay ──▶ /elevator/beacon
  └ ElevatorSim 모델 ◀─ /sim/elevator/command_raw ─  (JSON ↔ 사용자 메시지) ◀─ /elevator/command
  └ 카·문 USD 이동      /sim/elevator/status (지상 진실, 평가용)
                        /sim/elevator/inject (고장 주입)
```

Isaac 내장 rclpy는 사용자 정의 메시지를 쓸 수 없다. 그래서 Isaac은 JSON String 토픽만 쓰고, `elevator_sim_relay`가 형 변환을 맡는다.

모델 동작 (`amr_sim_bridge/elevator_model.py`):

| 동작 | 값 |
|---|---|
| 카 이동 | 사다리꼴 속도 프로파일, 1.0 m/s · 0.5 m/s² (짧은 구간은 삼각형) |
| 문 | 열림·닫힘 각 1.5 s, 열림 유지 **6~10 s 난수**(seed 고정) |
| 문 보호 | 문 평면에 물체가 있으면 다시 열린다. 미션 로직은 이 동작에 의존하지 않음 |
| 버튼 장치 | 명령 수신 0.2 s 뒤 버튼을 누르고 `BUTTON_ACK` 발송 |
| 홀 비콘 | 1 s마다 "나는 N층 비콘" (`PRESENCE`) 발송 |
| RF 수신 세기 | 같은 층 −50 dBm. 인접 층은 20 % 확률로 −82 dBm 누설. 카 안에서는 도착 층 문이 절반 이상 열렸을 때 −58 dBm |
| 문 상태 | 모델은 문 상태를 **알려주지 않는다** — AMR이 LiDAR로 직접 판별 |

고장 주입 항목 (`FAULT_DEFAULTS`):

| 키 | 의미 |
|---|---|
| `drop_ack_probability` / `drop_next_acks` | ACK 확률적 / 다음 N개 확정 누락 |
| `ack_extra_delay_s` | ACK 지연 |
| `duplicate_probability` / `reorder_probability` | 프레임 중복 / 순서 뒤바뀜 |
| `drop_presence_probability` | 홀 비콘 누락 |
| `wrong_floor_code` | 잘못된 층 비콘 추가 발송 |
| `stamp_lag_s` | 수신 시각 지연 (오래된 데이터) |
| `stop_at_floor_code` | 중간 층 정차 (다른 승객) |
| `hold_time_s` | 문 열림 유지 시간 고정 |
| `door_never_opens` | 도착해도 문이 안 열림 |
| `button_fails` | 장치 고장 (버튼도 안 누르고 응답도 없음) |

## 3. 노드 구성

`experiment.sh sim world:=building`에서 `elevator`가 자동으로 true가 되어 아래 노드가 추가된다
(`amr_stack/launch/experiment.launch.py`). 실차 bringup에서는 기본으로 꺼져 있다.

| 노드 | 패키지 | 역할 | 실차 공용 |
|---|---|---|---|
| `elevator_monitor` | `amr_elevator` | LiDAR 문 상태, 현재 층, 카 안/밖 판별 → `/elevator/door`, `/floor/state`, `/floor/events` | ✅ |
| `floor_mission_manager` | `amr_elevator` | 층간 미션 상태 머신 → 액션 `/floor_mission/navigate`, `/floor_mission/status` | ✅ |
| `elevator_sim_relay` | `amr_sim_bridge` | JSON ↔ `/elevator/command`·`/elevator/beacon` 변환 | ❌ (실차는 `elevator_can_link`) |
| `navigation_startup` (확장) | `amr_stack` | `/localization/suspend` — 카 안에서 위치 추정 일시 정지 | ✅ |
| `navigation_manager` (확장) | `amr_stack` | `/cmd_vel_maneuver` + `/navigation/maneuver_lease` — 전용 저속 기동 경로 | ✅ |

## 4. 핵심 알고리즘

### 4.1 문 상태 판별 — LiDAR (`door_detector.py`)

- 문 선분을 지나는 빔(문틀 가장자리 0.08 m 제외) 중에서, 거리가 문 평면보다 0.3 m 이상 먼 빔의 비율을 `open_ratio`로 계산한다.
- `open_ratio` ≥ 0.7이면 OPEN, ≤ 0.2이면 CLOSED, 그 사이는 OPENING/CLOSING이다. 연속 2스캔이 같아야 확정한다.
- 유효 빔이 10개 미만이거나 문까지 4 m 넘게 떨어져 있으면 `UNKNOWN`이다. 이때 미션은 기다린다.

### 4.2 층·영역 판별 (`floor_state.py`, `monitor_node.py`)

- **영역**: `OUTSIDE` ↔ `IN_ELEVATOR`. 문턱선 통과(위치), LiDAR 둘러싸임 비율, 문이 열려 있었는지를 함께 본다.
  문이 열리지 않았는데 문턱선을 넘으면 `UNKNOWN`과 `RULE_VIOLATION`이 되어 미션이 정지한다.
- **카 안 위치**: AMCL을 쓰지 않는다. 카 진입 직전에 검증된 `map→odom`(anchor)에 오도메트리를 더한다.
  로봇 전체가 카 안에 있으면 카 내벽 직선 맞춤(`car_matcher.py`, Gauss-Newton)으로 보정한다.
  층이 바뀌면 카 기준 상대 자세를 유지한 채 anchor를 새 층으로 옮긴다.
- **층 값**: 층은 카를 통해서만 바뀐다.
  - 출발: ACK 받은 목표 층이 있고 문이 닫혀 있음 → `TRANSIT`
  - 도착: 문 OPEN + 강한 홀 비콘(≥ −70 dBm) → `BEACON`
  - 하차 후 위치 추정 READY → `CONFIRMED`
  - 비콘 없이 문만 열리면 `ASSUMED`로 표시하고 **하차를 금지**한다.
- **비콘 필터**: 장치별로 다음을 처리한다.
  - 2 s 넘게 지난 신호는 폐기
  - 5 s 안에 같은 시퀀스가 오면 중복으로 보고 폐기
  - 8비트 시퀀스 창(128) 기준으로 과거 번호면 폐기
  - 카 이동 중 비콘은 무시
  - 밖에 있을 때 다른 층 비콘은 누설로 보고 `floor_conflict` 경고만 낸다

### 4.3 직선 기동 제어기 (`maneuver.py`)

1.1 m 출입구는 현재 Nav2 설정(inflation 1.6 m, robot_radius 0.44 m)으로는 경로가 나오지 않는다.
실차 복도 주행에 영향을 주지 않도록 `navigation.yaml`은 그대로 두고, 탑승·카 안·하차 구간만 전용 제어기가 맡는다.

- 순서: `ALIGN`(목표 방향 회전) → `DRIVE`(직진 + 방향 보정 + 감속) → `TURN`(최종 방향) → `DONE`
- 최고 0.35 m/s, 회전 0.5 rad/s, 위치 허용 오차 0.05 m, 전방 여유 0.15 m 미만이면 정지
- `reverse` 모드: 현재 방향을 유지하며 후진. 문이 닫힐 때 후퇴하는 데 쓴다
- AUTO 모드일 때만 명령을 낸다. MANUAL(RB)로 바뀌면 즉시 0을 내고 단계는 그대로 둔다

### 4.4 층간 미션 상태 머신 (`mission_node.py`)

```text
IDLE ─(같은 층)───────────────────────────────────────────────────────▶ NAV_TO_GOAL ─▶ SUCCEEDED
     └(다른 층)▶ NAV_TO_HALL ▶ CALL_ELEVATOR ▶ WAIT_DOOR_OPEN ▶ BOARDING ▶ IN_CAR_ALIGN
                ▶ SELECT_FLOOR ▶ RIDING ▶ SWITCH_MAP ▶ ALIGHTING ▶ RELOCALIZE ▶ NAV_TO_GOAL
     복구 분기: BOARDING ─(문 닫힘)▶ BACKING_OUT ▶ RELOCALIZE ▶ CALL_ELEVATOR
                ALIGHTING ─(문 닫힘)▶ RETURN_TO_CAR ▶ ALIGHTING
     어느 단계든 실패 ▶ FAILED_SAFE (정지 유지, 사유 코드 표시)
```

**미션 접수 조건**: 진행 중인 미션이 없을 것, `OUTSIDE`이고 현재 층이 `BEACON`/`CONFIRMED`일 것, 위치 추정이 READY일 것.
실차에서는 목표 층이 `real_enabled: true`여야 한다.

| 단계 | 동작 | 다음 단계 조건 | 이상 처리 (오류 코드) |
|---|---|---|---|
| NAV_TO_HALL | Nav2로 `hall_wait_pose` 주행 | Nav2 성공 | `NAV_TO_HALL_FAILED`, 위치 추정 45 s 미복구 시 `LOCALIZATION_LOST` |
| CALL_ELEVATOR | `HALL_CALL(위/아래)` 송신. 문이 이미 열려 있으면 건너뜀 | 홀 장치 ACK | 3 s마다 재송신, 3회 실패 → `NO_HALL_ACK` |
| WAIT_DOOR_OPEN | LiDAR 문 감시. OPEN이면 위치 추정 정지 + 기동 임대 + 카 "열림" 버튼 | 준비 완료 | 120 s → 재호출 1회 → `DOOR_NOT_OPENED`; 준비 실패 `PREPARE_FAILED` |
| BOARDING | 직선 기동으로 카 중앙까지 진입 | 카 안 판정 + 기동 TURN/DONE | 문턱 전 문 닫힘 → BACKING_OUT; 30 s → `BOARDING_TIMEOUT` |
| BACKING_OUT | 홀 대기 위치로 후진 → 재위치 → 재호출 | — | 3회 → `BOARDING_FAILED` |
| IN_CAR_ALIGN | 카 중앙 정렬 + 문 쪽으로 180° 회전 | 기동 완료 | 20 s → `ALIGN_TIMEOUT` |
| SELECT_FLOOR | `CAR_FLOOR(목표 층)` 송신 | 카 장치 ACK | 3 s × 3회 → `NO_CAR_ACK` |
| RIDING | 정지 유지, 문·층 감시 | 목표 층 + 문 OPEN + `BEACON`/`CONFIRMED` | 아래 표 참고 |
| SWITCH_MAP | `/map_server/load_map`으로 목표 층 지도 로드 | 로드 성공 | `MAP_SWITCH_FAILED` (카 안 정지) |
| ALIGHTING | 직선 기동으로 `hall_exit_pose`까지 하차 | 기동 완료 + `OUTSIDE` | 문턱 전 문 닫힘 → RETURN_TO_CAR (2회 초과 `ALIGHTING_FAILED`); 30 s → `ALIGHTING_TIMEOUT` |
| RETURN_TO_CAR | 카 중앙으로 후진, "열림" 버튼 | 문 OPEN → ALIGHTING | 15 s마다 버튼 재시도, 초과 시 `DOOR_NOT_REOPENED` |
| RELOCALIZE | 위치 추정 재개 → 현재 자세로 `/initialpose` → READY 대기 | READY | 30 s → TIM 재위치 1회 → `RELOCALIZE_FAILED` |
| NAV_TO_GOAL | Nav2로 호실 목표 주행 | 성공 → SUCCEEDED | `NAV_TO_GOAL_FAILED` |

RIDING 단계의 이상 처리:

| 상황 | 처리 |
|---|---|
| 목표 층에서 문은 열렸는데 비콘이 없음 (`ASSUMED`) | 하차 금지, "열림" 버튼 1회. 6 s 안에 확인되지 않으면 `ARRIVAL_UNCONFIRMED` (카 안 정지) |
| 다른 층에서 문 열림 (다른 승객) | 하차하지 않음. 카가 20 s 넘게 멈춰 있으면 목표 층 버튼 재송신, 3회 초과 시 `RIDE_STALLED` |
| 180 s 안에 도착 못 함 | `RIDE_TIMEOUT` |

**공통 안전 규칙**

- `/floor/state`가 3 s 이상 끊기면 `FLOOR_STATE_LOST`, 코드 예외가 나면 `INTERNAL_ERROR`. 둘 다 즉시 정지한다.
- 취소 시 카 밖이면 위치 추정을 재개한다. 카 안이면 위치 추정을 정지 상태로 둔다.
- `FAILED_SAFE`에서는 정지를 유지하고 운영자 조치(MANUAL 수동 이동 또는 `/floor/set_current`)를 기다린다.

## 5. 시나리오

### 5.1 실행 방법

```bash
export ROS_DOMAIN_ID=107                     # 107 이외 도메인에서는 실행 거부

# 수동 실행 (웹 UI에서 층 탭 → 호실 선택)
./scripts/experiment.sh sim world:=building start_floor:=4F

# 1회 시험
python3 scripts/run_floor_trial.py --output logs/elevator/try1 \
    --start-floor 4F --floor 3F --room 302 [--fault '{"drop_next_acks":2}'] \
    [--fault-at 'RIDING={"faults":{"stamp_lag_s":3.0}}'] [--bag]

# 전체 시나리오
python3 scripts/run_floor_suite.py --output logs/elevator/<새 폴더> [--scenario 4F_to_3F ...]
```

`run_floor_trial.py`는 한 번 실행할 때마다 다음 일을 한다.
1. Isaac과 스택을 새로 띄운다.
2. 초기화 후 `/floor_mission/navigate` 목표를 보낸다.
3. 고장을 주입한다. 시작 시점(`--fault`)이나 특정 단계에 들어갈 때(`--fault-at`) 넣을 수 있다.
4. `result.json`, `metrics.json`, `samples.json`을 저장한다.

### 5.2 판정 기준

| 기대 | 통과 조건 |
|---|---|
| `success` | 목적지 도착 (`passed`) + 충돌 0 |
| `safe_stop` | 미션 실패로 끝남 + 지정된 오류 코드 + 로봇 정지 + 충돌 0 |

수집 지표는 다음과 같다. 위치 기준은 모두 지상 진실이다.
- 도착 위치·방향 오차, 목표 층 높이 일치
- 전체·단계별 소요 시간
- 충돌 수, Nav2 복구 횟수, 재시도 카운터
- RTF
- 층·영역 판별과 실제 값의 일치율

### 5.3 시나리오 목록과 결과 (`logs/elevator/final_20261014/`, 11/11 통과)

| 시나리오 | 경로 | 주입 고장 | 기대 | 결과 | 도착 오차 m | 소요 s | 재시도 |
|---|---|---|---|---|---|---|---|
| 4F_to_3F | 4F → 3F 302 | 없음 | success | 도착 | 0.49 | 129.1 | — |
| 4F_to_5F | 4F → 5F 502 | 없음 | success | 도착 | 0.41 | 129.6 | — |
| 3F_to_5F | 3F → 5F 503 | 없음 | success | 도착 | 0.11 | 119.9 | — |
| ack_lost_twice | 3F → 5F 505 | ACK 2회 연속 누락 | success | 도착 | 0.17 | 98.5 | 버튼 2 |
| ack_loss_50 | 4F → 3F 305 | ACK 50 % 누락 | success | 도착 | 0.15 | 91.6 | — |
| ack_delay_4s | 4F → 5F 505 | ACK 4 s 지연 | success | 도착 | 0.15 | 94.3 | 버튼 1 |
| duplicate_reorder | 4F → 3F 305 | 중복·순서 뒤바뀜 각 30 % | success | 도착 | 0.06 | 89.6 | 버튼 1 |
| other_passenger_stop | 3F → 5F 505 | 4F 중간 정차 | success | 4F에서 내리지 않고 5F 도착 | 0.15 | 113.3 | — |
| arrival_beacon_lost | 4F → 3F 305 | RIDING부터 비콘 100 % 누락 | safe_stop | `ARRIVAL_UNCONFIRMED`, 카 안 정지 | (남은 4.54) | 64.0 | — |
| arrival_beacon_stale | 4F → 5F 505 | RIDING부터 수신 시각 3 s 지연 | safe_stop | `ARRIVAL_UNCONFIRMED`, 카 안 정지 | (남은 4.56) | 70.9 | — |
| button_device_dead | 4F → 3F 305 | 버튼 장치 고장 | safe_stop | `NO_HALL_ACK`, 홀 정지 | (남은 3.54) | 41.6 | 버튼 2 |

**요약 (성공 8건)**

- 도착 오차: 평균 0.21 m, 최대 0.49 m
- 소요 시간: 평균 108.2 s
- 엘리베이터 구간(호출~재위치): 평균 42.2 s
  - 탑승 7.9 s · 카 안 정렬 7.8 s · 하차 8.1 s · 재위치 4.0 s
- 충돌 0, Nav2 복구 0, RTF 최소 0.9999
- 층·영역 판별 일치율 95.0~100 %. 불일치는 탑승·하차 순간의 판정 지연이며 최장 3.0 s였다.
- 모든 시나리오에서 최종 층·영역 판별이 실제와 일치했다.

**회귀 시험** (`final_regression/`): arena 0.064 m, floor4 405호 0.14 m, 402호 0.167·0.143 m, 모두 충돌 0.

## 6. 한계와 남은 일

| 항목 | 내용 |
|---|---|
| 지도 | 3F·5F는 4F 복사본. 실측 지도로 바꾸면 층마다 평면 좌표 오프셋을 장면 생성기와 런타임에 넣어야 한다 |
| 실차 | `real_enabled: false` (3F·5F). 실차 층간 이동은 막혀 있다 |
| CAN | 0x210(명령)·0x310(비콘) ID와 페이로드는 후보 상태. STM32 펌웨어 작업 필요 (`elevator_protocol.py` 한 곳만 수정하면 됨) |
| 카 "열림" 버튼 | 장치는 계획 단계. 버튼이 없어도 후퇴·재호출 규칙으로 동작한다 |
| 수치 | 층고 3.6 m, 카 속도 1.0 m/s, 센서 노이즈 0은 임시값이다 |
| 긴 복도 끝 호실 | 302·502처럼 x ≈ 33.7 m인 호실은 하차 후 주행 중 위치 오차가 0.53~0.76 m로 단일 층 시험(0.19~0.27 m)보다 컸다. 하차 직후 재위치 오차는 약 2 cm여서, 엘리베이터 구간이 아닌 4층 복도 위치 추정 문제로 보인다. 반복 측정이 필요하다 |
| 시작 경합 | 단일 층 월드에서도 Isaac·Nav2 시작 단계에서 간헐적으로 타임아웃이 난다. 재실행하면 통과한다 |

## 7. 관련 파일

| 분류 | 경로 |
|---|---|
| 층 설정 | `src/amr_system/amr_stack/config/floors.yaml`, 파서 `amr_stack/floor_config.py` |
| 공용 알고리즘 | `src/amr_system/amr_elevator/amr_elevator/` — `mission_node.py`, `monitor_node.py`, `floor_state.py`, `door_detector.py`, `car_matcher.py`, `maneuver.py`, `geometry.py` |
| 메시지 | `src/amr_system/amr_elevator_interfaces/` |
| 시뮬레이션 모델 | `src/amr_system/amr_sim_bridge/amr_sim_bridge/elevator_model.py`, `elevator_node.py`(relay) |
| Isaac 런타임 | `sim/scripts/elevator_runtime.py`, `sim/scripts/run_ros_sim.py` |
| 장면 | `sim/tools/build_building_scene.py` → `sim/building_G.usda`, `sim/layers/*.usda`, `sim/config/building.json` |
| 런치 | `src/amr_system/amr_stack/launch/experiment.launch.py`, `scripts/experiment.sh` |
| 시험 | `scripts/run_floor_trial.py`, `scripts/run_floor_suite.py`, `sim/config/scenarios.json` |
| 실차 CAN 경로 (미사용) | `src/amr_system/amr_can_bridge/amr_can_bridge/elevator_can_link.py`, `elevator_protocol.py` |
