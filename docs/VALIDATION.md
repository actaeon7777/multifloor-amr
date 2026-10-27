# 검증

시험 명령, 판정 기준, 현장 점검 항목을 모은다. 과거 실행 결과는 [실험 기록](EXPERIMENTS.md)에 날짜별로 남긴다.

## 1. 검증 단계

| 단계 | 확인 내용 | 실물 장치 사용 |
|---|---|---|
| 단위·HTTP | 좌표·요청 검증·모델·상태 신선도·원격 규약 | 없음 |
| 격리 ROS | 실제 노드를 테스트 전용 도메인에서 합성 입력으로 실행 | 없음 (CAN은 dry-run 또는 가짜 버스) |
| 브라우저 | 지도 조작·모바일 레이아웃·상태/전원/BMS 표시·원격 UI | 저장 지도와 fixture |
| 시뮬레이션 | Isaac Sim 폐루프 주행·시나리오 | 없음 |
| 장치 읽기 | OS·CPU·CAN 설정·Tailscale 주소 | 상태 읽기만 |
| 현장 주행 | 실제 조이스틱·위치 확인·목표 주행·정지·통신 단절 | 별도 현장 검증 |

단위 시험의 전압·BMS 수치와 가짜 Nav2 성공은 실측 배터리 상태나 실제 주행 성공을 뜻하지 않는다.
소프트웨어 fixture, 시뮬레이션, 실제 센서 수신, 실차 주행, 실제 호스트 배포의 판정을 구분해 기록한다.

## 2. 설정 기준값

| 항목 | 값 | 위치 |
|---|---|---|
| 실물 / 시뮬레이션 ROS 도메인 | `7` / `107` | `scripts/experiment.sh` |
| 로컬 웹 / 중앙 웹 포트 | `8080` / `8081` | `web_ui.launch.py`, `Control_Server` |
| CAN 비트레이트 | `500000 bit/s` | `deploy/jetson/can0.service` |
| Xbox 장치 이름 | `Xbox Series X Controller` | `joy_teleop.yaml` |
| 바퀴 지름 / 간격 / 회전 속도 제한 | `0.172 m` / `0.529 m` / `1.8 rev/s` | `control_mux.yaml`, URDF, `firmware/stm32/main.c` |
| AMCL 초기 / 주행 빔 수 | `1080` / `60` | `localization_startup.yaml` |
| LiDAR 최대 검증 거리 | `30.0 m` | `localization.yaml`, `localization_startup.yaml` |
| 공용 차량 상태 발행 주기 / 수신 만료 | `10 Hz` / `0.5 s` | `amr_vehicle_status` |
| 깊이 카메라 깊이 / 컬러 프로파일 | `320×200 10 Hz`(decimation 2 → 점군 160×100) / `640×360 5 fps` | `camera.launch.py` |
| 깊이 장애물 표시 높이 / 거리, 시야 밖 기억, 감시 허용 지연 | `0.06~1.0 m` / `3.0 m`, `10 s`, `1.0 s` | `navigation.yaml` `depth_layer`, `time_health` `depth_max_age` |
| 카메라 스냅숏 만료 / 중계 크기 상한 | `3 s` / `300 KiB` | `amr_web_ui/server.py`, `amr_remote_bridge/common.py` |

## 3. ROS 도메인 배정

| 도메인 | 용도 |
|---|---|
| 7 | 실차 |
| 107 | 시뮬레이션 |
| 142 | 오프라인 전역 경로 비교 |
| 173 | 수동 제어 파이프라인 |
| 174 | CAN 송신 브리지 |
| 184 | CAN 수신 |
| 187 | EKF |
| 189 | 지도·AMCL |
| 190 / 191 / 192 | navigation_manager / Nav2 파이프라인 / nav2 플러그인 |
| 193 | 웹 UI, 엘리베이터 신호·내비·미션 |
| 194 | 위치 시작 감독기, 시간 복구 |
| 195 | 공용 차량 상태, 엘리베이터 모니터 |
| 196 | TIM 연동 |
| 197 | 깊이 costmap, 합성 낮은 장애물 |

테스트 도메인은 실제 로봇 운용에 쓰지 않는다. 격리 ROS 시험에는 `ROS_LOCALHOST_ONLY=1`을 붙인다.

## 4. 소프트웨어 시험

### 준비

```bash
source /opt/ros/jazzy/setup.bash
CMAKE_BUILD_PARALLEL_LEVEL=2 MAKEFLAGS=-j2 \
  colcon build --base-paths src --symlink-install --executor sequential \
  --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3 -DCMAKE_BUILD_TYPE=Release
source scripts/env.sh
```

`scripts/build_ros.sh`는 위 제한(`-j2`, `--executor sequential`, `nice`)을 기본으로 적용하고 빌드 뒤 단위 시험과 격리 ROS 통합 시험(CAN dry-run, 도메인 160·localhost)까지 실행한다. 실차 스택이 떠 있으면 시작하지 않는다. `--no-rosdep`, `--build-only`, `--packages-select <pkg…>`로 줄일 수 있다. 시험 로그와 JUnit은 새 폴더에 보관한다.

### 단위·HTTP

```bash
python3 -m pytest -q tests \
  src/amr_system/amr_control/test/test_manual_gate.py \
  src/amr_system/amr_control/test/test_control_gate.py \
  src/amr_system/amr_control/test/test_wheel_limits.py \
  src/amr_system/amr_control/test/test_operator_hold.py \
  src/amr_system/amr_can_bridge/test \
  src/amr_system/amr_vehicle_status/test/test_model.py \
  src/amr_system/amr_web_ui/test \
  src/amr_system/amr_remote_bridge/test \
  src/amr_system/amr_elevator/test \
  src/amr_system/amr_stack/test/test_floor_config.py
```

- `test_wheel_limits.py`는 호스트 GCC로 `firmware/stm32/main.c`의 변환 함수를 컴파일해 상위 제한과 함께 검증한다(GCC 없으면 skip). 보드 빌드·업로드는 하지 않는다.
- `test_can_sender.py`의 python-can 메시지 검사에는 `python3-can`이 필요하다. 가짜 버스라 물리 CAN에 송신하지 않는다.
- `AMR_RUN_*` 환경변수가 없는 ROS 시험 모듈은 자동으로 skip된다.

### 격리 ROS

전체 실행기는 아래 스위트를 각자의 도메인에서 순서대로 실행한다. 실물 도메인 7과 시뮬레이션 107은 쓰지 않는다.

```bash
python3 scripts/test_ros_integration.py
```

| 스위트 | 시험 파일 | 환경변수 | 도메인 | 확인 내용 |
|---|---|---|---|---|
| manual | `amr_control/test/test_manual_pipeline.py` | `AMR_RUN_ROS_TESTS` | 173 | dry-run bringup + 가상 `/joy`: 수동 회귀, 연결된 조이스틱에서 AUTO 유지·LB 끼어들기·해제 후 AUTO, 타임아웃, `scripts/auto_turn_test.py` 실제 실행과 CAN 미리보기. E-stop 뒤 캐시 폐기, LB 재입력 필요 |
| can_bridge | `amr_can_bridge/test/test_bridge_ros.py` | `AMR_RUN_CAN_TESTS` | 174 | 독립 브리지 payload·watchdog, E-stop 인터록(0x202: 정지 중 송신 없음, 해제 뒤 자동 ARMING 영속도, gate 닫힘 재무장, 수신 끊김), `tx_rate` 하한 |
| telemetry | `amr_can_bridge/test/test_telemetry_ros.py` | `AMR_RUN_TELEMETRY_TESTS` | 184 | 실제 CAN 없이 ROS 메시지 |
| ekf | `amr_stack/test/test_ekf_ros.py` | `AMR_RUN_EKF_TESTS` | 187 | 모순된 입력 제외, 전진·gyro 응답, 평면 제약, TF 연결 |
| localization | `amr_stack/test/test_localization_ros.py` | `AMR_RUN_LOCALIZATION_TESTS` | 189 | 저장 지도 해상도·원점·late subscriber, map server/AMCL 활성 |
| navigation_manager | `amr_stack/test/test_navigation_manager_ros.py` | `AMR_RUN_NAVIGATION_TESTS` | 190 | 목표 취소, LB·E-stop 중단(재개 없음), 기동 임대 |
| nav2_pipeline | `amr_stack/test/test_navigation_pipeline_ros.py` | `AMR_RUN_NAV2_INTEGRATION` | 191 | 실제 Smac 2D/DWB/복구 + mux + CAN dry-run |
| web_ui | `amr_web_ui/test/test_web_ui_ros.py` | `AMR_RUN_WEB_UI_TESTS` | 193 | HTTP → 목표 관리기 → 가짜 Nav2, 초기 위치·relocalize, 공용 상태 구독 |
| vehicle_status | `amr_vehicle_status/test/test_node_ros.py` | `AMR_RUN_VEHICLE_STATUS_TESTS` | 195 | 미연동 플래그, 만료, 시계 불연속, 명령 publisher 부재 |
| navigation_startup | `amr_stack/test/test_navigation_startup_ros.py` | `AMR_RUN_STARTUP_TESTS` | 194 | 초기 위치 검증, 빔 전환, suspend |
| tim_localization | `amr_stack/test/test_tim_integration_ros.py` | `AMR_RUN_TIM_TESTS` | 196 | TIM → AMCL 합성 센서 연동 |
| depth_costmap | `amr_stack/test/test_depth_costmap_ros.py` | `AMR_RUN_DEPTH_COSTMAP_TESTS` | 197 | 실제 STVL: 10·12 cm 박스·바닥 제외, 시야 안 사라진 박스 ~1.4초 만료·짧은 누락은 유지, 사각대 8 cm 물체는 복구(2 m 보존) 뒤에도 남았다가 10초 뒤 만료, 전체 지우기. 실제 Smac: 깊이 점군만으로 기존 경로 무효화·우회 경로 생성; `ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST`도 지정 |

러너에 없는 시험:

```bash
ROS_DOMAIN_ID=192 ROS_LOCALHOST_ONLY=1 colcon test --packages-select amr_nav2_plugins --return-code-on-test-failure
ROS_DOMAIN_ID=194 ROS_LOCALHOST_ONLY=1 AMR_RUN_TIME_TESTS=1 python3 -m pytest -q tests/test_time_recovery_ros.py
ROS_DOMAIN_ID=193 ROS_LOCALHOST_ONLY=1 AMR_RUN_ELEVATOR_TESTS=1 \
  python3 -m pytest -q src/amr_system/amr_sim_bridge/test/test_elevator_ros.py
ROS_DOMAIN_ID=193 ROS_LOCALHOST_ONLY=1 AMR_RUN_ELEVATOR_NAV_TESTS=1 \
  python3 -m pytest -q src/amr_system/amr_elevator/test/test_elevator_nav_ros.py
ROS_DOMAIN_ID=193 ROS_LOCALHOST_ONLY=1 AMR_RUN_ELEVATOR_MISSION_TESTS=1 \
  python3 -m pytest -q src/amr_system/amr_elevator/test/test_mission_ros.py
ROS_DOMAIN_ID=195 ROS_LOCALHOST_ONLY=1 AMR_RUN_ELEVATOR_MONITOR_TESTS=1 \
  python3 -m pytest -q src/amr_system/amr_elevator/test/test_monitor_ros.py
```

단일 스위트를 직접 돌릴 때는 표의 도메인·환경변수를 붙인다. 예:
`ROS_DOMAIN_ID=173 ROS_LOCALHOST_ONLY=1 AMR_RUN_ROS_TESTS=1 python3 -m pytest -q src/amr_system/amr_control/test/test_manual_pipeline.py`.

### 브라우저와 JavaScript

```bash
python3 src/amr_system/amr_web_ui/test/browser_check.py
python3 Control_Server/scripts/browser_check.py
gjs src/amr_system/amr_web_ui/test/test_geometry.js src/amr_system/amr_web_ui/web/geometry.js
gjs src/amr_system/amr_web_ui/test/test_tracking_status.js src/amr_system/amr_web_ui/web/app.js
```

Chromium에서 실제 터치·드래그·pan·pinch·취소와 1440/1366/900/768/390 px 레이아웃을 확인한다. 전원·BMS·연결 끊김 표시는 fixture로 확인한다.

### 중앙 관제 서버 (FastAPI)

서버 PC에서 해당 CPU·Python으로 만든 가상환경으로 실행한다. 다른 장치에서 복사한 `.venv`를 쓰지 않는다.

```bash
cd Control_Server
.venv/bin/python -m pytest -q
```

ROS가 필요 없다. 웹 UI·중계 코드는 저장소의 `src/amr_system/`을 직접 읽는다. 실제 배포·프로세스 재시작·Tailscale 왕복은 별도 판정이다.

## 5. 시뮬레이션 시험

Isaac Sim이 있는 PC에서 도메인 107로 실행한다. 명령과 판정 기준은 [시뮬레이션](SIMULATION.md#5-시나리오와-자동-시험)과
[층간 이동](ELEVATOR.md#10-시나리오)에 있다.

```bash
export ROS_DOMAIN_ID=107
/usr/bin/python3 scripts/run_sim_trial.py --world arena --output reports/trials/regression_arena
/usr/bin/python3 scripts/run_floor_suite.py --output logs/elevator/suite_$(date +%Y%m%d_%H%M%S)
```

층간 이동 등 큰 변경 뒤에는 arena와 floor4 회귀 주행(`run_sim_trial.py`)을 함께 돌린다.

## 6. 장치 읽기와 현장 판정

```bash
cat /etc/os-release; uname -m
ip -details link show can0
tailscale ip -4
source scripts/env.sh && export ROS_DOMAIN_ID=7
ros2 param get /cmd_vel_to_can dry_run
ros2 topic echo /vehicle/status --once
ros2 topic echo /system/time_health --once --qos-durability transient_local
ros2 topic echo /localization/status --once --qos-durability transient_local
```

| 현장 항목 | 통과 기준 |
|---|---|
| CAN·센서 | CAN UP/500,000 bit/s, scan·wheel·IMU·EKF 지속 수신·유효 시각 |
| 조이스틱 | 실제 이름 일치, LB·축 매핑과 해제·단절 처리, 전원 ON/OFF → AUTO 유지, LB → 즉시 MANUAL·목표 중단, LB 해제 → 0.5초 뒤 AUTO·재개 없음, 재연결 후 `/joy` 재수신 ([확인 순서](REAL_VEHICLE.md#실제-컨트롤러-확인-순서)) |
| 위치 | 실제 지도·위치·방향·LiDAR 정합, 감독기 `READY`, 시간 `healthy=true` / `latched=false` |
| 공용 상태 | 10 Hz, 유효 source와 신선도, 미연동 항목이 정상값으로 표시되지 않음 |
| 실제 송신 | 점검 dry-run=True와 실제 운용=False를 각 실행에서 구분 |
| 주행·정지 | 목표 접수·출발·도착(조이스틱 연결 무관), LB 끼어들기·목표 취소·실제 정지 |
| 원격 | 실제 호스트 최신 배포, 인증·상태·목표·취소 왕복, 단절 뒤 새 명령 차단·재전송 없음, 층 탭·다른 층 목표 |
| BMS·드라이버 | 실제 연결 후 값·단위·유효 플래그·입력 중단 판정 |
| 펌웨어·구동 설정 | 장치 readback으로 STM32/ODrive 버전과 실제 제한값 |
| E-stop | [ESTOP_CAN_PROTOCOL.md §10](ESTOP_CAN_PROTOCOL.md#10-검증-현황-2026-10-21) 1–7번: `0x202` 주기, 누름 → `ESTOP`·`0x201` 차단, 떼면 승인 없이 `(0,0)` → `READY` → 새 명령으로만 이동(이전 목표 재출발 없음), 짧은 누름, 300 ms 공백 뒤 재무장, 수신 끊김 정지. 추가: LB·스틱을 누른 채 지난 E-stop 뒤 READY에서 움직이지 않음, 디버거 MCU 리셋 뒤 목표 중단·재무장, 회전 중 E-stop 시 래치 동안 `/odometry/filtered` yaw 변화 기록 |
| 깊이 카메라 (`start_camera:=true`) | 스택 내린 뒤 기준 `top -b -d 5 -n 12`(controller_server·time_health·web_ui·load avg) → 카메라만 `ros2 launch amr_stack camera.launch.py start_camera:=true`: `ros2 topic hz /camera/depth/points` 10±1 Hz, `/camera/color/image_raw/compressed` ≥2 Hz, 드라이버 CPU ≤35 % → 전체 스택: controller_server +≤30 %, time_health·web_ui 각 +≤3 %, load avg <6, 내부 갱신 전역 2 Hz·지역 10 Hz, DWB 20 Hz 유지(시각화 costmap 발행 전역 1 Hz·지역 2 Hz와 구분). 실제 10 cm 박스를 차체 앞 0.5·1·2 m에 놓아 양쪽 costmap 표시와 제거를 확인. 저속 주행에서는 점군 최초 관측·지도 반영·우회 경로 변경·감속 시작·최종 간격을 bag으로 기록하고, 사각 진입 시 장애물 표시가 유지되는지 확인. 전역 depth 추가에 따른 planner_server CPU 증분도 측정. USB 분리 → 1초 안에 `/system/time_health` `depth: missing or stale`, costmap "not current" 없음, 재연결 후 회복. 층간 이동 화면 카메라 카드 0.5초 갱신, 중앙 관제 `GET /robots/amr-01/api/camera.jpg` ≤300 KiB. 결과를 REAL_VEHICLE §9b에 기록 |
| 엘리베이터 RF | `candump can0,310:7FE`로 0x310·0x311 수신, 카 PRESENCE에도 `invalid` 0, 4F 홀 호출 → `COMPLETED`·BUTTON_ACK, DOOR_OPEN repeat 10 실제 반복, 통과층 정차 뒤 계속 탑승, 문턱 E-stop 시 문 유지, 웹 "엘리베이터 RF" 패널 |

## 7. 기록 원칙

1. 새 시험 결과는 [실험 기록](EXPERIMENTS.md)에 날짜·범위·통과 수와 함께 남기고, 원자료는 별도 날짜 폴더에 보관한다.
2. PASS에는 실행 환경·코드 커밋·명령·로그 또는 JUnit 경로를 남긴다.
3. 이번에 실행하지 않은 시험의 이전 PASS를 현재 PASS로 적지 않는다.
4. 시뮬레이션 수치를 실차 성능으로, fixture 결과를 실측으로 바꿔 쓰지 않는다.
