# multifloor-amr

G동 실내를 자율 주행하고 엘리베이터로 층간 이동하는 **차동구동 AMR**의 ROS 2 Jazzy 작업공간이다.
실차(Jetson + STM32 + ODrive)와 Isaac Sim 6.1.0 시뮬레이션이 같은 제어 스택을 쓴다.
목적지는 터치 키오스크·웹 UI·중앙 관제 콘솔에서 지정한다.

> 이 문서는 전체 구조 요약이다. 실행 절차와 세부 설정은 [문서](#8-문서) 표의 각 문서를 따른다.

## 1. 주요 기능

| 기능 | 내용 |
|---|---|
| 자율 주행 | 3F·4F·5F 실측 지도, AMCL + TIM 전역 재위치, Nav2(Smac 2D + RotationShim/DWB) |
| 층간 이동 | 엘리베이터 버튼은 로봇 안내에 따라 사람이 누름. 문 상태는 LiDAR, 도착 층은 카메라 층인식과 키오스크 "도착했나요?" 확인으로 판단 |
| 배송 키오스크 | 1920×1280 터치 화면. 호실 선택, 관리 모드(PIN 선택) |
| 순찰 | 4F 고정 경로 순찰, 코스트맵과 정적 지도 비교로 움직임 감지·스냅숏 |
| 중앙 관제 | 경량 콘솔(상태·정지·복귀·순찰), 감지 기록, 수령자 Slack DM |
| 안전 | 하드웨어 E-stop(CAN 0x202), 조이스틱 LB 끼어들기, 바퀴 속도 상한, 엘리베이터 승객·공간 확인 |
| 음성 안내 | HDMI 스피커로 배송·정체·E-stop·엘리베이터 안내, 주행 알림음 |

## 2. 빠른 실행

### 실차 (Jetson)

바탕화면의 **"AMR 시작"** 아이콘을 누르면 차량 스택·웹 UI·키오스크가 함께 뜬다. 끌 때는 **"AMR 종료"**.
터미널로 띄우는 방법, 중앙 관제 연결, 로그 확인은 [실행법.md](실행법.md)와 [docs/README.md](docs/README.md).

```bash
# 터미널 방식: 차량 스택 (도메인 7)
./scripts/experiment.sh real start_camera:=true
# 웹 UI (http://127.0.0.1:8080, 키오스크는 /kiosk)
source scripts/env.sh && export ROS_DOMAIN_ID=7
ros2 launch amr_web_ui web_ui.launch.py use_sim_time:=false host:=127.0.0.1 port:=8080
```

층간 이동 전에는 웹에서 현재 층을 지정해야 한다.

### 시뮬레이션 (PC)

최초 준비(ROS 패키지, Isaac 연결, 장면 생성)는 [docs/SIMULATION.md](docs/SIMULATION.md#1-준비).

```bash
./scripts/experiment.sh sim                         # 4층 + 전체 스택 (도메인 107)
./scripts/experiment.sh sim world:=building         # 3F·4F·5F + 엘리베이터
ROS_DOMAIN_ID=107 ros2 launch amr_web_ui web_ui.launch.py use_sim_time:=true port:=8080
```

시험 자동화는 `scripts/run_sim_trial.py`(단일 층), `scripts/run_floor_suite.py`(층간 이동).
전체 ROS 프로세스 정리는 `ros2kill`(`--dry-run`으로 미리보기).

### 빌드·시험

```bash
./scripts/build_ros.sh      # -j2·nice로 빌드 후 단위 + 격리 ROS 시험 (실차 스택이 떠 있으면 거부)
```

시험 종류와 명령은 [docs/VALIDATION.md](docs/VALIDATION.md).

## 3. 하드웨어

| 구분 | 내용 |
|---|---|
| 차량 컴퓨터 | NVIDIA Jetson, JetPack 7.2.1 / Ubuntu 24.04 / arm64 |
| 하위 제어기 | STM32 NUCLEO-F446RE → ODrive S1 2축 (node 0 = 좌, node 1 = 우). 펌웨어 원본은 별도 STM32 프로젝트 |
| 통신 | SocketCAN `can0`, 500 kbit/s (`deploy/jetson/can0.service`) |
| LiDAR | SLAMTEC S2 (`sllidar_ros2`, frame `laser`) |
| 깊이 카메라 | Orbbec Gemini 2 (`orbbec_camera`), 낮은 장애물과 층인식용 컬러 영상 |
| IMU | STM32를 거쳐 CAN으로 수신 (0x301 가속도, 0x302 자이로) |
| 화면·소리 | 터치 디스플레이(키오스크), HDMI 스피커 |
| 조작기 | Xbox Series X Controller (Bluetooth) |
| 배터리 | LFP 8S |
| 기구 | 바퀴 지름 0.172 m, 윤거 0.529 m, 바퀴 상한 1.8 rev/s, footprint 0.656 × 0.574 m, 질량 18 kg |
| 로봇 모델 | v119 URDF/Xacro + STL (`src/amr_description`) |
| 시뮬레이션 PC | Ubuntu 24.04 x86_64, Isaac Sim 6.1.0 |

## 4. 시스템 구조

### 명령 경로

```text
joy_node → joy_teleop → /cmd_vel_manual ───────────────┐
Nav2 / 엘리베이터 기동(/cmd_vel_maneuver)               ├→ control_mux → /cmd_vel → cmd_vel_to_can → CAN 0x201 → STM32 → ODrive
   → navigation_manager → /cmd_vel_auto ───────────────┘   (바퀴 속도 제한)
STM32 E-stop 상태 CAN 0x202 → /estop/status → 목표 중단, 해제 후 주행 준비
```

- **기본은 AUTO.** 조이스틱 연결·끊김은 모드를 바꾸지 않는다.
- **LB를 누르면 MANUAL**: 즉시 정지하고 목표를 중단한다. LB를 누른 채 스틱으로 수동 주행하고,
  놓은 뒤 0.5초 지나면 AUTO로 돌아간다. 중단된 목표는 재개하지 않는다.
- `control_mux`는 바퀴 속도 상한 `U = π·d·rev_s`를 넘는 명령을 `(v, ω)` 같은 비율로 줄여 회전 반경을 유지한다.
  통신 타임아웃·모터 오류 정지는 STM32가 맡는다.

자세한 내용은 [docs/REAL_VEHICLE.md](docs/REAL_VEHICLE.md).

### 위치 추정과 센서

```text
CAN 피드백 → can_telemetry → /wheel/odometry, /joint_states, /imu/data_raw
                                   ↓
                     robot_localization EKF (odom → base_footprint, 2D)
S2 LiDAR → /scan → AMCL (map → odom) ← TIM 전역 초기화 / 웹 초기 위치
Gemini 2 → depth_persistence_filter → 코스트맵 depth 층(STVL)
```

- 초기 위치는 `amr_tim_localization`이 후보를 내고 `navigation_startup`이 지도·스캔 정렬을 검증한 뒤 AMCL에 넘긴다.
- READY 이후 품질 저하는 `tracking_warning`으로만 표시하고 목표를 취소하지 않는다.

자세한 내용은 [docs/LOCALIZATION.md](docs/LOCALIZATION.md).

### 층간 이동

로봇은 엘리베이터를 물리적으로 조작하지 않는다(승강기협회 가이드라인 SPS-B-KOLA-0001-7700:2026 기준).

1. 홀 앞에서 호출 안내 → 사람이 버튼을 누름
2. 문 열림(LiDAR) → 탑승 통로·카 안 공간 확인 → 탑승, 카 안 180° 회전
3. 문이 열릴 때마다 카메라 층인식(`amr_floor_vision`, DINOv3 특징 kNN)이 층을 추정.
   확신이 높으면 자동 확정, 아니면 키오스크에서 "X층 맞나요?"로 사람이 확인
4. 목적층이면 하차 → 지도 전환·위치 재검증 → 목적지 주행

층 설정은 `src/amr_system/amr_stack/config/floors.yaml`, 탑승 튜닝은 `amr_elevator/config/elevator_tuning.yaml`.
설계·알고리즘·실차 기록은 [docs/ELEVATOR.md](docs/ELEVATOR.md).

### 시뮬레이션

Isaac Sim이 실차 드라이버 대신 같은 토픽을 발행한다(`amr_sim_bridge`). 브리지는 TF를 발행하지 않으며
TF는 실차와 같이 `robot_state_publisher`·EKF·AMCL이 맡는다. LiDAR는 C++ 배치 레이캐스트(`sim/native/`)가 기본이다.

| 방향 | 토픽 |
|---|---|
| 제어기 → Isaac | `/cmd_vel` |
| Isaac → 제어기 | `/clock`, `/wheel/odometry`, `/joint_states`, `/imu/data_raw`, `/scan`, `/sim/ground_truth`, `/sim/status` |

### ROS 도메인

| 용도 | `ROS_DOMAIN_ID` |
|---|---|
| 실차 | 7 |
| 시뮬레이션 | 107 |
| 격리 시험·벤치마크 | 142, 160, 173~198 (배정표는 [docs/VALIDATION.md](docs/VALIDATION.md#3-ros-도메인-배정)) |

## 5. ROS 패키지 (`src/`)

| 패키지 | 역할 |
|---|---|
| `amr_stack` | bringup·experiment·localization·navigation·camera launch, 설정 YAML, 지도. `navigation_manager`, `navigation_startup`, `joy_autoselect`, `camera_settings` |
| `amr_control` | `control_mux`(수동·자율 선택, 바퀴 속도 제한), `time_health`(센서·시간 감시) |
| `amr_can_bridge` | `cmd_vel_to_can`(송신), `can_telemetry`(휠 오도메트리·관절·IMU·E-stop 수신) |
| `amr_elevator` / `amr_elevator_interfaces` | 층간 이동: 문·층·안밖 판별(`elevator_monitor`), 미션(`floor_mission_manager`), 탑승 기록(`elevator_recorder`), 음성 안내(`announcement_player`) |
| `amr_floor_vision` | 카메라 층인식과 도착 사진 데이터셋. 별도 venv(`~/venvs/floorvision`)에서 실행 |
| `amr_patrol` | 고정 경로 순찰(`patrol_manager`), 움직임 감지(`motion_detector`) |
| `amr_depth_filter` | 깊이 점군에서 여러 프레임 이어지지 않는 점 제거 (C++) |
| `amr_web_ui` | 키오스크·관리자 웹 UI와 ROS 2 HTTP 브리지 (포트 8080) |
| `amr_remote_bridge` | 중앙 관제 서버와 상태·이벤트·명령 중계, Slack DM |
| `amr_vehicle_status` / `amr_vehicle_interfaces` | 공용 차량 상태(10 Hz). bringup 기본 꺼짐(`start_vehicle_status:=true`) |
| `amr_tim_localization` / `amr_localization_interfaces` | 전역 재위치 추정 서비스 |
| `amr_nav2_plugins` | 바퀴 속도 실현 가능성을 보는 DWB critic |
| `amr_rviz_plugins` | RViz TF 데드락 회피 플러그인 |
| `amr_sim_bridge` | Isaac ↔ 실차 인터페이스 어댑터, 시뮬레이션 엘리베이터 모델 |
| `amr_description` | v119 URDF/Xacro, 물리 URDF, 메시 |
| `teleop_tools` (`joy_teleop`, `teleop_tools_msgs`) | 조이스틱 입력 변환 (F1TENTH 소스 복사) |
| `sllidar_ros2` | SLAMTEC LiDAR 드라이버 |

## 6. 디렉터리

| 경로 | 내용 |
|---|---|
| `src/` | ROS 2 패키지 (`colcon build --base-paths src`) |
| `scripts/` | 실행(`experiment.sh`), 빌드(`build_ros.sh`), 시험·벤치마크·분석, 아이콘 실행기(`launcher/`), 키오스크(`kiosk/`) |
| `Control_Server/` | 중앙 관제 서버 (FastAPI, 포트 8081). 웹 UI·중계 코드는 `src/amr_system/`을 그대로 씀 |
| `sim/` | Isaac Sim 장면(USD), 설정, 실행·검증 스크립트, C++ LiDAR |
| `rl/` | 로컬 주행 정책 강화학습 실험 (Isaac Lab, colcon 대상 아님) |
| `docs/` | 운영·설계·검증 문서 |
| `firmware/stm32/` | STM32 `main.c` 사본. 주행 변환 함수를 `amr_control` 시험이 직접 컴파일해 검증 |
| `deploy/jetson/` | Jetson 시스템 설정 (`can0.service`) |
| `tools/` | 음성 안내 wav 생성(`tts/`) 등 보조 도구 |
| `tests/` | 작업공간 도구 단위 시험 |
| 루트 `*.sh` | Isaac Sim 진입점 (`env.sh`, `isaac-sim.sh`, `python.sh`, `run_amr.sh`, `build_scene.sh`) |

`bags/`, `datasets/`, `diagnostics/`, `logs/`, `build/`, `install/`는 git에 넣지 않는다.

### 배포

Jetson과 관제 서버 PC 모두 이 저장소를 clone해 두고 브랜치나 태그로 버전을 고른다. 빌드 결과물은 옮기지 않는다.

```bash
git fetch && git checkout <태그 또는 브랜치> && ./scripts/build_ros.sh
```

`main`에는 직접 push하지 않고 작업 브랜치 → PR로 합친다.

## 7. 현재 상태 (2026-11-03)

- 실차: 4층 배송 주행, 3F·4F·5F 사이 엘리베이터 층간 이동(사람 버튼·카메라 층인식·도착 확인) 시험 통과.
- 실차 미검증: 순찰 모드와 움직임 감지.
- 시뮬레이션 센서·물리 모델은 실물 보정 전이다. 제동거리·절대 위치 정확도를 시뮬레이션 수치로 확정하지 않는다.
- 날짜별 실험·튜닝 기록은 [docs/EXPERIMENTS.md](docs/EXPERIMENTS.md).

### 알려진 이슈

- 카메라 층인식 참조 사진이 없는 층(1·2·6·7층)에서 정차하면 자동 판정이 틀릴 수 있다. 참조가 있는 층 사이 이동만 자동 확정한다.
- 깊이 카메라는 검은 낮은 물체를 약 1.5 m 안에서만 본다.
- 조이스틱이 꺼졌거나 Bluetooth 범위 밖이면 LB로 끼어들 수 없다. 이때는 웹 **주행 목표 취소**나 하드웨어 E-stop을 쓴다.
- LB를 놓은 0.5초 뒤부터 중앙 관제 목표를 받으므로, 수동 조작 직후 사람 옆에서 원격 목표로 출발할 수 있다.
- AMR 자체 패키지 라이선스는 미정이다.

## 8. 문서

| 문서 | 내용 |
|---|---|
| [실행법.md](실행법.md) | 실차 실행 요약 (아이콘, 터미널, 키오스크, 주행 기록) |
| [docs/README.md](docs/README.md) | 실운용 상세 (웹 접속, 조작, 중앙 관제, 종료) |
| [docs/REAL_VEHICLE.md](docs/REAL_VEHICLE.md) | 실차 제어·센서 레퍼런스 (조이스틱, control_mux, CAN, E-stop, TF, EKF, Nav2, 깊이 카메라) |
| [docs/ESTOP_CAN_PROTOCOL.md](docs/ESTOP_CAN_PROTOCOL.md) | E-stop CAN 프로토콜 |
| [docs/LOCALIZATION.md](docs/LOCALIZATION.md) | 초기 위치 탐색(TIM)·검증·AMCL·주행 중 추적 |
| [docs/ELEVATOR.md](docs/ELEVATOR.md) | 층간 이동 설계·알고리즘·카메라 층인식·승객 안전·실차 기록 |
| [docs/KIOSK.md](docs/KIOSK.md) | 배송 키오스크 화면과 관리 모드 |
| [docs/PATROL.md](docs/PATROL.md) | 순찰 경로와 움직임 감지 |
| [docs/SIMULATION.md](docs/SIMULATION.md) | Isaac Sim 준비·실행·장면·센서 모델·자동 시험 |
| [docs/VALIDATION.md](docs/VALIDATION.md) | 설정 기준값, ROS 도메인 배정, 시험 명령, 현장 점검 항목 |
| [docs/EXPERIMENTS.md](docs/EXPERIMENTS.md) | 날짜별 실험·조사·튜닝 기록 |
| [docs/OPTIMIZATION_AUDIT_2026-10-27.md](docs/OPTIMIZATION_AUDIT_2026-10-27.md) | Jetson CPU 최적화 조사 |
| [Control_Server/README.md](Control_Server/README.md) | 중앙 관제 서버 설치·실행, 감지 기록, Slack 설정 |
| [rl/README.md](rl/README.md) | 강화학습 로컬 주행 정책 |
| [sim/native/README.md](sim/native/README.md) | C++ LiDAR 설계·빌드·검증 |
| [src/amr_description/README_ISAAC_SIM.md](src/amr_description/README_ISAAC_SIM.md) | URDF Isaac 임포트·물리 파라미터 |
| [GitHub Issues](https://github.com/actaeon7777/multifloor-amr/issues) | UI/UX 기능 백로그 (`backlog` 라벨) |
