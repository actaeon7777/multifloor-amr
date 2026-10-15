# AMR 프로젝트 개요

> 작성일: 2026-10-13 · 기준 워크스페이스: `amr_ws` (Jetson 제어 코드 + 로컬 PC Isaac Sim 환경 병합본)
> 세부 실행법·검증 절차는 각 문서 링크를 따른다. 이 문서는 전체 구조를 한눈에 보기 위한 요약이다.

## 1. 한 줄 요약

G동 4층 실내를 자율 주행하는 **차동구동 AMR**의 ROS 2 Jazzy 작업공간.
실차(Jetson + STM32 + ODrive)와 **Isaac Sim 6.1.0** 시뮬레이션이 같은 제어 스택을 공유하며,
웹 UI로 목적지를 지정하고 중앙 관제 서버로 원격 모니터링한다.

## 2. 플랫폼과 하드웨어

| 구분 | 내용 |
|---|---|
| 차량 컴퓨터 | NVIDIA Jetson, JetPack 7.2.1 / Ubuntu 24.04 / arm64 |
| 시뮬레이션 PC | Ubuntu 24.04 x86_64, Isaac Sim 6.1.0 (pip 빌드 6.1.0-rc.26) |
| 미들웨어 | ROS 2 Jazzy, `rmw_fastrtps_cpp` |
| 하위 제어기 | STM32 (`stm32_main_code.c`) → ODrive 2축 (node 0 = 좌, node 1 = 우) |
| 통신 | SocketCAN `can0`, 500 kbit/s (`can0.service`로 부팅 시 설정) |
| LiDAR | SLAMTEC S2 (`sllidar_ros2`, frame `laser`) |
| IMU | STM32를 거쳐 CAN으로 수신 (0x301 가속도, 0x302 자이로) |
| 조작기 | Xbox Series X Controller |
| 기구 | 바퀴 지름 0.172 m, 윤거 0.499 m, 바퀴 상한 1.8 rev/s, footprint 0.64 × 0.544 m |
| 로봇 모델 | v28 URDF/Xacro + STL (`src/amr_description`) |

## 3. 시스템 구조

### 명령 경로

```text
joy_node → /joy → joy_teleop → /cmd_vel_manual ─┐
                                                ├→ control_mux → /cmd_vel → cmd_vel_to_can → CAN 0x201 → STM32 → ODrive
Nav2 (navigation_manager) → /cmd_vel_auto ──────┘   (바퀴 속도 제한)
```

- **LB(버튼 6)**: 데드맨. MANUAL에서 누르는 동안만 수동 주행.
- **RB(버튼 7)**: MANUAL ↔ AUTO 토글. AUTO → MANUAL 전환 시 정지(목표는 유지, 일시정지).
- `control_mux`가 바퀴 속도 상한 `U = π·d·rev_s`를 넘는 명령은 `(v, ω)`를 같은 비율로 축소해 회전 반경을 유지한다.
  STM32는 역기구학과 모터 방향만 적용하며, 통신 타임아웃·모터 오류 정지는 하위에 남아 있다.

### 피드백·위치 추정 경로

```text
CAN 피드백 → can_telemetry → /wheel/odometry, /joint_states, /imu/data_raw
                                   ↓
                     robot_localization EKF (odom → base_footprint, 30 Hz, 2D)
S2 LiDAR → /scan → AMCL (map → odom) ← TIM 전역 초기화 / 웹 초기 위치
                                   ↓
                     Nav2: Smac 2D 플래너 + RotationShim(DWB) 컨트롤러
```

- 초기 위치: `amr_tim_localization`(TIM 2026 RRT/SMAD/TAM/ICP의 ROS 2 포팅)이 후보를 내고,
  `navigation_startup`이 지도·스캔 정렬을 검증한 뒤 AMCL에 `/initialpose`를 전달한다.
- AMCL은 초기 검증 1080빔 → 주행 60빔으로 전환, 관측 거리 30 m.
- READY 이후의 품질 저하는 `tracking_warning`으로만 표시하고 목표를 취소하지 않는다
  (센서 단절·TF 누락·공분산 이상 등은 READY 해제).

### 시뮬레이션 경로

Isaac Sim이 실차 드라이버 대신 같은 토픽을 발행한다 (`amr_sim_bridge`).

| 방향 | 토픽 |
|---|---|
| 제어기 → Isaac | `/cmd_vel` (Twist) |
| Isaac → 제어기 | `/clock`, `/wheel/odometry`, `/joint_states`, `/imu/data_raw`, `/scan`, `/sim/ground_truth`, `/sim/status` |

- LiDAR는 PhysX 장면 질의를 쓰는 C++ 배치 레이캐스트(`sim/native/`, 3240점, pybind11)가 기본이며
  `AMR_LIDAR_BACKEND=python`으로 Python 경로를 선택할 수 있다.
- 브리지는 TF를 발행하지 않는다. TF는 실차와 동일하게 `robot_state_publisher`/EKF/AMCL이 담당한다.

### ROS 도메인 분리

| 용도 | `ROS_DOMAIN_ID` |
|---|---|
| 실차 | 7 |
| 시뮬레이션 | 107 |
| 오프라인 플래너 벤치마크 | 142 |
| 격리 ROS 테스트 | 193, 195 |

## 4. ROS 패키지 (`src/`)

| 패키지 | 역할 |
|---|---|
| `amr_stack` | bringup·experiment·localization·navigation·slam·lidar launch와 설정 YAML, 지도. `navigation_manager`, `navigation_startup`, `localization_quality`, `joy_autoselect` 노드 |
| `amr_control` | `control_mux`(수동/자율 선택, 바퀴 속도 제한), 시간 감시, 직진·회전 시험 노드 |
| `amr_can_bridge` | `cmd_vel_to_can`(송신), `can_telemetry`(휠 오도메트리·관절·IMU 수신), 구동 프로토콜, 엘리베이터 RF 중계 `elevator_can_link`(기본 꺼짐) |
| `amr_web_ui` | 터치 지도 기반 목표 지정 웹 UI + 로컬 ROS 2 HTTP 브리지 (포트 8080) |
| `amr_remote_bridge` | AMR → 중앙 서버 상태·지도 전송, 명령 중계 (Tailscale 사설망) |
| `amr_vehicle_status` / `amr_vehicle_interfaces` | UI와 독립된 공용 차량 상태 발행(10 Hz)과 메시지 정의 |
| `amr_tim_localization` / `amr_localization_interfaces` | 전역 재위치 추정 서비스와 인터페이스 |
| `amr_nav2_plugins` | 바퀴 속도 실현 가능성을 평가하는 DWB critic |
| `amr_rviz_plugins` | RViz TF 데드락 회피 플러그인 |
| `amr_sim_bridge` | Isaac PhysX 구동기·센서 ↔ 실차 인터페이스 어댑터, 시뮬레이션 엘리베이터 모델·중계(`elevator_sim_relay`) |
| `amr_elevator` / `amr_elevator_interfaces` | 층간 이동: 문·층·안/밖 판별(`elevator_monitor`), 층간 미션(`floor_mission_manager`), 메시지·액션 |
| `amr_description` | v28 URDF/Xacro, 물리 URDF, 메시 |
| `teleop_tools` (`joy_teleop`, `teleop_tools_msgs`) | F1TENTH 소스에서 복사한 조이스틱 입력 변환 |
| `sllidar_ros2` | SLAMTEC LiDAR 드라이버 |

주요 설정: `src/amr_system/amr_stack/config/`
(`control_mux.yaml`, `can_bridge.yaml`, `can_telemetry.yaml`, `ekf.yaml`, `joy_teleop.yaml`,
`localization.yaml`, `localization_startup.yaml`, `navigation.yaml`, `slam.yaml`)

## 5. 디렉터리 구성

| 경로 | 내용 |
|---|---|
| `src/` | ROS 2 패키지 (빌드 대상, `colcon build --base-paths src`) |
| `sim/` | Isaac Sim 장면(USD), 설정(`config/*.json`), 실행·검증 스크립트, C++ LiDAR, 지도 생성 도구 |
| `maps/` | 참조되지 않는 지도 복사본. 실제 사용 지도는 `src/amr_system/amr_stack/maps/` (G_3/4/5_floor, sim_*) |
| `scripts/` | 실행(`experiment.sh`), 빌드, 시험·벤치마크, 기록(`record_trial.sh`), 릴리스 도구 |
| `Control_Server/` | 중앙 관제 FastAPI 서버 (포트 8081) |
| `docs/` | 운영·검증·Sim2Real 문서와 이력(`archive/`) |
| `tests/` | 워크스페이스 도구 단위 시험 |
| `reports/` | 장면 생성·검증 메타데이터, 원본 형상 해시(`source_snapshot.json`) |
| `logs/` | 시뮬레이션 시험·벤치마크 결과 |
| `runtime/` | Isaac Sim 런타임 링크와 캐시 (git 제외) |
| `stm32_main_code.c` | STM32 펌웨어 참고본 (`Core/Src/main.c`에 반영 후 빌드·업로드) |

## 6. 실행 요약

### 실차 (Jetson)

```bash
# 터미널 1 — 차량 스택 (도메인 7, G동 4층 지도)
cd /home/acus03/amr_ws && ./scripts/experiment.sh real

# 터미널 2 — 웹 UI
source scripts/env.sh && export ROS_DOMAIN_ID=7
ros2 launch amr_web_ui web_ui.launch.py use_sim_time:=false host:=127.0.0.1 port:=8080
```

브라우저 `http://127.0.0.1:8080` → **주행 준비 완료** 확인 후 목표 지정.
원격 접속(SSH 터널 18080), 중앙 관제(`remote_client`, 8081), 종료 순서는 [docs/README.md](docs/README.md).

### 시뮬레이션 (PC)

```bash
scripts/link_pip_isaacsim.sh          # pip Isaac 연결 (최초 1회)
./scripts/build_ros.sh                # ROS 빌드
./build_scene.sh                      # 지도·물리 URDF·AMR USD 생성 (필요 시)

./scripts/experiment.sh sim           # 4층 + 전체 스택 (도메인 107)
ROS_DOMAIN_ID=107 ros2 launch amr_web_ui web_ui.launch.py use_sim_time:=true port:=8080

python3 scripts/sim_demo.py --room 401 402   # 4층 호실 순회 데모
./run_amr.sh --test                          # 평지 구동 확인 (ROS 없이)
```

- 시험 자동화: `scripts/run_sim_trial.py --world arena|floor4 ...`, 시나리오는 `sim/config/scenarios.json`
  (straight, corner_left/right, s_curve, narrow, obstacle).
- 전체 ROS 프로세스 정리: `ros2kill` (`--dry-run`으로 미리보기).
- 층간 이동(3F·4F·5F, 엘리베이터): `./scripts/experiment.sh sim world:=building [start_floor:=3F]`,
  시나리오 `scripts/run_floor_suite.py`. 층 설정 `src/amr_system/amr_stack/config/floors.yaml`, 설계 `docs/ELEVATOR_DESIGN.md`.

## 7. 현재 상태 (2026-10-13 기준)

- colcon 16개 패키지 빌드, 네이티브 LiDAR 빌드 성공.
- 단위 테스트 통과: amr_sim_bridge 11, amr_can_bridge 45, amr_vehicle_status 17, amr_stack 67, amr_control 122, amr_web_ui 73.
- arena 통합 시험: Nav2 목표 도달, RTF 1.0, 복구 0, 충돌 0, 명령 주기 평균 50 ms.
- 4층 호실 401~406 시뮬레이션 주행 모두 도착 (위치 오차 0.10~0.22 m / 5~7°).
- 전역 경로 평활화 적용 (Smac2D smoother `w_smooth 0.95`, `w_data 0.02` 등):
  방향 반전 29→0회, 회전량 약 60% 감소. `navigation.yaml`은 실차와 공용이므로 실차 반영 시 코너 안쪽 여유 확인 필요.

- 층간 이동 시뮬레이션 (2026-10-14): 11개 시나리오 통과, 결과 `logs/elevator/REPORT.md`. 3F·5F는 4층 지도 복사본, 실차 미적용.

## 8. 알려진 이슈·확인할 항목

- 긴 복도 제자리 회전 중 AMCL/TF 불일치로 localization READY가 약 3초 해제되어 목표가 중단된 사례 (403, 재현 안 됨) — 실차에서 관찰 필요.
- 시험 종료(SIGINT) 시 `vehicle_status`가 rclpy 예외로 exit 1 (주행 중 영향 없음).
- sim 모드에서 `/motor/*/telemetry`가 없어 모터 상태가 stale로 표시됨.
- 시뮬레이션 센서·물리 모델은 **실물 보정 전**이다. 제동거리·절대 위치 정확도를 시뮬레이션 수치로 확정하지 않는다.
- 주행 중 위치 추정 오차가 복도 진행 방향으로 남아 있음 (평균 0.149 m, 최대 0.512 m, 1회 재현).
- AMR 자체 패키지 라이선스는 미정(`TODO`).
- 워크스페이스에 `.git`을 두지 않는다 (커밋 없는 `.git`이 있으면 `capture_manifest.py`가 실패).

## 9. 관련 문서

| 문서 | 내용 |
|---|---|
| [README.md](README.md) | 실차 빠른 실행 |
| [docs/README.md](docs/README.md) | 실운용 실행법 (조작·원격·중앙 관제·종료) |
| [docs/VALIDATION.md](docs/VALIDATION.md) | 검증 절차, 설정 기준값, 재현 명령 |
| [docs/SIM2REAL.md](docs/SIM2REAL.md) | 공통 제어·시뮬레이션 구조, 위치 추정 설계와 실험 기록 |
| [docs/REAL_AMR_BASELINE.md](docs/REAL_AMR_BASELINE.md) | 실차 주행 제어·조이스틱·CAN·빌드·주행 기록 |
| [docs/WORKSPACE_PORTABILITY.md](docs/WORKSPACE_PORTABILITY.md) | 워크스페이스 전달·4층 장면 |
| [MERGE_NOTES.md](MERGE_NOTES.md) | Jetson/PC 병합 기록과 최근 변경 |
| [docs/ELEVATOR_DESIGN.md](docs/ELEVATOR_DESIGN.md) | 층간 이동(엘리베이터) 설계·구현 반영 사항 |
| [HOST_PC_ISAAC_SIM_README.txt](HOST_PC_ISAAC_SIM_README.txt) | 새 호스트 PC Isaac Sim 준비 |
| [sim/native/README.md](sim/native/README.md) | C++ LiDAR 설계·빌드·검증 |
| [Control_Server/README.md](Control_Server/README.md) | 중앙 관제 서버 실행 |
