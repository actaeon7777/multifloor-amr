# 실험·조사 기록

날짜별 실험, 원인 조사, 튜닝 결과를 최신순으로 모은다. 현재 동작과 설정은 각 주제 문서
([실차](REAL_VEHICLE.md) · [위치 추정](LOCALIZATION.md) · [시뮬레이션](SIMULATION.md) · [층간 이동](ELEVATOR.md))가 기준이다.

> 원자료 폴더(`diagnostics/`, `logs/`, `reports/<하위 폴더>/`, `bags/`)는 git에 포함하지 않는다.
> `diagnostics/`는 Jetson 작업공간, 나머지는 시뮬레이션 PC 작업공간에 있다. 아래 경로는 그 기준이다.
> 시뮬레이션 수치는 실물 보정 전 모델(`calibration_status: uncalibrated`)의 결과이며 실차 성능으로 확정하지 않는다.

## 2026-10-16 · RotationShim: 목표에서의 반복 회전과 느린 정렬 회전

- `rotate_to_heading_once: true`: 경로가 10초마다 갱신(`nav_to_pose_with_consistent_replanning...`, 시뮬레이션에서 10.7초 간격)될 때마다
  shim이 다시 켜져, 목표 근처에서는 1 m 앞 경로점 대신 목표점 방향으로 돌리고 DWB 최종 정렬과 번갈아 도는 현상.
  "뒤의 목표"(arena, (-2, 0, 0)) 3회 중 1회가 목표에서 20초 이상 회전을 세 번 반복하다 시간 초과.
- `closed_loop: false`: closed loop는 측정 각속도 + 0.8 rad/s² × 0.05 s로 명령을 제한하는데, 제자리 회전 실측(IMU)이 명령의 약 85 %라
  명령이 약 0.26 rad/s(실측 0.22)에서 멈춤. open loop에서 명령 0.40, 실측 0.35 rad/s.
- Isaac 전체 스택 비교(시나리오별 1~3회, `reports/trials/shim_compare_20261016/`): 뒤의 목표 실패 1/3 → 0/3, 성공 시간 76·41초 → 약 27초.
  직진·좌우 코너·좁은 길·장애물 16.0/30.8/43.1/16.2/33.1초 → 12.6/20.0/17.5/13.9/23.3초. s_curve는 두 설정 모두 실패(별도 조사 필요).
- 실차 제자리 회전의 명령 대비 실측 비율은 아직 측정하지 않았다. 문서상 회전 슬립은 관찰됨(EKF 절 참고).

## 2026-10-16 · 시뮬레이션 센서 주기를 실물에 맞춤

- `sim/config/sensors.json` 휠 10→100 Hz, LiDAR 8.8→10 Hz (실물 bag `012425` 기준, IMU 10 Hz는 그대로). 10/04에 같은 변경을 했으나
  10/13 병합 때 PC 쪽 이전 값이 들어와 되돌아가 있었다.
- Isaac headless 스모크 시험(floor4, 6초): 휠 100.08 Hz, IMU 10.01 Hz, LiDAR 10.00 Hz(시뮬레이션 시간), 스캔 3,240점 전부 유효, 명령 단절 정지 통과.
  짧은 시험의 RTF는 0.876.
- arena 폐루프 시험(Nav2): 도착 3.4 cm, 경로 오차 RMS 1.2 cm, 충돌·복구·명령 제한 위반 0, 센서 공백 0, 명령 주기 평균 49.9 ms(75 ms 초과 0),
  `valid_for_comparison: true` (`reports/trials/rates_100_10_arena_004349/`).
- 10/13~14의 층간 이동 시나리오·호실 주행 결과는 이전 주기(휠 10 Hz, LiDAR 8.8 Hz)에서 측정한 값이다.

## 2026-10-16 · 중앙 관제 경유 층간 이동

- 관제 서버가 10/07에 복사해 둔 웹 UI 사본(`Control_Server/vendor/`)을 쓰고 있어 층 기능이 보이지 않았다.
  사본을 지우고 `src/amr_system/`을 직접 참조하도록 바꿨다.
- `amr_remote_bridge`가 `/api/floors`와 층별 정적 지도(`floor/<층>` 레이어)를 전달하도록 확장했다.
  다른 층 목표는 해당 층 지도의 버전·해시로 서버와 AMR 양쪽에서 검증한다.
- 시험 중 레이어 업로드 순서 커서가 특정 레이어를 계속 건너뛰는 결함을 발견해, 가장 오래 보내지 않은 레이어를 고르도록 고쳤다.
- 확인: `amr_remote_bridge` 31개, `amr_web_ui` 78개, Control_Server 60개 통과. 실제 `floors.yaml` 3개 층 지도로
  로컬 웹 → agent → FastAPI 경로의 층 목록·3F 지도·3F 302호 목표 전송을 확인했다. 실제 브라우저 조작은 하지 않았다.

## 2026-10-13~14 · 층간 이동(엘리베이터) 구현

설계와 최종 구조는 [층간 이동](ELEVATOR.md). 작업 전 백업: `backups/amr_ws_pre_elevator_20261013_230836.tar.gz`.
요약 보고 `logs/elevator/REPORT.md`, 최종 원자료 `logs/elevator/final_20261014/`.

### 1단계 — 3F·5F 임시 지도·장면, 층 설정 파일
- `floors.yaml` + 파서 `amr_stack/floor_config.py`(ROS 비의존). 3F·5F는 4F 복사본(`provisional_copy_of: "4F"`, `real_enabled: false`),
  각 지도 폴더에 `COPY_OF_4F.md`. `sim/tools/build_floor_layers.py`로 층 레이어 생성(4F 레이어는 바이트 단위 동일 확인).
- `test_floor_config.py` 32개 통과(지점 여유 0.3 m, 카 내부 벽 없음, USD 문 위치 일치, 임시본=원본 해시, 잘못된 설정 18종 거부).
  amr_stack 67→99. 회귀: arena 0.032 m, floor4 405호 0.164 m, 충돌·복구 0 (`logs/elevator/stage1/`).

### 2단계 — 엘리베이터 신호 인터페이스, 시뮬레이션 신호 발생기
- `amr_elevator_interfaces` 추가. 실차 경로 `elevator_protocol.py`(0x210/0x310, CRC-8, ID·배치는 제안값)와
  `elevator_can_link`(자체 소켓, 기본 dry_run, `start_elevator_link:=false`). 기존 `can_telemetry`·`cmd_vel_to_can`·펌웨어는 변경 없음.
- 시뮬레이션 `elevator_model.py`, `elevator_node.py`. `amr_sim_bridge/setup.cfg`로 launch_testing pytest 플러그인 비활성
  (모듈 하나의 skip이 디렉터리 전체를 skip시키는 문제).
- can_bridge 47→68, sim_bridge 11→44. ROS 시험 `test_elevator_ros.py` 2개(도메인 193). `test_ros_integration.py` 러너에는 미등록
  (러너가 스위트별 고유 도메인을 요구하는데 193/195는 이미 사용 중). 회귀 통과 (`logs/elevator/stage2/`).

### 3단계 — 층·상태 판별
- `amr_elevator`의 `elevator_monitor` 하나에 문 판별(`door_detector.py`)과 층·영역 판별(`floor_state.py`)을 합쳤다(위치 추정 공유).
- 단위 16개, ROS `test_monitor_ros.py` 1개(도메인 195, 4F→3F 탑승·하차·운영자 지정). 회귀 통과 (`logs/elevator/stage3/`).

### 4단계 — 엘리베이터 장면, 탑승·하차 주행
- 수직 적층 장면 `sim/building_G.usda`, kinematic 카, `elevator_runtime.py`, `elevator_sim_relay`, 직선 기동(`maneuver.py`),
  기동 임대(`/navigation/maneuver_lease`), 위치 추정 일시 정지(`/localization/suspend`), 카 내벽 맞춤(`car_matcher.py`).
- 디버깅:
  1. 슬래브를 Cube 충돌체로 만들면 카 안 제자리 회전 뒤 하차 중 x≈9.8 m에서 3.7° 기울며 멈춤 → 회전 없으면 정상,
     메시 바닥이면 정상임을 프로브로 확인 → 슬래브를 삼각형 메시로 교체.
  2. 카 바닥과 슬래브 사이 1.5 cm 틈 제거(카 바닥판을 2 cm 겹침).
  3. 문 근처에서 카 벽 맞춤이 홀 벽을 잘못 맞춰 추정이 0.2 m 뒤처짐 → 로봇 전체가 카 안일 때만 보정.
  4. 1회: Nav2 lifecycle의 map_server 설정 응답 타임아웃(시작 시 DDS 경합, 엘리베이터와 무관).
- navigation_manager 19개·navigation_startup 90개 기존 ROS 시험 통과(도메인 190·194). `navigation.yaml` 변경 없음.

### 5단계 — 층간 미션, 지도 전환·재위치, 웹 UI
- `floor_mission_manager`(action `/floor_mission/navigate`), `experiment.launch.py`의 `world:=building` 자동 구성, 웹 UI 층 탭·호실 선택.
- amr_web_ui 73→78(+ROS 3개, 도메인 193), JS 2개. Isaac: 4F→3F 302호 0.032 m, 4F→5F 502호 0.268 m,
  3F→5F 503호 0.78/0.17/0.33 m(0.78 m 1회는 긴 복도에서 AMCL이 진행 방향으로 0.8 m 뒤처진 기존 유형, 하차 직후 재위치 오차 2 cm),
  충돌 0 (`logs/elevator/stage4/`).

### 6단계 — 층간 시나리오 통합 시험
- `run_floor_trial.py`, `run_floor_suite.py`, `scenarios.json`의 `floor_scenarios` 11종. **최종 11/11 통과**.
  성공 8건 도착 오차 평균 0.21 m(최대 0.49 m), 충돌 0, Nav2 복구 0, RTF 1.0, 층·영역 일치 95~100 %. 시나리오별 표는 [층간 이동](ELEVATOR.md#시나리오-결과-2026-10-14).
- 시험 중 수정: `elevator_monitor`가 경고 이벤트 기록 시 rclpy 로거 예외로 종료(실차에도 해당); `/floor/state` 3 s 단절 시 안전 정지 추가;
  늦게 온 이전 버튼 응답 인정; Nav2 구간 위치 추정 일시 해제 시 재시도 소모 없이 최대 45 s 회복 대기; 시험 도구의 시작 시 고장 주입 미적용.
- 격리 ROS(193/195) monitor 1, mission 6, lease·suspend 2, sim+CAN 2, web_ui 3, vehicle_status 10 통과 (`logs/elevator/final_ros/`).
  단위: amr_elevator 23, amr_sim_bridge 45, amr_can_bridge 68, amr_stack 99, amr_control 122, amr_web_ui 78, amr_remote_bridge 28.
- 회귀: arena 0.064 m, floor4 405호 0.14 m, 402호 0.167·0.143 m (`logs/elevator/final_regression/`).
  Nav2 시작 경합으로 2회 재실행(단일 층 월드에서도 발생).

## 2026-10-13 · Jetson/PC 작업공간 병합

- 제어·기능 코드는 Jetson 작업공간, Isaac Sim 환경(`sim/assets/`, 장면, `reports/`, 일부 docs)은 시뮬레이션 PC 작업공간을 기준으로 합쳤다.
  `amr_master.stl`은 PC의 v28 조립체로 교체(`reports/source_snapshot.json` 해시와 일치).
- `scripts/link_pip_isaacsim.sh` 추가: pip Isaac Sim(`~/isaacsim-env`)을 `runtime/isaac-sim-6.1.0`에 standalone 구조로 연결.
  Isaac 내장 Jazzy 라이브러리(`exts/isaacsim.ros2.core/jazzy/lib`)를 `LD_LIBRARY_PATH`에 넣어야 ROS 2 브리지가 뜬다.
- 확인: `./run_amr.sh --test` 통과, 네이티브 LiDAR 빌드, colcon 16개 패키지 빌드, 단위 시험(amr_sim_bridge 11, amr_can_bridge 45,
  amr_vehicle_status 17, amr_stack 67, amr_control 122, amr_web_ui 73), arena 통합 시험(sim 26.1 s, RTF 1.0, 복구 0, 충돌 0, 명령 주기 평균 50 ms).
- 알려진 현상: 시험 종료(SIGINT) 시 `vehicle_status`가 rclpy 예외로 exit 1(종료 과정에서만). sim 모드에서 `/motor/*/telemetry`가 없어 모터 상태 stale.

## 2026-10-13 · 4층 호실 주행과 전역 경로 평활화 (시뮬레이션)

- 호실 401~406 모두 도착, 위치 오차 0.10~0.22 m / 5~7° (`scripts/sim_demo.py --room 401 402 ...`).
  403 첫 시도에서 긴 복도 제자리 회전 중 AMCL/TF 불일치로 READY가 약 3초 해제되어 목표 중단(재시도에서 재현 안 됨). 실차 관찰 항목.
- 전역 경로 평활화: `navigation.yaml` Smac2D smoother `w_smooth 0.3→0.95`, `w_data 0.2→0.02`, `max_iterations 1000→4000`, `refinement_num 4→6`.
  원인은 8방향 격자 계단 + 꺾이는 세로 복도에서 경로가 정중앙을 그대로 따라 흔들리는 것.
  - 오프라인 비교 `scripts/benchmark_global_planner.py`(도메인 142, Isaac 불필요): 6개 경로 방향 반전 29→0회, 회전량 약 60 % 감소,
    길이 약 1 % 감소, 최소 벽 거리 0.80→0.75 m(세로 복도 폭 1.6 m 병목). Theta*, Lattice(0.5 m)는 더 나빴다.
  - Isaac 주행(floor4): 세로 복도 진입 회전량 150→76°, 401호 127→94°, 반전 2→0, 소요 시간 동일, 최소 벽 거리 0.74→0.68 m(코너 안쪽),
    가로 복도 중앙 편차 2.0→4.5 cm.
  - 이 파일은 실차와 공용이다. 실차 반영 시 코너 안쪽 여유를 확인한다.

## 2026-10-07 · 문서 통합과 중앙 서버 재적용 (Jetson)

소프트웨어 시험 192개 통과, opt-in ROS 모듈 1개 제외. 세부 로그·JUnit·체크섬은 Jetson `diagnostics/docs_unification_20261007/`.
`PASS`는 해당 범위만 통과했다는 뜻이다.

| 항목 | 판정 | 원자료 (Jetson `diagnostics/`) |
|---|---|---|
| 중앙 대시보드 수치 깜빡임 수정 | PASS · 전달 지연 재현/원본 만료/브라우저 | `dashboard_flicker_20261007/` |
| CAN 전압·Heartbeat 실제 연동 | PASS · 실측 CAN 수신/격리 ROS/브라우저 | `power_fix_20261007/` |
| 새 재현 ROSBAG 기반 수정 | PASS · 위치/Action/HTTP/브라우저 | `repro_fix_20261007/` |
| 중앙 서버 최신본 재적용 | PASS · HTTP 54개, FastAPI 브라우저(5개 화면 크기·단절/재접속) | `control_server_refresh_20261007_161342/` |
| 단위·HTTP | PASS · 129개 | `docs_unification_20261007/unit_http.xml` |
| 공용 상태 ROS / 웹 ROS | PASS · 6개 / 3개 | `docs_unification_20261007/` |
| 로컬·원격 브라우저 | PASS · console error 0 | `docs_unification_20261007/browser_*.json` |
| 호스트 FastAPI 호환 | PASS · 54개 (임시 aarch64/Python 3.12 환경) | `docs_unification_20261007/host.xml` |
| 실제 호스트 최신 배포·왕복 | 미확인 | — |
| 실제 조이스틱·주행·정지·BMS·펌웨어 readback | 미실행 | — |

이전 기록: 공용 상태·UI `vehicle_status_20261007/`, 포팅·실물 센서 점검 `pc_update_20261006/REPORT.md`.

## 2026-10-04 · 위치 추정 시작·추적 조사

현재 설계는 [위치 추정](LOCALIZATION.md). 원자료는 시뮬레이션 PC `reports/localization_startup_validation/`.

- **TIM 초기 이식 검증**: C++ 코어 10개, 시작 감독기 41개, 실제 TIM→AMCL 합성 센서 연동 4개 통과. 4층 지도의 독립 raycaster 스캔 3개에서
  초기 위치 입력 없이 모두 정답 후보(위치 오차 1.0~2.3 cm, TIM 계산 4.8~5.0초). 긴 가로 복도 시험점은 정답이 1위여도 모호성 검사로 보류.
  잡음 없는 지도 기반 시험이다 (`tim_floor4_final.json`).
- **정밀 보정(fine ICP) 비교**: 세 시험점 위치 오차 2.245→0.297, 1.365→0.278, 1.031→0.436 cm, 추가 계산 0.005~0.167초
  (`tim_fine_disabled.json`, `tim_fine_enabled.json`). 최종 검증 C++ 17, 감독기 58, TIM→AMCL 5, Nav2 회귀 2 (총 82개),
  읽기 전용 파라미터 34개 변경 거절 확인 (`tim_accuracy_upgrade_validation.json`). 실제 Isaac·Jetson 첫 시도 성공률은 미측정.
- **수동 초기화 지연**: READY까지 12.983→4.100초(실제 AMCL, 3,240빔 합성 센서, 웹과 같은 초기 공분산, 각 1회, Nav2 lifecycle은 모의)
  (`scripts/benchmark_manual_localization_latency.py`, `manual_latency_comparison.json`).
- **주행 중 목표 취소 회귀 원인**: 저장소 기준 코드(`e0b5cd1`)의 `navigation_manager`에는 `/localization/status`에 의한 취소 조건이 없었다.
  자동 초기화 추가 시 감독기의 준비 상태를 주행 허용 조건으로 연결하면서, 초기화용 스캔·공분산 기준을 주행 중에도 취소에 쓴 것이 원인.
  제공 로그의 READY → VERIFYING → 취소 동안 TIM 새 탐색은 없었다. 첫 READY 이후 품질 경고가 초기화 상태를 되돌리던 연결을 제거했다.
  실제 Jetson 배포 바이너리 해시는 확인하지 않았다.
- **회전 스캔 보정**: `scripts/benchmark_rolling_scan_quality.py`. 4층 정적 형상, 8.8 Hz·3,240점, 0.4 m/s·0.6 rad/s 궤적 두 예에서
  정확한 위치로도 한 시각 투영 점수 0.614·0.544 → 빔별 보정 후 0.994·0.997 (`rolling_scan_repro.json`). 기하 재현이며
  04:18 주행은 원본 bag가 없어 직접 입증은 아니다.
- **AMCL 관측 거리 5 m→30 m**: 회전 보정만으로는 Isaac 재현에서 같은 경로에 정지. 첫 실패 스캔의 실제 자세 오차 4.6 cm·1.69°,
  30 m 검증 점수 0.681, 5 m 범위 0.836. 검증 거리만 5 m로 줄이면 이후 공분산 기준 초과로 다시 정지. AMCL 관측 자체를 30 m로 넓히고
  검증도 30 m로 맞췄다. 동일 bag 재생 비교 평균 위치 오차 7.09→4.30 cm, 방향 0.756→0.398° (`amcl_range_replay/comparison.json`).
  30 m에서도 위치 표준편차 0.352 m가 초기 기준 0.350 m를 넘어 46.50초에 목표 취소(`long_range_isaac_trial`) → 초기화 조건과 주행 경고를 분리.
  재현 원자료 `rolling_isaac_trial`, 오프라인 대조 `rolling_isaac_bag_diagnosis.json`.
- **수정 후 재주행**: 4층 목표 (41.41, 0.36, 0) 약 86초에 도착. 상태 1,722개 모두 READY, 공분산 경고 최대 37.7초·331스캔 지속에도
  취소·재발행·빔 변경 없음. 충돌·복구 0, 도착 오차 0.112 m, 주행 중 위치 오차 평균 0.149 m·최대 0.512 m(복도 진행 방향).
  수동 시작 위치 1회 재현이다 (`startup_scope_isaac_trial`, `startup_scope_validation.json`).

## 2026-10-04 · 센서 주기 정합 (시뮬레이션)

- 실물 bag `bags/012425`의 메시지 timestamp로 대조: 휠 odometry·joint states 약 100.077 Hz, IMU 10.000 Hz, LiDAR 10.011 Hz, EKF 30.000 Hz.
  LaserScan은 3,240점, 0.05–30 m 선언.
- 당시 `sim/config/sensors.json`의 휠 10→100 Hz, LiDAR 8.8→10 Hz로 바꾸고 Isaac 6초 headless 검사(휠 100.083 Hz, IMU 10.008 Hz,
  LiDAR 10.000 Hz, RTF 0.965)와 관련 시험 37개를 통과했다 (`reports/sensor_alignment_012425/`).
- 이 값은 10/13 병합 때 되돌아갔다가 10/16에 다시 적용했다.

## 2026-10-04 · 바닥 접촉 불안정 (시뮬레이션)

- 장거리 주행에서 바닥 중앙(x≈21.6 m) 부근 접촉 불안정. 50.7×23.1×0.1 m 바닥의 `boundingCube` 충돌 근사가 둥근 바퀴와의 접촉에서 차체를 떨리게 했다.
  센서·Nav2 없이 일정 속도 시험에서도 최대 기울기 3.01°, 높이 변화 8.72 mm → 정적 삼각형 메시(`physics:approximation = "none"`)로 0.050°, 0.258 mm.
- 접촉 보고는 `PhysxContactReportAPI.reportPairs`로 장애물에 한정(바닥 접촉 수집 시 PhysX 110.3.2 삼각형 재질 조회 경고가 대량 발생).
- 최종 검증: 자동 위치 찾기 후 장거리 도착·180° 회전·복귀, 장애물 접촉 0, RTF 1.000, 높이 변화 0.533 mm, 최대 기울기 0.073°, 회귀 19개 통과.
  수직 가속도 피크 6.874 m/s²가 진단 기준 5 m/s²를 넘어(49,532 구간 중 6개, 5–10 ms) 엄격한 `passed`는 `false`로 보존
  (`reports/driving_instability_20261004/`, `residual_peak_review.json`).

## 2026-10-04 · 제어기 재시작과 시간 이상 처리

- 현재 동작은 [실차 — 시간 이상 처리](REAL_VEHICLE.md#10-시간-이상-처리). 수정 전 소스는 `../backups/jetson_before_time_recovery_20261004_160315/`.
- 동일 웹 UI 프로세스를 유지한 두 번의 Isaac 자동 초기화·3 m 주행 성공, `TF_OLD_DATA` 경고 0건. 단위·HTTP 206개, 위치 초기화 86개,
  ROS 재시작·시간 오류·목표 차단·가짜 CAN 검증 통과 (`reports/time_recovery_20261004/`). 실물 배포와 실제 OS 시계 변경 시험은 미수행.

## 2026-10-03 · 시뮬레이션 실행 성능과 LiDAR 구현

현재 구성은 [시뮬레이션 — 성능 관련 설정](SIMULATION.md#6-성능-관련-설정). 원자료 `reports/performance/`.

- **C++ 도입 전 최적화**(EKF `predict_to_current_time`, LiDAR 각도 사전 계산·NumPy 일괄 회전, 관절 인덱스 재사용,
  웹 이미지 Pillow 테이블 변환, headless 숨은 뷰포트 렌더링 비활성):

  | 측정값 | 수정 전 | 수정 후 GUI 2회 |
  | --- | --- | --- |
  | 실시간 비율 | 0.885 | 0.974–0.992 |
  | Nav2 명령 간격 p95 | 111 ms | 약 70 ms |
  | 제어 주기 누락 경고 | 140회 | 18, 27회 |
  | LiDAR 계산 평균 / 물리 단계 | 1.70 ms | 약 1.15 ms |

  headless 숨은 뷰포트 비활성 전후: 제어 주기 경고 28→1회, 화면 처리 평균 13.59→1.45 ms. 웹 이미지 생성 지도 68.7→8.9 ms, costmap 112.2→17.7 ms.
- **Physics RaycastSensor API**: Isaac Sim 6.1의 `reading.depths`가 shape `(3240,)`, strides `(0,)`로 반환되어 첫 거리 하나가 반복되는 현상 재현
  (`sim/scripts/probe_native_lidar.py`, `native_lidar_probe.json`). 기본 경로에 쓰지 않았다.
- **PhysX C++ 배치 질의**: 254,673개 광선에서 Python 경로와 최대 거리 차이 0 m (`cpp_lidar_equivalence.json`).

  | 측정값 | Python | C++ |
  | --- | --- | --- |
  | LiDAR 처리 평균 / 물리 단계 | 1.132 ms | 0.283 ms |
  | LiDAR 처리 p95 | 1.401 ms | 0.357 ms |
  | 실시간 비율 | 0.991 | 1.000 |
  | 제어 주기 누락 경고 | 29회 | 27회 |

  전체 프로세스 CPU는 약 647 %→704 %여서 전체 CPU 감소는 확인되지 않았다 (`cpp_summary.json`).
- **GUI 로딩 대기**: 긴 프레임 7개 모두 `GeometryStreaming : postSync`의 `Wait for budget`에 약 60 ms. `SimulationApp(sync_loads=False)` 적용.

  | 측정값 | 동기 로딩 1회 | 변경 후 2회 |
  | --- | --- | --- |
  | GUI 갱신 p99 | 65.12 ms | 10.03, 6.19 ms |
  | GUI 갱신 최대 | 67.09 ms | 323.23, 8.23 ms |
  | 제어 주기 경고 | 17회 | 2, 1회 |

  첫 개선 주행의 단발성 323 ms 지연은 원인 미확인. 약 128–139 ms 전체 GC는 모두 `sim_time_s=0` 초기 로딩 단계에서 발생 (`gui_timing_summary.json`).
- PhysX 작업 스레드 0개 비교는 효과가 작아 기본값 유지. 3개 목표 시험에서 Nav2 재시도가 있어 경로 추종 튜닝 완료로 보지 않는다.

## 2026-10-02 · Nav2 튜닝 (실차 bag)

- RotationShim 참조점 0.50→1.00 m, 정렬 속도 0.40 rad/s: 기존 참조점을 정지 중 지나치며 뒤쪽으로 회전한 현상.
- DWB critic 가중치(`BaseObstacle 0.12`, `PathAlign 32`, `GoalAlign 8`, `PathDist 32`, `GoalDist 32`)와 최소 탐색 속도, 회전 가감속 1.60 rad/s²:
  세 번째 bag의 직진·코너·저속 회전 분석 (Jetson `diagnostics/navigation_2026-10-02_trial3/REPORT.md`).
- `/evaluation` 후보 궤적 발행 비활성: 전체 토픽 bag에서 이 토픽만 310 MiB/111초, 제어 주기 미준수 기록 (`diagnostics/navigation_2026-10-02/REPORT.md`).

## 2026-09-29 · 실차 센서·EKF·RViz

- IMU 정지 60초 자이로 XYZ 평균을 `gyro_bias`에 수동 반영, 재측정에서 Z 평균 편향 약 99.6 % 감소.
  관측 정지 평균 약 (-0.01275, 0.00151, 0.00025) rad/s (`diagnostics/stationary_2026-09-29_020033/report.md`).
- CAN 수신 실측 `diagnostics/telemetry_2026-09-29/live_result.json`, EKF `diagnostics/ekf_2026-09-29/report.md`,
  내비게이션 초기 기록 `diagnostics/navigation_2026-09-29/`, 튜닝 `diagnostics/navigation_tuning_2026-09-29/REPORT.md`.
- RViz 멈춤: 기존 Humble 환경에서 초기 위치 지정 뒤 `rviz2 is not responding` → TF 0.25.23 메시지 필터와 TF 수신 스레드 교착
  ([geometry2#992](https://github.com/ros2/geometry2/issues/992)). `amr_rviz_plugins/SingleThreadedTF`로 해결 (`diagnostics/rviz_hang_live_2026-09-29/`).
  강제 종료(-9)는 사용자가 응답 없는 창을 종료한 기록이며 OOM으로 단정하지 않는다.
