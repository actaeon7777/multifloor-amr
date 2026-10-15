# amr_ws 병합 기록 (2026-10-13)

기준: 제어·기능 코드는 `jetson_amr_ws`, Isaac Sim 환경은 `localpc_amr_ws/amr_ws`.
두 원본 폴더는 참고용으로 그대로 두며 빌드(`--base-paths src`)와 git에서 제외한다.

## 구성
- 전체: `jetson_amr_ws` 복사본
- PC에서 추가: `sim/assets/`, `sim/amr_flat_test.usda`, `reports/`,
  `docs/SIM2REAL.md`, `docs/REAL_AMR_BASELINE.md`, `docs/WORKSPACE_PORTABILITY.md`
- PC 버전으로 교체: `src/amr_description/meshes/reference/amr_master.stl`
  (v28 조립체, `reports/source_snapshot.json` 해시와 일치)
- 신규: `scripts/link_pip_isaacsim.sh` — pip Isaac Sim(`~/isaacsim-env`)을
  `runtime/isaac-sim-6.1.0`에 standalone 구조로 연결. Isaac 내장 Jazzy 라이브러리
  (`exts/isaacsim.ros2.core/jazzy/lib`)를 LD_LIBRARY_PATH에 넣어야 ROS 2 브리지가 뜬다.
- git 저장소는 두지 않는다. `.git`이 있는데 커밋이 없으면 `capture_manifest.py`가 실패한다.

## 이 PC 준비
```bash
scripts/link_pip_isaacsim.sh            # Isaac 연결 (한 번)
sudo apt-get install -y ros-jazzy-navigation2 ros-jazzy-nav2-bringup \
  ros-jazzy-robot-localization ros-jazzy-test-msgs ros-jazzy-slam-toolbox \
  ros-jazzy-xacro ros-jazzy-control-msgs liboctomap-dev python3-can
./scripts/build_ros.sh                  # 또는 colcon build --base-paths src --symlink-install
```

## 확인 결과
- `./run_amr.sh --test`: AMR_RUNTIME_PASS (평지 전진/후진/회전/정지)
- 네이티브 LiDAR 빌드 성공
- colcon 16개 패키지 빌드 성공
- 단위 테스트 통과: amr_sim_bridge 11, amr_can_bridge 45, amr_vehicle_status 17,
  amr_stack 67, amr_control 122, amr_web_ui 73
- 통합 시험 `scripts/run_sim_trial.py --world arena` 통과: Nav2로 (2,0) 목표 도달,
  sim 26.1s, RTF 1.0, recovery 0, 충돌 0, 명령 주기 평균 50ms(75ms 초과 0)

## 시뮬레이션 관련 참고
- 시험 종료(SIGINT) 시 vehicle_status가 rclpy 예외를 내며 exit 1로 끝난다. 종료 과정에서만
  발생하고 주행 중에는 문제가 없다. Jetson 코드에도 같은 현상이 있을 수 있다.
- sim 모드에서도 `vehicle_status`가 실행되며, `/motor/*/telemetry`가 없어서 모터 상태가 stale로 표시된다.
- 설치된 Isaac은 6.1.0-rc.26 pip 빌드다.

## 4층 호실 주행 (sim)
- 호실 좌표: `sim/config/floor4_rooms.json` (401~406, 남쪽 벽 철문, 동쪽 끝부터 401)
- 실행: `python3 scripts/sim_demo.py --room 401 402 ...` (Isaac 창 + AUTO 유지, RViz는 `--rviz`)
- 웹 UI 조작: `ROS_DOMAIN_ID=107 ros2 launch amr_web_ui web_ui.launch.py use_sim_time:=true port:=8080`
- 2026-10-13 결과: 401~406 모두 도착, 실제 위치 오차 0.10~0.22m / 5~7°
- 403 첫 시도에서 긴 복도 제자리 회전 중 AMCL/TF 불일치로 localization READY가 약 3초 해제되어 목표가 중단됨(재시도에서는 재현되지 않음). 실차에서도 관찰할 항목.

## 전역 경로 평활화 (2026-10-13)
- 변경: `navigation.yaml` Smac2D smoother `w_smooth 0.3→0.95, w_data 0.2→0.02,
  max_iterations 1000→4000, refinement_num 4→6`. 플래너·inflation(1.6 m)·travel multiplier는 그대로.
- 원인: 8방향 격자 계단 + 벽이 꺾이는 세로 복도에서 경로가 정중앙(medial axis)을 그대로 따라 흔들림.
- 오프라인 경로 비교 `scripts/benchmark_global_planner.py NAME [overrides.yaml]` (domain 142, Isaac 불필요):
  6개 경로 합계 방향 반전 29→0회, 회전량 약 60% 감소, 길이 약 1% 감소, 최소 벽 거리 0.80→0.75 m
  (세로 복도 폭 1.6 m 병목 지점). Theta*, Lattice(diff 0.5 m)는 더 나빴음.
- Isaac 실주행 비교(floor4, run_sim_trial): 세로 복도 진입 회전량 150→76°, 401호 127→94°, 반전 2→0,
  소요 시간 동일, 최소 벽 거리 0.74→0.68 m(코너 안쪽), 가로 복도 중앙 편차 2.0→4.5 cm.
- 이 파일은 Jetson 실차와 공용이다. 실차 반영 시 코너 안쪽 여유를 확인할 것.

## 층간 이동(엘리베이터) 작업 (2026-10-13~)
설계: `docs/ELEVATOR_DESIGN.md`. 작업 전 백업: `backups/amr_ws_pre_elevator_20261013_230836.tar.gz`.

### 1단계 — 3F·5F 임시 지도·장면, 층 설정 파일
- 층 설정 `src/amr_system/amr_stack/config/floors.yaml` + 파서 `amr_stack/floor_config.py` (ROS 비의존).
  층 → 지도(실차·sim) → USD 레이어 → 엘리베이터 대기·하차 지점 → 호실. 지도 교체 시 이 파일만 수정.
- 3F·5F는 4F 복사본(`provisional_copy_of: "4F"`, 실차 `real_enabled: false`): `maps/G_{3,5}_floor/`,
  `maps/sim_floor{3,5}/` (각 폴더 `COPY_OF_4F.md`), `sim/layers/{3,5}_floor_tmp_from_4F_map.usda`.
- `sim/tools/build_floor_layers.py`: floors.yaml의 모든 층 레이어 생성. `build_scene.sh` 첫 단계를 이것으로 교체
  (4F 레이어는 바이트 단위 동일 확인).
- 엘리베이터 E1(4F 문 중심 9.975, 3.31, 카는 서쪽): 카 내부 1.87×2.23 m, 문 1.10 m, 층 벽~카 안쪽 벽 0.66 m.
  카 중심 (8.2, 3.31). 홀 대기 (10.575, 3.31, π), 하차 (10.775, 3.31, 0).
- 실차 영향: 없음 (기존 4F 지도·launch·navigation.yaml 불변, 신규 지도 폴더 설치만 추가).
- 확인: `test_floor_config.py` 32개 통과 (지점 여유 0.3 m 실차·sim 지도 모두, 카 내부 벽 없음,
  USD 문 위치 일치, 임시본=원본 해시, 잘못된 설정 18종 거부). amr_stack 67→99 통과, 나머지 패키지 기준과 동일.
  회귀: arena 성공(도착 0.032 m, 충돌 0, 복구 0), floor4 405호 성공(도착 0.164 m, 충돌 0, 복구 0) —
  `logs/elevator/stage1/`.
- 참고: `scripts/verify_documentation.py`는 작업 전부터 47개 실패(병합으로 docs/ 문서 추가, diagnostics/ 미동봉).
  `docs/ELEVATOR_DESIGN.md` 추가로 `two-canonical-documents` 항목 목록만 늘어남.
- 루트 `maps/`는 참조되지 않는 복사본이므로 3F·5F를 넣지 않았다. 실제 지도는 `src/amr_system/amr_stack/maps/`.

### 2단계 — 엘리베이터 신호 인터페이스, 시뮬레이션 신호 발생기
- 신규 `amr_elevator_interfaces`: msg `ElevatorBeacon`·`ElevatorCommand`·`ElevatorDoorState`·`FloorState`·`FloorEvent`,
  srv `SetCurrentFloor`, action `NavigateToFloorPose`.
- 실차 경로(RF 모듈 → STM32 → CAN → Jetson): `amr_can_bridge/elevator_protocol.py`(0x210 명령·0x310 이벤트,
  CRC-8, **ID·바이트 배치는 제안값 TODO**), 노드 `elevator_can_link`(자체 소켓, 0x310 필터, 기본 dry_run).
  `bringup.launch.py`에 `start_elevator_link:=false`(기본 꺼짐), `elevator_link_dry_run:=true` 추가.
  기존 `can_telemetry`·`cmd_vel_to_can`·`stm32_main_code.c`는 변경 없음.
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

### 3단계 — 층·상태 판별 (실차 공용)
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

### 4단계 — 엘리베이터 장면, 탑승·하차 주행
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

### 5단계 — 층간 미션, 지도 전환·재위치, 웹 UI
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

### 6단계 — 층간 시나리오 통합 시험
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
