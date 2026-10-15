# Isaac Sim 시뮬레이션

실차와 같은 제어 스택(Nav2 → navigation_manager → control_mux)을 PC에서 Isaac Sim 6.1.0 PhysX 센서 폐루프로 실행한다.
목적은 반복 시험·기록·비교이며, 센서·물리 모델은 **실물 보정 전**이다. 시뮬레이션 수치로 실물의 제동거리나 절대 위치 정확도를 확정하지 않는다.
층간 이동 시뮬레이션은 [층간 이동](ELEVATOR.md), 과거 측정은 [실험 기록](EXPERIMENTS.md)에 있다.

## 1. 준비

Ubuntu 24.04 x86_64, ROS 2 Jazzy, Isaac Sim 6.1.0(현재 pip 빌드 6.1.0-rc.26). 시스템 Python은 `/usr/bin/python3`이다.

```bash
sudo bash scripts/install_ros_jazzy.sh        # 새 PC에서 한 번
sudo apt-get install -y ros-jazzy-navigation2 ros-jazzy-nav2-bringup \
  ros-jazzy-robot-localization ros-jazzy-test-msgs ros-jazzy-slam-toolbox \
  ros-jazzy-xacro ros-jazzy-control-msgs liboctomap-dev python3-can
scripts/link_pip_isaacsim.sh                  # pip Isaac(~/isaacsim-env)을 runtime/isaac-sim-6.1.0에 연결
./scripts/build_ros.sh                        # rosdep 의존성 + colcon 빌드 + 시험
python3 -m venv runtime/urdf-tools && runtime/urdf-tools/bin/python -m pip install -r sim/config/requirements-tools.txt
./build_scene.sh                              # 층 레이어·물리 URDF·건물 장면·AMR USD 생성
```

- 다른 Isaac 설치본을 쓰려면 `ln -s /설치경로 runtime/isaac-sim-6.1.0`(그 경로에 `python.sh`·`isaac-sim.sh` 필요).
  루트 `env.sh`가 `ISAACSIM_PATH`를 덮어쓰므로 export만으로는 바뀌지 않는다.
- `scripts/link_pip_isaacsim.sh`는 Isaac 내장 Jazzy 라이브러리(`exts/isaacsim.ros2.core/jazzy/lib`)를 `LD_LIBRARY_PATH`에 넣는다. 없으면 ROS 2 브리지가 뜨지 않는다.
- Isaac 설치본(`runtime/`), 생성 USD(`sim/assets/`, `sim/amr_flat_test.usda`, `sim/generated/`)는 git에서 제외한다. 새 PC에서는 `./build_scene.sh`로 다시 만든다.
- 지도 파일이 처음 생성됐다면 `amr_stack`을 다시 빌드해 설치 경로에 반영한다.
- 작업공간을 다른 위치로 옮기면 `build/`·`install/`을 새로 만든다. 스크립트와 USD 자산 경로는 폴더 위치를 따라간다.

## 2. 실행

```bash
./scripts/experiment.sh sim                               # 4층 + 조이스틱 + EKF + AMCL + navigation, 도메인 107
./scripts/experiment.sh sim world:=building start_floor:=3F   # 3개 층 + 엘리베이터
./scripts/experiment.sh sim headless:=true                # Isaac 창 없이
# 별도 터미널: 웹 UI
source scripts/env.sh
ROS_DOMAIN_ID=107 ros2 launch amr_web_ui web_ui.launch.py use_sim_time:=true port:=8080
```

| 명령 | 용도 |
|---|---|
| `python3 scripts/sim_demo.py --room 401 402` | 4층 호실 순회 데모 (Isaac 창 + AUTO 유지, `--rviz`) |
| `./run_amr.sh --test` | ROS 없이 평지 구동 확인 |
| `./run_amr.sh --world floor4` | ROS 없이 상세 4층 장면 |
| `ros2kill [--dry-run]` | 현재 PC·계정의 모든 ROS 2 노드·launch·웹 서버·Isaac 브리지를 SIGKILL (도메인 무관, 다른 PC 영향 없음) |

- `experiment.launch.py`는 기본 sim이며 각 하위 launch에 `use_sim_time`을 전달한다. GUI에서 일시정지·리셋하면 시험이 종료된다.
- PC의 자동 시험기는 실물 모드를 지원하지 않고, 실물 도메인(7)에서는 실행을 거부한다.
- 웹만 써도 실행 흐름은 같다. Isaac 창을 띄우지 않으려면 `headless:=true`.

## 3. 장면

| 월드 | 장면 | 내용 |
|---|---|---|
| `floor4` (기본) | `sim/capstone_amr_4_floor.usda` | 주석 이미지 기반 3 m 벽, 철문 15개, 엘리베이터 문 1개. 엘리베이터 앞 (11.5, 3.31, 0)에서 시작 |
| `building` | `sim/building_G.usda` | 3F·4F·5F 수직 적층 + kinematic 엘리베이터 카 ([층간 이동](ELEVATOR.md)) |
| `arena`, `straight`, `corner_left/right`, `s_curve`, `narrow`, `obstacle` | 합성 시험장 | `scripts/build_sim_map.py`가 ROS 지도와 PhysX 벽 생성 |

- GUI에서 `O`는 철문, `P`는 엘리베이터 문을 전환한다(`run_amr.sh`와 ROS 실행기에서 자동 등록).
  편집기로 USD를 직접 열었을 때는 Script Editor에서 실행한다:

  ```python
  from pathlib import Path
  import omni.usd
  scene = Path(omni.usd.get_context().get_stage().GetRootLayer().realPath)
  exec((scene.parent / "tools/map_door_controller.py").read_text())
  ```

- `build_scene.sh`는 상세 장면의 정적 충돌 형상을 레이저 높이에서 투영한 `src/amr_system/amr_stack/maps/sim_floor4/`도 만든다.
  움직이는 문짝은 정적 지도에서 빼고 실시간 LiDAR 장애물로 처리한다. 실물은 계속 `maps/G_4_floor/` 지도를 쓴다.
- 원본 지도·영역 주석을 고친 뒤 맵만 다시 만들려면 `python3 sim/tools/generate_occupancy_usd.py`, AMR USD까지는 `./build_scene.sh`.
- 바닥은 정적 삼각형 메시(`physics:approximation = "none"`)다. Cube 충돌 근사는 둥근 바퀴와의 접촉에서 차체를 떨게 했다.
  접촉 보고(`PhysxContactReportAPI.reportPairs`)는 장애물에만 둔다.
- 타이어는 1 mm 둥근 모서리, 접촉 탄성·감쇠를 쓴다. 차체 위치·속도·기울기를 강제 고정하지 않는다. 시뮬레이션 안정화용 값이며 실측 보정값이 아니다.

### 원본과 참조

| 용도 | 경로 |
|---|---|
| AMR URDF와 메시 | `src/amr_description/` ([Isaac 임포트 안내](../src/amr_description/README_ISAAC_SIM.md)) |
| 원본 형상 해시 | `reports/source_snapshot.json` (`prepare_physics_urdf.py`가 비교. 형상을 의도적으로 바꾸면 함께 갱신) |
| 현재 물리 설정 / 센서 설정 | `sim/config/physics.json` / `sim/config/sensors.json` |
| 모터 파라미터·과거 접촉 설정 근거 | `sim/reference/wa172e_actuator_parameters.yaml`, `amr_realistic_overrides.usda` |
| 층 지도·영역 주석 PNG | `src/amr_system/amr_stack/maps/` |
| 층 맵 USD | `sim/layers/*.usda` |
| C++ LiDAR | `sim/native/` ([설계·빌드](../sim/native/README.md)) |

## 4. 제어·센서 모델

- `amr_sim_bridge`가 Isaac 프로세스 안에서 ROS DDS에 참여해 실차 드라이버 대신 같은 토픽을 발행하고, 바퀴 관절의 속도 목표만 받는다.

  | 방향 | 토픽 |
  |---|---|
  | 제어기 → Isaac | `/cmd_vel` |
  | Isaac → 제어기 | `/clock`, `/wheel/odometry`, `/joint_states`, `/imu/data_raw`, `/scan`, `/sim/ground_truth`(평가 전용), `/sim/status` |

- TF는 실차와 같이 AMCL(map→odom), EKF(odom→base_footprint), robot_state_publisher가 맡는다. 브리지는 TF를 발행하지 않는다.
- `control_mux.yaml`의 기구 치수와 1.8 rev/s 제한이 DWB critic과 양쪽 구동 경로의 공통 기준이다.
- 구동기 모델: 50 ms 갱신, 벽시계 300 ms 명령 단절 정지, 상위 램프 없음. 물리 200 Hz(TGS).
- 휠: PhysX 실제 바퀴 회전량을 `WheelOdometry`에 전달한다. 양쪽 URDF 축이 같은 부호라 CAN 왼쪽 반전은 적용하지 않는다.
- IMU: base_link의 CAD 위치. 특정 가속도(정지 시 +g)·각속도, 자세 미측정 규약. 실물 gyro bias를 빼지 않는다.
- LiDAR: PhysX 충돌 형상에 대한 회전 스캔, 3,240점, 0.05–30 m, `laser` 프레임. 각 광선 취득 시각의 센서 자세를 반영하고
  첫 광선 시각을 stamp로, `time_increment = scan_time / (N-1)`을 쓴다([LaserScan 정의](https://github.com/ros2/common_interfaces/blob/jazzy/sensor_msgs/msg/LaserScan.msg)).
  200 Hz 물리 단계로 광선 시점을 양자화한다. 0.4 m/s·0.6 rad/s에서 한 스캔 동안 약 4 cm·3.4°가 바뀌므로 회전 스캔 시점을 무시하지 않는다.
- 주기·노이즈는 `sim/config/sensors.json`이 정한다(현재 휠 10 Hz, IMU 10 Hz, LiDAR 8.8 Hz, 노이즈 0).
  2026-10-04에 실물 bag 기준 휠 100 Hz·LiDAR 10 Hz로 맞췄다는 기록이 있으나 현재 파일 값과 다르다([실험 기록](EXPERIMENTS.md#2026-10-04--센서-주기-정합-시뮬레이션)). 재확인이 필요하다.
- 시뮬레이션 EKF는 `predict_to_current_time=true`(실물 기본값 false)로 측정 사이 상태를 현재 시각으로 예측해 30 Hz TF를 낸다.

### 조이스틱

- 시뮬레이션 실행에도 조이스틱이 포함된다(LB 수동, RB AUTO). 자동 시험은 `start_joy:=false`로 하드웨어 입력과 분리한다.
- `experiment.sh sim`은 `joy_teleop.yaml`의 `device_name`과 같은 장치를 자동 선택한다. 같은 이름이 여러 개면 실제 입력이 들어오는 장치를 기다리므로
  쓸 컨트롤러의 LB를 한 번 눌렀다 떼거나 스틱을 움직인다. 선택 로그가 나온 뒤 LB를 다시 누르면 수동 주행한다. 대기는 별도 프로세스라 다른 실행을 막지 않는다.
- 장치 번호는 저장하지 않는다. Bluetooth 재연결로 끊기면 스택을 재시작한다. 수동 지정은 `joy_device_id:=1`(자동 선택보다 우선).
- 합성 조이스틱 시험(최대 속도·회전·급반전·LB 해제·연결 단절 정지, 시뮬레이션 전용):

  ```bash
  ./python.sh sim/scripts/validate_stability.py
  ./scripts/experiment.sh sim headless:=true manual_test:=true
  ```

  결과는 `reports/floor4_stability.json`, `reports/floor4_manual.json`.

## 5. 시나리오와 자동 시험

`sim/config/scenarios.json`에 직선·좌우 코너·S자·1.2 m 통로·장애물 등장 시나리오와 층간 시나리오(`floor_scenarios`)가 있다.

```bash
source scripts/env.sh && export ROS_DOMAIN_ID=107
/usr/bin/python3 scripts/run_sim_trial.py --world arena|floor4 --output reports/trials/my_trial
/usr/bin/python3 scripts/run_sim_suite.py \
  --navigation-config src/amr_system/amr_stack/config/navigation.yaml \
  --scenario straight --scenario corner_left --output reports/trials/baseline_suite
# --scenario 생략 시 전체 6종, --final-seeds면 고정된 10개 센서 노이즈 시드
/usr/bin/python3 scripts/compare_trials.py \
  --baseline reports/trials/baseline_suite/straight_*/metrics.json \
  --candidate reports/trials/candidate_suite/straight_*/metrics.json
```

- 각 시험은 새 프로세스·같은 시작 자세로 시작한다. AMCL 초기 자세는 시험 조건으로 넣고 ground truth를 피드백하지 않는다.
- 저장하는 seed는 가상 센서 RNG에만 적용된다(Jazzy AMCL에는 `random_seed`가 없다). 같은 seed가 ROS 스케줄링이나 AMCL 궤적까지 같게 하지는 않는다.
- 시험 폴더에 rosbag, 실제 ROS 파라미터, 적용 Nav2 YAML, 소스·설정 해시, 추정·ground-truth 궤적, 지표를 저장한다. 고용량 `/evaluation`은 제외한다.
- 비교기 통과 조건: 같은 세계·seed·고정 설정, 5회 이상, 충돌·명령 제한 위반 0, 전회 목표 도달, RMS 경로 오차 또는 각속도 명령 변동 20 % 개선,
  다른 지표 10 % 이내 악화. 센서 지연·제어 주기 누락·실시간 비율 문제가 있는 기록은 제외하고, 필수 측정이 없으면 통과시키지 않는다.
  품질 기준: scan age 0.3 s, odometry age 0.25 s, Nav2 명령 간격 75 ms 초과, RTF 0.9 미만, 추정 pose 누락, 0.1 s 안에 0.2 m/0.35 rad 넘는 점프.
- footprint 여유거리는 지도 벽과 2D footprint의 기하학적 거리이고, 실제 접촉은 PhysX 접촉 이벤트로 판단한다.
- 현재 DWB 기준 YAML의 실물 이식용 개선 후보는 아직 선정하지 않았다. 전체 시나리오 5회·최종 10 seed 검증도 완료로 표시하지 않는다.

진단 도구:

| 도구 | 용도 |
|---|---|
| `scripts/benchmark_runtime.py [--initialization automatic] [--headless true] [--render-loading synchronous]` | TIM 초기화부터 Nav2 주행까지 실행 성능 |
| `scripts/benchmark_global_planner.py NAME [overrides.yaml]` | 오프라인 전역 경로 비교 (도메인 142, Isaac 불필요) |
| `scripts/benchmark_rolling_scan_quality.py`, `benchmark_manual_localization_latency.py`, `benchmark_amcl_scan_range.py`, `benchmark_tim_localization.py` | 위치 추정 재현 |
| `./python.sh sim/scripts/probe_floor_contact.py --assert-stable \| --verify-reports \| --floor legacy-box` | 바닥 접촉 회귀, 장애물 접촉 보고, 이전 충돌 형상 재현 |
| `./python.sh sim/scripts/probe_gui_runtime.py --render-loading nonblocking --exercise-doors --trace` | GUI 갱신·문 개폐 추적 |
| `scripts/verify_web_restart.py --output <폴더>` | 웹 UI 유지한 채 Isaac 재시작 2회 (도메인 107, 기존 스택 중단) |

- `AMR_SIM_DYNAMICS_TRACE=/절대/경로/trace.csv`: 물리 단계마다 차체 자세·속도·적용 명령 기록.
- `AMR_SIM_SENSOR_CONFIG` 또는 `run_ros_sim.py --sensor-config`: 그 실행에만 다른 센서 설정 적용.

### 기록과 비교

- 기존 스택 운용 중에는 `AMR_EXPERIMENT_MODE=sim ./scripts/record_trial.sh 이름`을 쓴다.
- 파라미터 수집은 `capture_parameters.py`의 한 DDS 접속으로 대상 노드를 동시에 읽고, 벽시계 8초 제한을 둔다. sim에서는 `cmd_vel_to_can`을 조회하지 않는다.
  미설정 파라미터는 `params/summary.json`, 노드 누락·응답 실패는 `.failed.txt`로 남는다.
- Jazzy rosbag2가 `action_msgs/msg/GoalStatusArray`의 메시지 정의를 비워 두는 경고를 낼 수 있다. 직렬화 데이터는 기록되며 해당 인터페이스가 설치된 환경에서 읽힌다.

## 6. 성능 관련 설정

현재 구성의 근거 측정은 [실험 기록 2026-10-03](EXPERIMENTS.md#2026-10-03--시뮬레이션-실행-성능과-lidar-구현).

```text
PhysX 200 Hz ─ 바퀴 / IMU ─ EKF 30 Hz ─ odom → base_footprint
           └ 2D LiDAR ─ AMCL ─ map → odom
웹 목표점 ─ navigation_manager ─ Nav2/DWB 20 Hz ─ control_mux ─ 가상 바퀴 구동
```

- **LiDAR**: 기본은 PhysX C++ 배치 질의(`sim/native/amr_lidar_native.cpp`, pybind11 2.13.6). 거리 질의·자기 충돌체 제외·가장 가까운 hit·미검출을 C++에서 처리한다.
  첫 실행에 GCC로 자동 빌드하고 캐시를 쓴다. 검증한 `omni.physx 110.3.2` 외 버전에서는 오류로 멈추며, `AMR_LIDAR_BACKEND=python`으로 Python 경로를 쓸 수 있다.
  실제 사용 구현은 `/sim/status`의 `lidar_backend`. Isaac Physics RaycastSensor API는 설치본의 `reading.depths` 결함 때문에 쓰지 않는다.
- **GUI 로딩**: `SimulationApp(sync_loads=False)`를 생성 시점부터 적용해 RTX 자산 로딩 대기가 물리·센서·`/clock`을 밀지 않게 한다.
  `AMR_RENDER_READY` 로그와 `/sim/status`의 `render_loading=nonblocking`으로 확인한다. 픽셀 단위로 모든 자산 로딩 완료가 필요한 데이터 수집은
  `AMR_RENDER_LOADING=synchronous ./scripts/experiment.sh sim`을 쓴다. 비동기 렌더링(`/app/asyncRendering`)은 false.
- **headless**: 숨겨진 기본 뷰포트 렌더링을 끈다. LiDAR·IMU는 PhysX 센서라 화면 없이 동작한다.
- 웹 지도·costmap 색상 변환은 Pillow 테이블 연산을 쓴다. PhysX 작업 스레드 수는 기본값을 유지한다
  ([NVIDIA 성능 문서](https://docs.isaacsim.omniverse.nvidia.com/6.1.0/reference_material/sim_performance_optimization_handbook.html)).

## 7. 실물 보정 계획

- 실물 기록: 정지 60초, 0.1·0.2·0.4 m/s 직진·정지, ±0.2·±0.4 rad/s 회전·방향 전환 각 3회. 모델 식별용과 검증용을 분리한다.
- `scripts/analyze_response.py bags/실험/bag --output response.json`: 토픽 주기·누락, 명령/바퀴 겉보기 응답 지연. 지연에는 telemetry·필터·램프가 섞여 있어
  ODrive 통신 지연만으로 해석하지 않으며 물리 파라미터를 자동 변경하지 않는다.
- `scripts/validate_model_response.py`: 검증 기록의 정상속도·상승시간·정지시간·정지거리를 측정 불확실성과 함께 받아 10 % 또는 불확실성 이내인지 검사한다.
- 미지의 관절 Coulomb 마찰은 0을 유지하고, 질량·관성을 궤적 맞추기용 자유 변수로 쓰지 않는다.
- 실물 비교는 0.2 m/s에서 시작하고 통과 후 0.4 m/s 범위에서 기준·후보를 번갈아 평가한다. 독립 측정 장비가 없으면 추정 궤적 기준으로 기록한다.
- 실제 STM32 펌웨어 identity와 ODrive readback 값은 아직 확인하지 않았다.

근거: [Isaac Sim 6.1 ROS 설치](https://docs.isaacsim.omniverse.nvidia.com/6.1.0/installation/install_ros.html),
[ROS Jazzy Ubuntu 설치](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html),
[Jazzy AMCL 소스](https://github.com/ros-navigation/navigation2/blob/jazzy/nav2_amcl/src/amcl_node.cpp).
