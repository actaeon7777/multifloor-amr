# rl — 로컬 주행 정책 강화학습 (Isaac Lab)

Nav2 `FollowPath`의 DWB 자리를 대체할 학습형 로컬 주행 정책. 학습은 Isaac Lab, 검증은 기존
Isaac + Nav2 시나리오(`scripts/run_sim_suite.py`, `scripts/compare_trials.py`)로 한다.
`src/` 밖에 있어 colcon 빌드 대상이 아니다.

## 환경

| 항목 | 값 |
|---|---|
| Isaac Lab | v3.0.0-EA (`~/IsaacLab`, 태그 고정) |
| Python 환경 | `~/IsaacLab/.venv` (uv). 주행 스택의 `~/isaacsim-env`와 분리 |
| Isaac Sim | 6.1.0.0 (pip, `--extra isaacsim`) |

```bash
cd ~/IsaacLab && OMNI_KIT_ACCEPT_EULA=YES uv sync --extra isaacsim --python 3.12
uv pip install --python ~/IsaacLab/.venv/bin/python -e ~/amr_ws/rl   # isaaclab.tasks 엔트리포인트 등록
```

## 구성

| 경로 | 내용 |
|---|---|
| `tools/export_robot_usd.py` | `sim/amr_flat_test.usda`의 물리 설정(구동 gain, 관절 저항, 바퀴 재질·접촉)을 로봇 하나짜리 USD(`assets/amr_rl.usda`, 생성물)로 옮긴다. `sim/scripts/build_scene.py` 이후 실행 |
| `amr_rl/robot.py` | Isaac Lab `ArticulationCfg`. 바퀴·윤거·속도 상한은 `sim/config/physics.json`과 `control_mux.yaml`에서 읽는다 |
| `amr_rl/command.py` | 행동 → (v, ω) → 바퀴 목표. `control_mux` 비율 축소와 시뮬레이션 `WheelCommandLimiter` 램프를 배치 텐서로 재현 |
| `amr_rl/scan.py` | 스캔 bin 최소값 풀링, footprint 여유. 학습과 ROS 추론 노드가 같은 함수를 쓴다 |
| `amr_rl/layout.py` | 에피소드 배치(시작·목표·박스) 샘플링 |
| `amr_rl/tasks/` | `AMR-Nav-Arena-v0` (v0: 10 × 10 m 아레나 + 박스 0~5개, 목표점 입력) |
| `scripts/validate_motion.py` | 로봇 응답을 `reports/runtime_validation.json`과 비교 (전진·후진·제자리 회전, 10 % 이내) |
| `scripts/smoke_env.py` | 학습 전 환경 점검: 스캔 범위, 충돌 판정, 리셋, 처리량 |
| `scripts/evaluate.sh` | 체크포인트 평가: 노이즈 없는 정책, env당 1 에피소드, 성공·충돌·시간 초과 |
| `scripts/analyze_collisions.sh` | 충돌 분석: 부딪힌 면·동작·시점·장애물·경로 여유별 분류, 장면 그림(`logs/rl/collisions_<run>_<model>/`) |
| `amr_rl/shim.py` | Nav2 RotationShimController(navigation.yaml 값) 재현. 평가 전용: `evaluate.sh`·`analyze_collisions.sh`의 `--shim`(이전 설정은 `--shim_closed_loop --shim_every_path`) |
| `tests/` | Isaac 없이 도는 단위 시험 (`command`, `scan`, `layout`) |

## 실행 순서

셸 프로필이 ROS를 source하므로 Isaac Lab venv에 ROS 경로가 섞인다(pytest 플러그인 충돌 등).
Isaac Lab 명령은 `PYTHONPATH`·`AMENT_PREFIX_PATH`·`LD_LIBRARY_PATH`를 비우고 실행한다.

```bash
export OMNI_KIT_ACCEPT_EULA=YES
LAB="env -u PYTHONPATH -u AMENT_PREFIX_PATH -u LD_LIBRARY_PATH"
PY=~/IsaacLab/.venv/bin/python
~/isaacsim-env/bin/python rl/tools/export_robot_usd.py           # sim/amr_flat_test.usda 이후
$LAB $PY -m pytest -q rl/tests
$LAB $PY rl/scripts/validate_motion.py --viz none --device cuda:0 --output logs/rl/validate_motion_gpu.json
$LAB $PY rl/scripts/smoke_env.py --viz none --num_envs 256
$LAB ~/IsaacLab/.venv/bin/isaaclab train --rl_library rsl_rl --task AMR-Nav-Arena-v0 --viz none
```

학습 결과는 `logs/rsl_rl/amr_nav_arena/<시각>/`(git 제외)에 저장된다.

평소에는 아래 스크립트를 쓴다(위 환경 변수 처리를 포함).

| 스크립트 | 용도 |
|---|---|
| `rl/scripts/train.sh [hydra 인자]` | 백그라운드 학습. 로그 경로·PID 출력, 터미널을 닫아도 계속 |
| `rl/scripts/watch.sh [--num_envs 4] [--show_scan]` | 별도 프로세스 GUI. 최신 run의 최신 체크포인트를 10초마다 확인해 교체. 창을 닫아도 학습에 영향 없음 |
| `rl/scripts/tensorboard.sh` | 학습 곡선 (http://localhost:6006) |

## 검증 결과 (2026-10-16)

| 항목 | 결과 |
|---|---|
| 로봇 응답 GPU (기준 `reports/runtime_validation.json`) | 전진 0.03 %, 후진 0.01 %, 회전 1.9 % / 1.8 % 차이, 질량 15.0 kg |
| 로봇 응답 CPU | 전진 0.07 %, 후진 0.01 %, 회전 0.01 % / 0.2 % 차이 |
| 스캔 vs 해석해 (256 env, 184,320빔) | 99 백분위 오차 0.07 mm, 불일치 1빔(박스 모서리 스침) |
| 스모크 (256 env, 전속 직진 20 s) | 충돌 513, 도착 5, NaN·이탈 없음, 약 4,800 env-step/s |
| 학습 80 iteration (약 2분) | 최근 1,024 에피소드 성공률 0 → 63 %, 충돌률 53 → 10 % |

GPU 회전 1.9 % 차이는 CPU에서는 0.2 %라 GPU PhysX 차이로 본다. 정확한 원통 설정(`configure_runtime`)은 GPU 결과를 바꾸지 않는다.

## Isaac Lab 3.0.0-EA에서 피한 문제

- **큰 바닥 상자**: 1000 m 바닥 하나를 공유하면 GPU PhysX에서 약 절반의 env가 3.4 cm 가라앉았다(60 m는 정상). env마다 12 m 바닥 타일을 둔다.
- **다중 매치 ray-cast 대상**: `Wall_[^/]*`처럼 env마다 여러 prim에 맞는 식 하나를 쓰면 env별 메시 표가 전치된다(env 0에 모든 env의 `Wall_N`). prim마다 식을 하나씩 둔다.
- **`ray_alignment="yaw"`**: 빔이 로봇 yaw를 따라 돌지 않았다(오차 = −yaw). `"base"`를 쓴다.
- **벽 ray-cast 대상**: 벽도 박스와 같이 kinematic rigid body + `track_mesh_transforms=True`(clone plan 기반 env 매핑)로 둔다. 위 두 문제를 고친 뒤 정적 대상으로 되돌려 다시 확인하지는 않았다.
- **리셋 높이**: articulation root가 `base_link`라 리셋 자세는 `base_link` 높이(0.088 m)로 쓴다.

## 과제 정의 (v1, `AMR-Nav-Arena-v0` 태스크 ID 유지)

- 맵: 10 × 10 m 아레나 + 내부 벽 0~4개(6 m·3 m, 막다른 곳·방·복도) + 상자 0~5개. 레이아웃 4,096개를 미리 만들고
  약 100 iteration마다 1/4씩 새로 만든다(`amr_rl/layout.py`).
- 전역 경로: 레이아웃마다 Smac 2D + global_costmap 규칙(`robot_radius 0.44`, `inflation_radius 1.6`,
  `cost_scaling_factor 2.0`, `cost_travel_multiplier 4.0`)으로 목표까지의 경로 거리장을 계산한다
  (`amr_rl/planner.py`, 16방향 wavefront). Smac처럼 장애물 0.44 m 이내 칸은 경로에 쓰지 않는다. 그 안쪽
  0.15~0.44 m 띠는 장애물에서 멀어지는 방향으로만 값을 채워, 로봇이 들어가도 장이 유한하고 경로가 계획된 쪽으로
  되돌아 나오게 한다. 경로는 이 장의 경사를 따라간다.
- 시작·목표점은 장애물에서 0.55 m 이상 떨어진다(목표 칸이 0.44 m 밖에 있고 제자리 회전 반경 0.43 m를 넘게).
- 관측 169: 스캔 72 bin × 2프레임(0.5° 720빔, 10 m), 목표 거리·방향(sin, cos), 목표 heading 오차(sin, cos),
  경로점 8개(0.25 m 간격, 로봇 좌표), 현재 (v, ω), 직전 행동
- 행동 2: v ∈ [0, 0.5] m/s, ω ∈ [−0.6, 0.6] rad/s (DWB 한계), 정책 단계마다 DWB `acc_lim` 변화 제한
- 성공: `StoppedGoalChecker`와 같게 0.15 m·0.15 rad 이내이고 0.03 m/s·0.05 rad/s 미만으로 정지
- 보상: 경로 거리 감소(+2/m), 도착 +10, 충돌 −10, 시간 −0.01/step, 장애물 25 cm 이내 접근(−0.1까지 선형),
  행동 변화량²(−0.01), 목표 0.4 m 이내에서 heading 오차 감소(+1/rad)
- 물리 200 Hz, 정책 20 Hz, 에피소드 60 s, 학습 환경 1024개(기본값, 학습 VRAM 약 3.3 GB, iteration당 약 1.4초)
- 300 iteration 시험: 성공률 0 → 92 %, 충돌률 80 → 7 % (2026-10-16)
- 충돌 분석(`rl/scripts/analyze_collisions.sh`, 2026-10-16, 경로 규칙 수정 전 v1·v1_stdfix): 충돌 7.7~8.2 % 중
  약 68 %가 경로 최소 여유 0.45 m 미만 에피소드(경로 거리장이 반폭 0.287 m, 차단 0.2 m 기준이라 로봇이 못 지나가는
  틈으로도 경로가 났다). 경로 여유 0.45 m 이상에서는 충돌 약 2.7 %, 대부분 출발 직후 고속 선회 중 앞 모서리 접촉
- v2_smac(경로 규칙 수정 후 1000 iteration, 2026-10-16) model_999, 3,072 에피소드: 성공 97.7 %, 충돌 1.0 %(31건 중 26건이
  출발 직후 고속 선회), 시간 초과 1.3 %, 도착 17.7초. 평가에만 이전 설정 shim(`--shim --shim_closed_loop --shim_every_path`, 스택 BT처럼 약 10.7초마다 새 경로)을 넣으면
  성공 95.4 %, 충돌 0.1 %, 시간 초과 4.6 %(전부 목표 0.3 m 안: 새 경로마다 shim이 목표점 쪽으로 돌려 최종 정렬과 반복),
  도착 27.2초. `closed_loop: false` + `rotate_to_heading_once: true`(현재 navigation.yaml, `--shim`)면 성공 99.0 %,
  충돌 0.07 %, 시간 초과 0.9 %, 도착 20.5초. closed_loop shim은 측정 각속도 + 0.04 rad/s로 명령을 제한하는데 시뮬레이션의
  제자리 회전은 명령의 약 81~85 %만 나와 0.4 rad/s에 도달하지 못한다(Nav2 전체 스택 시뮬레이션에서도 명령 0.26, 실제 0.22 rad/s)
- 아직 없음: 센서·명령 지연과 노이즈, 물리 무작위화, 움직이는 장애물, 실제 지도 기반 레이아웃 — 다음 단계
