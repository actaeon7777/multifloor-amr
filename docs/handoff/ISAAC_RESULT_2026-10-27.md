# Isaac PC 결과: 전역 inflation 1.0 장애물 옆 정지 확인 (2026-10-27)

지시 문서: `docs/handoff/ISAAC_CHECK_2026-10-27.md`. 지시 순서대로 0~3단계를 모두 실행했다.

## 환경

| 항목 | 값 |
|---|---|
| PC | `HP-PC`, x86_64, 24 코어, RTX 5080 |
| 커밋 | `ace6de4 docs: Isaac PC task for the global inflation obstacle-stall check` (4be83f9 이후) |
| Isaac Sim | 6.1.0 (`isaacsim-env`, headless, RTF 1.00) |
| ROS | Jazzy, STVL `ros-jazzy-spatio-temporal-voxel-layer 2.5.5` (이번에 설치) |

준비 중 특이사항
- 처음 빌드할 때 `amr_stack`이 실패했다. `build/amr_stack/maps/G_{3,5}_floor/`에 브랜치에서 지운 `*_tmp_from_4F.*`, `COPY_OF_4F.md` 파일을 가리키는 깨진 symlink가 남아 있었기 때문이다. 그 symlink만 지우고 다시 빌드해서 19개 패키지가 모두 빌드됐다. 코드는 바꾸지 않았다.
- 1단계 B를 위해 고친 `navigation.yaml`은 끝난 뒤 `git checkout --`으로 되돌렸고, `git diff`가 비어 있는 것을 확인했다.

## 1. 격리 ROS 시험 재현 (도메인 191)

```bash
source scripts/env.sh
export ROS_DOMAIN_ID=191 ROS_LOCALHOST_ONLY=1 AMR_RUN_NAV2_INTEGRATION=1
python3 -m pytest -q src/amr_system/amr_stack/test/test_navigation_pipeline_ros.py   # A·B 각 3회
# B: navigation.yaml 274행 inflation_radius 1.0 -> 1.6 (sed), 끝난 뒤 git checkout -- 로 복구
```

| 경우 | 통과 | 실패 | 실패 위치 | `Failed to make progress` (회차별) |
|---|---|---|---|---|
| A 전역 1.0 (현재) | 0/3 | 3/3 | `test_real_smac_dwb_mux_lb_takeover_and_recovery` 381행 `wait(result,65)` 시간 초과, 목표 RUNNING | 1, 3, 4 |
| B 전역 1.6 | 3/3 (`2 passed` ×3) | 0 | — | 0, 0, 0 |

- 다른 시험 `test_corridor_and_replan_when_new_obstacle_blocks_path`는 A와 B 모두 통과했다.
- A가 실패한 시점의 로봇 위치는 (0.15, 0.17), (0.20, 0.19), (0.24, 0.29)였다. 상자 앞에서 거의 움직이지 못했다.
- 시험 한 번에 걸린 시간은 A 163~171 s, B 142~153 s였다.

**판정:** x86에서도 Jetson과 똑같이 재현된다. 1.0이면 항상 실패하고 1.6이면 항상 통과한다. 원인은 Jetson CPU가 아니라 설정이다.

## 2. Isaac Sim `obstacle` 월드 (도메인 107)

```bash
cp src/amr_system/amr_stack/config/navigation.yaml runtime/isaac_check/a_infl10.yaml
# b/c/d: python3 + yaml로 해당 키 하나만 바꿔 저장 (docs/handoff/isaac_overrides/ 참조, 원본과 diff는 그 키 하나뿐임을 확인)
export ROS_DOMAIN_ID=107
for v in a_infl10 b_infl16 c_infl10_pathalign16 d_infl10_travel8; do
  for s in 1 2 3; do
    /usr/bin/python3 scripts/run_sim_trial.py --world obstacle --goal 3 0 0 --seed $s \
      --navigation-config runtime/isaac_check/$v.yaml --output reports/trials/isaac_check/${v}_s$s
  done
done
python3 scripts/compare_trials.py --baseline reports/trials/isaac_check/a_infl10_s*/metrics.json \
  --candidate reports/trials/isaac_check/<v>_s*/metrics.json
```

시나리오에 대한 주의: 상자는 sim 시간 10 s에 나타나는데, 목표는 초기화가 끝난 뒤인 sim 시간 12.9~15.3 s에 보내졌다. 그래서 실제로는 "주행 중 갑자기 나타난 장애물"이 아니라 **출발 시점에 이미 정면 1.1 m에 있던 장애물**을 돌아가는 상황이 됐다.

| 시도 | 도달 | 시간 s | 정지 시간 s | recovery | FMP | 최소 간격 m | 최종 위치 (x, y, yaw) |
|---|---|---|---|---|---|---|---|
| a_infl10 s1 | O | 94.1 | 53.5 | 3 | 2 | 0.126 | (2.96, 0.13, −7°) |
| a_infl10 s2 | X 시간 초과 | 101.1 | 52.5 | – | 5 | 0.134 | (0.69, 0.56, 30°) |
| a_infl10 s3 | X 시간 초과 | 101.0 | 49.4 | – | 5 | 0.126 | (0.62, 0.49, 33°) |
| b_infl16 s1 | O | 22.0 | 1.3 | 0 | 0 | 0.650 | (2.98, 0.19, −7°) |
| b_infl16 s2 | O | 26.8 | 1.3 | 0 | 0 | 0.653 | (2.98, 0.11, −7°) |
| b_infl16 s3 | O | 25.2 | 1.2 | 0 | 0 | 0.672 | (2.98, 0.13, −7°) |
| c_infl10_pathalign16 s1 | X 시간 초과 | 101.1 | 29.7 | – | 3 | 0.042 | (0.68, 0.48, 17°) |
| c_infl10_pathalign16 s2 | X 시간 초과 | 101.0 | 47.7 | – | 3 | 0.087 | (0.61, 0.46, 23°) |
| c_infl10_pathalign16 s3 | X 시간 초과 | 101.0 | 55.8 | – | 6 | 0.201 | (0.45, 0.42, −174°) |
| d_infl10_travel8 s1 | X 시간 초과 | 101.0 | 64.4 | – | 6 | 0.178 | (0.42, 0.44, −179°) |
| d_infl10_travel8 s2 | X 시간 초과 | 101.1 | 63.6 | – | 5 | 0.178 | (0.65, 0.57, 31°) |
| d_infl10_travel8 s3 | X 시간 초과 | 101.1 | 58.8 | – | 4 | 0.149 | (0.75, 0.61, 29°) |

- FMP는 `launch.log`에 나온 `Failed to make progress` 횟수다. 정지 시간은 `metrics.json`의 `command_stationary_time_s`다.
- 실패한 시도는 모두 상자(x 1.1~1.5, y ±0.3)의 왼쪽 앞인 (0.4~0.75, 0.42~0.61)에서 멈췄다. 1단계 시험과 같은 "장애물 옆 정지"다.
- 충돌: 성공한 시도는 `obstacle_contact_events`가 0이었다. 시간 초과한 시도는 그 값이 결과에 기록되지 않았다(예외 경로). 다만 ground truth 기준 최소 간격이 모두 양수(최소 0.042 m, c s1)여서 접촉은 없었다고 본다.
- a는 1/3만 성공했다. 그 1회도 94 s가 걸렸고 recovery를 3번 거쳤다.

`compare_trials.py` 결과(기준은 a). 세 후보 모두 게이트를 통과하지 못했다. 공통 원인은 반복이 5회 미만이고, 기준 a 자체가 `valid_for_comparison=false`라는 점이다. b와 d는 planner/costmap을 바꿔서 `fixed_inputs_sha256`도 다르다.

| 후보 | 시간 평균 (a → 후보) | 도착 거리 m | path_error_rms m | 각속도 변동 | 추가 사유 |
|---|---|---|---|---|---|
| b_infl16 | 98.7 → **24.7** | 1.65 → **0.15** | 0.028 → 0.077 | 13.6 → 6.8 | rms·max 경로 오차 10% 이상 증가 |
| c_infl10_pathalign16 | 98.7 → 101.0 | 1.65 → 2.46 | 0.028 → 0.024 | 13.6 → 11.6 | 전부 실패, 최소 간격 감소 |
| d_infl10_travel8 | 98.7 → 101.0 | 1.65 → 2.45 | 0.028 → 0.026 | 13.6 → 11.8 | 전부 실패 |

b의 경로 오차가 커 보이는 것은 b가 실제로 상자를 돌아 끝까지 갔기 때문이다(최대 |y| 1.34 m). a/c/d는 상자 앞에서 멈춰 있었으니 경로 오차가 작게 나온다. 경로 오차 증가는 추종 품질이 나빠졌다는 뜻이 아니다.

## 3. 전역 경로 오프라인 비교 (floor4 지도, static + inflation)

```bash
python3 scripts/benchmark_global_planner.py a_infl10
python3 scripts/benchmark_global_planner.py b_infl16 runtime/isaac_check/ov_b.yaml
python3 scripts/benchmark_global_planner.py d_infl10_travel8 runtime/isaac_check/ov_d.yaml
```

결과는 `logs/planner_bench/{a_infl10,b_infl16,d_infl10_travel8}.json`에 있다. c는 DWB만 바꾸므로 a와 같다.

| 경로 | 후보 | 길이 m | 꺾임 ° | 방향 역전 | 최소 간격 m | 평균 간격 m | 복도 중심 편차 m |
|---|---|---|---|---|---|---|---|
| spawn→401 | a 1.0 | 30.27 | 63.2 | 0 | 0.74 | 1.21 | 0.295 |
| | b 1.6 | 30.41 | 59.3 | 0 | 1.02 | 1.43 | 0.016 |
| | d travel 8 | 30.27 | 63.2 | 0 | 0.74 | 1.21 | 0.295 |
| 401→406 | a | 33.21 | 12.4 | 0 | 0.90 | 1.04 | 0.293 |
| | b | 33.39 | **47.6** | 0 | 1.00 | 1.35 | 0.015 |
| | d | 33.21 | 12.3 | 0 | 0.90 | 1.04 | 0.293 |
| 404→402 | a | 15.12 | 14.3 | 0 | 0.90 | 1.00 | 0.319 |
| | b | 15.21 | **35.2** | 0 | 1.00 | 1.26 | 0.073 |
| | d | 15.12 | 14.3 | 0 | 0.90 | 1.00 | 0.319 |
| 406→north | a | 22.93 | 116.9 | 0 | 0.53 | 1.13 | – |
| | b | 23.43 | 141.3 | 0 | 0.75 | 1.34 | – |
| | d | 22.93 | 116.9 | 0 | 0.53 | 1.13 | – |
| north→403 | a | 28.67 | 132.4 | 0 | 0.51 | 1.03 | 0.265 |
| | b | 29.60 | 152.4 | 0 | 0.75 | 1.25 | 0.058 |
| | d | 28.67 | 132.6 | 0 | 0.51 | 1.03 | 0.265 |
| spawn→mid_corr | a | 19.12 | 51.4 | 0 | 0.74 | 1.31 | 0.261 |
| | b | 19.31 | 43.6 | 0 | 1.14 | 1.51 | 0.011 |
| | d | 19.12 | 51.4 | 0 | 0.74 | 1.31 | 0.261 |

- "1.6이면 전역 경로가 이상해진다"는 숫자로도 일부 확인된다. 복도 직선 구간(401→406, 404→402)의 꺾임 합이 12~14°에서 35~48°로 2.5~4배가 된다. 길이는 0.1~0.9 m(최대 3%) 늘어난다. 다만 방향 역전은 어느 경로에서도 0이다. 1.6은 지그재그가 아니라 복도 중심선(편차 0.01~0.07 m)을 따라가느라 생기는 완만한 굽이다.
- 1.0의 직진성에는 대가가 있다. 경로가 복도 중심에서 약 0.26~0.32 m 벗어나 한쪽으로 치우친다.
- d(`cost_travel_multiplier` 8)의 경로는 a와 사실상 같다. 1.0일 때 복도 중심이 cost 0이라서 이 계수가 영향을 줄 곳이 없다. 2단계에서도 효과가 없었다.

## 해석

1. **설정이 원인이다.** x86 격리 시험에서 1.0은 0/3, 1.6은 3/3이었다. Jetson 결과와 같다.
2. **장애물 옆 정지를 없앤 후보는 b(전역 1.6) 하나뿐이다.** Isaac에서도 b는 3/3 성공했다. 평균 24.7 s, recovery 0, 최소 간격 0.65 m였다. 반면 a, c, d는 9회 중 8회가 상자 왼쪽 앞에서 멈춘 채 시간 초과됐다. 성공한 나머지 1회(a s1)도 53 s 동안 멈춰 있었고 recovery를 3번 거쳤다.
   - c(PathAlign 16)는 효과가 없었다. 최소 간격만 0.04 m까지 줄었다.
   - d(travel 8)도 효과가 없었다.
   - 따라서 원인은 DWB가 경로를 정렬하는 가중치나 Smac의 주행 비용이 아니다. 전역 경로가 상자에 얼마나 가깝게 붙느냐(1.0이면 장애물 옆 cost가 낮아 바짝 붙는 경로가 나옴)가 결정적이라고 본다.
3. **직진성은 지킬 수 없었다.** 이번 후보 중 1.0의 직진성을 유지하면서 정지를 없앤 것은 없다. 1.6의 대가는 복도 꺾임 2.5~4배, 길이 최대 +3%다. 방향 역전은 없다.

## 다음 실차 시험 권장값

**전역 `inflation_radius: 1.6`** (a1b3d3a 이전 값). 1단계 3/3, 2단계 3/3으로 정지를 없앤 것이 확인된 값은 이것 하나뿐이다. 복도 직진성 손실은 3단계 표 정도이고, 방향 역전은 없다.

이번에 시험하지 않았지만 1.0과 1.6 사이에서 직진성을 일부 지키는 방법의 후보:
- 전역 1.6을 유지한 채 `cost_scaling_factor`를 올려(예: 3~4) cost가 장애물에서 더 빨리 줄게 하는 방법
- 1.0에 Smac `cost_penalty`를 올리는 방법

둘 다 같은 1·2단계 절차로 확인해야 한다. 전역 1.3은 Jetson에서 이미 실패했으니 반경만 1.0~1.6 사이로 바꾸는 방법은 기대하기 어렵다.

## 파일

- 변형·오버라이드 YAML: `docs/handoff/isaac_overrides/` (`a_infl10`, `b_infl16`, `c_infl10_pathalign16`, `d_infl10_travel8`, `ov_b`, `ov_d`)
- 원자료(커밋 안 함, gitignore 대상): `reports/trials/isaac_check/*`, `logs/planner_bench/*.json`, `runtime/isaac_check/`
