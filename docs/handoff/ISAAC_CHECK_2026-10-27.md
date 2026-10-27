# Isaac PC 작업 지시: 전역 inflation 1.0 장애물 옆 정지 확인 (2026-10-27)

이 문서는 Isaac Sim PC에서 작업할 Claude 에이전트용이다. 위에서부터 순서대로 실행하고, 마지막 "보고" 절차대로 결과를 올린다.

## 배경

- 브랜치는 `no-resume-approval`, 기준 커밋은 `4be83f9`(WIP)다. Jetson 실차에서는 아직 검증하지 않은 코드다.
- 문제: 격리 ROS 시험 `src/amr_system/amr_stack/test/test_navigation_pipeline_ros.py::test_real_smac_dwb_mux_lb_takeover_and_recovery`가 Jetson에서 계속 실패한다.
  - 조건: LB 인계 → 해제 뒤 새 목표(2.3 m, 0.4 m 상자 옆 경유). 상자 0.6 m 앞, 50° 틀어진 자세에서 출발한다.
  - 증상: DWB가 v=0, w=±0.04~0.12 rad/s만 낸다. 10 s마다 `Failed to make progress`가 나오고, 이것이 3번 반복되다 목표가 실패한다.
- Jetson에서 하나씩 되돌려 확인한 결과:
  - 전역 costmap `inflation_radius`를 **1.0 → 1.6**(a1b3d3a 이전 값)으로 바꾸면 **2 passed**였다.
  - 전역 1.3은 실패했다. 지역도 1.0으로 맞춘 경우도 실패했다.
  - control_mux, bt_loop_duration, CAN 노드, STVL, BT, Debug 빌드, `rotate_to_heading_once`는 원인이 아니었다.
- 가설: 전역 1.0이면 Smac 경로가 작은 장애물 옆 약 0.5 m를 지난다. DWB는 사각형 footprint와 PathAlign 32 때문에 이 경로를 따라가지 못한다.
- 사용자 입장: 1.6은 복도에서 전역 경로가 크게 부풀어서 싫다. 1.0의 직진성은 유지하면서 장애물 옆 정지를 없애는 값을 찾고 싶다.

설정 위치는 `src/amr_system/amr_stack/config/navigation.yaml`이다.
- 전역 inflation: `global_costmap.global_costmap.ros__parameters.inflation_layer.inflation_radius` (현재 1.0, 274행 부근)
- 지역 inflation: `local_costmap.local_costmap.ros__parameters.inflation_layer.inflation_radius` (1.6)
- Smac: `planner_server.ros__parameters.GridBased.cost_travel_multiplier` (4.0)
- DWB: `controller_server.ros__parameters.FollowPath.PathAlign.scale` (32.0)

## 규칙

- 저장소의 추적 파일(코드·설정)은 커밋하지 않는다. 시험용 YAML은 `runtime/isaac_check/`(gitignore 대상)에 만든다.
- 1단계에서 `navigation.yaml`을 잠시 고친다면, 끝난 뒤 반드시 `git checkout -- src/amr_system/amr_stack/config/navigation.yaml`로 되돌린다. `git diff`가 비어 있는지 확인한다.
- 커밋·push는 "보고" 절의 파일만 한다. 브랜치는 `no-resume-approval`만 쓰고, main에는 push하지 않는다.
- ROS 도메인: 1단계는 191을, 2단계는 107을 쓴다(`run_sim_trial.py`가 107만 허용한다). 3단계 벤치마크는 스크립트가 자체 도메인을 쓴다. 실차 도메인 7은 쓰지 않는다.

## 0. 준비

```bash
cd <Isaac PC의 amr_ws>
git status                         # 커밋 안 한 변경이 있으면 사용자에게 물어본다
git fetch origin
git checkout no-resume-approval
git pull --ff-only
git log --oneline -1               # 4be83f9 이후인지 확인
dpkg -s ros-jazzy-spatio-temporal-voxel-layer >/dev/null 2>&1 || sudo apt install -y ros-jazzy-spatio-temporal-voxel-layer
./scripts/build_ros.sh --no-rosdep --build-only
source scripts/env.sh
mkdir -p runtime/isaac_check
```

STVL 패키지가 없으면 Nav2가 뜨지 않는다. `sudo`가 막히면 사용자에게 설치를 요청한다.

## 1. 격리 ROS 시험 재현 (Isaac 불필요)

목적: x86 PC에서도 같은 결과가 나오는지 확인한다. 같은 결과면 원인은 Jetson CPU가 아니라 설정이다.

```bash
source scripts/env.sh
export ROS_DOMAIN_ID=191 ROS_LOCALHOST_ONLY=1 AMR_RUN_NAV2_INTEGRATION=1
python3 -m pytest -q src/amr_system/amr_stack/test/test_navigation_pipeline_ros.py
```

다음 두 경우를 각각 **3회씩** 돌린다.
- A: 현재 그대로(전역 1.0)
- B: 전역 274행 부근 `inflation_radius: 1.0`을 `1.6`으로 바꾼 상태. symlink 설치라서 빌드 없이 바로 반영된다. 끝나면 되돌린다.

경우마다 통과/실패 횟수를 적는다. 실패하면 `/tmp/pytest-of-$USER/pytest-*/test_real_smac_dwb_mux_lb_take0/navigation.log`에서 `Failed to make progress` 횟수를 적는다.

## 2. Isaac Sim 장애물 시나리오

`obstacle` 월드는 이렇게 동작한다.
- (0,0,0)에서 출발해 목표 (3,0,0)으로 간다.
- 10 s 뒤 0.4×0.6×1.0 m 상자가 (1.3,0)에 나타난다.
- 주행 중 장애물이 생겨 경로를 다시 짜는 상황이다.

`navigation.yaml`을 복사해 변형 네 개를 만든다.

| 이름 | 변경 |
|---|---|
| `a_infl10` | 변경 없음(현재) |
| `b_infl16` | 전역 inflation 1.6 |
| `c_infl10_pathalign16` | 전역 1.0, `FollowPath.PathAlign.scale` 16.0 |
| `d_infl10_travel8` | 전역 1.0, `GridBased.cost_travel_multiplier` 8.0 |

```bash
cp src/amr_system/amr_stack/config/navigation.yaml runtime/isaac_check/a_infl10.yaml
# b, c, d는 python3 + yaml로 해당 키만 바꿔 runtime/isaac_check/에 저장한다
export ROS_DOMAIN_ID=107
for v in a_infl10 b_infl16 c_infl10_pathalign16 d_infl10_travel8; do
  for s in 1 2 3; do
    /usr/bin/python3 scripts/run_sim_trial.py --world obstacle --goal 3 0 0 --seed $s \
      --navigation-config runtime/isaac_check/$v.yaml --output reports/trials/isaac_check/${v}_s$s
  done
done
```

- 시도마다 기록할 것: 목표 도달 여부, 걸린 시간, 충돌 여부, `Failed to make progress` 횟수(`launch.log`), 장애물과의 최소 간격. `metrics.json`에 있으면 그 값을 쓴다.
- `scripts/compare_trials.py --baseline …a_infl10_s*/metrics.json --candidate …<v>_s*/metrics.json`으로 비교한다.
- 장애물이 정면에 나타나는 시나리오라 시험 실패 상황과 똑같지는 않다. **a가 전부 성공하더라도** 그 사실을 그대로 보고한다.

## 3. 전역 경로 오프라인 비교 (Isaac 불필요)

목적: 사용자가 말한 "1.6이면 전역 경로가 이상해짐"을 숫자로 확인한다. 2단계 후보들이 1.0의 직진성을 얼마나 유지하는지도 함께 본다.

```bash
python3 scripts/benchmark_global_planner.py a_infl10
python3 scripts/benchmark_global_planner.py b_infl16 runtime/isaac_check/ov_b.yaml
python3 scripts/benchmark_global_planner.py d_infl10_travel8 runtime/isaac_check/ov_d.yaml
```

오버라이드 YAML은 `navigation.yaml`과 같은 키 구조로, 바꾸는 키만 적는다. 예시(`ov_b.yaml`):

```yaml
global_costmap:
  global_costmap:
    ros__parameters:
      inflation_layer:
        inflation_radius: 1.6
```

`ov_d.yaml`은 `planner_server.ros__parameters.GridBased.cost_travel_multiplier: 8.0`이다. c는 DWB만 바꾸므로 이 단계는 a와 같다.

결과는 `logs/planner_bench/<이름>.json`에 저장된다. 경로별 꺾임, 방향 역전, 최소 간격, 길이를 표로 옮긴다.

## 보고

1. `docs/handoff/ISAAC_RESULT_2026-10-27.md`를 쓴다. 내용:
   - 각 단계의 표
   - 실행한 정확한 명령
   - 환경: PC 이름, `git log --oneline -1`, Isaac 버전
   - 해석: 1단계로 설정 원인이 맞는지, 2·3단계에서 어느 후보가 장애물 옆 정지를 없애면서 직진성을 지키는지
   - 다음 실차 시험에 권하는 값 하나
2. 실제로 쓴 오버라이드·변형 YAML을 `docs/handoff/isaac_overrides/`에 복사한다.
3. 아래 명령으로 그 파일들만 커밋해 push한다.

   ```bash
   git add docs/handoff/ISAAC_RESULT_2026-10-27.md docs/handoff/isaac_overrides/
   git commit -m "Isaac check: global inflation vs obstacle stall (results only)"
   git push origin no-resume-approval
   ```

4. 사용자에게 결론을 3줄로 알린다.

막히면(Isaac이 뜨지 않음, STVL 설치 불가, 시험이 무한 대기 등) 우회하지 말고 그 상태를 결과 파일에 적은 뒤 사용자에게 묻는다.
