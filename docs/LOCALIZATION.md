# 위치 추정과 주행 시작

실차와 시뮬레이션이 같은 흐름을 쓴다. 초기 위치를 찾아 검증하고, 검증된 뒤에만 Nav2를 시작해 목표를 받는다.
관련 측정과 원인 조사는 [실험 기록](EXPERIMENTS.md#2026-10-04--위치-추정-시작추적-조사)에 있다.

## 1. 구성

```text
S2 LiDAR /scan ─┬─────────────────────────▶ AMCL ── map → odom (유일한 발행자)
                │                            ▲
                │            /initialpose ───┘
                ▼
        navigation_startup (감독기) ── /localization/status
          ├─ 자동: amr_tim_localization (/localization/global_localize) ─ 후보만 반환
          ├─ 수동: 웹 UI·RViz 초기 위치
          └─ 검증 통과 → AMCL 빔 수 전환 → Nav2 lifecycle STARTUP → READY
```

- `localization`과 `navigation`을 함께 실행해도 된다. 감독기가 지도·스캔·odometry·센서 TF·활성 AMCL을 기다린다.
- 감독기는 자동 회전이나 이동 명령을 내지 않는다. TIM 노드는 후보만 반환하며 TF·`/initialpose`·주행 명령을 발행하지 않는다.
- 준비 전에는 웹 UI, 공개 navigation Action, `/goal_pose` 모두 목표를 거절한다.

## 2. 자동 초기 위치 (TIM)

기본 전역 초기화기는 TIM 2026 공개 구현을 ROS 2로 옮긴 `amr_tim_localization`이다.
정지 상태의 지도·스캔 복사본으로 RRT 후보 생성 → SMAD 순위 → TAM 방향 평가 → ICP 보정을 수행한다.

- 전역 탐색은 최대 1,080빔. 상위 후보(기본 5개)에만 최대 1,080빔과 원본 해상도 지도 점군으로 정밀 ICP를 적용한다.
  정밀 보정 전후를 같은 스캔으로 평가하고, 수렴·대응점·이동 한도·점수 조건을 통과하지 못하면 적용하지 않는다.
- 감독기가 LiDAR 위치를 센서 TF로 로봇 기준 위치로 바꾸고 지도·스캔 정렬을 확인한 뒤 AMCL에 `/initialpose`를 전달한다.
- 탐색 제한은 기본 30초(`fine_budget_sec: 3.0`은 그 안의 정밀 ICP 예산). 감독기도 별도 벽시계 제한으로 늦은 결과를 버린다.
- 탐색 중 수동 초기 위치, 지도 변경, 이동 명령, 큰 움직임, 목표 실행이 있으면 결과를 적용하지 않는다.
  정지 중 작은 odometry 변화는 최대 2 cm·0.01 rad(`tim_max_compensation_translation`·`_yaw`)까지 보상하고,
  추론 중 최대 이탈량이 한도를 넘었다 돌아온 경우도 폐기한다. 이동하면서 전역 탐색하는 기능은 아니다.
- 거절 사유는 `/localization/status`의 `tim_rejection_reason`, 최대 이탈량 `tim_max_translation`·`_yaw`,
  실제 보정량 `tim_compensation_translation`·`_yaw`, 센서별 수신 간격과 관측 지연도 함께 기록한다.
- 시간 초과·후보 없음은 수동 지정이나 자동 재시도로 복구한다. 실패해도 무작위 AMCL 초기화로 자동 전환하지 않는다.
  비교용 `amcl` backend를 고른 경우에만 `/reinitialize_global_localization`을 호출한다.
- navigation 시작 전에 AMCL 위치가 이미 있으면 그 추정을 보존하고 새 스캔으로 먼저 검증한다(자동 탐색과 같은 모호성 검사 적용).
- 반복되는 복도에서는 한 자리의 스캔만으로 위치가 구분되지 않을 수 있다. 격자 기반 경쟁 후보 검사는 모호한 위치를 거르는 추가 근거이며,
  임의 위치에서 항상 자동 초기화된다는 보장은 아니다.

알고리즘 출처: Zhang 외, *Tackling the Kidnapped Robot Problem via Sparse Feasible Hypothesis Sampling and Reliable Batched Multistage Inference*,
IEEE TIM 2026 ([논문](https://arxiv.org/abs/2511.01219), [공개 소스](https://github.com/tsdworks/relocalization_toolbox)).
이식 패키지에 원본 GPL-3.0 라이선스와 변경 내역을 보존한다(`src/amr_system/amr_tim_localization/UPSTREAM.md`).
원논문의 성공률·속도를 이 프로젝트의 실측값으로 쓰지 않는다.

## 3. 초기 위치 검증과 빔 수 전환

AMCL은 초기 탐색과 수동 초기 위치 검증에 `max_beams: 1080`을 쓰고, 검증이 끝나면 주행용 `max_beams: 60`으로 바꾼다.

- 작은 공분산만으로 판단하지 않는다. 지도·스캔 정렬(일치율 70 %, 지도 내 관측 85 %), 위치·방향 공분산, 입자 집중도,
  다른 복도 위치와의 유사도를 본다.
- 전환 시 입자 분포를 유지하고, 파라미터 적용 응답과 다시 읽은 실제 값이 일치하는지 확인한다. 전환 후 새 관측까지 검증한 뒤
  lifecycle manager에 STARTUP을 요청하므로, 초기 위치를 정하기 전에 costmap의 60초 TF 제한이 시작되지 않는다.
- 60초 자동 탐색 제한 후에도 수동 초기 위치를 지정하면 같은 navigation 프로세스에서 검증과 시작을 이어간다.

| 경로 | 요구 조건 |
|---|---|
| 수동 초기 위치 | 전역 탐색·복도 경쟁 후보 검사 생략. 1,080빔과 60빔 각각에서 최소 1초, 서로 다른 AMCL 추정·스캔 4쌍 |
| 자동 초기화 | 단계별 2.5초·6스캔 + 전역 모호성 검사 |

- 수동·엘리베이터 하차 초기 위치는 AMCL 검증 전에 지정 위치 주변(±1 m, ±25°)에서 스캔·지도 정합으로 보정한다
  (`manual_refinement`). 보정 위치가 일치율 70 %·지도 내 관측 85 %를 넘을 때만 좁은 분산(0.1 m, 3°)으로 AMCL을 다시
  초기화하고, 미달·시간 초과·보정 중 이동이면 지정 위치를 그대로 검증한다. 창 밖의 같은 모양 복도로는 옮기지 않는다.
  4층 출발 지점 실측 스캔에서 0.3~0.8 m·15~25° 어긋난 입력이 모두 같은 위치(일치율 약 81 %)로 약 0.4초에 수렴했다.
  결과는 `/localization/status`의 `refinement_*`와 로그에 남는다.
- 검증 중 정지 관측 갱신 요청은 0.25초 이상 간격, READY 후에는 1초 간격이다.
- 조정 파라미터: `manual_verification_duration`·`manual_verification_samples`, `initialization_update_interval`·`tracking_update_interval`.
- `/localization/status`에 `xy_std`, `yaw_std`, 검증 관측 수, 경과 시간을 표시하고 대기 사유가 바뀌면 로그로 알린다.
  초기 위치를 다시 지정하면 이전 관측을 버리고 새로 검증한다.

## 4. READY 이후 추적

- 검증된 초기 위치 이력을 유지한다. 스캔 일치율·지도 내 관측 비율·공분산이 초기 기준을 넘으면 `tracking_warning`으로만 표시하고
  목표를 취소하거나 초기화를 다시 하지 않는다. 이 수치는 초기 위치 승인 기준이지 주행 중 위치 상실 판정기가 아니다.
- 다음은 준비 상태를 해제한다: 센서 지연·잘못된 데이터, TF 누락, 유효하지 않은 공분산(NaN·음수·비대칭),
  이동 가능 영역 밖의 추정, 유효 빔 부족, Nav2 비활성.
- 해제 중 `navigation_manager`의 처리: 검증된 이력(`prior_verified`)이 있고 같은 세대에서 `WAITING_DATA`·`VERIFYING`으로만 바뀐 경우는
  목표를 유지한 채 속도 0으로 멈추고 최대 `localization_hold_grace`(5초) 동안 재검증을 기다린다(실차에서 CPU 부하로 odom TF가
  1.5초 넘게 늦어 READY가 1초쯤 풀렸다가 돌아오는 사례, 2026-10-23). 그 안에 READY로 돌아오면 같은 목표를 이어 간다.
  heartbeat 2초 단절, 새 세대(초기 위치 재지정·자동 탐색·지도 변경), `FAILED`·`WAITING_INITIAL_POSE`, 시간 이상은 즉시 목표를 종료하고
  복구돼도 자동 재개하지 않는다.
- READY 이후 공분산 경고 기준은 `tracking_max_xy_std`(0.6 m)·`tracking_max_yaw_std`(0.35 rad)다. 초기 승인 기준(0.35 m·0.25 rad)은
  출발 전 심사용이고, 긴 복도에서는 앞뒤 방향 불확실성이 0.36~0.39 m까지 정상적으로 커졌다가(실차 2026-10-23, 주행당 경고 44~102회)
  문·복도 끝이 보이면 줄어든다. 그 범위는 더 이상 경고하지 않는다. 일치율도 같은 이유로 READY 이후에는 `tracking_min_match_ratio`(60 %)를
  쓴다. 지도에 없는 가구·열린 문 앞(401호)에서는 맞는 위치에서도 68~70 %가 나온다(2026-10-24, 주행당 경고 41회).
- AMCL 추정과 `map → odom` TF의 차이는 READY 이후 1.0 m·0.5 rad(`tracking_tf_jump_xy`·`_yaw`)까지 경고로만 본다. AMCL은 TF를
  `transform_tolerance`만큼 미래 시각으로 내보내므로, 보정 직후 추정 시각의 TF는 이전·현재 보정이 섞여 한 번 어긋난다
  (실차 0.5 m/s 복도 주행에서 0.39 m, 2026-10-23). 그 이상은 다른 발행자·오래된 TF로 보고 지금처럼 해제한다. 초기 검증 중에는 기존 기준(0.35 m·0.25 rad) 그대로다.
- 해제 후 복구는 60빔 그대로 정지 후 새 AMCL·스캔 4쌍을 최소 1초 검증한다(`tracking_verification_duration`·`_samples`).
  TIM, 입자 전역 초기화, 복도 경쟁 후보 검사는 자동 재실행하지 않는다.
- 초기 위치 이력은 수동 위치 재지정, 명시적인 자동 탐색 재시도, 지도 변경 때 해제된다.
- 웹 UI는 READY 상태의 `위치 추적 경고`와 준비 상태 상실 후 재검증을 구분해 표시한다.
- 층간 이동 중에는 `/localization/suspend`로 `SUSPENDED` 상태가 되며, 재개 시 `/initialpose`를 기다린다([층간 이동](ELEVATOR.md)).

### 회전 스캔 보정

주행 검증에서는 `LaserScan.time_increment`와 `odom → laser` TF로 각 측정점의 취득 시각에 맞춰 이동·회전을 보정한다.
스캔 끝까지의 TF가 아직 없으면 신선도 제한 안의 이전 완전한 스캔을 쓰며, 최신 TF로 대체하거나 이동 명령으로 추측하지 않는다.
시각 정보가 0인 스캔은 기존 방식으로 평가하고 잘못된 시간 정보는 거절한다.
상태 토픽에 `scan_motion_compensated`, `scan_duration`, `scan_motion_translation`, `scan_motion_yaw`, `uncompensated_match_ratio`를 포함한다.
`uncompensated_match_ratio`는 보정 점수가 기준에 미달할 때만 따로 계산하고, 통과했을 때는 보정 점수와 같은 값이다(CPU 절감).

## 5. 설정

| 파일 | 내용 |
|---|---|
| `src/amr_system/amr_stack/config/localization.yaml` | AMCL. 1,080빔으로 시작, `laser_max_range: 30.0`, differential 모델, `update_min_d 0.1`, `update_min_a 0.1` |
| `src/amr_system/amr_stack/config/localization_startup.yaml` | 감독기. `initialization_max_beams: 1080`, `tracking_max_beams: 60`, `quality_max_beams: 360`, `quality_max_range`·`tracking_quality_max_range: 30.0`, `automatic_localization`, `global_localization_backend: tim` |
| `src/amr_system/amr_tim_localization/config/tim_localization.yaml` | TIM. 최대 1,080빔·30 m, 위치 후보 간격 0.4 m, 방향 32개, `fine_enabled`·`fine_max_beams`·`fine_max_candidates`·`fine_budget_sec` |

- AMCL 관측 거리를 바꾸면 주행 검증 거리(`tracking_quality_max_range`)도 함께 검토한다. 입력 스캔의 실제 `range_max`가 더 작으면 그 안에서만 쓴다.
- 감독기 없이 localization만 실행하면 1,080빔을 유지한다.
- 수동 초기 위치만 쓰려면 `automatic_localization: false`. 목표 준비 검사는 유지된다.
- `startup_config:=...`, `tim_config:=...`로 설정 파일을 바꿀 수 있다. TIM 설정은 YAML 수정 후 navigation을 재시작해 적용한다.
  정밀 보정 없이 비교하려면 `fine_enabled: false`. 세부 진단은 `tim_fine` 로그.
- TIM은 PCL·OpenCV·Eigen을 쓴다. 새 환경은 `scripts/build_ros.sh`의 rosdep 절차로 의존성을 설치한다.

## 6. 운용

- 웹 UI에 `주행 준비 완료`가 표시된 뒤 목표를 지정한다.
- 자동 탐색으로 위치를 확신하지 못하면 `초기 위치`로 위치·방향을 지정하거나, 목표를 취소하고 정지한 뒤 `자동 위치 찾기`로 재시도한다.
- READY·정지 상태의 AMCL 위치를 지도별로 `~/.ros/amr_last_pose.json`에 저장한다(`last_pose_file`, 2초·5 cm 간격, 추적 경고 중 제외).
  저장된 위치가 있으면 웹 UI에 `이전 위치로 시작`이 보이며, 누르면 그 위치를 지도에 표시만 한다. 실제 로봇 위치와 맞는지 보고
  `초기 위치 적용`을 눌러야 적용되고, 이후 일반 수동 위치와 같이 보정·검증한다. 전원이 꺼진 사이 로봇을 옮겼을 수 있어
  자동으로 적용하지 않는다. 코드 기본값은 빈 문자열(저장 안 함)이라 테스트는 실차 파일을 건드리지 않는다.
- Nav2 활성화 자체가 실패하면 로그 확인 후 navigation을 재시작한다.
- 비교용 `amcl` backend에서 `/reinitialize_global_localization`이 응답하지 않아 완료 여부를 모르면, 지연된 초기화가 수동 위치를 덮어쓸 수 있으므로
  localization과 navigation을 모두 재시작한다.

## 7. 남은 확인

- 4층 출발 지점(약 x 40.8, y 0.1)은 같은 모양의 복도 구간(약 7.7 m 간격)과 일치율 차이가 4 %p 정도라, 한 장의 스캔으로는
  자동 탐색이 원리적으로 확정할 수 없다(경쟁 후보 기준 10 %p). 이 지점에서는 수동 지정을 쓴다.

- 1,080빔 설정의 실제 Jetson 성공률·오수렴·지연, 주행용 60빔과 180빔 비교는 아직 측정하지 않았다.
- 시뮬레이션에서 긴 복도 진행 방향의 위치 추정 오차가 남아 있다(평균 0.149 m, 최대 0.512 m, 1회 재현).
- 긴 복도 제자리 회전 중 AMCL/TF 불일치로 READY가 약 3초 해제된 사례가 있다(재현 안 됨). 실차에서 관찰한다.
