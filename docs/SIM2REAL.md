# AMR 공통 제어·시뮬레이션 실험

실물 보정은 나중에 진행할 수 있다. 현재 단계는 실제 제어 소스를 PC에서 실행하고 PhysX 센서 폐루프, 반복 시험, 기록·비교와 소스 릴리스 절차를 확보하는 것이다. 보정 전 성능 수치로 실물의 제동거리나 절대 위치 정확도를 확정하지 않는다.

## 실행

```bash
cd amr_ws  # 전달받은 워크스페이스 폴더로 이동
# 새 PC에서 한 번 설치. 시스템 Python은 /usr/bin/python3이다.
sudo bash scripts/install_ros_jazzy.sh
./scripts/build_ros.sh
# 상세 4층 + 조이스틱 + EKF + AMCL + navigation, 도메인 107.
# GUI에서 일시정지/리셋하면 시험이 종료된다.
./scripts/experiment.sh sim
# 별도 터미널: 기존 웹 UI
source scripts/env.sh
ROS_DOMAIN_ID=107 ros2 launch amr_web_ui web_ui.launch.py use_sim_time:=true
# 화면 없이 자동 단일 시험. 실물 도메인에서는 실행을 거부한다.
source scripts/env.sh
export ROS_DOMAIN_ID=107
/usr/bin/python3 scripts/run_sim_trial.py --output reports/trials/my_straight
```

`amr` 또는 `source scripts/env.sh`로 환경을 불러오면 `ros2kill`도 등록된다.
`ros2kill --dry-run`은 종료할 프로세스만 표시하고, `ros2kill`은 현재 PC·현재 계정의
모든 ROS 2 노드·launch·웹 서버·Isaac ROS 브리지와 하위 프로세스를 SIGKILL로 종료한다.
ROS 도메인에 관계없이 적용되며 다른 PC의 노드를 종료하지 않는다.

실물 모드는 `./scripts/experiment.sh real`이다. 도메인 7에서 기존 CAN·LiDAR·조이스틱을 시작한다. 이 명령은 실물 주행 준비가 끝난 Jetson에서 운용한다. PC의 자동 시험기는 실물 모드를 지원하지 않는다. `can_dry_run:=true`는 송신만 비활성화하며 센서 수신은 유지한다.

Isaac 설치본과 생성 USD는 Git에서 제외한다. 새 PC에서는 기존 `build_scene.sh` 절차로 장면을 만든다. `experiment.launch.py`는 기본 sim이며 각 하위 launch에 `use_sim_time`을 전달한다. 독립적인 기존 bringup의 실물 기본 동작은 보존한다.

## 제어·센서 구조

### 초기 위치 탐색과 navigation 시작

`localization`과 `navigation`을 함께 실행해도 된다. `navigation_startup`이 지도,
스캔, odometry, 센서 TF와 활성 AMCL을 기다린다. 기본 전역 초기화기는 TIM 2026의
공개 구현을 ROS 2로 옮긴 `amr_tim_localization`이다. 정지 상태에서 지도와 스캔의
복사본을 `/localization/global_localize`에 보내 RRT 후보 생성, SMAD 순위 평가,
TAM 방향 평가, ICP 보정을 수행한다. 전역 탐색은 최대 1,080빔을 사용하고,
상위 후보에만 최대 1,080빔과 원본 해상도의 지도 점군을 사용해 정밀 보정한다.
정밀 보정 전후를 같은 스캔으로 평가하며, 수렴·대응점·이동 한도·점수 조건을
통과하지 못한 보정은 적용하지 않는다. 여러 후보를 반환하므로 단일 후보를 찾았다는
이유만으로 조기 종료하지 않는다. TIM 노드는 후보만 반환하며 TF나 `/initialpose`,
주행 명령을 발행하지 않는다. 감독기가 LiDAR 위치를 센서 TF로 로봇 기준 위치로
변환하고 지도·스캔 정렬을 확인한 뒤 AMCL에 `/initialpose`를 전달한다.
AMCL이 계속 `map → odom`의 유일한 발행자다. 자동 회전이나 이동 명령은 내보내지 않는다.
navigation 시작 전에 AMCL 위치가 이미 설정되어 있으면 그 추정을 보존하고
새 스캔으로 먼저 검증한다. 이전 초기화의 출처를 알 수 없으므로 이 경우도
자동 탐색과 동일한 모호성 검사를 적용하며, 필요하면 수동 위치를 다시 요청한다.

AMCL은 초기 탐색과 수동 초기 위치 검증에 `max_beams: 1080`을 사용한다.
작은 공분산만으로 준비 완료를 판단하지 않고 지도·스캔 정렬,
위치·방향 공분산, 입자 집중도와 다른 복도 위치의 유사도를 확인한다.
연속 검증을 통과하면 주행용 `max_beams: 60`으로 전환한다. 이때 AMCL의
입자 분포를 유지하며, 파라미터 적용 응답과 읽어온 실제 값이 일치하는지 확인한다.
전환 후 새 관측까지 검증한 뒤 lifecycle manager에 STARTUP을 요청한다. 따라서
초기 위치를 설정하기 전에 costmap의 60초 TF 제한이 시작되지 않는다.
60초 자동 탐색 제한 후에도 수동 초기 위치를 지정하면 같은 navigation
프로세스에서 검증과 시작을 이어간다.

수동 초기 위치는 전역 탐색·복도 경쟁 후보 검사를 생략하고 사용자가 지정한 위치의
스캔 정렬과 공분산을 확인한다. 초기 1,080빔과 전환 후 60빔 각각에서 최소 1초,
서로 다른 AMCL 추정과 LiDAR 스캔 4쌍을 요구한다. 자동 초기화는 기존의 단계별
2.5초·6스캔 및 전역 모호성 검사를 유지한다. 검증 중 정지 관측 갱신 요청은
0.25초 이상 간격(0.1초 검사 주기에서 보통 약 0.3초), READY 후에는 1초 간격이다.
이는 불확실성 기준을 낮추는 설정이 아니다. Web UI의 초기 위치 분산은 그대로
유지하므로 입력 위치가 부정확하거나 스캔이 지도에 맞지 않으면 대기가 길어질 수 있다.
`manual_verification_duration`·`manual_verification_samples`와
`initialization_update_interval`·`tracking_update_interval`로 조정한다.
`/localization/status`에 `xy_std`, `yaw_std`, 검증 관측 수와 경과 시간을 표시하며,
같은 VERIFYING 상태에서도 대기 사유가 바뀌면 로그로 알린다. 초기 위치를 다시
지정하면 이전 관측을 버리고 새 위치에서 검증을 다시 시작한다.

2026-10-04 수동 초기화 비교 시험에서는 READY까지 12.983초에서 4.100초로 줄었다.
실제 AMCL과 3,240빔 합성 센서, Web UI와 동일한 초기 공분산을 사용해 각 1회 측정한
결과이며, Nav2 lifecycle 응답은 모의 처리하므로 실제 서버 시작 시간은 포함하지 않는다.
재현 도구는 `scripts/benchmark_manual_localization_latency.py`, 원자료와 제한 사항은
`reports/localization_startup_validation/manual_latency_comparison.json`에 있다.

READY 이후에는 검증된 초기 위치 이력을 유지한다. 스캔 일치율·지도 내 관측 비율과
유효한 AMCL 공분산의 초기 검증 기준 초과는 `tracking_warning`으로 표시하며,
기존 목표를 취소하거나 초기화를 다시 실행하지 않는다. 이 수치는 초기 위치를
승인하는 기준이지 주행 중 위치 상실을 입증하는 판정기가 아니다. 경고의 지속
시간과 서로 다른 스캔 수는 진단 정보로만 제공한다. 반복 경고 시 지도·센서·
AMCL 설정을 확인하고 필요하면 사용자가 목표를 취소한 뒤 위치를 다시 지정한다.

센서 지연·잘못된 데이터, TF 누락, 유효하지 않은 공분산(NaN·음수·비대칭 등),
이동 가능 영역 밖의 추정, 유효 빔 부족과 Nav2 비활성은 준비 상태를 해제한다.
이때 기존 목표를 종료하며, 데이터 복구만으로 이전 목표를 자동 재개하지 않는다.
주행용 60빔을 유지한 채 정지 후의 새 AMCL·스캔 4쌍을 최소 1초 검증한다.
`tracking_verification_duration`·`tracking_verification_samples`가 이 복구 조건이다.
TIM, 입자 전역 초기화와 복도 경쟁 후보 검사는 자동 재실행하지 않는다.
초기 위치 이력은 수동 위치 재지정, 명시적인 자동 탐색 재시도, 지도 변경 때 해제된다.
Web UI는 READY 상태의 `위치 추적 경고`와 준비 상태 상실 후 재검증을 구분한다.

저장소 기준 코드(`HEAD`, `e0b5cd1`)의 `navigation_manager`에는
`/localization/status`에 의한 목표 취소 조건이 없었다. 자동 초기화 추가 시
`navigation_startup`의 준비 상태를 주행 허용 조건으로 연결하면서, 초기화용
스캔·공분산 기준을 주행 중에도 목표 취소에 사용한 것이 회귀 원인이었다.
제공 로그의 READY → VERIFYING → 목표 취소 동안 TIM의 새 탐색은 없었다.
Nav2를 초기 위치 검증 후 활성화하는 시작 절차와 상태 heartbeat 감시는 유지하고,
첫 READY 이후 품질 경고가 초기화 완료 상태를 되돌리던 연결을 제거했다.
실제 Jetson에 배포된 바이너리의 해시는 이번 비교에서 확인하지 않았다.

주행 검증에서는 `LaserScan.time_increment`와 `odom → laser` TF로 각 측정점의
취득 시각에 해당하는 이동·회전을 보정한다. 스캔 전체에는 시작 시각의 지도 보정
하나를 사용하고, 빔별 끝점과 가시선 시작점에 모두 취득 시각의 센서 위치를 적용한다.
스캔 끝까지의 TF가 아직 도착하지 않았다면 기존 신선도 제한 안의 완전한 이전
스캔을 사용한다. 최신 TF로 대체하거나 이동 명령으로 센서 위치를 추측하지 않는다.
시각 정보가 0인 순간 스캔은 기존 방식으로 평가하며, 잘못된 시간 정보는 거절한다.
정지 상태에서 수행하는 전역 초기화와 일치율·공분산·정지 조건은 유지한다.

회전 보정 필요성은 `scripts/benchmark_rolling_scan_quality.py`로 재현할 수 있다.
4층 정적 형상과 8.8Hz·3,240점, 0.4m/s·0.6rad/s 궤적을 사용한 두 예에서,
정확한 위치를 사용해도 한 시각으로 투영한 점수는 0.614·0.544였고 빔별 보정 후
0.994·0.997이었다. 총 32건의 보정 결과는 독립적인 빔별 평가와 일치했다.
이는 잡음·PhysX 자세 기울기·움직이는 문·AMCL 오차를 제외한 기하 재현이며,
2026-10-04 04:18 주행에는 원본 bag가 없어 당시 스캔 점수가 낮아진 원인을 직접 입증한 결과는 아니다.
원자료는 `reports/localization_startup_validation/rolling_scan_repro.json`이다.
정렬 실패 로그에는 보정 전후 일치율, 지도 내 관측 비율, 벽 관통 비율과 스캔 중
회전량을 남긴다. 상태 토픽에는 `scan_motion_compensated`, `scan_duration`,
`scan_motion_translation`, `scan_motion_yaw`, `uncompensated_match_ratio`도 포함한다.

실제 Isaac 재현에서는 회전 보정만 적용해도 같은 경로에서 정지했다. 첫 실패
스캔의 실제 자세 오차는 약 4.6cm·1.69도였으며, 30m 검증 점수는 0.681,
당시 AMCL과 동일한 5m 범위 점수는 0.836이었다. 전후 31개 스캔에서 30m 평가는
11개가 실패했고 5m 평가는 모두 통과했다. 그러나 검증 거리만 5m로 줄인 별도
재주행은 이후 공분산 기준을 초과해 다시 정지했다. 이에 AMCL의 관측 자체를
30m로 넓히고 초기·주행 검증도 30m로 일치시켰다. 실제 AMCL에 동일 bag를
재생한 비교에서는 평균 위치 오차가 7.09cm→4.30cm, 방향 오차가
0.756도→0.398도로 줄었다. 5m 재생 결과 77개는 원본의 추정값과 정확히 같았다.
비교 자료는 `amcl_range_replay/comparison.json`이며, 한 경로의 센서 재생으로
확인한 결과다. 이 거리 조정은 별도의 추적 정확도 개선이며 정지 문제의 해결책은 아니다.
30m로 재주행해도 위치 표준편차 0.352m가 초기 기준 0.350m를 넘자 46.50초 만에
목표가 취소됐다(`long_range_isaac_trial`). 따라서 초기화 조건과 주행 경고를 분리했다.
초기 검증의 빔 수·일치율 70%·지도 내 관측 85%·공분산 기준은 유지한다.
AMCL의 관측 거리 설정을 바꾸면 주행 검증 거리도 함께 검토해야 한다.
실제 재현 원자료는 `reports/localization_startup_validation/rolling_isaac_trial`,
오프라인 대조는 `rolling_isaac_bag_diagnosis.json`에 있다. 이는 제공된 원래 실행의
bag 분석이 아니라 같은 목표로 별도 재현한 데이터다.

수정 후 같은 4층 목표 `(41.41, 0.36, 0)`를 Isaac Sim에서 재주행해 약 86초에
도착했다. 제어기·플래너·AMCL·EKF·control_mux의 런타임 파라미터는 직전 실패
시험과 같았다. 주행 중 상태 1,722개가 모두 READY였으며, 공분산 경고가 최대
37.7초·331스캔 지속되어도 목표 취소, 초기 위치 재발행, AMCL 빔 변경은 없었다.
충돌·복구 동작은 0회, 실제 도착 위치 오차는 0.112m였다. 전체 주행 중 위치
추정 오차는 평균 0.149m, 최대 0.512m로 복도 진행 방향의 오차가 남아 있다.
이는 수동으로 알려진 시작 위치를 지정한 1회 재현이며 TIM 초기 위치 정확도나
실물 Jetson 성능을 측정한 시험은 아니다. 센서 모델도 미보정 상태다.
원자료는 `reports/localization_startup_validation/startup_scope_isaac_trial`,
원인·런타임 비교·제한 사항은 `startup_scope_validation.json`에 기록했다.
초기화·기하 검사 129개, 목표 관리 14개, Web UI 59개와 JS 표시 검증을 통과했다.

Web UI에 `주행 준비 완료`가 표시된 뒤 목표를 지정한다. 자동 탐색으로 위치를
확신하지 못하면 `초기 위치`로 위치·방향을 지정하거나, 목표를 취소하고 정지한
뒤 `자동 위치 찾기`로 재시도한다. 반복되는 복도에서는 한 자리의 스캔만으로
위치가 구분되지 않을 수 있다. 격자 기반 경쟁 후보 검사는 모호한 위치를
거르는 추가 근거이며, 임의 위치에서 항상 자동 초기화된다는 보장은 아니다.

준비 전에는 Web UI, 공개 navigation Action 및 `/goal_pose` 모두 목표를
거절한다. 준비 상태 상실 또는 재초기화 시 기존 목표를 종료하고 자동 속도를
0으로 만든다. 다시 준비되어도 이전 목표를 자동 재개하지 않는다.
실제 Nav2 활성화 자체가 실패한 경우는 로그 확인 후 navigation을 재시작한다.
TIM 탐색은 기본 30초 제한을 사용한다. 탐색 중 수동 초기 위치 지정, 지도 변경,
이동 명령, 큰 움직임 또는 목표 실행이 발생하면 이전 결과를 초기 위치로 적용하지 않는다.
정지 상태에서 생기는 작은 odometry 변화는 최대 2cm·0.01rad 범위에서 보상한다.
한도는 시작 설정의 `tim_max_compensation_translation`, `tim_max_compensation_yaw`다.
추론 중 최대 이탈량을 검사하므로 한도를 넘었다가 원위치로 돌아온 경우도 폐기한다.
입력 스캔 시각의 TIM 후보를 최신 유효 스캔 시각으로 이동·회전 보상한 뒤 정렬을
다시 검증하고, 해당 관측 시각을 붙여 AMCL에 전달한다. 센서 수신 공백·유효하지 않은
데이터·이동 명령·목표 발생을 각각 구분해 표시하며, 중간에 발생한 문제를 이후의
정상 수신만으로 지우지 않는다. 이 보상은 이동하면서 전역 탐색하는 기능이 아니다.
`/localization/status`의 `tim_rejection_reason`에 거절 사유를 기록하고,
`tim_max_translation`·`tim_max_yaw`에는 추론 중 최대 이탈량,
`tim_compensation_translation`·`tim_compensation_yaw`에는 실제 적용한 보정량을 기록한다.
센서별 수신 간격과 관측 시각 지연도 함께 남기므로 이동과 데이터 지연을 구분할 수 있다.
시간 초과나 후보 없음은 수동 지정 또는 자동 재시도로 복구한다. 비교용으로 예전
`amcl` backend를 선택한 경우에만 `/reinitialize_global_localization`을 호출한다.
그 서비스가 응답하지 않아 완료 여부를 알 수 없는 경우에는 지연된 초기화가 수동
위치를 덮어쓸 수 있으므로 localization과 navigation을 모두 재시작하도록 표시한다.

기준은 `src/amr_system/amr_stack/config/localization_startup.yaml`에 있다.
`initialization_max_beams: 1080`, `tracking_max_beams: 60`이 각 단계의 AMCL
빔 수를 정한다. 재초기화하면 다시 1,080빔으로 검증하며, 설정 실패 시 목표를 받지 않는다.
`localization.yaml`도 1,080빔으로 시작하므로 navigation보다 먼저 초기 위치를
지정해도 초기 AMCL 관측에 같은 설정이 적용된다. navigation 감독기 없이 localization만
실행하면 1,080빔을 유지한다. 변경 대상은 `max_beams`뿐이며 AMCL의
`laser_max_range: 30.0`을 사용하며 입력 스캔의 실제 `range_max`가 더 작으면
그 범위 안에서만 사용한다. 별도 지도·스캔 검증도 `quality_max_beams: 360`,
`quality_max_range: 30.0`, `tracking_quality_max_range: 30.0`으로 일치시킨다.
TIM 전역 탐색의 빔·거리 설정은 별개로
`src/amr_system/amr_tim_localization/config/tim_localization.yaml`에 있으며,
초기값은 최대 1,080빔·30m, 위치 후보 간격 0.4m, 방향 탐색 32방향이다.
최대 1,080빔 정밀 보정은 상위 후보에만 적용한다. 입력 스캔의 빔 수가 이보다 적으면
존재하는 관측만 사용하며, 비어 있는 각도에 새 관측을 만들어 넣지 않는다.
정밀 보정은 `fine_enabled`, `fine_max_beams`, `fine_max_candidates`로 조정한다.
기본 최대 후보 수는 5개이며, `fine_budget_sec: 3.0`은 전체 30초 제한 안에서
정밀 ICP 준비·시도에 사용하는 별도 예산이다. 촘촘한 스캔의 거리장 준비와 후보
재평가는 전체 요청 예산에 포함되며, 별도 ICP 예산을 시작하기 전에 수행한다.
PCL 등 라이브러리 연산은 중간에 즉시 중단되지 않을 수 있으므로, 감독기도 별도의
벽시계 제한으로 늦은 결과를 폐기한다. 이전 동작과 비교할 때는 `fine_enabled: false`를
사용한다. 정밀 보정의 세부 수렴·대응점·잔차 진단은 `tim_fine` 로그에 기록한다.
TIM 알고리즘 설정은 YAML을 수정한 뒤 navigation을 재시작해 적용한다.
입력 스캔의 실제 유효 범위 안에서만 사용하며,
TIM 입력 범위는 AMCL 파라미터와 독립적이다.
시뮬레이션과 실물 모두 같은 시작 흐름을 사용하며, `startup_config:=...`로
검증 설정 파일을, `tim_config:=...`로 TIM 설정 파일을 선택할 수 있다.
`global_localization_backend: tim`이 기본이며 실패 시 무작위 AMCL 초기화로
자동 전환하지 않는다. 수동 초기 위치만 사용할 때는 시작 설정 파일의
`automatic_localization`을 `false`로 설정한다. 목표 준비 검사는 유지된다.

변경 후 빌드와 실행 환경 갱신:

```bash
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-up-to amr_stack amr_web_ui --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3
source install/local_setup.bash
# Isaac Sim과 센서 스택까지 함께 시작
./scripts/experiment.sh sim
# 이미 Isaac Sim·bringup·localization이 실행 중이면 navigation만 시작
# ROS_DOMAIN_ID=107 ros2 launch amr_stack navigation.launch.py use_sim_time:=true
```

새 환경은 먼저 `scripts/build_ros.sh`의 rosdep 의존성 설치 절차를 사용한다.
TIM은 PCL·OpenCV·Eigen을 사용하며 필요한 의존성을 package.xml에 선언한다.
`/localization/status`, 감독기 및 TIM 노드 파라미터는 주행 기록에 포함된다.
1,080빔 설정의 실제 Jetson 성공률·오수렴·지연은 별도 주행 기록으로
확인해야 한다. 주행용 60빔과 180빔 비교는 아직 수행하지 않았다.

알고리즘 출처는 Zhang 외, *Tackling the Kidnapped Robot Problem via Sparse Feasible
Hypothesis Sampling and Reliable Batched Multistage Inference*, IEEE TIM 2026
([논문](https://arxiv.org/abs/2511.01219),
[공개 소스](https://github.com/tsdworks/relocalization_toolbox))다. 이식 패키지에는 원본
GPL-3.0 라이선스와 변경 내역을 보존한다. 원논문의 성공률·속도를 이 프로젝트의
실측값으로 사용하지 않는다. 지도·스캔으로 구별되지 않는 반복 복도는 여전히
초기 위치의 추가 정보가 필요하다.

2026-10-04 초기 이식 검증에서는 C++ 코어 10개, 시작 감독기 41개,
실제 TIM→AMCL 합성 센서 연동 4개 시험이 통과했다. 마지막 ICP 수정 뒤에는
두 실제 AMCL 연동 사례를 다시 통과했다. 뒤를 향한 실제 URDF 라이다 방향과
센서 위치 오프셋을 확인했고, 동일한 두 복도에서는 준비 완료를 차단했다.
TIM 노드를 포함한 기존 Nav2 실행·경로 계획·재계획 회귀 2개도 통과했다.
4층 지도에서 독립 raycaster로 만든 3개 스캔은 초기 위치를 입력하지 않고
모두 정답 후보를 찾았다(위치 오차 1.0~2.3cm, TIM 계산 4.8~5.0초).
긴 가로 복도 시험점은 정답 후보가 1위여도 별도 모호성 검사에서 주행을 보류한다.
결과와 실행 파라미터는 `reports/localization_startup_validation/tim_floor4_final.json`에 있다.
이는 잡음 없는 지도 기반 시험이며 Isaac 물리 주행이나 Jetson 실측 결과가 아니다.

정밀 보정 추가 후 같은 바이너리에서 `fine_enabled`만 바꿔 비교한 세 시험점의
위치 오차는 각각 2.245→0.297cm, 1.365→0.278cm, 1.031→0.436cm였다.
정답 후보가 1위인 결과는 3/3으로 유지됐고, 추가 계산은 약 0.005~0.167초였다.
이는 각 조건 한 번씩의 잡음 없는 합성 스캔 비교이며 실물 정밀도나 지연 보장은 아니다.
`tim_fine_disabled.json`, `tim_fine_enabled.json`에 지도·바이너리 해시와 실제 파라미터를
기록했다. 두 결과는 `reports/localization_startup_validation` 아래에 있다.
별도 모호성 검사에서는 긴 복도 시험점의 주행 보류가 그대로 유지됐다.
드리프트 보상·정밀 보정을 포함한 최종 검증은 C++ 코어 17개, 시작 감독기 58개,
실제 TIM→AMCL 연동 5개, Nav2 회귀 2개로 총 82개가 통과했다. 별도로 TIM의
읽기 전용 파라미터 34개 변경 거절도 확인했다. 최종 기록은
`reports/localization_startup_validation/tim_accuracy_upgrade_validation.json`에 있다.
실제 Isaac 실행이나 Jetson에서 첫 시도 성공률이 얼마나 높아지는지는 아직 측정하지 않았다.

### 공통 제어 경로

`Nav2 → navigation_manager → control_mux → /cmd_vel`까지 같은 코드다. sim에서는 `amr_sim_bridge`가 Isaac 프로세스 안에서 ROS DDS에 참여하고 바퀴 관절의 속도 목표만 전달한다. real에서는 기존 CAN 브리지가 처리한다. `control_mux.yaml`의 기구 치수와 1.8 rev/s 제한이 DWB critic과 양쪽 구동 경로의 공통 기준이다.

PhysX 실제 바퀴 회전량을 기존 `WheelOdometry`에 전달한다. 양쪽 URDF 바퀴 축은 같은 부호이므로 CAN의 왼쪽 encoder 반전은 적용하지 않는다. IMU는 base_link의 CAD 측정 위치에 부착하며 특정 가속도(정지 시 +g), 각속도와 자세 미측정 규약을 발행한다. 실물 gyro bias를 가상 센서에서 빼지 않는다. LiDAR는 PhysX 충돌 형상에 대해 회전 스캔을 수행한다. 3240점·10 Hz와 laser 프레임을 사용하며 각 ray 취득 시각의 센서 자세를 반영한다. 실물 bag `bags/012425/bag`의 LaserScan도 3,240점, 0.05–30 m를 선언한다. 이는 ROS 메시지의 유효 범위이며 센서의 모든 조건에서 보장되는 탐지 거리를 뜻하지 않는다. 실측 지연·노이즈 모델은 별도 보정이 필요하다.

2026-10-04 실물 `012425` bag의 메시지 timestamp로 센서 주기를 대조했다.
활성 수신 구간 기준 휠 odometry와 joint states는 약 100.077 Hz, IMU는 10.000 Hz,
LiDAR는 10.011 Hz, EKF는 30.000 Hz였다. `sim/config/sensors.json`의 휠 주기를
10→100 Hz, LiDAR를 8.8→10 Hz로 변경했다. IMU 10 Hz와 EKF 30 Hz, 휠 twist 및
IMU 공분산은 이미 일치한다. 이 변경은 ROS 발행 주기 정합이며 원시 CAN 갱신률이나
노이즈·지연·접촉 물리 보정을 의미하지 않는다. 대조 기록은
`reports/sensor_alignment_012425/bag_measurements.json`에 있다. 이전 성능 시험의
8.8 Hz 수치는 당시 조건이므로 그대로 보존한다.

변경 후 Isaac의 6초 headless 검사에서 휠 100.083 Hz, IMU 10.008 Hz,
LiDAR 10.000 Hz(시뮬레이션 시간 기준), 실시간 처리율 0.965를 확인했다.
명령 단절 정지와 관련 단위·ROS 통합 검사 37개도 통과했다. TIM의 두 합성 스캔
비교에서는 360빔 약 4.7–5.0초, 1,080빔 약 10.0–10.5초였으며 모두 올바른
최상위 후보를 반환했다. 이는 실제 재탐색 성공률이나 반복 복도 모호성 해결을
입증하지 않는다. 최종 기록은 `reports/sensor_alignment_012425/validation_summary.json`이다.

TF 소유자는 AMCL map→odom, EKF odom→base_footprint, robot_state_publisher 내부 TF다. `/sim/ground_truth`는 평가 전용이다. 구동기 모델은 50 ms 갱신, 벽시계 300 ms 명령 단절 처리, 기존 가감속 가정을 적용한다. 상위 제어기에 별도 램프를 추가하지 않았다. 정지·리셋 후 같은 시험을 계속하지 않는다.

## 시나리오와 비교

`sim/config/scenarios.json`에 직선·좌우 코너·S자·1.2 m 통로·장애물 등장 시나리오가 있다. `scripts/build_sim_map.py`는 이 합성 시험장의 ROS 지도와 PhysX 벽을 만든다.

기본 `world:=floor4`와 `./run_amr.sh`는 동일한 `sim/capstone_amr_4_floor.usda`를 연다.
주석 이미지 기반 3 m 벽, 철문 15개, 엘리베이터 문 1개를 포함하며 엘리베이터 앞
`(11.5, 3.31, yaw=0)`에서 시작한다. GUI에서 `O`는 철문, `P`는 엘리베이터 문을 전환한다.
`build_scene.sh`는 상세 장면의 정적 충돌 형상을 레이저 높이에서 투영한
`maps/sim_floor4/floor4.yaml`도 생성한다. 움직이는 문짝은 정적 지도에서 제외하고
실시간 LiDAR 장애물로 처리하므로 열린 문을 통과하는 경로를 막지 않는다.
원래 실물용 `maps/G_4_floor/4_floor_0522.yaml`은 실물 실행에서 계속 사용한다.

조이스틱은 시뮬레이션 실행에 포함된다. LB를 누른 동안 수동 주행, RB로 AUTO를
전환한다. 자동 시험에서는 `start_joy:=false`로 하드웨어 입력과 시험 입력을 분리한다.
`./scripts/experiment.sh sim`은 `joy_teleop.yaml`의 `device_name`과 같은 장치를
자동 선택한다. 하나면 바로 연결하고, 동일 이름이 중복 등록돼 있으면 실제 버튼·스틱
변화가 들어오는 장치를 기다린다. 이 경우 사용할 컨트롤러의 LB를 한 번 눌렀다 떼거나
스틱을 움직이면 된다. 자동 선택 로그가 나온 뒤 LB를 놓았다가 다시 누르면 수동 주행한다.
입력 없는 장치의 초기 축값이나 SDL 번호를 근거로 정상 장치를 추측하지 않으며,
여러 장치가 함께 반응하면 한 대만 조작할 때까지 대기한다. 이 대기는 별도 프로세스에서
진행하므로 시뮬레이션·위치 추정 실행을 지연시키지 않는다.

선택은 실행 시 수행하며 장치 번호를 저장하지 않는다. 실행 중 Bluetooth 재연결로
입력이 끊긴 경우 스택을 재시작하면 새 번호로 다시 선택한다. 수동 진단이 필요할 때만
`ros2 run joy joy_enumerate_devices`로 확인한 인덱스를
`./scripts/experiment.sh sim joy_device_id:=1`처럼 지정한다. 독립 bringup에서도
`joy_device_id:=1`을 사용할 수 있다. 명시적 ID는 자동 선택보다 우선하며 이름 우선
선택을 해제한다. 사용자 YAML에서 `device_name`이 빈 경우에도 기존 `device_id` 설정을
유지한다. `start_joy:=false`와 `manual_test:=true`는 실제 조이스틱 드라이버와
자동 선택 프로세스를 실행하지 않는다.
다음 명령은 시뮬레이션에서만 합성 조이스틱으로 최대 속도·회전·급반전·LB 해제·
연결 단절 정지를 시험하고 종료한다. 실물 모드에서는 실행을 거부한다.

```bash
./python.sh sim/scripts/validate_stability.py
./scripts/experiment.sh sim headless:=true manual_test:=true
```

결과는 `reports/floor4_stability.json`, `reports/floor4_manual.json`이다.
물리 안정화는 1 mm 둥근 타이어 모서리, 접촉 탄성·감쇠와 단일 평탄 바닥을 사용한다.
원통 코어 크기를 보정하여 원래 바퀴의 외경·폭을 유지하며, 차체 위치·속도·기울기를
강제 고정하지 않는다. 이 타이어 값은 시뮬레이션 안정화용이며 실측 보정값은 아니다.

2026-10-04 장거리 주행에서 바닥 중앙(x≈21.6 m) 부근의 접촉 불안정을 재현했다.
기존 50.7×23.1×0.1 m 바닥의 `boundingCube` 충돌 근사는 둥근 바퀴 원통과의 접촉에서
차체를 떨리게 했다. 센서·Nav2 없이 일정 속도로 움직이는 시험에서도 최대 기울기
3.01°, 높이 변화 8.72 mm가 발생했다. 같은 표면을 정적 삼각형 메시
(`physics:approximation = "none"`)로 처리하면 각각 0.050°, 0.258 mm로 줄었다.
바닥의 좌표·두께·마찰, 바퀴 치수·구동기·접촉 탄성은 변경하지 않았다.
생성 코드와 실제 USD 레이어를 함께 수정했으며, 재생성 결과가 일치함을 확인했다.

접촉 보고는 `PhysxContactReportAPI.reportPairs`로 실제 장애물에 한정한다.
기존 지표에서 제외하던 바닥 접촉을 수집할 때 PhysX 110.3.2의 삼각형 재질 조회
경고가 대량 발생했기 때문이다. 물리 충돌은 그대로 유지되며 정적 장애물과
도중에 활성화한 kinematic 장애물의 접촉 보고를 실제 PhysX 시험으로 검증했다.
원인 비교와 주행 기록은 `reports/driving_instability_20261004/`에 있다.

최종 검증은 휠 100 Hz·LiDAR 10 Hz·IMU 10 Hz를 유지하고 자동 위치 찾기 후
장거리 목표 도착, 180° 회전, 출발점 복귀를 모두 완료했다. 장애물 접촉 0회,
실시간 비율 1.000, 차체 높이 변화 0.533 mm, 최대 기울기 0.073°였다.
물리 단독 직진·좌우 회전 시험과 기존 회귀 시험 19개도 통과했다.
단, 전체 주행의 수직 가속도 피크는 6.874 m/s²로 진단 기준 5 m/s²를 넘었다.
총 49,532개 구간 중 6개 샘플, 연속 5–10 ms의 피크였으며 큰 차체 떨림·들뜸은
재현되지 않았다. 이 항목을 임의로 통과시키지 않아 `validation_summary.json`의
엄격한 전체 `passed`는 `false`로 보존한다. 상세치는 `residual_peak_review.json`에 있다.

```bash
# 문제가 발생했던 바닥 중앙 통과와 좌우 회전을 재검증
./python.sh sim/scripts/probe_floor_contact.py --assert-stable \
  --report reports/floor_contact_regression.json
# 장애물 접촉 보고가 유지되는지 확인
./python.sh sim/scripts/probe_floor_contact.py --verify-reports \
  --report reports/obstacle_contact_reporting.json
# 이전 충돌 형상의 실패를 별도 USD 세션에서만 재현
./python.sh sim/scripts/probe_floor_contact.py --floor legacy-box \
  --report reports/floor_contact_legacy.json
```

`scripts/benchmark_runtime.py --initialization automatic`은 실제 TIM 초기화부터
Nav2 주행까지 검증한다. `AMR_SIM_DYNAMICS_TRACE=/absolute/path/trace.csv`를 지정하면
물리 단계마다 차체 자세·속도와 적용 명령을 기록한다. 비교용
`AMR_SIM_SENSOR_CONFIG` 또는 `run_ros_sim.py --sensor-config`는 해당 시뮬레이션 실행에만
적용되며 기본 `sim/config/sensors.json`은 변경하지 않는다.

```bash
/usr/bin/python3 scripts/run_sim_suite.py \
  --navigation-config src/amr_system/amr_stack/config/navigation.yaml \
  --scenario straight --scenario corner_left --scenario corner_right \
  --output reports/trials/baseline_suite
# --scenario를 생략하면 전체 6종, --final-seeds이면 고정된 10개 센서 노이즈 시드
/usr/bin/python3 scripts/compare_trials.py \
  --baseline reports/trials/baseline_suite/straight_*/metrics.json \
  --candidate reports/trials/candidate_suite/straight_*/metrics.json
```

각 시험은 새 프로세스·같은 시작 자세로 시작한다. AMCL 초기 자세는 시험 조건으로 입력하며 ground truth를 피드백하지 않는다. Jazzy 기본 AMCL에는 `random_seed` 옵션이 없으므로 저장하는 seed는 가상 센서 RNG에만 적용된다. 기본 노이즈는 0이며 실물 보정값이 아니다. 동일 seed가 전체 ROS 스케줄링이나 AMCL의 동일 궤적까지 보장하지 않는다.

시험 디렉터리에 rosbag, 실제 ROS 파라미터, 적용 Nav2 YAML, 소스·설정 해시, 추정·ground-truth 궤적, 지표를 저장한다. 기본 기록에서 고용량 `/evaluation` 토픽은 제외한다. 기존 스택 운용 중에는 `AMR_EXPERIMENT_MODE=sim|real ./scripts/record_trial.sh 이름`을 사용한다.

수동 기록기는 `capture_parameters.py`의 한 DDS 접속으로 대상 노드를 동시에 읽는다.
전체 파라미터 조회에 벽시계 기준 8초 제한을 적용하고 sim에서는 실물용
`cmd_vel_to_can` 노드를 조회하지 않는다. 중첩된 costmap 설정과 EKF의 초기화된 값은
보존하며, 미설정 파라미터는 `params/summary.json`에 따로 표시한다. 노드 누락·응답
실패는 `.failed.txt`로 남기므로 비교 전에 확인한다. 준비 시간은 `session.txt`의
`preparation_duration_s`, 조회 시간은 `params/summary.json`에서 확인한다.
녹화 토픽은 `--topics`로 지정하며, PhysX 접촉 횟수는 `/sim/status`에 들어 있다.
현재 Jazzy rosbag2는 `action_msgs/msg/GoalStatusArray`를 액션 형식으로 분류한 뒤
내장 메시지 정의를 비워 두는 경고를 낼 수 있다. 상태 토픽의 직렬화 데이터는
계속 기록되며 해당 ROS 인터페이스가 설치된 환경에서 읽을 수 있다. 외부 도구의
독립적인 해석에는 메시지 정의가 필요하다. 이 경고 때문에 상태 토픽을 제외하지 않는다.

비교기는 동일 세계·seed·고정 설정, 5회 이상, 충돌·명령 제한 위반 0, 전회 목표 도달, RMS 경로 오차 또는 각속도 명령 변동 20% 개선, 다른 지표 10% 이내 악화를 검사한다. 센서 지연·제어 주기 누락·실시간 비율 문제가 있는 기록은 비교에서 제외한다. 필수 측정이 없으면 통과시키지 않는다. 각속도 변동에는 정상 코너 회전도 포함되므로 직선 구간의 불필요한 변동 개선은 직선 시나리오에서 판단한다. footprint 여유거리는 지도 벽과 지정 2D footprint의 기하학적 거리이고, 실제 접촉 여부는 별도 PhysX 접촉 이벤트를 사용한다.

현재 DWB 기준 YAML은 변경하지 않았다. 코너 예비 시험에서 벽 접촉과 주기 지연을 발견했으므로 실물 이식용 개선 후보는 아직 선정되지 않았다. 전체 시나리오 5회·최종 10 seed 성능 검증도 완료된 것으로 표시하지 않는다.

## 제어기 재시작과 시간 이상 처리 (2026-10-04)

수정 전 Jetson 이식용 소스는 `../backups/jetson_before_time_recovery_20261004_160315/`에
보관했다. 현재 작업 파일과 미추적 파일 446개를 포함하며, 압축 해제 후 내용 해시와
아카이브의 원본 권한을 검증했다. `RESTORE.txt`에 별도 폴더 복원 절차가 있다.

Web UI의 TF 버퍼는 노드 시계에 연결한다. 시뮬레이션 시각 역행 또는 위치 감독기
실행 UUID 변경 시 이전 동적 표시와 요청을 폐기하고 TF 구독을 다시 생성해
`/tf_static`도 재수신한다. 정적 지도는 유지하며 새 지도 수신 시 버전을 올린다.
목표는 자동 재전송하지 않는다. `session` 사이의 `generation`은 비교하지 않으며,
한 번 종료된 session에서 늦게 도착한 상태를 거부한다. 감독기의 generation은
벽시계와 독립적인 실행 내 카운터다.

bringup은 `time_health`를 실행한다. `/scan`, `/wheel/odometry`, `/imu/data_raw`,
`/odometry/filtered`, `odom → base_footprint` TF의 시각과 수신 신선도를 확인한다.
기본 허용 지연은 0.75초, 미래 시각 허용은 0.1초, 정상 유지 시간은 1초다.
AMCL의 미래 시각 `map → odom` TF는 이 센서 시각 검사 대상에 넣지 않는다.
`/system/time_health`가 비정상이거나 heartbeat가 0.6초 끊기면 control_mux는 AUTO
출력을 0으로 막고 navigation_manager는 실행·대기 목표를 중단한다. Web UI에서도
새 목표 입력을 차단한다. MANUAL의 기존 LB·명령 만료 규칙은 유지한다.

실물에서는 ROS/시스템 시간 증가량과 monotonic 증가량 차이가 0.5초를 넘으면
시간 오류를 유지한다. CAN 수신기는 이전 시간·샘플 조합만 비우고 누적 휠 위치는
유지한다. 센서 시각을 현재 수신 시각으로 덮어쓰지 않는다. 진단의 `clock_resets`와
`wheel_rebases`로 연속성이 끊긴 사실을 남긴다. 컴퓨터 시간이 안정되고 새 센서·TF가
정상이며 로봇이 정지한 상태에서만 다음 명령으로 오류 유지 상태를 해제할 수 있다.

```bash
# 실물은 domain 7, 모든 관련 노드 use_sim_time=false
ROS_DOMAIN_ID=7 ros2 service call /system/reset_time_fault std_srvs/srv/Trigger '{}'
```

해제 후 새 초기 위치 검증(자동 위치 찾기 또는 수동 초기 위치)이 필요하며 이전
목표는 재개되지 않는다. EKF/AMCL이 시간 역행 후 새 데이터를 처리하지 못하면
정지 상태에서 제어 스택을 재시작하고 위치를 다시 검증한다. 단순 Web UI 연결 끊김은
로봇의 센서 시간 오류와 별개다. 프로그램 방식의 정지는 물리 비상정지의 대체가 아니다.

Jetson 한 대에서 모든 ROS 노드를 실행하면 같은 OS 시계를 사용한다. 여러 컴퓨터에
ROS 노드를 분산하면 동일 기준의 chrony/NTP 동기화를 구성해야 한다. 브라우저만
다른 PC에서 여는 경우 브라우저 PC의 시계는 TF 기준에 관여하지 않는다.
큰 보정은 제어 스택 시작 전에 끝내고, 운용 중에는 점진 보정을 사용한다.
chrony를 재시작하면 부팅 때 허용한 step 조건도 다시 적용될 수 있으므로 주행 중
시간 서비스를 재시작하지 않는다. 실제 Jetson의 OS 시간·서비스는 이 작업에서 변경하지 않았다.

```bash
chronyc tracking
chronyc waitsync 30 0.05 0 1
# chrony 동기화 상태를 확인한 뒤에만 실물 스택 시작(점검 실패 시 실행 안 함)
AMR_REQUIRE_TIME_SYNC=1 ./scripts/experiment.sh real
```

위 preflight는 로컬 chrony 상태와 보고된 보정량만 검사한다. 상대 장치의 설정과
통신까지 검증하는 것은 아니므로 분산 노드의 시간 동기화 상태도 확인해야 한다.
정책 근거: [ROS Clock and Time](https://design.ros2.org/articles/clock_and_time.html),
[chrony makestep](https://chrony-project.org/doc/4.6/chrony.conf.html#makestep).

회귀 검증은 `scripts/verify_web_restart.py --output <새 보고서 디렉터리>`로 실행한다.
`scripts/env.sh`를 로드하고 도메인 107을 지정해야 하며, 기존 스택/Web UI가 있으면
중단한다. 한 Web UI 프로세스를 유지한 채 Isaac 자동 초기화·3m 주행을 두 번 수행한다.
실물 장치에는 접속하지 않는다.

이번 검증에서 동일 Web UI 프로세스를 유지한 두 번의 Isaac 자동 초기화·3m 주행이
성공했고 Web UI의 `TF_OLD_DATA` 경고는 0건이었다. 단위·HTTP 회귀 206개, 위치 초기화
회귀 86개와 별도의 ROS 재시작·시간 오류·목표 차단·가짜 CAN 검증을 통과했다.
기록은 `reports/time_recovery_20261004/validation_summary.json`에 있다.
실물 Jetson 배포·주행과 실제 OS 시계 변경 시험은 수행하지 않았다.

## 실행 성능과 2D LiDAR 구현 비교 (2026-10-03)

현재 목적은 상세 4층 장면에서 기존 웹·AMCL·DWB 제어기를 튜닝하는 것이다.
이 목적에는 물리 충돌 형상을 대상으로 하는 회전식 2D 스캔을 유지하면서,
광선 계산을 배열 단위로 처리하고 화면 렌더링과 센서 주기를 분리하는 구조가 적합하다.
시뮬레이터를 바꾸거나 LiDAR 점 수를 줄이는 것을 우선 해결책으로 삼지 않는다.

| 공식 구현 사례 | 계산 방식 | 이 프로젝트에 참고할 점 |
| --- | --- | --- |
| [Gazebo Harmonic](https://gazebosim.org/docs/harmonic/sensors/) | Ogre2 `gpu_lidar`, 예제는 수직 1줄·수평 640개·10 Hz | 엔진에서 광선을 처리하고 정해진 센서 주기로 스캔을 전달한다. 예제의 해상도를 실제 센서 설정으로 그대로 복사하지 않는다. |
| [Webots LiDAR](https://github.com/cyberbotics/webots/blob/master/docs/reference/lidar.md) | OpenGL 깊이 버퍼에서 스캔 행을 추출 | 2D 거리 배열만 필요하면 추가 point cloud 생성을 피한다. 센서 자체 형상과 최소 거리의 차폐 처리를 확인한다. |
| [Isaac Physics Raycast](https://docs.isaacsim.omniverse.nvidia.com/latest/sensors/isaacsim_sensors_physics_raycast.html) | C++ 센서에 광선 배열·광선별 시간 오프셋 제공 | 렌더링에 의존하지 않는 물리 단계별 회전 스캔의 장기 후보다. 현재 설치본의 Python 데이터 접근 문제를 아래와 같이 확인했다. |
| [Isaac RTX 2D LiDAR](https://docs.isaacsim.omniverse.nvidia.com/latest/ros2_tutorials/tutorial_series/tutorial_ros2_rtx_lidar.html) | RTX 센서와 ROS LaserScan writer | 공식 2D 예제가 있으며 완성된 스캔을 발행한다. 재질·반사 특성까지 평가할 때 후보로 삼는다. 이번 작업에서는 RTX 전환 성능을 측정하지 않았다. |

권장 데이터 흐름은 다음과 같다. ground truth는 평가용으로만 구독한다.

```text
PhysX 200 Hz ─ 바퀴 100 Hz / IMU 10 Hz ─ EKF 30 Hz ─ odom → base_footprint
           └ 2D LiDAR 10 Hz ─ AMCL                  ─ map → odom
                                            ↓
웹 목표점 ─ navigation_manager ─ Nav2/DWB 20 Hz ─ control_mux ─ 가상 바퀴 구동
```

센서의 회전 스캔·3,240개 광선·움직이는 문 감지·로봇 자세 반영을 유지한다.
최대 자동주행 설정 0.4 m/s, 0.6 rad/s에서는 한 스캔(100 ms) 동안 약 4 cm,
3.4도가 변할 수 있으므로 저속이어도 회전 스캔 시점을 무시하지 않는다.
[ROS LaserScan 정의](https://github.com/ros2/common_interfaces/blob/jazzy/sensor_msgs/msg/LaserScan.msg)에
맞춰 첫 광선 시각을 stamp로 사용한다. 현재 모델의 양 끝 각도를 포함하는 샘플링에서
`time_increment = scan_time / (N-1)`이다. 200 Hz 물리 단계로 광선 시점을 양자화하며,
센서 주기와 메시지의 거리 범위는 실물 bag과 맞췄으며, 노이즈·추가 전송 지연은 아직 실측 보정되지 않았다.

C++ 도입 전에 적용한 변경:

- 시뮬레이션 EKF의 `predict_to_current_time`을 활성화했다. 당시 입력 센서는 10 Hz였으며,
  필터가 측정 사이의 상태를 현재 시각으로 예측해 30 Hz TF를 출력한다. 실물 기본값은 false다.
  RotationShim이 현재 시각 TF를 기다리던 지연을 줄이며, TF stamp를 임의로 미래로 옮기지 않는다.
- LiDAR 각도·기본 방향을 미리 계산하고 각 물리 단계에서 필요한 광선의 회전을 NumPy로
  일괄 처리했다. 당시에는 각 광선의 PhysX 질의와 자기 형상 제외를 Python에서 호출했다.
  아래 초기 성능 표는 이 Python 경로를 사용한 결과이며, 후속 C++ 결과는 별도로 기재한다.
- 관절 인덱스를 재사용하고, 평가용 속도는 실제 발행이 필요한 주기에 읽는다.
- 웹 지도·costmap의 픽셀 색상 변환을 Pillow 테이블 연산으로 바꿨다. 모든 int8 입력값에 대해
  기존 이미지 색상과 투명도가 같은지 검사했다.
- headless 실행에서 숨겨진 기본 뷰포트의 렌더링도 비활성화했다. 사용 중인 LiDAR와 IMU는
  PhysX 센서이므로 화면 렌더링 없이 동작한다. GUI 실행의 화면은 계속 갱신한다.

`scripts/benchmark_runtime.py`로 GUI 1280×720·상세 4층·웹 HTTP 상태 조회를 동시에 사용하고,
같은 시작점에서 직진·좌회전·복귀의 세 목표를 비교했다. 센서·물리·제어 주파수와 DWB 후보 수는
변경하지 않았다. 결과 원본은 `reports/performance/`에 있다.

| 측정값 | 수정 전 | 수정 후 GUI 반복 2회 |
| --- | --- | --- |
| 실시간 비율(가상 시간/실제 시간) | 0.885 | 0.974–0.992 |
| Nav2 명령 간격 p95 | 111 ms | 약 70 ms |
| 제어 주기 누락 경고 | 140회 | 18회, 27회 |
| LiDAR 계산 평균 / 물리 단계 | 1.70 ms | 약 1.15 ms |
| 목표 도달 / 물리 접촉 | 3/3, 0회 | 각 3/3, 0회 |

최종 headless 설정에서도 같은 세 목표를 모두 도달하고 물리 접촉 0회를 확인했다.
숨은 뷰포트를 끄기 전/후 비교에서 실시간 비율은 모두 약 1.00이고, 제어 주기 경고는
28→1회, 화면 처리 함수 평균은 13.59→1.45 ms, 최대는 66.40→2.96 ms였다.
각 설정 한 번씩의 주행 결과이며 무경고 동작을 보장하지 않는다. 실행 중 EKF의
`use_sim_time=true`, `predict_to_current_time=true`, `frequency=30.0`도 직접 조회했다.
집계 결과는 `reports/performance/summary.json`, 최종 시험은
`reports/performance/optimized_headless_no_viewport/`에 있다.

웹 이미지 생성 단독 비교에서 지도는 68.7→8.9 ms, costmap은 112.2→17.7 ms였다.
이는 이미지 생성 비용이며 전체 웹 요청이 같은 배수로 빨라졌다는 뜻은 아니다.
기준·최초 개선 시험의 전체 명령 간격에는 목표 사이의 의도적인 약 1초 대기가 포함되어 있다.
후속 결과의 `within_goal_command_gap_s`는 서로 다른 목표 사이의 간격을 제외한다.
RTF에는 목표 사이 대기가 포함되므로 무제한 물리 처리량 수치로 해석하지 않는다.

C++ 도입 전 측정에서는 GUI `app.update()`에서 약 64–66 ms의 간헐적 지연, Python 전체 GC에서 약
128 ms의 정지가 관측됐다. GUI 지연의 후속 개선과 GC 발생 시점 확인은 아래에 기재한다. 3개 목표 시험에서
진행 정체 후 Nav2 재시도가 있었으므로 경로 추종 튜닝 완료나 실물 이식 승인으로 보지 않는다.
`recoveries` 필드는 피드백에서 관측한 최대값이며 전체 재시도 횟수가 아니다.
PhysX 작업 스레드를 0개로 바꾸는 비교도 했지만 주된 계산 시간 감소가 작아 기본값을
바꾸지 않았다. [NVIDIA 성능 문서](https://docs.isaacsim.omniverse.nvidia.com/6.1.0/reference_material/sim_performance_optimization_handbook.html)도
스레드 수를 장면에 맞춰 측정하도록 안내한다.

별도의 Physics RaycastSensor API는 `sim/scripts/probe_native_lidar.py`로 같은 4층 장면에서 확인했다.
설치된 Isaac Sim 6.1의 `reading.depths`가 shape `(3240,)`, strides `(0,)`로 반환되어
3,240개 값이 첫 거리 하나로 반복되는 현상을 재현했다. `hit_positions` 배열은 별도 좌표값을
제공하므로 위치에서 거리를 재구성하는 방법은 후보지만, 미검출·회전 중 자세·스캔 경계·자체
차폐를 추가 검증해야 한다. 이 API를 그대로 기본 경로에 넣지 않았으며 런타임 설치 파일도
수정하지 않았다. 자세한 측정은 `reports/performance/native_lidar_probe.json`에 있다.
정지 상태에서 교차점으로 재구성한 거리는 기존 질의와 최대 약 0.27 mm 차이였으며,
이는 센서의 전체 동작 검증 결과가 아니다.

후속 구현은 아래의 PhysX C++ 직접 질의를 사용한다. 위 `reading.depths` 배열을
사용하지 않으며 Isaac 설치 파일도 수정하지 않는다. 정적 점유지도에서만 광선을 계산하는
간이 2D 모델은 대량의 파라미터 예비 탐색 후보지만, 움직이는 문·차체 기울기·바퀴 접촉을
포함하는 상세 장면의 최종 확인은 현재 PhysX 장면을 대상으로 수행한다.

### PhysX C++ 배치 질의 적용

기본 LiDAR 경로를 `sim/native/amr_lidar_native.cpp`로 변경했다. PhysX의 기존 충돌
가속 구조를 그대로 사용해 거리 질의·자기 충돌체 제외·가장 가까운 hit 선택·미검출 처리를
C++ 안에서 완료한다. Python과는 물리 단계마다 방향 배열과 거리 배열을 주고받는다.
3,240점·10 Hz 회전 취득, 첫 광선 timestamp, 로봇 기울기와 움직이는 문 감지를 유지한다.

사용한 외부 라이브러리는 **PhysX 장면 질의 API + pybind11 2.13.6**이다.
[Gazebo의 GpuRays](https://github.com/gazebosim/gz-sensors/blob/gz-sensors8/src/GpuLidarSensor.cc),
[CARLA의 Unreal C++ 장면 질의](https://github.com/carla-simulator/carla/blob/ue5-dev/Unreal/CarlaUnreal/Plugins/Carla/Source/Carla/Sensor/RayCastSemanticLidar.cpp),
[Rmagine의 Embree/OptiX 백엔드](https://github.com/uos/rmagine)를 실제 소스로 비교했다.
현재 장면을 다른 엔진에 복제하는 통합 비용을 고려해 PhysX 재사용을 선택했다.
구현 구조·라이선스·SDK 호환성·재현 명령은 [네이티브 LiDAR 문서](../sim/native/README.md)에 있다.

소스·벤더 헤더·라이선스·빌드 산출물은 모두 워크스페이스 안에 있다. 첫 실행에 GCC로
자동 빌드하며, 이후 캐시를 사용한다. 현재 검증한 `omni.physx 110.3.2` 외 버전에서는
오류로 중단하며 `AMR_LIDAR_BACKEND=python`을 명시하면 기존 경로로 비교할 수 있다.
`/sim/status`의 `lidar_backend`로 실제 사용한 구현을 확인할 수 있다.

동일한 물리 장면에서 254,673개 광선을 비교한 결과 최대 거리 차이는 **0 m**였다.
직진·회전·곡선 주행, 문 개폐, 하우징 제외, 다른 로봇, 거리 범위와 미검출, 동적으로
추가한 자체 형상을 포함한다. 결과 원본은 `reports/performance/cpp_lidar_equivalence.json`이다.

동일한 GUI 1280×720·4층·Nav2·웹 HTTP 조회 조건에서 각 백엔드를 한 번씩 실행했다.
이 표의 Python 기준은 앞선 NumPy 최적화를 포함하며, C++와 센서·제어 설정이 같다.

| 측정값 | Python | C++ 기본 경로 |
| --- | --- | --- |
| LiDAR 처리 평균 / 물리 단계 | 1.132 ms | 0.283 ms |
| LiDAR 처리 p95 / 물리 단계 | 1.401 ms | 0.357 ms |
| 실시간 비율 | 0.991 | 1.000 |
| 목표 도달 / 물리 접촉 | 3/3, 0회 | 3/3, 0회 |
| TF 외삽 오류 | 0회 | 0회 |
| 제어 주기 누락 경고 | 29회 | 27회 |

LiDAR 처리 구간은 약 4배 빨라져 평균 시간이 약 75% 줄었다. 원본은
`reports/performance/cpp_python_gui/`, `cpp_gui/`, 집계는 `cpp_summary.json`이다.
전체 시뮬레이터 프로세스 CPU 평균은 한 코어 100% 기준 약 647%→704%여서 전체 CPU
사용량 감소가 확인된 것은 아니다. 두 시험은 Nav2 재시도·소요 시간이 다르며,
단일 주행의 CPU·완주 시간·경고 수 차이를 C++의 안정적인 개선율로 해석하지 않는다.
당시 GUI 갱신에서 약 67 ms 최대 지연과 Python GC에서 약 131 ms 정지가 관측됐다.
이 변경은 LiDAR 계산 비용 개선이며, 전체 제어 주기 경고 제거 또는 제어기 튜닝 완료를
뜻하지 않는다.

### GUI 로딩 대기 최적화

Kit CPU 추적에서 긴 프레임 7개 모두 `GeometryStreaming : postSync`의
`Wait for budget`에 약 60 ms를 소비했다. 같은 프레임에서 메인 스레드의 실제 CPU
사용 시간은 약 4–6 ms였다. RTX 예산 대기가 끝날 때까지 같은 실행 루프의 물리·센서·
`/clock` 갱신이 밀리는 것이 반복적인 지연의 원인이었다.

GUI 실행에 [공식 SimulationApp의 `sync_loads=False`](https://docs.isaacsim.omniverse.nvidia.com/latest/py/source/extensions/isaacsim.simulation_app/docs/api.html)를
초기화 시점부터 적용했다. 이 옵션은 렌더링이 자산 로딩 완료를 동기적으로 기다리는 동작을
해제한다. 장면 초기화 후에만 세부 설정을 바꾸는 시험에서는 60 ms 대기가 남았으므로
기본 적용은 생성자 설정을 사용한다. 물리 단계·센서·제어 주파수, 해상도, 조명과 렌더링
품질은 유지한다. 별도 시험한 비동기 렌더링은 USD 잠금 대기가 남아 채택하지 않았다.
`/app/asyncRendering`은 false이며 headless의 기존 설정도 유지한다.

같은 코드·상세 4층·GUI 1280×720·웹 HTTP 조회·세 목표를 사용하는 대조 시험:

| 측정값 | 기존 동기 로딩, 1회 | 변경 후, 반복 2회 |
| --- | --- | --- |
| GUI 갱신 p99 | 65.12 ms | 10.03 ms, 6.19 ms |
| GUI 갱신 최대 | 67.09 ms | 323.23 ms, 8.23 ms |
| 제어 주기 경고 | 17회 | 2회, 1회 |
| 목표 도달 / 물리 접촉 | 3/3, 0회 | 각 3/3, 0회 |
| 실시간 비율 | 1.000 | 0.989, 1.000 |

반복적인 60 ms 대기는 줄었지만 첫 개선 주행에 원인 미확인 단발성 GUI 지연 323 ms와
물리 단계 지연 137 ms가 있었다. 이 결과도 포함하며 최대 지연이나 무경고 동작을 보장하지
않는다. 두 번째 개선 주행의 남은 경고 1회는 진행 정체에 따른 costmap 초기화 직후였고,
다음 갱신을 기다리는 제어 구간이 약 105 ms였다. 로컬 costmap은 기존 10 Hz이고 Nav2는
초기화 후 지도 갱신을 기다린다. 제어기 튜닝은 별도 검증이 필요하다.

GC 측정에는 이제 발생 시각과 시뮬레이션 시간을 함께 저장한다. 이번 반복 시험의 약
128–139 ms 전체 GC는 모두 `sim_time_s=0`인 초기 로딩 단계에서 발생했다. 이전 기록의
GC 수치만으로 주행 중 제어 지연 원인이라고 해석하지 않는다.

기본 실행 명령은 같다. `AMR_RENDER_READY` 로그와 `/sim/status`의
`render_loading=nonblocking`으로 적용을 확인한다. 비교할 때만 다음을 사용한다.

```bash
AMR_RENDER_LOADING=synchronous ./scripts/experiment.sh sim
```

집계와 최종 소스 해시는 `reports/performance/gui_timing_summary.json`에 있다.
`gui_synchronous_control/`, `gui_nonblocking_startup/`, `gui_nonblocking_startup_2/`에
모든 주행 결과를 보관한다. `gui_nonblocking_1/`은 초기화 후 설정 변경의 중간 시험이다.
최종 GUI·문 개폐·센서·정지 검사는 `gui_nonblocking_verified/`에, 압축한 내부 추적은
`logs/gui_profiles/`에 있다. 새 자산을 실행 중 추가할 때 화면의 재질·형상이 몇 프레임
뒤에 나타날 수 있으므로, 픽셀 단위로 모든 자산 로딩 완료를 보장해야 하는 데이터 수집은
동기 로딩 모드를 사용한다.

재현할 때는 기존 도메인 107 스택을 종료하고 각 명령을 순차 실행한다.

```bash
ROS_DOMAIN_ID=107 /usr/bin/python3 scripts/benchmark_runtime.py \
  --output reports/performance/new_gui_nonblocking
ROS_DOMAIN_ID=107 /usr/bin/python3 scripts/benchmark_runtime.py \
  --output reports/performance/new_gui_synchronous --render-loading synchronous
./python.sh sim/scripts/probe_gui_runtime.py \
  --output logs/new_gui_trace --render-loading nonblocking --exercise-doors --trace
```

웹만 사용해도 실행 흐름은 같다. `headless:=true`는 Isaac 창을 생략하는 선택 사항이다.

```bash
amr
./scripts/experiment.sh sim headless:=true
# 다른 터미널
amr
ROS_DOMAIN_ID=107 ros2 launch amr_web_ui web_ui.launch.py use_sim_time:=true
```

재현 시험은 실행 중인 도메인 107 스택을 종료한 뒤 별도로 수행한다. 시험은 가상 조이스틱으로
AUTO를 허용하며 실제 CAN 송신 노드를 시작하지 않는다. 결과 디렉터리는 새 이름을 지정한다.

```bash
amr
ROS_DOMAIN_ID=107 /usr/bin/python3 scripts/benchmark_runtime.py --output reports/performance/new_gui
# 창 없이 비교: 같은 명령에 --headless true 추가
./python.sh sim/scripts/probe_native_lidar.py
```

## 나중에 실물 보정

실물 기록은 정지 60초, 0.1·0.2·0.4 m/s 직진·정지, ±0.2·±0.4 rad/s 회전·방향 전환을 각 3회 확보한다. 운용자가 기존 절차로 주행하고 기록한다. 모델 식별용과 검증용 기록을 분리한다.

```bash
/usr/bin/python3 scripts/analyze_response.py bags/실험/bag --output response.json
```

이 도구는 토픽 주기·누락과 명령/바퀴의 겉보기 응답 지연을 분석한다. 지연에는 telemetry·필터·램프 영향이 섞여 있으므로 ODrive 통신 지연만으로 해석하지 않는다. 물리 파라미터를 자동 변경하지 않는다. `validate_model_response.py`는 별도 검증 기록의 정상속도·상승시간·정지시간·정지거리와 측정 불확실성을 받아 10% 또는 불확실성 이내인지 검사한다. 미지의 관절 Coulomb 마찰은 0을 유지한다. 질량·관성을 궤적 맞추기용 자유 변수로 사용하지 않는다.

## Jetson 소스 릴리스와 복귀

```bash
# PC: 깨끗한 커밋에서 소스 묶음 생성
/usr/bin/python3 scripts/make_release.py --name experiment-infrastructure-v1
# Jetson: 기존 버전과 다른 releases/이름/ 폴더에 압축 해제
# 현행 장치/ROS 설정 읽기만 수행
./scripts/inventory_robot.sh /저장경로/robot-inventory
# Jetson에서 arm64로 빌드하고 CAN dry-run/기존 통합 시험 실행
./scripts/validate_release.sh
# 검증된 별도 릴리스 선택. 이 명령은 주행 노드를 시작하지 않는다.
./scripts/select_release.sh /새릴리스/amr_ws /운용경로/current
# 복귀도 보존한 이전 디렉터리를 같은 방법으로 선택한다.
```

x86 바이너리는 묶지 않는다. 릴리스 manifest의 `candidate_gate_passed: false`는 환경 코드 릴리스이며 실물 적용 성능 승인으로 해석하지 않는다. 실제 STM32 firmware identity와 ODrive readback 값은 아직 미확인이다. 백업 C 파일의 존재로 업로드된 펌웨어를 단정하지 않는다. 실물 비교는 0.2 m/s에서 시작하고 통과 후 기존 0.4 m/s 범위에서 기준/후보를 교대로 평가한다. 독립 측정 장비가 없는 실물 결과는 추정 궤적 기준으로 기록한다.

## 근거 문서

- [NVIDIA Isaac Sim 6.1 ROS 설치](https://docs.isaacsim.omniverse.nvidia.com/6.1.0/installation/install_ros.html)
- [ROS Jazzy Ubuntu deb 설치](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html)
- [Jazzy AMCL 소스](https://github.com/ros-navigation/navigation2/blob/jazzy/nav2_amcl/src/amcl_node.cpp)

자동 비교의 품질 기준: scan age 0.3 s, odometry age 0.25 s, 실제 관측한 Nav2 명령 간격 75 ms 초과, 실시간 비율 0.9 미만, 추정 pose 누락, 0.1 s 안에 0.2 m/0.35 rad를 넘는 추정 pose 점프를 확인한다. 로컬 개발용 기준이며 실제 센서 기록으로 정교화해야 한다. 현재 실물 보정은 완료를 주장하지 않고 후속 작업으로 남긴다.


## 부록: 시뮬레이션 주행 기록 (2026-10-13)

### 4층 호실 주행 (sim)
- 호실 좌표: `sim/config/floor4_rooms.json` (401~406, 남쪽 벽 철문, 동쪽 끝부터 401)
- 실행: `python3 scripts/sim_demo.py --room 401 402 ...` (Isaac 창 + AUTO 유지, RViz는 `--rviz`)
- 웹 UI 조작: `ROS_DOMAIN_ID=107 ros2 launch amr_web_ui web_ui.launch.py use_sim_time:=true port:=8080`
- 2026-10-13 결과: 401~406 모두 도착, 실제 위치 오차 0.10~0.22m / 5~7°
- 403 첫 시도에서 긴 복도 제자리 회전 중 AMCL/TF 불일치로 localization READY가 약 3초 해제되어 목표가 중단됨(재시도에서는 재현되지 않음). 실차에서도 관찰할 항목.

### 전역 경로 평활화 (2026-10-13)
- 변경: `navigation.yaml` Smac2D smoother `w_smooth 0.3→0.95, w_data 0.2→0.02,
  max_iterations 1000→4000, refinement_num 4→6`. 플래너·inflation(1.6 m)·travel multiplier는 그대로.
- 원인: 8방향 격자 계단 + 벽이 꺾이는 세로 복도에서 경로가 정중앙(medial axis)을 그대로 따라 흔들림.
- 오프라인 경로 비교 `scripts/benchmark_global_planner.py NAME [overrides.yaml]` (domain 142, Isaac 불필요):
  6개 경로 합계 방향 반전 29→0회, 회전량 약 60% 감소, 길이 약 1% 감소, 최소 벽 거리 0.80→0.75 m
  (세로 복도 폭 1.6 m 병목 지점). Theta*, Lattice(diff 0.5 m)는 더 나빴음.
- Isaac 실주행 비교(floor4, run_sim_trial): 세로 복도 진입 회전량 150→76°, 401호 127→94°, 반전 2→0,
  소요 시간 동일, 최소 벽 거리 0.74→0.68 m(코너 안쪽), 가로 복도 중앙 편차 2.0→4.5 cm.
- 이 파일은 Jetson 실차와 공용이다. 실차 반영 시 코너 안쪽 여유를 확인할 것.
