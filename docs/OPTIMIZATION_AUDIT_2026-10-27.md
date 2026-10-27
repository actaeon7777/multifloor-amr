# 최적화 조사 — 2026-10-27

## 조사 범위와 결론

현재 checkout(`40e1d6a` + 사용자의 미커밋 변경)을 기준으로 ROS 제어·CAN·위치 추정·Nav2·깊이 센서·엘리베이터·웹·중앙 관제·빌드·기록·시뮬레이션·RL·STM32 경로를 조사했다. 실행 빈도, 데이터 복사, 잠금, 직렬화, 반복 계산과 기존 프로파일을 중심으로 살폈다. 모든 파일의 모든 분기를 검증한 것은 아니다.

**총 28개 개선 후보를 정리했다. 우선할 것은 측정 집계 수정, 빌드 자원 제한, CAN 실행기, 중복 속도 발행, 웹 준비 상태 조회, PNG 잠금, 중앙 서버의 불필요한 상태 복사다.** 수치로 확인한 것은 과거 프로파일의 CPU 사용량이며, 아래 제안의 실제 절감률은 아직 측정하지 않았다.

이번에는 코드·설정 변경, ROS 노드 실행, CAN 접근, 빌드, 부하 시험을 하지 않았다. `ps`에는 도구 실행 프로세스만 보여 실차의 운행 여부와 현재 노드 부하를 확인할 수 없었다. 문서만 추가했다. 특히 엘리베이터 미커밋 변경은 분석 대상으로만 읽었으며, 10월 25일 프로파일에 그 비용이 반영됐다고 가정하지 않았다.

## 1. CPU 자료를 다시 계산한 결과

자료:

- `reports/profile/20261025_144835_before_opt.csv`: 187개 표본, 마지막 경과 시간 198.7초.
- `reports/profile/20261025_201213_after_opt.csv`: 220개 표본, 마지막 경과 시간 233.9초.
- 같은 폴더의 `pyspy_*_after.txt`와 [기존 실험 기록](EXPERIMENTS.md).

아래 표는 시작 5초를 제외하고 **장시간 기록된 개별 PID**를 선택한 결과다. CPU 100%는 코어 하나다. 평균은 CSV 표본의 산술평균이며 시간 가중 평균은 아니다. 서로 다른 시각의 주행이라 동일 조건의 엄밀한 A/B는 아니다. 이후 STVL 교체와 엘리베이터 변경도 있어 현재 CPU 수치로 해석하면 안 된다.

| 프로세스 | 변경 전 평균 % | 변경 후 평균 % | 변경 후 p95 % | 변경 후 PID |
|---|---:|---:|---:|---:|
| controller_server | 30.66 | 28.62 | 41.4 | 86730 |
| web_ui 실제 작업 프로세스 | 26.47 | **19.11** | 24.2 | 88659 |
| cmd_vel_to_can | 16.09 | **16.82** | 21.6 | 86700 |
| navigation_startup | 25.46 | 16.29 | 18.8 | 86748 |
| bt_navigator | 14.54 | 12.95 | 16.9 | 86738 |
| navigation_manager | 16.47 | 11.04 | 14.1 | 86744 |
| can_telemetry | 13.15 | 9.80 | 12.0 | 86695 |
| time_health | 12.04 | 9.58 | 11.3 | 86693 |
| elevator_monitor | 11.73 | 6.99 | 8.5 | 86751 |
| floor_mission | 11.62 | 4.22 | 5.6 | 86754 |

### O01 · P0 — 프로파일러 집계부터 수정

근거: [profile_nodes.py](../scripts/profile_nodes.py) 38–44, 80–107, 111–119행.

현재 명령행 전체의 부분 문자열로 이름을 정하고, 같은 이름의 **모든 프로세스 표본을 하나의 리스트에 넣어 평균**을 낸다. 실행기와 실제 작업 프로세스를 합산하지 않고 평균하므로 CPU가 희석된다. 프로파일러 자신의 출력 파일명에 노드 이름이 들어가도 그 노드로 분류될 수 있다.

후속 CSV의 `web_ui`는 PID 88643(220표본, 평균 0%), 88659(220표본, 평균 18.86%), 91147(14표본, 평균 8.01%)이 섞인다. 기존 문서의 약 9.4%를 실제 웹 작업 프로세스 비용으로 보면 안 된다. 짧은 PID의 정확한 명령행은 CSV에 없어 사후 확정할 수 없다. 시작 5초 제외 시 주요 작업 PID는 19.11%다.

개선: 실행 파일·ROS 노드·부모 PID·생성 시각을 함께 기록하고 실행기/계측기를 별도 분류한다. PID별 통계와 같은 시각의 노드별 CPU 합계를 모두 낸다. 첫 CPU 표본을 버리고, 실제 경과 시간을 사용한다. RSS 합계는 공유 페이지가 중복되므로 필요할 때 PSS/USS를 별도 측정한다.

판정: 동일 이름의 0% 실행기와 20% 작업 프로세스를 넣었을 때 노드 합계가 20%가 되어야 한다. 220표본을 220초로 출력하지 않아야 한다. 과거 문서의 Python/C++ 합계도 같은 원자료 기준으로 재검토한다.

## 2. 우선 처리할 항목

### O02 · P0 — 빌드 스크립트가 Jetson 자원 제한을 적용하지 않음

근거: [build_ros.sh](../scripts/build_ros.sh) 12–20행, [검증 지침](VALIDATION.md) 준비 절차.

스크립트는 매번 `rosdep update/install`, 전체 build, 전체 test를 수행한다. `--executor sequential`, `CMAKE_BUILD_PARALLEL_LEVEL=2`, `MAKEFLAGS=-j2`가 없고 테스트 병렬성도 제한하지 않는다. 지침의 안전한 수동 빌드 명령과 불일치한다.

개선: 의존성 설치를 별도 준비 단계로 분리하고 빌드의 기본 자원 제한을 명시한다. 전체 검증 모드는 유지하면서 개발용으로 변경 패키지 선택을 지원한다. 테스트에서도 자원 제한을 적용하고 ROS 시험은 지정 도메인·localhost로 실행한다. 주행 중 실행하지 않는 조건은 유지한다.

검증: 스크립트의 명령 인자를 정적으로 확인하고, 차량 정지 시 제한된 실제 빌드를 수행한다. 이번에는 실행하지 않았다.

### O03 · P1 — CAN 송신 노드의 실행기 비용

근거: [cmd_vel_to_can.py](../src/amr_system/amr_can_bridge/amr_can_bridge/cmd_vel_to_can.py) 294–306행. 과거 CPU 평균 16.82%.

다른 주요 Python 노드는 `EventsExecutor`를 선택하지만 이 노드는 `rclpy.spin_once(node, timeout_sec=0.05)`를 반복한다. `/cmd_vel` 수신이 많을수록 실행기 호출 비용도 커진다.

개선: 기존 노드들의 실행기 선택 방식을 적용하되 SIGINT/SIGTERM 요청, 실행기 깨우기, 종료 시 stop 전송과 context 생존 순서를 보존한다. EventsExecutor의 종료·큐 동작은 실제 설치 버전에서 검증해야 한다. 단순히 호출 한 줄만 치환해서 끝낼 항목은 아니다.

검증: 가짜 CAN 버스 및 도메인 174에서 E-stop, ARMING 영속도, 수신 단절, stale command, 신호 종료, 송신 간격 p95/p99를 비교한다. 20 Hz CAN 송신과 watchdog을 줄이는 변경은 포함하지 않는다.

### O04 · P1 — control_mux의 중복 `/cmd_vel` 발행

근거: [control_mux.py](../src/amr_system/amr_control/amr_control/control_mux.py) 79–118행, [time_health.py](../src/amr_system/amr_control/amr_control/time_health.py) 147–166행, `cmd_vel_to_can.py` 200–224행.

20 Hz 타이머 외에 joy·manual·auto·time_health·estop 콜백마다 `publish_command()`를 호출한다. 기본 구성의 timer 20 + time_health 20 + estop 20 + auto 20 Hz만 계산해도 약 80회/초의 발행 경로가 존재한다. 이는 코드상 예상치이고 실측 토픽 주파수는 아니다. CAN 송신 자체는 20 Hz 타이머이므로 불필요한 DDS 수신·Python 콜백 비용이 발생할 수 있다.

개선: 최신 입력/lease 갱신과 주기 출력을 분리하고, 일반적인 동일 출력은 합친다. LB 인계, E-stop, 건강 상태 악화, 모드 변경, 영속도 전환은 즉시 발행한다. 타이머 heartbeat는 남겨 downstream watchdog을 충족한다. 값이 같다는 이유만으로 무기한 발행을 생략하면 안 된다.

검증: 도메인 173·174·190에서 정상 명령 지연, 타임아웃 정지, LB 해제, E-stop 후 새 입력 요구, 취소를 비교한다. 순수 타이머 방식은 새 명령에 최대 한 주기 지연을 추가하므로 바로 채택하지 않는다.

### O05 · P1 — 웹의 준비 상태 조회를 20 Hz 작업에서 분리

근거: [ros_bridge.py](../src/amr_system/amr_web_ui/amr_web_ui/ros_bridge.py) 158, 606–619행. 목표 전송 시 재확인은 402행 및 `send_floor_goal()`에 있다.

매 50 ms마다 두 Action 서버에 `server_is_ready()`를 호출하며 위치·벡터·스캔·grid origin도 함께 갱신한다. 화면 표시용 준비 상태에 20 Hz discovery 조회가 필요한지 검토할 가치가 있다.

개선: 명령 큐·시간 이상·취소 처리는 현 주기를 유지하고, 준비 표시만 0.5~1초 간격 또는 그래프 변경 시 갱신한다. 목표 전송 직전에는 기존의 직접 재확인을 유지한다. 지도 좌표계가 `map`인 레이어의 origin 계산도 변경 시 처리할 수 있다.

검증: 웹 단위·HTTP 및 도메인 193에서 Action 서버 중단/재시작, floor goal, 취소, timeout을 확인한다. CPU와 UI의 준비 표시 지연을 함께 비교한다.

### O06 · P1 — PNG 인코딩 동안 ROS 공유 잠금을 점유

근거: [server.py](../src/amr_system/amr_web_ui/amr_web_ui/server.py) 983–994행, [geometry.py](../src/amr_system/amr_web_ui/amr_web_ui/geometry.py) 63–85행, `ros_bridge.py` 209–218행.

`with shared.lock` 안에서 `grid.png`를 읽는다. 캐시 미스면 그 안에서 전체 이미지 변환과 PNG 압축을 한다. HTTP 스레드의 작업이 지도 업데이트·상태 갱신 콜백을 기다리게 만들 수 있다. PNG 지연 생성과 P 모드 최적화는 이미 있지만 잠금 범위 문제는 남아 있다.

개선: 잠금 아래에서 grid 데이터·크기·버전의 스냅샷만 확보하고 인코딩은 밖에서 수행한다. 결과를 캐시할 때 grid 객체와 버전이 여전히 같은지 확인하고, 응답의 이미지/버전 쌍을 일치시킨다. 요청 여러 개의 동일 버전 중복 인코딩도 합친다.

검증: PNG 생성과 costmap 교체/증분 갱신의 동시성, 이미지 버전, ROS callback 지연과 HTTP p99를 확인한다.

### O07 · P1 — 중앙 관제의 신선도 확인이 전체 상태 복사를 유발

근거: [remote server.py](../src/amr_system/amr_remote_bridge/amr_remote_bridge/server.py) 296–323, 344–375행.

`camera()`와 `layer()`는 신선도 확인만 하려고 `snapshot()`을 부른다. 그 안에서 스캔 점 목록을 포함한 `deepcopy(robot.state)`와 age 계산이 수행된다. 모든 로봇이 공유하는 `RLock` 안에서 실행된다. `floors()`도 전체 snapshot을 만든다.

개선: `_require_fresh_robot()`와 실제 응답 snapshot 생성을 분리한다. camera/layer에서는 필요한 객체와 버전만 읽는다. 전체 상태 응답은 불변 센서 데이터와 요청별 age/lease 데이터를 나눠 복사를 제한한다. 명령 검증 경로에서 실제 필요한 필드를 빠뜨리지 않도록 별도 확인한다.

검증: 원격 브리지와 `Control_Server/tests/test_concurrency.py`에서 stale/lease 실패 판정이 동일한지, 이미지 조회 중 다른 로봇의 exchange 지연이 줄었는지 확인한다.

## 3. 중간 우선순위: 불필요한 데이터 처리

| ID / 우선순위 | 근거와 현재 비용 | 개선 방향 | 유지할 조건·검증 |
|---|---|---|---|
| **O08 / P2** 엘리베이터 CAN 폴링 | `amr_can_bridge/elevator_can_link.py:142`의 10 ms poll + `:241`의 `spin_once`. 텔레메트리는 이미 blocking receive thread를 사용함. | 수신 대기/guard condition 또는 제한된 수신 큐로 전환하고 1 Hz 진단을 분리. 먼저 EventsExecutor만 적용한 대안도 비교. | RF 프레임 timestamp·순서·ACK·종료 보존. [CAN 규약](ELEVATOR_CAN_PROTOCOL.md)의 신선도 조건 확인. 수신량과 CPU는 새 측정 필요. |
| **O09 / P2** HTTP 연결 재사용 | `amr_remote_bridge/common.py:114`의 `urllib` opener. 설치된 `/usr/lib/python3.12/urllib/request.py:1329`는 `Connection: close`를 강제함. 원격 agent는 5 Hz 루프에서 로컬 상태/이미지·host 요청을 반복. | 호스트별 지속 연결을 재사용하는 클라이언트 검토. 연결 생성 비용·RTT부터 측정. | proxy 우회, redirect 차단, 인증, 응답 크기 제한, timeout 보존. 실패한 명령 POST의 자동 재시도 금지. 공유 연결의 thread 안전성 확인. |
| **O10 / P2** 표시용 스캔을 항상 처리·전송 | `ros_bridge.py:289–326`, `web/app.js:321–345`, `agent.py:81–145,246`. 화면에서 스캔을 감춰도 전체 점 목록이 상태 응답에 들어가고 브라우저에서 버킷을 생성함. agent는 이미지 요청 후 상태를 한 번 더 읽음. | 표시 데이터와 명령/health 메타데이터를 분리. 표시 안 하는 항목은 요청하지 않으며, 표시용 점 수·주기만 선택 가능하게 함. 새 scan revision일 때만 브라우저 재분류. | `/scan` 원본, AMCL, costmap, 문 감지 해상도는 유지. 신선도·map version·재접속 동작 검증. agent의 두 번째 조회는 오래된 상태 전송 방지이므로 무조건 제거하지 않음. |
| **O11 / P2** FastAPI 이벤트 루프의 JSON CPU 작업 | `Control_Server/app/http.py:28–31,80–85`: async 수신 경로에서 loads 후 유한성 검사 목적 dumps; 응답에서도 dumps. 큰 상태에는 scan과 base64 이미지가 포함됨. | 유한성 검사를 보존하며 JSON 중복 순회 축소, 큰 직렬화는 사전 처리/제한된 worker로 이동. 로봇별 잠금 검토. | NaN/Infinity/overflow·본문 크기·timeout 거절 유지. 1/8/32대와 복수 조회자로 p95/p99 측정. Registry가 프로세스 로컬이므로 uvicorn workers만 늘리면 안 됨. |
| **O12 / P2** TIM 지도 전처리 캐시가 요청마다 사라짐 | `amr_tim_localization/src/localizer.cpp:85–98`에서 core를 지역 객체로 생성. `relocalization_2d.cpp:63–105,119–138`에는 지도/샘플링 캐시가 있지만 다음 요청까지 살아 있지 않음. `fine_refinement.cpp:38,61`도 거리장·fine map을 다시 준비. | 서비스 수명 동안 불변 지도 전처리를 유지하고 coarse/fine 단계에서 재사용 가능한 거리장을 공유. 층별 캐시는 용량 제한. | 지도 내용·origin·해상도·샘플링/설정 키로 무효화. timeout 후 상태, 고정 seed, 후보 점수/위치 재현성 확인. 첫 요청의 전체 탐색 시간까지 사라지는 것은 아님. |
| **O13 / P2** READY 증거 평가의 반복 비용 | `navigation_startup.py:1079–1170,1559–1572`. 새 scan/pose 캐시는 이미 있으나 스캔이 계속 새로 오면 covariance·TF·360빔 품질 평가가 반복. | covariance는 pose revision별 캐시. hard validity와 품질 경고 계산을 분리한 뒤 후자의 주기만 실험적으로 낮춤. | stale/TF 상실/불명확한 자유 공간/초기 admission 판단을 함께 늦추지 않음. 도메인 194에서 움직이는 스캔·동일 stamp 교체·시간 점프 회귀. 10 Hz 전체를 1 Hz로 단순 변경 금지. |
| **O14 / P2** 엘리베이터 미션 대기 중에도 센서 처리 | 미커밋 `mission_node.py:219–221,604–658`. scan은 매번 투영·clearance를 계산. depth 콜백 초기에 phase 검사해도 typed PointCloud2 수신/역직렬화는 이미 발생함. | 비활성 상태에서는 최신 원자료와 수신 시각만 보관하고 필요한 phase에서 최신 샘플을 1회 처리. depth는 raw 구독 또는 phase별 구독을 비교. 각도·dtype·고정 회전 캐시. | phase 진입 시 새로운 유효 센서가 없으면 이동 금지. stale·epoch·시각 점프 처리 유지. 현재 미커밋 시험을 먼저 기준으로 삼아야 하며 10/25 CPU를 근거로 이 비용을 단정하지 않음. |
| **O15 / P2** marking 전용 점군에도 제외 점을 그대로 전달 | `depth_persistence_filter.cpp:114–150`: 높이/거리 밖 점과 invalid 점은 keep 기본값에 의해 남고, 일부 비지속 obstacle만 제거. 출력은 두 costmap과 미션으로 전달됨. | 별도 marking 전용 출력에 유효·높이·거리·지속성 조건을 통과한 점만 담는 방안. raw frustum clearing은 유지. 바이트 수 및 downstream 변환 비용 비교. | 현재 `/points_filtered`의 모든 소비자 확인. 빈 출력에서도 올바른 header/health 유지. 8/10/12 cm 물체, 바닥, 경계, 사각대 기억 검증. 단순 점 수 감소율을 CPU 감소율로 간주하지 않음. |
| **O16 / P3** 깊이 지속 필터의 hash node 할당 | `persistence.hpp:58–82`: bucket 재사용과 셀별 판단 캐시는 이미 있으나 unordered set/map의 clear·insert는 node 할당을 계속 유발할 수 있음. | allocation 측정 후 flat table·메모리 pool·프레임 표식 또는 bounded rolling grid 비교. | 음수/큰 좌표, 이웃 셀, 시간 초기화, 프레임 연속성 보존. 과거 필터 CPU 2.8% 수준이라 제어·웹보다 후순위. |

## 4. 주행 알고리즘: 계측 후에만 선택할 후보

다음 항목은 단순한 불필요 작업 제거보다 주행 품질과 장애물 대응에 미치는 영향이 크다. 현재 설정의 선정 이유와 기존 회귀 결과를 함께 비교해야 한다.

| ID / 우선순위 | 근거 | 조사/개선 방향 | 판정 |
|---|---|---|---|
| **O17 / P2** DWB 평가 예산 | `navigation.yaml:95–118`: 12×24=288 nominal 후보, 20 Hz, polygon footprint critic. 과거 controller CPU 평균 28.62%, p95 41.4%. | 컨트롤러 평가와 costmap 갱신 시간을 먼저 분리. critic별 비용·조기 탈락 비율을 측정하고 후보 수/순서의 작은 변화만 A/B. | 50 ms 주기 준수율, 좁은 복도·급회전·후방 목표·종점 각도·저상 장애물 확인. footprint 검사 삭제나 센서 주기 감소를 우선안으로 쓰지 않음. 288에서 추가 축소한 이득은 미확정. |
| **O18 / P2** Smac smoothing 예산 | `navigation.yaml:25–31`: max_iterations 4000, refinement_num 6, tolerance 1e-10. | 경로별 smoothing 시간과 실제 수렴 횟수를 기록해 덜 필요한 반복을 줄일 수 있는지 조사. | 기존 heading reversal 감소·벽 간격 유지. BT는 2 Hz 경로 유효성 검사 + 10초 만료 재계획이므로 planner가 2 Hz마다 무조건 전체 계획한다고 가정하지 않음(`navigate_to_pose_keep_nearby.xml:22–34`). |
| **O19 / P2** BT tick과 서비스 작업 | `navigation.yaml:430`의 bt_loop_duration 10 ms; 과거 bt_navigator 평균 12.95%. | active/idle 및 내부 callback별 프로파일 후 20 ms 등 후보를 격리 환경에서 비교. | cancel/LB 인계, 경로 무효화, 회복 동작 반응 시간 비교. CPU 수치만 보고 tick을 느리게 변경하지 않음. |
| **O20 / P3** ROS 프로세스/DDS 비용 | `navigation.launch.py`의 독립 Nav2 프로세스 및 여러 Python TF 소비자. | 계측에서 통신·역직렬화가 주요 비용일 때 Nav2 composition, 필요한 TF만 구독, 메시지 파이프라인 통합을 개별 검토. | 구성·장애 격리·기동 안정성 확인. camera는 기존 component load 실패로 단독 노드로 바뀌었으므로 무조건 다시 합치지 않음. intra-process가 자동으로 모든 복사를 없앤다고 가정하지 않음. |
| **O21 / P3** 사용 시 vehicle_status | `amr_vehicle_status/node.py:42–60,313`의 TF·다수 typed 구독·기본 실행기. bringup 기본값은 이미 false. | 다시 필요한 경우 executor/중복 상태 변환부터 측정하고 공용 상태 경로 재사용. C++ 이식은 마지막 후보. | 현재 꺼진 노드를 개선해도 기본 운용 CPU는 줄지 않음. 미연동/신선도 표시 의미 보존. |
| **O22 / P3** LiDAR 보상 버퍼 재사용 | `sllidar_node.cpp:374–400`에서 scan마다 각도 보상 배열 new/memset/delete. | 보상 배열 크기가 바뀔 때만 재할당하고 재사용. | scan 데이터·stamp·range 경계 동등성. 이미 Release이고 후처리도 벡터화됐으므로 추가 효과는 측정 필요. 원본 빔 수 감소는 별도 고위험 실험으로 분리. |

## 5. 운영·시뮬레이션·학습

| ID / 우선순위 | 근거 | 개선 방향 | 판정/적용 범위 |
|---|---|---|---|
| **O23 / P2** 기록 데이터와 I/O | `scripts/record_trial.sh:topics`에서 scan/raw scan·깊이·JPEG·costmap 전체/증분 등을 함께 기록하고 sqlite3 사용. | 운용/센서 진단/주행 회귀의 명시적 기록 프로파일을 분리. 토픽별 bytes/s·누락·write 지연을 확인한 뒤 저장 형식/분할/오프라인 압축을 비교. | 현재 전체 진단 모드 유지. 필요한 근거 토픽을 무조건 제외하지 않음. 온라인 압축은 Jetson CPU 증가도 비교. 디스크 사용률은 이번 읽기에서 7%여서 용량 확보보다 순간 I/O·주행 지연이 우선. |
| **O24 / P3** 정적 웹 자산 캐시 | `amr_web_ui/server.py:947–1008`, `Control_Server/app/main.py` 정적 파일 경로: 파일 재읽기, 모두 no-store. | JS/CSS와 버전 지정 정적 지도에 ETag/재검증 또는 build hash 캐시. 중앙의 로봇별 JS 변환도 버전별 캐시. | state·health·명령·실시간 카메라에는 같은 정책을 적용하지 않음. 주기 CPU보다는 재접속/새 탭 로딩 개선. |
| **O25 / P2** RL 필드 조회가 필요한 점보다 큰 복사를 수행 | `rl/amr_rl/planner.py:147–166`: `fields.view(...)[env_index]`로 선택된 환경의 S×S 전체 field를 만든 뒤 소수 점의 4개 셀을 gather. trace에서는 반복 호출. | 원본 field에서 `(env,row,col)`의 네 꼭짓점만 직접 조회. offset tensor도 device/dtype별 캐시. | 학습 PC에서 `rl/tests/test_planner.py`의 보간·경계·gradient 결과 비교, env 수별 GPU 메모리/시간 측정. 이득은 S²와 질의 점 수에 따라 달라짐. Jetson 주행 개선과 구분. |
| **O26 / P2** RL wavefront의 불변 계산과 반복 kernel | `planner.py:118–134`: 최대 2000회×16방향 루프에서 cost 기반 step을 매번 계산, 16회마다 torch.equal로 종료 판정. `nav_arena_env.py:210–211`은 로그마다 두 item 호출. | step/higher mask 등 불변 계산을 밖으로 옮기는 대안과 메모리 증가 비교. kernel 결합/컴파일 가능성 측정. 로그 scalar 수집은 묶어서 처리. | field 도달성·거리·동일 seed·학습 성능 유지. LayoutPool이 이미 field를 재사용하므로 매 physics step마다 wavefront를 돈다고 해석하지 않음. |
| **O27 / P2** 시뮬레이터 프로파일 자체의 장시간 비용 | `sim/scripts/runtime_profile.py:36–65`: 모든 표본·spike를 계속 저장하며 5초마다 전체 정렬과 전체 JSON 재작성. `run_ros_sim.py:335`에서 주 루프가 호출. | 누적 count/sum/max와 bounded 표본/분위수 추정, spike 스트리밍 또는 종료 시 정렬을 사용. 로그 I/O를 시간 측정 경로에서 분리. | `AMR_SIM_PROFILE` 활성화 시에만 해당. 장시간 RAM과 5초 주기 지연 비교, 보고된 p95/p99의 정확도/근사 여부 표시. |
| **O28 / P3** 펌웨어 CAN 완료 폴링 | `firmware/stm32/main.c:1299–1326`: TX 완료까지 1 tick sleep 반복, 임계 timeout 100 ms. | MCU RTOS trace에서 해당 경로가 실제 지연 원인일 때 TX callback/event 기반 대기 검토. | 현재도 sleep과 E-stop 검사로 양보하므로 busy-wait 폭주로 단정하지 않음. abort·정지 우선순위·watchdog·bus-off 회복 벤치 검증 필수. 저장소 파일과 실제 STM32 배포 펌웨어 동일성은 확인하지 못함. |

## 6. 이미 적용되어 새 최적화로 세지 않은 것

- 주요 Python 노드 EventsExecutor, CAN telemetry blocking RX thread, numpy scan/품질 계산, 고정 TF·각도 캐시.
- 웹 snapshot 얕은 복사, scan JSON 요청 시 캐시, costmap PNG 지연 생성/P 모드/압축 1.
- 깊이 320×200 10 Hz와 decimation 2, 컬러 640×360 5 fps, 쓰지 않는 센서 스트림 비활성화.
- local/global costmap 표시 주파수 2/1 Hz, 증분 발행. 내부 갱신 10/2 Hz와 구분됨.
- DWB 후보 800→288, short circuit, evaluation/trajectory/cost-grid debug 발행 비활성화.
- AMCL 초기 1080빔→tracking 60빔, joint_states 10 Hz, Nav2 feedback 중계 제한.
- CAN tx_preview와 motor telemetry의 구독자 없는 경우 작업 생략.
- 브라우저가 숨겨지면 polling 중단, 필요 시 requestAnimationFrame, kiosk 경량 상태 응답.
- 원격 agent의 원본 state JSON 전달, 레이어 버전 캐시, reconnect backoff와 명령 중복 방지.
- C++ Release 설정. 실제 `build/*/CMakeFiles/*/flags.make`도 확인했으며 TIM은 CMakeCache의 build type 문자열이 비어 있어도 **실제 tim_core flags는 `-O3 -DNDEBUG`**였다. 다시 O3를 켜면 빨라진다는 제안은 부적절하다.
- STVL 전환은 10/27에 이미 적용됐다. 10/25 controller 수치로 현재 STVL 비용을 확정할 수 없다.
- TIM `nice -n 10`, 기본 vehicle_status 비활성화. 우선순위 조정은 총 연산량 감소와 구분한다.

GPU 이식, 전면 C++ 재작성, watchdog/health 주기 축소, `-ffast-math`, 무제한 빌드 병렬화, Nav2 안전 여유 축소를 일괄 처방으로 제안하지 않는다. 위 후보를 단계별로 검증한 후에도 남는 병목을 대상으로 결정한다.

## 7. 실행 순서와 검증 기준

1. **O01·O02**: 측정 분류와 빌드 경로부터 바로잡는다. 별도 PID/노드 통계, 코드·설정 hash, 카메라/웹 클라이언트 수, 차량 단계, 온도·클록을 기록한다.
2. **O05·O06·O07**: 화면/관제 중복 작업을 먼저 줄인다. 계산량·잠금 시간을 줄이면서 명령과 stale 판정을 보존할 수 있는 범위다.
3. **O03·O04·O08**: CAN/제어 실행 비용을 줄인다. 가짜 버스, 격리 ROS, 시뮬레이션, 별도 현장 검증 순서로 진행한다.
4. **O09–O16**: payload·센서·위치 추정. 원격 다중 클라이언트와 엘리베이터 대기/탑승/하차를 나눠 측정한다.
5. **O17–O22**: 위 작업 이후에도 controller/BT/위치 지연이 남으면 주행 품질 A/B를 진행한다.
6. **O23–O28**: 운용 기록·학습·장시간 시뮬레이션 등 해당 환경에서 별도로 검증한다.

평균 CPU만으로 채택하지 않는다. 노드 CPU/PSS, callback 및 `/cmd_vel` 지연 p95/p99, CAN 송신 간격, 20 Hz controller 주기 준수율, TF age, HTTP p95/p99, payload bytes/s, 카메라 end-to-end age, 최저 장애물 검출·정지거리, 취소/LB/E-stop 반응을 함께 비교한다. 변경은 하나씩 동일 입력으로 A/B하고 결과를 합산해 예상 절감률을 만들지 않는다.

관련 시험은 [VALIDATION.md](VALIDATION.md)의 단위·HTTP 명령 및 지정 도메인을 따른다. 제어 173, CAN 174/184, 내비게이션 190/191/192, 웹·엘리베이터 193/195, startup 194, TIM 196, depth 197이며 `ROS_LOCALHOST_ONLY=1`을 붙인다. 실차 7·시뮬레이션 운용 107을 격리 시험에 사용하지 않는다. 일부 과거 replay 스크립트의 151 같은 임의 도메인을 그대로 복사하지 않는다.

빌드는 차량 정지 확인 후 다음 제한을 유지한다. 아직 이번 조사에서 실행한 명령이 아니다.

```bash
source /opt/ros/jazzy/setup.bash
CMAKE_BUILD_PARALLEL_LEVEL=2 MAKEFLAGS=-j2 \
  colcon build --base-paths src --symlink-install --executor sequential \
  --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3 -DCMAKE_BUILD_TYPE=Release
```

## 8. 이번에 수행한 확인과 한계

- `git status --short`, `git diff --stat`, `git rev-parse --short HEAD`: 기존 미커밋 변경과 기준 커밋 확인.
- `rg`, `nl`, `sed`, `cat`: 위에 명시한 코드·설정·검증 지침·실험 기록 및 실제 빌드 flags 확인.
- Python 표준 라이브러리 `csv`, `collections`, `statistics`: 두 CSV의 PID별 평균/p95 재집계. ROS·CAN 모듈은 import하지 않음.
- `ps -eo pid,comm,pcpu,pmem`, `free -h`, `df -h .`: 실행 환경에서 읽기만 수행. PID 가시성 제한으로 현재 실차 프로파일을 얻지 못함. 순간 메모리 수치로 누수 여부를 판정하지 않음.
- 단위/ROS/브라우저/시뮬레이션 시험과 새 성능 benchmark는 **미실행**. 과거 PASS를 이번 PASS로 기재하지 않음.

PID별 표를 재현하는 읽기 전용 계산은 다음과 같다. 각 파일에서 주요 작업 PID와 실행기를 따로 확인해야 하며, 긴 수명의 PID라는 조건만으로 작업 프로세스를 자동 확정하지 않는다.

```python
import collections, csv, pathlib, statistics

for path in sorted(pathlib.Path('reports/profile').glob('*.csv')):
    samples = collections.defaultdict(list)
    with path.open() as handle:
        for row in csv.DictReader(handle):
            if float(row['time']) >= 5:
                samples[row['label'], row['pid']].append(float(row['cpu_percent']))
    print(path.name)
    for (label, pid), values in sorted(samples.items()):
        if len(values) <= 100:
            continue
        values.sort()
        print(label, pid, len(values), round(statistics.mean(values), 2),
              values[min(len(values) - 1, int(.95 * len(values)))])
```
