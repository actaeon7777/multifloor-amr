> **이력 문서** · 2026-10-07 정리 시점의 이전 안내입니다. 현재 실행은 [최신 운영 문서](../../../../../README.md), 검증은 [검증 문서](../../../../../VALIDATION.md)를 사용합니다.

# AMR Web UI — Clarity 운영 화면

2026-10-07: `ui_concepts_20261006/a.html`의 최신 컨셉 A를 운영 웹에 적용했습니다.
좌측 메인 / 차량 상태 메뉴, 중앙 지도, 우측 진행도·진행 거리·남은 거리·속도·목적지,
좌측 하단 서버 주소와 배터리 표시를 제공합니다. `/#vehicle-status`로 차량 상태를 바로 열 수 있습니다.
모바일에서는 상단 메뉴 버튼으로 화면을 전환합니다.

2026-10-07 공용 차량 상태로 전환: 차량 상태 화면은
`amr_vehicle_interfaces/msg/VehicleStatus` 타입의 `/vehicle/status`를 구독합니다.
상태 발행 노드는 `amr_stack/bringup.launch.py`가 시작하며 웹을 종료해도 계속 동작합니다.
분리 실행과 다른 ROS 소비자의 사용법은 [공용 차량 상태](../amr_vehicle_status/README.md)를 참고하세요.
지도 렌더링·주행 Action·목표 허용 검증은 기존 경로를 사용합니다.

진행 거리는 웹 목표 실행 동안 공용 `wheel_distance_m`의 차이로 계산합니다.
진행도는 `진행 거리 / (진행 거리 + Nav2 남은 거리)`의 추정값이며 도착 완료 시 100%입니다.
바퀴 수신 공백·odom 불연속으로 `wheel_distance_epoch`가 바뀌면 전체 거리의 연속성을 확인할 수 없어
진행 거리·진행도를 표시하지 않습니다. 새 목표 또는 발행 노드 재시작 시 이전 값을 정리합니다.
상태 토픽이 0.5초 이상 끊기면 차량 표시 데이터를 만료시킵니다.

차량 상태 화면의 좌우 rpm·센서 빈도·경과 시간·CAN 오류 카운터·최종 속도 명령·
지도상 위치·제어 모드·위치 검증·시간 감시 정보는 모두 공용 상태 메시지에서 가져옵니다.
이 구독은 화면 표시용이며 기존 주행 허용·조이스틱·CAN 정책은 유지합니다.

드라이버 상태·오류 코드·온도·Heartbeat·입력 전압·버스 전류와 BMS는 현재 ROS 출력이 없어
미연동으로 표시합니다. 배터리 잔량은 전압 데이터가 없으면 `—%`입니다.
컨셉 A의 운용 표시 기준은 25.6 V=0%, 27.4 V=100%의 선형 전압 추정이며 BMS SOC와 구분합니다.
시안의 예시 전압·잔량·목적지 이름을 운영 화면에 넣지 않습니다.

Jetson 또는 ROS 2 개발 PC에서 실행하는 로컬 웹 서버입니다. 브라우저의 터치 좌표를 `map` 좌표로 변환하고,
기존 `navigation_manager`의 공개 `/navigate_to_pose` Action에 목표를 전송합니다.
RViz의 `/goal_pose` 입력과 같은 목표 관리 경로를 사용하며 접수·결과·현재 목표 취소를 확인합니다.
기존 mux·조이스틱·CAN 정책을 변경하지 않습니다.

## 실행

```bash
amr
ros2 launch amr_web_ui web_ui.launch.py
```

먼저 workspace에서 `colcon build --symlink-install --packages-select amr_web_ui`로 설치합니다.
stack·localization·navigation은 기존 실행법대로 별도로 실행합니다.
접속 주소는 Jetson에서 `http://localhost:8080`, 같은 LAN에서는 `http://<웹 서버 IP>:8080`입니다.
`host:=127.0.0.1 port:=8081`처럼 바인딩과 포트를 변경할 수 있습니다.

Isaac Sim의 `./scripts/experiment.sh sim`에 연결할 때는 별도 터미널에서 다음과 같이 실행합니다.
웹 서버도 도메인 107과 시뮬레이션 시간을 사용해야 로봇 위치와 스캔을 표시할 수 있습니다.

```bash
amr
ROS_DOMAIN_ID=107 ros2 launch amr_web_ui web_ui.launch.py use_sim_time:=true
```

이 PC의 접속 주소는 `http://127.0.0.1:8080`입니다.

Isaac/제어 스택을 재시작하면 Web UI가 시간 역행과 실행 ID 변경을 감지해 이전
TF·동적 표시·대기 요청을 정리하고 자동으로 새 데이터를 받습니다. 이전 목표는
자동 재전송하지 않습니다. Web UI를 계속 켜둔 채 제어 스택만 재시작할 수 있습니다.
표준 런치는 `/system/time_health`가 정상이어야 새 목표를 허용합니다. 이 감시 노드는
`bringup.launch.py`에 포함됩니다. 실물 시계 이상 후 복구와 다중 컴퓨터 시간 동기화
절차는 [SIM2REAL](../../../docs/SIM2REAL.md#제어기-재시작과-시간-이상-처리-2026-10-04)을 참고하세요.

런타임 의존성은 ROS 2 Jazzy, Pillow, PyYAML이며 Ubuntu 패키지 `python3-pil`, `python3-yaml`을 사용합니다.
프런트엔드는 로컬 HTML/CSS/JavaScript이며 npm·CDN·rosbridge가 필요 없습니다.
데스크톱과 터치 브라우저를 지원하며 실제 장착 디스플레이의 터치와 배율은 현장에서 확인합니다.

## 조작

- 목표 지정 모드의 탭/클릭은 즉시 전송합니다. 드래그는 누른 지점의 목표 방향을 지정합니다.
- 단순 탭은 로봇에서 목표를 향하는 방향을 기본 사용합니다. 도착 방향 입력을 편집하면 그 값을 사용합니다.
- 초기 위치 모드에서는 `/initialpose`를 전송합니다. covariance는 현재 `navigation.rviz`의 값을 사용합니다.
  전송 직후 목표 입력을 막고 감독기의 새 위치 검증을 기다립니다. 지도·스캔 정합도 확인합니다.
- `자동 위치 찾기`는 `/localization/relocalize`를 한 번 호출합니다. 자동 회전·이동 없이 전역 탐색을 재시작합니다.
  기본 초기화기 TIM이 최대 1,080빔·30m 스캔으로 위치 후보를 찾고, 상위 후보를 최대 1,080빔으로
  정밀 보정한 뒤 검증한 후보를 AMCL에 전달합니다. 정지 중 작은 odometry 변화는 보상하며,
  이동 명령·허용 범위를 넘은 움직임·목표 실행·센서 수신 문제는 원인을 구분해 표시합니다.
  위치가 모호하면 `위치 확인 필요`를 표시하며, 정지 상태에서 상단 `초기 위치`로 직접 지정할 수 있습니다.
- 자동·수동 위치 재설정은 주행·일시정지·출발 대기 목표를 취소하고 정지한 뒤 가능합니다.
  2초 이내의 속도 데이터에서 선속도 0.02m/s 이하, 각속도 0.03rad/s 이하를 확인합니다.
- 위치 상태는 `센서 수신 대기 → 자동 위치 찾는 중 → 위치 확인 중 → 주행 준비 완료`로 표시합니다.
  감독기 상태가 없거나 2초 이상 지연되면 목표를 보내지 않습니다.
  초기 탐색·위치 검증에는 AMCL 1,080빔을 사용하고, 주행용 60빔 전환과 새 관측 검증까지
  마쳐야 준비 완료가 됩니다. 별도 지도·스캔 검증도 최대 360빔을 사용합니다.
  AMCL과 별도 검증의 최대 거리는 모두 30m로 유지합니다.
- 지도 이동 모드, 휠, 두 손가락, 확대 버튼, 전체 보기, 로봇 따라가기를 제공합니다.
- 키보드: 지도에 포커스를 두고 방향키로 0.25m씩 목표를 조정하고 Enter로 전송합니다.
- 취소는 웹 서버가 보유한 현재 GoalHandle만 대상으로 합니다. 다른 클라이언트의 목표를 일괄 취소하지 않습니다.
- 현재 MANUAL 시험 모드에서는 접수된 목표를 보관하고 `출발 대기`를 표시합니다. RB AUTO에서 주행합니다.

## 시각화

`navigation.rviz`의 기본 2D 표시를 참고합니다. 지도는 전체 격자 PNG로 캐싱하고 동적 오버레이는 별도 Canvas로 그립니다.
2D 화면에는 CAD 모델과 TF 트리 탐색, AMCL particles가 포함되지 않습니다.

| 레이어 | ROS 데이터 |
|---|---|
| 지도 | `/map`, `nav_msgs/msg/OccupancyGrid` |
| 로봇 위치·방향 | TF `map → base_footprint` |
| AMCL 화살표·위치 공분산 타원 | `/amcl_pose` |
| LiDAR 점 | `/scan`, 측정 시각 기준 TF 변환, 유효한 전체 ray 사용 |
| 전역·지역 경로 | `/plan`, `/local_plan` |
| 지역·전역 costmap | `/{local,global}_costmap/costmap` + `costmap_updates` |
| 충돌 검사 외곽선 | `/local_costmap/published_footprint` |
| 주행 상태·시험 모드 | `/navigation/status`, `/control_mux/mode` |
| 위치 탐색·주행 준비 상태 | `/localization/status`, transient-local JSON |
| 추정 속도 | `/odometry/filtered` |

지도의 origin 회전, 화면 확대·이동, Y축 반전을 처리합니다. 지도·AMCL·costmap은 transient-local로 구독합니다.
스캔은 SensorDataQoS를 사용하고 측정 시각에 TF가 없으면 제한된 시간 안에 재시도합니다.
지역 costmap의 origin과 증분 갱신을 반영하며 TF를 얻지 못하면 해당 격자를 표시하지 않습니다.
명령 위치는 지도 버전·지도 경계·점유값으로 검사하지만, 로봇 footprint와 경로의 실제 통과 가능 여부는 Nav2가 판정합니다.

## 통신과 상태

Python 표준 HTTP 서버를 사용합니다. `/api/state`를 약 5Hz로 조회하고 화면은 최대 15FPS로 렌더링합니다.
첫 구현에는 WebSocket이나 외부 웹 프레임워크를 추가하지 않았습니다. 지도 PNG는 버전이 바뀔 때만 받습니다.
HTTP 스레드는 명령을 큐에 넣고, ROS executor가 Action 호출과 publisher 처리를 소유합니다.

- `POST /api/goal`: `request_id`, `map_version`, `x`, `y`, `yaw`.
- `POST /api/initialpose`: 같은 위치 payload.
- `POST /api/relocalize`: `request_id`, 지도가 수신된 경우 `map_version`. 위치 좌표는 필요 없습니다.
- `POST /api/cancel`: `request_id`.
- POST에는 `Content-Type: application/json`, `X-AMR-Request: 1`을 사용합니다.
- HTTP 202는 웹 서버의 요청 접수입니다. ROS 접수는 상태의 `goal.status=ACCEPTED`로 별도 확인합니다.
- 자동 위치 찾기의 `STARTED` 응답은 탐색 시작만 의미합니다. 주행 가능 여부는 별도 감독기 `READY`로 확인합니다.
- 목표는 프런트엔드·HTTP 접수·ROS 실행 직전에 위치 감독기 `READY`, 상태 신선도, TF·로봇 위치, Action 연결을 확인합니다.
  공개 Action 서버의 존재만으로 Nav2 준비 완료를 판단하지 않습니다.
- 초기 위치·자동 탐색 요청 후 감독기의 `session`/`generation`이 새 시도로 바뀌어야 이전 `READY` 차단이 해제됩니다.
  heartbeat `stamp`도 확인하므로 transient-local에 남은 오래된 `READY`는 주행을 허용하지 않습니다.
- 요청 ID 중복은 다시 실행하지 않습니다. 연결 복구나 HTTP 응답 지연에도 프런트엔드는 명령을 자동 재전송하지 않습니다.
- 연결이 끊기면 명령 버튼을 비활성화하고, 재연결 시 현재 상태만 조회합니다.
- 오래된 스캔은 표시를 중단하고 위치·속도의 데이터 지연을 구분합니다.
- 웹 프로세스 재시작 후 이전 GoalHandle은 복원하지 않습니다. 기존 ROS 목표가 있다면 ROS 쪽에서 관리합니다.
- 이 서버는 인증 없는 개발 LAN용입니다. 공개 인터넷으로 노출하는 배포 구성은 포함하지 않습니다.

## ROS 없는 미리보기

workspace 루트에서 실행합니다. 실제 지도만 불러오며 ROS는 초기화하지 않습니다.

```bash
PYTHONPATH=src/amr_system/amr_web_ui python3 -m amr_web_ui.server \
  --preview-map maps/G_4_floor/4_floor_0522.yaml --host 127.0.0.1
```

## 검증

```bash
amr
PYTHONPATH="$PWD/src/amr_system/amr_web_ui:$PWD/src/amr_system/amr_stack:$PYTHONPATH" \
  python3 -m pytest -q src/amr_system/amr_web_ui/test/test_web_ui.py \
    src/amr_system/amr_web_ui/test/test_grid_transforms.py \
    src/amr_system/amr_web_ui/test/test_display_telemetry.py

ROS_DOMAIN_ID=193 ROS_LOCALHOST_ONLY=1 AMR_RUN_WEB_UI_TESTS=1 \
  PYTHONPATH="$PWD/src/amr_system/amr_web_ui:$PWD/src/amr_system/amr_stack:$PYTHONPATH" \
  python3 -m pytest -q src/amr_system/amr_web_ui/test/test_web_ui_ros.py

gjs src/amr_system/amr_web_ui/test/test_geometry.js src/amr_system/amr_web_ui/web/geometry.js

python3 src/amr_system/amr_web_ui/test/browser_check.py
```

통합 테스트는 격리 도메인에서 HTTP 요청 → 실제 목표 관리기 → 가짜 Nav2 서버를 확인합니다.
실차·CAN·센서 드라이버는 시작하지 않습니다.
브라우저 검사에서는 지도 조작, 차량 상태 전환, 1440/1366/900/768/390px 화면,
최신·지연·연결 끊김 표시와 초기 차량 상태 진입 후 지도 맞춤을 확인합니다.
결과와 스크린샷은 `diagnostics/concept_a_20261007/browser`에 저장합니다.
