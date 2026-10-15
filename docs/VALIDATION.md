# AMR 검증 절차와 결과

기준일: **2026-10-07 · Asia/Seoul**. 실행법은 [최신 운영 문서](README.md)에서 관리한다.
이 문서는 판정 기준·재현 명령·이번 확인 결과를 관리하며, 과거 결과를 이번 검증 결과로 합산하지 않는다.

## 1. 검증 범위

| 단계 | 확인 내용 | 실물 장치 사용 |
|---|---|---|
| 문서·설정 | 링크·명령 문법·런치 옵션·구성 값·이력 보존 | 없음 |
| 단위·HTTP | 좌표·요청 검증·모델·상태 신선도·원격 규약 | 없음 |
| 격리 ROS | 생성 메시지·공용 상태·웹 Action·재시작·만료 | 합성 ROS 입력, 실제 CAN 송신 없음 |
| 브라우저 | 지도 조작·모바일·상태/전원/BMS 표시·원격 UI | 저장 지도와 테스트 fixture |
| 장치 읽기 | OS·CPU·CAN 설정·Tailscale 주소·파일 존재 | 상태 읽기만 수행 |
| 현장 주행 | 실제 조이스틱·위치 확인·목표 주행·정지·통신 단절 | 별도 현장 검증 |

단위 시험의 전압·BMS 수치와 가짜 Nav2 성공은 실측 배터리 상태나 실제 주행 성공을 뜻하지 않는다.

## 2. 문서·설정 독립 검증

작업공간 루트에서 실행한다.

```bash
python3 scripts/verify_documentation.py --output diagnostics/operating_readme_20261007
```

이 검사는 운영 명령을 실행하지 않는다. 다음 항목을 독립적으로 대조한다.

- 활성 문서의 로컬 링크·절 제목, bash 코드 블록의 문법
- 문서에 적힌 `ros2 launch` 파일·옵션과 실제 `DeclareLaunchArgument`
- 실운용 실행 명령과 점검·개발 명령의 분리, 아래 설정 기준값과 실제 소스
- 상태 노드가 bringup에 포함되는지, 실제 인터페이스 파일의 존재
- 문서 이력 manifest의 보존본 체크섬과 원문 백업 체크섬
- 활성 안내가 옛 문서를 현재 실행 기준으로 참조하지 않는지

결과는 지정 폴더의 `documentation.json`에 남긴다.
문서와 설정을 바꾸면 이 검사를 다시 실행한다. 성공해도 현장 주행 판정은 별도로 수행한다.

### 설정 대조 기준

실행 안내에는 조작과 실행 명령만 두고, 소스 대조에 사용하는 기준값은 이곳에서 관리한다.

| 항목 | 값 |
|---|---|
| 실물 ROS 도메인 | `7` |
| 시뮬레이션 ROS 도메인 | `107` |
| 로컬 웹 포트 | `8080` |
| 중앙 웹 포트 | `8081` |
| CAN 비트레이트 | `500000 bit/s` |
| Xbox 장치 이름 | `Xbox Series X Controller` |
| 바퀴 지름 | `0.172 m` |
| 바퀴 간격 | `0.499 m` |
| 상위 바퀴 회전 속도 제한 | `1.8 rev/s` |
| AMCL 초기 빔 수 | `1080` |
| AMCL 주행 빔 수 | `60` |
| LiDAR 최대 검증 거리 | `30.0 m` |
| 공용 차량 상태 발행 주기 | `10 Hz` |
| 공용 상태 수신 만료 | `0.5 s` |

## 3. 소프트웨어 재현 명령

### 3.1 준비

```bash
cd /home/acus03/amr_ws
source scripts/env.sh
```

소스 변경으로 빌드가 필요하면 아래 명령을 먼저 사용한다.

```bash
source /opt/ros/jazzy/setup.bash
CMAKE_BUILD_PARALLEL_LEVEL=2 MAKEFLAGS=-j2 \
  colcon build --base-paths src --symlink-install --executor sequential \
  --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3
source scripts/env.sh
```

시험 로그와 JUnit 결과는 새 검증 폴더에 보관한다.

### 3.2 단위·HTTP

```bash
python3 -m pytest -q \
  tests \
  src/amr_system/amr_vehicle_status/test/test_model.py \
  src/amr_system/amr_web_ui/test/test_display_telemetry.py \
  src/amr_system/amr_web_ui/test/test_web_ui.py \
  src/amr_system/amr_web_ui/test/test_grid_transforms.py \
  src/amr_system/amr_remote_bridge/test/test_remote_bridge.py \
  --junitxml=diagnostics/docs_unification_20261007/unit_http.xml
```

### 3.3 공용 상태 ROS — 도메인 195

```bash
ROS_DOMAIN_ID=195 ROS_LOCALHOST_ONLY=1 AMR_RUN_VEHICLE_STATUS_TESTS=1 \
  python3 -m pytest -q src/amr_system/amr_vehicle_status/test/test_node_ros.py \
  --junitxml=diagnostics/docs_unification_20261007/vehicle_status_ros.xml
```

빈 상태의 미연동 플래그, 생성 메시지, 잘못된 입력, source 만료, READY·시간 상태 만료,
시계 불연속, 누락된 진단 카운터와 실제 명령 publisher 부재를 확인한다.

### 3.4 웹·두 번째 공용 상태 소비자 ROS — 도메인 193

```bash
ROS_DOMAIN_ID=193 ROS_LOCALHOST_ONLY=1 AMR_RUN_WEB_UI_TESTS=1 \
  python3 -m pytest -q src/amr_system/amr_web_ui/test/test_web_ui_ros.py \
  --junitxml=diagnostics/docs_unification_20261007/web_ros.xml
```

실제 HTTP → 목표 관리기 → 가짜 Nav2, MANUAL 대기·AUTO 전달·취소·초기 위치·relocalize,
같은 공용 상태를 웹과 다른 ROS 소비자가 구독하는 경로, 만료·재정렬·이전 세션 거부를 확인한다.
실차 센서 드라이버·CAN 송신·STM32 설정은 시작하거나 변경하지 않는다.

### 3.5 브라우저와 JavaScript

```bash
python3 src/amr_system/amr_web_ui/test/browser_check.py
python3 src/amr_system/amr_remote_bridge/test/browser_check.py
gjs src/amr_system/amr_web_ui/test/test_geometry.js src/amr_system/amr_web_ui/web/geometry.js
gjs src/amr_system/amr_web_ui/test/test_tracking_status.js src/amr_system/amr_web_ui/web/app.js
```

Chromium의 실제 터치·드래그·pan·pinch·취소와 1440/1366/900/768/390px 레이아웃을 확인한다.
전원 전압 추정·BMS 우선 표시·입력 없음·연결 끊김은 테스트 fixture로 확인한다.
기본 결과는 [로컬 브라우저 결과](../diagnostics/concept_a_20261007/browser/browser_result.json)와
[원격 브라우저 결과](../diagnostics/amr_remote_bridge/browser_result.json)에 남는다.

### 3.6 호스트 FastAPI

호스트 `Control_Server`의 해당 CPU/파이썬으로 만든 가상환경에서 실행한다.

```bash
cd /home/kim/Control_Server
.venv/bin/python -m pytest -q \
  --junitxml=diagnostics/verification_host.xml
```

Jetson에 복사된 호스트의 x86_64 / Python 3.10 `.venv`를 현지 환경으로 간주하지 않는다.
이번 Jetson의 호스트 호환성 검사는 독립된 임시 aarch64 / Python 3.12 환경 `/tmp/amr-concept-a-host-check`에서 실행했다.
기존 호스트 배포 `.venv`는 변경하지 않았다.
실제 호스트 배포·프로세스 재시작·Tailscale 왕복은 별도 판정이다.

### 3.7 전체 ROS 회귀 — 확장 검증

```bash
python3 scripts/test_ros_integration.py
```

이 실행기는 수동 제어·CAN 브리지·텔레메트리·EKF·AMCL·목표 관리·Nav2·웹·위치 시작·TIM·공용 차량 상태를
각각 격리 도메인에서 순서대로 검증한다. 실물 도메인 7과 시뮬레이션 도메인 107을 사용하지 않는다.
현재 스코프와 실행 결과는 [실행기](../scripts/test_ros_integration.py)와
[통합 결과](../diagnostics/porting/integration/results.json)를 확인한다.
이번에 실행하지 않은 확장 시험은 이전 PASS를 현재 PASS로 기재하지 않는다.

Isaac 전체 시뮬레이션·성능·실물 보정은 해당 runtime과 시험 환경에서 수행한다.
이 Jetson에는 Isaac runtime이 없으므로 이번 검증에 포함하지 않는다.

### 3.8 층간 이동(엘리베이터) — 격리 ROS와 Isaac 시뮬레이션

설계와 판정 규칙은 [층간 이동 설계](ELEVATOR_DESIGN.md)에 있다. 격리 ROS 시험은 실제 CAN을 열지 않는다.

```bash
ROS_DOMAIN_ID=193 ROS_LOCALHOST_ONLY=1 AMR_RUN_ELEVATOR_TESTS=1 \
  python3 -m pytest -q src/amr_system/amr_sim_bridge/test/test_elevator_ros.py
ROS_DOMAIN_ID=193 ROS_LOCALHOST_ONLY=1 AMR_RUN_ELEVATOR_NAV_TESTS=1 \
  python3 -m pytest -q src/amr_system/amr_elevator/test/test_elevator_nav_ros.py
ROS_DOMAIN_ID=193 ROS_LOCALHOST_ONLY=1 AMR_RUN_ELEVATOR_MISSION_TESTS=1 \
  python3 -m pytest -q src/amr_system/amr_elevator/test/test_mission_ros.py
ROS_DOMAIN_ID=195 ROS_LOCALHOST_ONLY=1 AMR_RUN_ELEVATOR_MONITOR_TESTS=1 \
  python3 -m pytest -q src/amr_system/amr_elevator/test/test_monitor_ros.py
```

Isaac Sim이 있는 PC에서 시나리오 전체(정상 3종 + 신호 누락·지연·중복·다른 층 정차·장치 고장)를 실행한다.
`sim/config/scenarios.json`의 `floor_scenarios`가 기대 결과(`success` 또는 `safe_stop`)를 정한다.

```bash
export ROS_DOMAIN_ID=107
/usr/bin/python3 scripts/run_floor_suite.py --output logs/elevator/suite_$(date +%Y%m%d_%H%M%S)
```

결과는 출력 폴더의 `summary.md`·`summary.json`과 시나리오별 `result.json`·`metrics.json`이다.
3F·5F는 4층 지도 복사본이며 시뮬레이션 수치는 실물 보정 전 값이다.

## 4. 장치 읽기와 현장 판정

읽기 명령은 다음과 같다.

```bash
cat /etc/os-release
uname -m
ip -details link show can0
tailscale ip -4
```

스택 실행 후 수신 상태를 확인한다. 응답이 없으면 조회를 종료하고 실제 스택·도메인을 확인한다.

```bash
source scripts/env.sh
export ROS_DOMAIN_ID=7
ros2 param get /cmd_vel_to_can dry_run
ros2 topic echo /vehicle/status --once
ros2 topic echo /system/time_health --once --qos-durability transient_local
ros2 topic echo /localization/status --once --qos-durability transient_local
```

| 현장 항목 | 통과 기준 |
|---|---|
| CAN·센서 | CAN UP/500,000 bit/s, 원본 scan·wheel·IMU·EKF 지속 수신·유효 시각 |
| 조이스틱 | 실제 이름 일치, LB·RB·축 매핑과 해제·단절 처리 확인 |
| 위치 | 실제 지도·위치·방향·LiDAR 정합, 감독기 `READY`, 시간 `healthy=true` / `latched=false` |
| 공용 상태 | 10 Hz, 유효 source와 신선도, 미연동 항목이 정상값으로 표시되지 않음 |
| 실제 송신 | 점검 dry-run=True와 실제 운용=False를 각 실행에서 구분 |
| 주행·정지 | 현장 준비 후 목표 접수·출발·도착, MANUAL 전환·목표 취소·실제 정지 확인 |
| 원격 | 실제 호스트 최신 배포, 인증·상태·목표·취소 왕복, 단절 뒤 새 명령 차단·재전송 없음 |
| BMS·드라이버 | 실제 연결 후 값·단위·유효 플래그·입력 중단 판정 확인 |
| 펌웨어·구동 설정 | 장치 readback으로 STM32/ODrive 버전과 실제 제한값 확인 |

이번 문서 정리에서는 실차 주행·CAN 모터 명령 송신·펌웨어 변경·실제 호스트 배포를 수행하지 않는다.
해당 항목은 현장 검증을 마친 뒤 실제 결과와 기록 경로를 갱신한다.

## 5. 이번 검증 결과

소프트웨어 결과는 최초 문서 통합 시 실행한 기록이다. 소프트웨어 시험 192개가 통과했고 opt-in ROS 모듈 1개는 제외됐다. 실행법을 실운용 기준으로 줄인 뒤에는 문서·명령 검사만 다시 수행했고, 중앙 서버 최신본 재적용에서는 해당 서버의 HTTP·브라우저 시험을 별도로 다시 수행했다. `PASS`는 해당 범위만 통과했다는 뜻이다.
세부 로그·JUnit·체크섬은 [이번 검증 폴더](../diagnostics/docs_unification_20261007/)에 보관한다.

| 항목 | 판정 | 근거 |
|---|---|---|
| 실운용 실행법·명령·링크·설정·원문 보존 | PASS | [최신 문서 결과](../diagnostics/operating_readme_20261007/documentation.json) |
| 중앙 대시보드 수치 깜빡임 수정 | PASS · 전달 지연 재현/원본 만료/브라우저 | [수정 결과](../diagnostics/dashboard_flicker_20261007/REPORT.md) · [검증](../diagnostics/dashboard_flicker_20261007/verification.json) |
| CAN 전압·Heartbeat 실제 연동 | PASS · 실측 CAN 수신/격리 ROS/브라우저 | [수정 결과](../diagnostics/power_fix_20261007/REPORT.md) · [실측값](../diagnostics/power_fix_20261007/live_power.json) |
| 새 재현 ROSBAG 기반 수정 | PASS · 위치/Action/HTTP/브라우저 | [수정 결과](../diagnostics/repro_fix_20261007/REPORT.md) · [최종 검증](../diagnostics/repro_fix_20261007/verification.json) |
| 최초 문서 통합 당시 검사 | PASS · 188개 항목 | [이전 문서 결과](../diagnostics/docs_unification_20261007/documentation.json) |
| 중앙 서버 최신본 재적용 | PASS · vendor 59개 일치, HTTP 54개 | [재적용 기록](../diagnostics/control_server_refresh_20261007_161342/verification.json) |
| 재적용 후 FastAPI 브라우저 | PASS · 메인·차량 상태·5개 화면 크기·단절/재접속 | [주행 화면](../diagnostics/control_server_refresh_20261007_161342/browser/browser_result.json) · [차량 상태](../diagnostics/control_server_refresh_20261007_161342/browser/health_result.json) |
| 단위·HTTP | PASS · 129개, opt-in ROS 모듈 1개 skip | [JUnit](../diagnostics/docs_unification_20261007/unit_http.xml) · [로그](../diagnostics/docs_unification_20261007/unit_http.log) |
| 공용 상태 ROS / 웹 ROS | PASS · 6개 / 3개 | [공용 상태](../diagnostics/docs_unification_20261007/vehicle_status_ros.xml) · [웹](../diagnostics/docs_unification_20261007/web_ros.xml) |
| 로컬·원격 브라우저 | PASS · console error 0 | [로컬](../diagnostics/docs_unification_20261007/browser_local.json) · [원격](../diagnostics/docs_unification_20261007/browser_remote.json) |
| 호스트 FastAPI 호환 | PASS · 54개 | [JUnit](../diagnostics/docs_unification_20261007/host.xml) · [로그](../diagnostics/docs_unification_20261007/host.log) |
| CAN 설정·AMR Tailscale 주소 | 읽기 확인 | [CAN](../diagnostics/docs_unification_20261007/can0.txt) · [IP](../diagnostics/docs_unification_20261007/amr_ip.txt) |
| 실물 도메인 7의 ROS 노드 | 실행 중 스택 없음 | [조회](../diagnostics/docs_unification_20261007/live_nodes.txt) |
| 현재 Isaac runtime | 없음 | 이 Jetson 작업공간에 `runtime/isaac-sim-6.1.0` 없음 |
| 실제 호스트 최신 배포·왕복 | 미확인 | 로컬 배포 소스 검증과 구분 |
| 실제 조이스틱·주행·정지·BMS·펌웨어 readback | 이번 미실행 | 현장 검증으로 확인 |
| 전체 ROS 확장 회귀·Isaac 전체 시험 | 이번 미실행 | 위 재현 명령으로 별도 수행 |

이전 작업의 공용 상태·UI 검증은 [2026-10-07 기록](../diagnostics/vehicle_status_20261007/verification.json),
과거 포팅·실물 센서 점검은 [2026-10-06 보고서](../diagnostics/pc_update_20261006/REPORT.md)로 보존한다.
그 결과를 새 실행·배포·실주행이 완료됐다는 주장으로 바꾸지 않는다.

## 6. 갱신 규칙

1. 코드·설정·실행법 변경은 최신 운영 문서 본문에서 수정한다.
2. 새 시험은 이 문서의 결과 표를 갱신하고 별도 날짜/세션의 원본 결과를 보관한다.
3. PASS 결과에는 실행 환경·코드 체크섬·명령·로그 또는 JUnit 경로를 남긴다.
4. 소프트웨어 fixture, 시뮬레이션, 실제 센서 수신, 실차 주행, 실제 호스트 배포의 판정을 구분한다.
5. 이전 안내를 다시 활성 문서로 끌어오지 않고 필요한 근거만 이력에서 연결한다.
