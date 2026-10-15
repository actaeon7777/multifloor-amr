> **이력 문서** · 2026-10-07 정리 시점의 이전 안내입니다. 현재 실행은 [최신 운영 문서](../../../README.md), 검증은 [검증 문서](../../../VALIDATION.md)를 사용합니다.

# 새 포팅본 AMR 실행 가이드

작성일: 2026-10-07 · 대상: `amr_real_20261006.tar.gz`를 반영한 Jetson AMR

현재 작업공간 **`/home/acus03/amr_ws`에서 실행**한다. Jetson 빌드는 완료돼 있어 일상 실행 때 다시 압축을 풀거나 빌드할 필요는 없다.
기존 CAN·라이다 설정, Tailscale, 인증 토큰과 `amr` 별칭을 유지한다.

**AMR에서 직접 사용할 때는 터미널 1·2만 필요하다. 다른 노트북에서 중앙 관제를 사용할 때는 중앙 서버와 터미널 3도 실행한다.**

## 1. 실행 위치와 접속 주소

| 실행 위치 | 실행할 항목 | 역할 |
|---|---|---|
| AMR Jetson — 터미널 1 | `experiment.sh real` | 센서·제어·위치추정·내비게이션 통합 실행 |
| AMR Jetson — 터미널 2 | `amr_web_ui` | 로컬 웹 화면과 API |
| AMR Jetson — 터미널 3 | `remote_client` | AMR과 중앙 서버 연결; 원격 관제 사용 시 필요 |
| 중앙 서버 PC | `Control_Server` | Tailscale을 통한 관제 웹 제공 |
| 운용 노트북 | 웹 브라우저 | 중앙 서버에 접속해 상태 확인·목표 지정 |

| 항목 | 현재 구성 값 |
|---|---|
| 중앙 서버 PC / 프로젝트 | `100.88.249.77` / `/home/kim/Control_Server` |
| 중앙 관제 웹 | <http://100.88.249.77:8081/robots/amr-01/> |
| AMR Tailscale IP | `100.106.98.122` |
| AMR 로컬 웹·API | `http://127.0.0.1:8080` |
| 로봇 ID | `amr-01` |
| ROS | Jazzy, `ROS_DOMAIN_ID=7`, `use_sim_time=false` |
| 기본 지도 | `src/amr_system/amr_stack/maps/G_4_floor/4_floor_0522.yaml` |
| 인증 파일 | 각 장치의 `~/.config/amr_remote_bridge/token` — 기존 공통 값 사용 |

`127.0.0.1`은 브라우저를 실행한 장치 자신을 뜻한다. 노트북에서는 중앙 관제 주소를 사용한다.
중앙 서버·AMR·운용 노트북은 같은 Tailscale 네트워크에 접속하고 해당 포트에 접근할 수 있어야 한다.

## 2. 실행 전 확인

1. 기존 주행 스택·웹 API·`remote_client`가 켜져 있다면 정상 종료한다. 같은 노드를 중복 실행하지 않는다.
2. CAN·라이다·조이스틱을 연결한다. 조이스틱의 LB/RB와 스틱은 놓은 상태로 시작한다.
3. 현재 장소가 기본 G동 4층 지도와 맞는지 확인한다. 다른 장소라면 해당 지도 YAML을 지정한다.
4. AMR에서 CAN 상태와 조이스틱 인식을 확인한다.

```bash
cd /home/acus03/amr_ws
source scripts/env.sh
export ROS_DOMAIN_ID=7
ip -details link show can0
ros2 run joy joy_enumerate_devices
```

CAN은 기존 `UP`, `bitrate 500000` 설정을 확인한다. 실행 스크립트가 CAN 비트레이트를 설정해 주지는 않는다.
기본 조이스틱 이름은 `Xbox Series X Controller`이다. 장치가 나오지 않으면 연결부터 확인한다.

### 포팅 당시 확인된 범위

2026-10-06 점검에서는 **CAN 송신 없는 실제 센서 수신, 지도 API, 시간 상태까지 확인**했다.
자동 위치 찾기는 `WAITING_INITIAL_POSE`에 머물렀고 조이스틱이 감지되지 않았다.
따라서 현장에서 조이스틱 연결과 초기 위치 검증을 마쳐야 하며, 실제 주행 성공까지 확인된 상태는 아니다.
상세 결과는 [포팅·검증 보고서](../../../../diagnostics/pc_update_20261006/REPORT.md)에 있다.

## 3. 중앙 서버 실행 — 원격 관제 사용 시

**이 절의 명령은 AMR이 아니라 중앙 서버 PC에서 실행한다.**

포팅 당시 중앙 서버 PC가 오프라인이어서 호스트 업데이트는 적용하지 못했다.
아직 적용하지 않았다면 AMR에 준비된 `control_server_update_20261006.tar.gz`와 `.sha256`을 이용해
[중앙 서버 업데이트 안내](../Control_Server/UPDATE_20261006.md)를 먼저 수행한다. 이 작업은 최초 1회 적용 절차다.

업데이트 이후 평소 시작 명령:

```bash
cd /home/kim/Control_Server
.venv/bin/python scripts/serverctl.py start
.venv/bin/python scripts/serverctl.py status
```

기존 `.venv`, `config/server.env`, 인증 파일을 사용한다. 정상적인 기존 토큰을 새로 만들거나 덮어쓰지 않는다.
중앙 서버만 켠 상태에서는 로봇 데이터가 없을 수 있다. 아래 AMR의 세 터미널까지 실행해야 연결된다.

## 4. AMR 터미널 1 — 통합 스택

### 최초 점검: CAN 송신 없이 실행

```bash
cd /home/acus03/amr_ws
./scripts/experiment.sh real can_dry_run:=true
```

이 프로세스를 켜 둔 채 다음 터미널로 진행한다. 실제 CAN 수신과 라이다를 사용하지만,
이 실행의 CAN 명령 송신 노드는 모터 명령을 전송하지 않는다. 시뮬레이션이 아니므로 실제 센서 연결은 필요하다.

통합 스택에 포함되는 항목:

- 조이스틱·수동/AUTO 제어와 CAN 송수신 노드
- 라이다·차체 TF·바퀴 odometry·IMU·EKF
- 지도·AMCL·TIM 자동 위치 찾기·초기 위치 검증
- Nav2와 시작 감독기, 시간 상태 감시

위치 검증 후 감독기가 Nav2를 활성화한다. 별도의 `bringup`, `localization`, `navigation` 실행을 추가하지 않는다.
`experiment.sh`에서 `real`을 생략하면 기본값이 `sim`이므로 위 명령 그대로 사용한다.

### 실제 주행: CAN 송신을 켜고 실행

먼저 점검을 마치고 이 문서의 종료 순서로 dry-run을 종료한다.
현장 운용 준비가 끝나면 **터미널 1의 명령을 아래 명령으로 바꾸고**, 터미널 2·3도 다시 실행한다.

```bash
cd /home/acus03/amr_ws
./scripts/experiment.sh real
```

**`can_dry_run`의 기본값은 `false`이므로 이 명령은 실제 CAN 모터 명령을 송신한다.**
재시작 후에는 위치와 준비 상태를 다시 확인한다.

| 실행 명령의 옵션 | 동작 |
|---|---|
| `real can_dry_run:=true` | 실제 센서 수신, 이 노드의 CAN 명령 송신 없음 |
| `real` | 실제 센서 수신 및 CAN 명령 송신 |
| `map:=/실제/지도/map.yaml` 추가 | 기본 G동 4층 지도를 지정한 지도로 변경 |
| `joy_device_id:=0` 추가 | 자동 이름 선택 대신 SDL 장치 번호 사용; 실제 조회된 번호로 변경 |
| `use_rviz:=true` 추가 | RViz도 실행 |

예를 들어 다른 지도에서 송신 없이 점검할 때는 다음처럼 실행한다. 지도 경로는 실제 파일로 바꾼다.

```bash
./scripts/experiment.sh real can_dry_run:=true map:=/실제/지도/map.yaml
```

## 5. AMR 터미널 2 — 웹 UI·API

```bash
cd /home/acus03/amr_ws
source scripts/env.sh
export ROS_DOMAIN_ID=7
ros2 launch amr_web_ui web_ui.launch.py use_sim_time:=false host:=127.0.0.1 port:=8080
```

AMR 자체 브라우저에서는 <http://127.0.0.1:8080>을 연다.
중앙 관제 연결도 같은 로컬 API를 사용하므로 이 터미널을 켜 둔다.

중앙 서버 없이 노트북에서 확인하려면, AMR에 SSH 접속이 가능한 환경에서
**노트북 터미널**에 다음 명령을 실행하고 유지한다.

```bash
ssh -N -L 18080:127.0.0.1:8080 acus03@100.106.98.122
```

그다음 노트북 브라우저에서 `http://127.0.0.1:18080`을 연다.
이 경로는 AMR 로컬 웹으로 연결되며 중앙 서버와 터미널 3이 필요 없다. SSH 인증은 기존 AMR 계정을 사용한다.

## 6. AMR 터미널 3 — 중앙 관제 연결

```bash
cd /home/acus03/amr_ws
source scripts/env.sh
export ROS_DOMAIN_ID=7
ros2 run amr_remote_bridge remote_client \
  --server-url http://100.88.249.77:8081 \
  --local-url http://127.0.0.1:8080 \
  --token-file "$HOME/.config/amr_remote_bridge/token" \
  --robot-id amr-01
```

운용 노트북에서 <http://100.88.249.77:8081/robots/amr-01/>을 연다.

- 로그인 이름: `operator`
- 비밀번호: 기존 공통 인증 파일의 값

중앙 관제 주소의 `8081`과 AMR 로컬 API의 `8080`을 구분한다.
인증 파일은 중앙 서버와 AMR에 같은 값이 있어야 한다.

## 7. 위치 확인과 주행 조작

### 위치 확인

1. MANUAL 상태에서 차량을 정지시키고 LB와 스틱을 놓는다.
2. 웹 지도가 실제 장소와 일치하는지 확인한다. 시작 시 자동 위치 찾기와 검증이 진행된다.
3. `위치 확인 필요`가 계속되면 `초기 위치`로 실제 위치와 방향을 지정하고 `초기 위치 적용`을 누른다.
4. 다시 자동 검색하려면 `자동 위치 찾기`를 누른다. 이 기능 자체는 차량을 회전시키지 않는다.
5. `위치 확인 중` → `주행 준비 중` → **`주행 준비 완료`**까지 기다리고 지도상의 차량 위치와 라이다 정합을 확인한다.

`자동 위치 찾기`나 `초기 위치` 요청이 접수된 것만으로 위치 검증이 완료된 것은 아니다.
위치를 다시 지정할 때는 대기·일시정지 중인 목표도 `주행 목표 취소`로 취소하고 실제 정지를 확인한다.
전역 위치가 아직 없으면 `/scan`을 수신하더라도 지도 위 라이다가 표시되지 않을 수 있다.

### 수동·자동 주행

| 조작 | 동작 |
|---|---|
| MANUAL에서 LB를 누른 채 스틱 조작 | 수동 주행; 왼쪽 스틱 전후 이동, 오른쪽 스틱 좌우 회전 |
| MANUAL에서 LB 놓기 | 수동 주행 정지 요청 |
| MANUAL에서 웹 목표 지정·전송 | 목표를 `출발 대기`로 보관 |
| RB 눌러 AUTO로 전환 | 준비가 완료되고 목표가 있으면 자동 주행 시작 |
| AUTO에서 RB 눌러 MANUAL로 전환 | 자동 주행 정지 요청; 목표가 일시정지 상태로 남을 수 있음 |
| 웹 `주행 목표 취소` | 보관·진행 중인 목표 취소; 완료 응답 확인 |

AUTO에서는 LB와 수동 스틱 입력을 사용하지 않는다. **AUTO 중 LB를 놓는 것을 정지 조작으로 사용하지 않는다.**
MANUAL로 돌아온 뒤 수동 주행을 재개하려면 LB를 한 번 놓았다가 다시 누른다.
브라우저 닫기나 Tailscale 연결 끊김도 차량 정지 명령이 아니다.

## 8. 선택: 상태 확인과 주행 기록

### 별도 AMR 터미널에서 상태 확인

```bash
cd /home/acus03/amr_ws
source scripts/env.sh
export ROS_DOMAIN_ID=7
ros2 param get /cmd_vel_to_can dry_run
ros2 topic echo /system/time_health --once --qos-durability transient_local
ros2 topic echo /localization/status --once --qos-durability transient_local
ros2 topic echo /control_mux/mode --once --qos-durability transient_local
```

점검 실행은 `dry_run=True`, 실제 운용은 `False`인지 확인한다.
시간 상태는 `healthy=true`, `latched=false`, 자율주행 위치 준비는 `ready=true`를 확인한다.
응답이 오지 않으면 `Ctrl+C`로 조회를 끝내고 스택 실행 여부와 도메인 `7`을 점검한다.

센서가 계속 들어오는지 확인하려면 다음 명령을 하나씩 실행한다. 각 조회는 `Ctrl+C`로 끝낸다.

```bash
ros2 topic hz /scan
ros2 topic hz /wheel/odometry
ros2 topic hz /imu/data_raw
ros2 topic hz /odometry/filtered
```

### 주행 전 기록 시작

```bash
cd /home/acus03/amr_ws
AMR_EXPERIMENT_MODE=real ./scripts/record_trial.sh real_first_trial
```

통합 스택을 먼저 실행하고, 위 명령의 기록 시작 메시지를 확인한 뒤 주행한다.
`real_first_trial`은 영문·숫자·밑줄·하이픈으로 바꿀 수 있다.

- 저장 위치: `bags/YYYY-MM-DD/HHMMSS_real_first_trial/` — 한국 시간 기준
- ROS bag: 해당 폴더의 `bag/`
- 함께 저장: 설정 사본·노드 파라미터·실험 정보·수집 토픽 목록
- 주요 기록: 속도 명령·바퀴/필터 odometry·IMU·라이다·지도·TF·경로·위치/주행 상태

`AMR_EXPERIMENT_MODE=real`을 생략하면 기본값이 시뮬레이션이므로 생략하지 않는다.
종료는 기록 터미널에서 `Ctrl+C`를 누르고 저장 완료 메시지를 기다린다.

## 9. 종료 순서

1. AUTO이면 RB로 MANUAL 전환한다. LB와 스틱을 놓는다.
2. 웹에서 `주행 목표 취소`를 누르고 취소 완료와 **실제 바퀴 정지**를 확인한다.
3. 기록 중이면 기록 터미널에서 `Ctrl+C`를 누르고 저장 완료를 기다린다.
4. 터미널 1에서 `Ctrl+C`를 누르고 통합 스택 종료를 기다린다.
5. 터미널 3의 `remote_client`, 터미널 2의 웹 API를 각각 `Ctrl+C`로 종료한다.
6. SSH 터널을 사용했다면 노트북의 터널 터미널도 `Ctrl+C`로 종료한다.

중앙 관제까지 종료할 때는 **중앙 서버 PC**에서 실행한다.

```bash
cd /home/kim/Control_Server
.venv/bin/python scripts/serverctl.py stop
```

## 10. 실행이 안 될 때

| 증상 | 확인할 내용 |
|---|---|
| `ros2` 또는 패키지를 찾지 못함 | AMR 터미널에서 `source scripts/env.sh`; 현재 `install/local_setup.bash` 존재 확인 |
| 지도만 나오고 차량 데이터가 없음 | 터미널 1·2 실행, 도메인 `7`; 원격이면 터미널 3도 실행했는지 확인 |
| `WAITING_INITIAL_POSE` / `위치 확인 필요` | 실제 장소와 지도 확인 후 초기 위치·방향 지정, 검증 결과 확인 |
| `/scan`은 나오지만 지도 위 라이다가 없음 | 전역 위치와 `map` 기준 TF가 아직 준비되지 않았는지 확인 |
| 시간 상태 `healthy=false` 또는 `latched=true` | 주행을 중단하고 센서 수신·시각 변경 원인을 확인한 뒤 스택을 재시작하고 위치 재검증 |
| 조이스틱이 인식되지 않음 | `joy_enumerate_devices` 결과와 설정 이름 확인; 필요 시 조회된 SDL 번호로 `joy_device_id` 지정 |
| 명령은 보이는데 dry-run에서 차량이 움직이지 않음 | `can_dry_run:=true`의 정상 동작; 실송신은 점검 종료 후 별도 실행 |
| 중앙 웹에 접속되지 않음 | 중앙 서버 전원·Tailscale 연결·`serverctl.py status`와 주소/포트 확인 |
| 로그인 실패 / 401 | 사용자 이름 `operator`, 기존 공통 인증 값 확인 |
| 로그인되지만 로봇이 offline 또는 데이터 대기 | AMR의 웹 API와 `remote_client` 실행 및 클라이언트 오류 확인 |
| 중앙 웹에 새 위치 찾기 버튼이 없음 | 중앙 서버 업데이트 적용·재시작, 웹 파일 경로 확인 후 브라우저 새로고침 |
| `Address already in use` | 같은 포트의 기존 웹 프로세스를 해당 터미널에서 정상 종료 후 재실행 |

이전 문서의 ‘초기 위치 입력 후 Nav2 별도 실행’ 절차는 새 통합 실행과 혼용하지 않는다.

## 11. A안 대시보드 시안과 실제 운용 UI 구분

위 `8080`·`8081`에서 실행되는 화면은 현재 포팅된 **실제 운용 웹 UI**다.
요청한 새 메인/차량 상태 화면은 [A안 HTML](../../../../ui_concepts_20261006/a.html),
전체 후보는 [시안 목록](../../../../ui_concepts_20261006/index.html)을 브라우저로 열어 확인한다.

A안은 아직 실제 CAN·ROS 데이터와 연결되지 않은 별도 HTML 시안이다.
배터리도 `25.6 V = 0%`, `27.4 V = 100%`를 기준으로 1% 단위 표시하는 시안 계산이며,
현재 실제 운용 웹에 실측 잔량 표시가 적용된 것은 아니다.

## 12. 백업·관련 문서

- 원본 src 백업: `backups/amr_src_before_migration_20261006_225723.tar.gz`
- [포팅 내용과 검증 결과](../../../../diagnostics/pc_update_20261006/REPORT.md)
- [중앙 서버 업데이트·복귀 절차](../Control_Server/UPDATE_20261006.md)
- [Jetson 이전 버전 복귀 절차](ROLLBACK_PC_UPDATE_20261006.md)
