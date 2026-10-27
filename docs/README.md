# AMR 실운용 실행법

> 다른 문서: [실차 레퍼런스](REAL_VEHICLE.md) · [위치 추정](LOCALIZATION.md) · [층간 이동](ELEVATOR.md) · [시뮬레이션](SIMULATION.md) · [검증](VALIDATION.md) · [실험 기록](EXPERIMENTS.md)

AMR에서 터미널 2개를 사용한다. 중앙 관제를 사용할 때만 서버와 AMR 터미널 3을 추가한다.
기본 지도는 G동 4층이다.

## 1. AMR 실행

### 터미널 1 — 차량 스택

```bash
cd /home/acus03/amr_ws
./scripts/experiment.sh real
```

센서·조이스틱·모터 제어·위치 추정·자동 주행·차량 상태를 함께 실행한다.
깊이 카메라까지 쓰려면 `./scripts/experiment.sh real start_camera:=true`로 실행한다(낮은 장애물 반영, 층간 이동 화면 전방 카메라 카드).
카메라를 켜면 카메라가 멈췄을 때도 자율주행이 멈추므로, 카메라 USB가 빠져 있으면 켜지 않는다.

### 터미널 2 — 웹 UI

```bash
cd /home/acus03/amr_ws
source scripts/env.sh
export ROS_DOMAIN_ID=7
ros2 launch amr_web_ui web_ui.launch.py \
  use_sim_time:=false host:=127.0.0.1 port:=8080
```

두 터미널은 운용 중 계속 실행해 둔다.

## 2. 웹 접속

AMR의 브라우저에서 **http://127.0.0.1:8080**을 연다.

노트북에서 직접 접속하려면 노트북 터미널에서 아래 명령을 실행해 둔다.

```bash
ssh -N -L 18080:127.0.0.1:8080 acus03@100.87.75.31
```

노트북 브라우저에서 **http://127.0.0.1:18080**을 연다.
중앙 관제를 사용하면 [중앙 관제 접속](#4-중앙-관제-사용-시)의 주소를 연다.

## 3. 주행 조작

비상정지 버튼을 누르면 진행 중인 목표가 중단된다. 버튼을 원위치로 돌리면 별도 승인 없이 잠시 **주행 준비 중**을 거쳐 주행 가능 상태가 된다.
이전 목표는 다시 출발하지 않으므로 목적지를 새로 지정한다. 자세한 동작은 [실차 레퍼런스](REAL_VEHICLE.md#e-stop-연동-0x202)에 있다.

웹에 **주행 준비 완료**가 표시되면 목적지를 지정한다.
**위치 확인 필요**가 표시되면 차량을 정지시키고 지도 상단의 **초기 위치**를 선택해
현재 위치와 방향을 지정한다.

| 조작 | 동작 |
|---|---|
| LB 누르기 | 자율주행 즉시 정지·목표 중단, 수동 입력 대기 (누르고 있는 동안 자율주행 목표 거절) |
| LB를 누른 채 왼쪽 스틱 | 전후 이동 |
| LB를 누른 채 오른쪽 스틱 | 회전 |
| LB 놓기 | 수동 주행 정지, 0.5초 뒤 새 목적지를 받음 |
| **목표 지정** 선택 후 지도 터치/클릭 | 목적지 전송 |
| 지도에서 누른 채 드래그 | 목적지와 도착 방향 전송 |
| 층 탭·호실 목록에서 호실 선택 | 그 호실 문 앞으로 이동. 다른 층(3F·5F)이면 엘리베이터로 층간 이동 (실차 `start_elevator_link:=true elevator_link_dry_run:=false` 필요) |
| **주행 목표 취소** | 현재 웹 주행 목표 취소 |

- 조이스틱을 켜 두어도 자율주행에는 영향이 없다. 연결·Bluetooth 끊김·재연결 모두 주행을 멈추지 않는다.
- 자율주행 중 **LB를 누르면** 즉시 멈추고 목표가 중단된다(웹 표시 **수동 조작 중**). 그대로 LB를 누른 채 스틱을 움직이면 수동 주행한다.
  조이스틱을 LB를 누른 채로 켠 경우에만 LB를 한 번 놓았다가 다시 눌러야 한다.
- LB를 놓아도 중단된 목표는 다시 출발하지 않는다(웹 표시 **수동 조작으로 중단됨**). 목적지를 다시 지정해야 출발한다.
- 자율주행을 멈출 때는 LB를 누르거나 웹의 **주행 목표 취소**를 누른다. 조이스틱이 꺼졌거나 범위 밖이면
  손에 든 정지 수단이 없으므로 웹 화면이나 하드웨어 비상정지 버튼을 가까이 둔다.
- LB를 놓은 0.5초 뒤부터 중앙 관제의 목표도 받는다. 수동 조작 직후 주변 사람이 있으면 관제 측과 먼저 확인한다.
- 브라우저를 닫아도 접수된 자동 주행 목표는 계속 실행된다.

## 4. 중앙 관제 사용 시

### 중앙 서버 PC — 경량 콘솔 실행

저장소 전체를 clone한 서버 PC에서 Python 3으로 실행한다. 설치 및 토큰 생성은
[Control_Server 안내](../Control_Server/README.md)를 따른다.

```bash
cd ~/multifloor-amr/Control_Server
python3 scripts/serverctl.py start
```

브라우저에서 **http://<서버 PC Tailscale IP>:8081/**을 연다.
로그인 이름은 **operator**, 비밀번호는 `~/.config/amr_remote_bridge/token`의 공용 토큰이다.
콘솔은 모든 로봇의 층·숫자 좌표·작업·배터리·정지 상태를 표시하고 **정지·해제·복귀**를 제공한다.
**순찰** 행에서 등록 경로를 선택하고 **순찰 시작**을 확인하면 야간 순찰을 요청한다.
순찰 중에는 경로·진행 단계와 `순찰 중` 배지가 표시되며 **순찰 정지**로 중단한다.
경로 없음·오프라인·관리자 정지·진행 중 목표·순찰 중에는 시작 버튼이 잠긴다.
순찰 중 키오스크는 배송을 받지 않으며, 기존 정지·복귀 명령도 순찰을 중단한다.
접수 후 작업·순찰 상태가 바뀌는지 확인한다([키오스크 순찰 안내](KIOSK.md#야간-순찰)).
정지는 수 초 지연될 수 있는 소프트웨어 정지이며 E-stop이 아니다.

### AMR — 중앙 서버 연결

아이콘 실행 시 `~/.config/amr/agent.env`가 있으면 에이전트도 시작한다.
설정 예시는 [실행법의 아이콘 섹션](../실행법.md#아이콘으로-실행-터미널-123을-한-번에)을 참고한다.
터미널 방식은 차량 스택과 로컬 웹 UI를 실행한 뒤 추가한다.
서버와 로봇의 토큰 파일은 같은 값이어야 하며 두 프로그램 모두 protocol 2를 사용해야 한다.

```bash
cd /home/acus03/amr_ws
source scripts/env.sh
ros2 run amr_remote_bridge remote_client \
  --server-url http://100.93.113.95:8081 \
  --local-url http://127.0.0.1:8080 \
  --token-file "$HOME/.config/amr_remote_bridge/token" \
  --robot-id amr-01
```

주소는 실제 서버 PC Tailscale IP로 바꾼다. 에이전트는 요약만 전송하며 이미지 전송은 없다.

### Slack 도착 DM (서버 PC만)

`~/.config/amr_remote_bridge/slack.env`에 `SLACK_BOT_TOKEN=xoxb-...`를 저장하고,
`slack_rooms.json`에 `{"401": "U0123ABCD"}`처럼 호실별 수신자를 지정한다.
앱 생성·설치와 `chat:write`, `im:write` 권한 설정은 [Slack 설정](../Control_Server/README.md#slack-도착-dm-설정-선택)을 따른다.
서버가 진행 상태를 관측한 배송의 도착만 DM으로 알린다. 복귀 및 서버 연결 전에 완료된 목표는 제외한다.
토큰이 없으면 Slack만 꺼진다. 로그는 서버의 `Control_Server/diagnostics/server.log`에서 확인한다.

## 5. 종료

1. LB를 누르거나 웹에서 **주행 목표 취소**를 눌러 자율주행을 멈춘다. 수동 주행 중이면 LB와 스틱을 놓아 차량을 정지시킨다.
2. 중앙 관제를 사용 중이면 AMR 터미널 3에서 `Ctrl+C`를 누른다.
3. AMR 터미널 2, 터미널 1 순서로 `Ctrl+C`를 누른다.
4. 노트북에서 SSH 터널을 사용 중이면 해당 터미널에서 `Ctrl+C`를 누른다.

중앙 관제 서버도 종료할 때는 중앙 서버 PC에서 실행한다.

```bash
cd ~/multifloor-amr/Control_Server   # 서버 PC에서 저장소를 clone한 위치
.venv/bin/python scripts/serverctl.py stop
```
