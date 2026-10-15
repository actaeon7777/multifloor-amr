# AMR 실운용 실행법

AMR에서 터미널 2개를 사용한다. 중앙 관제를 사용할 때만 서버와 AMR 터미널 3을 추가한다.
기본 지도는 G동 4층이다.

## 1. AMR 실행

### 터미널 1 — 차량 스택

```bash
cd /home/acus03/amr_ws
./scripts/experiment.sh real
```

센서·조이스틱·모터 제어·위치 추정·자동 주행·차량 상태를 함께 실행한다.

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
ssh -N -L 18080:127.0.0.1:8080 acus03@100.106.98.122
```

노트북 브라우저에서 **http://127.0.0.1:18080**을 연다.
중앙 관제를 사용하면 [중앙 관제 접속](#4-중앙-관제-사용-시)의 주소를 연다.

## 3. 주행 조작

웹에 **주행 준비 완료**가 표시되면 목적지를 지정한다.
**위치 확인 필요**가 표시되면 차량을 정지시키고 지도 상단의 **초기 위치**를 선택해
현재 위치와 방향을 지정한다.

| 조작 | 동작 |
|---|---|
| MANUAL에서 LB를 누른 채 왼쪽 스틱 | 전후 이동 |
| MANUAL에서 LB를 누른 채 오른쪽 스틱 | 회전 |
| MANUAL에서 LB 놓기 | 수동 주행 정지 |
| **목표 지정** 선택 후 지도 터치/클릭 | 목적지 전송 |
| 지도에서 누른 채 드래그 | 목적지와 도착 방향 전송 |
| RB | MANUAL ↔ AUTO 전환 |
| **주행 목표 취소** | 현재 웹 주행 목표 취소 |

MANUAL에서 지정한 목표는 출발 대기 상태로 보관되고, RB로 AUTO 전환하면 출발한다.
자동 주행 중 RB로 MANUAL 전환하면 목표를 유지한 채 일시정지한다.
MANUAL 복귀 후 수동 주행은 LB를 놓았다가 다시 누른다.

자동 주행을 멈출 때는 RB로 MANUAL 전환한다. LB 해제는 수동 주행 정지에만 사용한다.
브라우저를 닫아도 접수된 자동 주행 목표는 계속 실행된다.

## 4. 중앙 관제 사용 시

### 중앙 서버 PC — 서버 실행

```bash
cd /home/kim/Control_Server
.venv/bin/python scripts/serverctl.py start
```

### AMR 터미널 3 — 중앙 서버 연결

AMR의 터미널 1·2를 실행한 상태에서 추가한다.

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

브라우저에서 **http://100.88.249.77:8081/robots/amr-01/**을 연다.
로그인 이름은 **operator**, 비밀번호는 기존에 설정한 공용 토큰이다.
로봇 목록은 **http://100.88.249.77:8081/fleet**에서 연다.
층 탭·호실 선택과 층간 이동 지시는 로봇에서 직접 연 웹과 같은 방식으로 중앙 관제에서도 할 수 있다.

## 5. 종료

1. AUTO이면 RB를 눌러 MANUAL로 전환한다. 웹에서 **주행 목표 취소**를 누른 뒤 LB와 스틱을 놓아 차량을 정지시킨다.
2. 중앙 관제를 사용 중이면 AMR 터미널 3에서 `Ctrl+C`를 누른다.
3. AMR 터미널 2, 터미널 1 순서로 `Ctrl+C`를 누른다.
4. 노트북에서 SSH 터널을 사용 중이면 해당 터미널에서 `Ctrl+C`를 누른다.

중앙 관제 서버도 종료할 때는 중앙 서버 PC에서 실행한다.

```bash
cd /home/kim/Control_Server
.venv/bin/python scripts/serverctl.py stop
```
