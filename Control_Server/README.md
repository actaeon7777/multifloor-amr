# AMR 중앙 관제 서버

서버 PC에서 Python 3.10 이상과 표준 라이브러리만으로 실행하는 경량 콘솔이다.
ROS, 웹 프레임워크, 가상환경 설치가 필요 없다. 저장소의 `amr_remote_bridge` 코드와 콘솔 자산을
직접 읽으므로 저장소 전체를 같은 구조로 clone한다. 기존 `.venv/bin/python`이 있으면 실행기가 사용한다.

```bash
git clone <저장소 URL> ~/multifloor-amr
cd ~/multifloor-amr/Control_Server
python3 scripts/create_token.py
cp config/server.env.example config/server.env   # 기본값이면 생략 가능
python3 scripts/serverctl.py start               # 로그인된 서버 PC의 Tailscale IPv4
# 로컬 확인만 할 때: python3 scripts/serverctl.py start --local
```

브라우저에서 `http://<서버 PC Tailscale IP>:8081/`을 연다. 로그인 이름은 `operator`,
비밀번호는 `~/.config/amr_remote_bridge/token` 파일의 값이다. 이 토큰과 같은 값을 로봇의
같은 경로에 안전하게 배치하고 파일 권한을 600으로 둔다. `tailscale ip -4`로 서버 주소를 확인한다.
포그라운드 실행은 `scripts/run_server.sh --tailscale` 또는 `--local`이다.

콘솔은 모든 로봇의 층·숫자 좌표·작업·배터리·연결 나이·관리자 정지·모드/E-stop을 1초마다
표시한다. 명령은 **정지**, **해제**, **복귀**, **순찰 시작**, **순찰 정지**이며 지도·실시간 영상·목표 위치 입력은 제공하지 않는다.
정지는 수 초 지연될 수 있는 소프트웨어 정지이며 비상 정지(E-stop)가 아니다.
로봇 연결 설정은 [실운용 실행법](../docs/README.md#4-중앙-관제-사용-시)을 참고한다.
서버와 로봇 에이전트는 모두 protocol 2여야 한다. protocol 1 연결은 거부된다.

순찰 행에서 로봇이 제공한 경로를 선택하고 **순찰 시작**을 누른 뒤 확인한다.
등록 경로 없음·오프라인·관리자 정지·진행 중 목표·순찰 중에는 시작 버튼이 잠긴다.
순찰 중에는 경로·진행 순서·이동/관측/복귀 단계와 `순찰 중` 배지를 표시하고 **순찰 정지**가 켜진다.
요청 접수는 순찰 시작 완료를 뜻하지 않으며 다음 상태 갱신에서 작업·진행 상태를 확인한다.
순찰 중 키오스크는 배송을 받지 않는다. 기존 **정지**와 **복귀**도 순찰 정지 명령을 먼저 보낸다.
API 경로와 응답 규약은 [protocol 2 안내](../src/amr_system/amr_remote_bridge/README.md)를 참고한다.

## 감지 기록

카드 아래 **감지 기록**은 로봇에서 업로드한 최근 50건을 2초마다 확인한다.
시간·로봇·층·문구·심각도와 스냅숏 썸네일을 표시하며 **감지만**은 서버의 전체 기록에서 `security.*`를 조회한다. 기록은 발생 시각순이며 늦은 이미지가 순서를 바꾸지 않는다.
썸네일을 누르면 원본을 새 탭에서 연다. 이미지에도 운영자 인증이 필요하다.
**알림음 켜기**를 누르면 새 감지에 짧은 3회 소리와 페이지 제목 깜박임으로 알린다.
기본은 꺼짐이며 설정은 브라우저에 저장된다. 재방문 시 화면 클릭/키 입력이 있어야 오디오가
활성화되며 최초 과거 기록에는 소리를 내지 않는다. 감지 이벤트는 Slack DM을 보내지 않는다.

기본 저장 위치는 `Control_Server/data`이며 시작할 때 자동 생성한다.
`config/server.env`의 `AMR_DATA_DIR` 또는 `python3 -m app --host 127.0.0.1 --data-dir <경로>`로
바꿀 수 있다. SQLite는 `events.sqlite`, 이미지는 `images/<robot_id>/<id>.jpg`에 저장한다.
`AMR_MAX_EVENTS=5000`, `AMR_MAX_IMAGE_BYTES=1073741824`를 넘으면 오래된 기록과 이미지를
함께 삭제한다. 디렉터리는 서버 실행 계정이 쓸 수 있어야 하며 재시작 시에도 보존한다.
로봇은 서버 장애 동안 JSON을 스풀에 유지하고 재접속 후 중복 없이 이어서 보낸다.
로봇 원본 이미지는 업로더가 삭제하지 않는다. 400 KiB 초과·파일 누락·권한 오류·디렉터리 경로인
이미지는 경고 후 생략하고, 승인된 이벤트의 JSON을 지워 다음 기록을 처리한다.
업로드 재시도 간격은 상태 exchange와 독립적이다. 408/429를 제외한 4xx 거절 JSON은 `bad/`에 보관한다.

## Slack 도착 DM 설정 (선택)

1. Slack 관리 화면에서 앱을 만들고 사용할 워크스페이스를 선택한다.
2. OAuth & Permissions의 Bot Token Scopes에 `chat:write`, `im:write`를 추가한다.
3. 앱을 워크스페이스에 설치/승인하고 Bot User OAuth Token(`xoxb-…`)을 받는다.
   워크스페이스 정책에 따라 앱을 초대/승인한다. DM 수신자는 같은 워크스페이스의 사용자여야 한다.
4. 서버 PC에만 `~/.config/amr_remote_bridge/slack.env`를 만들고 다음 내용을 저장한다.
   파일 권한은 600으로 두며 토큰을 저장소에 커밋하거나 로봇에 배포하지 않는다.

   ```dotenv
   SLACK_BOT_TOKEN=xoxb-여기에-봇-토큰
   ```

5. 사용자 프로필의 사용자 ID 복사로 수신자 ID를 확인하고,
   `~/.config/amr_remote_bridge/slack_rooms.json`에 호실별 수신자를 등록한다.

   ```json
   {"401": "U0123ABCD", "402": "U0456EFGH"}
   ```

6. `python3 scripts/serverctl.py restart`로 설정을 반영한다.

경로는 `config/server.env`의 `AMR_SLACK_ENV`, `AMR_SLACK_ROOMS` 또는
`python3 -m app --host 127.0.0.1 --slack-env <경로> --slack-rooms <경로>`로 바꿀 수 있다.
봇 토큰이 없으면 알림만 비활성화된다. 호실 매핑이 없으면 해당 호실을 로그에 한 번 남긴다.
서버가 진행 중 목표를 본 뒤 성공 상태를 받았을 때만 수신자 DM을 보낸다. 복귀 목표와 처음부터
성공으로 관측된 목표는 제외한다. `(robot_id, request_id)` 최근 256건을 메모리에서 중복 제거한다.
전송은 백그라운드 큐에서 처리하고 최대 5회 시도하며, 429의 Retry-After를 따른다.
서버 재시작으로 메모리 기록은 초기화된다. 알림은 배송 수령 확인을 대신하지 않는다.

## 운영과 검증

```bash
python3 scripts/serverctl.py status
python3 scripts/serverctl.py restart
python3 scripts/serverctl.py stop
scripts/check_connection.sh                      # Tailscale 및 인증 응답 확인
python3 scripts/verify_host.py                   # 콘솔과 로봇 목록 GET 검증
python3 -m pytest -q                             # pytest 설치 환경에서 bridge 시험
```

서버 로그는 `Control_Server/diagnostics/server.log`, 프로세스 기록은 `.run/server.json`이다.
로그는 시작 시 새로 기록한다. `verify_host.py` 결과는 `diagnostics/host_result.json`에 저장한다.
테스트의 로컬 HTTP 스텁은 로봇 제어 API를 호출하지 않으며 Slack도 가짜 전송 함수로 시험한다.
루프백 소켓 생성이 허용되는 환경에서 HTTP 시험을 실행해야 한다.
선택적 브라우저 시험은 Playwright/Chromium이 있는 환경에서 `python3 scripts/browser_check.py`로 실행한다.
