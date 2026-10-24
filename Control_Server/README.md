# AMR 중앙 관제 서버

중앙 서버 PC에서 실행한다.
웹 화면과 중계 코드는 저장소의 `src/amr_system/`(amr_web_ui, amr_remote_bridge, amr_control)을
그대로 쓰므로 `Control_Server/`만 따로 복사하지 말고 저장소 전체를 받아 둔다.

```bash
cd ~/multifloor-amr/Control_Server   # 서버 PC에서 저장소를 clone한 위치
.venv/bin/python scripts/serverctl.py start
```

브라우저에서 **http://100.88.249.77:8081/robots/amr-01/**을 연다.
로그인 이름은 **operator**, 비밀번호는 기존에 설정한 공용 토큰이다.
AMR 연결과 주행 조작은 [실운용 실행법](../docs/README.md#4-중앙-관제-사용-시)을 사용한다.

## 새 PC에 설치 (포팅)

`Control_Server/`만 복사하면 안 된다. `app/config.py`가 `../src/amr_system/`의 `amr_remote_bridge`·`amr_web_ui`·`amr_control`을 그대로 읽으므로
저장소 전체가 같은 구조로 있어야 한다. ROS는 필요 없다(순수 Python 3.12 + Tailscale).

```bash
git clone <저장소 URL> ~/multifloor-amr          # 전체 clone (브랜치는 AMR과 같은 것)
cd ~/multifloor-amr/Control_Server
python3 -m venv .venv
.venv/bin/pip install -r requirements.lock.txt      # fastapi, uvicorn 고정 버전
.venv/bin/pip install -r requirements-dev.txt       # 테스트용(선택)
cp config/server.env.example config/server.env      # 포트·robot_id 등. 기본값이면 생략 가능
.venv/bin/python scripts/create_token.py            # ~/.config/amr_remote_bridge/token 생성(이미 있으면 유지)
scripts/check_connection.sh [AMR Tailscale IP]      # Tailscale 로그인·AMR 도달 확인
.venv/bin/python scripts/serverctl.py start         # Tailscale IP에 바인드. --local이면 127.0.0.1
.venv/bin/python scripts/verify_host.py             # 인증·상태 endpoint 읽기 확인
.venv/bin/python -m pytest -q                       # 선택: 서버·중계 시나리오 전체
```

- 토큰은 AMR의 `~/.config/amr_remote_bridge/token`과 **같은 값**이어야 한다. 새로 만들었다면 AMR 쪽 파일에 복사한다(권한 600).
- 서버 주소는 `tailscale ip -4`가 알려 주는 이 PC의 IP다. AMR 터미널 3의 `--server-url`과 브라우저 주소를 그 IP로 바꾼다.
- 로그는 `diagnostics/server.log`, 상태는 `serverctl.py status`.

### AMR 작업 트리를 커밋 전에 반영하기

AMR(Jetson)에서 고친 웹 화면·중계 코드가 아직 커밋되지 않았으면 서버 PC에서 아래처럼 끌어온 뒤 재시작한다.

```bash
cd ~/multifloor-amr
rsync -av acus03@100.106.98.122:amr_ws/src/amr_system/amr_web_ui/web/ src/amr_system/amr_web_ui/web/
rsync -av acus03@100.106.98.122:amr_ws/src/amr_system/amr_web_ui/amr_web_ui/ src/amr_system/amr_web_ui/amr_web_ui/
rsync -av acus03@100.106.98.122:amr_ws/src/amr_system/amr_remote_bridge/ src/amr_system/amr_remote_bridge/
rsync -av acus03@100.106.98.122:amr_ws/Control_Server/app/ Control_Server/app/
cd Control_Server && .venv/bin/python scripts/serverctl.py restart
```

서버를 종료할 때는 아래 명령을 실행한다.

```bash
cd ~/multifloor-amr/Control_Server   # 서버 PC에서 저장소를 clone한 위치
.venv/bin/python scripts/serverctl.py stop
```
