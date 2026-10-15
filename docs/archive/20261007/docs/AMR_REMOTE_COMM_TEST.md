> **이력 문서** · 2026-10-07 정리 시점의 이전 안내입니다. 현재 실행은 [최신 운영 문서](../../../README.md), 검증은 [검증 문서](../../../VALIDATION.md)를 사용합니다.

# AMR 테더링 통신·실주행 테스트 — 터미널별 최종 명령

호스트 중앙 서버는 켜둔다. 기존 SSH 연결을 종료하고 AMR을 휴대폰 **모바일 데이터 테더링**에 연결한 뒤, 아래 명령을 **AMR 로컬 터미널**에서 실행한다.

앞부분은 ROS·CAN을 시작하지 않는 **미리보기 통신 시험**이다. 실제 주행은 아래
**실주행 전환 — AMR 터미널 5개** 절차를 사용한다. 인증 파일은 이미 전달한 파일을 사용한다.

## AMR 터미널 1 — 연결 확인 → 미리보기 API

먼저 연결과 인증 파일을 확인한다.

```bash
cd /home/acus03/amr_ws

tailscale status
tailscale ip -4
tailscale ping --timeout=5s --c=3 100.88.249.77

stat -c '%a %n' "$HOME/.config/amr_remote_bridge/token"

curl --noproxy '*' -i --connect-timeout 3 --max-time 5 \
  http://100.88.249.77:8081/fleet
```

AMR IP는 `100.106.98.122`, 파일 권한은 `600`, HTTP 응답은 **401**인지 확인한다. 401은 인증 없이 서버 포트까지 도달한 정상 응답이다.

이어서 같은 터미널에서 미리보기 API를 실행하고 그대로 둔다. 같은 API가 8090에서 실행 중이면 중복 실행하지 않는다.

```bash
cd /home/acus03/amr_ws

PYTHONPATH=src/amr_system/amr_web_ui \
python3 -m amr_web_ui.server \
  --preview-map maps/G_4_floor/4_floor_0522.yaml \
  --host 127.0.0.1 --port 8090
```

## AMR 터미널 2 — 중앙 서버 접속 클라이언트

터미널 1의 API가 실행된 뒤 입력한다. 같은 ID의 클라이언트를 중복 실행하지 않는다.

```bash
cd /home/acus03/amr_ws
source install/setup.bash

ros2 run amr_remote_bridge remote_client \
  --server-url http://100.88.249.77:8081 \
  --local-url http://127.0.0.1:8090 \
  --token-file "$HOME/.config/amr_remote_bridge/token" \
  --robot-id amr-preview
```

정상 연결 로그:

```text
Connected to central host as amr-preview
```

이 터미널도 실행 상태로 둔다.

## 호스트 PC 브라우저 — 지도·목표·취소 시험

```text
http://100.88.249.77:8081/robots/amr-preview/
```

로그인: 사용자 `operator`, 비밀번호는 서버 공통 토큰.

**미리보기 표시 확인 → 지도 빈 공간 클릭 → 목표 마커·방향 확인 → 목표 취소** 순서로 시험한다. 기본 주소 `/`는 `amr-01` 화면이므로 위 주소를 사용한다.

## AMR 터미널 3 — 상태 왕복 확인

호스트에서 목표를 지정한 뒤 실행한다. 취소 후 다시 실행하면 로컬 `goal`이 `null`로 바뀌어야 한다.

```bash
cd /home/acus03/amr_ws

curl --noproxy '*' -sS --max-time 5 \
  http://127.0.0.1:8090/api/state | python3 -m json.tool
```

중앙 서버의 등록 목록·미리보기 상태도 확인하려면 이어서 실행한다. 인증 파일을 자동으로 읽으며 토큰 내용은 출력하지 않는다.

```bash
cd /home/acus03/amr_ws

PYTHONPATH=src/amr_system/amr_remote_bridge python3 - <<'PY'
import base64
import json
from pathlib import Path
from amr_remote_bridge.common import HttpClient, loads, read_token

token = read_token(Path.home() / '.config/amr_remote_bridge/token')
auth = base64.b64encode(('operator:' + token).encode()).decode()
client = HttpClient()
for path in ('/api/robots', '/robots/amr-preview/api/state'):
    status, _, body = client.request(
        'http://100.88.249.77:8081' + path,
        headers={'Authorization': 'Basic ' + auth}, timeout=5)
    print(path, 'HTTP', status)
    print(json.dumps(loads(body), ensure_ascii=False, indent=2))
PY
```

두 응답 모두 **HTTP 200**, 목록의 `amr-preview.online`은 `true`, 상태의 `preview`는 `true`인지 확인한다.

## 단절·재접속 시험과 종료

- 단절 시험: 미리보기 목표 지정 후 **터미널 2만 Ctrl+C** → 중앙 연결 끊김·버튼 비활성화 확인 → 터미널 2 명령 재실행 → 목표가 자동 재전송되지 않는지 확인.
- 테더링 시험: 미리보기 API와 클라이언트를 유지하고 테더링을 끊었다 복구 → 자동 재접속 확인.
- 종료: 중앙 화면에서 목표 취소 → **터미널 2 Ctrl+C → 터미널 1 Ctrl+C**.

실제 주행 중 단절하면 이미 접수한 목표가 자동 정지되는 기능은 현재 없다.
위의 미리보기 단계에서는 `8090 / amr-preview`만 사용한다.

---

## 실주행 전환 — AMR 터미널 5개

실제 주행은 별도 문서 [AMR 중앙 관제 실주행 테스트](AMR_REMOTE_REAL_DRIVE_TEST.md)를 사용한다.

실주행 터미널 1~5, 선택 기록 터미널 6, 초기 위치 설정·AUTO 허용·정지·종료 순서를 해당 문서에 정리했다.

**실주행 실행 순서: 터미널 1 → 2 → 4 → 5 → 중앙 웹 초기 위치 설정·정합 확인 → 터미널 3 → Nav2 활성 상태 확인 → 목표 지정 → AUTO.**
Nav2를 먼저 실행하고 초기 위치 입력이 60초 이상 늦어지면 활성화가 실패할 수 있다.
초기 위치를 설정한 뒤 네 Nav2 노드가 `active [3]`인지 확인한다. 이미 실패했다면
실주행 문서의 **초기 위치 입력이 늦어서 Nav2 활성화에 실패한 경우** 절차로 복구한다.
