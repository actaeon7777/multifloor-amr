# AMR Remote Bridge

AMR이 중앙 서버에 상태·지도를 전송하고 로컬 웹 API로 명령을 중계한다.
실행과 접속은 [실운용 실행법](../../../docs/README.md#4-중앙-관제-사용-시)을 사용한다.

protocol 1의 구현은 `amr_remote_bridge/common.py`, `agent.py`, `server.py`에서 관리한다.

## 전달 내용

- 상태 `/api/state`와 실시간 레이어(`map`, `local_costmap`, `global_costmap`) PNG.
- 층 목록 `/api/floors`와 각 층의 정적 지도 PNG(레이어 이름 `floor/<층>`). 층 지도는 버전이 바뀌거나
  서버가 요청할 때만 다시 보낸다. 층 기능이 없는 웹 UI는 빈 목록으로 전달된다.
- 명령 `goal`·`initialpose`·`cancel`·`relocalize`. `goal`에 `floor`·`room`이 있고 로봇의 현재 층과 다르면
  서버는 해당 층 지도의 버전·해시로 검증하고(`map_layer: floor/<층>`), 클라이언트는 전달 직전에 로컬의
  같은 층 지도와 대조한다. 그 뒤는 로컬 웹 UI가 층간 미션으로 넘긴다.
