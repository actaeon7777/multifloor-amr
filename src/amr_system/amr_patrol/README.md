# amr_patrol

현재 층의 YAML 경로를 순서대로 주행하고 `floors.yaml`의 home으로 복귀한다.
`patrol_manager`는 명령을 받기 전에는 주행하지 않는다. 탐지와 층간 이동은 포함하지 않는다.

- `routes.py`: ROS 없이 경로 검증, 같은 층 home 좌표 해석.
- `patrol.py`: ROS 없이 순찰 상태·재시도·관측·배터리 복귀 결정.
- `patrol_node.py`: 입력 인터록, `/navigate_to_pose` 목표 및 취소, 상태·이벤트 발행.
- `test/test_routes.py`, `test/test_patrol.py`: 단위 시험.
- `test/test_patrol_ros.py`: 명시적 opt-in, localhost 도메인 198의 가짜 action 통합 시험.

설정·토픽 계약·운용 제약·시험 명령은 저장소의 [docs/PATROL.md](../../../docs/PATROL.md)를 참조한다.
