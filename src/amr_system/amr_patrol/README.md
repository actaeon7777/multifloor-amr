# amr_patrol

현재 층의 YAML 경로를 순서대로 주행하고 `floors.yaml`의 home으로 복귀한다.
`patrol_manager`는 명령을 받기 전에는 주행하지 않는다. `motion_detector`는 순찰 중에만
정적 지도에 없는 이동 장애물을 추적하고 카메라 스냅숏과 이벤트를 저장한다. 층간 순찰은 포함하지 않는다.

- `routes.py`: ROS 없이 경로 검증, 같은 층 home 좌표 해석.
- `patrol.py`: ROS 없이 순찰 상태·재시도·관측·배터리 복귀 결정.
- `patrol_node.py`: 입력 인터록, `/navigate_to_pose` 목표 및 취소, 상태·이벤트 발행.
- `detection.py`: numpy/scipy 기반 지도 차분·8방향 클러스터·이동 추적, ROS 비의존.
- `detector_node.py`: 순찰 동안 센서 구독, TF 자세·회전 억제, JPEG와 `security.motion`, 전체 이벤트 스풀.
- `spool.py`: `~/amr_data/events/queue` 원자적 JSON 저장, 기본 2000개 한도와 미참조 이미지 7일 보관.
- `test/test_routes.py`, `test/test_patrol.py`, `test/test_detection.py`, `test/test_spool.py`: 단위 시험.
- `test/test_patrol_ros.py`: 명시적 opt-in, localhost 도메인 198의 가짜 action 통합 시험.
- `test/test_detector_ros.py`: 같은 opt-in·도메인, 합성 지도·TF·JPEG로 감지/스풀/구독 해제 확인.

`experiment.launch.py`의 `start_patrol:=true`는 두 노드를 시작한다. 영상 구독은 순찰 때만 열지만
`/robot/event` depth 10 구독은 항상 유지해 기존 `patrol.*`도 저장한다. 외부 비-ROS uploader가
스풀을 중앙 서버로 전달하고 전송 완료 파일을 삭제한다. 이 패키지는 전송을 담당하지 않는다.

Nav2의 `always_send_full_costmap: false` 설정에서는 시작 시 받은 전체 costmap이 오래된 자료일
수 있다. 전체 그리드가 1 Hz로 갱신된다고 가정하지 않고, 활성화 뒤 update 사각형으로 확인된
coverage에서만 후보를 만들며 처음 5초(`warmup_s`)는 추적만 한다. TF 구독도 순찰 동안만 유지한다.
거리 경계에 닿는 객체는 통째로 제외하고, 연속 프레임 매칭과 속도 상한(`max_person_speed`),
원점 점유 셀 비움(`vacated_fraction`)을 확인한다. 위치별 반복 억제(`repeat_radius_m`)는 트랙이
소실돼도 유지한다. JPEG는 저장할 때만 바이트로 복사하며, 큐 JSON이 참조하는 이미지는 삭제하지 않는다.

설정·토픽 계약·운용 제약·시험 명령은 저장소의 [docs/PATROL.md](../../../docs/PATROL.md)를 참조한다.
