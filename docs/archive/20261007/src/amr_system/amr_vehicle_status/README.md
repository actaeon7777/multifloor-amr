> **이력 문서** · 2026-10-07 정리 시점의 이전 안내입니다. 현재 실행은 [최신 운영 문서](../../../../../README.md), 검증은 [검증 문서](../../../../../VALIDATION.md)를 사용합니다.

# 공용 차량 상태

`amr_vehicle_interfaces/msg/VehicleStatus`를 `/vehicle/status`에 기본 10 Hz로 발행합니다.
상태 발행기는 웹 UI와 독립적으로 실행되며 차량 상태를 읽는 다른 ROS 제어기도 같은 메시지를 구독할 수 있습니다.

`amr_stack/bringup.launch.py`가 상태 노드를 기본 시작합니다. 분리 실행:

```bash
ros2 launch amr_vehicle_status vehicle_status.launch.py
# 시뮬레이션에서 분리 실행할 때는 use_sim_time:=true
ros2 topic echo /vehicle/status
ros2 interface show amr_vehicle_interfaces/msg/VehicleStatus
```

이미 bringup에서 실행 중이면 별도 상태 노드를 추가로 실행하지 않습니다.
bringup의 `start_vehicle_status:=false`로 별도 실행 구성을 선택할 수 있습니다.

메시지는 다음 정보를 제공합니다.

- 좌측 node 1 / 우측 node 0의 전진 방향 rpm과 개별 드라이버 데이터의 유효 상태
- LiDAR·바퀴 odom·IMU·EKF·최종 속도 명령의 수신 빈도, 측정 시각, 경과 시간
- 지도 좌표의 차량 위치, 제어 모드, 주행 상태, 위치 감독기 검증과 시간 감시 결과
- CAN 진단 오류 카운터와 연속성이 확인된 구간의 바퀴 이동 거리
- 드라이버 전압·전류·온도·Heartbeat와 BMS 필드. 현재 입력이 없는 필드는 `received=false`

각 `DataStatus`의 `received && valid && fresh`가 참일 때 해당 값을 사용합니다.
`valid=true, fresh=false`는 정상 형식으로 수신했으나 오래된 데이터입니다.
`received=true, valid=false`는 현재 입력의 시간 또는 값이 유효하지 않은 상태입니다.
숫자 0만으로 정지·오류 없음·방전 여부를 판단하지 않습니다.
`source_stamp`는 `stamp_valid=true`일 때 원본 ROS 측정 시각입니다.
`rate_hz`는 수신 노드의 최근 최대 2초 / 200개 표본으로 계산한 빈도이며 `rate_valid`로 유효성을 확인합니다.

발행 QoS는 reliable / volatile / depth 1입니다. 주기는 기본 10 Hz이며 1~50 Hz로 설정합니다.
`header.stamp`는 ROS 시간, `published_wall_stamp`는 UTC 시스템 시간입니다.
`publisher_session`은 발행 프로세스마다 바뀌고, `sequence`는 메시지마다 증가합니다.
`epoch`는 시계 불연속 시 증가합니다. 소비자는 이전 세션이나 순번의 메시지를 최신 상태로 되살리지 않습니다.

다른 제어기는 아래처럼 타입을 구독하고, 콜백의 메시지를 보관할 때 수신 만료도 검사합니다.

```python
from amr_vehicle_interfaces.msg import VehicleStatus
from rclpy.qos import QoSProfile

self.create_subscription(VehicleStatus, '/vehicle/status', self.on_vehicle_status,
                         QoSProfile(depth=1))
```

소비자는 로컬 monotonic 시간으로 마지막 메시지 수신 이후 0.5초를 넘으면 상태를 만료시킵니다.
콜백에서 `published_wall_stamp`도 확인하여 이미 0.5초 이상 오래된 패킷은 최신 상태로 접수하지 않습니다.
표본별 경과 시간에는 발행 이후 전송·보관 지연을 더한 뒤 `timeout_sec`와 비교합니다.
여러 컴퓨터에서는 기존 시간 동기화 구성을 사용합니다.
제어 모드와 주행 상태는 원본의 retained 변경 이벤트이며 `*_received`가 생산 노드의 생존을 보증하지 않습니다.
`localization_ready`는 감독기의 최신 검증 결과이고, 실제 주행 명령은 기존 Action·mux의 최종 검증을 거칩니다.

발행기는 `/cmd_vel`·CAN·주행 Action·초기 위치에 명령을 보내지 않습니다.
웹은 차량 상태 화면에 이 토픽을 사용합니다. 지도와 주행 요청 처리는 기존 지도·Action·감독기 경로를 사용합니다.
웹 목표별 진행 거리는 공용 `wheel_distance_m`의 차이로 계산합니다.
`wheel_distance_epoch`가 바뀌면 이전 목표의 전체 이동 거리는 연속성이 끊겼으므로 표시하지 않습니다.

검증:

```bash
python3 -m pytest -q src/amr_system/amr_vehicle_status/test/test_model.py
ROS_DOMAIN_ID=194 ROS_LOCALHOST_ONLY=1 AMR_RUN_VEHICLE_STATUS_TESTS=1 \
  python3 -m pytest -q src/amr_system/amr_vehicle_status/test/test_node_ros.py
```

ROS 검사는 격리 도메인에서 빈 상태, 실제 생성 메시지, 잘못된 입력, READY·시간 감시 만료,
시계 불연속과 명령 publisher 부재를 확인합니다. 웹 ROS 검사에서는 같은 상태 토픽을
웹과 두 번째 ROS 소비자가 함께 구독하고, 원본 센서 중단 후 신선도 해제를 확인합니다.
