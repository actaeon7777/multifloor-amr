# 공용 차량 상태

웹과 독립적으로 `/vehicle/status`를 발행한다. [차량 스택](../../../docs/README.md#1-amr-실행)에 포함돼 함께 실행된다.

- 타입: [amr_vehicle_interfaces/msg/VehicleStatus](../amr_vehicle_interfaces/msg/VehicleStatus.msg)
- 주기: 기본 10 Hz
- QoS: reliable / volatile / depth 1
- 다른 제어기는 같은 토픽과 메시지 타입을 구독한다.

측정값은 `received && valid && fresh`가 참일 때 사용한다.
미연동 항목은 `received=false`이며, 수신 만료나 발행 세션 변경은 소비자가 처리한다.
상태 노드는 차량 제어 명령을 보내지 않는다.

좌우 드라이버의 CAN 입력 전압·전류는 `left_motor/right_motor`의 `power`,
`input_voltage_v`, `bus_current_a`에 담긴다. Heartbeat의 상태·오류도 함께 제공한다.
`battery_voltage_v`는 양쪽 최신 전압의 평균이며 `battery_voltage_available`로 사용 가능 여부를 판정한다.
한쪽 전압이 2.5초 이상 갱신되지 않으면 평균 전압도 만료된다. BMS SOC·온도와 드라이버 온도는 아직 미연동이다.
