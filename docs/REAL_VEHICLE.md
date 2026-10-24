# 실차 제어·센서 레퍼런스

Jetson(JetPack 7.2.1 / Ubuntu 24.04 / arm64)에서 도는 실차 스택의 동작과 설정을 정리한다.
일상 운용 절차는 [실운용 실행법](README.md), 시험 명령은 [검증](VALIDATION.md), 과거 측정은 [실험 기록](EXPERIMENTS.md)에 있다.

## 1. 구성

```text
joy_node → /joy → joy_teleop → /cmd_vel_manual ─┐
                                                ├→ control_mux → /cmd_vel → cmd_vel_to_can → CAN 0x201 → STM32 → ODrive
Nav2 → navigation_manager → /cmd_vel_auto ──────┘   (바퀴 속도 제한)        ▲
                                                          STM32 E-stop 상태 0x202 ┘ → /estop/status

CAN 피드백 → can_telemetry → /wheel/odometry, /joint_states, /imu/data_raw → EKF → odom → base_footprint
S2 LiDAR → /scan_raw → scan_body_filter → /scan → AMCL → map → odom
```

| 패키지 | 실차에서의 역할 |
|---|---|
| `amr_stack` | bringup·localization·navigation·experiment launch와 설정 YAML, 지도 |
| `amr_control` | `control_mux`(수동/자율 선택, 바퀴 속도 제한), `time_health` |
| `amr_can_bridge` | `cmd_vel_to_can`(송신), `can_telemetry`(수신), `elevator_can_link`(층간 이동용, 기본 꺼짐) |
| `amr_description` | v80 URDF, 실물 `laser`·`imu_link` 프레임 |
| `teleop_tools` | F1TENTH 소스에서 복사한 `joy_teleop` (출처 `src/amr_system/teleop_tools/AMR_SOURCE.md`) |

설정 파일은 모두 `src/amr_system/amr_stack/config/`에 있다. 설정 변경은 해당 launch 재시작 후 적용된다.
AMR 자체 패키지 라이선스는 정책 결정 전까지 `TODO`이며, 복사한 패키지는 원래 라이선스를 유지한다.

`./scripts/experiment.sh real`이 도메인 7에서 위 전체(센서·조이스틱·모터 제어·위치 추정·자동 주행·차량 상태)를 한 번에 실행한다.
구성 요소를 따로 띄우는 방법은 [11장](#11-구성-요소별-실행)에 있다.

## 2. 빌드

ROS Jazzy를 source한 셸에서 빌드한다. Jetson 메모리를 고려해 병렬 작업 수를 2로 제한한다.

```bash
cd /home/acus03/amr_ws
source /opt/ros/jazzy/setup.bash
MAKEFLAGS=-j2 CMAKE_BUILD_PARALLEL_LEVEL=2 colcon build --base-paths src --symlink-install \
  --parallel-workers 2 --cmake-args -DCMAKE_BUILD_TYPE=Release -DPython3_EXECUTABLE=/usr/bin/python3
```

`source scripts/env.sh`가 Jazzy와 AMR 패키지를 불러오고 `ROS_DOMAIN_ID`를 기본 7로 둔다(이미 지정한 값은 유지).
Jetson에는 이를 실행하는 `amr` 별칭과 `ros2 launch amr_stack bringup.launch.py`를 실행하는 `stack` 별칭이 설정되어 있다.
같은 ROS 도메인에서 다른 주행 노드를 함께 실행하지 않는다.

CAN 송신에는 `sudo apt-get install python3-can`이 필요하다. `can0`는 [`deploy/jetson/can0.service`](../deploy/jetson/can0.service)
(`/etc/systemd/system/`에 설치)가 부팅 시 500 kbit/s 일반 CAN으로 설정한다.
상태는 `systemctl status can0.service`, `ip -details link show can0`로 확인한다.

## 3. 조이스틱과 control_mux

### 매핑

Xbox 컨트롤러의 `/joy` 인덱스(0부터). `joy.device_name`은 SDL 이름 `Xbox Series X Controller`로 지정해 다른 입력 장치를 고르지 않는다.
연결 방식에 따라 이름이 다르면 `ros2 run joy joy_enumerate_devices`로 확인하고 `joy_teleop.yaml`의 `device_name`을 바꾼다.

| 기능 | 입력 | 출력 |
|---|---|---|
| LB 끼어들기·데드맨 | `buttons[6]` | 누르는 순간 MANUAL(자율 정지·목표 중단), 누르는 동안만 수동 명령 허용, 마지막 입력 0.5초 뒤 AUTO |
| 조이스틱 연결 | `/joy` 수신 | 모드에 영향 없음 (RB 등 다른 버튼도 모드를 바꾸지 않음) |
| 전진·후진 | `axes[1]` | `linear.x`, 최대 ±1.00 m/s, 전진 양수 |
| 좌회전·우회전 | `axes[2]` | `angular.z`, 최대 ±0.80 rad/s, 좌회전 양수 |

`axes[0]`·`axes[3]`은 사용하지 않는다. 스틱 데드존 0.05, `/joy` 20 Hz. 설정은 `joy_teleop.yaml`.
`autorepeat_rate`(20 Hz)는 스틱을 움직이지 않고 LB만 누르고 있어도 `/joy`를 계속 발행하게 하므로 0으로 바꾸지 않는다(LB 유지·`/joy` 만료 판정에 필요).
YAML 없이 노드를 단독 실행하면 코드 기본 상한 0.20 m/s, 0.40 rad/s를 사용한다.

### 모드 규칙

모든 속도 토픽은 `geometry_msgs/msg/Twist`(`linear.x=v`, `angular.z=w`)다.
자율 노드는 `/cmd_vel_auto`에 발행하고 **`/cmd_vel`은 mux만 발행**한다. `/cmd_vel`을 쓰는 기존 자율 노드에는
그 노드에만 `--ros-args -r /cmd_vel:=/cmd_vel_auto`를 적용한다.

- 시작과 기본은 AUTO다. 조이스틱 연결·끊김·재연결, 잘못된 `/joy`는 모드를 바꾸지 않는다.
- 유효한 `/joy`에서 LB가 눌려 있으면 즉시 MANUAL. 연결된 상태에서 LB 해제가 한 번 관찰됐다면 그 누름이 곧 수동 무장이라
  바로 수동 입력 대기(`waiting_command`)가 된다. 연결 시 LB가 이미 눌려 있으면 MANUAL(자율 정지)이지만 수동은 LB를 놓았다 다시 눌러야 한다.
- 마지막으로 LB가 눌린 `/joy`로부터 `manual_holdoff`(0.50초)가 지나면 AUTO로 돌아간다. 40 ms 같은 짧은 LB 튐은 MANUAL을 끝내지 않는다.
- MANUAL에서는 자율 명령을 버린다. AUTO에서는 스틱·수동 명령을 무시하고 수동 입력 오류도 자율 출력을 덮어쓰지 않는다.
- 모드 전환마다 0.10초 동안 0을 출력하고, 이전 모드에서 받은 명령은 버린다.
- 자율 명령이 0.50초 만료되거나 잘못되면 0을 출력한다. 수동으로 대체하지 않으며 새 유효 명령이 오면 재개한다.
- LB를 누른 채 `/joy`가 끊기면 0.50초 뒤 수동 출력을 멈추고 AUTO로 돌아간다. 재연결 후 수동은 LB 재입력이 필요하다.
- `joy_autoselect`는 같은 이름의 컨트롤러가 하나뿐이면 이름(`device_name`)으로 joy_node를 실행해, Bluetooth가 다른 SDL 번호로
  재연결해도 `/joy`가 다시 나오게 한다. 같은 이름이 여러 개면 고른 번호로 고정하므로 재연결이 보장되지 않는다(실차에서 한 대만 켠다).
- `/control_mux/mode`(`MANUAL`/`AUTO`)와 `/control_mux/status`는 transient-local이라 늦게 구독해도 마지막 상태를 받는다.

### 정지 처리

- MANUAL에서 LB 해제는 0.10초(`release_debounce`, 0.0이면 즉시) 확인 뒤 정지한다. 더 짧은 변화는 채터링으로 무시한다.
- 짧거나 잘못된 Joy 배열은 두 모드 모두 즉시 정지한다.
- `/joy`와 현재 모드의 속도 명령은 각각 0.50초 이상 새로 오지 않으면 정지한다. 여기에 최대 한 타이머 주기(0.05초)와 실행 지연이 더해질 수 있다.
- 수동 조작은 시작, `/joy` 만료, 잘못된 수동 입력 뒤에 유효한 LB 해제와 새 누름을 요구한다.
  `/joy`와 LB가 유효한데 수동 명령만 만료되면 출력만 0으로 멈추고 새 명령이 오면 LB 재입력 없이 복구된다.
- 시간 판정은 monotonic clock을 사용해 ROS 시간 정지·변경의 영향을 받지 않는다.
- 장치·드라이버가 연결이 끊긴 뒤에도 오래된 스틱 값을 계속 발행하면 타임아웃으로는 무선 손실을 감지할 수 없다. 실제 Bluetooth 전원 차단 시험이 필요하다.
- `control_mux`가 종료되면 CAN 브리지가 `/cmd_vel` 수신 타임아웃으로 0을 보낸다. 브리지 종료(SIGINT/SIGTERM) 시 0 프레임을 5회 송신 시도한다.
  전원 차단·SIGKILL·CAN 단선에서는 보장되지 않으므로 STM32의 수신 타임아웃 정지(300 ms)에 의존한다.

`/control_mux/status` 값:

| 값 | 의미 |
|---|---|
| `active` | 주행 허용 |
| `release_pending` / `deadman_released` | LB 해제 확인 중 / LB 해제 정지 |
| `command_timeout` | 수동 명령 지연 (자동 복구) |
| `joy_timeout` | Joy 연결 만료 (LB 재입력 필요) |
| `invalid_joy` / `invalid_command` | 잘못된 입력 |
| `awaiting_release` / `waiting_command` | 시작 후 LB 해제 대기 / LB 눌림, 새 수동 명령 대기 |
| `auto_waiting_command`, `auto_active`, `auto_command_timeout`, `auto_invalid_command` | 자율 모드 상태 |

정지 판단은 최종 `/cmd_vel`에서 한다. `/cmd_vel_manual`은 LB 해제 시 발행을 멈추므로 마지막 메시지가 화면에 남아 있을 수 있다.

### 실제 컨트롤러 확인 순서

1. 시작 후 LB를 한 번 놓는다(LB를 누른 채 시작하면 출력은 0).
2. LB 없이 스틱을 움직여도 `/cmd_vel`이 0인지 확인한다.
3. LB + 왼쪽 스틱 앞/뒤 → `linear.x` 양/음, LB + 오른쪽 스틱 왼/오 → `angular.z` 양/음.
4. 스틱 중립 → 0, 스틱을 기울인 채 LB를 0.10초 이상 놓음 → 0.
5. 컨트롤러 전원을 끄거나 Bluetooth를 끊어 정지와 재연결을 확인한다. `/joy`가 0.50초 이상 끊긴 뒤에는 LB를 다시 눌러야 한다.
6. 컨트롤러를 켜고 끄는 동안 `/control_mux/mode`가 `AUTO`로 유지되는지, LB를 누르면 즉시 `MANUAL`, 놓으면 0.5초 뒤 `AUTO`가 되는지 확인한다
   (`ros2 topic echo /control_mux/mode --qos-durability transient_local`). 재연결 후 `ros2 topic hz /joy`가 다시 나와야 한다(Bluetooth 무입력 시 실측 약 13~15 Hz, 조작 중 더 높음).
7. 바퀴를 띄우고 웹에서 목표를 보내 주행 중 컨트롤러를 켠다 → 주행 계속. LB를 누른다 → 즉시 0, 웹 **수동 조작 중**, 목표 전송 거절.
   LB를 놓는다 → 정지 유지, 웹 **수동 조작으로 중단됨** → 새 목표로만 다시 출발. 주행 중 컨트롤러를 끈다 → 주행 계속.

## 4. 바퀴 속도 제한과 STM32 변환

- `control_mux`가 모드를 고른 뒤 바퀴 속도를 제한한다. `control_mux.yaml`의 `wheel_diameter` 0.172 m, `track_width` 0.529 m, `max_wheel_rev_s` 1.8 rev/s.
- 바퀴 선속도 상한 `U = π × wheel_diameter × max_wheel_rev_s`, 필요한 최대 바퀴 속도 `M = |v| + |ω| × track_width / 2`.
  `M > U`이면 `(v, ω)` 모두에 `U / M`을 곱해 회전 반경을 유지한다. 후진·제자리 회전도 같다.
  기본값에서 직진 최대 약 0.973 m/s, `(1.0, 0.8)` 요청은 약 `(0.803, 0.642)`. 원래 요청은 `/cmd_vel_manual`, 제한 후는 `/cmd_vel`에서 본다.
- [`firmware/stm32/main.c`](../firmware/stm32/main.c)는 받은 `(v, ω)`에 역기구학과 모터 장착 방향만 적용한다.
  바퀴별 클리핑과 별도 gain은 제거했고, 통신 타임아웃과 모터 오류 정지는 유지한다.
  `drive_wheel_diameter_m`·`drive_track_width_m`는 상위 설정과 일치해야 하며 `drive_node0/1_direction`은 방향 부호(±1)다.
- 바퀴 속도 상한은 상위 `max_wheel_rev_s`에서 바꾼다. 다른 CAN 송신 경로를 쓰면 그 경로에서도 상한을 적용해야 한다.
- 상위에는 가감속 제한을 두지 않고 ODrive의 기존 `VEL_RAMP`가 처리한다. ODrive 설정은 이 저장소에서 바꾸지 않는다.
- 펌웨어 변경은 `main.c`를 STM32 프로젝트의 `Core/Src/main.c`에 반영해 빌드·업로드해야 적용된다. `colcon build`는 펌웨어를 올리지 않는다.
  `amr_control`의 `test_wheel_limits.py`가 이 C 파일의 변환 함수를 호스트 GCC로 컴파일해 상위 제한과 함께 검증한다.

## 5. CAN 송신 (`cmd_vel_to_can`)

| 항목 | 값 |
|---|---|
| 채널 | `can0` (SocketCAN), 500 kbit/s 일반 CAN |
| CAN ID | `0x201`, 표준 11비트, DLC 8 |
| byte 0–3 / 4–7 | `float32 v` m/s / `float32 w` rad/s, little endian |
| 송신 주기 | 20 Hz |
| 정지 데이터 | `0000000000000000` |
| 예시 | `v=0.2, w=0.4` → `201#CDCC4C3ECDCCCC3E` |

- 기본 bringup은 실제 송신까지 한다. `can_dry_run:=true`는 소켓을 열지 않고 `/cmd_vel_to_can/tx_preview`에
  `DRY_RUN can0 201#... v=... w=... reason=...`만 출력한다(수신 노드는 별도로 실제 CAN을 읽는다).
  bringup의 `can_dry_run`이 YAML `dry_run`보다 우선한다. 설정 파일은 `can_bridge.yaml`.
- 브리지는 인터페이스 설정을 바꾸지 않는다. 송신 시작 전 UP 플래그를 확인하고 DOWN이면 안내를 출력하고 종료한다.
  송신 모드에서 장치를 열 수 없으면 dry_run으로 자동 전환하지 않고 실패한다.
- 정지 조건:
  - `/cmd_vel`이 0.50초 만료되면 0을 보내고, 새 유효 명령이 오면 복구한다.
  - 시작 시, 잘못된 입력·송신 오류 뒤에는 새 0 명령을 받아야 비영점 명령을 허용한다(MANUAL은 LB 재입력, AUTO는 모드 전환 시 0 출력).
  - 송신 오류 시 이전 속도를 버리고 0 프레임만 재시도한다. 송신 대기는 최대 0.01초.
- 기존 키보드 송신 프로그램이나 다른 CAN 브리지와 함께 실행하지 않는다. 최초 실차 검증은 바퀴를 띄운 상태에서 정지와 방향을 먼저 확인한다.
  실제 프레임은 `candump can0,201:7FF`로 관찰한다.

### E-stop 연동 (`0x202`)

STM32는 하드웨어 E-stop을 `0x202` 상태 프레임으로 알리고, 버튼이 원위치로 돌아오면 스스로 정지를 해제한다.
**재가동 승인은 Jetson 책임**이라 `cmd_vel_to_can`이 아래 상태 머신으로 `0x201` 송신을 막는다. 프로토콜 전체는 [ESTOP_CAN_PROTOCOL.md](ESTOP_CAN_PROTOCOL.md)에 있다.

| 상태 | 뜻 | `0x201` 송신 |
|---|---|---|
| `UNKNOWN` | 부팅 후 `0x202`를 아직 받지 못함 | 보내지 않음 |
| `ESTOP` | 버튼 눌림·단선, `trip_count` 증가, `0x202` 0.5초 미수신, 잘못된 상태 프레임 | 보내지 않음 |
| `DISARMED` | STM32는 해제됐지만 운영자 승인 전 | 보내지 않음 |
| `ARMING` | 승인 후 STM32 주행 gate가 열리기를 기다림 | `(0, 0)`만 |
| `READY` | `drive_enabled=1` | `/cmd_vel` (승인 뒤 새로 받은 0 명령 이후부터) |
| `BYPASS` | `can_dry_run:=true` 또는 `require_estop_status: false` | 기존 동작 |

- **매 부팅과 매 E-stop 뒤에 재개 승인이 필요하다.** `DISARMED`에서만 받아들이며, 로봇 현장에서 아래 셋 중 하나로 한다.
  - 로봇 웹 UI의 **재개 승인**을 3초 안에 두 번 누른다(첫 누름은 확인 요청). 원격 중계·Control_Server 화면에는 버튼이 없다.
  - 조이스틱: LB를 뗀 채 스틱을 중앙에 두고 승인 버튼(`control_mux.yaml`의 `approve_button`, 기본 11 = Menu ≡)을 2초 누른다.
    누른 채로 E-stop을 지났거나 도중에 LB·스틱을 움직이면 떼었다가 다시 눌러야 한다. 버튼 번호는 실제 패드에서 `ros2 topic echo /joy`로 확인한다.
  - 아래 서비스.

  ```bash
  ROS_DOMAIN_ID=7 ros2 service call /estop/approve_resume std_srvs/srv/Trigger '{}'
  ROS_DOMAIN_ID=7 ros2 topic echo /estop/status --once   # state, stm32.trip_count 등 JSON
  ```

- 승인은 주행을 다시 시작하지 않는다. E-stop이 나면 `navigation_manager`가 목표를 끝내고(층간 미션은 `ESTOP`으로 중단),
  `control_mux`가 캐시된 AUTO 명령을 버리고 수동 gate를 닫는다. 승인 뒤에는 새 목표나 **LB를 새로 누른** 조이스틱 입력으로만 움직인다.
- `0x202`의 `sequence`가 뒤로 가거나 64 넘게 건너뛰면(STM32 재부팅, 상태 유실) E-stop과 같이 처리한다. IWDG 리셋이 0.5초 안에 끝나도 승인을 다시 받는다.
- 승인 뒤 5초 넘게 `ARMING`이면 상태 안내가 "STM32 주행 준비가 늦어짐"으로 바뀐다. ODrive 연결·오류를 확인한다.
- `READY`에서 STM32 gate가 닫히면(명령 공백 300 ms, ODrive fault, bus-off) E-stop이 아니므로 승인 없이 `ARMING`으로 돌아가 `(0, 0)`부터 다시 보낸다.
- E-stop 상태(`ESTOP`·`DISARMED`·`UNKNOWN`)에서는 종료 시 정지 프레임도 보내지 않는다. 0 명령이 STM32 gate를 열기 때문이다.
- `0x202`를 보내지 않는 구 펌웨어에서는 `can_bridge.yaml`의 `require_estop_status: false`로 기존 동작을 쓴다. 이때는 E-stop 뒤 재가동을 브리지가 막지 못한다.
- 상태 프레임은 `candump can0,202:7FF`로 관찰한다. 래치 중 IMU(`0x301`/`0x302`)는 끊기고, 해제 뒤 `seq`가 건너뛰는 것은 정상이다(진단 `imu_seq_gaps`).
  EKF는 yaw rate를 IMU에서만 받으므로, 회전 중 E-stop이면 래치 동안 odom yaw가 흐를 수 있다(실측 전). 해제 뒤 IMU가 돌아와도
  `time_health`가 1초 동안 회복을 기다려 그사이 목표가 거절될 수 있다.

## 6. CAN 수신 (`can_telemetry`)

수신 노드는 CAN을 송신하지 않는다. 설정은 `can_telemetry.yaml`이며 송수신 `channel`을 바꾸면 두 YAML을 함께 맞춘다.

| 토픽 | 메시지 | 실측 주기 | 내용 |
|---|---|---|---|
| `/wheel/odometry` | `nav_msgs/Odometry` | 약 100 Hz | 엔코더 위치 차분 평면 위치, 측정 속도 |
| `/joint_states` | `sensor_msgs/JointState` | 약 100 Hz | 좌우 구동 바퀴, rad·rad/s |
| `/imu/data_raw` | `sensor_msgs/Imu` | 약 10 Hz | 가속도 m/s², 각속도 rad/s (sensor-data QoS) |
| `/diagnostics` | `diagnostic_msgs/DiagnosticArray` | 1 Hz | 수신 신선도, 폐기·재기준 횟수 |

### 프로토콜과 좌표

- ODrive node 0 `0x009`는 **오른쪽**, node 1 `0x029`는 **왼쪽**. `<ff>` 위치[turn]·속도[turn/s], 전진 기준 오른쪽 +1·왼쪽 -1.
  STM32의 과거 LEFT/RIGHT 매크로 이름과 실제 장착 위치를 혼동하지 않는다.
- ODrive 보고값을 바퀴 회전수로 보는 비율(`encoder_turns_per_wheel_turn`)은 1이다. 감속비나 출력축 매핑이 다르면 맞춰야 한다.
- `0x301` 가속도, `0x302` 각속도는 `<hhhH>`(XYZ int16 + 공통 uint16 시퀀스). 같은 시퀀스끼리만 결합하고 wraparound를 처리한다.
- STM32는 IMU 범위 레지스터 CTRL6/CTRL8을 설정하거나 보내지 않는다. 전원 초기값 ±2 g(0.061 mg/LSB), ±125 dps(4.375 mdps/LSB)를 가정한다.
  센서 범위를 바꾸면 YAML도 맞춘다. 정지 시 +Z 약 9.71 m/s²를 관측했다.
- IMU 축은 +X 전방, +Y 좌측, +Z 위로 `imu_link`에 그대로 표현한다. 중력은 제거하지 않는다. 자세값이 없으므로 `orientation_covariance[0] = -1`.
- 측정 시각은 SocketCAN 커널 수신 시각이다. `use_sim_time=false`가 필요하다.

근거: [ODrive CAN Encoder Estimates](https://docs.odriverobotics.com/v/latest/manual/can-protocol.html#get-encoder-estimates),
[ST LSM6DSV16X 데이터시트](https://www.st.com/resource/en/datasheet/lsm6dsv16x.pdf) Table 3·CTRL6/CTRL8.

### 데이터 누락

- 양쪽 휠이 새로 들어오고 시각 차이가 20 ms 이내일 때만 결합한다. 위치는 엔코더 위치 차분으로 적분하고 최초 위치는 (0,0,0)이다.
- 250 ms 이상 끊기거나 비현실적 점프가 있으면 그 구간을 적분하지 않고 기준을 다시 잡는다. 위치 공분산을 키우고 진단 WARN을 유지한다.
- 오래된 프레임·잘못된 DLC·NaN/Inf·RTR·확장 프레임은 쓰지 않는다. 수신이 끊기면 해당 토픽 발행을 멈추고 `/diagnostics`로 알린다.
  진단은 정지 명령을 보내지 않는다.
- `/wheel/odometry`는 `odom` → `base_footprint`이지만 **TF는 발행하지 않는다**. 그 TF는 EKF만 맡는다.
- 공분산은 보수적인 초기 추정치다. IMU 자동 영점 보정은 하지 않고, 정지 측정으로 구한 `gyro_bias`를 시작 시 읽는다.
  전원·온도 조건이 달라지면 정지 편향을 다시 확인한다.

## 7. 로봇 모델과 TF

고정 TF 발행자는 `robot_state_publisher` 하나다.

```text
base_footprint
└── base_link                      z = 0.086 m
    ├── imu_link                   xyz = (0.097, 0, 0.021) m  (최신 CAD, 2026-10-23)
    ├── lidar_mount_link           z = -0.029 m
    │   └── lidar_link             z = 0.137605 m
    │       └── laser              yaw = pi
    └── 바퀴·캐스터 링크            v28 관절 구조 유지(v80 윤거 0.529 m)
```

- `base_link`는 구동축 중앙, +X 전방·+Y 좌측·+Z 위. `base_link → laser`는 `(0, 0, 0.108605)` m, yaw = pi.
- `/scan.header.frame_id=laser`, `inverted=false`를 유지한다. S2 런치 인자 `serial_port`, `serial_baudrate`, `scan_mode`로 드라이버를 바꿀 수 있다.
- v80 차체의 2020 기둥 4개가 스캔 평면을 지나 약 ±22~30°, ±149~157° 방향 0.27~0.28 m에 보인다. 드라이버는 `/scan_raw`로 내보내고
  `scan_body_filter`가 `scan_body_filter.yaml`의 `base_link` 상자(기둥 외곽 + 0.03 m) 안에 떨어지는 점을 NaN으로 바꿔 `/scan`을 낸다.
  상자는 Nav2 footprint 안쪽이다. 기둥 모서리를 스친 빔은 기둥보다 10~20 cm 먼 0.36~0.50 m에 중간 거리 점을 내고(최대 2.9°
  바깥), footprint 밖이라 상자로는 지워지지 않아 코스트맵에 로봇에 붙은 장애물로 남았다(2026-10-23, DWB "No valid trajectories").
  그래서 각 기둥이 가리는 방향 ± `shadow_margin_deg`(3°) 안에서 `shadow_range`(0.7 m) 이내 점도 지운다. 기둥 너머는 원래 보이지 않으므로
  잃는 영역은 여유 각도뿐이다(0.7 m에서 폭 약 3.7 cm). 실측: 로봇 밖 0.7 m 이내 점이 스캔당 32.6개에서 0개, 추가로 지운 점은 유효 점의 1.3 %.
  `laser` TF를 받기 전에는 스캔을 내보내지 않는다. `start_lidar:=false`이면 필터도 뜨지 않는다. `record_trial.sh`는 두 토픽을 모두 기록한다.
- URDF는 `src/amr_description/urdf/capstone_amr.urdf.xacro`와 확장본 `.urdf`. 런치는 확장본을 읽어 mesh 경로를
  `package://amr_description/meshes/visual/`로 바꿔 `/robot_description`으로 제공한다. Xacro 수정 시 확장본도 재생성한다.
- 구동 바퀴 TF는 CAN 측정 `/joint_states`로 갱신된다. 캐스터는 측정값이 없어 발행하지 않는다.
- 바퀴 중심 간격은 URDF·상위 제한·STM32 소스 모두 0.529 m다(v80 CAD).

## 8. EKF

```text
/wheel/odometry ─ vx, vy=0 제약 ─┐
                               ├─ ekf_filter_node ─ /odometry/filtered (30 Hz) + odom → base_footprint
/imu/data_raw ─── gyro Z ───────┘
```

- `robot_localization/ekf_node`, 설정 `ekf.yaml`. `two_d_mode=true`, `world_frame=odom`, `base_link_frame=base_footprint`.
- 휠은 측정 전진 속도만 쓰고 차동구동 횡속도 0 제약(공분산 0.01)을 더한다. 휠 적분 위치·yaw rate는 융합하지 않는다
  (회전 시험에서 슬립과 휠·IMU 불일치 관찰). 회전은 보정된 IMU gyro Z를 쓴다. IMU 자세·가속도는 사용하지 않는다.
- 별도 `/odom` 토픽은 없고 Nav2는 `/odometry/filtered`를 쓴다. EKF 재시작 시 원점이 초기화된다.
- 절대 방향 관측이 없어 gyro 바이어스에 따른 yaw 드리프트와 슬립에 따른 거리 오차가 남는다.
  입력이 끊겨도 예측을 계속하므로 출력 Hz만으로 센서 건강을 판단하지 말고 CAN `/diagnostics`의 freshness를 함께 본다.
- 근거: [robot_localization 센서 구성 지침](https://github.com/cra-ros-pkg/robot_localization/blob/humble-devel/doc/configuring_robot_localization.rst).

## 9. 지도·위치 추정·내비게이션

### 지도와 AMCL

- 실차 기본 지도는 `src/amr_system/amr_stack/maps/G_4_floor/4_floor_0522.yaml`(0.05 m, 1014×462셀, 원점 (-3.8, -1.54, 0)).
  원본 YAML/PGM은 수정하지 않는다. 층별 지도는 `floors.yaml`에서 관리한다([층간 이동](ELEVATOR.md)).
- `localization.launch.py` = map_server + amcl + lifecycle_manager. AMCL이 `map → odom`을 발행하고 `/amcl_pose`를 낸다.
  `/map`은 transient-local이다. SLAM과 AMCL은 동시에 `map → odom`을 발행하면 안 되므로 SLAM을 먼저 종료한다.
- AMCL은 `set_initial_pose=false`, `always_reset_initial_pose=true`로 초기 위치 입력을 기다린다. 초기 위치 검증·자동 탐색·빔 수 전환은
  [위치 추정](LOCALIZATION.md)에 있다.

### Nav2

```text
웹 UI / RViz /navigate_to_pose 또는 /goal_pose
  → navigation_manager (목표 보관·재개, 명령 허용)
  → /navigation/navigate_to_pose (내부 BT navigator)
  → Smac 2D → RotationShim(DWB) / 복구
  → /cmd_vel_navigation → navigation_manager → /cmd_vel_auto → control_mux → CAN
```

| 상황 | 동작 |
|---|---|
| MANUAL(LB 끼어들기)에서 목표 지정 | 거절. 웹 UI·중앙 관제는 409와 안내 문구 |
| AUTO에서 목표 지정 | 경로 계산 후 주행 |
| 주행 중 LB 누름(MANUAL) | mux 정지, 내부 Nav2 작업 취소, 목표 중단(`OVERRIDDEN`, 결과 ABORTED) |
| LB 해제로 AUTO 복귀 | 아무 목표도 재개하지 않음. 새 목표가 와야 주행 |
| 주행 중 조이스틱 연결·끊김 | 영향 없음 |
| 새 목표 | 이전 목표 종료 후 교체 |
| 취소 | 실행·보관 목표 모두 삭제 |
| 도착 또는 최종 실패 / navigation 재시작 | 목표 종료, 자동 재실행·복원하지 않음 |

- `/navigation/status`(transient-local): `IDLE`, `WAITING_NAV2`, `RUNNING`, `STOPPING`, `MANEUVER`, `MANUAL`(LB 끼어들기 중, 목표 없음),
  `SUCCEEDED`, `FAILED`, `CANCELED`, `OVERRIDDEN`(LB 끼어들기로 중단). 명령 전달은 20 Hz, 마지막 명령이 0.20초 지연되면 0을 전달한다.
- 목표 교체·LB 해제 직후 새 목표에서도 이전 내부 액션의 실제 terminal result를 기다린 뒤 다음을 실행한다. Nav2 핵심 프로세스가 죽으면 navigation launch도 종료한다.
- MANUAL에서는 엘리베이터 기동 임대(`/navigation/maneuver_lease`)도 거절한다.
- 취소: `ros2 service call /navigate_to_pose/_action/cancel_goal action_msgs/srv/CancelGoal "{}"`.
- `scripts/auto_turn_test.py` 등 다른 `/cmd_vel_auto` 발행 프로그램은 navigation과 동시에 실행하지 않는다.

주요 설정 (`navigation.yaml`, 실차·시뮬레이션 공용):

| 항목 | 값 |
|---|---|
| 자율 속도 상한 | 0.40 m/s, 0.60 rad/s. 정상 추종은 전진·제자리 회전, 후진은 복구에서만 |
| 바퀴 제한 | `amr_nav2_plugins::WheelSpeedCritic`이 상한을 넘는 후보 제외. 치수는 launch가 `control_mux.yaml`에서 읽음 (`control_config:=`로 변경 시 양쪽에 전달) |
| 지역 제어 | RotationShim(새 목표에서만, 방향 차 0.60 rad 초과 시 0.40 rad/s로 정렬(open loop), 0.20 rad 이내에서 넘김, 참조점 1.00 m) + DWB 20 Hz, 예측 1.5초, 선속 12·각속 24 샘플 |
| DWB 가감속 | 선형 ±0.50 m/s², 회전 ±1.60 rad/s² (ODrive 실측값 아님), shim ±0.80 rad/s² |
| DWB critic | `BaseObstacle 0.12`, `PathAlign 32`(전방 0.50 m), `GoalAlign 8`, `PathDist 32`, `GoalDist 32`, `PreferForward`(미세 명령 비용 0.5) |
| costmap | 전역 `map` 0.05 m 2 Hz, 반경 0.44 m / 지역 `odom` 6×6 m 10 Hz, 0.656×0.574 m 직사각형 + padding 0.02 m |
| 장애물 | `/scan` 반영 5 m, ray tracing 6 m, inflation 1.60 m·감소 계수 2.0, LiDAR 0.5초 지연 시 오래된 것으로 처리 |
| 전역 계획 | Smac 2D, 미확인 공간 통과 안 함, smoother(`w_smooth 0.95`, `w_data 0.02`), 2 Hz 유효성 확인·10초 만료 재계획 |
| 도착 | 위치 0.15 m·방향 0.15 rad + `StoppedGoalChecker` |
| 복구 | costmap 갱신, 90° 회전(0.40 rad/s), 5초 대기, 0.30 m 후진(0.05 m/s), 6회 소진 시 실패 |

- 전역 계획 반경 0.44 m 때문에 약 0.88 m 미만 통로에는 경로가 나오지 않는다. 엘리베이터 출입구는 전용 기동 제어기가 맡는다.
- `/evaluation` 후보 궤적 발행은 기본 비활성이다(대용량, 제어 주기 영향).
- 단일 목표 주행만 제공한다. 다중 경유점·순찰·도킹은 없다 (UI/UX 백로그는 GitHub Issues).

### RViz

- `rviz2 -d src/amr_system/amr_stack/rviz/navigation.rviz`. Fixed Frame `map`, **2D Pose Estimate** = `/initialpose`, **2D Goal Pose** = `/goal_pose`.
  목표는 navigation_manager를 거쳐 LB 끼어들기·취소·속도 제한을 똑같이 적용받는다.
- 기본 화면은 `amr_rviz_plugins/SingleThreadedTF`로 TF 수신·표시를 한 스레드에서 처리한다(TF 메시지 필터 교착 회피). 화면을 저장할 때도
  `Transformation -> Current` 선택을 유지한다. 플러그인이 설치되지 않았으면 navigation launch가 빌드 안내와 함께 중단한다.
- Navigation 2 패널·GoalTool·ParticleCloud는 기본 화면에서 뺐다. 15 fps, CAD 모델·전역 costmap·경로 화살표는 기본으로 끈다.
- 초록 **Robot Footprint**는 충돌 검사 외곽(0.656×0.574 m + 0.02 m), 장애물 주변 색 띠는 inflation 비용(1.60 m 전체가 통행 금지는 아님).

## 9a. CPU 부하 (Jetson Orin Nano, 2026-10-23)

- 실차 스택 전체를 올리면 부하 평균이 코어 6개에 12~13까지 올라 `odom → base_footprint` TF가 평균 0.3초, 최대 3초 늦게 도착했다.
  그러면 `navigation_startup`이 최신 스캔을 변환하지 못해 READY를 잠시 풀고 `navigation_manager`가 목표를 중단했다
  (`Localization readiness lost`). 컨트롤러 주기도 20 Hz에서 9~15 Hz로 떨어졌다.
- 코어 1개 기준 CPU: `vehicle_status` 67 %, `navigation_startup` 58 %, `web_ui` 42 %, `can_telemetry` 33 %, `control_mux`·`time_health`·
  `elevator_monitor`·`floor_mission_manager` 각 25 %. 모두 Python이며 대부분 `/tf`(57 msg/s)를 구독한다.
- `vehicle_status`는 웹 텔레메트리 카드만 쓰므로 bringup 기본값을 `start_vehicle_status:=false`로 바꿨다. 필요하면 인자로 켠다.
- 주행 중 Chromium은 웹 UI 탭 하나만 둔다. 다른 VS Code 서버·Codex·cpptools를 띄워 두면 RAM 100 MB·스왑 0까지 떨어진 적이 있다.
- 주행 4회에서 멈춤 13회가 모두 `navigation_startup`의 "odom·base·LiDAR 좌표 변환을 기다립니다"였고, 그때 이 노드는 CPU 92 %였다
  (DDS 지연은 정상). 파이썬 노드라 TF 수신 스레드가 자기 연산에 밀린 것이다. py-spy(`sudo sysctl kernel.yama.ptrace_scope=0` 후
  `py-spy record -p <pid> -d 15 -f raw`)로 잰 비중: 빔별 TF 조회 360회/틱 27 %, 입자 2,000개 역직렬화 9 %, `valid_scan` 파이썬 루프 6 %,
  10 Hz 틱마다 같은 스캔 재채점. 조치: TF는 10 ms 간격 지점만 조회해 보간, 입자 구름은 초기 검증 때만 역직렬화, 스캔 검사 numpy,
  READY에서는 새 스캔·AMCL 추정이 왔을 때만 채점, 보정 전 점수는 정렬 실패 때만 계산. 재시작 직후 측정 42 %(채점 전 두 항목 적용 전).
- `web_ui` 50~75 %: `/api/state`마다 상태 deepcopy(30 %), 스캔 3,240점 파이썬 변환(13 %), 코스트맵 갱신마다 PNG 인코딩(9 %).
  얕은 복사, numpy 변환, 요청 시 인코딩으로 바꿨다.
- `can_telemetry` 30~40 %: 5 ms 타이머로 초당 200번 실행기를 깨워 CAN 소켓을 비웠다(프레임은 초당 약 60개). 이제 수신 스레드가
  커널 `recv`에서 블록하다가 프레임이 올 때만 깨고, 실행기는 1 Hz 진단 타이머만 돌린다. 메시지 내용·타임스탬프는 그대로다.

## 10. 시간 이상 처리

- bringup은 `time_health`를 실행한다. `/scan`, `/wheel/odometry`, `/imu/data_raw`, `/odometry/filtered`, `odom → base_footprint` TF의 시각과 신선도를 본다.
  허용 지연 0.75초, 미래 시각 0.1초, 정상 유지 1초. AMCL의 미래 시각 `map → odom`은 검사하지 않는다.
- `/system/time_health`가 비정상이거나 heartbeat가 0.6초 끊기면 control_mux는 AUTO 출력을 0으로 막고, navigation_manager는 목표를 중단하며,
  웹 UI는 새 목표를 막는다. MANUAL의 LB 규칙은 그대로다.
- 실물에서는 ROS/시스템 시간 증가량과 monotonic 증가량 차이가 0.5초를 넘으면 오류를 유지(latch)한다. CAN 수신기는 이전 시간·샘플 조합만 비우고
  누적 휠 위치는 유지하며, 진단 `clock_resets`·`wheel_rebases`로 남긴다.
- 시간이 안정되고 센서·TF가 정상이며 정지 상태일 때만 해제할 수 있다. 해제 후 위치를 다시 검증해야 하고 이전 목표는 재개되지 않는다.

```bash
ROS_DOMAIN_ID=7 ros2 service call /system/reset_time_fault std_srvs/srv/Trigger '{}'
```

- 웹 UI의 TF 버퍼는 노드 시계에 연결되어, 시뮬레이션 시각 역행이나 감독기 실행 UUID 변경 시 동적 표시와 요청을 버리고 TF 구독을 다시 만든다.
- Jetson 한 대에서 모든 ROS 노드를 돌리면 같은 OS 시계를 쓴다. 여러 컴퓨터에 노드를 나누면 chrony/NTP 동기화를 구성한다.
  큰 보정은 스택 시작 전에 끝내고 주행 중에는 시간 서비스를 재시작하지 않는다.

```bash
chronyc tracking
chronyc waitsync 30 0.05 0 1
AMR_REQUIRE_TIME_SYNC=1 ./scripts/experiment.sh real   # chrony 점검 실패 시 실행하지 않음
```

근거: [ROS Clock and Time](https://design.ros2.org/articles/clock_and_time.html), [chrony makestep](https://chrony-project.org/doc/4.6/chrony.conf.html#makestep).
프로그램 방식의 정지는 물리 비상정지를 대체하지 않는다.

## 11. 구성 요소별 실행

점검·디버깅할 때 쓴다. 각 터미널에서 `source scripts/env.sh`(또는 `amr`)를 먼저 실행하고, 이미 실행 중인 항목은 중복 실행하지 않는다.

| bringup 옵션 | 기본값 | 기능 |
|---|---|---|
| `start_lidar` | `true` | S2 드라이버 + 차체 스캔 필터 |
| `use_rviz` | `false` | RViz |
| `start_joy` | `true` | 조이스틱 드라이버 |
| `start_can_bridge` | `true` | CAN 송신 브리지 |
| `start_can_telemetry` | `true` | 휠·IMU CAN 수신 |
| `start_ekf` | `true` | EKF와 `odom → base_footprint` |
| `can_dry_run` | `false` | `true`이면 CAN 송신 없이 미리보기 |
| `start_elevator_link` / `elevator_link_dry_run` | `false` / `true` | 층간 이동 CAN 중계 |

```bash
# 차량·센서·EKF
ros2 launch amr_stack bringup.launch.py
# CAN 송신 없이 실제 LiDAR·센서·TF·EKF 확인
ros2 launch amr_stack bringup.launch.py can_dry_run:=true start_joy:=false
# CAN·LiDAR 없이 mux 토픽만 확인
ros2 launch amr_stack bringup.launch.py can_dry_run:=true start_lidar:=false start_can_telemetry:=false
# LiDAR와 로봇 TF만 (bringup과 동시에 실행하지 않음)
ros2 launch amr_stack lidar.launch.py
# 지도·AMCL, 목표점 주행
ros2 launch amr_stack localization.launch.py [map:=/absolute/path/map.yaml]
ros2 launch amr_stack navigation.launch.py [use_rviz:=true]
```

수신 노드만, 송신 브리지만 따로 실행할 때:

```bash
ros2 run amr_can_bridge can_telemetry --ros-args --params-file \
  install/amr_stack/share/amr_stack/config/can_telemetry.yaml
ros2 launch amr_stack bringup.launch.py start_can_bridge:=false
ros2 run amr_can_bridge cmd_vel_to_can [--ros-args -p dry_run:=true]
```

### 자율 모드 회전 시험

`scripts/auto_turn_test.py`는 MANUAL → AUTO 전환(LB를 눌렀다 놓고 0.5초)을 본 뒤 우회전 2초(-0.20 rad/s) → 좌회전 2초(+0.20 rad/s)를 20 Hz로 반복하고,
그 밖에는 0을 발행한다. 스크립트 시작 시 이미 AUTO이면 시작하지 않는다. LB를 누르면(MANUAL) 멈추고, 놓으면 우회전부터 다시 시작한다.

```bash
ros2 launch amr_stack bringup.launch.py can_dry_run:=true start_lidar:=false start_can_telemetry:=false
python3 scripts/auto_turn_test.py
ros2 topic echo /cmd_vel                              # 최종 명령
ros2 topic echo /cmd_vel_to_can/tx_preview             # CAN 변환 미리보기
ros2 topic echo /control_mux/mode --qos-durability transient_local
```

확인: 회전 도중 컨트롤러를 켜거나 꺼도 회전은 계속되고, LB를 누르면 즉시 0. 스크립트를 종료하면 0.

## 12. 주행 기록

주행 스택을 실행한 상태에서 별도 터미널에서 실행한다.

```bash
./scripts/record_trial.sh [실험이름]
```

- 한국 시간 기준 `bags/YYYY-MM-DD/HHMMSS[_실험이름]/`에 새 폴더를 만들고 기존 기록을 지우지 않는다.
- `config_snapshot/`(프로젝트 설정), `params/`(주요 노드 실행 파라미터, 노드당 최대 15초, 실패는 `params/collection.log`),
  `bag/`(전체·숨김 토픽 SQLite3), `session.txt`(시각과 ROS 환경)를 저장한다.
- 차량을 정지시킨 뒤 Ctrl+C로 끝낸다. 정상 종료 시 `bag/metadata.yaml`과 `bag_info.txt`가 생성된다.
  서비스 요청·응답과 액션 목표 요청 전체를 저장하는 방식은 아니다.
- 시뮬레이션에서는 `AMR_EXPERIMENT_MODE=sim ./scripts/record_trial.sh 이름`을 쓴다([시뮬레이션](SIMULATION.md#기록과-비교)).
