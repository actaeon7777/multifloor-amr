# AMR workspace — 주행 제어·LiDAR·로봇 TF

ROS 2 Jazzy용 차동구동 AMR 작업공간입니다. F1TENTH와 같은 역할 구분을 유지합니다.

JetPack 7.2.1 / Ubuntu 24.04 / arm64 포팅 내용과 검증 결과는
[PORTING_JAZZY.md](PORTING_JAZZY.md)에 기록합니다. 기존 압축본은 `/home/acus03/2204_amr`에 보존합니다.

## 현재 구현 범위

- `amr_web_ui`: 웹 지도 터치·드래그로 목표점과 도착 방향을 지정합니다. 기존 목표 관리기에 연결하며
  지도·AMCL·스캔·경로·costmap·footprint를 표시합니다. [웹 UI 실행과 미리보기](src/amr_system/amr_web_ui/README.md).
- `amr_stack`: 조이스틱, `joy_teleop`, `control_mux`, CAN 브리지, S2 LiDAR, `robot_state_publisher`, `robot_localization` EKF를 함께 실행합니다. RViz는 기본으로 꺼집니다.
- `amr_description`: v28 URDF의 차체·장착 TF와 실물 `laser`, `imu_link` 프레임을 제공합니다.
- `teleop_tools/joy_teleop`, `teleop_tools/teleop_tools_msgs`: 기존 F1 소스에서 복사한
  입력 변환 패키지입니다. 출처는 `src/amr_system/teleop_tools/AMR_SOURCE.md`에 기록합니다.
  F1 설치 환경을 source하지 않아도 AMR 작업공간에서 빌드·실행할 수 있습니다.
- `amr_control`: LB(6) 데드맨 수동 조작과 RB(7) 토글 자율 모드의 `control_mux`입니다.
  자율 모드에서는 수동 입력을 무시하고, 자율 OFF 전환 시 정지합니다.
- `amr_can_bridge`: `/cmd_vel`을 STM32용 CAN 프레임으로 변환하는 `cmd_vel_to_can`을 구현했습니다.
  `can_telemetry`는 실제 CAN 피드백을 수신해 휠 오도메트리, 구동 바퀴 관절 상태, IMU 측정값을 발행합니다.
- `can_bridge.yaml`: 성공했던 키보드 송신 코드와 같은 프로토콜 및 타임아웃을 설정합니다.
- AMR 자체 패키지 라이선스는 정책 결정 전까지 `TODO`이며, 복사한 패키지는 원래 라이선스를 유지합니다.

현재 경로:

```text
joy_node → /joy → joy_teleop → /cmd_vel_manual → control_mux → /cmd_vel
                   /joy ────────────────────────↑               ↓
자율주행 노드 ───────────────→ /cmd_vel_auto ──────┘               ↓
                                                       cmd_vel_to_can
                                                        ↓ (실송신 모드)
                                                      CAN → STM32
```

기본 bringup은 실제 CAN 송신까지 실행하므로 연결된 STM32를 통해 바퀴가 구동될 수 있습니다.
`can_dry_run:=true`는 송신 브리지의 CAN 소켓을 열지 않고 미리보기만 출력합니다.
수신 노드는 별도로 실제 CAN을 읽습니다. CAN 장치 없는 시험에는 `start_can_telemetry:=false`도 지정합니다.
기본 bringup은 S2 LiDAR와 URDF 고정 TF도 실행합니다. `stack` 한 번으로 실행하며 RViz는 `use_rviz:=true`일 때만 켜집니다.
지도·AMCL은 `localization.launch.py`, Smac 2D·DWB 목표점 주행은 `navigation.launch.py`로 별도 실행합니다.

## 조이스틱 매핑

기존 환경에서 Xbox Wireless Controller의 `/joy`로 확인한 0부터 시작하는 인덱스입니다.
새 환경에서도 같은 매핑을 유지하며, 실제 컨트롤러 재연결 검증은 별도로 수행합니다.
`joy.device_name`은 현재 SDL에서 확인한 `Xbox Series X Controller`로 지정하여 다른 입력 장치를 선택하지 않습니다.
USB 등 연결 방식에 따라 이름이 다르면 `ros2 run joy joy_enumerate_devices`로 SDL 이름을
확인하고 `joy_teleop.yaml`의 `device_name`을 그 이름으로 변경합니다.

| 기능 | 입력 | 출력 |
|---|---|---|
| LB 데드맨 | `buttons[6]` | 자율 OFF에서 누르는 동안만 수동 명령 허용 |
| RB 토글 | `buttons[7]` | 한 번 누르면 자율 ON, 다시 누르면 OFF + 정지 |
| 전진·후진 | `axes[1]` | `linear.x`, 최대 ±1.00 m/s, 전진 양수 |
| 좌회전·우회전 | `axes[2]` | `angular.z`, 최대 ±0.80 rad/s, 좌회전 양수 |

`axes[0]`과 `axes[3]`은 사용하지 않습니다. 스틱 데드존은 0.05, `/joy` 반복률은 20 Hz입니다.
설정은 `src/amr_system/amr_stack/config/joy_teleop.yaml`에 있습니다.
속도 상한을 바꾸려면 `control_mux.yaml`과 `can_bridge.yaml`의 제한도 함께 검토해야 합니다.
설정 변경은 bringup 재시작 후 적용됩니다. YAML 없이 노드를 단독 실행하면 코드 기본 상한인 0.20 m/s, 0.40 rad/s를 사용합니다.
조이스틱 선속도 요청 상한은 1.00 m/s이며, 최종 `/cmd_vel`에는 아래의 상위 바퀴 속도 제한이 적용됩니다.

### 상위 바퀴 속도 제한과 하위 변환

- `control_mux`가 수동·자율 모드를 선택한 뒤 바퀴 속도를 제한합니다. 설정은 `control_mux.yaml`의
  `wheel_diameter`(0.172 m), `track_width`(0.499 m), `max_wheel_rev_s`(1.8 rev/s)입니다.
- 어느 바퀴든 상한을 넘으면 선속도와 각속도에 같은 축소 비율을 적용하여 요청한 회전 반경을 유지합니다.
  후진·좌우 회전에도 동일하게 적용하며, 제자리 회전도 바퀴 상한 안으로 제한합니다.
- 바퀴 선속도 상한 `U = π × wheel_diameter × max_wheel_rev_s`와
  필요한 최대 바퀴 속도 `M = |v| + |ω| × track_width / 2`를 계산합니다.
  `M > U`이면 `(v, ω)` 모두에 `U / M`을 곱하고, 그 외에는 그대로 전달합니다.
  기본값에서 직진 최대 출력은 약 0.973 m/s이며, `(v=1.0, ω=0.8)` 요청은 약 `(0.811, 0.649)`로 전달됩니다.
  스틱의 원래 요청은 `/cmd_vel_manual`, 제한 후 값은 `/cmd_vel`과 CAN 미리보기에서 확인합니다.
- `stm32_main_code.c`는 받은 `(v, ω)`에 역기구학과 모터 장착 방향만 적용합니다.
  바퀴별 클리핑과 선속도·각속도·좌우 바퀴의 별도 gain은 제거했습니다. STM32의 통신 타임아웃과 모터 오류 정지는 유지합니다.
- STM32의 `drive_wheel_diameter_m`·`drive_track_width_m`는 상위 설정과 일치해야 하며,
  `drive_node0_direction`·`drive_node1_direction`은 방향 부호(±1)로 사용합니다.
  바퀴 속도 상한은 이제 상위 `max_wheel_rev_s`에서 변경합니다. 다른 CAN 송신 경로를 사용하면 그 경로에서도 상한을 적용해야 합니다.

상위 변경은 bringup 재시작 후 적용됩니다. 하위 변경은 수정한 `stm32_main_code.c`를 STM32 프로젝트의
`Core/Src/main.c`에 반영하여 빌드·업로드해야 적용됩니다. `stm32_code.zip`은 기존 원본으로 보존했습니다.
일반·고속 모드는 구분하지 않고 선속도 요청 상한 1.00 m/s, 각속도 요청 상한 0.80 rad/s를 사용합니다.
상위에는 가감속 제한을 추가하지 않으며 가감속은 ODrive의 기존 `VEL_RAMP` 설정에서 처리합니다.
ODrive 내부 속도 제한과 램프 설정은 이 변경으로 바뀌지 않습니다.

## 빌드와 실행

ROS Jazzy를 source한 셸에서 빌드합니다. Jetson 메모리를 고려해 병렬 작업 수를 2로 제한합니다.

```bash
cd /home/acus03/amr_ws
source /opt/ros/jazzy/setup.bash
MAKEFLAGS=-j2 CMAKE_BUILD_PARALLEL_LEVEL=2 colcon build --symlink-install \
  --parallel-workers 2 --cmake-args -DCMAKE_BUILD_TYPE=Release
```

기존 bringup을 Ctrl+C로 종료한 뒤 다시 실행합니다. 각 터미널에서 `amr`을 실행하면
작업공간으로 이동하고 `scripts/env.sh`를 source하여 Jazzy와 AMR 패키지를 로딩합니다.
기본 ROS 도메인은 7이며 이미 지정한 `ROS_DOMAIN_ID`는 유지합니다.
같은 ROS 도메인에서 다른 주행 노드를 함께 실행하지 마세요.

터미널 1 (`can0`를 500 kbit/s로 설정하고 UP 상태로 준비한 뒤 실행):

```bash
amr
ros2 launch amr_stack bringup.launch.py
```

터미널 2:

```bash
amr
ros2 topic echo /cmd_vel
```

필요하면 별도 터미널에서 `/joy`와 `/cmd_vel_manual`을 확인합니다.
`amr` 별칭이 없는 셸은 작업공간에서 `source scripts/env.sh`를 실행합니다.
`can0.service`는 부팅 시 500 kbit/s 일반 CAN으로 설정합니다.
현재 상태는 `systemctl status can0.service`와 `ip -details link show can0`로 확인합니다.

## 주행 실험 기록

주행 스택을 실행한 상태에서 별도 터미널에 아래 명령을 입력합니다. ROS 환경은 자동 로딩하므로
별도 source나 빌드 없이 사용할 수 있습니다.

```bash
cd /home/acus03/amr_ws
./scripts/record_trial.sh
```

선택적으로 실험 이름을 붙일 수 있습니다: `./scripts/record_trial.sh corner_test`.
매번 한국 시간 기준 `bags/YYYY-MM-DD/HHMMSS[_실험이름]/`에 새 폴더를 만들고,
같은 초에 재실행하면 숫자 접미사를 붙입니다. 기존 기록은 보존하며 자동 삭제하지 않습니다.

실험 폴더에는 `config_snapshot/`(프로젝트 설정), `params/`(주요 노드의 실행 파라미터),
`bag/`(전체 토픽과 숨김 토픽의 SQLite3 기록), `session.txt`(시각과 ROS 환경)가 저장됩니다.
파라미터 수집은 노드당 최대 15초이며, 미실행 또는 응답 실패는 `params/collection.log`에 남기고
기록을 계속합니다. 실행 파라미터는 기록 시작 직전의 값입니다.

차량을 정지시킨 뒤 기록 터미널에서 **Ctrl+C**를 누르고 스크립트 종료를 기다립니다.
정상 종료 시 `bag/metadata.yaml`이 자동 생성되고, `bag_info.txt`와 실험 폴더 경로가 출력됩니다.
분석할 때는 해당 실험 폴더 전체를 사용합니다. 이 스크립트의 기록은 토픽 메시지이며,
서비스 요청·응답 및 액션 목표 요청 전체를 저장하는 방식은 아닙니다.

## 실제 컨트롤러 확인 순서

1. 시작 후 LB를 한 번 놓습니다. LB를 누른 상태로 시작하면 출력은 0입니다.
2. LB를 누르지 않고 스틱을 움직여도 `/cmd_vel`이 0인지 확인합니다.
3. LB를 누른 채 왼쪽 스틱을 앞으로 밀면 `linear.x > 0`, 뒤로 밀면 `< 0`인지 확인합니다.
4. LB를 누른 채 오른쪽 스틱을 왼쪽으로 밀면 `angular.z > 0`, 오른쪽은 `< 0`인지 확인합니다.
5. 스틱을 중립으로 놓으면 출력이 0인지 확인합니다.
6. 자율 OFF에서 스틱을 기울인 상태로 LB를 0.10초 이상 놓으면 출력이 0이 되는지 확인합니다.
   짧은 해제 펄스는 채터링으로 무시합니다. RB는 별도의 자율 모드 전환 버튼입니다.
7. 컨트롤러 전원을 끄거나 Bluetooth 연결을 끊고 정지 및 재연결 동작을 확인합니다.
   `/joy`가 0.50초 이상 끊긴 뒤에는 LB를 놓았다 다시 눌러야 명령을 허용합니다.
   `/joy`는 정상이고 수동 속도 명령만 지연된 경우에는 새 명령으로 자동 복구됩니다.

`/cmd_vel_manual`은 LB 해제 시 발행을 중단합니다. 자율 ON에서는 이 토픽을 무시합니다. 마지막 메시지가 화면에 남아 있을 수 있으므로
정지 판단은 **최종 `/cmd_vel`**에서 합니다. 최종 출력은 정지 중에도 0을 반복 발행합니다.

## RB 자율 모드와 회전 테스트

모든 속도 토픽은 `geometry_msgs/msg/Twist`이며 `linear.x=v`, `angular.z=w`입니다.
자율주행 노드는 **`/cmd_vel_auto`**에 발행하고, **`/cmd_vel`은 mux만 발행**합니다.
기존 자율 노드가 `/cmd_vel`을 사용하면 그 노드에만
`--ros-args -r /cmd_vel:=/cmd_vel_auto`를 적용합니다. bringup 전체에 적용하지 않습니다.

- `joy.sticky_buttons`는 false입니다. 전체 버튼 토글 대신 mux에서 RB만 토글합니다.
- 시작은 MANUAL입니다. RB를 0.10초 이상 놓은 뒤 누르면 AUTO로 전환합니다.
  계속 누르거나 0.10초보다 짧게 놓았다 누르면 다시 토글되지 않습니다.
- AUTO에서는 LB/스틱/수동 명령을 무시합니다. 수동 입력 오류도 자율 출력을 덮어쓰지 않습니다.
- 모드 전환마다 0.10초 동안 0을 출력하고, AUTO 진입 전에 수신한 자율 명령은 버립니다.
- AUTO OFF 시 정지하며, LB가 이미 눌려 있어도 수동으로 즉시 움직이지 않습니다.
  이후 LB를 놓았다 다시 누르고 새 수동 명령이 와야 합니다.
- 자율 명령이 0.50초 만료되거나 잘못된 명령이면 0을 출력합니다. 수동으로 대체하지 않으며,
  AUTO 상태에서 새 유효 자율 명령이 오면 재개합니다.
- `/joy`가 0.50초 만료되거나 잘못된 입력이면 정지하고 MANUAL로 돌아갑니다.
  재연결 시 RB가 눌려 있어도 AUTO로 복귀하지 않습니다. 놓았다 다시 눌러야 합니다.
- bringup 설정에서 수동·자율 입력 상한은 선속도 ±1.00 m/s, 각속도 ±0.80 rad/s이며, 최종 출력에는 공통 바퀴 속도 제한도 적용합니다.
- `/control_mux/mode`는 `MANUAL`/`AUTO`를, `/control_mux/status`는 세부 상태를 발행합니다.
  두 토픽은 transient-local이므로 늦게 구독해도 마지막 상태를 받을 수 있습니다.

기존 bringup을 종료한 뒤, **각 터미널에서 `amr`을 먼저 실행**합니다.
토픽 검증에는 CAN을 송신하지 않는 다음 실행을 사용합니다.

```bash
# 터미널 1: bringup (CAN 연결/설정 불필요)
ros2 launch amr_stack bringup.launch.py can_dry_run:=true start_lidar:=false start_can_telemetry:=false

# 터미널 2: 테스트 자율주행 노드
python3 src/auto_turn_test.py

# 터미널 3: 자율 입력
ros2 topic echo /cmd_vel_auto

# 터미널 4: mux가 선택한 최종 주행 명령
ros2 topic echo /cmd_vel

# 선택: 별도 터미널에서 현재 모드
ros2 topic echo /control_mux/mode --qos-durability transient_local

# 선택: CAN 변환 결과 (송신하지 않는 미리보기)
ros2 topic echo /cmd_vel_to_can/tx_preview
```

`src/auto_turn_test.py`는 AUTO 상태에서 **우회전 2초 → 좌회전 2초**를 계속 반복합니다.
선속도는 항상 0, 각속도는 우회전 -0.20 rad/s, 좌회전 +0.20 rad/s입니다.
20 Hz로 발행하며, AUTO가 아니면 0을 계속 발행합니다. RB OFF는 반복을 중단하고,
다시 ON 하면 우회전부터 시작합니다. 스크립트를 AUTO 상태에서 시작해도 즉시 시작합니다.
시간은 스크립트의 AUTO 수신 시점 기준이며 mux 전환 직후에는 0.10초의 정지 구간이 있습니다.
수동 조작은 RB OFF 후 LB를 놓았다 다시 눌러 재개합니다.

확인 순서:

1. 시작 시 LB/RB를 놓고 `/cmd_vel`이 0인지 확인합니다.
2. RB를 누르면 `/cmd_vel_auto`에 `w=-0.20`, 2초 뒤 `+0.20`, 2초 뒤 다시 `-0.20`이 반복됩니다.
   `/cmd_vel`도 전환 정지 구간 이후 같은 명령을 출력해야 합니다.
3. AUTO 중 LB를 누르고 스틱을 움직여도 최종 출력은 자율 명령만 따라야 합니다.
4. 회전 도중 RB를 다시 누르면 최종 출력이 즉시 0이 되어야 합니다.
   LB가 이미 눌려 있어도 그대로 정지해야 합니다.
5. RB OFF/ON으로 우회전부터 다시 시작되는지 확인합니다.
6. 자율 명령 발행 중 스크립트를 종료하면 최종 출력이 0이어야 합니다.
   `/joy` 연결 단절 시에는 정지와 MANUAL 복귀를 확인합니다.

실제 CAN 시험 시에는 위 dry-run bringup을 종료하고 인자 없이 bringup을 실행합니다.
토픽의 0 명령은 정지 요청이며 실제 바퀴의 정지 시간은 하위 제어기에 달려 있습니다.

## 정지 처리와 한계

- MANUAL에서 LB 해제는 기본 0.10초 동안 확인한 뒤 정지합니다. 그보다 짧은 버튼 변화는 마지막 유효 명령을
  유지합니다. 확인 시간은 `release_debounce`이며 0.0으로 설정하면 즉시 해제 방식이 됩니다.
- 짧거나 잘못된 Joy 배열은 두 모드 모두 즉시 정지합니다. 속도 명령 검증은 선택된 입력에 적용합니다.
- `/joy`와 현재 모드의 속도 명령은 각각 0.50초 이상 새로 수신되지 않으면 정지합니다.
  해제 확인/타임아웃 정지에는 추가로 최대 한 타이머 주기(기본 0.05초)와 실행 지연이 걸릴 수 있습니다.
- 수동 조작은 시작, `/joy` 연결 만료, 잘못된 수동 입력 뒤에 유효한 LB 해제 입력과 새 누름을 요구합니다.
  다시 누른 뒤 수신한 새 수동 명령만 사용합니다.
- `/joy`와 LB가 계속 유효한데 수동 명령만 만료되면 출력만 0으로 일시 중단합니다.
  새 명령이 도착하면 LB 재입력 없이 복구됩니다. 만료된 명령 자체를 재사용하지 않습니다.
- 새 수동 명령은 즉시 `/cmd_vel`에 반영하며, 타이머는 기본 20 Hz로 출력을 갱신하고 정지 조건을 확인합니다.
- 시간 판정은 수신 시각의 monotonic clock을 사용하므로 ROS 시간 정지·변경의 영향을 받지 않습니다.
- 장치/드라이버가 연결이 끊긴 뒤에도 오래된 스틱 값을 계속 발행하면 이 타임아웃만으로는
  무선 손실을 감지할 수 없습니다. 실제 Bluetooth 전원 차단 시험이 필요합니다.
- `control_mux` 프로세스 자체가 종료되면 CAN 브리지가 `/cmd_vel` 수신 타임아웃으로 0을 보냅니다.
  브리지 종료(SIGINT/SIGTERM) 시에는 0 프레임을 5회 송신 시도합니다. 전원 차단, SIGKILL,
  CAN 단선/버스 오류에서는 이 송신이 보장되지 않으므로 STM32의 수신 타임아웃 정지를 확인해야 합니다.

정지 이유 확인:

```bash
amr
ros2 topic echo /control_mux/status --qos-durability transient_local
```

`active`: 주행 허용, `release_pending`: LB 해제 확인 중, `deadman_released`: LB 해제 정지,
`command_timeout`: 수동 명령 지연(자동 복구), `joy_timeout`: Joy 연결 만료(LB 재입력 필요),
`invalid_joy`/`invalid_command`: 잘못된 입력, `awaiting_release`: 시작 후 LB 해제 대기,
`waiting_command`: LB가 눌렸고 새 수동 명령 대기입니다.
자율 모드는 `auto_waiting_command`, `auto_active`, `auto_command_timeout`,
`auto_invalid_command`로 구분합니다. 만료/잘못된 입력은 로그에도 표시합니다.

## CAN 브리지 실행

성공했던 키보드 송신 코드의 형식을 그대로 사용합니다.

| 항목 | 값 |
|---|---|
| 채널 | `can0` (SocketCAN) |
| CAN ID | `0x201` / 513, 표준 11비트 데이터 프레임 |
| DLC | 8, CAN FD 아님 |
| byte 0–3 | `float32 v`, m/s, little endian |
| byte 4–7 | `float32 w`, rad/s, little endian |
| 송신 주기 | 20 Hz |
| 정지 데이터 | `0000000000000000` |
| 전진 + 좌회전 예시 | `v=0.2, w=0.4` → `201#CDCC4C3ECDCCCC3E` |

후속 제공된 STM32 설정에서 비트레이트는 500 kbit/s로 확인했습니다.
브리지는 네트워크 인터페이스 설정을 변경하지 않습니다.
실차에서는 `can0`가 500 kbit/s의 일반 CAN 모드로 이미 UP 상태여야 합니다.
`ip -details link show can0`로 확인합니다. [python-can SocketCAN 문서](https://python-can.readthedocs.io/en/stable/interfaces/socketcan.html)

실제 송신을 위한 시스템 의존성:

```bash
sudo apt-get install python3-can
```

먼저 CAN을 연결하지 않고 미리보기를 확인합니다. 기존 bringup은 종료한 뒤 실행합니다.

```bash
amr
ros2 launch amr_stack bringup.launch.py can_dry_run:=true start_lidar:=false start_can_telemetry:=false
```

다른 터미널에서:

```bash
amr
ros2 topic echo /cmd_vel_to_can/tx_preview
```

`DRY_RUN can0 201#... v=... w=... reason=...` 형태의 문자열을 확인할 수 있습니다.
MANUAL에서 LB를 놓거나 AUTO에서 RB로 OFF 전환하면 데이터가 8바이트 0으로 바뀌어야 합니다. 이 토픽은 미리보기/송신 요청 기록이며,
STM32 수신이나 바퀴 반응을 확인하는 피드백이 아닙니다.

실제 CAN 송신을 켜는 명령:

```bash
amr
ros2 launch amr_stack bringup.launch.py
```

기존 키보드 송신 프로그램이나 다른 CAN 브리지는 함께 실행하지 마세요.
최초 실차 검증은 바퀴를 띄운 상태에서 LB 해제 정지와 방향을 먼저 확인합니다.
실제 프레임은 `can-utils`가 설치되어 있으면 `candump can0,201:7FF`로 관찰할 수 있습니다.

이미 실행 중인 수동 bringup에 브리지만 별도로 붙일 때는 bringup의 브리지를 끕니다.

```bash
ros2 launch amr_stack bringup.launch.py start_can_bridge:=false
# 다른 터미널에서 (기본값은 실제 CAN 송신)
ros2 run amr_can_bridge cmd_vel_to_can
# 송신 없는 미리보기는 위 노드를 종료한 후 다음으로 실행
ros2 run amr_can_bridge cmd_vel_to_can --ros-args -p dry_run:=true
```

브리지 설정 파일은 `src/amr_system/amr_stack/config/can_bridge.yaml`입니다.
bringup의 `can_dry_run` 인자는 YAML의 `dry_run`보다 우선합니다. 설정은 재시작 시 적용합니다.
단독 실행 시 같은 YAML을 사용하려면 `--ros-args --params-file`로 해당 파일을 지정합니다.

브리지는 다음 조건으로 정지합니다.

- `/cmd_vel`이 0.50초 이상 만료되면 0을 보내며, 새 유효 명령이 오면 자동 복구합니다.
- 시작 시, 잘못된 입력 또는 CAN 송신 오류 뒤에는 새 0 명령을 받은 뒤에야 새 비영점 명령을 허용합니다.
  MANUAL에서는 LB를 놓았다 다시 누르고, AUTO에서는 RB OFF 후 다시 ON 합니다. STM32 코드의 300 ms 수신 타임아웃은 변경하지 않습니다.
- CAN 송신 오류 시 이전 속도를 버리고 0 프레임 송신만 재시도합니다. 오류가 해소되어
  0 프레임 쓰기가 성공한 후 새 0 명령을 받아야 다시 주행 명령을 허용합니다.
- 송신 대기는 최대 0.01초로 제한합니다. 타임아웃 판정은 monotonic clock을 사용합니다.
- 송신 모드에서 CAN 장치를 열 수 없으면 시작에 실패하며 dry_run으로 자동 전환하지 않습니다.
- 실제 송신 시작 전에 인터페이스의 UP 플래그를 확인합니다. DOWN이면 조치 방법을 출력하고 브리지를 종료합니다. `stack`은 CAN 설정이나 UP 전환을 수행하지 않습니다. UP 확인은 버스의 실제 송수신 성공을 보장하지 않습니다.

## 소프트웨어 검증

상태 전이와 입력 유효성 테스트:

```bash
cd /home/acus03/amr_ws
PYTHONPATH=src/amr_system/amr_control python3 -m pytest -q \
  src/amr_system/amr_control/test/test_manual_gate.py \
  src/amr_system/amr_control/test/test_control_gate.py \
  src/amr_system/amr_control/test/test_wheel_limits.py
```

`test_wheel_limits.py`는 호스트 GCC로 실제 STM32 변환 함수를 컴파일하여 상위 제한과 하위 변환을 함께 검증합니다.
보드 전체 빌드·펌웨어 업로드·실차 검증은 수행하지 않습니다.

빌드된 AMR 환경을 source한 터미널에서 ROS 파이프라인 테스트:

```bash
ROS_DOMAIN_ID=173 ROS_LOCALHOST_ONLY=1 ROS_LOG_DIR=/tmp/amr_manual_pipeline_logs \
AMR_RUN_ROS_TESTS=1 python3 -m pytest -q \
  src/amr_system/amr_control/test/test_manual_pipeline.py
```

이 테스트는 테스트 전용 도메인 173에서 `start_joy:=false can_dry_run:=true start_lidar:=false start_can_telemetry:=false`로 bringup을 실행하고
가상 `/joy`와 자율 입력을 발행하여 수동 회귀, RB 토글, 수동 차단, 타임아웃,
실제 `auto_turn_test.py`의 2초씩 회전 및 최종 CAN payload 미리보기까지 확인합니다. 도메인 173은 실제 로봇 운용에 사용하지 마세요.
실제 컨트롤러의 Bluetooth 연결 안정성을 검증하는 테스트는 아닙니다.

CAN payload·watchdog·송신 오류·종료 처리 테스트:

```bash
PYTHONPATH=src/amr_system/amr_can_bridge python3 -m pytest -q \
  src/amr_system/amr_can_bridge/test/test_drive_protocol.py \
  src/amr_system/amr_can_bridge/test/test_can_sender.py
```

`test_can_sender.py`의 실제 python-can 메시지 검사에는 `python3-can`이 필요합니다.
가짜 버스를 사용하므로 물리 CAN에는 송신하지 않습니다.
AMR 환경을 source한 셸에서 독립 브리지 ROS 테스트:

```bash
ROS_DOMAIN_ID=174 ROS_LOCALHOST_ONLY=1 ROS_LOG_DIR=/tmp/amr_can_pipeline_logs \
AMR_RUN_CAN_TESTS=1 python3 -m pytest -q \
  src/amr_system/amr_can_bridge/test/test_bridge_ros.py
```

도메인 174 역시 테스트 전용입니다. 실제 하드웨어 검증은 수행하지 않는 테스트입니다.

## 다음 단계

1. 실제 컨트롤러로 위 조작과 무선 연결 해제 시험 수행.
2. 구현된 Nav2 목표점 주행은 아래 **Smac 2D + DWB 내비게이션** 절차로 실행합니다.
3. 성공했던 `can0` 설정과 STM32 타임아웃 정지를 확인한 뒤 실차 CAN 송신 검증.
4. 구현된 `/wheel/odometry`와 `/imu/data_raw`를 실제 직진·회전 측정과 비교해 부호·거리·바이어스·공분산을 검증.
5. EKF·AMCL·Nav2 연결은 구현되어 있습니다. 실제 주행 정확도와 가감속 모델 조정은 소프트웨어 통합 시험과 구분합니다.


## 통합 실행과 TF

기존 `stack` alias는 `ros2 launch amr_stack bringup.launch.py`입니다. alias 변경 없이
기존 주행 제어 + S2 `/scan` + `robot_state_publisher` + `can_telemetry` + EKF를 실행합니다.
기존 별도 LiDAR/TF 런치는 Ctrl+C로 종료한 뒤 아래를 실행합니다.

```bash
amr
stack
```

| 옵션 | 기본값 | 기능 |
|---|---|---|
| `start_lidar` | `true` | S2 드라이버 실행 여부 |
| `use_rviz` | `false` | 선택적으로 RViz 실행 |
| `start_joy` | `true` | 실제 조이스틱 드라이버 실행 여부 |
| `start_can_bridge` | `true` | CAN 송신 브리지 실행 여부 |
| `start_can_telemetry` | `true` | 휠·IMU CAN 수신 노드 실행 여부 |
| `start_ekf` | `true` | EKF와 `odom → base_footprint` TF 실행 여부 |
| `ekf_config` | 패키지 `config/ekf.yaml` | EKF 설정 파일 |
| `can_dry_run` | `false` | `true`이면 CAN 송신 없이 미리보기 |

```bash
stack use_rviz:=true
# CAN 송신 없이 실제 LiDAR와 TF 확인 (기존 stack은 먼저 종료)
stack can_dry_run:=true start_joy:=false
# 구동 제어 없이 LiDAR와 로봇 TF만 실행
ros2 launch amr_stack lidar.launch.py
```

`lidar.launch.py`는 bringup 내부에서 포함하므로 `stack`과 별도로 동시에 실행하지 않습니다.
SDK의 `ros2 launch sllidar_ros2 sllidar_s2_launch.py`는 센서 드라이버만 실행합니다.
`serial_port`, `serial_baudrate`, `scan_mode`는 기존 S2 런치 인자로 변경할 수 있습니다.
AMR에서는 `/scan.header.frame_id=laser`, `inverted=false`를 유지합니다.

고정 TF의 발행자는 `robot_state_publisher` 하나입니다. 별도의
`amr_base_to_laser` static publisher는 제거했습니다.

```text
base_footprint
└── base_link                      z = 0.086 m
    ├── imu_link                   xyz = (0.160, 0.0035, 0.016) m, rpy = 0
    ├── lidar_mount_link           z = -0.029 m
    │   └── lidar_link             z = 0.137605 m
    │       └── laser              xyz = 0, yaw = pi
    └── 바퀴·캐스터 링크            기존 v28 관절 구조
```

`base_link`는 구동축 중앙이며 +X 전방, +Y 좌측, +Z 위입니다.
`base_link → laser`의 합성 위치는 `(0, 0, 0.108605)` m, yaw는 pi입니다.
로봇 정면의 물체는 `base_link`의 +X에 표시됩니다. RViz Fixed Frame도 `base_link`입니다.
`imu_link`는 센서 칩 중심이며 `/imu/data_raw`의 기준 프레임입니다.

URDF는 `src/amr_description/urdf/capstone_amr.urdf.xacro`와
확장본 `.urdf`에 보관합니다. 런치는 설치된 확장본을 읽고 mesh 경로를
`package://amr_description/meshes/visual/`로 변환하여 `/robot_description`으로 제공합니다.
Xacro 수정 시 확장본도 재생성해야 합니다. 원본의 상대 mesh 경로는 CAD 가져오기에 유지합니다.

고정 관절 TF는 `/tf_static`으로 바로 발행합니다. 구동 바퀴 TF는 실제 CAN 측정값으로 발행하는
`/joint_states`에 따라 갱신됩니다. 캐스터 관절은 측정값이 없어 발행하지 않습니다.
EKF는 `/odometry/filtered`와 `odom → base_footprint` TF를 발행합니다. 별도 `/odom` 토픽은 없으며 Nav2는 `/odometry/filtered`를 사용합니다. AMCL은 `localization.launch.py`, 경로 계획·제어는 `navigation.launch.py`로 별도 실행합니다. 원본 휠 토픽은 `/wheel/odometry`입니다.

바퀴 중심 간격은 URDF·상위 제한 설정·STM32 소스 모두 **0.499 m**입니다.
STM32 보드에 적용하려면 수정된 `stm32_main_code.c`를 실제 펌웨어 프로젝트에 반영해
빌드·업로드해야 합니다. 이 작업공간의 `colcon build`가 STM32 펌웨어를 업로드하지는 않습니다.


## 실제 CAN 피드백과 센서 토픽

`stack`에서 `can_telemetry`가 기본 실행됩니다. 수신 노드는 CAN 프레임이나 요청을 송신하지 않으며,
STM32가 모터를 제어하는 기존 경로는 유지합니다. 이번 수신 기능 때문에 STM32를 다시 업로드할 필요는 없습니다.

| 토픽 | 메시지 | 실제 확인 주기 | 내용 |
|---|---|---|---|
| `/wheel/odometry` | `nav_msgs/Odometry` | 약 100 Hz | 엔코더 위치 차분 기반 평면 위치, 측정 속도 |
| `/joint_states` | `sensor_msgs/JointState` | 약 100 Hz | 좌우 구동 바퀴만, rad 및 rad/s |
| `/imu/data_raw` | `sensor_msgs/Imu` | 약 10 Hz | 가속도 m/s², 각속도 rad/s |
| `/diagnostics` | `diagnostic_msgs/DiagnosticArray` | 1 Hz | 수신 신선도, 폐기·재기준 설정 횟수 |

다음은 **CAN 송신 없이** 센서와 TF를 확인하는 실행입니다. 기존 stack은 먼저 종료합니다.

```bash
amr
stack can_dry_run:=true start_joy:=false
```

다른 터미널에서:

```bash
amr
ros2 topic hz /wheel/odometry
ros2 topic hz /imu/data_raw
ros2 topic echo /wheel/odometry --once
ros2 topic echo /imu/data_raw --once --qos-reliability best_effort
ros2 topic echo /diagnostics --once
```

각 `topic hz`는 Ctrl+C로 종료하고 다음 명령을 실행합니다. RViz는 필요할 때만
`use_rviz:=true`를 추가합니다. IMU 구독은 sensor-data QoS(best effort)를 지원해야 합니다.
수신 노드만 실행하려면 다음 명령을 사용합니다.

```bash
ros2 run amr_can_bridge can_telemetry --ros-args --params-file \
  /home/acus03/amr_ws/install/amr_stack/share/amr_stack/config/can_telemetry.yaml
```

설정은 `src/amr_system/amr_stack/config/can_telemetry.yaml`입니다. 실행 중 변경하지 않고
재시작해 적용합니다. 송신과 수신의 `channel`을 변경할 경우 두 YAML을 함께 맞춥니다.

### 프로토콜과 좌표

- ODrive node 0의 `0x009`는 **오른쪽**, node 1의 `0x029`는 **왼쪽**입니다.
  `<ff>` 위치[turn]·속도[turn/s]이며 전진 기준 오른쪽 +1, 왼쪽 -1을 적용합니다.
  STM32의 과거 LEFT/RIGHT 매크로 이름과 실제 장착 위치를 혼동하지 않습니다.
- ODrive 보고값을 바퀴 회전수로 보는 비율은 현재 1입니다. 감속비 또는 ODrive 출력축 매핑이
  다르면 `encoder_turns_per_wheel_turn`을 실제 보고 단위에 맞춰야 합니다.
- `0x301` 가속도, `0x302` 각속도는 `<hhhH>`의 XYZ int16 + 공통 uint16 시퀀스입니다.
  같은 시퀀스끼리만 결합하고 wraparound를 처리합니다.
- 현재 STM32는 IMU 범위 레지스터 CTRL6/CTRL8을 설정하거나 CAN으로 읽어 보내지 않습니다.
  변환은 전원 초기값인 ±2g, ±125dps를 가정합니다. 가속도는 0.061 mg/LSB,
  자이로는 4.375 mdps/LSB입니다. 실제 센서 범위를 바꾸면 YAML도 반드시 맞춥니다.
  재업로드만으로 IMU 전원이 재설정된다고 가정하지 않으며, 기존에 범위를 바꿨다면 전원 재인가 또는
  레지스터 확인이 필요합니다. 정지 시 +Z 약 9.71 m/s²를 관측했으나 자이로 범위는 정지 데이터만으로 검증할 수 없습니다.
- IMU 축은 사용자가 확인한 +X 전방, +Y 좌측, +Z 위이며 `imu_link`에 그대로 표현합니다.
  중력을 제거하지 않습니다. CAN에는 자세값이 없으므로 `orientation_covariance[0] = -1`로
  표시하며 identity quaternion을 실제 자세 측정값으로 사용하지 않습니다.
- 측정 시각은 SocketCAN 커널 수신 시각입니다. 센서 내부 샘플 시각은 현 프로토콜에 없으며,
  실제 샘플 지연·휠 간 시차는 남아 있습니다. `use_sim_time=false`가 필요합니다.

프로토콜 근거: [ODrive CAN Encoder Estimates](https://docs.odriverobotics.com/v/latest/manual/can-protocol.html#get-encoder-estimates),
[ST LSM6DSV16X 데이터시트](https://www.st.com/resource/en/datasheet/lsm6dsv16x.pdf) Table 3, CTRL6/CTRL8,
설치된 ROS `sensor_msgs/msg/Imu.msg` 정의.

### 데이터 누락과 위치추정 연결

휠 양쪽이 새로 들어오고 시각 차이가 20ms 이내일 때만 결합합니다. 위치는 명령 속도가 아니라
엔코더 위치 차분으로 적분하고, 속도는 측정된 좌우 속도에서 계산합니다. 최초 위치는 (0,0,0)입니다.
250ms 이상 끊기거나 비현실적 위치 점프가 있으면 해당 구간을 적분하지 않고 엔코더 기준을 다시 잡습니다.
그동안의 이동은 복구하지 못하므로 위치 공분산을 키우고 진단 WARN을 유지합니다.
재시작하면 오도메트리 원점도 초기화됩니다. 작은 엔코더 리셋은 점프 한계로 검출되지 않을 수 있습니다.

오래된 프레임·잘못된 DLC·NaN/Inf·RTR·확장 프레임은 측정값으로 쓰지 않습니다.
휠/IMU 수신이 끊기면 해당 토픽 발행을 멈추고 `/diagnostics`로 알립니다.
진단은 주행 정지 명령을 보내지 않습니다. 기존 주행 watchdog은 별도이며,
향후 자율주행 단계에서 센서 유효성에 따른 주행 허용 정책을 연결해야 합니다.

`/wheel/odometry`는 `header.frame_id=odom`, `child_frame_id=base_footprint`입니다.
**이 수신 노드는 `odom → base_footprint` TF를 발행하지 않습니다.** 통합된 EKF만 그 TF를 맡습니다.
현재는 RViz Fixed Frame을 `base_link`로 사용합니다.

공분산은 보수적인 초기 추정치이며 실차 보정값이 아닙니다. IMU 자동 영점 보정도 하지 않습니다.
2026-09-29 바닥 정지 60초의 자이로 XYZ 평균을 `gyro_bias`에 수동 반영했고,
별도의 60초 재측정에서 Z 평균 편향이 약 99.6% 감소했습니다. 순간 흔들림은 남아 있습니다.
보정 전후 기록: `diagnostics/stationary_2026-09-29_020033/report.md`.
보정은 수신 노드 시작 시 읽으며, 전원·온도 조건이 달라지면 정지 편향 재확인이 필요합니다.
관측한 정지 자이로 평균은 대략 (-0.01275, 0.00151, 0.00025) rad/s였으며 온도·재시작에 따라 달라질 수 있습니다.
저속 직진·좌회전으로 입력 부호를 확인했고, IMU 바이어스를 정지 측정으로 보정했습니다.
EKF 통합 후 실제 주행에서 융합 결과와 공분산은 추가 검증해야 합니다. 캐스터 각도는 측정하지 않습니다.

### 수신 기능 검증

```bash
PYTHONPATH=src/amr_system/amr_can_bridge python3 -m pytest -q \
  src/amr_system/amr_can_bridge/test/test_telemetry.py

# amr로 환경을 source한 뒤 실행; 실제 CAN을 열지 않는 ROS 메시지 시험
ROS_DOMAIN_ID=184 ROS_LOCALHOST_ONLY=1 AMR_RUN_TELEMETRY_TESTS=1 \
  python3 -m pytest -q src/amr_system/amr_can_bridge/test/test_telemetry_ros.py
```

실제 센서 수신 검증 기록은 `diagnostics/telemetry_2026-09-29/live_result.json`에 있습니다.
CAN 수신 기능은 펌웨어를 수정하지 않습니다. EKF 통합 내용은 아래에 별도로 기록합니다.


## EKF 위치추정

필요 패키지는 `sudo apt install ros-jazzy-robot-localization`으로 설치합니다.
`stack` 하나로 EKF도 함께 실행됩니다. RViz는 기존처럼 선택 사항입니다.

```text
/wheel/odometry ─ vx, vy=0 제약 ─┐
                               ├─ ekf_filter_node ─ /odometry/filtered (30 Hz)
/imu/data_raw ─── gyro Z ───────┘                  └ odom → base_footprint TF
                                                        └ base_link → 센서·바퀴
```

- 설정: `src/amr_system/amr_stack/config/ekf.yaml`, 노드: 공식 `robot_localization/ekf_node`.
- `two_d_mode=true`, `world_frame=odom`, `base_link_frame=base_footprint`.
- 휠은 측정 전진 속도만 사용하며, 차동구동의 횡속도 0 제약도 공분산 0.01로 사용합니다.
  횡속도 0은 모델 가정이므로 실제 횡슬립을 측정하거나 제거하지 않습니다.
- 휠에서 이미 적분한 위치·방향과 휠 yaw rate는 융합하지 않습니다. 기존 회전 시험에서 슬립과
  휠·IMU 불일치가 관찰되어 회전 입력은 보정된 IMU gyro Z로 시작합니다.
- IMU orientation은 미제공(-1 공분산)이므로 사용하지 않으며, 가속도·중력 보정도 이번 EKF에서 사용하지 않습니다.
- 입력 공분산은 보수적인 초기값을 유지합니다. 정지 노이즈만으로 실주행 공분산을 확정하지 않습니다.
- `/odometry/filtered`는 표준 `nav_msgs/msg/Odometry`이고, 위치·자세·속도·공분산을 담습니다.
  `/wheel/odometry`, `/imu/data_raw`는 그대로 유지합니다. `map → odom`은 추후 SLAM/AMCL이 맡습니다.
- EKF 재시작 시 상대 원점이 초기화됩니다. 절대 방향 관측이 없으므로 gyro 바이어스에 따른 yaw 드리프트와
  휠 선속도 슬립에 따른 거리 오차가 남습니다. EKF가 슬립을 자동 판별·보정하는 것은 아닙니다.
- 입력이 끊겨도 EKF는 예측을 계속할 수 있습니다. 출력 Hz만으로 센서 건강을 판단하지 말고
  CAN 수신 `/diagnostics`의 freshness를 함께 확인합니다. 자율주행 허용 정책과의 연결은 후속 범위입니다.

```bash
amr
stack
# 다른 터미널
amr
ros2 topic hz /odometry/filtered
ros2 topic echo /odometry/filtered --once
ros2 run tf2_ros tf2_echo odom base_footprint
# RViz에서 주행 궤적을 볼 때 Fixed Frame을 odom으로 선택
```

CAN 송신 없이 실제 센서·TF·EKF만 검증할 때에는 기존 stack을 종료한 뒤:

```bash
ros2 launch amr_stack bringup.launch.py start_can_bridge:=false start_joy:=false start_lidar:=false
```

이 옵션에서는 CAN 송신 브리지를 실행하지 않습니다. mux의 0 속도 토픽은 남지만 CAN으로 송신되지 않습니다.
EKF를 제외한 이전 구성을 쓰려면 `stack start_ekf:=false`로 실행합니다.

합성 입력 ROS 통합 시험 (물리 CAN과 조이스틱 없이 실제 EKF·URDF 실행):

```bash
ROS_DOMAIN_ID=187 ROS_LOCALHOST_ONLY=1 AMR_RUN_EKF_TESTS=1 \
  python3 -m pytest -q src/amr_system/amr_stack/test/test_ekf_ros.py
```

이 시험은 휠 위치·yaw rate와 IMU 가속도를 일부러 모순되게 넣어 제외 여부를 검증하고,
전진 속도·양방향 gyro 응답, 평면 제약, odom에서 센서까지의 TF 연결을 확인합니다.

설정 근거: [robot_localization Humble 센서 구성 지침](https://github.com/cra-ros-pkg/robot_localization/blob/humble-devel/doc/configuring_robot_localization.rst),
[공식 EKF 설정 예시](https://github.com/cra-ros-pkg/robot_localization/blob/humble-devel/params/ekf.yaml).
프로젝트 검증 결과는 `diagnostics/ekf_2026-09-29/report.md`에 기록합니다.


## 기존 G_4_floor 지도와 AMCL

기본 지도: `/home/acus03/amr_ws/maps/G_4_floor/4_floor_0522.yaml`.
원본 YAML/PGM은 변경하지 않습니다. 해상도 0.05m, 1014×462셀, 원점 (-3.8, -1.54, 0)입니다.
AMCL과 map server 의존성: `sudo apt install ros-jazzy-nav2-amcl ros-jazzy-nav2-map-server ros-jazzy-nav2-lifecycle-manager`.

SLAM Toolbox를 종료한 뒤, 터미널 1에서 센서·제어·EKF를 실행합니다:

```bash
amr
stack
```

터미널 2에서 지도·AMCL을 별도로 실행합니다. 이미 stack이 켜져 있다면 아래만 추가합니다:

```bash
amr
ros2 launch amr_stack localization.launch.py
```

`stack`은 localization을 포함하지 않습니다. `start_localization` 옵션은 제거했습니다.
각 런치는 해당 터미널의 Ctrl+C로 개별 종료합니다. localization은 한 번만 실행합니다.
다른 지도를 쓰려면 실행 명령 뒤에 `map:=/absolute/path/map.yaml`을 추가합니다.
지도 작성 SLAM과 AMCL은 map → odom을 동시에 발행하면 안 되므로 SLAM 프로세스를 먼저 종료합니다.

구성: map_server + amcl + lifecycle_manager_localization. lifecycle manager가 두 노드를 자동 활성화합니다.
EKF는 odom → base_footprint, AMCL은 초기 위치와 스캔을 받은 뒤 map → odom을 발행합니다.
/map은 nav_msgs/msg/OccupancyGrid이며 transient-local QoS입니다. AMCL은 /scan과 기존 TF를 사용합니다.
지도 기준 위치는 /amcl_pose로 나옵니다. 이 localization 런치에는 경로 계획·제어기가 없으며, 목표점 주행은 별도 navigation 런치가 담당합니다.

AMCL은 `set_initial_pose=false`, `always_reset_initial_pose=true`로 실제 초기 위치 입력을 기다립니다.
RViz는 자동 실행하지 않습니다. `amr`로 환경을 로딩한 터미널에서
`rviz2 -d /home/acus03/amr_ws/src/amr_system/amr_stack/rviz/navigation.rviz`로 준비된 화면을 엽니다.
Fixed Frame `map`, Map(/map), LaserScan(/scan), 로봇 기준축과 AMCL 추정 위치가 설정되어 있습니다.
Map 표시의 Durability는 Transient Local로 설정합니다. **2D Pose Estimate**로 지도 위 실제 로봇 위치와 방향을 지정합니다.
이는 /initialpose 입력이며 구동 명령이 아닙니다. 초기 위치를 준 뒤 레이저 점이 지도 벽에 겹치는지 확인해야 합니다.
초기 위치 입력 전에는 map → odom 또는 /amcl_pose가 없을 수 있습니다.

AMCL 초기 설정은 config/localization.yaml입니다. base_footprint/odom/map 프레임, differential motion model,
laser_max_range=5.0m, update_min_d=0.1m, update_min_a=0.1rad를 사용합니다.
SLAM과 AMCL은 서로 다른 알고리즘이므로 기존 SLAM 설정 전체를 AMCL 설정으로 복사하지 않습니다.
같은 SLAM 파라미터로 만든 지도라도 현재 실제 벽 배치와 스캔 정합을 확인해야 합니다.

무구동 통합 시험:

```bash
ROS_DOMAIN_ID=189 ROS_LOCALHOST_ONLY=1 AMR_RUN_LOCALIZATION_TESTS=1 \
  python3 -m pytest -q src/amr_system/amr_stack/test/test_localization_ros.py
```

실제 저장 지도의 해상도·원점·셀값·late subscriber 전달, map server/AMCL 활성 상태 및 프레임 설정을 검사합니다.
센서·CAN 송신·조이스틱은 실행하지 않으며, 실제 위치추정 정확도를 검증하는 시험은 아닙니다.


## Smac 2D + DWB 내비게이션

ROS 2 Jazzy의 Smac 2D 전역 플래너와 DWB 지역 제어기를 사용합니다. 기존 stack과
localization 실행 구조를 유지합니다. navigation 런치 자체는 센서·CAN·EKF·AMCL을 실행하지 않습니다.

```bash
sudo apt-get install ros-jazzy-navigation2 ros-jazzy-nav2-bringup
cd /home/acus03/amr_ws
env -i HOME="$HOME" PATH=/usr/bin:/bin LANG=C.UTF-8 /bin/bash --noprofile --norc -c \
  'source /opt/ros/jazzy/setup.bash && colcon build --symlink-install'
```

각 터미널에서 `amr`로 환경을 로딩한 뒤 실행합니다. 이미 실행 중인 항목은 중복 실행하지 않습니다.

```bash
# 터미널 1: 실제 차량·센서·EKF (기존 stack)
amr
stack

# 터미널 2: 기존 지도·AMCL
amr
ros2 launch amr_stack localization.launch.py

# 터미널 3: 목표점 주행·RViz
amr
ros2 launch amr_stack navigation.launch.py use_rviz:=true
```

RViz Fixed Frame은 `map`입니다. 현재 위치를 AMCL에 지정하는 도구는 `2D Pose Estimate`,
목적지와 도착 방향을 지정하는 도구는 **2D Goal Pose**입니다. 지도에서 클릭하고 드래그해 방향을 지정합니다.
RViz는 기본 실행되지 않으며 `use_rviz:=true`를 지정했을 때만 실행합니다.
`stack use_rviz:=true`와 동시에 실행하면 RViz가 두 개 뜨므로 navigation 쪽 화면 하나를 사용합니다.

### 목표, RB, 속도 명령

```text
RViz /navigate_to_pose 또는 /goal_pose
  → navigation_manager (목표 보관·재개, 명령 허용)
  → /navigation/navigate_to_pose (내부 BT navigator)
  → Smac 2D → 방향 정렬 / DWB / 회전·후진 복구
  → /cmd_vel_navigation → navigation_manager → /cmd_vel_auto
  → 기존 control_mux → /cmd_vel → 기존 CAN 브리지 → STM32
```

`/navigate_to_pose`는 표준 `nav2_msgs/action/NavigateToPose`입니다. `/goal_pose`는
`geometry_msgs/msg/PoseStamped`이며 동일한 관리 경로로 전달됩니다. 목표는 `map` 기준의
유한한 평면 위치·방향을 사용합니다. 내부 액션과 `/cmd_vel_navigation`은 Nav2 연결용입니다.
`src/auto_turn_test.py` 등 다른 `/cmd_vel_auto` 발행 프로그램은 navigation과 동시에 실행하지 않습니다.

| 상황 | 동작 |
|---|---|
| MANUAL에서 목표 지정 | 목표를 보관하고 AUTO 진입 대기 |
| AUTO에서 목표 지정 | 경로 계산 후 주행 |
| RB OFF 또는 Joy 만료로 MANUAL 전환 | 기존 mux가 정지, 내부 Nav2 작업 중단, 목표 보관 |
| RB로 AUTO 재진입 | 이전 내부 작업의 종료 확인 후 현재 위치에서 재계획 |
| 일시정지 중 수동 주행 | 기존 LB 방식으로 조작, 재개 시 바뀐 위치 사용 |
| 새 목표 지정 | 이전 목표 종료 후 새 목표로 교체 |
| 목표 취소 서비스 호출 | 실행·보관 목표 모두 삭제 |
| 도착 또는 최종 실패 | 목표 종료, RB OFF/ON으로 재실행하지 않음 |
| navigation 재시작 | 이전 목표를 자동 복원하지 않음 |

RB 일시정지 동안 외부 목표 액션은 유지됩니다. 내부 진행 타이머·복구 동작은 중단됩니다.
정지/재개 상태는 아래 토픽에서 확인합니다. 주행 일시정지·재개에는 **RB**를 사용합니다.
기본 RViz는 Nav2 패널을 사용하지 않습니다. 실행·보관 목표를 완전히 삭제하려면:

```bash
ros2 service call /navigate_to_pose/_action/cancel_goal action_msgs/srv/CancelGoal "{}"
```

초기 위치를 다시 지정할 때는 RB로 정지한 뒤 `2D Pose Estimate`를 사용하고 지도·스캔 정렬을 확인합니다.

```bash
ros2 topic echo /navigation/status --qos-durability transient_local
```

상태는 `IDLE`, `WAITING_AUTO`, `WAITING_NAV2`, `RUNNING`, `PAUSED`, `STOPPING`,
`SUCCEEDED`, `FAILED`, `CANCELED`입니다. 명령 전달은 20Hz이며 마지막 명령이 0.20초 이상
지연되면 0을 전달합니다. 기존 mux와 CAN 브리지의 watchdog도 유지됩니다.
목표 교체·빠른 RB OFF/ON에서도 이전 내부 액션의 취소 응답만으로 다음 실행을 시작하지 않고,
실제 terminal result를 기다립니다. 내부 응답이 지연되어도 새 목표를 중복 실행하지 않습니다.
Nav2 핵심 프로세스가 종료되면 navigation 런치도 함께 종료하며 자동 재시작하지 않습니다.

### 차량 설정과 알고리즘

- `config/navigation.yaml`: Nav2·목표 관리 설정. `launch/navigation.launch.py`: 노드·토픽 연결.
- 자율주행 선속도 요청 상한 0.40 m/s, 회전 속도 상한 0.60 rad/s. DWB의 정상 추종은 전진·제자리 회전,
  자동 후진은 복구 동작에서만 수행합니다.
- `amr_nav2_plugins::WheelSpeedCritic`이 좌우 바퀴 중 하나라도 기존 상한을 넘는 속도 후보를 제외합니다.
  바퀴 지름·간격·최대 회전수는 **기존 control_mux.yaml에서 런치가 읽어 전달**합니다.
  바퀴 사양에 따른 직진 상한은 약 0.973 m/s이며, navigation은 재시험을 위해 0.40 m/s로 제한합니다.
- 별도 control 설정으로 stack을 실행했다면 navigation에도 동일한 `control_config:=...`를 전달합니다.
- DWB는 20Hz, `LimitedAccelGenerator`, 예측 1.5초, 선속도 12개·각속도 24개 기본 샘플을 사용합니다.
  장애물 비용과 polygon footprint 충돌 검사를 함께 적용합니다.
- 구현 중 보완: Smac 2D의 급한 출발 방향 변화에서 저가속 DWB가 정체하는 현상을 줄이기 위해
  `FollowPath`는 기존 `nav2_rotation_shim_controller::RotationShimController`로 감싸고,
  실제 `primary_controller`는 `dwb_core::DWBLocalPlanner`로 유지합니다. 새 경로와 방향 차이가
  0.60rad를 넘으면 최대 0.40rad/s로 제자리 정렬하고 0.20rad 이내에서 DWB에 넘깁니다.
  경로 시작점에서 1.00m 떨어진 점을 참조합니다. 2026-10-02 bag에서 기존 0.50m 참조점을
  정지 중 지나치며 뒤쪽으로 회전한 현상이 확인되어 속도와 참조 거리를 함께 조정했습니다.
  이 회전도 footprint 충돌 검사와 동일한 목표 관리·mux 경로를 거칩니다.
- DWB의 최소 탐색 속도는 병진 0.02m/s·회전 0.04rad/s입니다. 둘 중 하나가 충족되는 후보를
  허용하며, `PreferForward`는 병진 0.10m/s와 회전 0.12rad/s에 모두 못 미치는 미세 명령에
  비용 0.5를 줍니다. 나머지 후보에는 `0.5 × |각속도|`의 회전 비용을 줍니다.
  회전 최소값은 목표 정지 판정의 0.05rad/s보다 작아 마지막 방향 정렬을 마칠 수 있습니다.
  이는 정상 추종의 후보 평가 설정이며 정지 명령·일시정지·취소를 제한하지 않습니다.
  벽 inflation 비용 `BaseObstacle.scale=0.12`, 경로 방향 `PathAlign.scale=32.0`과
  전방 투영 0.50m, 지역 목표 방향 `GoalAlign.scale=8.0`, 경로 거리 `PathDist.scale=32.0`,
  목표 접근 `GoalDist.scale=32.0`을 사용합니다.
  [세 번째 bag의 직진·코너·저속 회전 분석](diagnostics/navigation_2026-10-02_trial3/REPORT.md)에 근거를 기록합니다.
- DWB 가감속 모델은 선형 ±0.50 m/s², 회전 ±1.60 rad/s²입니다. **ODrive 실측값이 아니며**
  회전 값은 기록에서 작은 조향 명령에 실제 차체 회전이 부족했던 현상을 고려한 탐색 범위 조정입니다.
  정지 상태에서 한 주기에 ±0.08rad/s 후보까지 평가할 수 있습니다. 방향 정렬 shim은 ±0.80rad/s²입니다.
  ODrive 펌웨어·VEL_RAMP 설정은 변경하지 않습니다. velocity smoother를 추가하지 않습니다.
- 전역 costmap은 `map`, 전체 지도, 0.05m, 2Hz. 지역 costmap은 `odom`, 6×6m, 0.05m, 10Hz입니다.
  화면 발행은 전역 1Hz·지역 2Hz이며 변화한 영역을 전송합니다. 충돌 검사 갱신률은 그대로입니다.
- 전역은 차체 외접원+여유를 포함한 반경 0.44m, 지역은 CAD 외곽 0.640×0.544m의 직사각형과
  padding 0.02m를 사용합니다. 추가 돌출 장비가 없는 기본 차체를 가정합니다.
- `/scan` 장애물 반영 5m, ray tracing 6m, inflation 반경 1.60m·비용 감소 계수 2.0입니다.
  LiDAR 갱신이 0.5초 이상 지연되면 obstacle layer가 오래된 것으로 처리됩니다.
- Smac 2D는 미확인 공간을 통과하지 않으며, 내장 smoother와 비용 가중치 4.0을 사용합니다.
  `nav_to_pose_with_consistent_replanning_and_if_path_becomes_invalid.xml`을 사용합니다.
  경로 유효성을 2Hz로 확인하고 장애물로 막힘·목표 변경·10초 만료 때 재계획합니다.
  유효한 우회 경로를 유지하므로 대칭 장애물 양쪽 경로가 매초 교대하며 회전하는 현상을 줄입니다.
- 도착 허용 오차는 위치 0.15m·방향 0.15rad이며 `StoppedGoalChecker`가 정지 조건도 확인합니다.
- 기본 복구는 costmap 갱신, 90° 회전(최대 0.40rad/s), 5초 대기, 0.30m 후진(0.05m/s)입니다.
  복구 재시도 6회를 소진하면 목표를 실패로 종료합니다.
- `/odometry/filtered`는 EKF 속도 입력, 차량 위치는 기존 `map → odom → base_footprint` TF를 사용합니다.
  CAN 센서 freshness 진단을 자율 허용 조건에 직접 연결하는 기능은 이번에 추가하지 않았습니다.
- 단일 목표 주행을 제공합니다. 다중 경유점·순찰·도킹은 포함하지 않습니다.
- `/evaluation` 후보 궤적 전체 발행은 기본 비활성입니다. 전체 토픽 bag 수집 때 이 토픽만
  310MiB/111초를 차지했으며 제어 주기 미준수도 기록됐습니다.
  [2026-10-02 분석 및 튜닝 검증](diagnostics/navigation_2026-10-02/REPORT.md)을 참고하세요.

### 구동 없는 자동 테스트

AMR 환경을 로딩한 셸에서 실행합니다. 테스트는 물리 CAN·조이스틱·LiDAR를 실행하지 않습니다.
도메인 190/191/192는 테스트용이며 실제 로봇과 분리합니다.

```bash
ROS_DOMAIN_ID=192 ROS_LOCALHOST_ONLY=1 colcon test --packages-select amr_nav2_plugins --return-code-on-test-failure
ROS_DOMAIN_ID=190 ROS_LOCALHOST_ONLY=1 AMR_RUN_NAVIGATION_TESTS=1 \
  python3 -m pytest -q src/amr_system/amr_stack/test/test_navigation_manager_ros.py
ROS_DOMAIN_ID=191 ROS_LOCALHOST_ONLY=1 AMR_RUN_NAV2_INTEGRATION=1 \
  python3 -m pytest -q src/amr_system/amr_stack/test/test_navigation_pipeline_ros.py
```

통합 시험은 가상 지도·LaserScan·TF·오도메트리·Joy를 제공하고 실제 Smac 2D/DWB/복구 노드와
기존 mux, CAN **dry-run** 브리지를 연결합니다. 물리적 제동·슬립·위치추정 정확도는 이 시험의 대상이 아닙니다.
실행 기록과 변경 전 문서는 `diagnostics/navigation_2026-09-29/`에 보관합니다.

### RViz 경량 화면과 장애물 표시

기존 Humble 환경에서 초기 위치 지정 뒤 `rviz2 is not responding`으로 멈추던 원인은 TF 0.25.23의
메시지 필터와 TF 수신 스레드 사이 교착 상태였습니다. 실제 멈춘 프로세스의 호출 스택으로
확인했으며, [상위 오류 보고](https://github.com/ros2/geometry2/issues/992)와 일치합니다.
기본 화면은 `amr_rviz_plugins/SingleThreadedTF`를 사용해 RViz의 TF 수신·표시 처리를
하나의 실행 스레드에서 순서대로 실행합니다. 시스템 TF 라이브러리와 차량 제어에는 적용하지 않습니다.
빌드는 기존 `--packages-up-to amr_stack`에 새 플러그인이 포함됩니다. navigation launch는
같은 작업공간의 설치된 플러그인 경로를 RViz에 직접 전달하므로 기존 터미널에서도 적용됩니다.
플러그인이 설치되지 않았으면 실행 전에 빌드 안내와 함께 중단합니다. 이미 실행 중인
RViz는 navigation을 재시작해야 새 코드를 읽습니다. 사용자 화면을 저장할 때도
`Transformation -> Current`의 `amr_rviz_plugins/SingleThreadedTF` 선택을 유지합니다.
진단 기록은 `diagnostics/rviz_hang_live_2026-09-29/`에 보관합니다.

기본 `navigation.rviz`는 RViz 표준 `SetGoal` 도구로 `/goal_pose`를 발행합니다.
목표는 기존 navigation_manager를 거쳐 RB 모드·취소·속도 제한을 동일하게 적용받습니다.
기존 Humble Nav2 1.1.20 RViz 플러그인의 종료 충돌을 피하기 위해 Navigation 2 패널,
GoalTool, ParticleCloud를 기본 화면에서 제거했습니다. 프레임은 15fps, CAD 모델과 전역
costmap 표시·경로별 화살표는 기본으로 끕니다. CAD 모델은 Displays에서 필요할 때 켤 수 있습니다.

- 차량 둘레의 초록 **Robot Footprint**: 차체 0.640×0.544m + 사방 padding 0.02m의 충돌 검사 외곽.
- 장애물 주변의 색 띠: inflation 비용. 가까울수록 높은 비용이며 1.60m 전체가 통행 금지는 아닙니다.
- 전역 계획의 반경 0.44m는 방향을 고려하지 않는 Smac 2D의 외접원입니다. 약 0.88m 미만
  통로에는 경로를 만들 수 없습니다. 지역 polygon만 줄여도 이 한계는 바뀌지 않습니다.

RViz 비정상 종료 시 navigation 전체 종료 정책은 유지합니다. 재실행 후 새 목표를 지정합니다.
변경 전 강제 종료(-9)는 사용자가 응답 없는 창을 종료한 기록이며 OOM으로 단정하지 않습니다.
진단·비교 실험·적용 설정은 `diagnostics/navigation_tuning_2026-09-29/REPORT.md`에 기록합니다.
