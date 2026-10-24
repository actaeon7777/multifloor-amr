# 엘리베이터 RF 중계 CAN 프로토콜

- 문서 버전: 1.1 초안 (2026-10-21) — `0x210` byte 5 `repeat`(열림 반복) 추가. 프레임 `version`은 1 그대로
- CAN 프레임 프로토콜 버전: **1**
- 대상 독자

  | 장치 | 볼 곳 |
  |---|---|
  | 하위제어기 NUCLEO-F446RE (CAN 쪽) | 전체 |
  | 상위제어기 Jetson `elevator_can_link` | §1–§5, §7–§9 |

- 상태: **펌웨어 구현 완료**(CAN 미연결 상태로 단독 검증). Jetson은 §7.2를 반영했다(2026-10-21, 단위·ROS 시험). 통합 미시험 (§10)

하위제어기는 엘리베이터 비콘과의 무선 구간을 맡고, 그 결과를 이 문서의 CAN 프레임으로 Jetson과 주고받는다.
무선 구간(nRF24 설정, RF 프레임, 비콘 동작)은 [엘리베이터 RF 프로토콜](ELEVATOR_RF_PROTOCOL.md)에 있다. 이 문서에서 "RF §n"은 그 문서의 절이다.
엘리베이터 미션 전체 동작은 [층간 이동](ELEVATOR.md)에 있다.

---

## 1. 구성

```text
 비콘 ◀── nRF24 ──▶ 하위제어기 (Nucleo) ◀── CAN can0 ──▶ Jetson: elevator_can_link ◀──▶ /elevator/command, /elevator/beacon
                                        0x210 명령 ▶
                                        ◀ 0x310 이벤트, 0x311 진단
```

| 주체 | 책임 |
|---|---|
| Jetson | 버튼을 언제 누를지 결정하고(미션) `0x210`을 보낸다. 3초 안에 BUTTON_ACK가 없으면 새 sequence로 다시 보낸다. 층·문 상태 판단 |
| 하위제어기 | `0x210`을 검증해 대상 비콘에 RF로 전달하고(RF §7.4), 비콘 PRESENCE·RESULT를 `0x310`으로, 처리 경과를 `0x311`로 알린다 |

---

## 2. 버스와 ID

| 항목 | 값 |
|---|---|
| 버스 | 500 kbit/s, Classic CAN, 표준 11-bit ID, DATA 프레임 |
| DLC | 8 |
| 바이트 순서 | little-endian |
| 무결성 | **byte 7 = byte 0–6의 CRC-8/SMBUS** (다항식 0x07, 초기값 0, 검사값 `crc8("123456789") = 0xF4`). RF 프레임과 같은 함수 |

| ID | 방향 | 내용 | 상태 |
|---|---|---|---|
| `0x210` | Jetson → Nucleo | 버튼 명령 (§3) | 양쪽 구현 (1.1 `repeat` 포함) |
| `0x310` | Nucleo → Jetson | 비콘 이벤트: PRESENCE, BUTTON_ACK (§4) | 양쪽 구현 (카 PRESENCE 허용) |
| `0x311` | Nucleo → Jetson | 무선 진단 (§5) | 양쪽 구현. Jetson은 미션·웹 UI·`/diagnostics`에 사용 |

### ID 충돌

ODrive CANSimple ID는 `(node_id << 5) | cmd`다.

| 커스텀 ID | CANSimple로 보면 |
|---|---|
| `0x210` | node 16, cmd 0x10 |
| `0x310` / `0x311` | node 24, cmd 0x10 / 0x11 |

현재 ODrive는 node 0/1이라 겹치지 않는다. [E-stop 문서](ESTOP_CAN_PROTOCOL.md#id-충돌-주의)와 같이 **ODrive를 node 16·24로 설정하지 않는다.**
사용 중인 ID 전체: `0x001`–`0x038`(ODrive 0/1), `0x201`(주행), `0x202`(E-stop), `0x210`, `0x301`·`0x302`(IMU), `0x310`, `0x311`.

버스 부하: `0x310`은 들리는 비콘 수 × 1 Hz + 응답, `0x311`은 1 Hz + 명령당 2–3개 + 비콘당 0.1 Hz다. 주행 명령(`0x201`, 20 Hz)에 비해 무시할 수준이다.

---

## 3. `0x210` 버튼 명령 (Jetson → Nucleo)

| byte | 형식 | 필드 | 설명 |
|---|---|---|---|
| 0 | uint8 | `version` | 1 |
| 1 | uint8 | `sequence` | Jetson 명령 번호 → RF `cmd_seq`. 재시도마다 새 번호 |
| 2 | uint8 | `command` | `1` HALL_CALL, `2` CAR_FLOOR, `3` DOOR_OPEN |
| 3 | int8 | `floor_code` | HALL_CALL: 호출 층, CAR_FLOOR: 누를 층, DOOR_OPEN: 0 |
| 4 | uint8 | `direction` | HALL_CALL: `1` UP / `2` DOWN, 그 외 `0` |
| 5 | uint8 | `repeat` | **DOOR_OPEN 전용:** 열림 버튼을 약 1초 간격으로 누를 횟수. `0`·`1` = 한 번, `2`–`15` = 그 횟수. 기본 사용값 **10** (AMR이 내리는 동안 약 10초). 다른 명령은 **0**. 대상 비콘은 Nucleo가 command·floor_code로 정한다 |
| 6 | — | 예약 | 0 |
| 7 | uint8 | `crc8` | |

**Nucleo 검증** (실패 시 `0x311` BAD_FRAME, RF로 보내지 않음): DLC 8, 표준 ID, DATA 프레임, CRC, `version`=1, byte 6 = 0, byte 5 `repeat`는 DOOR_OPEN이면 0–15 그 밖에는 0, 인자 규칙
(command 1–3, HALL_CALL은 층≠0·방향∈{1,2}, CAR_FLOOR는 층≠0·방향=0, DOOR_OPEN은 층=0·방향=0). Jetson `elevator_protocol.validate_command`와 같은 규칙이다.

**대상 비콘**: HALL_CALL → `0x40 + floor_code`(홀 비콘), CAR_FLOOR·DOOR_OPEN → `0x21`(카 비콘 E1). 범위 밖이면 `0x311` NO_ROUTE.

---

## 4. `0x310` 비콘 이벤트 (Nucleo → Jetson)

| byte | 형식 | 필드 | PRESENCE에서 | BUTTON_ACK에서 |
|---|---|---|---|---|
| 0 | uint8 | `version` | 1 | 1 |
| 1 | uint8 | `sequence` | RF `seq` (비콘 번호) | RF `seq` (비콘 번호) |
| 2 | uint8 | `event` | `1` PRESENCE | `2` BUTTON_ACK |
| 3 | int8 | `floor_code` | RF `floor_code` (홀: 층, 카: 0) | RESULT의 `floor_code` (비콘 자신의 층) |
| 4 | uint8 | `location` | RF `role` (`0` 홀, `1` 카) | RESULT 보낸 비콘의 role |
| 5 | uint8 | `ack_sequence` | 0 | RESULT `cmd_seq` |
| 6 | int8 | `rssi_dbm` | 수신 세기 등급 (아래 표) | 수신 세기 등급 |
| 7 | uint8 | `crc8` | | |

- PRESENCE는 받을 때마다 보낸다(같은 `src`·`seq` 5초 안 중복은 한 번만). 홀·카 비콘 모두 보낸다.
- BUTTON_ACK는 RESULT가 **성공(`PRESSED` 0x00, `DUPLICATE` 0x01)** 일 때만 보낸다. 실패 결과는 `0x311`로만 알린다.
  대기 중인 명령과 `cmd_seq`가 맞지 않는 늦은 성공 RESULT도 보낸다(Jetson은 이전 번호의 늦은 응답도 받아들인다).
- 홀 비콘은 `floor_code` ≠ 0, 카 비콘은 `floor_code` = 0이다.
- RF 수신 후 **20 ms 안에** CAN 송신 대기열에 넣는다. 늦어진 프레임을 쌓아 두었다가 보내지 않는다(Jetson은 커널 타임스탬프 기준 0.5초 넘은 프레임을 버린다).

**수신 세기 등급** — nRF24에는 RSSI가 없어 RPD 1비트(RF §7.3)를 등급값으로 싣는다.

| 상황 | `rssi_dbm` |
|---|---|
| RPD = 1 (> −64 dBm, LNA 이득 포함) | **−60** |
| RPD = 0, 또는 FIFO에서 2개 이상을 한 번에 꺼내 어느 패킷의 RPD인지 모름 | **−80** |

- 실제 dBm이 아니다. Jetson의 홀 비콘 기준 −70 dBm(`presence_min_rssi`)과 맞물려 "RPD=1인 홀 비콘만 층 근거로 쓴다"는 뜻이 된다.
- **0을 보내지 않는다.** Jetson은 0을 "세기 모름"으로 보고 세기 검사를 건너뛴다.

---

## 5. `0x311` 무선 진단 (Nucleo → Jetson)

| byte | 공통 |
|---|---|
| 0 | `version` = 1 |
| 1 | `kind`: `1` COMMAND_STATUS, `2` LINK_STATUS, `3` DEVICE_STATUS |
| 2–6 | kind별 |
| 7 | `crc8` |

### kind 1 COMMAND_STATUS — 명령 처리 단계가 바뀔 때마다

| byte | 필드 | 설명 |
|---|---|---|
| 2 | `cmd_seq` | `0x210` sequence (BAD_FRAME이면 받은 byte 1 그대로) |
| 3 | `status` | 아래 표 |
| 4 | `device_result` | RESULT 결과 코드(RF §5.5), 없으면 `0xFF` |
| 5 | `target` | 대상 비콘 node_id, 정하지 못했으면 0 |
| 6 | `attempts` | COMMAND RF 송신 시도 횟수(비콘 깨어남 단위) |

| status | 이름 | 언제 | `0x310` |
|---|---|---|---|
| 1 | `DELIVERED` | 비콘이 COMMAND를 하드웨어 ACK함 | — |
| 2 | `COMPLETED` | 성공 RESULT 수신 | BUTTON_ACK 보냄 |
| 3 | `DEVICE_REJECTED` | 실패 RESULT 수신 (`device_result` ≥ 0x80) | 없음 |
| 4 | `RF_TIMEOUT` | 명령 수신 후 2.5초 안에 전달 못 함 | 없음 |
| 5 | `NO_RESULT` | 전달 후 1.5초 안에 RESULT 없음 | 없음 |
| 6 | `NO_ROUTE` | 대상 비콘을 정할 수 없음 | 없음 |
| 7 | `BAD_FRAME` | `0x210` 검증 실패 | 없음 |
| 8 | `REPLACED` | 같은 비콘에 새 명령이 와서 이전 명령을 버림 | 없음 |

### kind 2 LINK_STATUS — 1초마다

| byte | 필드 | 설명 |
|---|---|---|
| 2 | `flags` | bit0 `RADIO_OK`(nRF24 레지스터 검증 통과), bit1 `RX_OVERFLOW`(지난 1초에 RX FIFO 가득 참), bit2 `TX_TIMEOUT`(TX_DS·MAX_RT 무응답) |
| 3 | `rx_ok` | 유효 RF 수신 수 (하위 8비트, 누적) |
| 4 | `rx_bad` | 폐기한 RF 수신 수 (하위 8비트, 누적) |
| 5 | `tx_fail` | COMMAND MAX_RT 횟수 (하위 8비트, 누적) |
| 6 | `pending` | 대기 중인 명령 수 |

### kind 3 DEVICE_STATUS — 비콘별, PRESENCE flags가 바뀔 때와 10초마다

| byte | 필드 | 설명 |
|---|---|---|
| 2 | `node_id` | 비콘 |
| 3 | `flags` | PRESENCE flags (RF §5.5: BOOT, BATT_LOW, BUTTON_FAULT, CONFIG_ERR, HAS_DOOR_OPEN, HAS_UP, HAS_DOWN) |
| 4–5 | `battery_mv` | uint16 LE |
| 6 | `fw_version` | |

---

## 6. 하위제어기 구현 요구

- CAN 수신 필터에 `0x210`을 추가한다. 현재 필터 뱅크에 빈 슬롯이 부족하면 새 뱅크를 쓴다. 수신 콜백에 `0x210` 분기를 넣는다.
- `0x210`을 받으면 §3 검증 → 대상 결정 → RF 대기 슬롯에 넣는다. 이후 RF 동작은 RF §7.4를 따른다.
- RF 중계는 **E-stop과 독립**이다. 래치 중에도 비콘 전달과 명령 중계를 계속한다(층 추적이 끊기지 않게). 언제 누를지는 Jetson 미션이 판단한다.
- CAN 송신은 E-stop 정지 프레임이 우선이다([E-stop 문서](ESTOP_CAN_PROTOCOL.md) §4.3). `0x310`/`0x311`은 그 뒤에 보낸다.
  다만 RF 처리(PRESENCE → COMMAND 3 ms)가 CAN mailbox 대기에 막히면 안 된다(RF §7.5).

### 6.1 구현 메모 (펌웨어 `main.c`, `elevator_rf.c`)

- **수신 필터:** bank 1의 4번째 슬롯에 원래 `0x021`이 중복으로 들어 있었다. 이 슬롯을 `0x210`으로 바꿨다. 필터는 DATA 프레임만 통과시킨다.
- **수신 처리:** CAN 수신 인터럽트는 원본 8바이트를 큐에 넣기만 한다. 검증은 RF_Task가 한다.
- **송신 순서:** RF_Task가 `0x310`/`0x311`을 큐(16개)에 넣으면, CAN_Task가 루프마다 1개씩 보낸다.
  - E-stop 프레임이 송신 중이거나 정지 묶음을 보낼 차례(`estop_can_mailbox_hold=1`)이면 보내지 않는다.
  - 큐에 들어간 지 20 ms가 넘은 프레임은 보내지 않고 버린다(`elev_can_tx_stale`).
  - 큐가 가득 차면 새 프레임을 버린다(`elev_can_tx_dropped`).
  - E-stop 래치 중에도 정지 묶음 사이에 계속 보낸다.
- **CAN 미연결(ACK 없음):** E-stop 상태 프레임이 mailbox를 계속 점유하므로 `0x310`/`0x311`은 나가지 않고 큐가 찬다. 연결하면 오래된 프레임은 stale로 버리고 새 프레임부터 나간다.
- **LINK_STATUS 카운터:**
  - `rx_bad`는 형식 오류로 버린 프레임만 센다. 같은 (src, seq) 중복과 역순 폐기는 따로 `elev_rx_dup`에 센다.
  - `tx_fail`은 MAX_RT와 송신 무응답(20 ms)을 합친 값이다.
- **늦게 온 RESULT:** 대기 중인 `cmd_seq`와 맞지 않는 RESULT에도 `0x311` COMPLETED/DEVICE_REJECTED를 `attempts=0`으로 보낸다.

---

## 7. 상위제어기 (Jetson)

### 7.1 ROS 매핑 (현재 구현)

| CAN | ROS | 노드 |
|---|---|---|
| `0x210` ← | `/elevator/command` (`ElevatorCommand`) | `elevator_can_link` |
| `0x310` → | `/elevator/beacon` (`ElevatorBeacon`, `source="can"`, 수신 시각 = 커널 타임스탬프) | `elevator_can_link` |

- 열거값은 ROS 메시지 [`ElevatorCommand`·`ElevatorBeacon`](../src/amr_system/amr_elevator_interfaces/msg/)과 같다.
- `elevator_monitor`: `BeaconFilter`(2초 넘은 프레임 폐기, 5초 안 같은 번호 중복 폐기, 차이 128 이상 역순 폐기), 홀 PRESENCE + 세기 ≥ −70 + 문 열림으로 층 확정.
- `floor_mission_manager`: 명령마다 sequence를 1씩 늘려 새로 보내고, 3초 안에 같은 location의 BUTTON_ACK(이전 번호 포함)가 없으면 다시 누른다. 3회 실패하면 `NO_HALL_ACK`/`NO_CAR_ACK`.

### 7.2 이 프로토콜에 맞추기 위한 변경 (Jetson 반영 2026-10-21, 실차 통합 미시험)

| 파일 | 변경 | 필수 |
|---|---|---|
| [`elevator_protocol.py`](../src/amr_system/amr_can_bridge/amr_can_bridge/elevator_protocol.py) | `decode_event`가 카 비콘 PRESENCE(`location=1`, `floor_code=0`)를 받아들이게 한다. 지금은 "only hall devices announce a floor"로 거절해 `invalid`가 1초마다 늘고 진단이 WARN이 된다 | **필수** |
| 같은 파일 | "PROPOSAL / TODO(firmware)" 주석을 이 문서 버전으로 바꾼다 | 필수 |
| [`elevator_can_link.py`](../src/amr_system/amr_can_bridge/amr_can_bridge/elevator_can_link.py) | 커널 필터에 `0x311` 추가(mask `0x7FE`로 0x310–0x311), 디코드해 `/diagnostics`에 표시 | 선택 |
| `floor_state.py` | 변경 없음. 카 PRESENCE는 층 판단에서 이미 무시되고, −60/−80은 −70 기준과 맞는다 | — |
| `elevator_protocol.py` | **(1.1)** `encode_command`/`validate_command`에 byte 5 `repeat`. DOOR_OPEN은 0–15, 다른 명령은 0. 기본 사용값 10은 미션 파라미터 `door_open_repeat_exit`가 정한다 | 필수 |
| `floor_mission_manager` | **(1.1)** 내릴 때 DOOR_OPEN을 repeat 10으로 한 번 보낸다. BUTTON_ACK는 "열림 반복 시작"을 뜻한다. 1초마다 다시 보내지 않는다 | 필수 |
| 시험 | §9 시험 벡터 추가 | 필수 |

- 시뮬레이션(`elevator_model.py`)은 −50/−82/−58 dBm을 쓴다. 기준 −70과 같은 결론이 나므로 바꾸지 않는다.

---

## 8. 전체 시간 예산

| 구간 | 최대 | 근거 |
|---|---|---|
| Jetson `0x210` → Nucleo 대기 슬롯 | 수 ms | CAN |
| 무선 구간 (PRESENCE 대기 ~ RESULT 수신) | 약 1.5 s | RF §8 |
| Nucleo → `0x310` BUTTON_ACK | 20 ms | §4 |
| **합계** | **약 1.5 s** | Jetson 응답 대기 3 s 안 |

---

## 9. 시험 벡터

hex, 바이트 순서 그대로. `0x210`/`0x310`은 현재 [`elevator_protocol.py`](../src/amr_system/amr_can_bridge/amr_can_bridge/elevator_protocol.py)의
`encode_command`/`encode_event` 출력과 같음을 확인했다(카 PRESENCE 제외, §7.2). 대응하는 RF 프레임은 RF §10에 있다.

| ID | 상황 | data |
|---|---|---|
| `0x210` | HALL_CALL 4F DOWN, seq 0x15 | `01 15 01 04 02 00 00 20` |
| `0x210` | CAR_FLOOR 3F, seq 0x16 | `01 16 02 03 00 00 00 49` |
| `0x210` | DOOR_OPEN, seq 0x18 | `01 18 03 00 00 00 00 A8` |
| `0x210` | DOOR_OPEN ×10 (1.1), seq 0x19, repeat 10 | `01 19 03 00 00 0A 00 03` |
| `0x310` | 카 BUTTON_ACK (열림 ×10 시작), seq 0x92, ack 0x19 | `01 92 02 00 01 19 C4 E8` |
| `0x311` | COMPLETED, cmd 0x19, 대상 0x21, 1회 | `01 01 19 02 00 21 01 2F` |
| `0x310` | 4F 홀 PRESENCE, seq 0x2A, RPD=1 | `01 2A 01 04 00 00 C4 91` |
| `0x310` | 3F 홀 PRESENCE, seq 0x10, RPD=0 | `01 10 01 03 00 00 B0 00` |
| `0x310` | 4F 홀 BUTTON_ACK, seq 0x2B, ack 0x15 | `01 2B 02 04 00 15 C4 08` |
| `0x310` | 카 BUTTON_ACK, seq 0x90, ack 0x16 | `01 90 02 00 01 16 C4 79` |
| `0x310` | 카 PRESENCE, seq 0x8F (현재 Jetson은 거절, §7.2) | `01 8F 01 00 01 00 C4 F8` |
| `0x311` | COMPLETED, cmd 0x15, 대상 0x44, 1회 | `01 01 15 02 00 44 01 0D` |
| `0x311` | DEVICE_REJECTED UNSUPPORTED, cmd 0x17, 대상 0x45 | `01 01 17 03 82 45 01 17` |
| `0x311` | RF_TIMEOUT, cmd 0x16, 대상 0x21, 2회 | `01 01 16 04 FF 21 02 49` |
| `0x311` | LINK_STATUS, RADIO_OK, rx_ok 0x34 | `01 02 01 34 00 00 00 1E` |
| `0x311` | DEVICE_STATUS 0x44, flags 0x60, 3000 mV, fw 1 | `01 03 44 60 B8 0B 01 9D` |

---

## 10. 검증 현황 (2026-10-21)

| 항목 | 상태 |
|---|---|
| `0x210`/`0x310`/`0x311` 시험 벡터 16개가 Jetson `elevator_protocol.py` 출력과 일치 | 확인 (2026-10-21, `test_elevator_protocol.py`) |
| Jetson: 카 PRESENCE 허용, `0x311` 디코드, `repeat`, 미션 반영 | 구현·단위/ROS 시험 통과 (2026-10-21) |
| Nucleo host 시험 (`tools/check_elevator.py`): §9 벡터 13개 일치, 검증 규칙(BAD_FRAME 11종, NO_ROUTE), 라우팅, COMMAND_STATUS 전 상태, LINK/DEVICE 주기 | 통과 |
| Nucleo host 시험 (`tools/check_estop_can.py`): `0x210` 수신 → RF 큐, E-stop 송신 중 대기, 래치 중 정지 묶음 사이 송신, 20 ms stale 폐기, 정지 묶음 차례에 대기 프레임 즉시 abort | 통과 |
| Nucleo 실기 (CAN 미연결, 디버거 주입): HALL_CALL 4F → 2.5 s 뒤 RF_TIMEOUT(seq 0x15), CRC 오류 → BAD_FRAME, 21층 → NO_ROUTE, E-stop 래치 중에도 중계 동작 | 확인 |
| Nucleo 실기 + 4F 홀 비콘 bench: 디버거 주입 HALL_CALL 2회 → COMPLETED (RF 왕복 확인, RF §12) | 확인 |
| Nucleo 실기: E-stop 래치 중 HALL_CALL 왕복 COMPLETED, 정지 프레임과 공존 (§6 "E-stop과 독립") | 확인 |
| Nucleo 실기: `0x310`/`0x311` 실제 버스 송신, Jetson 수신 | 미시험 (CAN 미연결) |
| 통합: Jetson 미션 → 실제 버튼 누름 → BUTTON_ACK | 미시험 |

---

## 변경 이력

| 버전 | 날짜 | 내용 |
|---|---|---|
| 1.0 초안 | 2026-10-21 | 최초 작성. `0x210`/`0x310` 배치 확정(기존 Jetson 구현과 동일), `0x311` 신규 |
| 1.0 초안 | 2026-10-21 | 펌웨어 구현 반영: §6.1 구현 메모, §10 검증 현황. 프레임 형식 변경 없음 |
| **1.1 초안** | 2026-10-21 | `0x210` byte 5 `repeat` (DOOR_OPEN 열림 반복, 0–15). 펌웨어는 이를 RF COMMAND byte 10으로 그대로 전달한다. §3, §7.2, §9 |
