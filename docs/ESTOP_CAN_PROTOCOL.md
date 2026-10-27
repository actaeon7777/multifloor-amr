# AMR E-stop CAN 프로토콜 변경 명세 (Jetson 브릿지용)

- 문서 버전: 1.1 (2026-10-21) — 버튼 복귀 시 자동 해제 방식으로 변경
  - 1.0 대비: 수동 reset이 없어지고 해제 확정 시간이 20 ms에서 200 ms로 늘었다. 재가동 승인은 Jetson 책임이다.
  - Jetson 운용 변경 (2026-10-24): 운영자 재개 승인을 없앴다. 브릿지는 STM32 해제를 보면 스스로 `ARMING`으로 가고,
    재가동 방지는 "E-stop 때 목표·명령 캐시를 버리고 새 명령으로만 움직인다"로 한다. 아래 §6의 승인 관련 문구는 이 변경으로 대체됐다.
- 대상 펌웨어: `C:\STM32\AMR` (NUCLEO-F446RE), E-stop CAN 연동 빌드 `build/EstopDebug/Test_v2.elf`
- 대상 독자: Jetson ROS 2 CAN 브릿지 개발자
- 상태 프레임 프로토콜 버전: **1**

이 문서는 STM32 하위제어기에 하드웨어 E-stop을 추가하면서 바뀐 CAN 동작을 정리한 것이다. 기존 프레임의 ID와 페이로드 형식은 바뀌지 않았다. 바뀐 것은 **신규 상태 프레임 `0x202` 추가**와 **기존 `0x201` 주행 명령의 수용 조건**이다.

### 역할 분담

| 주체 | 책임 |
|---|---|
| STM32 | 버튼이 눌리거나 단선되면 즉시 ODrive를 정지시키고 `0x202`로 알린다. 버튼이 원위치로 돌아와 LOW가 200 ms 유지되면 **STM32 쪽 정지를 자동으로 해제**하고 알린다. |
| Jetson | E-stop을 받으면 자율주행을 해제하고 목표와 명령 캐시를 버린다. STM32가 해제를 알리면 운영자 승인 없이 재무장하지만, **이전 운행은 이어가지 않고** 새 목표·새 조작으로만 움직인다. |

STM32의 자동 해제는 "버튼이 원위치에 있다"는 뜻일 뿐 "이전 운행을 이어도 된다"는 뜻이 아니다. 브릿지가 해제 알림만 보고 이전 운행을 자동으로 이어가면 E-stop 규격(해제가 재가동을 일으키면 안 됨)을 위반한다.

---

## 1. 변경 요약

| 항목 | 변경 전 | 변경 후 | 브릿지 영향 |
|---|---|---|---|
| `0x202` E-stop 상태 | 없음 | STM32 → Jetson 신규 프레임, 100 ms 주기 + 상태 변화 시 | **디코더 추가 필요** |
| `0x201` 수용 조건 | 항상 수용 | E-stop 래치 중, ODrive 준비 전, NaN/Inf이면 **거절** | 명령이 조용히 무시될 수 있음 |
| `0x201` 재개 조건 | 명령이 다시 오면 즉시 재개 | **준비 완료 후 `(v=0, w=0)` 1프레임을 먼저 받아야** 이후 명령 수용 | 재개 시 영속도 handshake 필요 |
| `0x201` 300 ms timeout | 목표를 0으로 하고, 명령이 다시 오면 자동 재개 | 목표를 0으로 하고 **gate도 닫음** → 영속도부터 다시 보내야 함 | 명령 공백 후 재개 절차 변경 |
| 거절된 `0x201` | — | timeout 타이머를 갱신하지 않음 | 거절 중에는 정지 유지 |
| `0x301`/`0x302` IMU | 항상 송신 | **E-stop 래치 중 송신 중단**, 해제 시 쌓인 샘플 폐기 (§7) | IMU 공백과 `seq` 건너뜀 처리 |
| ODrive 정지 프레임 | 없음 | 래치 중 `0x002`/`0x022` Estop, `0x007`/`0x027` IDLE 반복 | 버스에서 관측 가능, 브릿지 처리 불필요 |
| E-stop 해제 | — | **버튼이 복귀해 LOW가 200 ms 유지되면 자동 해제**. 별도 reset 명령 없음 | 해제 후 재가동 승인은 브릿지/상위 계층이 담당 |

---

## 2. 버스 파라미터 (변경 없음)

| 항목 | 값 |
|---|---|
| 인터페이스 | STM32F446 CAN1, PB8 RX / PB9 TX |
| 비트레이트 | 500 kbit/s (APB1 42 MHz, prescaler 6, 1+11+2 tq, sample point 85.7 %) |
| 프레임 | Classic CAN, 표준 11-bit ID, DATA 프레임 |
| 바이트 순서 | 모든 다바이트 필드 **little-endian** |
| float | IEEE-754 binary32 |
| 자동 재전송 | 켜짐. Bus-off 시 펌웨어가 200 ms 간격으로 재시작 |

---

## 3. CAN ID 목록

| 방향 | ID | DLC | 내용 | 변경 |
|---|---|---|---|---|
| Jetson → STM32 | `0x201` | 8 | float32 `v_mps` (byte 0–3), float32 `w_radps` (byte 4–7) | 형식 유지, **수용 조건 변경** (§5) |
| STM32 → Jetson | `0x202` | 8 | E-stop 상태 (§4) | **신규** |
| STM32 → Jetson | `0x301` | 8 | 가속도 int16 x/y/z + uint16 seq | 형식 유지, 래치 중 중단 (§7) |
| STM32 → Jetson | `0x302` | 8 | 자이로 int16 x/y/z + uint16 seq | 형식 유지, 래치 중 중단 (§7) |
| STM32 → ODrive 0/1 | `0x002` / `0x022` | 0 | CANSimple Estop (cmd 0x02) | 신규 |
| STM32 → ODrive 0/1 | `0x007` / `0x027` | 4 | Set_Axis_State IDLE: `01 00 00 00` | 기존 명령 추가 사용 |
| ODrive 0/1 → STM32 | `0x001` / `0x021` | 8 | Heartbeat | 기존 |

STM32의 수신 필터는 `0x201`과 ODrive 노드 0/1의 heartbeat, encoder, bus V/I만 받는다. **Jetson이 그 밖의 ID를 보내도 STM32는 받지 않는다.**

### ID 충돌 주의

ODrive CANSimple ID는 `(node_id << 5) | cmd`이다. 이 공식으로 보면 커스텀 ID가 다음 ODrive 노드 영역과 겹친다.

| 커스텀 ID | 겹치는 CANSimple 해석 |
|---|---|
| `0x201` | node 16, cmd 0x01 (Heartbeat) |
| `0x202` | node 16, cmd 0x02 (**Estop**) |
| `0x301` / `0x302` | node 24, cmd 0x01 / 0x02 (**Estop**) |

현재 ODrive는 node 0/1이라 충돌하지 않는다. **ODrive를 node 16이나 24로 설정하면 안 된다.** 그러면 STM32 상태 프레임과 IMU 프레임이 그 ODrive에 Estop 명령으로 해석된다. 버스에 다른 장치를 추가할 때는 전체 ID 목록을 다시 점검할 것.

---

## 4. `0x202` E-stop 상태 프레임 (버전 1)

### 4.1 페이로드

| byte | 형식 | 필드 | 설명 |
|---|---|---|---|
| 0 | uint8 | `version` | 항상 `1`. 다르면 프레임 전체를 무시하고 상태 미수신으로 취급 |
| 1 | uint8 | `flags` | §4.2 |
| 2–3 | uint16 LE | `trip_count` | 부팅 후 E-stop 트립 횟수 (하위 16비트) |
| 4–5 | uint16 LE | `sequence` | 상태 프레임 **enqueue** 횟수 (하위 16비트) |
| 6 | uint8 | `release_count` | 래치 해제 횟수 (하위 8비트). 자동 해제마다 1 증가 |
| 7 | uint8 | `reset_rejected` | 거절된 수동 reset 요청 횟수 (하위 8비트). 디버거 진단용으로 평소에는 0 |

byte 6/7의 배치는 1.0과 같다. 이전 이름은 `reset_accepted`/`reset_rejected`였다. 레이아웃이 그대로라 `version`은 1로 유지한다.

- 모든 카운터는 wrap하고 MCU가 재부팅되면 0으로 돌아간다.
- `sequence`는 송신 완료가 아니라 송신 대기열 등록(enqueue) 횟수다. 버스 오류로 abort된 프레임도 번호를 소비하므로 **번호가 비는 것만으로 수신 누락이라고 단정할 수 없다.** 증가 여부만으로 STM32가 살아 있는지 판단하고, 누락 판정에는 쓰지 말 것.

### 4.2 flags

| bit | mask | 이름 | 1일 때 의미 |
|---|---|---|---|
| 0 | `0x01` | `RAW` | PC10 입력 HIGH: 버튼 눌림 **또는 배선 단선** (회로상 구별 불가) |
| 1 | `0x02` | `DEBOUNCED` | 눌림 상태이거나, 해제가 아직 확정되지 않음 (LOW가 200 ms 미만 유지) |
| 2 | `0x04` | `LATCHED` | STM32 정지 래치. 눌림/단선 순간 1, 버튼 복귀 후 LOW가 200 ms 유지되면 자동으로 0 |
| 3 | `0x08` | `INHIBIT` | STM32가 ODrive 구동 명령을 차단 중 |
| 4 | `0x10` | `ODRV_IDLE` | 두 ODrive의 최신 heartbeat가 모두 IDLE. 래치 중에는 트립 이후에 받은 heartbeat일 때만 1 |
| 5 | `0x20` | `BUS_OFF` | STM32 CAN이 bus-off를 관측 (bus-off 중에는 송신이 안 되므로 실제로 보일 일은 드묾) |
| 6 | `0x40` | `ODRV_HB_OK` | 두 ODrive heartbeat를 모두 500 ms 이내에 받음 |
| 7 | `0x80` | `DRIVE_ENABLED` | 주행 gate가 열림: ODrive 준비 후 영속도 명령을 받았고 아직 timeout이 나지 않음 |

판정 규칙:

- **정지 상태 = `LATCHED | INHIBIT` 중 하나라도 1.** 해제 직후 아주 짧은 순간 `LATCHED=0`, `INHIBIT=1`일 수 있다. 두 비트를 OR로 판정할 것.
- **STM32 해제(`stopped=0`)는 이전 운행 재개가 아니다.** 브릿지는 E-stop을 볼 때마다 명령 캐시를 버리고, 해제 뒤 `(0,0)`부터 다시 시작한다 (§6).
- `trip_count`가 늘었다면 그사이에 E-stop이 있었다는 뜻이다. 매우 짧은 누름이나 순간 단선은 `0x202`에서 `stopped=1`이 보이기 전에 해제될 수 있으므로, 브릿지는 **`trip_count` 증가도 E-stop 발생으로 처리**해야 한다. 래치는 최소 200 ms 유지되므로 100 ms 주기 상태 프레임으로 대부분 관측되지만, 프레임이 누락될 수 있다.
- `ODRV_IDLE=1`은 ODrive가 IDLE 상태를 보고했다는 뜻일 뿐이다. 바퀴 속도가 0이라는 확인이 아니다.
- 부팅 직후 약 200 ms 동안은 버튼이 눌리지 않아도 `DEBOUNCED=1`일 수 있다 (해제 확정 대기).

### 4.3 송신 시점

| 상태 | 송신 |
|---|---|
| 평상시 | flags가 바뀌면 즉시, 그 외에는 100 ms마다 |
| 래치 중 | 약 100 ms마다 두 묶음: `[0x002, 0x022, 0x202]` → 송신이 끝나면 바로 `[0x007, 0x027, 0x202]`. **두 `0x202`가 수 ms 간격으로 붙어서 올 수 있다** |

- 정지 프레임 묶음은 송신 mailbox 3개를 모두 쓴다.
  - **래치 중:** 묶음을 보낼 차례에 다른 프레임(IMU, 엘리베이터 `0x310`/`0x311`)이 mailbox에 남아 있으면 **즉시 abort**한다.
  - **평상시 상태 프레임:** 비워질 때까지 기다리고, 100 ms 넘게 비지 않으면 abort한다.
  - E-stop 프레임이 송신 중이거나 묶음을 기다리는 동안에는 엘리베이터 프레임을 새로 넣지 않는다.
  - 따라서 엘리베이터 중계가 정지 송신을 늦추지 않는다.
- ACK가 없으면(버스 미연결 등) 프레임 하나당 최대 100 ms 기다린 뒤 abort하고 다시 시도한다.
- 따라서 **주기는 보장값이 아니다.** 수신 timeout은 100 ms보다 넉넉하게 잡을 것 (§6).

### 4.4 디코더 예시 (Python, python-can)

```python
import struct

ESTOP_STATUS_ID = 0x202
FLAG_RAW, FLAG_DEBOUNCED, FLAG_LATCHED, FLAG_INHIBIT = 0x01, 0x02, 0x04, 0x08
FLAG_ODRV_IDLE, FLAG_BUS_OFF, FLAG_ODRV_HB_OK, FLAG_DRIVE_ENABLED = 0x10, 0x20, 0x40, 0x80

def decode_estop_status(msg):
    """Return a dict, or None if the frame is not a valid v1 status frame."""
    if (msg.arbitration_id != ESTOP_STATUS_ID or msg.is_extended_id
            or msg.is_remote_frame or msg.dlc != 8):
        return None
    version, flags, trip, seq, rst_ok, rst_ng = struct.unpack('<BBHHBB', bytes(msg.data))
    if version != 1:
        return None
    return {
        'raw':           bool(flags & FLAG_RAW),
        'debounced':     bool(flags & FLAG_DEBOUNCED),
        'latched':       bool(flags & FLAG_LATCHED),
        'inhibit':       bool(flags & FLAG_INHIBIT),
        'odrv_idle':     bool(flags & FLAG_ODRV_IDLE),
        'bus_off':       bool(flags & FLAG_BUS_OFF),
        'odrv_hb_ok':    bool(flags & FLAG_ODRV_HB_OK),
        'drive_enabled': bool(flags & FLAG_DRIVE_ENABLED),
        'stopped':       bool(flags & (FLAG_LATCHED | FLAG_INHIBIT)),
        'trip_count': trip, 'sequence': seq,
        'release_count': rst_ok, 'reset_rejected': rst_ng,
    }
```

예시 프레임 (실제 보드 관측값과 일치):

| 상황 | data (hex) | 해석 |
|---|---|---|
| 정상 대기 (ODrive 미연결) | `01 00 ...` | 정지 아님, gate 닫힘 |
| 눌림, 래치 | `01 0F 01 00 xx xx 00 00` | raw/debounced/latched/inhibit, trip 1회 |
| 버튼 복귀 후 200 ms 이내 | `01 0E ...` | raw만 0, 해제 확정 대기 → **여전히 정지** |
| 자동 해제 후 | `01 00 01 00 xx xx 01 00` | STM32 정지 해제(release 1회), gate는 아직 닫힘 (bit7=0). Jetson은 `(0,0)`부터 다시 보낸다 |

---

## 5. `0x201` 주행 명령 수용 규칙

### 5.1 거절 조건

STM32는 아래 중 하나라도 해당하면 `0x201`을 **응답 없이 버린다**. 거절된 프레임은 300 ms timeout 타이머도 갱신하지 않는다.

1. E-stop 래치 중이거나 CAN 구동이 차단됨 (`LATCHED | INHIBIT`)
2. `v` 또는 `w`가 NaN 또는 Inf
3. ODrive 두 축의 초기화가 끝나지 않음 (Closed Loop 진입 전, 또는 fault 상태)
4. gate가 닫혀 있는데(`DRIVE_ENABLED=0`) 받은 명령이 영속도가 아님

DLC가 8이 아니거나 RTR 프레임이거나 확장 ID인 `0x201`은 예전과 같이 무시한다.

### 5.2 gate (영속도 handshake)

- gate가 닫혀 있을 때 `v == 0.0 && w == 0.0`인 프레임을 받으면 gate가 열린다 (`DRIVE_ENABLED=1`). 단, 조건 1–3이 모두 해소된 상태여야 한다.
- gate가 열린 뒤에는 0이 아닌 명령도 받는다.
- gate가 닫히는 경우:
  - E-stop 트립
  - E-stop 해제 직후 ODrive 재초기화 시작
  - 명령 공백 300 ms 초과 (거절된 프레임은 공백으로 친다)
  - CAN bus-off 복구
  - ODrive fault 후 재초기화 (fault 동안 명령이 거절되므로 300 ms 뒤 timeout으로도 닫힘)
- **`-0.0`은 0으로 취급된다** (IEEE 비교). 아주 작은 값(예: `1e-9`)은 0이 아니라서 거절된다.

### 5.3 브릿지가 ODrive 준비를 아는 방법

`0x202`에는 "ODrive Closed Loop 준비 완료" 비트가 없다. 브릿지는 다음 방법으로 준비를 확인한다.

- 정지가 아니고(`stopped=0`) gate가 닫혀 있으면(`DRIVE_ENABLED=0`), 평소 명령 주기로 `(0, 0)`을 계속 보낸다.
- 다음 `0x202`에서 `DRIVE_ENABLED=1`이 보이면 그때부터 새 명령을 보낸다.
- `ODRV_HB_OK=1`은 heartbeat가 살아 있다는 뜻일 뿐이고, 준비 완료와는 다르다.

---

## 6. 브릿지 권장 동작

### 6.1 상태 머신

```
          E-stop 이벤트 (stopped=1, 또는 trip_count 증가)
          / 0x202 수신 timeout / version 오류  ── 어느 상태에서든 ──┐
                                                                 ▼
[UNKNOWN] ──0x202 수신──────────────────────────────▶ [ESTOP]
                                                  자율주행 해제, 명령 차단,
                                                  큐/goal 폐기, (0,0)도 보내지 않음
                                                        │ stopped=0 (버튼 복귀, STM32 자동 해제)
                                                        ▼
                                                  [ARMING]  (0,0)만 송신, 받아 둔 명령 폐기
                                                        │ drive_enabled=1
                                                        ▼
                                                  [READY] 재무장 이후 상위 계층이 새로 낸 명령만 전달
                                                        │ drive_enabled=0 (명령 공백 300 ms/fault/bus-off)
                                                        └──▶ [ARMING]
```

- 부팅 직후 `UNKNOWN`에서는 아무것도 보내지 않는다. 첫 정상 `0x202`를 받으면 `ARMING`으로 간다.
- `READY`에서 `drive_enabled=0`이 됐다고 해서 E-stop은 아니다. E-stop 판정은 `stopped`와 `trip_count`로 한다.

### 6.2 필수 사항

1. **E-stop 관측:** `stopped=1` 또는 `trip_count` 증가를 보면 자율주행을 해제하고 진행 중인 목표를 중단한다. 해제 뒤에도 그 목표는 다시 출발하지 않는다. 이것이 이 시스템의 **재가동 방지 장치**다.
2. **`0x202` 수신 timeout:** 마지막 유효 `0x202` 이후 일정 시간(제안값 **500 ms**)이 지나면 E-stop과 똑같이 처리한다.
   - 이 값은 제안값이다. 버스 부하와 래치 중 송신 지연을 시험한 뒤 확정한다.
3. **정지 중 명령 차단:** `ESTOP`과 `UNKNOWN`에서는 `0x201`을 보내지 않는다. 래치 중에는 STM32도 거절하지만, STM32 해제 뒤에는 브릿지만이 명령을 막는다.
4. **정지 후 큐 폐기:** 정지를 관측하면 브릿지 내부의 명령 큐와 캐시(마지막 `cmd_vel`, 내비게이션 goal 등)를 비운다. 재개 후 이전 비영속도 목표를 다시 보내면 안 된다.
5. **영속도는 출발이 아니다:** ARMING에서 `(0,0)`을 보내는 것은 STM32 gate를 여는 handshake일 뿐이다. 실제 이동은 재무장 이후 상위 계층(내비게이션, 텔레옵)이 **새로** 낸 명령으로만 재개한다.
6. **명령 주기:** 운행 중에는 300 ms보다 충분히 짧은 주기로 `0x201`을 보낸다. 공백이 300 ms를 넘으면 STM32가 gate를 닫고, ARMING부터 다시 해야 한다.
7. **NaN/Inf 송신 금지:** 거절되어 정지가 유지된다.

### 6.3 ROS 2 노출 권장

`raw`, `debounced`, `latched`, `inhibit`, `odrv_idle`, `odrv_hb_ok`, `drive_enabled`, `trip_count`, `release_count`, `reset_rejected`, 마지막 수신 시각을 각각 노출한다. 최소한 다음 넷은 구분해서 표시한다.

- **버튼 상태:** `raw`
- **STM32 정지:** `latched | inhibit`
- **브릿지 상태:** `ESTOP` / `ARMING` / `READY`
- **운행 가능:** `drive_enabled`

### 6.4 MCU 재부팅 감지

카운터가 부팅 시 0으로 초기화되므로, `sequence`가 작은 값으로 되돌아가거나 `trip_count`/`release_count`가 감소하면 STM32 재부팅을 의심할 수 있다. 다만 wrap과 구별되지 않으므로 보조 지표로만 쓴다. 재부팅 뒤에도 gate는 닫힌 상태로 시작한다.

- 버튼을 누른 채 부팅하면 처음부터 `LATCHED=1`이다.
- **운행 중 STM32가 재부팅되면 `0x202`가 잠시 끊긴다.** 브릿지는 이를 수신 timeout(또는 `sequence` 불연속)으로 보고 `ESTOP`으로 가서 목표를 중단한 뒤, 상태가 돌아오면 `ARMING`부터 다시 한다.

---

## 7. IMU 프레임(`0x301`/`0x302`) 동작 변경

- **래치 중에는 IMU 프레임을 보내지 않는다.** 정지 프레임 송신이 버스를 우선 사용하기 때문이다. E-stop 동안 IMU 토픽에 공백이 생긴다.
- IMU 샘플링은 래치 중에도 계속되지만 STM32 내부 큐(16개)가 가득 차면 새 샘플을 버린다.
- **해제 순간 STM32가 이 큐를 비운다.** 그래서 해제 뒤에 나오는 IMU 프레임은 해제 이후의 새 샘플이다. 단, 해제 처리 직전에 측정된 샘플 1개가 섞일 수 있다 (100 ms 주기).
- 래치 동안의 샘플은 전송되지 않으므로 해제 후 첫 프레임의 `seq`는 크게 건너뛴다. 브릿지는 이 불연속을 오류가 아닌 정상 동작으로 처리하고, 필터나 적분기를 다시 초기화할지 결정해야 한다.

---

## 8. 정지와 해제 시퀀스 (참고)

```
PC10 HIGH (버튼 눌림/단선)
 └─ EXTI ISR: latched=1, inhibit=1, stop_pending=1  (CAN HAL 호출 없음)
     └─ CAN_Task (다음 루프. 보통 수 ms, 대기 경로에 따라 10 ms 이상 가능 — 실측 전):
          대기 중 송신 전부 abort, 목표 속도 0, gate 닫힘
          [0x002, 0x022, 0x202] → [0x007, 0x027, 0x202] → ~100 ms 후 반복 …

버튼 복귀 (또는 단선 복구)
 └─ 인터럽트로 LOW 감지 → 바운스가 멎고 LOW가 200 ms 연속 유지
     (그사이 HIGH나 새 엣지가 오면 200 ms를 처음부터 다시 셈)
     └─ latched=0 (release_count++)  ← 자동 해제, reset 명령 없음
         └─ CAN_Task: inhibit=0, 정지 프레임 중단, IMU 큐 비움, 0x202로 해제 알림, ODrive 재초기화 시작
              heartbeat 대기 → Clear_Errors → 속도 모드 설정 → Closed Loop → 준비 완료
              (모터는 영속도 유지 상태로 전류가 들어감. 이동은 하지 않음)
              → Jetson (0,0) 송신 → drive_enabled=1 → 새 명령 수용
```

- 하나의 E-stop 래치 동안에는 `trip_count`가 1만 증가한다 (래치 중 바운스는 추가 트립이 아님).
- STM32가 정지 이벤트를 CAN_Task에서 처리하기 전에는 해제되지 않는다. 아무리 짧게 눌러도 정지 프레임 묶음은 최소 한 번 송신 시도된다.
- ODrive CANSimple Estop은 축을 오류 상태로 만든다. 해제 뒤 재초기화 단계에서 Clear_Errors를 보낸다.
- **단선이 복구돼도 자동 해제된다.** 커넥터 접촉 불량이 있으면 정지와 해제가 반복될 수 있다. 그때마다 목표가 중단되므로 `trip_count`가 자주 늘면 배선을 점검할 것.

---

## 9. 범위와 한계

- **Jetson → STM32 reset/arm 명령은 없고 필요하지도 않다.** STM32 정지는 버튼이 복귀하면 자동으로 해제된다. 재무장은 Jetson 내부 상태로만 관리한다. 이 문서에 없는 명령을 가정하지 말 것.
- 펌웨어에는 수동 reset 모드(`ESTOP_AUTO_RELEASE=0`)가 컴파일 옵션으로 남아 있다. 현재 빌드에서는 쓰지 않는다.
- STM32의 CAN TXOK는 버스상 송신 완료를 뜻할 뿐, 특정 ODrive나 Jetson이 처리했다는 ACK가 아니다.
- Estop/IDLE 요청은 구동 해제 요청이다. 기계적 제동이나 물리적 정지 확인과 같지 않다.
- CAN이 끊기면 정지 프레임은 도달하지 않는다. 이 경우 ODrive watchdog과 별도 하드웨어 정지 경로가 최종 안전층이며, 그 설정과 동작은 이번 변경의 검증 범위가 아니다.
- **하위제어기 MCU 워치독 (IWDG, 약 2 s, LSI 오차로 1.4–3.8 s):**
  - E-stop 폴링 태스크가 감시한다. CAN_Task, IMU_Task, RF_Task 중 하나라도 1 s 넘게 멈추면 IWDG 갱신을 멈추고 MCU를 리셋한다. 감시 태스크 자신, 스케줄러, 인터럽트가 멈춰도 마찬가지로 리셋된다.
  - 리셋 중과 재부팅 직후에는 `0x202`가 끊긴다. **Jetson은 이를 수신 timeout으로 보고 E-stop과 똑같이 처리해야 한다** (§6.2, §6.4).
  - 재부팅 후 주행 gate는 닫힌 채로 시작한다.
  - 직전 리셋이 워치독 때문이었는지는 RAM 변수 `sys_iwdg_reset`/`sys_reset_csr`로 알 수 있다. CAN으로는 알리지 않는다.
- ODrive 실장 펌웨어 버전과 Estop(cmd 0x02) 지원 여부는 아직 확인하지 못했다.
  - 참고: https://docs.odriverobotics.com/v/latest/manual/can-protocol.html

---

## 10. 검증 현황 (2026-10-21)

| 항목 | 상태 |
|---|---|
| 펌웨어 빌드, host 시험: 자동 해제(200 ms 창, 바운스 시 재시작, pending 엣지, 단선 복구, 눌린 채 부팅, tick wrap), 정지 처리 전 해제 금지, 해제 후 상태 프레임, 재가동 gate, 재트립 | 통과 |
| host 시험: 수동 reset 모드 회귀 (`ESTOP_AUTO_RELEASE=0`) | 통과 |
| 실제 보드, 자동 해제 펌웨어, CAN 미연결: 눌림 → 래치(flags 15), 떼면 reset 없이 자동 해제(flags 0, release 1, 정지 프레임 중단), 재트립 → 재해제(trip 2, release 2) | 통과 |
| 실제 보드, 자동 해제 + 엘리베이터 RF 펌웨어: 버튼을 뺀(단선) 상태로 부팅 → 부팅 샘플로 래치(boot_level 1, poll_latch 1, flags 15), 80 s 넘게 유지 | 확인 |
| 실제 보드, 엘리베이터 RF 펌웨어 (mailbox hold/즉시 abort 포함), CAN 미연결: 누름 → 래치(flags 15, EXTI), 떼면 자동 해제(flags 0, 정지 프레임 중단), 재트립 → 재해제(trip 2, release 2). 래치 중 4F 비콘 HALL_CALL 왕복 COMPLETED, 정지 프레임 송신 대기열 등록 계속 | 통과 (2026-10-21) |
| candump로 `0x002/0x022`, `0x007/0x027`, `0x202` 실제 수신 확인 | **미완료** |
| ODrive 실제 정지, heartbeat IDLE, 정지 지연 측정 | **미완료** |
| Jetson 브릿지 디코드, 수신 timeout, ARMING 절차 | 구현·단위/ROS 시험 통과 (2026-10-21, `cmd_vel_to_can`·`estop_protocol.py`). `sequence` 불연속(뒤로 감, 64 넘는 점프)도 E-stop으로 처리한다. 2026-10-24 운영자 승인 단계 제거. **실차 미시험** |
| 버스 부하, 장시간, bus-off 복구 중 래치 보존 | **미완료** |

통합 시험 시 브릿지 쪽에서 확인할 것:

1. `0x202`가 DLC 8, version 1로 100 ms 안팎 주기로 수신되는지
2. 버튼을 누르면 `stopped=1`이 되고, 자율주행 해제, `0x201` 차단, 큐 폐기가 일어나는지
3. 버튼을 떼면 `stopped=0`이 되고, 이전 목표가 다시 출발하지 않는지
4. 해제 뒤 `(0,0)` → `drive_enabled=1` → 새 명령으로만 움직이는지
5. 아주 짧게 눌렀을 때 `trip_count` 증가만으로도 `ESTOP`으로 가는지
6. 명령 공백이 300 ms를 넘은 뒤 ARMING부터 다시 하는지
7. 상태 수신 timeout(500 ms 제안)에 정지로 전환되는지
