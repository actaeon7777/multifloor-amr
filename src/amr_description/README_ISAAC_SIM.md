# Capstone AMR — Isaac Sim 사용 안내

## 바로 임포트할 파일

워크스페이스 루트(`amr_ws`)를 기준으로 Isaac Sim에서 다음 파일을 직접 임포트한다.

```text
src/amr_description/urdf/capstone_amr.urdf
```

이 URDF는 메시를 상대 경로로 참조하므로 `amr_description` 폴더 구조를 유지해야 한다.

## v119 형상 업데이트 (2026-10-20)

`AMR_MASTER_Linear v119.step`을 반영했다. 상세는 [`reports/UPDATE_V119.md`](reports/UPDATE_V119.md).

- 하우징 4조각(`housing.stl`): 외곽 X ±328, Y ±278, Z −55~80 mm, 구동휠 위 펜더는 Z 100 mm까지.
  차체 길이가 0.640 → **0.656 m**가 됐다. 폭은 구동휠 바깥면 기준 0.574 m로 같다.
- Orbbec Gemini 2 깊이 카메라: 하우징 앞코에 있고 **12° 위로 들려 있다**. `camera_link` 프레임을 추가했다(아래 절).
- 후면 XT90 커넥터는 `chassis.stl`에 합쳤다.
- 충돌체: 하부 프레임 box와 서스펜션 box 2개를 하우징 box 1개와 펜더 box 2개로 바꿨다.
- 하우징 상면(80 mm)과 펜더(99.9 mm)는 LiDAR 스캔 평면(108.605 mm)보다 낮다. 카메라 시야(91° × 66°)도 하우징에 가리지 않는다.
- 전체 질량을 15.0 → **18.0 kg**로 올렸다(2026-10-20 지정값). 늘어난 3.0 kg은 모두 `base_link`(9.8 → 12.8 kg)에 넣었다.
- Nav2 지역 costmap footprint를 X ±0.320 → ±0.328 m로 맞췄다(`navigation.yaml`, `rl/amr_rl/scan.py`).

## v80 형상 업데이트 (2026-10-16)

`AMR_MASTER_Linear v80.step` 전체 조립체를 반영했다. 상세 비교는
[`reports/UPDATE_V80.md`](reports/UPDATE_V80.md).

- 구동휠 간격(윤거) 0.499 → **0.529 m**. 휠 위치는 좌우 각 15 mm 바깥으로 이동했고 높이·전후 위치는 같다.
- 섀시: 알루미늄 프로파일 하부 프레임(640 × 520 mm, Z −54~1 mm), 2020 기둥 4개와 상부 레일(Z 166 mm까지),
  전방 마스트(Z 656 mm까지)와 그 위의 Z10T(Z 576~754 mm). `chassis.stl`과 `z10t.stl`로 메시화했다.
- 구동휠 서스펜션: MGN12 리니어 가이드·모터 브래킷(`drive_suspension.stl`). **이 모델에서는 강체로 고정**했다.
- v28의 중앙 마운팅 플레이트는 v80에 없어 `mounting_plate.stl`과 해당 visual을 뺐다.
- LiDAR 마운트·본체, 캐스터 4개, 구동휠 메시의 위치·체적은 v28과 같다.
- 질량 15.0 kg와 `base_link` 관성 근사는 v28 값을 유지했다. v80 실물을 다시 잰 값이 아니다.
- 2020 기둥 4개가 LiDAR 스캔 평면(Z 108.605 mm)을 지난다. 실물 스캔에는 약 ±22~30°, ±149~157° 방향
  0.27~0.28 m 거리에 차체가 보인다. 실차는 `amr_stack`의 `scan_body_filter`가 이 점과, 기둥 모서리를 스친 빔이 0.7 m 이내에
  내는 중간 거리 점(기둥 방향 ±3°)을 지운다.
  시뮬레이션 LiDAR는 자기 충돌체를 무시하므로 기둥 너머가 가려지는 것은 재현하지 않는다.

## v28 형상 업데이트 (2026-09-28)

`AMR_MASTER v28.step`, `LiDAR Mount v3.step`, `Mounting_Plate v3.step`을 반영했다.
v23/v28의 52개 조립 항목에서 배치, 외곽, 체적을 비교했으며 변경된 부품은
플레이트와 LiDAR 마운트이다. 별도 제공된 두 STEP의 형상도 v28 조립체와 대조했다.

- 플레이트: 외곽 560 × 360 × 5 mm 유지. 중앙 80 × 80 mm 개구부와
  앞뒤로 연결된 40 × 35 mm 확장부를 실제 메시로 반영했다.
- LiDAR 마운트: 80 × 80 × 115 mm. 하단이 5 mm 연장되었고 상부 양측 벽이 추가됐다.
  기존 4개 카운터보어 체결 구멍도 메시에서 유지했다. 충돌체는 5개 box로 갱신했다.
- 장착 위치, LiDAR 센서 프레임, 구동휠·캐스터 설정은 기존과 같다.
- 질량 15.0 kg 및 관성 근사는 기존 값이다. 수정 후 질량을 새로 측정한 값은 아니다.

(이 플레이트는 v80에서 제거됐다.)

## 권장 URDF Importer 설정

- Robot Type: `Wheeled`
- Base Type: `Mobile`
- Allow Self-Collision: 끔
- Collision From Visuals: 끔 — URDF에 단순 충돌체가 이미 정의되어 있음
- Merge Fixed Joints: 끔 — 빈 `lidar_link` 센서 프레임을 별도 prim으로 유지
- Merge Mesh: 켬
- 출력: `capstone_amr.usd`로 저장

섀시 시각 STL은 287,840 triangles이지만 충돌 형상은 CAD 부품 외곽에서 만든 box들이다.

| 충돌 box | 크기 (m) | 중심 (m) |
|---|---|---|
| 하우징 (하부 프레임·서스펜션·카메라 포함) | 0.656 × 0.556 × 0.135 | (0, 0, 0.0125) |
| 펜더 ×2 | 0.228 × 0.098 × 0.020 | (0, ±0.229, 0.090) |
| 2020 기둥 ×4 | 0.020 × 0.040 × 0.200 | (0.270 / −0.260, ±0.1326, 0.066) |
| 상부 측면 레일 ×2 | 0.510 × 0.040 × 0.020 | (0.005, ±0.1326, 0.156) |
| 전방 가로대 | 0.040 × 0.3107 × 0.020 | (0.240, 0, 0.176) |
| 마스트 | 0.020 × 0.040 × 0.470 | (0.250, 0, 0.421) |
| Z10T 마운트 하부 | 0.0764 × 0.100 × 0.055 | (0.2372, 0, 0.6035) |
| Z10T 본체 | 0.123 × 0.233 × 0.123 | (0.2308, 0, 0.6925) |

하우징 box는 앞코의 곡면, 휠 아치와 상판 옆 어깨(일부 Z 60~80 mm)까지 채운다. 구동휠과 겹치지만
부모–자식 관절 쌍이라 PhysX가 충돌을 검사하지 않는다(Self-Collision 끔).

## 조인트 드라이브

임포트 후 구동휠만 Velocity Drive로 설정한다.

```text
left_drive_wheel_joint
right_drive_wheel_joint
```

- Drive mode: Velocity
- Stiffness: `0`
- Damping: 로봇이 목표 속도를 추종할 때까지 조정
- Max force / Max velocity: 실제 모터 사양으로 입력

다음 8개 캐스터 조인트는 모두 수동이어야 한다.

```text
front_left_caster_swivel_joint
front_left_caster_wheel_joint
front_right_caster_swivel_joint
front_right_caster_wheel_joint
rear_left_caster_swivel_joint
rear_left_caster_wheel_joint
rear_right_caster_swivel_joint
rear_right_caster_wheel_joint
```

캐스터 조인트에는 Drive를 만들지 않거나 `None`으로 설정한다.

## PhysX 마찰 재질

Gazebo의 `<mu1>`/`<mu2>` 태그는 제거했다. Isaac Sim에서 PhysX Material을 만들어 collision prim에 연결한다.

초기 조정값:

| 대상 | Static friction | Dynamic friction | Restitution |
|---|---:|---:|---:|
| 좌우 구동휠 | 1.0 | 1.0 | 0.0 |
| 캐스터 휠 | 0.0~0.1 | 0.0~0.1 | 0.0 |

캐스터가 떨리거나 회전을 방해하면 캐스터 마찰과 swivel joint friction/damping을 낮춘다. 구동휠이 미끄러지면 휠/바닥 재질의 friction combine mode와 마찰값을 함께 확인한다.

## Differential Controller 입력

```text
Wheel Radius   = 0.086 m
Wheel Distance = 0.529 m
Joint order    = left_drive_wheel_joint, right_drive_wheel_joint
```

컨트롤러 출력은 휠 선속도가 아니라 휠 각속도(rad/s)이다.

## 질량 배분

v119 AMR 전체 질량 `18.0 kg`(2026-10-20 지정값, v28 실측은 15.0 kg)를 다음처럼 배분했다. 구동휠과 캐스터는 실측값이다.

| 항목 | 개수 | 개당 질량 | 합계 |
|---|---:|---:|---:|
| `base_link` 나머지 | 1 | 12.8 kg | 12.8 kg |
| WA172E 구동휠 모터 | 2 | 2.2 kg | 4.4 kg |
| 캐스터 조립체 | 4 | 0.2 kg | 0.8 kg |
| **총합** | | | **18.0 kg** |

`base_link`의 12.8 kg에는 배터리 약 4.5 kg, 프레임과 상부 구조, 마스트, Z10T, 하우징, 전장품,
LiDAR 본체와 마운트, Gemini 2가 모두 포함된다. 관성은 v28의 bounding box 근사를 질량만 바꿔 쓴다. 따라서 라이다 링크에 별도 질량을
또 넣지 않는다. 캐스터 한 개의 0.2 kg은 현재 상판 0.05 kg, 스위블 프레임
0.10 kg, 바퀴 0.05 kg으로 임시 분배했다.

## LiDAR 링크와 Isaac Sim 센서

CAD의 `AMR_MASTER v28.step`에서 확인하고 `AMR_MASTER_Linear v80.step`에서 다시 대조한 고정 링크 구조가 URDF에 포함되어 있다.
기존 센서 프레임의 미세한 CAD 오차를 정리한 위치·회전 기준을 유지했다.

```text
base_link
└─ lidar_mount_joint   xyz="0 0 -0.029"
   └─ lidar_mount_link
      └─ lidar_joint   xyz="0 0 0.137605"
         └─ lidar_link
```

`lidar_link`가 스캔 원점이며 최종 위치는 다음과 같다.

```text
base_link 기준  xyz = 0 0 0.108605 m
base_link 기준  rpy = 0 0 0 rad
지면 기준 높이       = 0.194605 m
```

URDF 임포트 후 RTX LiDAR prim은 `lidar_link` 아래에 만들고 로컬 transform을
`xyz=0 0 0`, `rpy=0 0 0`으로 둔다. 센서 모델명/데이터시트가 정해지면 해당
Isaac Sim 사전 설정 또는 사용자 JSON 설정을 연결한다. 현재 URDF에는 모델을
임의로 가정한 RTX 스캔 프로파일을 넣지 않았다.

라이다 본체 collision은 반지름 `0.0385 m`, 높이 `0.038884 m`의 원통으로,
v3 마운트 collision은 5개의 box로 단순화했다(작은 체결 구멍 생략). 라이다 본체와 마운트의 visual,
collision 및 질량은 `base_link`에 합쳤다. `lidar_mount_link`와 `lidar_link`는
Isaac Sim 센서 부착에 사용할 좌표계만 보존하는 빈 고정 링크이다.

## Xacro 원본

치수를 수정할 때는 다음 파일을 변경한다.

```text
src/amr_description/urdf/capstone_amr.urdf.xacro
```

ROS 2 `xacro`가 설치된 환경에서 URDF를 다시 생성한다.

```bash
source /opt/ros/jazzy/setup.bash
xacro src/amr_description/urdf/capstone_amr.urdf.xacro -o src/amr_description/urdf/capstone_amr.urdf
```

웹 프리뷰의 치수를 바꾸면 화면에서 생성되는 Xacro도 함께 갱신된다. 그 내용을 위 Xacro 파일에 반영하고 다시 전개해야 한다.

## 메시 구분

- `meshes/visual`: URDF가 실제 사용하는 개별 링크 메시
- `meshes/reference`: CAD 조립 확인용이며 URDF에서 사용하지 않음

`chassis.stl`, `drive_suspension.stl`, `z10t.stl`, `housing.stl`은 STEP 조립 좌표(= `base_link`, mm)로
바로 저장했다. `gemini2.stl`은 `camera_link` 좌표(mm)다. `lidar_mount.stl`, `lidar_body.stl`은 부품 로컬 좌표를 유지한다.

`amr_master.stl`을 URDF visual에 추가하면 섀시와 움직이는 휠/캐스터가 중복되므로 사용하지 않는다.
이 참조 메시는 v119 전체 조립체(STEP 그대로, 중복 배치된 부품 포함)로 갱신했다.
LiDAR 본체 형상은 BREP 유효성 검사를 통과하지 못해 기존 `lidar_body.stl`을 유지했다.

## 깊이 카메라 링크 (Gemini 2)

```text
base_link
└─ camera_joint   xyz="0.320488 0.025 0.061017"  rpy="0 -0.20944 0"
   └─ camera_link
```

- `camera_link`는 왼쪽(+Y) IR 렌즈의 광축 위, 렌즈 앞면에 있다. +X가 광축이고, 피치 −12°는 위를 보는 방향이다.
  지면 기준 높이는 0.147 m다.
- 이 위치는 OrbbecSDK_ROS2의 `camera_link`가 depth(왼쪽 IR) 원점이라는 가정에 따른 것이다.
  depth·color·IR·optical 프레임은 드라이버가 공장 캘리브레이션으로 발행하므로 URDF에는 넣지 않았다.
  드라이버의 `camera_link` 이름을 그대로 쓰면 TF가 이어진다.
- 실차 확인: `ros2 run tf2_ros tf2_echo camera_link camera_right_ir_frame`가 약 (0, −0.050, 0)이면 가정이 맞다.
  (0, +0.050, 0)이면 `camera_y`를 −0.025로 바꾼다.
- 카메라 visual(`gemini2.stl`)은 `camera_link` 좌표계 메시이며 `base_link`에 붙였다. 카메라 collision은 하우징 box 안에 있다.
- Isaac Sim 깊이 카메라 prim은 아직 만들지 않았다. 만들 때는 `camera_link` 아래에 두고, USD 카메라의 −Z 시선을 `camera_link` +X에 맞춘다.

## 확인이 남은 물리 파라미터

- v119 무게중심과 관성. 총질량은 18.0 kg로 맞췄지만 무게중심과 관성은 v28 bounding box 근사다
- 구동휠 리니어 서스펜션의 스프링 상수·행정. 현재는 강체로 고정했다

- 캐스터 0.2 kg의 링크별 질량 분배와 각 부품 관성은 임시값
- `base_link`의 무게중심과 관성은 현재 섀시 bounding box 근사값
- 실제 라이다 모델의 거리, 각분해능, 회전/스캔 주파수 확인 필요
- 휠 메시 파일명은 WA171이지만 질량 근거 주석은 WA172E 데이터시트 기준
- 실제 무게중심, 관성 텐서, 모터 최대 토크와 최대 각속도 확인 필요
