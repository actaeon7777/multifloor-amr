# Capstone AMR — Isaac Sim 사용 안내

## 바로 임포트할 파일

워크스페이스 루트(`amr_ws`)를 기준으로 Isaac Sim에서 다음 파일을 직접 임포트한다.

```text
src/amr_description/urdf/capstone_amr.urdf
```

이 URDF는 메시를 상대 경로로 참조하므로 `amr_description` 폴더 구조를 유지해야 한다.

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

변경 전 파일은 프로젝트 루트의 `backups/before_v28_update.zip`에 보관했다.

## 권장 URDF Importer 설정

- Robot Type: `Wheeled`
- Base Type: `Mobile`
- Allow Self-Collision: 끔
- Collision From Visuals: 끔 — URDF에 단순 충돌체가 이미 정의되어 있음
- Merge Fixed Joints: 끔 — 빈 `lidar_link` 센서 프레임을 별도 prim으로 유지
- Merge Mesh: 켬
- 출력: `capstone_amr.usd`로 저장

섀시 시각 STL은 597,040 triangles이지만 충돌 형상은 `0.640 x 0.520 x 0.085 m` box를 사용한다. 정밀한 외곽 충돌이 필요하면 나중에 여러 box 또는 저해상도 convex collision으로 교체한다.

중앙 `mounting_plate.stl`은 `base_link` visual에 별도로 포함된다. 크기는
`0.560 x 0.360 x 0.005 m`, Z 범위는 `-0.034~-0.029 m`이다. 기존 섀시
box collision이 이 영역을 이미 포함하므로 판 전용 collision은 중복 생성하지 않는다.
따라서 중앙 개구부는 visual에 표현되지만 현재의 단순 섀시 collision에는 뚫려 있지 않다.

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
Wheel Distance = 0.499 m
Joint order    = left_drive_wheel_joint, right_drive_wheel_joint
```

컨트롤러 출력은 휠 선속도가 아니라 휠 각속도(rad/s)이다.

## 질량 배분

실측한 AMR 전체 질량 `15.0 kg`에 맞춰 다음처럼 배분했다.

| 항목 | 개수 | 개당 질량 | 합계 |
|---|---:|---:|---:|
| `base_link` 나머지 | 1 | 9.8 kg | 9.8 kg |
| WA172E 구동휠 모터 | 2 | 2.2 kg | 4.4 kg |
| 캐스터 조립체 | 4 | 0.2 kg | 0.8 kg |
| **총합** | | | **15.0 kg** |

`base_link`의 9.8 kg에는 배터리 약 4.5 kg, 프레임, 상판, 전장품,
LiDAR 본체와 LiDAR 마운트가 모두 포함된다. 따라서 라이다 링크에 별도 질량을
또 넣지 않는다. 캐스터 한 개의 0.2 kg은 현재 상판 0.05 kg, 스위블 프레임
0.10 kg, 바퀴 0.05 kg으로 임시 분배했다.

## LiDAR 링크와 Isaac Sim 센서

CAD의 `AMR_MASTER v28.step`에서 재확인한 고정 링크 구조가 URDF에 포함되어 있다.
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

`mounting_plate.stl`, `lidar_mount.stl`, `lidar_body.stl`은 STEP 조립 좌표와
부품 로컬 좌표를 각각 유지해 배치했으며 URDF와 웹 프리뷰가 동일한 값을 사용한다.

`amr_master.stl`을 URDF visual에 추가하면 섀시와 움직이는 휠/캐스터가 중복되므로 사용하지 않는다.
이 참조 메시도 v28 전체 조립체로 갱신했다. 전체 CAD에는 BREP 유효성 검사를
통과하지 못하는 구성 형상이 있어 참조 시각화 용도로만 사용한다. 이번에 교체한
두 부품은 개별 BREP 유효성 및 폐쇄 메시 검사를 통과했다.
전체 참조 STL 변환에서 삼각화되지 않은 CAD 면 24개가 생략되었다. URDF는 이 참조
파일을 사용하지 않으며, 교체한 두 부품의 메시에는 생략된 면이 없다.

## 확인이 남은 물리 파라미터

- 캐스터 0.2 kg의 링크별 질량 분배와 각 부품 관성은 임시값
- `base_link`의 무게중심과 관성은 현재 섀시 bounding box 근사값
- 실제 라이다 모델의 거리, 각분해능, 회전/스캔 주파수 확인 필요
- 휠 메시 파일명은 WA171이지만 질량 근거 주석은 WA172E 데이터시트 기준
- 실제 무게중심, 관성 텐서, 모터 최대 토크와 최대 각속도 확인 필요
