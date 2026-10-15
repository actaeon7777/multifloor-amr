> **이력 문서** · 2026-10-07 정리 시점의 이전 안내입니다. 현재 실행은 [최신 운영 문서](../../../README.md), 검증은 [검증 문서](../../../VALIDATION.md)를 사용합니다.

# Capstone AMR 작업 공간

워크스페이스 내부 `src/amr_description`의 v28 형상, 20개 링크, 19개 관절을 기준으로
Isaac Sim 6.1.0 / PhysX 환경을 구성한다.

ROS 2 공통 실행·기록·시험·Jetson 소스 릴리스 사용법은 [실험 안내](../docs/SIM2REAL.md)를 참고한다.
실물 보정은 후속 단계이며, 현재 모델은 `uncalibrated`로 기록한다. `laser`, `lidar_link`, `imu_link`를 보존한다.

현재 기본 장면은 문·엘리베이터가 있는 상세 4층이다. ROS 제어는
`./scripts/experiment.sh sim`으로 실행하며 조이스틱·EKF·AMCL·navigation을 포함한다.
웹 UI는 [실험 안내](../docs/SIM2REAL.md)의 별도 터미널 명령으로 연다.
LiDAR 거리 질의와 충돌 결과 처리는 기본적으로 C++를 사용한다. 기존 실행 명령으로
자동 빌드되며, 라이브러리·호환 버전·검증은 [C++ LiDAR 안내](../../../../sim/native/README.md)에 있다.
GUI 실행은 렌더링의 동기 자산 로딩 대기를 해제해 센서 주기 지연을 줄인다.
비교 결과와 기존 설정 선택법은 [실험 안내](../docs/SIM2REAL.md)의 GUI 최적화 항목에 있다.

접촉 설정 `v29_compliant_wheel_contact`는 바퀴의 원래 외경·질량·관성·모터 제한을
유지하며 둥근 타이어 모서리와 접촉 탄성·감쇠로 진동을 줄인다. 상세 바닥은 단일
상자 충돌체로 처리한다. 현재 결과는 `reports/floor4_stability.json`과
`reports/floor4_manual.json`에서 확인한다. 실물 타이어 보정값은 아니다.

2026-10-03 초기 설치 및 `v28_physx_si_resistance_v2` 검증 기록:

- NVIDIA 공식 ZIP 체크섬 일치, 독립 설치 및 실행 확인
- 최신 원본 21개 파일 보존, Xacro 전개 일치, 20개 링크·19개 관절 검사 통과
- PhysX CPU dynamics 200 Hz, RTX GPU 렌더링 확인
- 런타임 총질량 15.000002 kg, 토크 제한 12.357 Nm, 회전속도 제한 18.8496 rad/s
- 최신 주행 수치와 성공 여부는 `reports/runtime_validation.json`에서 확인
- 수동 저항의 엔진 적용 검사는 `reports/passive_validation.json`에서 확인
- `reports/amr_preview.png`: 1280×720 실제 렌더링

초기 v28 물리 검증 결과 (현재 접촉 설정의 결과는 위 보고서 참조):

- 구동휠의 수동 점성 감쇠가 런타임에서 0.1000000015 Nm·s/rad로 확인됨
- 전진 0.4070 m, 후진 0.4630 m, 좌/우 제자리 회전 +0.8395/-0.8344 rad
- 전진/후진 정지 이동거리 약 0.020 m, 모든 정지 후 바퀴 속도 0.05 rad/s 미만
- 전 구간 명령 변화율 최대 12.5664 rad/s², 과도한 명령·방향 반전 단위 검사 통과
- 모터를 제거한 바퀴는 0.1초 동안 2.0 → 0.6037 rad/s로 감속
- 별도 시험용 운동 마찰 0.02 Nm에 대해 감속으로 역산한 토크 0.02035 Nm

위 수치는 정해진 평지/공중 고정 시험 조건의 결과이며 실물 측정 결과가 아니다.
수정 전 설정·스크립트·검증 기록은 `backups/before_joint_physics_fix_20261003_030814.zip`에 보관했다.

이 결과는 평지 단일 AMR의 초기 환경 검증이다. 실제 로봇과의 물리 파라미터
보정이 완료되었다는 뜻은 아니다. 검증된 파일 해시는
`reports/verified_artifacts.json`에 기록되어 있다.

## 구성

```text
amr_ws/
├── src/amr_description/       최신 원본 복사본과 생성된 물리 URDF/Xacro
├── sim/config/physics.json   모터·접촉·solver 설정 및 근거
├── sim/scripts/              생성, 변환, 실행 및 검증 도구
├── sim/native/               C++ LiDAR와 외부 헤더·라이선스
├── sim/tools/                G동 4층 맵 생성 및 문 제어 도구
├── sim/layers/               생성된 G동 4층 맵 USD
├── sim/reference/            모터·접촉·solver 설정의 보존 근거
├── sim/assets/               URDF에서 변환한 USD와 payload
├── sim/amr_flat_test.usda     로컬 바닥·조명·카메라·AMR 장면 (빌드 후 생성)
├── sim/capstone_amr_4_floor.usda  G동 4층 상세 맵과 현재 AMR을 합친 장면
├── runtime/isaac-sim-6.1.0/   전용 Isaac Sim 및 동봉 Python (설치 후 생성)
├── runtime/downloads/        공식 설치 ZIP와 이어받기 정보
├── reports/                  원본 해시, URDF 검사, 실행 검사, 프리뷰
└── logs/                     다운로드·설치·실행 로그
```

## 실행

작업 공간 루트에서 실행한다.

```bash
cd amr_ws                    # 전달받은 폴더의 위치를 기준으로 이동
./run_amr.sh                 # 상세 4층 장면을 열고 Play 버튼으로 시뮬레이션
./run_amr.sh --world flat    # 독립 평지 시험장
./isaac-sim.sh               # 일반 Isaac Sim 편집기
./build_scene.sh             # 설정 변경 후 URDF와 USD 장면 재생성
./run_amr.sh --test          # 정지·전진·후진·좌우 회전·제동 및 런타임 속성 검사
./python.sh sim/scripts/validate_stability.py # 상세 4층 200 Hz 기울기·진동 검사
./scripts/experiment.sh sim headless:=true manual_test:=true # 전체 수동 제어 경로 자동 검사
./python.sh sim/scripts/validate_passive.py  # 모터를 제거한 수동 저항 검사
python3 sim/scripts/test_drive_controller.py # 명령 제한·방향 반전 경계 검사
./run_amr.sh --preview       # reports/floor4_preview.png 렌더링
./python.sh your_script.py   # 이 설치본의 Python으로 실행
```

`env.sh`는 위 스크립트에서 자동으로 불러온다. 시스템 Python이나 다른 Conda
환경에 Isaac Sim 패키지를 설치하지 않는다. 초기 실행 시 셰이더 캐시 생성에
시간이 걸릴 수 있다. ROS 2 개발용 시스템 설치와 colcon 빌드는 별도이며,
이 작업 공간의 직접 URDF→USD 변환에는 ROS 2 설치가 필요하지 않다.

## 폴더 전달과 G동 4층 맵

프로젝트 자산·원본 검사·설정 근거는 모두 `amr_ws` 안에 있다. 다른 위치에
폴더를 복사해도 형제 폴더의 `amr_description`이나 백업을 찾지 않는다.
USD의 sublayer/reference와 지도 YAML의 이미지 경로는 상대 경로이며, 실행
스크립트는 자신의 위치에서 워크스페이스 루트를 구한다.

상세 맵 원본은 `src/amr_system/amr_stack/maps/G_4_floor/`의 PGM/YAML과
`4_floor_0522(영역지정).png`다. 생성 USD는 `sim/layers/4_floor_0522_map.usda`이며,
벽 높이 3 m, 철문 15개, 엘리베이터 문 1개를 포함한다. 최상위
`sim/capstone_amr_4_floor.usda`는 이 맵과 현재 v28 AMR을 내부 상대 경로로 합친다.
`python3 sim/tools/generate_occupancy_usd.py`로 맵만 다시 생성할 수 있고,
`./build_scene.sh`는 맵과 AMR 장면을 함께 생성한다. 문 제어 방법은
[폴더 전달 안내](../docs/WORKSPACE_PORTABILITY.md)에 있다.

실행 가능한 폴더를 그대로 전달할 때는 `runtime/`, `sim/assets/`, 생성된
`sim/amr_flat_test.usda`, `reports/scene_build.json`도 함께 복사한다. Git이나
소스 릴리스에는 Isaac 설치본·생성 AMR USD가 빠지므로 새 PC에서 설치·장면
빌드가 필요하다. ROS의 `build/`·`install/`은 생성 당시 경로를 포함하므로
폴더 이동 후에는 새 위치에서 다시 빌드한다. 운영체제와 ROS 설치는 기존
시스템 의존성이다.

## 원본과 물리 설정

- 기준 원본: `src/amr_description/urdf/capstone_amr.urdf` 및 `.urdf.xacro`
- 생성물: 같은 폴더의 `capstone_amr_physics.urdf` 및 `.urdf.xacro`
- 수정 위치: 기준 Xacro/URDF와 `sim/config/physics.json`
- 생성 명령: `python3 sim/scripts/prepare_physics_urdf.py`

현재 생성기는 전달받은 Xacro와 전개된 URDF에 같은 제한·마찰 변경을 적용한다.
형상을 수정할 때는 기준 Xacro를 수정하고 기준 URDF도 다시 전개한 후 물리
변형을 생성한다. 생성된 물리 파일은 직접 수정하지 않는다.

메시, visual/collision 형상, 관절 위치·축·부모/자식 관계, 센서 좌표계와
질량·관성은 최신 원본을 보존한다. 생성기는 원본과 비교하여 이를 검사한다.
`reports/source_snapshot.json`에 최초 원본 파일들의 SHA-256이 기록되어 있다.

물리 변형에는 백업의 WA172E/ODrive 기록을 근거로 구동휠 토크 제한
12.357 Nm, 회전속도 제한 18.8496 rad/s를 적용한다.
Isaac Sim 전용 바퀴 접촉 재질, force 방식 속도 드라이브, 토크-속도
근사 모델, TGS 및 200 Hz 설정은 USD에 저장한다. URDF만 가져올 경우
이 Isaac Sim 전용 설정까지 모두 적용되는 것은 아니다.

USD 각속도 및 각속도 gain의 단위를 변환하며, 실행 검사는 런타임에
실제 읽힌 토크 제한·속도 제한·gain도 확인한다. 드라이브 gain 5 Nm/(rad/s)는
백업의 PI 응답 근사값이다. URDF의 수동 점성 감쇠 0.1 Nm·s/rad는
`PhysxJointAxisAPI:angular`의 `viscousFrictionCoefficient`에 독립적으로
연결한다. USD에는 0.001745329 Nm·s/degree, 런타임에는 0.1 Nm·s/rad이다.

### 관절 마찰의 단위와 미측정값

- `static_friction_nm`: 회전을 시작하기 전 최대 저항 토크
- `dynamic_friction_nm`: 회전 중 쿨롱 마찰 토크 (URDF `friction`에도 사용)
- `viscous_damping_nm_s_per_rad`: 속도에 비례하는 수동 저항 (URDF `damping`)
- 정지 마찰은 운동 마찰 이상이어야 하며, 음수·비유한 값은 생성 단계에서 거부한다.
- PhysX의 기존 `jointFriction`은 무차원 하중 계수이므로 0으로 비활성화했다.
  기존 값 0.02/0.00005/0.001은 `provenance`에만 보존하며 토크값으로 재사용하지 않는다.
- 구동휠·캐스터의 쿨롱 마찰 토크는 미측정 상태이므로 **0으로 두어 생략**했다.
  마찰이 실제로 0이라는 측정 결과가 아니다. 구동휠 점성 감쇠 0.1도 임시값이다.
- 바닥 접촉 마찰(구동휠 1.0/0.8, 캐스터 0.15/0.10)은 별개이며 기존 값을 유지했다.
- 캐스터 8개 관절은 드라이브 없이 수동으로 동작한다.
- 일반 URDF에는 별도의 정지/운동 마찰 토크 두 개를 모두 표현할 수 없다.
  이 모델은 URDF에 운동 마찰을 내보내고, 분리된 정지 마찰은 PhysX 속성에 저장한다.

### 구동 명령

`sim/scripts/drive_controller.py`의 `AMRDriveController`가 GUI 실행과 주행 검사에서
공통으로 매 물리 스텝 직전에 바퀴 명령을 처리한다. 속도 명령은 바퀴당
실물 `control_mux.yaml`의 1.8 rev/s(±11.310 rad/s)로 제한하고, 두 바퀴의 비율을 유지하여 포화시킨다. 가속·감속은
각각 12.566 rad/s²이며 정지·역방향 전환에도 적용한다. 물리 엔진의 하드 제한
18.8496 rad/s는 축별 API에 별도로 보존한다. 이 제한은 목표값의 변화율이며,
접촉·미끄러짐·외력에 따른 실제 차체 가속도를 강제로 덮어쓰지는 않는다.

단독 입력은 생성된 controller의 `command_twist(linear_m_s, angular_rad_s)`
또는 `command_wheels(left_rad_s, right_rad_s)`로 전달해야 한다. USD 속성이나
Articulation 텐서에 직접 쓰면 이 명령 제한을 우회한다. 기본 GUI 실행은 영속도
명령으로 시작한다. 타임라인 Stop 시 명령과 내부 상태를 초기화한다.

`validate_passive.py`는 메모리의 session layer에서만 로봇을 공중에 고정하고
모터 DriveAPI를 제거한다. 왼쪽 바퀴 감쇠와 오른쪽 바퀴의 합성 마찰 토크
(정지 0.04/운동 0.02 Nm)로 엔진 단위를 검사한다. 이 합성값은 실물 보정값이
아니며 AMR 장면 파일에 저장되지 않는다.

질량 15 kg은 기존 질량 배분이며, 최신 형상 변경 후의 신규 실측값이 아니다.
무게중심·관성, 캐스터 링크별 질량 배분·마찰은 근사/튜닝값이다.
실물과의 주행 일치, 고속/급가감속·경사로, 배터리·열 모델은 추가 검증 대상이다.
기존 DWA 코드의 차체 속도 직접 보정은 사용하지 않는다.

## 설치 및 검증 상태 확인

- `logs/download.log`: 공식 배포본 다운로드 진행률
- `reports/download_verified.json`: 공식 ZIP MD5 일치 확인 후 생성
- `reports/urdf_validation.json`: 최신 구조·형상·센서·질량 검사
- `reports/scene_build.json`: 실제 URDF→USD 변환 성공 후 생성
- `reports/runtime_validation.json`: 실제 PhysX 주행 검사 통과 후 생성
- `reports/passive_validation.json`: 구동을 제거한 바퀴 감쇠·정지/운동 마찰 검사
- `reports/amr_preview.png`: 실제 렌더링 결과

파일이 없으면 해당 단계는 완료되지 않은 상태이다. URDF 정적 검사만으로
GPU 렌더링 또는 물리 실행 성공을 의미하지 않는다.

이 PC는 Ubuntu 24.04, RTX 4070 Ti 12 GB, NVIDIA 595.91.07이다.
공식 표의 GPU 최소값은 RTX 4080 / 16 GB이므로 대규모 환경·다중 센서
작업에는 메모리 제약이 있다. 기본 프리뷰는 1280×720과 단일 AMR로 시작한다.

## 공식 참고 자료

- [Isaac Sim 6.1 다운로드](https://docs.isaacsim.omniverse.nvidia.com/6.1.0/installation/download.html)
- [Workstation 설치](https://docs.isaacsim.omniverse.nvidia.com/6.1.0/installation/install_workstation.html)
- [요구 사양](https://docs.isaacsim.omniverse.nvidia.com/6.1.0/installation/requirements.html)
- [URDF Importer API](https://docs.isaacsim.omniverse.nvidia.com/6.1.0/py/source/extensions/isaacsim.asset.importer.urdf/docs/api.html)
