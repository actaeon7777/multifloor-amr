> **이력 문서** · 2026-10-07 정리 시점의 이전 안내입니다. 현재 실행은 [최신 운영 문서](../../../README.md), 검증은 [검증 문서](../../../VALIDATION.md)를 사용합니다.

# JetPack 7.2.1 / Ubuntu 24.04 / ROS 2 Jazzy 포팅

검증일: 2026-10-03, 작업공간: `/home/acus03/amr_ws`.

기존 Ubuntu 22.04 / Humble 소스의 10개 패키지를 현재 Jetson에서 다시 빌드했다.
모든 소프트웨어 통합 시험이 통과했고, 실제 S2 LiDAR·CAN 수신·EKF·SLAM과
Jetson 화면의 RViz 시작·종료를 확인했다. Xbox 실물은 검증 시점에 연결되지 않았다.
실물 버튼 입력과 차량 주행 시험, STM32 펌웨어 업로드는 수행하지 않았다.

## 환경과 공식 기준

| 항목 | 확인한 값 |
|---|---|
| OS / CPU | Ubuntu 24.04.5 LTS / aarch64 |
| NVIDIA L4T | `nvidia-l4t-core 39.2.1-20260806224157` |
| 커널 | `6.8.12-1021-tegra` |
| ROS | `/opt/ros/jazzy`, ROS Desktop와 개발 도구 |
| 개발 도구 | Python 3.12.3, GCC 13.3.0, CMake 4.4.3 |
| 기본 ROS 도메인 | 7, 사용자가 지정한 값이 있으면 유지 |

ROS 설치는 [공식 Jazzy Ubuntu deb 설치 절차](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html)의
Noble arm64 패키지 경로를 사용했다. 문서 접근 제한 시에는
[공식 문서 저장소의 Jazzy 원문](https://github.com/ros2/ros2_documentation/blob/jazzy/source/Installation/Ubuntu-Install-Debs.rst)을 확인했다.
Nav2 설정은 [공식 Humble 이후 이행 지침](https://docs.nav2.org/jazzy/configuration_and_development/migration_guides/Humble/),
[Iron 이후 이행 지침](https://docs.nav2.org/jazzy/configuration_and_development/migration_guides/iron/Iron/),
[Jazzy 기본 파라미터](https://github.com/ros-navigation/navigation2/blob/jazzy/nav2_bringup/params/nav2_params.yaml)를 기준으로 수정했다.
SLAM 활성화에는 [공식 Jazzy 동기식 lifecycle 런치](https://github.com/SteveMacenski/slam_toolbox/blob/jazzy/launch/online_sync_launch.py)를 사용한다.

## 변경 내용

- `rosdep`으로 Jazzy arm64 의존성을 설치했다. Nav2, robot_localization,
  python-can, SLAM Toolbox 등이 포함되며 최종 `rosdep check`는 모두 충족한다.
- 네 AMR Python 패키지에서 불필요한 `ament_python` buildtool 의존성을 제거했다.
  `ament_python` 빌드 타입은 유지하고, Jazzy 템플릿 방식의 pytest 의존성을 추가했다.
- 일반 pytest 패키지에서 launch_testing의 선행 import가 opt-in 모듈의 skip을
  패키지 전체 수집 중단으로 전파하는 문제를 `setup.cfg`로 해결했다.
- Nav2 플러그인 식별자의 `::`, 복수 progress checker, behavior server의
  local/global frame·costmap·footprint 파라미터와 navigator 플러그인을 적용했다.
  기본 BT 라이브러리는 Jazzy가 로딩하므로 이전 명시 목록을 제거했다.
  빈 YAML 배열은 타입 없는 ROS 파라미터 오류를 일으켜 기본값을 사용하도록 생략했다.
- 기존 Twist 속도 경로, 차체 치수, 바퀴 제한, 모드 전환과 정지 정책을 유지했다.
  Smac smoother의 기존 refinement 횟수 4를 명시했다. 주행 튜닝값을 변경하지 않았다.
- 기본 지도를 `amr_stack` 설치 자원으로 포함하고 AMCL 기본 경로를 package share로
  변경했다. 루트 `maps`는 소스 지도 디렉터리의 심볼릭 링크라 기존 기록·저장 경로도 유지된다.
- `slam.launch.py`는 공식 lifecycle 런치를 포함한다. `config/slam.yaml`은 공식
  동기식 기본값에 기존 AMR의 5 m 범위, 1초 지도 갱신, 0.1 m/rad 이동 기준을 적용한다.
- `scripts/env.sh`, Jazzy 환경의 기록 스크립트, `amr`·`stack`·`cbuild` 별칭을 준비했다.
  빌드는 Jetson 메모리를 고려해 colcon과 내부 빌드 모두 병렬 2로 제한한다.
- 기존 F1 joy_teleop의 중립 버튼 기본 동작을 유지했다. 그 동작과 맞지 않던
  상위 원본 테스트를 실제 ROS 입력·출력 검사로 수정하고 전용 executor를 사용했다.
- 좁은 통로 시험의 35초 제한은 현재 보존된 튜닝에서 부족했다. 기존 설정 그대로
  진단했을 때 약 63초 후 충돌 없이 도착했다. 제한을 90초로 조정하고,
  재계획 횟수를 경과 시간에 맞춰 제한하며 모든 경로가 통로 안에 유지되는 검사를 추가했다.

파일별 원본 대비 변경 목록은 [source-changes.json](../../../../diagnostics/porting/source-changes.json)에 있다.

## 장치·시스템 설정

| 항목 | 적용 결과 |
|---|---|
| S2 LiDAR | CP2102N, 일련번호 `1805f83a4477ed119669f0fafdf7b791`, 1,000,000 baud |
| LiDAR 고정 경로 | 기존 `/dev/serial/by-id/...` 유지, 추가 `/dev/rplidar` 링크 |
| udev | `/etc/udev/rules.d/99-amr-s2.rules`, 해당 일련번호만 대상 |
| 사용자 권한 | `acus03`의 `dialout`, `input` 그룹; 현재 LiDAR 접근 ACL도 적용 |
| CAN | `/etc/systemd/system/can0.service`, 부팅 자동 시작, 현재 active |
| CAN 설정 | 일반 CAN 500,000 bit/s, FD off, restart 100 ms |
| CAN sample point | 요청 0.857, 현재 50 MHz 클럭에서 커널이 선택한 값 0.850 |
| Xbox | 원래 LB 6 / RB 7 / axes 1·2 매핑 유지, SDL 이름으로 장치 선택 |

udev 규칙 사본은 `../2204_amr/migration/system-config/99-amr-s2.rules`에 있다.
기존 F710 규칙과 Humble bashrc 참조 파일은 이력으로 보존한다.
새 그룹 권한은 새 로그인 세션에서 확실하게 반영된다.

Xbox의 기본 SDL 이름은 `Xbox Wireless Controller`다. 연결한 뒤 다음으로 실제 이름을 확인한다.

```bash
amr
ros2 run joy joy_enumerate_devices
```

이름이 다르면 `src/amr_system/amr_stack/config/joy_teleop.yaml`의 `joy.device_name`을
그 이름으로 변경한다. [공식 joy 구현](https://github.com/ros-drivers/joystick_drivers/blob/ros2/joy/src/joy.cpp)은
지정한 이름을 찾지 못하면 다른 장치를 열지 않는다. 이 머신의 Xbox `xpad`와
`hid_microsoft` 기본 드라이버는 확인됐지만 실물 버튼·무선 단절 시험은 아직 수행하지 않았다.

## 검증 결과

| 검증 | 결과 |
|---|---|
| 전체 빌드 | 10개 패키지 성공, 종료 코드 0 |
| 기본 colcon 테스트 | 333항목, 실패·오류 0, skip 37 |
| opt-in ROS 통합 | 8개 suite / 15개 테스트, 모두 통과, skip 0 |
| 실제 S2 스캔 | 10개 스캔, 3,240 샘플/스캔, 약 8.8 Hz; laser TF 정상 |
| 실제 CAN 수신 | 10초에 2,837 프레임; 휠·IMU 프레임 수신 |
| 실제 CAN → ROS → EKF | 휠 1,312 / IMU 132 / 관절 1,311 / filtered odom 393 메시지 |
| 실제 SLAM | lifecycle active, 151×185 지도, 0.05 m 해상도, known cell 17,571 |
| 실제 RViz | OpenGL 4.6, AMR TF 플러그인 로딩, 10초 실행 후 종료 코드 0 |
| Chromium 웹 시험 | 데스크톱·모바일 터치 목표, 드래그 방향, pan·pinch, 취소 통과; 콘솔 오류 0 |
| JS 좌표 시험 | 회전된 지도 원점·화면/지도 왕복·방향 계산 통과 |
| URDF·설치 자원 | 20 링크 / 19 관절 연결 정상, 지도와 SLAM 런치 설치 확인 |

기본 테스트의 skip 37은 opt-in ROS 모듈 8개와 생성된 인터페이스 코드의 cppcheck 29항목이다.
opt-in ROS 모듈은 별도 실행해서 모두 통과했다. 기본 colcon 항목 수에는 CTest와
생성 인터페이스 lint 결과도 포함된다.

통합 시험은 수동 입력→mux→CAN dry-run, 휠·IMU 파서, 실제 EKF와 AMCL,
목표 보관·취소·재개, 실제 Smac/DWB 경로 추종, 복구와 장애물 재계획,
웹 HTTP→목표 관리기를 검사한다. 물리 모터 송신 없이 합성 입력을 사용했다.
실제 하드웨어 시험도 CAN sender를 실행하지 않았으며 최종 CAN TX 카운터는 0이다.
실제 센서 시험은 정상 수신과 정지 상태를 확인한 것이며 실차 제동·주행 성능 검증은 아니다.

종합 결과: [verification-summary.json](../../../../diagnostics/porting/verification-summary.json).
JUnit, 런타임 로그·Nav2 결과, 장치 측정 JSON과 브라우저 캡처는
`diagnostics/porting/` 아래에 보존했다.

## 실행·재검증

새 터미널의 `amr` 별칭을 사용하거나 다음으로 환경을 로딩한다.

```bash
cd /home/acus03/amr_ws
source scripts/env.sh
```

센서 확인만 할 때:

```bash
# LiDAR만 실행
ros2 launch amr_stack lidar.launch.py

# CAN 송신 없이 실제 LiDAR·CAN 텔레메트리·EKF 실행
ros2 launch amr_stack bringup.launch.py start_can_bridge:=false start_joy:=false

# 별도 터미널에서 지도 작성; AMCL과 동시에 실행하지 않는다.
ros2 launch amr_stack slam.launch.py
```

기본 `stack`은 원래대로 실제 CAN 송신을 포함한다. 실제 Xbox 연결과 정지·입력 확인은
[실행법.md](실행법.md)와 [README.md](README.md)의 순서를 따른다.

빌드·테스트 재실행:

```bash
cd /home/acus03/amr_ws
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src --rosdistro jazzy -y
MAKEFLAGS=-j2 CMAKE_BUILD_PARALLEL_LEVEL=2 colcon build --symlink-install \
  --parallel-workers 2 --cmake-args -DCMAKE_BUILD_TYPE=Release
source scripts/env.sh
ROS_DOMAIN_ID=196 ROS_LOCALHOST_ONLY=1 QT_QPA_PLATFORM=offscreen \
  colcon test --executor sequential --return-code-on-test-failure
colcon test-result
python3 scripts/test_ros_integration.py
```

ROS 통합 실행기는 각 suite에 지정된 localhost 전용 도메인을 자동 적용한다.
실행 기록은 `diagnostics/porting/integration/`에 저장한다.

## 원본 보존

`/home/acus03/2204_amr`의 tar.gz·ZIP SHA256 검증 후 원본을 추출했고, 그 압축본은 보존했다.
`stm32_main_code.c`, `../2204_amr/migration/firmware/stm32_code.zip`, 원본 PGM·지도 YAML은
압축본과 바이트 단위로 동일하다. 결과는
[originals-preserved.json](../../../../diagnostics/porting/originals-preserved.json)에 있다.
원본 `migration/SHA256SUMS.txt`는 이행 전 기준이므로 수정된 소스의 현재 체크섬으로 갱신하지 않았다.
과거 문서가 참조하는 원래 bags·diagnostics는 이행 압축본에 포함되지 않았으며,
새 포팅의 검증 자료는 별도로 생성했다.
