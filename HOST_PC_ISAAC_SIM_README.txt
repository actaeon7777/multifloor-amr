AMR Isaac Sim 호스트 이전용 소스 패키지 (2026-10-13)

1. 구성
ZIP을 풀면 amr_ws/ 폴더가 생성됩니다.
src/의 ROS 소스, 로봇 URDF/Xacro/STL, 지도, sim/의 USD 맵·설정·스크립트,
네이티브 LiDAR 소스와 외부 헤더/라이선스, scripts/, Control_Server/ 소스,
문서 및 테스트를 포함합니다. maps/는 원본 심볼릭 링크 대신 실제 파일로 담았습니다.
원본 워크스페이스의 파일은 변경하지 않았습니다.

2. 제외한 항목
build/, install/, log/, runtime/, 가상환경과 백업 가상환경, Python 캐시,
Playwright 브라우저 설치본, .vscode/, 실행 상태(.run), diagnostics/,
backups/, bags/, 루트 reports/, 기존 배포 압축파일과 RELEASE.json,
UI 참고 영상·디자인 시안은 포함하지 않았습니다.
필요한 설치/재빌드 스크립트와 requirements 파일은 보존했습니다.
TRANSFER_MANIFEST.json에 포함 파일의 SHA-256이 기록되어 있습니다.

3. Isaac Sim 준비 (현재 프로젝트의 Linux 셸 스크립트 기준)
프로젝트 코드에 지정된 기준 런타임은 Isaac Sim 6.1.0입니다.
Isaac Sim 자체는 이 ZIP에 포함하지 않습니다.
이미 설치된 Isaac Sim을 이용하려면 압축 해제한 amr_ws 안에서:

  mkdir -p runtime reports
  ln -s /실제/IsaacSim/설치경로 runtime/isaac-sim-6.1.0

위 경로에는 python.sh와 isaac-sim.sh가 있어야 합니다.
또는 env.sh의 ISAACSIM_PATH를 실제 설치 경로로 변경할 수 있습니다.
env.sh가 해당 변수를 덮어쓰므로 export만 바꾸는 방법은 적용되지 않습니다.
다른 Isaac Sim 버전이나 Windows 호스트에서는 API/실행 스크립트 조정이 필요할 수 있습니다.

4. URDF 도구 준비 및 장면 생성
현재 원본에는 sim/assets/와 sim/amr_flat_test.usda가 없습니다.
따라서 최상위 4층 USD를 열기 전에 호스트에서 장면을 생성해야 합니다.
amr_ws 폴더에서 아래를 실행합니다 (Python venv/pip 사용 가능 환경 필요):

  python3 -m venv runtime/urdf-tools
  runtime/urdf-tools/bin/python -m pip install -r sim/config/requirements-tools.txt
  ./build_scene.sh
  ./run_amr.sh

압축 해제 도구가 실행 권한을 복원하지 않았다면 먼저 실행합니다:
  chmod +x ./*.sh scripts/*.sh

build_scene.sh는 맵, 물리 URDF, AMR USD 장면을 생성합니다.
생성 후 Isaac Sim 편집기에서 sim/capstone_amr_4_floor.usda를 열 수도 있습니다.
직접 URDF→USD 장면 생성에는 ROS 설치가 필요하지 않습니다.

5. ROS 연동을 사용할 경우
호스트에 ROS 2 Jazzy 및 빌드 의존성을 준비한 후 scripts/build_ros.sh로
build/install을 새로 생성합니다. 기존 PC의 install을 복사해 사용하지 않습니다.
ROS 시뮬레이션 실행은 scripts/experiment.sh sim 경로를 참고하세요.
C++ LiDAR의 호스트 컴파일러/SDK 요구사항은 sim/native/README.md에 있습니다.
상세 시뮬레이션 문서는 docs/archive/20261007/root/README.md 및
 docs/archive/20261007/docs/SIM2REAL.md에 있습니다 (이력 문서).
루트 README.md는 실물 AMR 운영 안내이므로 시뮬레이션 준비는 본 문서를 먼저 보세요.

6. 검증 범위
압축 파일의 CRC와 포함 파일 SHA-256을 검사합니다.
호스트 PC의 GPU/Isaac Sim 실행 및 ROS 재빌드는 이 패키징 과정에서 수행하지 않았습니다.
