# amr_ws 폴더 전달

`amr_ws`는 프로젝트 자산을 자체적으로 포함한다. 실행·장면 빌드 시
`/home/user/capstone`의 다른 하위 폴더를 읽을 필요가 없다.

## 내부 원본과 참조

| 용도 | amr_ws 기준 경로 |
| --- | --- |
| 기준 AMR URDF와 메시 | `src/amr_description/` |
| 최초 원본 해시 | `reports/source_snapshot.json` |
| 모터 파라미터 근거 | `sim/reference/wa172e_actuator_parameters.yaml` |
| 과거 접촉·solver 설정 근거 | `sim/reference/amr_realistic_overrides.usda` |
| G동 4층 PGM/YAML·영역 주석 PNG | `src/amr_system/amr_stack/maps/G_4_floor/` |
| 상세 4층 맵 USD | `sim/layers/4_floor_0522_map.usda` |
| 상세 맵과 현재 AMR을 합친 장면 | `sim/capstone_amr_4_floor.usda` |
| C++ LiDAR 소스·외부 헤더·라이선스 | `sim/native/` |
| C++ LiDAR 빌드 도구·산출물 | `scripts/build_native_lidar.py`, `build/amr_lidar_native/` |

`prepare_physics_urdf.py`는 내부 원본의 URDF·Xacro·메시를 최초 해시와
비교한다. 원본 형상을 의도적으로 갱신할 때는 변경을 검토한 뒤 이 해시
기록도 함께 갱신해야 한다. 형제 폴더의 원본을 비교 대상으로 읽지 않는다.
`sim/reference/`의 과거 설정은 근거 자료이고, 현재 물리 설정은
`sim/config/physics.json`이다.

## 상세 4층 장면

워크스페이스 루트에서 실행한다.

```bash
./run_amr.sh --world floor4
# 원본 지도·영역 주석을 수정한 뒤 맵만 다시 생성
python3 sim/tools/generate_occupancy_usd.py
# AMR USD까지 다시 생성
./build_scene.sh
```

Isaac Sim 편집기에서 `sim/capstone_amr_4_floor.usda`를 직접 열 수도 있다.
AMR은 엘리베이터 앞 `(11.5, 3.31, 0.002)`에서 시작한다. 철문과 엘리베이터
문 제어는 `run_amr.sh`와 ROS 실행기의 GUI에서 자동 등록된다.
일반 편집기로 USD를 직접 열었을 때는 Script Editor에서 다음 코드를 실행한다.

```python
from pathlib import Path
import omni.usd

scene = Path(omni.usd.get_context().get_stage().GetRootLayer().realPath)
exec((scene.parent / "tools/map_door_controller.py").read_text())
```

`O`는 철문, `P`는 엘리베이터 문을 전환한다.

ROS 폐루프 실행 `./scripts/experiment.sh sim`도 이 상세 장면을 기본으로 사용한다.
`build_scene.sh`는 해당 장면과 일치하는 정적 ROS 지도 `maps/sim_floor4/`도 생성한다.
지도 파일이 처음 생성된 경우 `amr_stack`을 다시 빌드해 ROS 설치 경로에 반영한다.

## 전달 방식

실행 가능한 워크스페이스를 복사할 때는 내부 `runtime/`, `sim/assets/`,
생성된 `sim/amr_flat_test.usda`와 `reports/scene_build.json`도 포함한다.
스크립트와 USD 자산 경로는 새 폴더 위치를 따라간다.

Git 저장소(`git clone`)는 맵·원본·해시·참고 설정·
생성 도구를 포함하지만 Isaac 설치본과 생성된 AMR USD는 포함하지 않는다.
새 PC에서는 내부 `runtime/`에 실행 환경을 준비하고 `./build_scene.sh`로
장면을 생성한다. ROS를 사용할 경우 새 위치에서 `build/`·`install/`을
재생성한다. OS·드라이버·ROS 같은 시스템 의존성은 기존 설치 절차를 따른다.

ROS 시뮬레이터의 LiDAR는 첫 실행에 시스템 GCC로 자동 빌드된다. 외부 헤더는 내부에
포함되어 빌드 시 다운로드가 필요하지 않으며, 폴더를 옮기면 새 경로에서 재빌드한다.
Isaac SDK 헤더는 내부 `runtime/isaac-sim-6.1.0`에서 읽는다. 검증된 버전과 Python
비교 경로는 [C++ LiDAR 안내](../sim/native/README.md)를 참고한다.
