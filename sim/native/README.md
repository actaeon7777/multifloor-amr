# PhysX C++ 배치 LiDAR

기본 ROS 시뮬레이터는 `amr_lidar_native.cpp`를 사용한다. 기존 200 Hz 물리 단계에서
그 시점까지 취득해야 하는 광선만 C++에 배열로 전달한다. C++가 현재 PhysX 장면의
거리 질의, 자기 로봇 충돌체 제외, 가장 가까운 외부 충돌 선택, 미검출의 `inf` 처리를
완료하고 자신이 소유하는 NumPy float32 배열을 반환한다.

3,240점·10 Hz 회전 스캔, 자세 변화에 따른 광선 방향, 첫 광선의 timestamp와
`time_increment`, 움직이는 문 감지는 기존과 같다. `run_ros_sim.py`는 주기·자세를
관리하고 `SimBridge.scan()`은 NumPy 노이즈/범위 처리와 ROS 메시지 발행을 맡는다.
각 광선마다 Python 함수를 호출하거나 충돌체 경로를 문자열로 변환하는 비용이 사라진다.

## 라이브러리 선택과 실제 사례

| 사례 | 사용하는 엔진·라이브러리 | 적용 판단 |
| --- | --- | --- |
| [Gazebo GpuLidarSensor.cc](https://github.com/gazebosim/gz-sensors/blob/gz-sensors8/src/GpuLidarSensor.cc) | `gz::rendering::GpuRays`, C++ 센서와 배열 복사·스캔 발행. [Harmonic 예제](https://gazebosim.org/docs/harmonic/sensors/)는 Ogre2 렌더러 사용 | 광선을 엔진 내부에서 처리하고 배열을 넘기는 구조를 참고. Isaac에 Gazebo 렌더러를 추가하지 않음 |
| [CARLA RayCastSemanticLidar.cpp](https://github.com/carla-simulator/carla/blob/ue5-dev/Unreal/CarlaUnreal/Plugins/Carla/Source/Carla/Sensor/RayCastSemanticLidar.cpp) | Unreal 물리 장면의 `ParallelLineTraceSingleByChannel`, C++ `ParallelFor`, 물리 읽기 잠금과 자기 객체 제외 | 현재 시뮬레이터 물리 장면을 직접 질의하는 구조를 채택. 이 소스는 UE5이며 PhysX 기반이라고 가정하지 않음 |
| [Rmagine](https://github.com/uos/rmagine) | C++ 거리 센서 라이브러리, CPU Embree·GPU OptiX/Vulkan 백엔드, 별도 메시 지도 | 대량 광선·다수 센서의 후보. 현재 Isaac 장면을 별도 지도에 복제하고 문·충돌 형상 갱신을 동기화하는 통합 작업이 필요하다고 판단해 이번에는 추가하지 않음 |

이 프로젝트가 실제로 사용하는 외부 구성요소는 **Isaac에 로드된 PhysX 장면 질의 API**와
**pybind11 2.13.6**이다. 교차점 계산 엔진을 자체 구현하지 않는다.
[공식 IPhysxSceneQuery 헤더](https://github.com/NVIDIA-Omniverse/PhysX/blob/3ca45ad36e9755f7c8c5bea9f7c57d308d9f0c54/ovphysx/ovruntime/include/omni/physx/IPhysxSceneQuery.h)의
`raycastAll`을 이용해 현재 충돌 가속 구조를 재사용한다. 가장 가까운 hit 하나만 요청하면
LiDAR 하우징 뒤의 벽을 잃으므로, 모든 hit 중 자기 로봇을 제외한 최솟값을 C++에서 선택한다.
[pybind11 NumPy 지원](https://pybind11.readthedocs.io/en/stable/advanced/pycpp/numpy.html)으로
배열을 전달한다. 별도 GPU 렌더·메시 복제·점군 생성은 필요하지 않다.

배치 내부는 순차 질의다. 200 Hz 단계당 평균 162개 광선을 처리하며, 질의 중 GIL은
해제하지만 PhysX 호출은 원래 시뮬레이션 스레드에서만 허용한다. CARLA의 병렬 실행은
그 엔진의 잠금 API를 전제로 하므로 그대로 이식하지 않았다.

## 빌드와 실행

평소 실행 명령을 그대로 사용하면 첫 실행 시 자동 빌드한 뒤 C++를 사용한다.

```bash
amr
./scripts/experiment.sh sim
```

수동 빌드는 다음과 같다. 네트워크 다운로드나 pip 설치가 필요하지 않다.

```bash
amr
/usr/bin/python3 scripts/build_native_lidar.py
```

소스·벤더 헤더·라이선스는 `sim/native/`, 빌드 스크립트는 `scripts/`, 바이너리와
빌드 명세는 `build/amr_lidar_native/`에 있다. 시스템 GCC, 표준 C++ 라이브러리와
워크스페이스의 `runtime/isaac-sim-6.1.0` SDK 헤더가 필요하다. 경로·소스·컴파일러가
바뀌면 재빌드한다. 릴리스 소스 목록과 실행 manifest에도 `sim/native/`가 포함된다.

`AMR_LIDAR_READY` 로그 또는 `/sim/status`의 `lidar_backend`가
`physx_scene_query_cpp_v1`이면 실제 C++ 경로로 실행 중이다. 비교·문제 진단용 Python
경로는 다음처럼 명시적으로 선택한다.

```bash
AMR_LIDAR_BACKEND=python ./scripts/experiment.sh sim
```

C++ 초기화 실패 때 Python으로 조용히 전환하지 않는다. 현재 검증한 조합은 Isaac Sim
6.1.0, `omni.physx 110.3.2`, CPython 3.12, Linux x86_64다. PhysX 버전이 다르면
오류로 중단한다. 이 경우 Python 경로를 선택하거나 해당 SDK에서 다시 검증해야 한다.

PhysX Python 바인딩에 등록된 인터페이스를 pybind11의 타입 변환으로 받아 사용한다.
포인터 주소 추출이나 비공개 Python 객체 레이아웃 접근은 사용하지 않는다. 설치된
바인딩의 pybind11 internals v5/GCC ABI 1016과 일치하도록 pybind11 2.13.6 및
GCC `-fabi-version=16`을 사용한다. 최신 PhysX 헤더는 다른 식별자 타입을 사용하므로
헤더만 임의로 올리면 안 된다. 벤더 버전·원본 URL·해시는
`third_party/provenance.json`, 라이선스는 각 라이브러리 하위 폴더에 보관한다.

## 검증

```bash
amr
./python.sh sim/scripts/validate_cpp_lidar.py
ROS_DOMAIN_ID=107 ./scripts/isaac_ros.sh --headless true --smoke-test
ROS_DOMAIN_ID=107 /usr/bin/python3 scripts/benchmark_runtime.py \
  --output reports/performance/new_cpp_gui --lidar-backend cpp
ROS_DOMAIN_ID=107 /usr/bin/python3 scripts/benchmark_runtime.py \
  --output reports/performance/new_python_gui --lidar-backend python
```

벤치마크는 실행 중인 도메인 107 스택이 있으면 중단한다. 가상 조이스틱·Nav2와 웹
HTTP 조회로 세 목표를 시험한다. 두 벤치마크를 동시에 실행하지 않는다.

거리 동등성 검사는 동일한 현재 자세에서 기존 Python 경로와 비교한다. 실제 4층 장면의
직진·회전·곡선·문 개폐, 자기 하우징, 다른 로봇, 최소·최대 거리, 미검출, 자체 형상 추가,
비연속 입력 배열, 결과 배열의 수명과 종료 후 접근을 검사한다. 결과는
`reports/performance/cpp_lidar_equivalence.json`에 저장한다. ROS smoke 시험은
실제 DDS 명령, 물리 이동, 센서 발행과 명령 단절 후 정지를 검사한다.

기존 `probe_native_lidar.py`에서 확인한 `reading.depths`의 stride 문제는 별개의
RaycastSensor API다. 이 구현은 그 배열을 읽지 않고 기존 PhysX 장면 질의를 직접
호출한다. Isaac 런타임 설치 파일은 수정하지 않는다.
