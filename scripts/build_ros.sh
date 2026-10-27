#!/usr/bin/env bash
# Build and test the workspace within the Jetson's limits: two compile jobs,
# one package at a time, tests on an isolated ROS domain. Driving nodes starve
# when a build takes every core (docs/VALIDATION.md "준비").
#
#   scripts/build_ros.sh                 rosdep + build + all tests
#   scripts/build_ros.sh --no-rosdep     skip rosdep update/install
#   scripts/build_ros.sh --build-only    no tests
#   scripts/build_ros.sh --packages-select amr_elevator amr_stack
#                                        build/test only these packages (implies --no-rosdep)
set -eo pipefail
workspace_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
[[ -f /opt/ros/jazzy/setup.bash ]] || {
  echo "Install ROS first: sudo bash $workspace_dir/scripts/install_ros_jazzy.sh" >&2; exit 1;
}

rosdep=1
run_tests=1
packages=()
while [[ $# -gt 0 ]]; do
  case $1 in
    --no-rosdep) rosdep=0; shift ;;
    --build-only) run_tests=0; shift ;;
    --packages-select) shift; while [[ $# -gt 0 && $1 != --* ]]; do packages+=("$1"); shift; done; rosdep=0 ;;
    -h|--help) sed -n '2,10p' "${BASH_SOURCE[0]}"; exit 0 ;;
    *) echo "Unknown option: $1" >&2; exit 2 ;;
  esac
done

if pgrep -f 'cmd_vel_to_can|control_mux' >/dev/null; then
  echo "The vehicle stack is running; stop it before building (CPU starvation stalls the robot)." >&2
  exit 1
fi

# Exclude Conda executables from the ROS Python/CMake build.
export PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin
unset PYTHONHOME PYTHONPATH CONDA_PREFIX
source /opt/ros/jazzy/setup.bash
cd "$workspace_dir"

# Resource limits for every colcon/cmake/make/ctest invocation below.
export CMAKE_BUILD_PARALLEL_LEVEL=2 MAKEFLAGS=-j2
select=()
[[ ${#packages[@]} -gt 0 ]] && select=(--packages-select "${packages[@]}")

if [[ $rosdep -eq 1 ]]; then
  rosdep update --rosdistro jazzy
  rosdep install --from-paths src --ignore-src -r -y --rosdistro jazzy
fi
nice -n 10 colcon build --base-paths src --symlink-install --executor sequential "${select[@]}" \
  --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3 -DCMAKE_BUILD_TYPE=Release
source install/local_setup.bash
[[ $run_tests -eq 1 ]] || exit 0

# Isolated domain so the tests never meet the vehicle (7) or another test run.
export ROS_DOMAIN_ID=160 ROS_LOCALHOST_ONLY=1 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
nice -n 10 colcon test --base-paths src --executor sequential "${select[@]}" \
  --event-handlers console_direct+ --ctest-args -j2
colcon test-result --verbose
if [[ ${#packages[@]} -eq 0 ]]; then
  /usr/bin/python3 -m pytest -q tests sim/scripts/test_drive_controller.py
  python3 scripts/test_ros_integration.py
fi
